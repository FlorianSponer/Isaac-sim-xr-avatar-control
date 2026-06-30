"""Standalone, headset-free test for the crouch/sit logic (pure Python, no Isaac/USD).

Run:  python exts/avatar_xr_control/tests/test_crouch_sit.py

Feeds synthetic pelvis/head trajectories straight into CrouchSitController and asserts
the functional requirements: no hyperextension / over-collapse, knee within clamp range,
feet planted during crouch and released in sit, crouch_factor continuity (no popping),
and the sit gate fires only on the scripted sit (not the pure-crouch sweep).
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "avatar", "xr", "control"))
import crouch_sit as cs                                            # noqa: E402

# --- synthetic rig (metres) ---
STANDING = 1.6                     # calibrated standing head Y
GROUND = 0.0
HIP_Y = 0.86                       # standing hip height (slightly bent)
L1 = L2 = 0.45                     # thigh / calf
FEET = {"L": (-0.10, 0.0, 0.0), "R": (0.10, 0.0, 0.0)}            # neutral feet (under hips)
LEG = {"L": (L1, L2), "R": (L1, L2)}
GY = {"L": GROUND, "R": GROUND}
HIP = (0.0, HIP_Y, 0.0)
FWD = (0.0, 0.0, -1.0)            # facing -Z; backward = +Z
DT = 1.0 / 60.0

_fail = [0]


def check(cond, msg):
    if not cond:
        _fail[0] += 1
        print("  FAIL:", msg)


def reach_ok(out):
    """foot target never exceeds leg length and never collapses below min fold."""
    hip = out["dropped_hip"]
    for side, ft in out["foot_targets"].items():
        d = cs._len3(cs._sub3(ft, hip))
        maxr = (L1 + L2) * cs.DEFAULT_PARAMS["reach_frac"]
        minf = cs.min_fold_dist(L1, L2, cs.DEFAULT_PARAMS["knee_max_deg"])
        check(d <= maxr + 1e-4, f"hyperextension d={d:.3f} > {maxr:.3f} ({side})")
        check(d >= minf - 1e-4, f"over-collapse d={d:.3f} < {minf:.3f} ({side})")
        k = cs.knee_flexion_deg(hip, ft, L1, L2)
        check(cs.DEFAULT_PARAMS["knee_min_deg"] - 1.0 <= k <= cs.DEFAULT_PARAMS["knee_max_deg"] + 1.0,
              f"knee {k:.1f}deg out of range ({side})")


# ============================================================ Phase 1: crouch sweep
def test_crouch_sweep():
    print("[Phase 1] pure-crouch sine sweep (stand -> deep crouch -> stand)")
    ctl = cs.CrouchSitController()
    N = 240
    prev_cf = 0.0
    lock_xz = None
    max_sit = 0.0
    deepest_cf = 0.0
    for i in range(N):
        # head drops up to 0.70 m then back, pelvis stays over the feet
        drop = 0.70 * (0.5 - 0.5 * math.cos(2.0 * math.pi * i / N))
        head_y = STANDING - drop
        out = ctl.update(head_y, STANDING, (0.0, 0.0), FWD, HIP,
                         FEET, LEG, GY, DT, force_sit=False)
        reach_ok(out)
        # crouch_factor continuity (no pop)
        check(abs(out["crouch_factor"] - prev_cf) < 0.2,
              f"crouch_factor jump {out['crouch_factor'] - prev_cf:+.3f} at frame {i}")
        prev_cf = out["crouch_factor"]
        deepest_cf = max(deepest_cf, out["crouch_factor"])
        max_sit = max(max_sit, out["sit_factor"])
        # feet planted in XZ once active
        if out["active"]:
            if lock_xz is None:
                lock_xz = {s: (v[0], v[2]) for s, v in out["foot_targets"].items()}
            for s, ft in out["foot_targets"].items():
                check(abs(ft[0] - lock_xz[s][0]) < 2e-3 and abs(ft[2] - lock_xz[s][1]) < 2e-3,
                      f"foot {s} drifted during crouch")
                check(abs(ft[1] - GROUND) < 1e-6, f"foot {s} left the ground in crouch")
    check(deepest_cf > 0.8, f"crouch_factor never deep (max {deepest_cf:.2f})")
    check(max_sit < 0.5, f"sit gate fired during pure crouch (sit_factor {max_sit:.2f})")
    print(f"  deepest crouch_factor={deepest_cf:.2f}  max sit_factor={max_sit:.3f}")


# ============================================================ Phase 2: scripted sit
def test_sit_trajectory():
    print("[Phase 2] scripted sit (lower + move back + settle)")
    ctl = cs.CrouchSitController()
    fired = False
    fired_frame = -1
    planted_xz = None
    released = False
    descend, settle = 70, 90
    for i in range(descend + settle):
        if i < descend:
            t = i / descend
            head_y = STANDING - 0.95 * t          # head drops to a seated height
            pelvis_z = 0.30 * t                   # pelvis moves backward (+Z)
        else:
            head_y = STANDING - 0.95               # settled (constant -> vy -> 0)
            pelvis_z = 0.30
        out = ctl.update(head_y, STANDING, (0.0, pelvis_z), FWD, HIP,
                         FEET, LEG, GY, DT, force_sit=False)
        reach_ok(out)
        if planted_xz is None and out["active"]:
            planted_xz = {s: (v[0], v[2]) for s, v in out["foot_targets"].items()}
        if out["sit_factor"] > 0.5 and not fired:
            fired, fired_frame = True, i
        # in deep sit the feet must be RELEASED forward (z goes negative = forward)
        if out["sit_factor"] > 0.8 and planted_xz is not None:
            if out["foot_targets"]["L"][2] < planted_xz["L"][1] - 0.05:
                released = True
    check(fired, "sit gate never fired on the scripted sit trajectory")
    check(released, "feet were not released forward while seated")
    print(f"  sit fired at frame {fired_frame}, feet released forward={released}")


# ============================================================ Phase 3: force_sit + noise
def test_force_and_noise():
    print("[Phase 3] force_sit override + HMD noise does not trip sit")
    ctl = cs.CrouchSitController()
    # brief noisy head dropouts around standing must NOT trigger sit
    tripped = False
    for i in range(120):
        noise = 0.04 * math.sin(i * 1.7) + (0.25 if i % 37 == 0 else 0.0)  # spikes
        out = ctl.update(STANDING - noise, STANDING, (0.0, 0.0), FWD, HIP,
                         FEET, LEG, GY, DT, force_sit=False)
        reach_ok(out)
        if out["sit_factor"] > 0.5:
            tripped = True
    check(not tripped, "noise/standing tripped the sit gate")

    ctl2 = cs.CrouchSitController()
    out = None
    for _ in range(60):
        out = ctl2.update(STANDING, STANDING, (0.0, 0.0), FWD, HIP,
                          FEET, LEG, GY, DT, force_sit=True)
        reach_ok(out)
    check(out["sit_factor"] > 0.8, "force_sit did not drive the avatar into sit")
    print(f"  noise tripped sit={tripped}  force_sit factor={out['sit_factor']:.2f}")


if __name__ == "__main__":
    test_crouch_sweep()
    test_sit_trajectory()
    test_force_and_noise()
    if _fail[0]:
        print(f"\n{_fail[0]} CHECK(S) FAILED")
        sys.exit(1)
    print("\nALL CHECKS PASSED")
