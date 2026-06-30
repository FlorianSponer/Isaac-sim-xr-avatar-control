"""Pure crouch/sit decision + geometry for the procedural leg IK.

Dependency-free (plain floats / 3-tuples + math) so it runs in CI/WSL without
Isaac Sim or USD. `extension.py` feeds it world-space scalars/tuples each frame and
converts the returned foot targets back into Gf vectors for the two-bone solve.

Frame convention: WORLD, Y-up, metres. XZ tuples are (x, z); 3-tuples are (x, y, z);
`forward` is a horizontal unit 3-tuple (y == 0). All thresholds that are metric are
expressed as fractions of the calibrated standing head height, so re-calibration
rescales them automatically.
"""

import math

# --- tunable defaults (see plan); all metric thresholds are fractions of standing ---
DEFAULT_PARAMS = dict(
    crouch_full_drop=0.45,    # head drop (x standing) mapped to crouch_factor = 1
    crouch_enter=0.06,        # head-drop fraction to START crouching (hysteresis)
    crouch_exit=0.04,         # head-drop fraction to STOP crouching
    crouch_smooth_rate=10.0,  # 1/s exp smoothing on crouch_factor (anti-pop)
    sit_height_frac=0.55,     # pelvis below this x standing -> sit height condition
    sit_back_frac=0.18,       # pelvis behind foot centroid by this x leg length
    sit_vy_max=0.08,          # m/s settled vertical-velocity gate
    sit_dwell=0.40,           # s all sit conditions must hold (debounce)
    sit_exit_frac=0.35,       # drop recovers above this x standing -> leave sit
    sit_smooth_rate=6.0,      # 1/s exp smoothing on sit_factor
    sit_drop_frac=0.55,       # max pelvis drop (x standing) while seated
    knee_min_deg=5.0,         # knee flexion clamp (lower)
    knee_max_deg=145.0,       # knee flexion clamp (upper) -> min foot-fold distance
    reach_frac=0.98,          # max foot reach as fraction of (l1 + l2)
    active_eps=0.02,          # crouch/sit factor above which the controller is "active"
)


# --------------------------------------------------------------------------- math
def _len3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def min_fold_dist(l1, l2, knee_max_deg):
    """Hip->foot distance at maximum knee flexion (law of cosines)."""
    interior = math.radians(180.0 - knee_max_deg)
    return math.sqrt(max(0.0, l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * math.cos(interior)))


def knee_flexion_deg(hip, foot, l1, l2):
    """Knee flexion angle in degrees (0 = straight leg, larger = more bent)."""
    d = _len3(_sub3(foot, hip))
    d = max(1e-6, min(d, l1 + l2))
    c = (l1 * l1 + l2 * l2 - d * d) / (2.0 * l1 * l2)
    c = max(-1.0, min(1.0, c))
    return 180.0 - math.degrees(math.acos(c))


def clamp_reach(hip, foot, l1, l2, reach_frac=0.98, min_fold=None):
    """Clamp a foot target so the leg neither hyperextends (> reach_frac*(l1+l2))
    nor over-folds (< min_fold). Returns (clamped_foot, was_clamped)."""
    if min_fold is None:
        min_fold = abs(l1 - l2) + 0.02
    v = _sub3(foot, hip)
    d = _len3(v)
    max_r = (l1 + l2) * reach_frac
    if d < 1e-6:
        return (hip[0], hip[1] - min_fold, hip[2]), True
    if d > max_r:
        s = max_r / d
        return (hip[0] + v[0] * s, hip[1] + v[1] * s, hip[2] + v[2] * s), True
    if d < min_fold:
        s = min_fold / d
        return (hip[0] + v[0] * s, hip[1] + v[1] * s, hip[2] + v[2] * s), True
    return foot, False


# --------------------------------------------------------------------- controller
class CrouchSitController:
    """Stateful per-frame crouch/sit estimator. One instance per avatar."""

    def __init__(self, params=None):
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)
        self.reset()

    def reset(self):
        self.crouch_factor = 0.0
        self.sit_factor = 0.0
        self._crouching = False        # hysteresis latch
        self._sitting = False
        self._sit_timer = 0.0
        self._prev_head_y = None
        self._head_vy = 0.0
        self._locked = {}              # side -> (x, z) foot XZ frozen on crouch entry
        self.gates = {"height": False, "back": False, "settled": False}

    # ......................................................................
    def _knee_cap(self, hip, leg_lens, ground_y):
        """Max vertical hip drop keeping every PLANTED foot reachable (no over-fold)."""
        cap = 1e9
        for side, lock in self._locked.items():
            l1, l2 = leg_lens[side]
            mf = min_fold_dist(l1, l2, self.p["knee_max_deg"])
            horiz = math.hypot(hip[0] - lock[0], hip[2] - lock[1])
            rem = mf * mf - horiz * horiz
            c = 1e9 if rem <= 0.0 else (hip[1] - ground_y[side]) - math.sqrt(rem)
            cap = min(cap, max(0.0, c))
        return cap

    # ......................................................................
    def update(self, head_y, calib_head_y, pelvis_xz, forward, hip_world,
               feet, leg_lens, ground_y, dt, force_sit=False):
        """Advance one frame.

        head_y       live HMD head world Y          calib_head_y  standing head Y
        pelvis_xz    (x, z)                          forward       horizontal unit (x,y,z)
        hip_world    (x, y, z) un-dropped hip        feet          {side:(x,y,z)} neutral foot
        leg_lens     {side:(l1, l2)}                 ground_y      {side: y}
        dt           seconds                         force_sit     UI override

        Returns a dict: crouch_factor, sit_factor, hip_drop, head_drop, dropped_hip,
        foot_targets {side:(x,y,z)}, gates, head_vy, active.
        """
        p = self.p
        standing = max(0.5, calib_head_y)
        raw_drop = max(0.0, calib_head_y - head_y)

        # --- smoothed head vertical velocity (for the settled gate) ---
        if self._prev_head_y is None:
            self._prev_head_y = head_y
        vy = (head_y - self._prev_head_y) / max(dt, 1e-4)
        self._prev_head_y = head_y
        self._head_vy += (vy - self._head_vy) * (1.0 - math.exp(-8.0 * dt))

        # --- crouch_factor (hysteresis latch + exp smoothing) ---
        if self._crouching:
            if raw_drop < p["crouch_exit"] * standing:
                self._crouching = False
        elif raw_drop > p["crouch_enter"] * standing:
            self._crouching = True
        cf_target = min(1.0, raw_drop / (p["crouch_full_drop"] * standing)) if self._crouching else 0.0
        self.crouch_factor += (cf_target - self.crouch_factor) * (1.0 - math.exp(-p["crouch_smooth_rate"] * dt))

        # --- sit gate (height AND back AND settled), dwell + hysteresis, or force ---
        cxz = ((feet["L"][0] + feet["R"][0]) * 0.5, (feet["L"][2] + feet["R"][2]) * 0.5)
        back = (cxz[0] - pelvis_xz[0]) * forward[0] + (cxz[1] - pelvis_xz[1]) * forward[2]
        leg_len = leg_lens["L"][0] + leg_lens["L"][1]
        self.gates["height"] = raw_drop > p["sit_height_frac"] * standing
        self.gates["back"] = back > p["sit_back_frac"] * leg_len
        self.gates["settled"] = abs(self._head_vy) < p["sit_vy_max"]

        if self.gates["height"] and self.gates["back"] and self.gates["settled"]:
            self._sit_timer += dt
        else:
            self._sit_timer = 0.0
        if self._sitting:
            if not force_sit and raw_drop < p["sit_exit_frac"] * standing:
                self._sitting = False
        elif force_sit or self._sit_timer >= p["sit_dwell"]:
            self._sitting = True
        sf_target = 1.0 if self._sitting else 0.0
        self.sit_factor += (sf_target - self.sit_factor) * (1.0 - math.exp(-p["sit_smooth_rate"] * dt))

        active = (self.crouch_factor > p["active_eps"]) or (self.sit_factor > p["active_eps"])

        # --- foot XZ lock: freeze when crouch/sit engages, release when standing ---
        if active:
            for side in feet:
                self._locked.setdefault(side, (feet[side][0], feet[side][2]))
        else:
            self._locked = {}

        # --- pelvis drop (head follows): cap by knee limit (crouch) / sit depth ---
        crouch_cap = self._knee_cap(hip_world, leg_lens, ground_y) if self._locked else 1e9
        sit_cap = p["sit_drop_frac"] * standing
        cap = crouch_cap * (1.0 - self.sit_factor) + sit_cap * self.sit_factor
        hip_drop = min(raw_drop, cap)
        head_drop = hip_drop
        dropped_hip = (hip_world[0], hip_world[1] - hip_drop, hip_world[2])

        # --- per-leg foot targets: planted (crouch) blended to forward (sit) ---
        targets = {}
        for side in feet:
            l1, l2 = leg_lens[side]
            gy = ground_y[side]
            lock = self._locked.get(side, (feet[side][0], feet[side][2]))
            planted = (lock[0], gy, lock[1])
            seat_h = max(0.05, dropped_hip[1] - gy)
            reach = p["reach_frac"] * (l1 + l2)
            h = math.sqrt(max(0.0, reach * reach - seat_h * seat_h))      # forward foot reach
            sit_fwd = (dropped_hip[0] + forward[0] * h, gy, dropped_hip[2] + forward[2] * h)
            sf = self.sit_factor
            ft = (planted[0] * (1.0 - sf) + sit_fwd[0] * sf, gy,
                  planted[2] * (1.0 - sf) + sit_fwd[2] * sf)
            mf = min_fold_dist(l1, l2, p["knee_max_deg"])
            ft, _ = clamp_reach(dropped_hip, ft, l1, l2, p["reach_frac"], mf)
            targets[side] = ft

        return {
            "crouch_factor": self.crouch_factor,
            "sit_factor": self.sit_factor,
            "hip_drop": hip_drop,
            "head_drop": head_drop,
            "dropped_hip": dropped_hip,
            "foot_targets": targets,
            "gates": dict(self.gates),
            "head_vy": self._head_vy,
            "active": active,
        }
