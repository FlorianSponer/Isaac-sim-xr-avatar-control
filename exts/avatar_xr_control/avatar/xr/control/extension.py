import asyncio
import json
import math
import os
import time

import omni.ext
import omni.ui as ui
import omni.usd
from omni.usd import StageEventType
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, UsdShade, UsdPhysics, Vt

from .crouch_sit import CrouchSitController

# PhysX scene-query support is optional: if omni.physx is unavailable the
# extension still loads and the physics-collision correction simply stays off.
try:
    import carb
    from omni.physx import get_physx_scene_query_interface, get_physx_interface
    _PHYS_AVAILABLE = True
except Exception as _e:               # pragma: no cover - depends on runtime
    carb = None
    get_physx_scene_query_interface = None
    get_physx_interface = None
    _PHYS_AVAILABLE = False
    print(f"[avatar_xr_control] omni.physx unavailable; physics collision off: {_e}")

# ---------------------------------------------------------------------------
# Data directory — calibration, recordings and diagnostic dumps live here.
# Overridable via AVATAR_XR_DATA_DIR; falls back to the historical project
# folder when present (dev machine), else a per-user folder, so calibration
# persistence works on any deployment target.
# ---------------------------------------------------------------------------

_LEGACY_DATA_DIR = r"c:\World\Institut_Setup3"


def _resolve_data_dir():
    d = os.environ.get("AVATAR_XR_DATA_DIR")
    if not d:
        d = (_LEGACY_DATA_DIR if os.path.isdir(_LEGACY_DATA_DIR)
             else os.path.join(os.path.expanduser("~"), "avatar_xr_control"))
    try:
        os.makedirs(d, exist_ok=True)
    except Exception as e:
        print(f"[avatar_xr_control] data dir '{d}' unavailable ({e}); "
              f"falling back to cwd")
        d = os.getcwd()
    return d


DATA_DIR = _resolve_data_dir()


def _data_path(name):
    return os.path.join(DATA_DIR, name)


# Per-frame diagnostic file dumps (_armvec_debug / _follow_debug /
# _rest_world_debug). Development aids — off unless explicitly enabled, so a
# deployed build doesn't write to disk every second.
DEBUG_FILES = os.environ.get("AVATAR_XR_DEBUG_FILES", "0").lower() in (
    "1", "true", "on")

DEFAULT_SKEL_PATH = (
    "/Root/female_adult_business_02/ManRoot/female_adult_business_02"
    "/female_adult_business_02/female_adult_business_02"
)

_SPINE = "RL_BoneRoot/Hip"
_ARM   = "RL_BoneRoot/Hip/Waist/Spine01/Spine02"
_LH    = _ARM + "/L_Clavicle/L_Upperarm/L_Forearm/L_Hand"
_RH    = _ARM + "/R_Clavicle/R_Upperarm/R_Forearm/R_Hand"

JOINT_MAP = {
    # Spine
    "waist":      _SPINE + "/Waist",
    "spine01":    _SPINE + "/Waist/Spine01",
    "spine02":    _SPINE + "/Waist/Spine01/Spine02",
    "neck1":      _ARM   + "/NeckTwist01",
    "neck2":      _ARM   + "/NeckTwist01/NeckTwist02",
    "head":       _ARM   + "/NeckTwist01/NeckTwist02/Head",
    # Right arm
    "r_clavicle": _ARM + "/R_Clavicle",
    "r_upperarm": _ARM + "/R_Clavicle/R_Upperarm",
    "r_forearm":  _ARM + "/R_Clavicle/R_Upperarm/R_Forearm",
    "r_hand":     _RH,
    # Left arm
    "l_clavicle": _ARM + "/L_Clavicle",
    "l_upperarm": _ARM + "/L_Clavicle/L_Upperarm",
    "l_forearm":  _ARM + "/L_Clavicle/L_Upperarm/L_Forearm",
    "l_hand":     _LH,
    # Right hand fingers
    "r_thumb1":   _RH + "/R_Thumb1",
    "r_thumb2":   _RH + "/R_Thumb1/R_Thumb2",
    "r_thumb3":   _RH + "/R_Thumb1/R_Thumb2/R_Thumb3",
    "r_index1":   _RH + "/R_Index1",
    "r_index2":   _RH + "/R_Index1/R_Index2",
    "r_index3":   _RH + "/R_Index1/R_Index2/R_Index3",
    "r_mid1":     _RH + "/R_Mid1",
    "r_mid2":     _RH + "/R_Mid1/R_Mid2",
    "r_mid3":     _RH + "/R_Mid1/R_Mid2/R_Mid3",
    "r_ring1":    _RH + "/R_Ring1",
    "r_ring2":    _RH + "/R_Ring1/R_Ring2",
    "r_ring3":    _RH + "/R_Ring1/R_Ring2/R_Ring3",
    "r_pinky1":   _RH + "/R_Pinky1",
    "r_pinky2":   _RH + "/R_Pinky1/R_Pinky2",
    "r_pinky3":   _RH + "/R_Pinky1/R_Pinky2/R_Pinky3",
    # Left hand fingers
    "l_thumb1":   _LH + "/L_Thumb1",
    "l_thumb2":   _LH + "/L_Thumb1/L_Thumb2",
    "l_thumb3":   _LH + "/L_Thumb1/L_Thumb2/L_Thumb3",
    "l_index1":   _LH + "/L_Index1",
    "l_index2":   _LH + "/L_Index1/L_Index2",
    "l_index3":   _LH + "/L_Index1/L_Index2/L_Index3",
    "l_mid1":     _LH + "/L_Mid1",
    "l_mid2":     _LH + "/L_Mid1/L_Mid2",
    "l_mid3":     _LH + "/L_Mid1/L_Mid2/L_Mid3",
    "l_ring1":    _LH + "/L_Ring1",
    "l_ring2":    _LH + "/L_Ring1/L_Ring2",
    "l_ring3":    _LH + "/L_Ring1/L_Ring2/L_Ring3",
    "l_pinky1":   _LH + "/L_Pinky1",
    "l_pinky2":   _LH + "/L_Pinky1/L_Pinky2",
    "l_pinky3":   _LH + "/L_Pinky1/L_Pinky2/L_Pinky3",
}

# XR hand pose name → (avatar joint key right, avatar joint key left)
FINGER_POSE_MAP = [
    ("thumb_proximal",     "r_thumb1",  "l_thumb1"),
    ("thumb_distal",       "r_thumb2",  "l_thumb2"),
    ("thumb_tip",          "r_thumb3",  "l_thumb3"),
    ("index_proximal",     "r_index1",  "l_index1"),
    ("index_intermediate", "r_index2",  "l_index2"),
    ("index_distal",       "r_index3",  "l_index3"),
    ("middle_proximal",    "r_mid1",    "l_mid1"),
    ("middle_intermediate","r_mid2",    "l_mid2"),
    ("middle_distal",      "r_mid3",    "l_mid3"),
    ("ring_proximal",      "r_ring1",   "l_ring1"),
    ("ring_intermediate",  "r_ring2",   "l_ring2"),
    ("ring_distal",        "r_ring3",   "l_ring3"),
    ("little_proximal",    "r_pinky1",  "l_pinky1"),
    ("little_intermediate","r_pinky2",  "l_pinky2"),
    ("little_distal",      "r_pinky3",  "l_pinky3"),
]


# ---------------------------------------------------------------------------
# One Euro Filter — speed-adaptive low-pass for noisy VR tracking input.
# Canonical reference (jaantollander / Casiez et al.). One instance per scalar.
# ---------------------------------------------------------------------------

def _smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)


def _exp_smooth(a, x, x_prev):
    return a * x + (1 - a) * x_prev


class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def __call__(self, t, x):
        t_e = t - self.t_prev
        if t_e <= 0.0:
            return self.x_prev
        a_d = _smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = _exp_smooth(a_d, dx, self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = _smoothing_factor(t_e, cutoff)
        x_hat = _exp_smooth(a, x, self.x_prev)
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat


class Vec3OneEuro:
    """Three One Euro filters — one per position component."""
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._f = None

    def filter(self, v):
        t = time.monotonic()
        if self._f is None:
            self._f = [OneEuroFilter(t, v[i], min_cutoff=self.min_cutoff,
                                     beta=self.beta, d_cutoff=self.d_cutoff)
                       for i in range(3)]
            return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))
        # Keep params live-tunable.
        for f in self._f:
            f.min_cutoff = self.min_cutoff
            f.beta = self.beta
        return Gf.Vec3f(float(self._f[0](t, v[0])),
                        float(self._f[1](t, v[1])),
                        float(self._f[2](t, v[2])))

    def reset(self):
        self._f = None


def _quat_dot(a: "Gf.Quatf", b: "Gf.Quatf") -> float:
    ia, ib = a.GetImaginary(), b.GetImaginary()
    return (float(a.GetReal()) * float(b.GetReal())
            + float(ia[0]) * float(ib[0]) + float(ia[1]) * float(ib[1])
            + float(ia[2]) * float(ib[2]))


def _quat_nlerp(a: "Gf.Quatf", b: "Gf.Quatf", t: float) -> "Gf.Quatf":
    """Normalized lerp a→b by t (≈ slerp for close quats), hemisphere-aware."""
    if _quat_dot(a, b) < 0.0:
        bi = b.GetImaginary()
        b = Gf.Quatf(-float(b.GetReal()), -float(bi[0]), -float(bi[1]), -float(bi[2]))
    ai, bi = a.GetImaginary(), b.GetImaginary()
    s = 1.0 - t
    w = float(a.GetReal()) * s + float(b.GetReal()) * t
    x = float(ai[0]) * s + float(bi[0]) * t
    y = float(ai[1]) * s + float(bi[1]) * t
    z = float(ai[2]) * s + float(bi[2]) * t
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return a
    return Gf.Quatf(w / n, x / n, y / n, z / n)


class QuatOneEuro:
    """One-Euro-style adaptive low-pass for a unit quaternion: strong smoothing
    when nearly still (kills controller rotation jitter), low lag when turning
    fast (the blend rises with angular speed). The orientation analogue of
    Vec3OneEuro; uses real wall-clock dt so it is frame-rate correct."""
    def __init__(self, min_cutoff=2.0, beta=0.1):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self._prev = None
        self._t = None

    def filter(self, q: "Gf.Quatf") -> "Gf.Quatf":
        now = time.monotonic()
        if self._prev is None or self._t is None:
            self._prev, self._t = q, now
            return q
        dt = now - self._t
        if dt <= 0.0:
            return self._prev
        self._t = now
        # Angular speed between frames (deg/s) drives the adaptive cutoff.
        d = _quat_angle(_quat_mul(q, _quat_conj(self._prev)))   # radians ≥ 0
        cutoff = self.min_cutoff + self.beta * (math.degrees(d) / dt)
        r = 2.0 * math.pi * cutoff * dt
        a = r / (r + 1.0)
        self._prev = _quat_nlerp(self._prev, q, a)
        return self._prev

    def reset(self):
        self._prev = None
        self._t = None


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _quat_mul(a: Gf.Quatf, b: Gf.Quatf) -> Gf.Quatf:
    w1, x1 = a.GetReal(), a.GetImaginary()
    w2, x2 = b.GetReal(), b.GetImaginary()
    return Gf.Quatf(
        w1*w2 - x1[0]*x2[0] - x1[1]*x2[1] - x1[2]*x2[2],
        w1*x2[0] + x1[0]*w2 + x1[1]*x2[2] - x1[2]*x2[1],
        w1*x2[1] - x1[0]*x2[2] + x1[1]*w2 + x1[2]*x2[0],
        w1*x2[2] + x1[0]*x2[1] - x1[1]*x2[0] + x1[2]*w2,
    )


def _quat_conj(q: Gf.Quatf) -> Gf.Quatf:
    im = q.GetImaginary()
    return Gf.Quatf(q.GetReal(), -im[0], -im[1], -im[2])



def _vec3f(v) -> Gf.Vec3f:
    return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))


def _normalize(v: Gf.Vec3f) -> Gf.Vec3f:
    n = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if n < 1e-8:
        return Gf.Vec3f(0, 0, 1)
    return Gf.Vec3f(v[0]/n, v[1]/n, v[2]/n)


def _quat_from_to(from_v: Gf.Vec3f, to_v: Gf.Vec3f) -> Gf.Quatf:
    dot = from_v[0]*to_v[0] + from_v[1]*to_v[1] + from_v[2]*to_v[2]
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9999:
        return Gf.Quatf(1, 0, 0, 0)
    if dot < -0.9999:
        perp = Gf.Vec3f(1, 0, 0)
        if abs(from_v[0]) > 0.9:
            perp = Gf.Vec3f(0, 1, 0)
        return Gf.Quatf(0, perp[0], perp[1], perp[2])
    cx = from_v[1]*to_v[2] - from_v[2]*to_v[1]
    cy = from_v[2]*to_v[0] - from_v[0]*to_v[2]
    cz = from_v[0]*to_v[1] - from_v[1]*to_v[0]
    w  = math.sqrt((1 + dot) * 2)
    iw = 1.0 / w
    q  = Gf.Quatf(w * 0.5, cx*iw, cy*iw, cz*iw)
    n  = math.sqrt(q.GetReal()**2 + sum(x**2 for x in q.GetImaginary()))
    im = q.GetImaginary()
    return Gf.Quatf(q.GetReal()/n, im[0]/n, im[1]/n, im[2]/n)


def _quat_rotate(q: Gf.Quatf, v: Gf.Vec3f) -> Gf.Vec3f:
    """Rotate vector v by unit quaternion q (v' = q·v·q⁻¹, via the cross form)."""
    im = q.GetImaginary()
    qx, qy, qz, qw = float(im[0]), float(im[1]), float(im[2]), float(q.GetReal())
    # t = 2 * (q.xyz × v)
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    # v' = v + qw*t + q.xyz × t
    return Gf.Vec3f(
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    )


def _quat_scale_angle(q: Gf.Quatf, w: float, max_angle: float = None) -> Gf.Quatf:
    """Scale a rotation's ANGLE by w (a partial rotation toward q), optionally
    clamped to max_angle radians. Used to apply a fractional clavicle follow.
    Assumes q has w ≥ 0 (true for _quat_from_to), so the angle is in [0, π]."""
    real = max(-1.0, min(1.0, float(q.GetReal())))
    angle = 2.0 * math.acos(real)
    s = math.sqrt(max(0.0, 1.0 - real * real))   # sin(angle/2)
    if angle < 1e-6 or s < 1e-8:
        return Gf.Quatf(1, 0, 0, 0)
    im = q.GetImaginary()
    ax, ay, az = float(im[0]) / s, float(im[1]) / s, float(im[2]) / s
    new_angle = angle * w
    if max_angle is not None:
        new_angle = max(-max_angle, min(max_angle, new_angle))
    h  = new_angle * 0.5
    sh = math.sin(h)
    return Gf.Quatf(math.cos(h), ax * sh, ay * sh, az * sh)


def _clamp_quat_angle(q: Gf.Quatf, max_angle: float) -> Gf.Quatf:
    """Clamp a rotation's TOTAL angle to max_angle radians about its own axis
    (a single cone limit on the orientation), leaving smaller rotations
    untouched. Used to keep the hand from rotating to impossible orientations."""
    w = float(q.GetReal())
    im = q.GetImaginary()
    x, y, z = float(im[0]), float(im[1]), float(im[2])
    if w < 0.0:                      # canonical hemisphere (q and -q are equal)
        w, x, y, z = -w, -x, -y, -z
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    if angle <= max_angle:
        return q
    s = math.sqrt(max(0.0, 1.0 - w * w))   # sin(angle/2)
    if s < 1e-8:
        return Gf.Quatf(1, 0, 0, 0)
    ax, ay, az = x / s, y / s, z / s
    h  = max_angle * 0.5
    sh = math.sin(h)
    return Gf.Quatf(math.cos(h), ax * sh, ay * sh, az * sh)


def _swing_twist(q: Gf.Quatf, axis: Gf.Vec3f):
    """Decompose q into (swing, twist) about a UNIT axis, with q = swing · twist.
    `twist` is the rotation about `axis` (e.g. forearm roll), `swing` is the
    remaining bend. Twist is returned in the canonical hemisphere (w ≥ 0) so its
    angle can be scaled. Used to move wrist roll onto the forearm bone."""
    im = q.GetImaginary()
    # Project the quaternion's vector part onto the axis → the twist's vector part.
    d = float(im[0]) * axis[0] + float(im[1]) * axis[1] + float(im[2]) * axis[2]
    tw = Gf.Quatf(float(q.GetReal()), axis[0] * d, axis[1] * d, axis[2] * d)
    n = math.sqrt(tw.GetReal() ** 2 + sum(float(c) ** 2 for c in tw.GetImaginary()))
    if n < 1e-8:
        twist = Gf.Quatf(1, 0, 0, 0)
    else:
        twist = Gf.Quatf(tw.GetReal() / n, tw.GetImaginary()[0] / n,
                         tw.GetImaginary()[1] / n, tw.GetImaginary()[2] / n)
    if twist.GetReal() < 0.0:               # canonical hemisphere (q ≡ -q)
        ti = twist.GetImaginary()
        twist = Gf.Quatf(-twist.GetReal(), -ti[0], -ti[1], -ti[2])
    swing = _quat_mul(q, _quat_conj(twist))
    return swing, twist


def _soft_cap_max(v: float, limit: float, band: float) -> float:
    """Softly cap v at `limit`: pass through until `limit-band`, then compress
    the remainder asymptotically so v approaches but never reaches `limit`. Used
    so the hand keeps responding to the controller right up to the workspace
    boundary instead of locking onto a hard wall."""
    if band <= 1e-6:
        return min(v, limit)
    knee = limit - band
    if v <= knee:
        return v
    return knee + band * (1.0 - math.exp(-(v - knee) / band))


def _quat_axis_angle(axis: Gf.Vec3f, angle: float) -> Gf.Quatf:
    """Quaternion for a rotation of `angle` radians about a UNIT axis."""
    h = angle * 0.5
    s = math.sin(h)
    return Gf.Quatf(math.cos(h), float(axis[0]) * s, float(axis[1]) * s,
                    float(axis[2]) * s)


def _quat_angle(q: Gf.Quatf) -> float:
    """Total rotation angle (radians, ≥0) of a unit quaternion."""
    w = abs(max(-1.0, min(1.0, float(q.GetReal()))))
    return 2.0 * math.acos(w)


def _apply_invert(w, x, y, z, inv_x, inv_y, inv_z) -> Gf.Quatf:
    return Gf.Quatf(
        w,
        x * (-1 if inv_x else 1),
        y * (-1 if inv_y else 1),
        z * (-1 if inv_z else 1),
    )


def _sub(a, b):
    return Gf.Vec3f(a[0]-b[0], a[1]-b[1], a[2]-b[2])


def _add(a, b):
    return Gf.Vec3f(a[0]+b[0], a[1]+b[1], a[2]+b[2])


def _scale(v, s):
    return Gf.Vec3f(v[0]*s, v[1]*s, v[2]*s)


def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def _len(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])


def _two_bone_ik_full(root, target, len1, len2, pole):
    """Analytic 2-bone IK returning the elbow plus its swivel basis.

    root   : shoulder world position (Gf.Vec3f)
    target : desired wrist world position
    len1   : upper-arm length, len2 : forearm length
    pole   : a hint vector for which way the elbow points (the natural pole)

    Returns (elbow, wrist, base, h, u, v):
      base : point on the root→wrist line directly under the elbow
      h    : radius of the elbow circle around that line
      u, v : orthonormal basis perpendicular to root→wrist. u is the preferred
             bend direction from `pole`, so the elbow at swivel angle 0 is
             base + h·u. The elbow may be swung to
             base + h·(cosφ·u + sinφ·v) for ANY swivel φ while keeping both
             bone lengths and the wrist position exact — this is the single
             free DOF used for self-collision avoidance.
    """
    to_target = _sub(target, root)
    eps  = 1e-5
    raw  = _len(to_target)
    # Clamp the reach to [|len1-len2|, len1+len2] so the triangle is solvable.
    max_reach = len1 + len2
    min_reach = abs(len1 - len2)
    dist = max(min_reach + eps, min(raw, max_reach - eps))
    dir_t = _scale(to_target, 1.0 / max(raw, eps))
    # Distance from root to the elbow's projection on the root→target line.
    a = (len1*len1 - len2*len2 + dist*dist) / (2.0 * dist)
    h = math.sqrt(max(0.0, len1*len1 - a*a))
    base = _add(root, _scale(dir_t, a))
    # u = component of pole perpendicular to the limb axis (the natural bend).
    pole_perp = _sub(pole, _scale(dir_t, _dot(pole, dir_t)))
    pl = _len(pole_perp)
    if pl < eps:
        # Pole parallel to limb; pick an arbitrary perpendicular.
        ref = Gf.Vec3f(0, 1, 0) if abs(dir_t[1]) < 0.9 else Gf.Vec3f(1, 0, 0)
        pole_perp = _sub(ref, _scale(dir_t, _dot(ref, dir_t)))
        pl = _len(pole_perp)
    u = _scale(pole_perp, 1.0 / pl)
    # v completes a right-handed frame (dir_t, u, v) — the second swivel axis.
    v = Gf.Vec3f(dir_t[1]*u[2] - dir_t[2]*u[1],
                 dir_t[2]*u[0] - dir_t[0]*u[2],
                 dir_t[0]*u[1] - dir_t[1]*u[0])
    elbow = _add(base, _scale(u, h))
    wrist = _add(root, _scale(dir_t, dist))
    return elbow, wrist, base, h, u, v


def _yaw_of(m: Gf.Matrix4d) -> float:
    """Yaw (radians) of a pose matrix's horizontal forward direction.
    Forward = local -Z (OpenXR and avatar convention); yaw 0 = facing stage -Z,
    so R_y(yaw) rotates the rest avatar onto the pose's heading."""
    fwd = m.TransformDir(Gf.Vec3d(0, 0, -1))
    return math.atan2(-float(fwd[0]), -float(fwd[2]))


def _rot_y(v, ang: float) -> Gf.Vec3f:
    """Rotate a vector by ang radians around +Y."""
    c, s = math.cos(ang), math.sin(ang)
    return Gf.Vec3f(float(v[0]) * c + float(v[2]) * s,
                    float(v[1]),
                    -float(v[0]) * s + float(v[2]) * c)


def _yaw_quatf(ang: float) -> Gf.Quatf:
    """Quaternion for a rotation of ang radians around +Y."""
    h = ang * 0.5
    return Gf.Quatf(math.cos(h), 0.0, math.sin(h), 0.0)


def _correct_xr_quat(quatd) -> Gf.Quatf:
    """Convert an OpenXR device-pose quaternion into the avatar stage frame.
    Avatar faces -Z, right = +X, up = +Y — same convention as OpenXR.
    Only negate X and Z to match stage handedness."""
    return _apply_invert(
        float(quatd.GetReal()),
        float(quatd.GetImaginary()[0]),
        float(quatd.GetImaginary()[1]),
        float(quatd.GetImaginary()[2]),
        True, False, True,
    )


# ---------------------------------------------------------------------------
# Skeleton wrapper
# ---------------------------------------------------------------------------

class _AvatarSkel:

    # Camera-follow yaw op on the avatar root — dedicated suffix so it can be
    # identified (and reset) independently of the asset's own xform ops.
    FOLLOW_YAW_OP = "xformOp:rotateY:avatarFollow"

    def __init__(self, stage, skel_path: str):
        anim_path = skel_path + "/xr_anim"

        # Avatar ROOT prim path (the movable prim two levels above the skeleton,
        # e.g. /Root/<avatar>). Resolve it FIRST: if a previous session left the
        # camera-follow yaw op authored on it, zero that op BEFORE caching any
        # rest transforms below — otherwise every cached rest position/rotation
        # is tilted by the stale yaw and the whole frame mapping breaks.
        parts = skel_path.strip("/").split("/")
        self.root_path = ("/" + "/".join(parts[:2])) if len(parts) >= 2 else skel_path
        root_prim = stage.GetPrimAtPath(self.root_path)
        if root_prim.IsValid():
            for op in UsdGeom.Xformable(root_prim).GetOrderedXformOps():
                # Plain "xformOp:rotateY" is the leftover op the first follow
                # implementation created (appended innermost → it rolled the
                # avatar about world -Z instead of yawing). The asset itself
                # authors no plain rotateY, so both ops are safe to zero.
                if op.GetOpName() in (self.FOLLOW_YAW_OP, "xformOp:rotateY"):
                    op.Set(0.0)

        skel_prim = stage.GetPrimAtPath(skel_path)
        if not skel_prim.IsValid():
            raise RuntimeError(f"Skeleton prim not found: {skel_path}")

        skel   = UsdSkel.Skeleton(skel_prim)
        joints = list(skel.GetJointsAttr().Get())

        self.head_idx    = joints.index(JOINT_MAP["head"])
        self.waist_idx   = joints.index(JOINT_MAP["waist"])
        self.spine01_idx = joints.index(JOINT_MAP["spine01"])
        self.spine02_idx = joints.index(JOINT_MAP["spine02"])
        self.r_clav_idx  = joints.index(JOINT_MAP["r_clavicle"])
        self.r_upper_idx = joints.index(JOINT_MAP["r_upperarm"])
        self.r_fore_idx  = joints.index(JOINT_MAP["r_forearm"])
        self.r_hand_idx  = joints.index(JOINT_MAP["r_hand"])
        self.l_clav_idx  = joints.index(JOINT_MAP["l_clavicle"])
        self.l_upper_idx = joints.index(JOINT_MAP["l_upperarm"])
        self.l_fore_idx  = joints.index(JOINT_MAP["l_forearm"])
        self.l_hand_idx  = joints.index(JOINT_MAP["l_hand"])

        # Finger joint indices keyed by JOINT_MAP key
        _non_finger = {"head", "waist", "spine01", "spine02", "neck1", "neck2",
                       "r_clavicle", "r_upperarm", "r_forearm", "r_hand",
                       "l_clavicle", "l_upperarm", "l_forearm", "l_hand"}
        self.finger_idx = {}
        for key in JOINT_MAP:
            if key not in _non_finger:
                try:
                    self.finger_idx[key] = joints.index(JOINT_MAP[key])
                except ValueError:
                    pass

        anim_prim = stage.GetPrimAtPath(anim_path)
        if not anim_prim.IsValid():
            anim_prim = stage.DefinePrim(anim_path, "SkelAnimation")

        self.anim = UsdSkel.Animation(anim_prim)
        self.anim.GetJointsAttr().Set(skel.GetJointsAttr().Get())

        binding = UsdSkel.BindingAPI.Apply(skel_prim)
        binding.GetAnimationSourceRel().SetTargets([anim_path])

        rest = skel.GetRestTransformsAttr().Get()
        translations, rotations, scales = [], [], []
        for xf in rest:
            t = xf.ExtractTranslation()
            q = xf.ExtractRotationQuat()
            translations.append(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
            rotations.append(Gf.Quatf(
                float(q.GetReal()),
                float(q.GetImaginary()[0]),
                float(q.GetImaginary()[1]),
                float(q.GetImaginary()[2]),
            ))
            scales.append(Gf.Vec3h(1, 1, 1))

        self.anim.GetTranslationsAttr().Set(Vt.Vec3fArray(translations))
        self.anim.GetRotationsAttr().Set(Vt.QuatfArray(rotations))
        self.anim.GetScalesAttr().Set(Vt.Vec3hArray(scales))

        self._rotations    = rotations
        self._scales       = scales
        self._translations = translations   # mutated by the pelvis-drop (legs)
        # Deferred-write state: while the tracking loop runs, joint writes only
        # mutate the arrays above and flush() pushes them to USD once per frame.
        self._defer        = False
        self._rot_dirty    = False
        self._trans_dirty  = False

        # TRUE-WORLD joint transforms via the canonical UsdSkel query. This is
        # render-faithful — it composes the full transform stack (rest pose +
        # every prim above the skeleton), unlike the old hand-rolled walk which
        # was skeleton-local and disagreed with what the mesh actually renders.
        # Pin the cache on self so the query stays valid.
        self._skel_cache = UsdSkel.Cache()
        query = self._skel_cache.GetSkelQuery(skel)
        if not query:
            raise RuntimeError(f"UsdSkelSkeletonQuery invalid for {skel_path}")
        self._skel_query = query   # reused for LIVE joint transforms each frame
        xf = UsdGeom.XformCache(Usd.TimeCode.Default())
        # atRest=True → rest pose, ignores the bound xr_anim (avoids feedback).
        self._rest_world = query.ComputeJointWorldTransforms(xf, True)

        def world_pos(idx):
            return _vec3f(self._rest_world[idx].ExtractTranslation())

        def world_rot(idx):
            q = self._rest_world[idx].ExtractRotationQuat()
            return Gf.Quatf(float(q.GetReal()),
                            float(q.GetImaginary()[0]),
                            float(q.GetImaginary()[1]),
                            float(q.GetImaginary()[2]))

        # Forearm rest world rotations — parent frame for wrist→local conversion
        self.r_fore_world_q_rest = world_rot(self.r_fore_idx)
        self.l_fore_world_q_rest = world_rot(self.l_fore_idx)

        # Hand rest local rotations — T-pose calibration target for the wrist
        self.r_hand_local_rest_q = self._rotations[self.r_hand_idx]
        self.l_hand_local_rest_q = self._rotations[self.l_hand_idx]

        # --- Arm IK rest data (shoulder=upperarm, elbow=forearm, wrist=hand) ---
        # World rest positions of the three arm joints, used as the IK chain.
        self.r_shoulder_pos = world_pos(self.r_upper_idx)
        self.r_elbow_pos    = world_pos(self.r_fore_idx)
        self.r_wrist_pos    = world_pos(self.r_hand_idx)
        self.l_shoulder_pos = world_pos(self.l_upper_idx)
        self.l_elbow_pos    = world_pos(self.l_fore_idx)
        self.l_wrist_pos    = world_pos(self.l_hand_idx)

        # World rest rotations of the arm joints (delta IK applies on top of these)
        self.r_upper_world_q_rest = world_rot(self.r_upper_idx)
        self.l_upper_world_q_rest = world_rot(self.l_upper_idx)

        # Head rest position (cached-rest IK frame) — anchor for the head
        # collider proxy used by the PhysX self-collision layer.
        self.head_pos = world_pos(self.head_idx)

        # Parent world rotations for world→local conversion of arm joints
        self.r_upper_parent_world_q = world_rot(self.r_clav_idx)
        self.l_upper_parent_world_q = world_rot(self.l_clav_idx)

        # --- Clavicle (shoulder girdle) rest data for VRIK-style shoulder follow ---
        # The clavicle is partially rotated toward the hand each frame so reaches
        # lift/protract the shoulder. Need: the clavicle world rest position (the
        # pivot), its world rest rotation (= the upper-arm parent rotation), and
        # ITS parent's world rotation (Spine02) for the world→local conversion.
        self.r_clav_pos = world_pos(self.r_clav_idx)
        self.l_clav_pos = world_pos(self.l_clav_idx)
        self.r_clav_world_q_rest = self.r_upper_parent_world_q
        self.l_clav_world_q_rest = self.l_upper_parent_world_q
        self.clav_parent_world_q = world_rot(self.spine02_idx)

        # --- Spine rest data for body-turn-to-reach ---
        # When a reach exceeds the arm's natural range, the torso rotates toward
        # it so the arm returns to plausible motion. Need: spine01 pivot + rest
        # rotation + its parent rotation (spine01 is driven by an op on spine02).
        self.spine01_pos = world_pos(self.spine01_idx)
        self.spine01_world_q_rest = world_rot(self.spine01_idx)
        self.spine02_world_q_rest = world_rot(self.spine02_idx)
        # spine01's parent is spine02
        self.spine01_parent_world_q = world_rot(self.spine02_idx)

        def _dist(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        # Bone lengths (constant) for the 2-bone IK solver
        self.r_upperarm_len = _dist(self.r_shoulder_pos, self.r_elbow_pos)
        self.r_forearm_len  = _dist(self.r_elbow_pos, self.r_wrist_pos)
        self.l_upperarm_len = _dist(self.l_shoulder_pos, self.l_elbow_pos)
        self.l_forearm_len  = _dist(self.l_elbow_pos, self.l_wrist_pos)

        # --- Leg IK rest data (procedural walk, Route B) ---------------------
        # Discover leg joints by their leaf bone name (Reallusion/CC rig:
        # <side>_Thigh / <side>_Calf / <side>_Foot, all under Hip). Defensive:
        # if any are missing, self.legs stays empty and the walk no-ops.
        def _find_leaf(name):
            for i, jp in enumerate(joints):
                if jp.split("/")[-1] == name:
                    return i
            return None

        hip_idx = _find_leaf("Hip")
        self.hip_idx          = hip_idx
        self.hip_rest_pos     = world_pos(hip_idx) if hip_idx is not None else None
        # Hip is never re-rotated by the rig driver, so its live world rotation
        # equals its rest world rotation — the static parent frame for the thighs.
        self.hip_world_q_rest = world_rot(hip_idx) if hip_idx is not None else Gf.Quatf(1, 0, 0, 0)
        # --- Pelvis-drop frame data (foot-placement IK item 2) --------------
        # The drop lowers the Hip in world -Y and raises the Waist by the same
        # amount, so the legs sink (more knee bend / longer reach) while the torso
        # and HEAD stay put — essential in first-person VR. Translations are in
        # the joint's PARENT-local frame, so cache the rest translations and the
        # world→local rotations needed to express a world -Y/+Y offset.
        root_idx = next((i for i, jp in enumerate(joints) if "/" not in jp), None)
        self._rlroot_world_q = world_rot(root_idx) if root_idx is not None else Gf.Quatf(1, 0, 0, 0)
        self.hip_rest_translate   = (translations[hip_idx]
                                     if hip_idx is not None else Gf.Vec3f(0, 0, 0))
        self.waist_rest_translate = translations[self.waist_idx]
        self.legs = {}
        for side in ("L", "R"):
            ti = _find_leaf(f"{side}_Thigh")
            ci = _find_leaf(f"{side}_Calf")
            fi = _find_leaf(f"{side}_Foot")
            if ti is None or ci is None or fi is None:
                continue
            toe_i = _find_leaf(f"{side}_ToeBase")   # optional (foot roll)
            thigh, knee, ankle = world_pos(ti), world_pos(ci), world_pos(fi)
            # Provisional knee-bend pole = the rest knee's offset from the
            # hip→ankle line. Refined to a guaranteed-FORWARD direction below;
            # on a near-straight rest leg this offset is tiny and can point
            # slightly backward, which would hyperextend (knee bends the wrong way).
            axis     = _normalize(_sub(ankle, thigh))
            ko       = _sub(knee, thigh)
            knee_off = _sub(ko, _scale(axis, _dot(ko, axis)))
            rest_local = {ti: rotations[ti], ci: rotations[ci], fi: rotations[fi]}
            if toe_i is not None:
                rest_local[toe_i] = rotations[toe_i]
            self.legs[side] = {
                "thigh_idx": ti, "calf_idx": ci, "foot_idx": fi, "toe_idx": toe_i,
                "thigh_pos": thigh, "knee_pos": knee, "ankle_pos": ankle,
                "pole": knee_off,   # refined just below
                "thigh_len": _dist(thigh, knee), "calf_len": _dist(knee, ankle),
                # Ankle→toe length (forefoot) — used to lift the foot target while
                # it rolls so the toe/heel don't dip through the floor. Defaults to
                # 0.12 m if the rig has no toe joint.
                "foot_len": (_dist(ankle, world_pos(toe_i))
                             if toe_i is not None else 0.12),
                "thigh_q_rest": world_rot(ti),
                "calf_q_rest":  world_rot(ci),
                "foot_q_rest":  world_rot(fi),
                "toe_q_rest":   world_rot(toe_i) if toe_i is not None else None,
                "rest_thigh_dir": _normalize(_sub(knee, thigh)),
                "rest_calf_dir":  _normalize(_sub(ankle, knee)),
                # pristine local rotations to restore when the walk is toggled off
                "rest_local": rest_local,
            }
        if self.legs:
            # Anatomical FORWARD in the rest world frame, derived from geometry
            # (no facing assumption): forward = (hip-to-hip axis) × down. Knees
            # only bend forward, so this is the reliable IK pole. Keep a leg's own
            # rest offset only if it already agrees with forward.
            def _cross(a, b):
                return Gf.Vec3f(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
                                a[0]*b[1]-a[1]*b[0])
            down = Gf.Vec3f(0.0, -1.0, 0.0)
            if "L" in self.legs and "R" in self.legs:
                hip_axis = _normalize(_sub(self.legs["R"]["thigh_pos"],
                                           self.legs["L"]["thigh_pos"]))
                fwd_rest = _normalize(_cross(hip_axis, down))
            else:
                hip_axis = Gf.Vec3f(1.0, 0.0, 0.0)
                fwd_rest = Gf.Vec3f(0.0, 0.0, -1.0)
            # Avatar RIGHT axis in the rest/solve frame — the medial-lateral hinge
            # the foot pitches about for heel-strike / toe-off (foot roll).
            self.leg_right_axis = hip_axis
            # Local-units-per-metre (rig may be authored in cm and scaled): a calf
            # joint's local translation length = thigh bone length in local units,
            # vs the same bone measured in world metres. Used to convert the metric
            # pelvis drop into the joint-local translation it is written in.
            any_leg = next(iter(self.legs.values()))
            wl = _dist(any_leg["thigh_pos"], any_leg["knee_pos"])
            ll = _len(translations[any_leg["calf_idx"]])
            self.leg_local_per_m = (ll / wl) if wl > 1e-6 else 1.0
            for leg in self.legs.values():
                ko = leg["pole"]
                # Drop any vertical component, keep horizontal; force forward.
                ko = Gf.Vec3f(float(ko[0]), 0.0, float(ko[2]))
                if _len(ko) > 1e-3 and _dot(_normalize(ko), fwd_rest) > 0.3:
                    leg["pole"] = _normalize(ko)
                else:
                    leg["pole"] = fwd_rest
            toes = [s for s, l in self.legs.items() if l["toe_idx"] is not None]
            print(f"[avatar_xr_control] Leg IK ready: {sorted(self.legs)} "
                  f"fwd=({fwd_rest[0]:+.2f},{fwd_rest[1]:+.2f},{fwd_rest[2]:+.2f}) "
                  f"toes={sorted(toes)}")
        else:
            self.leg_right_axis = Gf.Vec3f(1.0, 0.0, 0.0)
            self.leg_local_per_m = 1.0
            print("[avatar_xr_control] Leg IK: no leg joints found "
                  "(procedural walk disabled)")

        # --- Torso self-collision volume (rest frame) — BASE geometry ---
        # A vertical ELLIPTICAL capsule the arm-IK swivel keeps the elbows/
        # forearms/hands out of. Here we cache only the BASE anchors; the actual
        # volume (centre offset, width, depth, extent) is built by the extension
        # via _rebuild_torso() from live-tunable params, because the shoulder
        # JOINT line sits at the upper back — the chest is forward of it (avatar
        # forward = world -Z) — so the centre must be shiftable onto the chest.
        waist_pos          = world_pos(self.waist_idx)
        self.waist_rest_pos = waist_pos   # rest-frame anchor for crouch follow
        self.shoulder_mid  = _scale(_add(self.r_shoulder_pos, self.l_shoulder_pos), 0.5)
        self.shoulder_span = _len(_sub(self.r_shoulder_pos, self.l_shoulder_pos))
        self.waist_y       = float(waist_pos[1])
        self.torso_height  = max(0.05, float(self.shoulder_mid[1] - self.waist_y))
        self.fwd_dir       = Gf.Vec3f(0.0, 0.0, -1.0)   # avatar faces world -Z
        # Sensible defaults so collision works before any UI tuning; overwritten
        # by _rebuild_torso(). torso_top/bottom share x,z (vertical axis).
        self.torso_top    = Gf.Vec3f(self.shoulder_mid[0], self.shoulder_mid[1],
                                     self.shoulder_mid[2])
        self.torso_bottom = Gf.Vec3f(self.shoulder_mid[0],
                                     self.waist_y - self.torso_height,
                                     self.shoulder_mid[2])
        self.torso_half_x = 0.13
        self.torso_half_z = 0.14

        # Waist local rest rotation — base for the static 5° forward tilt
        # (the OSC-driven waist lean was removed; it felt unnatural).
        self.waist_local_rest_q = rotations[self.waist_idx]

        # Avatar ROOT prim rest frame (path resolved + follow op reset at the
        # top of __init__). Camera-follow writes the root's translate and the
        # dedicated follow-yaw op each frame; cache the rest transform taken at
        # init: live world = root_delta ∘ rest.
        #
        # IMPORTANT frame note: the asset's root-LOCAL frame is Z-up
        # (Reallusion/CC import; a rotation op on the prim converts to the
        # stage's Y-up). All camera-follow math therefore stays in WORLD frame
        # (Y-up, metres) — never use root-local vectors there.
        root_prim = stage.GetPrimAtPath(self.root_path)
        root_m = Gf.Matrix4d(1.0)
        if root_prim.IsValid():
            root_m = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
        self.root_rest_xform     = root_m
        self.root_rest_xform_inv = root_m.GetInverse()
        # World-frame rest data for the camera follow: the head-bone world
        # position and the root's rest translation (= the yaw pivot, since the
        # translate op is the outermost op and the parent is identity).
        head_w = self._rest_world[self.head_idx].ExtractTranslation()
        self.head_rest_world     = Gf.Vec3d(head_w)
        self.root_rest_translate = root_m.ExtractTranslation()

        # --- Procedural finger-curl axes (controller fallback) ---------------
        # When optical hand tracking is absent (controllers held), the fingers
        # are curled procedurally about the knuckle line (index→pinky) — the
        # axis all four fingers flex around. Cache, per finger joint, that axis
        # expressed in the joint's PARENT-local frame plus its rest local
        # rotation, so a curl is rest_local · axisAngle(axis, θ) (a pre-multiply
        # in the parent frame). SteamVR-Skeletal-Input-style fallback.
        self.finger_curl = {}

        def _parent_key(k):
            # r_index1 → r_hand ; r_index2 → r_index1 ; r_index3 → r_index2
            if k[-1] == "1":
                return ("r_hand" if k.startswith("r_") else "l_hand")
            return k[:-1] + str(int(k[-1]) - 1)

        def _cross(a, b):
            return Gf.Vec3f(a[1] * b[2] - a[2] * b[1],
                            a[2] * b[0] - a[0] * b[2],
                            a[0] * b[1] - a[1] * b[0])

        for hand_right in (True, False):
            pre = "r_" if hand_right else "l_"
            idx1 = self.finger_idx.get(pre + "index1")
            pky1 = self.finger_idx.get(pre + "pinky1")
            if idx1 is None or pky1 is None:
                continue
            k_world = _normalize(_sub(world_pos(pky1), world_pos(idx1)))  # knuckle line
            hand_idx = self.r_hand_idx if hand_right else self.l_hand_idx
            hand_pos = world_pos(hand_idx)
            # Pick the knuckle-line sense whose +rotation flexes the fingers TOWARD
            # the palm. The palm normal n = fingerdir × knuckleline is purely the
            # palmar/dorsal axis (no radial component), and the THUMB always sits on
            # the palmar side, so the sign of n·(hand→thumb) cleanly says which way
            # is palmar — then choose the axis sense whose +curl moves there.
            sign = 1.0
            th_axis = None
            mid1 = self.finger_idx.get(pre + "mid1")
            mid3 = (self.finger_idx.get(pre + "mid3")
                    or self.finger_idx.get(pre + "mid2"))
            th1  = self.finger_idx.get(pre + "thumb1")
            th3  = self.finger_idx.get(pre + "thumb3")
            if mid1 is not None and mid3 is not None:
                fdir = _normalize(_sub(world_pos(mid3), world_pos(mid1)))  # distal
                n = _normalize(_cross(fdir, k_world))                      # palmar/dorsal
                if th1 is not None:
                    palmar = _sub(world_pos(th1), hand_pos)
                    if _dot(n, palmar) < 0.0:
                        n = _scale(n, -1.0)
                # +rotation about (sign·k) moves the fingertip along sign·(k × fdir);
                # want that toward the palm (+n).
                if _dot(_cross(k_world, fdir), n) <= 0.0:
                    sign = -1.0
                # Thumb flexes about cross(thumbdir, palmnormal). A +curl about it
                # moves the thumb tip toward the palm (+n) — the SAME side the
                # fingers flex to — so the sign is correct BY CONSTRUCTION (no sign
                # test; the earlier test flipped it the wrong way). Works even when
                # the thumb is STRAIGHT at rest (a per-joint bone×bone hinge can't —
                # it's degenerate there). thumbdir = thumb1→thumb3.
                if th1 is not None and th3 is not None:
                    a = _cross(_sub(world_pos(th3), world_pos(th1)), n)
                    if _len(a) > 1e-5:
                        th_axis = _normalize(a)
            for key, idx in self.finger_idx.items():
                if not key.startswith(pre):
                    continue
                pidx = self.finger_idx.get(_parent_key(key), hand_idx)
                parent_world_q = world_rot(pidx)
                # Thumb uses its own (palm-relative) hinge; fingers use the knuckle line.
                src = (th_axis if ("thumb" in key and th_axis is not None)
                       else _scale(k_world, sign))
                axis_local = _normalize(_quat_rotate(_quat_conj(parent_world_q), src))
                self.finger_curl[key] = {
                    "idx":   idx,
                    "rest":  self._rotations[idx],
                    "axis":  axis_local,
                    "seg":   key[-1],            # "1"/"2"/"3" segment
                    "thumb": "thumb" in key,
                }

        # --- Head-region geometry for a NON-DEFORMING first-person head hide ---
        # The head-chop scales the head joint (deforms the mesh) and a near-clip
        # plane would hide close objects too. Instead, if the asset exposes the
        # head as separable geometry, we hide that geometry directly (mesh
        # visibility / a cutout material on the face-skin subset) — no geometry
        # change, no clipping, objects near the eyes still render. If nothing
        # separable is found, set_head_hidden() falls back to the head-chop.
        self._head_meshes  = []     # whole-mesh head parts (eyes/teeth/lashes/hair/…)
        self._head_subsets = []     # (subset_path, original_bound_material_path)
        self._head_hidden_mat_path = skel_path + "/xr_head_hidden_mat"
        # Parts hidden along with the head; SKIN tokens mark the FACE itself —
        # region-hide is only viable when the face skin is coverable, otherwise the
        # face would still block the view and we must use the head-chop fallback.
        _PART_TOKENS = ("eye", "cornea", "tearline", "eyelash", "lash", "brow",
                        "teeth", "tooth", "tongue", "hair", "head", "face")
        _SKIN_TOKENS = ("head", "face")
        face_covered = False

        def _bound_mat_name(prim):
            try:
                mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                if mat and mat.GetPrim().IsValid():
                    return mat.GetPath().name.lower(), mat.GetPath()
            except Exception:
                pass
            return "", None

        try:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(self.root_path)):
                if prim.IsA(UsdGeom.Mesh):
                    nm = prim.GetName().lower()
                    mn, _ = _bound_mat_name(prim)
                    # Only WHOLE-mesh head parts; never the body skin mesh.
                    if (any(t in nm or t in mn for t in _PART_TOKENS)
                            and "body" not in nm and "body" not in mn):
                        self._head_meshes.append(prim.GetPath())
                        if any(t in nm or t in mn for t in _SKIN_TOKENS):
                            face_covered = True
                elif prim.IsA(UsdGeom.Subset):
                    nm = prim.GetName().lower()
                    mn, mpath = _bound_mat_name(prim)
                    if any(t in nm or t in mn for t in _SKIN_TOKENS):
                        self._head_subsets.append((prim.GetPath(), mpath))
                        face_covered = True
        except Exception as e:
            print(f"[avatar_xr_control] head-region discovery error: {e}")
        # Region-hide only if the FACE skin is coverable (else fall back to chop).
        self._head_region_ok = face_covered
        print(f"[avatar_xr_control] head hide: {len(self._head_meshes)} meshes, "
              f"{len(self._head_subsets)} subsets, face_covered={face_covered}"
              f"{'' if self._head_region_ok else '  → head-chop fallback'}")

        print(f"[avatar_xr_control] Setup OK — {len(joints)} joints")

    def write_joint_rotation(self, idx: int, quatf: Gf.Quatf):
        self._rotations[idx] = quatf
        if self._defer:
            self._rot_dirty = True
        else:
            self.anim.GetRotationsAttr().Set(Vt.QuatfArray(self._rotations))

    def write_joint_translation(self, idx: int, vec: Gf.Vec3f):
        self._translations[idx] = vec
        if self._defer:
            self._trans_dirty = True
        else:
            self.anim.GetTranslationsAttr().Set(Vt.Vec3fArray(self._translations))

    def set_deferred(self, on: bool):
        """Batch mode for the tracking loop: a tracked frame makes ~40-50 joint
        writes (arms, hands, 30 finger joints, legs, head), and each immediate
        write serialises the FULL joint array into USD and triggers change
        processing. Deferred, the writes collapse into one Set per attribute per
        frame via flush(). Turning batching off flushes pending writes, so
        immediate-write semantics are restored for UI callbacks."""
        self._defer = bool(on)
        if not on:
            self.flush()

    def flush(self):
        if self._rot_dirty:
            self._rot_dirty = False
            self.anim.GetRotationsAttr().Set(Vt.QuatfArray(self._rotations))
        if self._trans_dirty:
            self._trans_dirty = False
            self.anim.GetTranslationsAttr().Set(Vt.Vec3fArray(self._translations))

    def joint_world_positions(self):
        """LIVE world positions of every joint (indexed by joint index),
        evaluating the bound xr_anim — render-faithful, so they match the
        skinned mesh. Returns None if the query is unavailable. The joint PRIM
        xforms stay at bind pose (the mesh is posed via UsdSkel skinning), so
        this query — not ComputeLocalToWorldTransform on the bone prims — is the
        only correct source of the posed joint locations."""
        q = getattr(self, "_skel_query", None)
        if q is None:
            return None
        xf = UsdGeom.XformCache(Usd.TimeCode.Default())
        xforms = q.ComputeJointWorldTransforms(xf, False)   # atRest=False → posed
        if not xforms:
            return None
        return [_vec3f(m.ExtractTranslation()) for m in xforms]

    def _set_head_chop(self, hidden: bool):
        """First-person 'head chop' (VRChat technique): scale the head joint to
        ~0 so the head mesh collapses and never blocks the eye-level camera.
        DEFORMS the mesh — used only as a fallback."""
        s = 0.001 if hidden else 1.0
        self._scales[self.head_idx] = Gf.Vec3h(s, s, s)
        self.anim.GetScalesAttr().Set(Vt.Vec3hArray(self._scales))

    def _ensure_hidden_material(self, stage):
        """A cutout material (opacity 0 below an opacity threshold) that RTX fully
        discards — used to make the face-skin subset invisible without deforming."""
        prim = stage.GetPrimAtPath(self._head_hidden_mat_path)
        if prim and prim.IsValid():
            return UsdShade.Material(prim)
        mat = UsdShade.Material.Define(stage, self._head_hidden_mat_path)
        sh = UsdShade.Shader.Define(stage, self._head_hidden_mat_path + "/Surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def _set_head_region_hidden(self, hidden: bool):
        """Hide/show the separable head geometry: mesh visibility for whole-mesh
        head parts, and a cutout material on the face-skin subset(s). No geometry
        deformation, no clipping — objects near the eyes still render."""
        stage = self.anim.GetPrim().GetStage()
        for mpath in self._head_meshes:
            img = UsdGeom.Imageable(stage.GetPrimAtPath(mpath))
            img.MakeInvisible() if hidden else img.MakeVisible()
        if self._head_subsets:
            hide_mat = self._ensure_hidden_material(stage) if hidden else None
            for spath, orig in self._head_subsets:
                api = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(spath))
                if hidden:
                    api.Bind(hide_mat)
                elif orig is not None:
                    api.Bind(UsdShade.Material(stage.GetPrimAtPath(orig)))
                else:
                    api.UnbindDirectBinding()

    def set_head_hidden(self, hidden: bool, use_chop: bool = False):
        """Hide the head for first person. Default: make the head's render region
        invisible (no deformation, no clipping, close objects unaffected). Falls
        back to the joint-scale head-chop when the asset has no separable head
        region, or when use_chop is forced."""
        if self._head_region_ok and not use_chop:
            self._set_head_region_hidden(hidden)
            if self._scales[self.head_idx] != Gf.Vec3h(1, 1, 1):  # undo any chop
                self._set_head_chop(False)
        else:
            if self._head_region_ok:        # switching to chop → restore region
                self._set_head_region_hidden(False)
            self._set_head_chop(hidden)


# ---------------------------------------------------------------------------
# XR helpers
# ---------------------------------------------------------------------------

def _get_pose(device):
    """Virtual-world (stage-space) pose — use for orientations.
    _correct_xr_quat and IK fore_parent are both in stage space, so the rotation
    from this source composes correctly. Falls back to raw if VW is unavailable."""
    if device is None:
        return None
    try:
        m = device.get_virtual_world_pose()
        if m is not None:
            return m
    except Exception:
        pass
    try:
        poses = device.get_all_raw_poses()
        desc  = poses.get('') if poses else None
        if desc is not None and desc.validity_flags != 0:
            return Gf.Matrix4d(desc.pose_matrix)
    except Exception:
        pass
    return None


def _get_pose_raw(device):
    """Physical-tracking-space pose — use for IK positions only.
    Raw poses are unaffected by Kit's virtual-world origin shifts, so relative
    hand/head positions stay stable when Kit's locomotion fires on the thumbstick."""
    if device is None:
        return None
    try:
        poses = device.get_all_raw_poses()
        desc  = poses.get('') if poses else None
        if desc is not None and desc.validity_flags != 0:
            return Gf.Matrix4d(desc.pose_matrix)
    except Exception:
        pass
    try:
        m = device.get_virtual_world_pose()
        if m is not None:
            return m
    except Exception:
        pass
    return None





def _bone_rotation_from_vectors(from_dir, to_dir, rest_world_q, parent_world_q):
    """Compute local joint rotation that points a bone from rest direction toward a target direction."""
    delta       = _quat_from_to(from_dir, to_dir)
    world_q     = _quat_mul(delta, rest_world_q)
    local_q     = _quat_mul(_quat_conj(parent_world_q), world_q)
    return local_q, world_q


def _swing_to_local(rest_dir_w, target_dir_w, bone_rest_w,
                    parent_rest_w, parent_live_w, bone_rest_local):
    """Bake a bone's IK rotation as a minimal swing (rest_dir → target_dir) and
    return (local_rotation, bone_live_world_rotation).

    Unlike _bone_rotation_from_vectors this is EXACT at rest: the swing is
    expressed in the parent-LIVE frame and composed on top of the bone's authored
    rest LOCAL rotation, so when target_dir == rest_dir (and the parent is
    unmoved) it returns the stored rest local verbatim. That avoids the
    world→local reconstruction error (rig scale / axis conversion) that
    _bone_rotation_from_vectors carries — invisible on the always-moving arms but
    enough to visibly tilt a leg that is momentarily at its rest pose.

    All *_w args are world-space rotation quats; directions are world unit vectors.
    """
    # Accurate live world rotation (world swing on the accurate rest world quat),
    # exact at rest — used as the parent frame for the next bone down the chain.
    bone_world_live = _quat_mul(_quat_from_to(rest_dir_w, target_dir_w), bone_rest_w)
    # Same swing expressed in the parent-local frame, composed onto rest local.
    rest_pl = _normalize(_quat_rotate(_quat_conj(parent_rest_w), rest_dir_w))
    targ_pl = _normalize(_quat_rotate(_quat_conj(parent_live_w), target_dir_w))
    local   = _quat_mul(_quat_from_to(rest_pl, targ_pl), bone_rest_local)
    return local, bone_world_live


def _trigger_value(device):
    """Return the controller trigger as 0..1 (index-finger driver), or 0.0."""
    if device is None:
        return 0.0
    best = 0.0
    for gesture in ("value", "click", "force", "touch"):
        try:
            if device.has_input_gesture("trigger", gesture):
                v = float(device.get_input_gesture_value("trigger", gesture))
                if v > best:
                    best = v
        except Exception:
            pass
    return best


def _squeeze_value(device):
    """Return the controller grip/squeeze as 0..1 (the fist driver), or 0.0.
    Kept separate from the trigger so the index finger can track the trigger
    independently of the other fingers."""
    if device is None:
        return 0.0
    best = 0.0
    for comp in ("squeeze", "grip"):
        for gesture in ("value", "click", "force", "touch"):
            try:
                if device.has_input_gesture(comp, gesture):
                    v = float(device.get_input_gesture_value(comp, gesture))
                    if v > best:
                        best = v
            except Exception:
                pass
    return best


# Thumbstick component name varies by controller profile — probe candidates.
_STICK_COMPONENTS = ("thumbstick", "joystick", "trackpad", "stick")


def _stick_xy(device):
    """Return the controller thumbstick as (x, y) in -1..1, or (0, 0).
    Probes common component names guarded by has_input_gesture so an unknown
    profile never raises. y is OpenXR convention: +y = stick pushed forward."""
    if device is None:
        return 0.0, 0.0
    for comp in _STICK_COMPONENTS:
        try:
            if device.has_input_gesture(comp, "x") and device.has_input_gesture(comp, "y"):
                x = float(device.get_input_gesture_value(comp, "x"))
                y = float(device.get_input_gesture_value(comp, "y"))
                return x, y
        except Exception:
            pass
    return 0.0, 0.0


# ---------------------------------------------------------------------------
# XR capture / replay — record a live session's RAW device data to a JSONL file,
# then play it back later (no headset) to fine-tune the IK against real motion.
# ---------------------------------------------------------------------------

def _mat_to_list(m):
    return [float(m[i][j]) for i in range(4) for j in range(4)]


def _list_to_mat(v):
    return Gf.Matrix4d(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
                       v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15])


class _RawPoseDesc:
    """Mimics an OpenXR raw-pose descriptor for replay (pose_matrix + validity)."""
    def __init__(self, pose_matrix, validity_flags):
        self.pose_matrix = pose_matrix
        self.validity_flags = validity_flags


class _PlaybackDevice:
    """Stand-in for an XR input device that returns RECORDED data, so the entire
    tracking pipeline runs unchanged off a recording. Implements only the slice
    of the device API the extension actually reads."""
    def __init__(self, rec):
        self._r = rec or {}

    def get_virtual_world_pose(self, pose=None):
        if pose is None:
            v = self._r.get("vw")
            return _list_to_mat(v) if v else None
        fl = self._r.get("fingers", {}).get(pose)
        return _list_to_mat(fl) if fl else None

    def get_all_raw_poses(self):
        v = self._r.get("raw")
        if not v:
            return {}
        return {"": _RawPoseDesc(_list_to_mat(v), int(self._r.get("raw_valid", 0)))}

    def has_input_gesture(self, comp, gest):
        return f"{comp}.{gest}" in self._r.get("gestures", {})

    def get_input_gesture_value(self, comp, gest):
        return float(self._r.get("gestures", {}).get(f"{comp}.{gest}", 0.0))

    def get_hand_tracking_data_source(self):
        return self._r.get("src")

    # Used only by the finger-diagnostics button; harmless stubs for replay.
    def has_input(self, comp):
        return any(k.startswith(comp + ".") for k in self._r.get("gestures", {}))

    def get_input_gesture_names(self, comp):
        return [k.split(".", 1)[1] for k in self._r.get("gestures", {})
                if k.startswith(comp + ".")]


# ---------------------------------------------------------------------------
# Extension
# ---------------------------------------------------------------------------

class AvatarXRControlExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        self._skel_path     = DEFAULT_SKEL_PATH
        self._skel          = None
        self._stage_sub     = None
        self._tracking_task = None
        self._xr_active     = False
        self._xr_seq_busy   = False   # a Start/Stop XR sequence is running
        self._xr            = None
        self._head_dev      = None
        self._left_dev      = None
        self._right_dev     = None
        self._r_wrist_offset = None  # set by Calibrate button (T-pose)
        self._l_wrist_offset = None
        # IK calibration: XR wrist world pos captured in T-pose + map scale.
        self._r_wrist_xr_tpose = None
        self._l_wrist_xr_tpose = None
        # Head→shoulder offset (XR space) captured in T-pose. Each frame the live
        # shoulder anchor = live_head_pos + this offset, so the arm reference
        # follows the user's head as they move/turn (not frozen at calibration).
        self._r_shoulder_off = None
        self._l_shoulder_off = None
        self._real_arm_len = 1.0
        self._ik_scale = 1.0      # avatar_arm_len / real_arm_len
        self._ik_scale_mult = 1.0 # fine-tune multiplier: 1.0 = avatar reach matches
                                  # the user 1:1 (was 1.1, which capped the arm at
                                  # ~89% of the user's extension → ~30% of frames
                                  # clamped; far reaches now extend via clav follow)
        self._ik_enabled = False
        self._armvec_tick = 0
        if DEBUG_FILES:
            try:
                open(_data_path("_armvec_debug.txt"), "w").close()
            except Exception:
                pass
        # Elbow pole bias. Down dominates so elbows hang down like a relaxed
        # human arm for the common reaches; outward (per-arm) + back keep the
        # elbow off the torso. With kinematic colliders, very strong outward bias
        # ensures natural arm clearance and prevents visual intersection.
        # All live-tunable via the UI buttons.
        self._pole_down = 2.0     # weight of downward component (dominant)
        self._pole_back = 2.0     # weight of backward (+Z) component (strong for clearance)
        self._pole_out  = 1.8     # weight of outward (sideways) component (strong for clearance)
        # Extra DOWN bias as the arm drops below horizontal, so the elbow re-asserts
        # the "points down" rule when the hand comes from an up-reach to below the
        # head (otherwise back/out from the up pose lingers and down never wins).
        self._pole_down_below = 2.0

        # VRIK-style "bend goal": bias the elbow swivel by the controller roll so
        # the elbow lifts/drops with the user's forearm twist. DISABLED (0.0): the
        # roll angle wraps at ±180°, so near palm-rotated poses it flipped sign and
        # swung the elbow ~180° around the arm axis — the replay diag showed the
        # elbow jumping 20–40 cm/frame while the hand was still (21% of R frames).
        # Re-enable only with an unwrapped + smoothed roll (see _wrist_roll).
        self._elbow_roll_weight = 0.0
        self._wrist_roll = {True: 0.0, False: 0.0}

        # Procedural finger curl (controller fallback, SteamVR-Skeletal-Input
        # style): with controllers held there is no optical finger data, so the
        # trigger curls the index and grip/squeeze curls the other fingers + a
        # softer thumb. Optical hand tracking, when present, always takes priority.
        self._finger_curl_on   = True
        self._finger_curl_deg  = 80.0   # full-curl angle (scaled per segment)
        self._finger_thumb     = 0.6    # thumb curl as a fraction of grip
        self._finger_ease_rate = 12.0   # ease rate (1/s) so fingers don't pop
        self._finger_drive     = {True: [0.0, 0.0], False: [0.0, 0.0]}  # eased [grip, trigger]

        # VRIK-style clavicle/shoulder follow: each frame the clavicle is rotated
        # a FRACTION (_clav_weight) of the way toward pointing at the hand target,
        # clamped to _clav_max radians. This lifts/protracts the shoulder on high,
        # forward and cross-body reaches so the arm doesn't dig into the torso —
        # the hand stays on the controller; only the arm ROOT moves.
        self._clav_follow = True
        self._clav_weight = 0.60   # was 0.55: a touch more shoulder lead on reaches
        self._clav_max    = math.radians(25)   # was 20: extends the forward protraction
        # Max forward shoulder protraction on a full cross-body reach (rad). Swings
        # the upper arm in front of the torso so it doesn't cut through the chest.
        # Raised 30→40°: the analytic swivel alone can't clear cross-body reaches
        # (the shoulder sits inside the torso depth band), so lean harder on the
        # shoulder swing. Full clearance still needs the PhysX self-collision layer.
        self._protract_max = math.radians(40)
        # #1 Graceful reach extension: at the edge of reach the shoulder girdle
        # protracts toward the hand, so far/forward reaches extend by ~this many
        # metres and lead with the shoulder instead of stopping dead at the soft
        # cap. Matches what the clavicle follow above can physically deliver (the
        # cap is raised by this, the protraction supplies the extra reach). It does
        # NOT let the arm reach beyond shoulder-protraction + arm length — a target
        # well past that (e.g. the synthetic 1.4× poses) is physically out of range.
        self._clav_reach_bonus = 0.06

        # Torso self-collision volume placement/size (live-tunable, metres).
        # The shoulder-joint line sits at the upper back; _torso_fwd shifts the
        # capsule centre forward onto the chest (avatar forward = world -Z).
        self._torso_fwd    = 0.06   # forward shift of the capsule centre
        self._torso_half_x = 0.16   # lateral half-width (fitted to this avatar)
        self._torso_half_z = 0.13   # front-back half-depth (fitted to this avatar)
        self._torso_autofit = False  # mesh auto-fit off (unreliable frame on this asset)
        # Soft body collision: push the hand target out of the torso so the
        # hand/forearm rest on the body surface instead of clipping through.
        self._body_push    = True

        # --- PhysX scene-query collision (supplements the analytic torso) ---
        # A secondary correction layer: real PhysX overlap/sweep queries against
        # actual colliders catch what the single analytic torso ellipse can't —
        # the head, hips, the OPPOSITE arm (self), and scene objects (env). The
        # analytic capsule stays the primary smooth solver; physics only nudges
        # the result when real geometry is violated. ON by default: the analytic
        # swivel alone cannot clear cross-body reaches (the shoulder sits inside
        # the torso), so the real torso collider is needed for clean clearance.
        # The scene + proxies build lazily on the first tracked frame.
        self._phys_collision  = True    # master toggle for the physics layer
        self._phys_env        = True    # sub-toggle: clamp hand to scene objects
        self._phys_probe_r    = 0.02    # extra probe inflation over the limb radius
        self._phys_max_steps  = 8       # max swivel steps searched for self-clear
        self._phys_scene_ready = False  # set once the scene + proxies are built
        self._phys_setup_tried = False  # one-shot guard for the lazy auto-setup
        self._phys_sim_running = False  # gravity-free sim attached for queries
        self._phys_time        = 0.0    # accumulated sim time (s)
        # Per-arm rest-frame (shoulder, elbow, wrist) of the LAST solved pose —
        # used to position the opposite-arm collider proxies for arm-vs-arm.
        self._arm_solved   = {True: None, False: None}
        # Per-arm eased swivel angle chosen by the physics self-refine (rad).
        self._phys_phi     = {True: None, False: None}
        # Shoulder push perturbs the IK shoulder without moving the rendered
        # (clavicle-rooted) shoulder, so it adds snap for little visible gain —
        # OFF by default; the clavicle "Shoulder follow" handles shoulder lift.
        self._phys_shoulder_push = False
        # Eased body-push correction per arm (rest-frame vector). The hand glides
        # onto/off the body instead of snapping when the push turns on/off.
        self._push_corr    = {True: Gf.Vec3f(0, 0, 0), False: Gf.Vec3f(0, 0, 0)}
        self._push_rate    = 20.0   # ease rate (1/s) for the body-push correction

        # --- Anatomical joint limits ---
        # Keep the arm in a human range so it can't reach impossible poses (e.g.
        # the right arm behind the back and across to the left). Defined in the
        # avatar-local frame the IK solves in (+X right, +Y up, -Z forward).
        self._limits_on    = True
        self._lim_back     = 0.45   # max backward (+Z) component of arm dir
        self._lim_cross_f  = -0.65  # right arm: most-left X reachable IN FRONT
        self._lim_cross_b  = 0.02   # right arm: most-left X reachable BEHIND
        self._lim_reach    = 0.98   # max reach as fraction of (len1+len2)
        self._lim_soft     = 0.12   # soft-clamp band for the cone (0 → hard wall)
        self._lim_wrist    = math.radians(85)  # max hand rotation from neutral. 70°
                                               # clamped real wrist flexion on ~49%
                                               # of frames (replay diag); 85° admits
                                               # genuine range, still anatomically sane

        # --- Body-turn-to-reach (Phase 2) ---
        # When an arm reach exceeds the natural cone, rotate the torso toward the
        # reach so the arm returns to plausible motion without dropping the hand
        # off the controller or hitting the mesh.
        self._body_turn    = True
        self._body_turn_max = math.radians(25)  # max torso yaw assist (rad)
        self._body_turn_smooth = 6.0  # exp ease rate (1/s), like _follow_yaw_rate
        self._spine_yaw = None  # live smoothed spine yaw (rad), None = unset (tighter)

        # Shoulder anchor yaw (separate from body yaw): shoulders track large sustained
        # turns but not momentary head glances, reducing wobble during normal tracking.
        self._shoulder_yaw = None  # low-pass filtered yaw for shoulder anchor (rad)

        # Phase 1: One Euro Filter on the 3 tracked positions (head + 2 hands).
        self._smooth_on    = True
        self._smooth_cutoff = 1.0   # min_cutoff Hz: lower = less jitter, more lag
        self._smooth_beta   = 0.02  # higher = less lag during fast motion
        self._filt_head  = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)
        self._filt_rhand = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)
        self._filt_lhand = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)
        # Orientation low-pass on the controller wrist quaternion (positions were
        # filtered but rotation was not, so controller spin jitter reached the
        # hands). Adaptive: smooths when still, low lag when turning fast.
        self._rot_cutoff = 2.0    # min_cutoff Hz for wrist orientation
        self._rot_beta   = 0.10   # higher = less lag during fast wrist motion
        self._filt_rrot  = QuatOneEuro(self._rot_cutoff, self._rot_beta)
        self._filt_lrot  = QuatOneEuro(self._rot_cutoff, self._rot_beta)

        self._xr_cam_path    = "/_xr/stage/xrCamera"

        # Measured per-frame time (s), updated in _tracking_loop. All exponential
        # eases use this instead of a hardcoded 60 fps step, so they stay correctly
        # paced (not sluggish) when the real frame rate is low.
        self._frame_dt       = 0.016
        self._last_frame_t   = None

        # Tracking-spike rejection: occasional dropouts/reacquisitions teleport a
        # tracked point >2 m in one frame. Jumps faster than _spike_vmax are held
        # at the previous position for up to _spike_max_hold frames (a sustained
        # move past that is accepted, so genuine relocations aren't frozen).
        self._spike_vmax     = 6.0    # m/s; above human hand speed, below glitches
        self._spike_max_hold = 4
        self._last_pos       = {}     # last accepted position per channel
        self._spike_hold     = {}     # consecutive-reject counter per channel

        # --- XR capture / replay ---
        # Record raw device data each frame during a live session to a JSONL file,
        # then replay it (no headset) to fine-tune the IK against real motion. The
        # recording's first line is a calibration snapshot so replay reproduces the
        # same hand→avatar mapping; tuning params stay live so they can be adjusted.
        self._rec_path        = _data_path("_xr_recording.jsonl")
        self._rec_enabled     = False
        self._rec_file        = None
        self._rec_t0          = 0.0
        self._rec_count       = 0
        self._play_enabled    = False
        self._play_frames     = []
        self._play_idx        = 0
        self._play_loop       = True
        self._play_saved_devs = None
        self._live_calib      = None      # live calibration stashed during replay
        self._play_prev_t     = None      # recorded timestamp of the last replayed frame
        self._play_started_loop = False   # replay started the loop (offline) → stop it
        self._replay_capturing = False    # a replay-metrics capture is running
        self._replay_capture_path = _data_path("_replay_capture.csv")

        # Locomotion: the right thumbstick moves the XR ORIGIN (camera rig);
        # the avatar follows the camera via _apply_camera_follow. Step in metres.
        self._move_step = 0.25
        # Right thumbstick locomotion: smooth glide per frame.
        self._stick_loco_on = True
        self._stick_speed   = 1.5    # metres per second at full deflection
        self._stick_deadz   = 0.15   # ignore small stick noise

        # --- Procedural walk (Route B): synthesise a stepping gait from the
        # avatar root's horizontal velocity and solve 2-bone IK per leg so the
        # feet plant on the ground instead of sliding. No leg trackers needed.
        # OFF by default so it can never disturb the validated arm/hand tracking.
        self._legs_on        = True   # on by default
        self._gait_phase     = 0.0    # global gait cycle phase [0,1) (L leads)
        self._gait_speed     = 0.0    # smoothed body speed (m/s)
        # Gait constants — tuned in-headset and hardcoded here.
        self._gait_speed_min    = 0.06  # below this the avatar stands (no stepping)
        self._gait_lift         = 0.06  # swing-foot apex height (m)
        self._gait_stride_mult  = 0.3   # stride length scale
        self._gait_cadence_mult = 1.2   # steps-per-second scale
        # Foot-placement IK extras (pelvis drop / foot roll).
        self._gait_drop_max  = 0.14     # max pelvis drop to let a leg reach (m)
        self._pelvis_drop    = 0.0      # smoothed live pelvis drop (m)
        self._gait_heel_deg  = 14.0     # heel-strike / swing dorsiflexion angle
        self._gait_toe_deg   = 26.0     # toe-off plantarflexion + toe bend angle
        self._gait_roll_sign = 1.0      # flip if heel/toe roll is inverted on rig
        self._gait_heel_frac = 0.45     # heel offset behind ankle as a fraction of
                                        # foot_len (for the foot-roll floor lift)
        self._gait_heading   = None   # last good travel direction (world, Y-up)
        self._gait_root_prev = None   # previous body XZ for the velocity calc
        self._body_pos       = None   # clean body world XZ (set by camera follow)
        self._body_yaw_live  = 0.0    # live body yaw (set by camera follow)
        self._foot_state     = {
            "L": {"world": None, "from": None, "plant": None, "region": None},
            "R": {"world": None, "from": None, "plant": None, "region": None},
        }

        # Crouch / sit (inferred from pelvis height vs calibrated standing height;
        # no leg trackers). The controller holds all decision/geometry logic and is
        # unit-tested standalone (crouch_sit.py / tests/test_crouch_sit.py).
        self._crouch_sit = CrouchSitController()
        self._force_sit  = False      # UI override: bypass the sit heuristic
        self._crouch_ankle_max_deg = 25.0  # max ankle dorsiflexion before the heel lifts
        self._calib_head_y = None     # standing HMD head Y, captured at T-pose calib
        self._cs_last    = None       # last controller output (for the live label)
        self._cs_lbl     = None       # omni.ui label (set in _build_ui)

        # Camera follow (immersion): pin the avatar root under the XR camera
        # each frame, VRChat-style — the camera rig is the authority, the body
        # follows. Yaw lerps toward the HMD heading so head glances don't
        # twitch the whole body; XZ only (feet stay on the floor).
        self._follow_on        = True
        self._follow_yaw_rate  = 5.0   # exponential yaw realignment rate (1/s)
        self._follow_smooth_on = True
        self._filt_root        = Vec3OneEuro(1.0, 0.02)
        self._root_yaw         = None  # smoothed root yaw (radians), None = unset
        self._follow_tick      = 0
        self._calib_head_yaw   = None  # raw head yaw captured at T-pose calib
        self._body_yaw         = None  # live raw body yaw for the arm IK

        # First-person view: camera sits at the avatar's EYE point. The head
        # mesh is hidden while tracking (head chop) so it can't block the view.
        self._eye_up   = 0.10   # metres above the head-joint origin
        self._eye_fwd  = 0.15   # metres toward the face (avatar forward = -Z);
                                # past the face so the head mesh stays behind the
                                # near plane and doesn't clip the view
        self._hide_head_on = True
        # First-person head hide mode: False = non-deforming region hide (default),
        # True = legacy joint-scale 'head chop' fallback.
        self._head_chop_fallback = False

        # Palm reference point: wrist extended toward the fingers by this much.
        # Still used to place the palm PHYSICS collider (the in-extension grab UI
        # was removed as redundant; grabbing is handled elsewhere).
        self._grab_reach     = 0.09   # metres wrist→palm along the hand direction
        self._hand_world  = {True: None, False: None}  # current hand world pos
        # Live forearm world rotation from the IK each frame — used as the hand's
        # parent frame so hand orientation tracks the arm (not just the T-pose).
        self._fore_world_live = {True: None, False: None}
        # Live forearm pointing direction + local rotation from the IK each frame.
        # Used to drive the forearm ROLL from the controller (swing-twist), so
        # wrist twist follows instead of piling onto the hand joint + clamp.
        self._fore_dir_live   = {True: None, False: None}
        self._fore_local_live = {True: None, False: None}
        # Fraction of controller wrist-twist routed onto the forearm bone (the
        # rest stays on the hand). ~0.65 mimics radius/ulna; 0 disables.
        self._fore_roll = 0.85   # route 85% of wrist twist onto the forearm
                                 # (natural pronation) so it doesn't pile onto the
                                 # hand joint and hit the wrist clamp (was 0.65 →
                                 # ~33° residual hand twist; 0.85 → ~14°)
        # Previous elbow swivel direction per arm (unit bend vector, rest frame).
        # Used by the self-collision solver for temporal continuity/smoothing so
        # the elbow doesn't pop when the clearing side flips or contact toggles.
        self._elbow_bend = {True: None, False: None}
        # Live world elbow position per arm (collision-test position; for debug viz).
        self._elbow_world = {True: None, False: None}
        # Live world simulated hand position per arm (Input Simulation IK target).
        self._sim_hand_world = {True: None, False: None}
        # Raw simulated CONTROLLER world position per arm (the IK input, before
        # scaling/limits/pushes) — recorded by the debug-capture loop.
        self._sim_ctrl_world = {True: None, False: None}
        # Unclamped 1:1-mapped target (world), captured during a replay-metrics
        # capture so follow error is measured against where the controller maps,
        # not the mis-framed raw controller position.
        self._map_ideal_world = {True: None, False: None}
        # Per-arm hand-orientation diagnostic, filled during a replay capture so we
        # can tune _fore_roll (twist routed to the forearm) and _lim_wrist (clamp).
        self._hand_orient_diag = {True: None, False: None}
        self._debug_capturing = False
        # Synthetic wrist twist (rad, roll about the forearm axis) driven in sim so
        # the forearm-roll path is testable without a headset; the capture sweeps
        # it and reads back how it was distributed (forearm vs hand).
        self._sim_wrist_twist = 0.0
        self._sim_twist_diag = {True: None, False: None}
        # Debug: draw the self-collision volume (torso capsule + limb test points)
        # as marker prims so the test geometry can be verified against the mesh.
        self._coll_debug_on = False  # collider proxies hidden by default

        # --- Input Simulation (for testing without headset) ---
        self._sim_enabled = False
        self._sim_pose = "tpose"  # current simulated pose name
        self._sim_time = 0.0

        self._build_ui()

        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="avatar_xr_control.stage"
        )
        self._init_task = asyncio.ensure_future(self._deferred_init())

    def on_shutdown(self):
        if getattr(self, "_init_task", None) is not None:
            self._init_task.cancel()
            self._init_task = None
        self._stop_tracking()
        self._phys_stop_sim()
        self._stage_sub = None
        self._skel      = None
        # getattr-guarded: if on_startup failed partway, _window may not exist.
        window = getattr(self, "_window", None)
        if window:
            window.destroy()
            self._window = None

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            self._skel = None
            stage = omni.usd.get_context().get_stage()
            if stage:
                self._try_init(stage)

    async def _deferred_init(self):
        """Wait for the skeleton prim to appear, then initialise. Fast polling
        covers the common case (stage already open); a slow indefinite retry
        follows so a heavy/Nucleus-streamed stage that takes longer to compose
        its referenced assets doesn't leave the extension permanently stuck on
        a red timeout — it self-heals whenever the prim shows up, including
        after a later stage reopen (self._skel is reset to None then)."""
        for _ in range(300):                      # ~5s at 16ms
            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self._skel_path).IsValid():
                self._try_init(stage)
                return
            await asyncio.sleep(0.016)
        self._set_status(
            f"Waiting for skeleton at {self._skel_path} ...", error=False)
        while True:
            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self._skel_path).IsValid():
                self._try_init(stage)
                return
            await asyncio.sleep(2.0)

    def _try_init(self, stage):
        if self._skel is not None:
            return
        try:
            self._sanitize_camera_ops(stage)
        except Exception:
            pass  # repair is best-effort, must never block init
        try:
            self._skel = _AvatarSkel(stage, self._skel_path)
            # Mesh auto-fit is opt-in: it proved unreliable on this asset's
            # skinned-mesh frame (picked a wrong axis → oversized box). Default to
            # the validated manual dims (0.16/0.13); call it only when asked.
            if self._torso_autofit:
                self._fit_torso_from_mesh(stage)
            self._rebuild_torso()   # build the collision volume from tunables
            # Calibration: reuse a saved one (calibrate once), else fall back to
            # sensible defaults so the avatar tracks with NO calibration at all.
            if not self._load_calibration():
                self._apply_default_calibration()
            self._set_status("Ready", error=False)
            self._dump_rest_world()
        except Exception as e:
            self._set_status(str(e), error=True)

    # Skeleton-joint LEAF names whose skinned vertices make up the torso column
    # (chest + belly + pelvis). Clavicle/neck/arms/legs are excluded so the fit
    # gives the CHEST width, not the shoulder breadth.
    _TORSO_JOINT_LEAVES = {"Hip", "Waist", "Spine01", "Spine02", "Spine"}

    def _fit_torso_from_mesh(self, stage):
        """Auto-size the analytic torso volume (width/depth/forward shift) from the
        avatar's actual skinned mesh, so it matches the body without manual tuning.
        Selects torso vertices by skin weight (dominant influence on a spine/waist/
        hip joint — cleanly excludes the arms even in the T-pose rest), then fits
        the lateral/depth extents around the chest centre. Returns True on success;
        on any problem it leaves the existing (manual/default) values untouched."""
        sk = self._skel
        if sk is None:
            return False
        try:
            skel = UsdSkel.Skeleton(stage.GetPrimAtPath(self._skel_path))
            skel_joints = list(skel.GetJointsAttr().Get() or [])

            def torso_index_set(joint_tokens):
                return {i for i, tok in enumerate(joint_tokens)
                        if str(tok).split("/")[-1] in self._TORSO_JOINT_LEAVES}

            mid_x = float(sk.shoulder_mid[0])
            mid_y = float(sk.shoulder_mid[1])

            def frame_ok(wpts):
                """Accept a candidate transform only if the torso cloud lands on
                the body (centred under the shoulders, spanning chest→pelvis, and
                taller than it is wide) — rejects wrong/rotated (Z-up) frames."""
                n = len(wpts)
                cx = sum(p[0] for p in wpts) / n
                cy = sum(p[1] for p in wpts) / n
                ymin = min(p[1] for p in wpts); ymax = max(p[1] for p in wpts)
                hw = max(abs(p[0] - cx) for p in wpts)
                if abs(cx - mid_x) > 0.30:
                    return False
                if not (mid_y - 0.70 < cy < mid_y + 0.20):
                    return False
                h = ymax - ymin
                return 0.15 < h < 0.90 and h > hw

            xfc = UsdGeom.XformCache(Usd.TimeCode.Default())
            root = stage.GetPrimAtPath(self._avatar_root_path())
            xs, zs = [], []
            for prim in Usd.PrimRange(root):
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                binding = UsdSkel.BindingAPI(prim)
                ip = binding.GetJointIndicesPrimvar()
                wp = binding.GetJointWeightsPrimvar()
                if not ip or not wp or not ip.HasAuthoredValue():
                    continue
                es = ip.GetElementSize()
                idx = ip.ComputeFlattened()
                wts = wp.ComputeFlattened()
                pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                if not pts or not idx or es <= 0:
                    continue
                ja = binding.GetJointsAttr()
                mesh_joints = (ja.Get() if ja and ja.HasAuthoredValue()
                               else skel_joints)
                tset = torso_index_set(mesh_joints)
                if not tset:
                    continue
                local = []
                for v in range(len(pts)):
                    s = 0.0
                    base = v * es
                    for k in range(es):
                        if idx[base + k] in tset:
                            s += wts[base + k]
                    if s > 0.5:
                        local.append(Gf.Vec3d(pts[v][0], pts[v][1], pts[v][2]))
                if len(local) < 20:
                    continue
                # Skinned-mesh points need the geom-bind transform, not the prim
                # transform — and the asset's local frame may be Z-up. Try the
                # plausible frames and keep the first that lands on the body.
                M = xfc.GetLocalToWorldTransform(prim)
                gbt_attr = prim.GetAttribute("primvars:skel:geomBindTransform")
                gbt = gbt_attr.Get() if gbt_attr and gbt_attr.HasAuthoredValue() else None
                cands = []
                if gbt is not None:
                    cands += [gbt, gbt * M, M * gbt]
                cands.append(M)
                for T in cands:
                    wpts = [T.Transform(p) for p in local]
                    if frame_ok(wpts):
                        for w in wpts:
                            xs.append(float(w[0])); zs.append(float(w[2]))
                        break

            if len(xs) < 50:
                print(f"[avatar_xr_control] torso auto-fit: no mesh frame landed on "
                      f"the body ({len(xs)} usable verts) — keeping current torso size")
                return False

            cz = sum(zs) / len(zs)

            def pct(vals, p):
                s = sorted(vals)
                return s[min(len(s) - 1, int(p * len(s)))]

            # Half-width around the body centreline; half-depth + forward shift
            # around the chest centroid. 95th percentile ignores seam/cloth outliers.
            mid_x = float(sk.shoulder_mid[0])
            half_x = pct([abs(x - mid_x) for x in xs], 0.95)
            half_z = pct([abs(z - cz)    for z in zs], 0.95)
            self._torso_half_x = max(0.08, min(0.30, half_x))
            self._torso_half_z = max(0.07, min(0.30, half_z))
            self._torso_fwd    = max(0.0, min(0.20, float(sk.shoulder_mid[2]) - cz))
            print(f"[avatar_xr_control] torso auto-fit from mesh "
                  f"({len(xs)} verts): half_x={self._torso_half_x:.3f} "
                  f"half_z={self._torso_half_z:.3f} fwd={self._torso_fwd:.3f}")
            return True
        except Exception as e:
            print(f"[avatar_xr_control] torso auto-fit failed: {e}")
            return False

    def _rebuild_torso(self):
        """(Re)build the torso self-collision capsule from the live-tunable
        placement/size params and the skeleton's cached base anchors. Centre is
        shifted forward (onto the chest) along the avatar forward direction."""
        sk = self._skel
        if sk is None:
            return
        c = _add(sk.shoulder_mid, _scale(sk.fwd_dir, self._torso_fwd))
        sk.torso_top    = Gf.Vec3f(c[0], sk.shoulder_mid[1], c[2])
        sk.torso_bottom = Gf.Vec3f(c[0], sk.waist_y - sk.torso_height, c[2])
        sk.torso_half_x = self._torso_half_x
        sk.torso_half_z = self._torso_half_z

    # ------------------------------------------------------------------
    # PhysX scene-query collision layer (supplements the analytic torso)
    # ------------------------------------------------------------------
    # Body-collider PROXY: a small set of explicitly-positioned colliders
    # (static UsdPhysics.CollisionAPI shapes) that follow the avatar each frame
    # via _rest_to_world — the same mapping the IK uses. They cannot live on the
    # skinned bone prims (UsdSkel deformation never moves those prims, so PhysX
    # would see them frozen at rest); instead we drive these dedicated prims from
    # the cached rest positions + the live solved arm poses. PhysX scene queries
    # (overlap/sweep) then test candidate arm poses against the REAL geometry,
    # catching the head, hips, opposite arm and scene objects the single analytic
    # torso ellipse can't represent. We only need queries, never simulation.

    _PHYS_PROXY_ROOT = "/World/_xr_body_colliders"
    _PHYS_SCENE_PATH = "/World/physicsScene"

    def _ensure_phys_scene(self, stage):
        """Create a UsdPhysics.Scene (queries only — gravity off) if absent."""
        if stage.GetPrimAtPath(self._PHYS_SCENE_PATH).IsValid():
            return True
        try:
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path(self._PHYS_SCENE_PATH))
            scene.CreateGravityMagnitudeAttr().Set(0.0)
            return True
        except Exception as e:
            print(f"[avatar_xr_control] Failed to create physics scene: {e}")
            return False

    def _build_phys_proxies(self, stage):
        """Create the body-collider proxy prims once (idempotent). Each is a
        static collider (CollisionAPI, no rigid body) sized to the body part;
        only its translate/orient is rewritten each frame in _update_phys_proxies.
        Capsule total length ≈ height + 2·radius, so height = L − 2·radius."""
        sk = self._skel
        if sk is None:
            return False
        root = self._PHYS_PROXY_ROOT
        if not stage.GetPrimAtPath(root).IsValid():
            scope = UsdGeom.Scope.Define(stage, root)
            # Hidden by default; the collision-debug toggle reveals the whole
            # scope at once (per-prim invisibility would prune the toggle).
            UsdGeom.Imageable(scope.GetPrim()).MakeInvisible()

        def _proxy(prim):
            UsdPhysics.CollisionAPI.Apply(prim)
            # Kinematic rigid body: PhysX tracks our per-frame transform writes
            # and refreshes the scene-query structure on each sim step, which a
            # plain static collider does not do without simulation. Gravity is 0
            # on the scene, so nothing falls; we never flush results back to USD,
            # so the body is driven purely by the transforms we author.
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateRigidBodyEnabledAttr().Set(True)
            rb.CreateKinematicEnabledAttr().Set(True)
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp()

        def cap(name, radius, length):
            path = root + "/" + name
            if stage.GetPrimAtPath(path).IsValid():
                return
            g = UsdGeom.Capsule.Define(stage, path)
            g.CreateAxisAttr().Set("Y")
            g.CreateRadiusAttr().Set(float(radius))
            g.CreateHeightAttr().Set(float(max(0.01, length - 2.0 * radius)))
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.25, 0.25)])
            g.CreateDisplayOpacityAttr().Set([0.30])
            _proxy(g.GetPrim())
            UsdGeom.Xformable(g.GetPrim()).AddOrientOp()

        def sph(name, radius):
            path = root + "/" + name
            if stage.GetPrimAtPath(path).IsValid():
                return
            g = UsdGeom.Sphere.Define(stage, path)
            g.CreateRadiusAttr().Set(float(radius))
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.25, 0.25)])
            g.CreateDisplayOpacityAttr().Set([0.30])
            _proxy(g.GetPrim())

        def ellip_cyl(name, sides=16):
            # Elliptic-cylinder torso proxy as a convex-hull mesh. A far better
            # torso fit than a box: rounded sides the upper arm slides along
            # instead of catching a hard corner, and the cross-section matches the
            # analytic torso ellipse (half-axes hx/hz). Unit shape — radius 0.5 in
            # X/Z, height 1 in Y — so the SAME per-frame scale op (2·hx, L, 2·hz)
            # makes it an ellipse of half-axes hx, hz and height L, exactly like
            # the old unit cube. PhysX needs convexHull for a kinematic mesh body.
            path = root + "/" + name
            if stage.GetPrimAtPath(path).IsValid():
                return
            g = UsdGeom.Mesh.Define(stage, path)
            pts = []
            for y in (-0.5, 0.5):
                for i in range(sides):
                    a = 2.0 * math.pi * i / sides
                    pts.append(Gf.Vec3f(0.5 * math.cos(a), y, 0.5 * math.sin(a)))
            counts, idx = [], []
            for i in range(sides):              # side quads
                j = (i + 1) % sides
                counts.append(4)
                idx += [i, j, j + sides, i + sides]
            counts.append(sides)                # bottom cap
            idx += list(range(sides - 1, -1, -1))
            counts.append(sides)                # top cap
            idx += list(range(sides, 2 * sides))
            g.CreatePointsAttr().Set(pts)
            g.CreateFaceVertexCountsAttr().Set(counts)
            g.CreateFaceVertexIndicesAttr().Set(idx)
            g.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.25, 0.25)])
            g.CreateDisplayOpacityAttr().Set([0.30])
            _proxy(g.GetPrim())
            mc = UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim())
            mc.CreateApproximationAttr().Set("convexHull")
            xf = UsdGeom.Xformable(g.GetPrim())
            xf.AddOrientOp()
            xf.AddScaleOp()

        ellip_cyl("torso")
        sph("head",   0.10)
        cap("r_upper", self._UPPER_R, sk.r_upperarm_len)
        cap("r_fore",  self._FORE_R, sk.r_forearm_len)
        cap("l_upper", self._UPPER_R, sk.l_upperarm_len)
        cap("l_fore",  self._FORE_R, sk.l_forearm_len)
        # Palm colliders — kinematic spheres at each hand so the hands push objects
        # (the forearm capsule stops at the wrist). Placed each frame below.
        sph("r_palm", self._PALM_R)
        sph("l_palm", self._PALM_R)
        return True

    def _set_proxy_xform(self, stage, name, world_pos, quat, scale=None):
        """Position one proxy prim at world_pos (and orient/scale it if given)."""
        prim = stage.GetPrimAtPath(self._PHYS_PROXY_ROOT + "/" + name)
        if not prim.IsValid():
            return
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(float(world_pos[0]), float(world_pos[1]),
                                float(world_pos[2])))
            elif t == UsdGeom.XformOp.TypeOrient and quat is not None:
                op.Set(Gf.Quatf(quat))
            elif t == UsdGeom.XformOp.TypeScale and scale is not None:
                op.Set(Gf.Vec3f(float(scale[0]), float(scale[1]),
                                float(scale[2])))

    def _update_phys_proxies(self, stage):
        """Reposition every body proxy from the current rest data + last solved
        arm poses, mapped to live world. Static-collider transform writes are
        picked up by the PhysX scene query (no simulation step needed)."""
        sk = self._skel
        if sk is None:
            return

        # Show the proxy shapes only while the collision-debug viz is on, so the
        # red body capsules/spheres can be checked against the rendered mesh.
        root_prim = stage.GetPrimAtPath(self._PHYS_PROXY_ROOT)
        if root_prim.IsValid():
            img = UsdGeom.Imageable(root_prim)
            if self._coll_debug_on:
                img.MakeVisible()
            else:
                img.MakeInvisible()

        def place_cap(name, a_rest, b_rest):
            aw = self._rest_to_world(a_rest)
            bw = self._rest_to_world(b_rest)
            mid = _scale(_add(aw, bw), 0.5)
            d = _sub(bw, aw)
            L = _len(d)
            axis = _scale(d, 1.0 / L) if L > 1e-5 else Gf.Vec3f(0, 1, 0)
            q = _quat_from_to(Gf.Vec3f(0, 1, 0), axis)   # local +Y → segment dir
            self._set_proxy_xform(stage, name, mid, q)

        def place_sph(name, p_rest):
            self._set_proxy_xform(stage, name, self._rest_to_world(p_rest), None)

        def place_cap_w(name, aw, bw):
            # World-space capsule placement (endpoints already in live world),
            # used for the arm proxies driven by the live skeleton joints.
            mid = _scale(_add(aw, bw), 0.5)
            d = _sub(bw, aw)
            L = _len(d)
            axis = _scale(d, 1.0 / L) if L > 1e-5 else Gf.Vec3f(0, 1, 0)
            q = _quat_from_to(Gf.Vec3f(0, 1, 0), axis)
            self._set_proxy_xform(stage, name, mid, q)

        def place_sph_w(name, pw):
            self._set_proxy_xform(stage, name, pw, None)

        # Live posed joint world positions from the UsdSkel query (render-faithful:
        # the mesh is skinned to these). The joint PRIM xforms stay at bind pose, so
        # this — not ComputeLocalToWorldTransform on the bone prims — is what makes
        # the colliders track the actual pose (arms AND crouch) not just the root.
        jw = sk.joint_world_positions()

        # Crouch / pelvis-drop follow: the torso anchors are cached in the REST
        # frame, which _rest_to_world only moves by the root delta — so a crouch
        # (pelvis drop) wouldn't lower them. Shift the top anchor by the live
        # displacement of the shoulders and the bottom anchor by the waist's, so
        # the torso (and head) lower/lean with the body.
        dtop = dbot = Gf.Vec3f(0, 0, 0)
        if jw is not None:
            live_sh = _scale(_add(jw[sk.r_upper_idx], jw[sk.l_upper_idx]), 0.5)
            dtop_full = _sub(live_sh, self._rest_to_world(sk.shoulder_mid))
            dbot_full = _sub(jw[sk.waist_idx], self._rest_to_world(sk.waist_rest_pos))
            # Follow the crouch VERTICALLY only — the spine's forward tilt would
            # otherwise drag the box off the body centre toward the front.
            dtop = Gf.Vec3f(0.0, float(dtop_full[1]), 0.0)
            dbot = Gf.Vec3f(0.0, float(dbot_full[1]), 0.0)

        # Torso (elliptic cylinder): centre between the torso anchors, local +Y
        # along the (vertical) torso axis; live half-axes from the Width/Depth data.
        tb = _add(self._rest_to_world(sk.torso_bottom), dbot)
        tt = _add(self._rest_to_world(sk.torso_top), dtop)
        mid = _scale(_add(tb, tt), 0.5)
        d = _sub(tt, tb)
        L = _len(d)
        axis = _scale(d, 1.0 / L) if L > 1e-5 else Gf.Vec3f(0, 1, 0)
        q = _quat_from_to(Gf.Vec3f(0, 1, 0), axis)
        # The torso must stay NARROWER than the shoulder joints, otherwise the
        # shoulder (the fixed upper-arm root) sits inside it and the upper arm is
        # forced to intersect — a hanging/straight arm has no swivel freedom to
        # escape. Cap the half-width to just inside the shoulder span and inset the
        # depth a little so it represents the solid torso, not its bounding box.
        sh_off = sk.shoulder_span * 0.5
        box_hx = min(sk.torso_half_x, max(0.05, sh_off - 0.04))
        box_hz = max(0.05, sk.torso_half_z - 0.02)
        self._set_proxy_xform(stage, "torso", mid, q,
                              scale=(2.0 * box_hx, L, 2.0 * box_hz))
        # Head: place at its posed joint so it follows the crouch; fall back to the
        # cached-rest position if the skel query is unavailable.
        if jw is not None:
            place_sph_w("head", jw[sk.head_idx])
        else:
            place_sph("head", sk.head_pos)
        for is_right in (True, False):
            pre = "r" if is_right else "l"
            up_idx  = sk.r_upper_idx if is_right else sk.l_upper_idx
            fo_idx  = sk.r_fore_idx  if is_right else sk.l_fore_idx
            hd_idx  = sk.r_hand_idx  if is_right else sk.l_hand_idx
            mid_idx = sk.finger_idx.get("r_mid1" if is_right else "l_mid1")
            if jw is not None and None not in (up_idx, fo_idx, hd_idx):
                sh_w, el_w, wr_w = jw[up_idx], jw[fo_idx], jw[hd_idx]
                place_cap_w(pre + "_upper", sh_w, el_w)
                place_cap_w(pre + "_fore", el_w, wr_w)
                # Palm = wrist↔mid-finger-base midpoint (covers the palm); fall back
                # to extending along the forearm if that finger joint is missing.
                if mid_idx is not None:
                    palm_w = _scale(_add(wr_w, jw[mid_idx]), 0.5)
                else:
                    fdir = _sub(wr_w, el_w)
                    fl = _len(fdir)
                    palm_w = (_add(wr_w, _scale(fdir, self._grab_reach / fl))
                              if fl > 1e-5 else wr_w)
                place_sph_w(pre + "_palm", palm_w)
                continue

            # Fallback (skel query unavailable): cached-rest reconstruction.
            solved = self._arm_solved.get(is_right)
            if solved is None:
                sh = sk.r_shoulder_pos if is_right else sk.l_shoulder_pos
                el = sk.r_elbow_pos if is_right else sk.l_elbow_pos
                wr = sk.r_wrist_pos if is_right else sk.l_wrist_pos
            else:
                sh, el, wr = solved
            place_cap(pre + "_upper", sh, el)
            place_cap(pre + "_fore", el, wr)
            fdir = _sub(wr, el)
            fl = _len(fdir)
            palm = (_add(wr, _scale(fdir, self._grab_reach / fl))
                    if fl > 1e-5 else wr)
            place_sph(pre + "_palm", palm)

    def _phys_setup(self, stage):
        """Ensure the physics scene + proxies exist; set _phys_scene_ready.
        Runs a single test overlap so the console shows whether scene queries are
        actually returning hits in this runtime (the main integration risk)."""
        if not _PHYS_AVAILABLE or stage is None or self._skel is None:
            self._phys_scene_ready = False
            return False
        try:
            if not self._ensure_phys_scene(stage):
                return False
            self._build_phys_proxies(stage)
            # Attach + step a gravity-free simulation so the kinematic proxies are
            # cooked into the PhysX scene and the scene-query structure is live.
            self._phys_start_sim()
            self._update_phys_proxies(stage)
            self._phys_step(1.0 / 60.0)
            self._phys_scene_ready = True
            # Diagnostic: overlap a sphere at the torso centre; it should hit the
            # torso proxy now that the sim is attached.
            n = self._phys_overlap_count(self._rest_to_world(self._skel.shoulder_mid),
                                         0.05)
            print(f"[avatar_xr_control] PhysX scene-query test: {n} hit(s) at torso "
                  f"centre (expect ≥1 now that the query sim is attached)")
            return True
        except Exception as e:
            import traceback
            print(f"[avatar_xr_control] PhysX setup failed: {e}\n"
                  f"{traceback.format_exc()}")
            self._phys_scene_ready = False
            return False

    def _phys_start_sim(self):
        """Attach a gravity-free PhysX simulation so scene queries are live and
        kinematic-proxy transform writes are tracked. Idempotent."""
        if not _PHYS_AVAILABLE or self._phys_sim_running:
            return
        try:
            get_physx_interface().start_simulation()
            self._phys_sim_running = True
            self._phys_time = 0.0
        except Exception as e:
            print(f"[avatar_xr_control] PhysX start_simulation failed: {e}")

    def _phys_step(self, dt):
        """Advance the query simulation. We deliberately do NOT flush results back
        to USD (no update_transformations), so nothing visible moves — only the
        internal PhysX state the scene queries read is refreshed."""
        if not (_PHYS_AVAILABLE and self._phys_sim_running):
            return
        try:
            get_physx_interface().update_simulation(float(dt), float(self._phys_time))
            self._phys_time += float(dt)
        except Exception:
            pass  # a failed step must never break tracking

    def _phys_stop_sim(self):
        """Detach the query simulation (on disable / shutdown)."""
        if not (_PHYS_AVAILABLE and self._phys_sim_running):
            return
        try:
            get_physx_interface().reset_simulation()
        except Exception:
            pass
        self._phys_sim_running = False

    def _phys_overlap_count(self, world_pos, radius):
        """Raw overlap hit count at a world point (diagnostic)."""
        if not _PHYS_AVAILABLE:
            return 0
        n = [0]
        def report(hit):
            n[0] += 1
            return True
        try:
            get_physx_scene_query_interface().overlap_sphere(
                float(radius),
                carb.Float3(float(world_pos[0]), float(world_pos[1]),
                            float(world_pos[2])),
                report, False)
        except Exception:
            return 0
        return n[0]

    def _apply_pelvis_drop(self, drop, head_drop=0.0):
        """Lower the Hip by `drop` metres (world -Y) and raise the Waist to keep the
        torso/head fixed by `drop - head_drop`. With head_drop=0 (gait reach) the
        head stays put; with head_drop=drop (crouch/sit) the head LOWERS with the
        pelvis to follow the HMD. Joint translations are PARENT-local and may be in
        non-metric units, so convert via the cached rotations and units scale."""
        sk = self._skel
        if sk is None or sk.hip_idx is None:
            return
        s = drop * sk.leg_local_per_m
        w = (drop - head_drop) * sk.leg_local_per_m          # waist compensation
        # world -Y in Hip's parent (root) local frame
        hip_dir = _quat_rotate(_quat_conj(sk._rlroot_world_q), Gf.Vec3f(0.0, -1.0, 0.0))
        sk.write_joint_translation(
            sk.hip_idx, _add(sk.hip_rest_translate, _scale(hip_dir, s)))
        # world +Y in Waist's parent (Hip) local frame
        waist_dir = _quat_rotate(_quat_conj(sk.hip_world_q_rest), Gf.Vec3f(0.0, 1.0, 0.0))
        sk.write_joint_translation(
            sk.waist_idx, _add(sk.waist_rest_translate, _scale(waist_dir, w)))

    def _phys_overlap_body(self, world_pos, radius, allowed):
        """True if the probe sphere overlaps any proxy whose prim path is in
        `allowed` (a set of full proxy paths). Scoped this way so the physics
        self-refine only reacts to the body parts the analytic ellipse can't
        model (head + opposite arm) — the torso/pelvis stay analytic."""
        if not (_PHYS_AVAILABLE and self._phys_scene_ready):
            return False
        hit_found = [False]
        def report(hit):
            if str(hit.collision) in allowed:
                hit_found[0] = True
                return False                  # relevant hit found — stop
            return True                       # keep scanning (torso / self / env)
        try:
            get_physx_scene_query_interface().overlap_sphere(
                float(radius),
                carb.Float3(float(world_pos[0]), float(world_pos[1]),
                            float(world_pos[2])),
                report, False)
        except Exception:
            return False
        return hit_found[0]

    def _phys_seg_clear_body(self, p_rest, q_rest, radius, allowed,
                             t0=0.0, n=4):
        """True if the segment p→q (rest frame, sampled t0..1) is clear of the
        `allowed` proxies. Points are mapped to live world before each probe."""
        d = _sub(q_rest, p_rest)
        for i in range(n + 1):
            f = t0 + (1.0 - t0) * (i / n)
            pt = self._rest_to_world(_add(p_rest, _scale(d, f)))
            if self._phys_overlap_body(pt, radius, allowed):
                return False
        return True

    def _phys_seg_pen_count(self, p_rest, q_rest, radius, allowed,
                            t0=0.0, n=4):
        """How many sampled points of the segment p→q penetrate the `allowed`
        proxies (0 = clear). A coarse penetration measure (overlaps are boolean,
        no depth) used to drive the elbow toward the LEAST-penetrating swivel
        when no angle fully clears."""
        count = 0
        d = _sub(q_rest, p_rest)
        for i in range(n + 1):
            f = t0 + (1.0 - t0) * (i / n)
            pt = self._rest_to_world(_add(p_rest, _scale(d, f)))
            if self._phys_overlap_body(pt, radius, allowed):
                count += 1
        return count

    def _phys_refine_elbow(self, shoulder, wrist, base, h, u, v, is_right, elbow):
        """Secondary self-collision pass against the REAL body proxies: the torso
        BOX, the head, and the OPPOSITE arm. Swivels the elbow around the fixed
        shoulder→wrist axis to the nearest angle that clears them (wrist fixed ⇒
        hand + bone lengths preserved), then eases toward it. The hand-push has
        already moved the wrist OUT of the torso box before IK, so a clearing
        angle reliably exists (this is what makes the torso swivel stable — the
        earlier instability came from the fat circular capsule + hand-inside
        cases). When nothing is penetrated the analytic elbow is returned
        untouched, so physics never perturbs a clear pose."""
        if not (_PHYS_AVAILABLE and self._phys_scene_ready) or h < 0.03:
            self._phys_phi[is_right] = None
            return elbow
        root = self._PHYS_PROXY_ROOT
        opp = "l" if is_right else "r"      # the OTHER arm
        allowed = {root + "/torso",
                   root + "/head",
                   root + "/" + opp + "_upper",
                   root + "/" + opp + "_fore"}
        up_r = self._UPPER_R + self._phys_probe_r
        fo_r = self._FORE_R + self._phys_probe_r

        def elbow_at(phi):
            # The elbow rides a circle of radius h around `base` in the u/v plane.
            bend = _add(_scale(u, math.cos(phi)), _scale(v, math.sin(phi)))
            return _add(base, _scale(bend, h))

        def pen_count(e):
            # Skip the shoulder-anchored half of the upper arm: it is always at
            # the body surface (the shoulder sits on the torso), so testing it
            # would flag an unavoidable contact every frame. The elbow point e is
            # the shared endpoint of both segments, so it is always tested.
            c = self._phys_seg_pen_count(shoulder, e, up_r, allowed, t0=0.5)
            fdir = _sub(wrist, e)
            flen = _len(fdir)
            hand_end = (_add(wrist, _scale(fdir, self._HAND_EXT / flen))
                        if flen > 1e-5 else wrist)
            return c + self._phys_seg_pen_count(e, hand_end, fo_r, allowed)

        # Analytic swivel angle of the incoming elbow, in the current basis.
        bend = _sub(elbow, base)
        phi0 = math.atan2(_dot(bend, v), _dot(bend, u))
        prev = self._phys_phi.get(is_right)

        if pen_count(elbow_at(phi0)) == 0:
            # Natural elbow clears the real body → follow the analytic pole. Ease
            # toward it (don't reset history) so returning from a held pose is
            # smooth, never a snap.
            phi_target = phi0
        elif (prev is not None and pen_count(elbow_at(prev)) == 0
              and elbow_at(prev)[1] <= elbow_at(phi0)[1] + 0.05):
            # HYSTERESIS: last frame's elbow still clears AND isn't above the
            # natural (pole) elbow → KEEP it. This holds a stable cleared pose so
            # the elbow doesn't hunt/flip near the torso/face. The height guard is
            # what lets a STUCK-UP elbow fall back down: if prev sits above natural
            # (a leftover up-reach swivel), we don't hold it — we re-search below.
            phi_target = prev
        else:
            # Search both ways for a clear angle. Prefer the LOWEST elbow (gravity /
            # "elbow points down"), with continuity (nearest the previous swivel) as
            # a tiebreak to damp side-flips when two angles are equally low.
            ref = prev if prev is not None else phi0
            step = self._SWIVEL_STEP
            clear_cands = []
            best_pen = (pen_count(elbow_at(phi0)), elbow_at(phi0)[1], phi0)
            for i in range(1, self._phys_max_steps + 1):
                mag_clears = []
                for sgn in (1.0, -1.0):
                    phi = phi0 + sgn * step * i
                    e = elbow_at(phi)
                    c = pen_count(e)
                    if c == 0:
                        d_cont = abs((phi - ref + math.pi) % (2 * math.pi) - math.pi)
                        mag_clears.append((e[1], d_cont, phi))   # lowest elbow first
                    elif (c, e[1], phi) < best_pen:
                        best_pen = (c, e[1], phi)
                if mag_clears:
                    mag_clears.sort()          # lowest elbow first, then nearest-prev
                    clear_cands = mag_clears
                    break
            phi_target = clear_cands[0][2] if clear_cands else best_pen[2]

        # Ease toward the target so the elbow never pops on a contact flip.
        if prev is None:
            phi = phi_target
        else:
            dphi = (phi_target - prev + math.pi) % (2 * math.pi) - math.pi
            phi = prev + dphi * (1.0 - math.exp(-self._SWIVEL_RATE * self._frame_dt))
        self._phys_phi[is_right] = phi
        return elbow_at(phi)

    def _phys_push_point_out_torso(self, p_rest, radius):
        """Push a point (rest frame) out of the real torso box collider, marching
        horizontally outward from the torso's vertical axis until a probe sphere
        of `radius` clears it. Used both for the hand TARGET (so the hand/forearm
        can't end up inside the body — the case no elbow swivel can fix) and for
        the IK SHOULDER (so the upper-arm root starts outside the torso). Returns
        the (possibly pushed-out) rest-frame point."""
        if not (_PHYS_AVAILABLE and self._phys_scene_ready):
            return p_rest
        sk = self._skel
        allowed = {self._PHYS_PROXY_ROOT + "/torso"}
        pw = self._rest_to_world(p_rest)
        if not self._phys_overlap_body(pw, radius, allowed):
            return p_rest
        # Radial direction from the torso vertical axis (horizontal only).
        tb = self._rest_to_world(sk.torso_bottom)
        tt = self._rest_to_world(sk.torso_top)
        axis = _sub(tt, tb)
        al2 = _dot(axis, axis)
        if al2 < 1e-9:
            return p_rest
        t = max(0.0, min(1.0, _dot(_sub(pw, tb), axis) / al2))
        c = _add(tb, _scale(axis, t))
        radial = Gf.Vec3f(pw[0] - c[0], 0.0, pw[2] - c[2])
        rl = _len(radial)
        radial = (_scale(radial, 1.0 / rl) if rl > 1e-5
                  else self._rest_to_world_dir(sk.fwd_dir))
        # March out until the probe clears the torso, but cap the total travel so
        # a deep penetration can't yank the hand far in one frame (the eased
        # correction at the call site smooths what remains).
        max_push = 0.15
        moved = 0.0
        while moved < max_push:
            pw = _add(pw, _scale(radial, 0.02))
            moved += 0.02
            if not self._phys_overlap_body(pw, radius, allowed):
                break
        return self._world_to_rest(pw)

    def _rest_to_world_dir(self, d):
        """Rotate a rest-frame direction into live world (no translation)."""
        a = self._rest_to_world(Gf.Vec3f(0, 0, 0))
        b = self._rest_to_world(d)
        v = _sub(b, a)
        n = _len(v)
        return _scale(v, 1.0 / n) if n > 1e-6 else Gf.Vec3f(0, 0, -1)

    def _phys_env_clamp(self, target_rest, is_right):
        """Sweep the hand sphere from its previous world position toward the new
        target; if it would cross a SCENE object (not a body proxy or the avatar
        itself), clamp the target back to the contact point so the hand rests on
        the surface instead of clipping through. Resting (near-static) contact is
        not pushed — this only stops motion into an object."""
        if not (_PHYS_AVAILABLE and self._phys_scene_ready):
            return target_rest
        tgt_w = self._rest_to_world(target_rest)
        prev_w = self._hand_world.get(is_right)
        if prev_w is None:
            return target_rest
        d = _sub(tgt_w, prev_w)
        dist = _len(d)
        if dist < 1e-4:
            return target_rest
        unit = _scale(d, 1.0 / dist)
        root = self._PHYS_PROXY_ROOT
        avatar_root = self._avatar_root_path()
        best = {"dist": None}
        def report(hit):
            p = str(hit.collision)
            if p.startswith(root) or p.startswith(avatar_root):
                return True                   # ignore body proxies & self mesh
            if best["dist"] is None or hit.distance < best["dist"]:
                best["dist"] = float(hit.distance)
            return True
        r = self._FORE_R + self._phys_probe_r
        try:
            get_physx_scene_query_interface().sweep_sphere(
                float(r),
                carb.Float3(float(prev_w[0]), float(prev_w[1]), float(prev_w[2])),
                carb.Float3(float(unit[0]), float(unit[1]), float(unit[2])),
                float(dist), report, False)
        except Exception:
            return target_rest
        if best["dist"] is None:
            return target_rest
        clamped_w = _add(prev_w, _scale(unit, max(0.0, best["dist"])))
        return self._world_to_rest(clamped_w)

    _CAMERA_FIX_PATHS = ("/OmniverseKit_Persp", "/OmniverseKit_Top",
                         "/OmniverseKit_Front", "/OmniverseKit_Right")

    def _sanitize_camera_ops(self, stage):
        """Repair viewport cameras whose xformOpOrder lists ops with MISSING
        attributes (seen on /OmniverseKit_Persp: order [translate, orient,
        scale] but no xformOp:scale attribute — a stale camera override saved
        into the stage). Kit's camera manipulator then errors on every zoom:
        the op name is in the order, so AddXformOp refuses to re-create it.
        Fix: author the missing attributes with identity values."""
        _DEFAULTS = {
            "translate": (Sdf.ValueTypeNames.Double3,  Gf.Vec3d(0, 0, 0)),
            "orient":    (Sdf.ValueTypeNames.Quatd,    Gf.Quatd(1, 0, 0, 0)),
            "scale":     (Sdf.ValueTypeNames.Double3,  Gf.Vec3d(1, 1, 1)),
            "rotateXYZ": (Sdf.ValueTypeNames.Double3,  Gf.Vec3d(0, 0, 0)),
            "transform": (Sdf.ValueTypeNames.Matrix4d, Gf.Matrix4d(1.0)),
        }
        for path in self._CAMERA_FIX_PATHS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            order = prim.GetAttribute("xformOpOrder")
            tokens = order.Get() if order else None
            if not tokens:
                continue
            for token in tokens:
                name = str(token)
                attr = prim.GetAttribute(name)
                if attr and attr.IsValid():
                    continue
                parts = name.split(":")   # "xformOp:<type>[:suffix]"
                spec = _DEFAULTS.get(parts[1]) if len(parts) >= 2 else None
                if spec is None:
                    continue
                try:
                    prim.CreateAttribute(name, spec[0]).Set(spec[1])
                    print(f"[avatar_xr_control] Repaired missing {name} on {path}")
                except Exception:
                    pass

    def _dump_rest_world(self):
        """GATE 1 (offline, no headset): write the canonical world-space rest
        positions + arm directions to a file so the user can confirm the rest
        arm now reads sideways (world ±X), matching the visible T-pose."""
        sk = self._skel
        if sk is None or not DEBUG_FILES:
            return
        def fv(v):
            return f"({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f})"
        r_sh, r_el, r_wr = sk.r_shoulder_pos, sk.r_elbow_pos, sk.r_wrist_pos
        l_sh, l_el, l_wr = sk.l_shoulder_pos, sk.l_elbow_pos, sk.l_wrist_pos
        r_dir = _normalize(_sub(r_el, r_sh))
        l_dir = _normalize(_sub(l_el, l_sh))
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self._skel_path)
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            ex = m.TransformDir(Gf.Vec3d(1, 0, 0))
            ey = m.TransformDir(Gf.Vec3d(0, 1, 0))
            ez = m.TransformDir(Gf.Vec3d(0, 0, 1))
            lines = [
                "=== Canonical rest-world dump (GATE 1) ===",
                f"skel prim local +X -> world ({ex[0]:+.3f},{ex[1]:+.3f},{ex[2]:+.3f})",
                f"skel prim local +Y -> world ({ey[0]:+.3f},{ey[1]:+.3f},{ey[2]:+.3f})",
                f"skel prim local +Z -> world ({ez[0]:+.3f},{ez[1]:+.3f},{ez[2]:+.3f})",
                "",
                f"R shoulder = {fv(r_sh)}",
                f"R elbow    = {fv(r_el)}",
                f"R wrist    = {fv(r_wr)}",
                f"R shoulder->elbow dir = {fv(r_dir)}   (expect ~world X for T-pose)",
                "",
                f"L shoulder = {fv(l_sh)}",
                f"L elbow    = {fv(l_el)}",
                f"L wrist    = {fv(l_wr)}",
                f"L shoulder->elbow dir = {fv(l_dir)}   (expect ~world -X for T-pose)",
            ]
            with open(_data_path("_rest_world_debug.txt"),
                      "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        except Exception as e:
            print(f"[avatar_xr_control] rest-world dump failed: {e}")

    def _apply_skel_path(self):
        self._skel_path = self._skel_path_field.model.get_value_as_string().strip()
        self._skel = None
        self._set_status("Reinitialising...", error=False)
        if getattr(self, "_init_task", None) is not None:
            self._init_task.cancel()
            self._init_task = None
        stage = omni.usd.get_context().get_stage()
        if stage:
            self._try_init(stage)
        else:
            self._init_task = asyncio.ensure_future(self._deferred_init())

    def _init_xr(self):
        try:
            from omni.kit.xr.core import XRCore
            xr = XRCore.get_singleton()
            self._head_dev  = xr.get_input_device("/user/head")
            self._left_dev  = xr.get_input_device("/user/hand/left")
            self._right_dev = xr.get_input_device("/user/hand/right")
            self._xr = xr
            ok = any(d is not None for d in (self._head_dev, self._right_dev))
            return ok, "XR devices acquired" if ok else "No XR devices found (is XR session active?)"
        except Exception as e:
            return False, str(e)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self):
        S_LABEL = {"font_size": 11, "color": 0xFFCCCCCC}
        S_WARN  = {"font_size": 11, "color": 0xFFFFAA33}
        BTN     = {"font_size": 11}
        # CollapsableFrame look — styled header (Label::title) + subtle box.
        FRAME = {
            "CollapsableFrame": {"background_color": 0xFF2A2A2A,
                                 "secondary_color": 0xFF3A3A3A,
                                 "border_radius": 4, "padding": 4},
            "CollapsableFrame:hovered": {"secondary_color": 0xFF505050},
            "Label::title": {"font_size": 12, "color": 0xFFDDDDDD},
        }

        self._window = ui.Window("Avatar XR Control", width=360, height=0)
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):

                ui.Spacer(height=2)
                self._status_lbl = ui.Label("waiting for stage...", height=16, style=S_WARN)

                # --- XR SESSION (primary action) -------------------------------
                with ui.CollapsableFrame("XR Session", collapsed=False, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        # Start XR: enable stream -> wait 10s -> start tracking.
                        # Stop XR reverses it. Both disable while a sequence runs.
                        self._startxr_btn = ui.Button(
                            "Start XR", clicked_fn=self._start_xr_combined, height=30,
                            style={"font_size": 13},
                            tooltip="Stream starten, dann nach 10s automatisch Tracking")
                        self._stopxr_btn = ui.Button(
                            "Stop XR", clicked_fn=self._stop_xr_combined, height=28,
                            style={"font_size": 12},
                            tooltip="Tracking stoppen, dann nach 10s Stream beenden")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Reset view to eyes",
                                      clicked_fn=self._parent_camera_to_head, style=BTN,
                                      tooltip="Kamera erneut auf Augenhöhe des Avatars setzen")
                            ui.Button("Hide head on/off",
                                      clicked_fn=self._toggle_hide_head, style=BTN,
                                      tooltip="Kopf ausblenden (First-Person)")
                            ui.Button("Head: region/chop",
                                      clicked_fn=self._toggle_head_mode, style=BTN,
                                      tooltip="Non-deforming region hide vs. legacy joint-scale head-chop")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Eye height -", clicked_fn=self._eye_up_dn, style=BTN)
                            ui.Button("Eye height +", clicked_fn=self._eye_up_up, style=BTN)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Eye fwd -", clicked_fn=self._eye_fwd_dn, style=BTN)
                            ui.Button("Eye fwd +", clicked_fn=self._eye_fwd_up, style=BTN)
                        self._eye_lbl = ui.Label("", height=14, style=S_LABEL)
                        self._update_eye_lbl()
                        self._xr_session_lbl = ui.Label("inactive", height=14, style=S_LABEL)
                        self._track_lbl = ui.Label("", height=14, style=S_LABEL)

                # --- HAND CALIBRATION (primary action) -------------------------
                with ui.CollapsableFrame("Hand Calibration", collapsed=False, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Label("Stand in T-pose, then press:", height=14, style=S_LABEL)
                        ui.Button("Calibrate hands (T-pose)", clicked_fn=self._calibrate_hands,
                                  height=26, style=BTN)
                        self._calib_lbl = ui.Label("not calibrated", height=16, style=S_WARN)
                        ui.Button("Diagnose finger tracking", clicked_fn=self._diagnose_fingers,
                                  height=24, style=BTN,
                                  tooltip="Put controllers DOWN first. Writes _finger_diag.txt")

                # --- LOCOMOTION ------------------------------------------------
                with ui.CollapsableFrame("Locomotion", collapsed=True, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Button("Camera follow on/off",
                                  clicked_fn=self._toggle_follow, style=BTN,
                                  tooltip="Avatar folgt der XR-Kamera (Position + Blickrichtung)")
                        self._follow_lbl = ui.Label("", height=16, style=S_LABEL)
                        self._update_follow_lbl()
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Body turn -", clicked_fn=self._follow_rate_dn, style=BTN,
                                      tooltip="slower body realignment to head yaw")
                            ui.Button("Body turn +", clicked_fn=self._follow_rate_up, style=BTN,
                                      tooltip="faster body realignment to head yaw")
                        ui.Line(style={"color": 0xFF333333})
                        ui.Button("Right stick glide on/off",
                                  clicked_fn=self._toggle_stick_loco, style=BTN,
                                  tooltip="Move the player (camera) with the right controller thumbstick")
                        self._stick_lbl = ui.Label("", height=16, style=S_LABEL)
                        self._update_stick_lbl()
                        ui.Line(style={"color": 0xFF333333})
                        ui.Button("Procedural walk (legs) on/off",
                                  clicked_fn=self._toggle_legs, style=BTN,
                                  tooltip="Synthesise a stepping gait from movement so the "
                                          "legs walk instead of sliding (no leg trackers needed)")
                        self._legs_lbl = ui.Label("", height=16, style=S_LABEL)
                        self._update_legs_lbl()
                        ui.Line(style={"color": 0xFF333333})
                        ui.Label("Crouch / Sit (inferred from headset height):",
                                 height=14, style=S_LABEL)
                        self._cs_lbl = ui.Label("crouch 0.00  sit 0.00", height=16,
                                                style=S_LABEL)
                        ui.Button("Force sit on/off", clicked_fn=self._toggle_force_sit,
                                  style=BTN, tooltip="Bypass the sit heuristic (testing)")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Crouch depth -", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("crouch_full_drop", -0.02))
                            ui.Button("Crouch depth +", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("crouch_full_drop", +0.02))
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Sit height -", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("sit_height_frac", -0.02))
                            ui.Button("Sit height +", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("sit_height_frac", +0.02))
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Sit back -", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("sit_back_frac", -0.02))
                            ui.Button("Sit back +", style=BTN,
                                      clicked_fn=lambda: self._cs_tune("sit_back_frac", +0.02))
                        ui.Line(style={"color": 0xFF333333})
                        ui.Label("Manual step (moves the camera rig):", height=14, style=S_LABEL)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("< Left", clicked_fn=self._move_left, style=BTN)
                            ui.Button("Forward", clicked_fn=self._move_fwd, style=BTN)
                            ui.Button("Right >", clicked_fn=self._move_right, style=BTN)
                        ui.Button("Back", clicked_fn=self._move_back, height=24, style=BTN)

                # --- AVATAR PATH (setup, collapsed) ----------------------------
                with ui.CollapsableFrame("Avatar Path", collapsed=True, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Label("Skeleton USD path", height=14, style=S_LABEL)
                        self._skel_path_field = ui.StringField(height=22, style={"font_size": 10})
                        self._skel_path_field.model.set_value(self._skel_path)
                        ui.Button("Apply path & reinitialise", height=24,
                                  clicked_fn=self._apply_skel_path, style=BTN)

                # Advanced tuning is intentionally NOT exposed in the UI: every
                # parameter (IK reach, elbow pole, torso volume, body/physics
                # collision, wrist/elbow roll, finger curl, joint limits, shoulder
                # follow, smoothing) keeps its tuned default from on_startup so the
                # extension works out of the box. The backend logic and the +/-
                # handler methods remain available for programmatic tweaking.

                # --- INPUT SIMULATION (testing, collapsed) ----------------------
                with ui.CollapsableFrame("Input Simulation", collapsed=True, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Button("Run debug capture (auto)", height=28, style=BTN,
                                  clicked_fn=self._start_debug_capture,
                                  tooltip="One click: enables simulation, steps the avatar through every test pose, and writes a CSV comparing the simulated controller target vs. the resulting avatar joint positions. No manual toggling or visual inspection needed.")
                        self._debug_cap_lbl = ui.Label("", height=16, style=S_LABEL)
                        ui.Line(style={"color": 0xFF333333})
                        ui.Label("Simulate hand movements without headset:", height=14, style=S_LABEL)
                        ui.Button("Start simulation", clicked_fn=self._start_simulation, height=24, style=BTN)
                        ui.Button("Stop simulation", clicked_fn=self._stop_simulation, height=24, style=BTN)
                        self._sim_lbl = ui.Label("simulation: off", height=16, style=S_LABEL)
                        ui.Line(style={"color": 0xFF333333})
                        ui.Label("Test poses:", height=14, style=S_LABEL)
                        with ui.HStack(spacing=4, height=24):
                            ui.Button("T-pose", clicked_fn=lambda: self._set_sim_pose("tpose"), style=BTN)
                            ui.Button("Reach fwd", clicked_fn=lambda: self._set_sim_pose("reach_fwd"), style=BTN)
                            ui.Button("Reach up", clicked_fn=lambda: self._set_sim_pose("reach_up"), style=BTN)
                        with ui.HStack(spacing=4, height=24):
                            ui.Button("Reach left", clicked_fn=lambda: self._set_sim_pose("reach_left"), style=BTN)
                            ui.Button("Reach right", clicked_fn=lambda: self._set_sim_pose("reach_right"), style=BTN)
                            ui.Button("Reach down", clicked_fn=lambda: self._set_sim_pose("reach_down"), style=BTN)
                        with ui.HStack(spacing=4, height=24):
                            ui.Button("Fwd+Up", clicked_fn=lambda: self._set_sim_pose("reach_fwd_up"), style=BTN)
                            ui.Button("Fwd+Down", clicked_fn=lambda: self._set_sim_pose("reach_fwd_down"), style=BTN)
                            ui.Button("Left+Up", clicked_fn=lambda: self._set_sim_pose("reach_left_up"), style=BTN)
                        with ui.HStack(spacing=4, height=24):
                            ui.Button("Right+Up", clicked_fn=lambda: self._set_sim_pose("reach_right_up"), style=BTN)
                            ui.Button("Fwd+Left", clicked_fn=lambda: self._set_sim_pose("reach_fwd_left"), style=BTN)
                            ui.Button("Fwd+Right", clicked_fn=lambda: self._set_sim_pose("reach_fwd_right"), style=BTN)
                        with ui.HStack(spacing=4, height=24):
                            ui.Button("Left+Down", clicked_fn=lambda: self._set_sim_pose("reach_left_down"), style=BTN)
                            ui.Button("Right+Down", clicked_fn=lambda: self._set_sim_pose("reach_right_down"), style=BTN)
                        self._pose_lbl = ui.Label("pose: tpose", height=14, style=S_LABEL)

                # --- XR CAPTURE (record a live demo, replay to fine-tune) -------
                with ui.CollapsableFrame("XR Capture (record / replay)",
                                         collapsed=True, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Label("Record a live XR demo, then replay it (no headset)\n"
                                 "to fine-tune the IK against real motion.",
                                 height=28, style=S_LABEL)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Record live", clicked_fn=self._start_recording,
                                      style=BTN,
                                      tooltip="Start XR first, then record raw head/controller/finger data each frame")
                            ui.Button("Stop", clicked_fn=self._stop_recording, style=BTN)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Replay", clicked_fn=self._start_replay, style=BTN,
                                      tooltip="Play the recording through the avatar; tweak tuning while it loops")
                            ui.Button("Stop", clicked_fn=self._stop_replay, style=BTN)
                            ui.Button("Loop on/off", clicked_fn=self._toggle_play_loop,
                                      style=BTN)
                        ui.Button("Replay + capture metrics", clicked_fn=self._start_replay_capture,
                                  style=BTN,
                                  tooltip="Step the recording through the IK and write follow-error / reach / clip stats for your REAL motion -> _replay_capture.csv")
                        self._capture_lbl = ui.Label("", height=16, style=S_LABEL)
                        self._update_capture_lbl()

                ui.Spacer(height=4)


    # ------------------------------------------------------------------
    # Hand calibration
    # ------------------------------------------------------------------

    @staticmethod
    def _apply_hand_axes(quatd) -> Gf.Quatf:
        """Fixed hand orientation axis remap: negate X, swap Y/Z, negate new Y."""
        im = quatd.GetImaginary()
        return Gf.Quatf(float(quatd.GetReal()),
                        -float(im[0]), -float(im[2]), float(im[1]))

    def _calibrate_hands(self):
        asyncio.ensure_future(self._calibrate_countdown(10))

    def _ik_scale_up(self):
        self._ik_scale_mult = min(3.0, self._ik_scale_mult + 0.1)
        self._set_lbl("_ik_scale_lbl", f"IK reach mult: {self._ik_scale_mult:.2f}")

    def _ik_scale_down(self):
        self._ik_scale_mult = max(0.1, self._ik_scale_mult - 0.1)
        self._set_lbl("_ik_scale_lbl", f"IK reach mult: {self._ik_scale_mult:.2f}")

    # ------------------------------------------------------------------
    # Locomotion — translate the avatar root prim through the stage
    # ------------------------------------------------------------------

    def _avatar_root_path(self):
        # Skeleton path is /Root/<avatar>/ManRoot/...; the movable root is the
        # prim two levels up: /Root/<avatar>.
        parts = self._skel_path.strip("/").split("/")
        if len(parts) >= 2:
            return "/" + "/".join(parts[:2])
        return self._skel_path

    def _move_root(self, dx=0.0, dy=0.0, dz=0.0, quiet=False):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(self._avatar_root_path())
        if not prim.IsValid():
            self._set_track(f"root prim not found: {self._avatar_root_path()}", 0xFF4444FF)
            return
        xform = UsdGeom.Xformable(prim)
        # Find or create a translate op.
        translate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
        cur = translate_op.Get() or Gf.Vec3d(0, 0, 0)
        translate_op.Set(Gf.Vec3d(cur[0] + dx, cur[1] + dy, cur[2] + dz))
        if not quiet:   # per-frame stick locomotion suppresses the label spam
            self._set_track(
                f"root pos=({cur[0]+dx:.2f}, {cur[1]+dy:.2f}, {cur[2]+dz:.2f})",
                0xFF44FF44)

    def _step_origin(self, dx=0.0, dz=0.0):
        """Manual step: move the XR origin (camera rig) camera-relative.
        The avatar follows the camera via _apply_camera_follow."""
        if self._xr is None:
            ok, _ = self._init_xr()
            if not ok:
                self._set_track("no XR session - manual step needs XR", 0xFFFFAA33)
                return
        try:
            self._xr.schedule_move_space_origin_relative_to_camera(dx, 0.0, dz)
            self._set_track(f"origin step ({dx:+.2f}, {dz:+.2f})", 0xFF44FF44)
        except Exception as e:
            self._set_track(f"origin step failed: {e}", 0xFF4444FF)

    # Camera-relative axes: camera looks -Z, right = +X.
    def _move_fwd(self):   self._step_origin(dz=-self._move_step)
    def _move_back(self):  self._step_origin(dz=+self._move_step)
    def _move_left(self):  self._step_origin(dx=-self._move_step)
    def _move_right(self): self._step_origin(dx=+self._move_step)

    def _update_stick_lbl(self):
        state = "ON" if self._stick_loco_on else "OFF"
        self._stick_lbl.text = f"right-stick glide (camera): {state} ({self._stick_speed:.1f} m/s)"

    def _toggle_stick_loco(self):
        self._stick_loco_on = not self._stick_loco_on
        self._update_stick_lbl()

    # --- First-person view controls ---
    def _update_eye_lbl(self):
        head = "hidden" if self._hide_head_on else "visible"
        mode = "chop" if self._head_chop_fallback else "region"
        self._eye_lbl.text = (
            f"eye height +{self._eye_up:.2f}m  fwd {self._eye_fwd:.2f}m  "
            f"head: {head} ({mode})")

    def _toggle_hide_head(self):
        self._hide_head_on = not self._hide_head_on
        if self._skel is not None:
            self._skel.set_head_hidden(self._hide_head_on, self._head_chop_fallback)
        self._update_eye_lbl()

    def _toggle_head_mode(self):
        """Switch between the non-deforming region hide and the head-chop fallback."""
        self._head_chop_fallback = not self._head_chop_fallback
        if self._skel is not None:
            self._skel.set_head_hidden(self._hide_head_on, self._head_chop_fallback)
        self._update_eye_lbl()

    def _eye_up_up(self):
        self._eye_up = min(0.40, self._eye_up + 0.02)
        self._update_eye_lbl()
        self._parent_camera_to_head()   # re-teleport so the change is felt now

    def _eye_up_dn(self):
        self._eye_up = max(-0.20, self._eye_up - 0.02)
        self._update_eye_lbl()
        self._parent_camera_to_head()

    def _eye_fwd_up(self):
        # Move the camera further forward (out the front of the face) to clear the
        # head mesh from the near plane.
        self._eye_fwd = min(0.40, self._eye_fwd + 0.02)
        self._update_eye_lbl()
        self._parent_camera_to_head()   # re-teleport so the change is felt now

    def _eye_fwd_dn(self):
        self._eye_fwd = max(-0.10, self._eye_fwd - 0.02)
        self._update_eye_lbl()
        self._parent_camera_to_head()

    # --- Camera follow controls ---
    def _update_follow_lbl(self):
        state = "ON" if self._follow_on else "OFF"
        self._follow_lbl.text = (
            f"camera follow: {state}  (body turn rate {self._follow_yaw_rate:.1f}/s)")

    def _toggle_follow(self):
        self._follow_on = not self._follow_on
        # Re-seed yaw/position smoothing so re-enabling doesn't jump from
        # stale filter state.
        self._root_yaw = None
        self._filt_root.reset()
        self._update_follow_lbl()

    def _follow_rate_up(self):
        self._follow_yaw_rate = min(20.0, self._follow_yaw_rate + 1.0)
        self._update_follow_lbl()

    def _follow_rate_dn(self):
        self._follow_yaw_rate = max(0.5, self._follow_yaw_rate - 1.0)
        self._update_follow_lbl()

    def _update_legs_lbl(self):
        state = "ON" if self._legs_on else "OFF"
        self._legs_lbl.text = f"procedural walk (legs): {state}"

    def _toggle_legs(self):
        self._legs_on = not self._legs_on
        # Reset gait + crouch/sit state so re-enabling doesn't lurch from stale anchors.
        self._gait_root_prev = None
        self._gait_speed = 0.0
        self._crouch_sit.reset()
        for s in self._foot_state.values():
            s["world"] = s["from"] = s["plant"] = s["region"] = None
        # Restore the rest leg pose + pelvis so nothing freezes mid-stride when off.
        if (not self._legs_on and self._skel is not None
                and getattr(self._skel, "legs", None)):
            for leg in self._skel.legs.values():
                for idx, q in leg["rest_local"].items():
                    self._skel.write_joint_rotation(idx, q)
            self._pelvis_drop = 0.0
            self._apply_pelvis_drop(0.0, 0.0)
        self._update_legs_lbl()

    def _toggle_force_sit(self):
        self._force_sit = not self._force_sit

    def _cs_tune(self, key, delta):
        """Live-tune a crouch/sit threshold from the +/- buttons (clamped sane)."""
        lo, hi = {"crouch_full_drop": (0.20, 0.70),
                  "sit_height_frac":  (0.35, 0.75),
                  "sit_back_frac":    (0.05, 0.40)}.get(key, (0.0, 1.0))
        self._crouch_sit.p[key] = max(lo, min(hi, self._crouch_sit.p[key] + delta))

    def _apply_crouch_sit(self, sk, rxz, dt):
        """Infer crouch depth / sitting from the pelvis (HMD) transform and drive
        the legs accordingly: lower the pelvis (HEAD FOLLOWS down), plant the feet
        while crouching / move them forward while sitting, and solve. Returns True
        when it took over the legs this frame (so the gait pass is skipped).

        All decision + geometry math lives in the dependency-free CrouchSitController
        (unit-tested in tests/test_crouch_sit.py); here we only marshal world-space
        Gf values to/from plain tuples and feed the result into the existing solve."""
        ctl = getattr(self, "_crouch_sit", None)
        if ctl is None:
            return False
        hm = _get_pose(self._head_dev)
        if hm is None:
            return False
        head_y  = float(hm.ExtractTranslation()[1])
        calib_y = self._calib_head_y or float(sk.head_rest_world[1])

        fwd  = _rot_y(Gf.Vec3f(0, 0, -1), self._body_yaw_live)
        hipw = self._rest_to_world(sk.hip_rest_pos)
        feet, lens, gy = {}, {}, {}
        for side, leg in sk.legs.items():
            nf = self._rest_to_world(leg["ankle_pos"])
            feet[side] = (float(nf[0]), float(nf[1]), float(nf[2]))
            lens[side] = (leg["thigh_len"], leg["calf_len"])
            gy[side]   = float(nf[1])

        out = ctl.update(
            head_y, calib_y, (float(rxz[0]), float(rxz[2])),
            (float(fwd[0]), float(fwd[1]), float(fwd[2])),
            (float(hipw[0]), float(hipw[1]), float(hipw[2])),
            feet, lens, gy, dt, self._force_sit)
        self._cs_last = out
        if self._cs_lbl is not None:
            g = out["gates"]
            self._cs_lbl.text = (
                f"crouch {out['crouch_factor']:.2f}  sit {out['sit_factor']:.2f}  "
                f"[{'H' if g['height'] else '-'}{'B' if g['back'] else '-'}"
                f"{'S' if g['settled'] else '-'}]  drop {out['hip_drop'] * 100:.0f}cm")

        if not out["active"]:
            return False

        # Pelvis sinks with the head following (head_drop == hip_drop); feet were
        # placed (planted/forward) and reach/knee-clamped by the controller.
        self._pelvis_drop = out["hip_drop"]
        self._apply_pelvis_drop(out["hip_drop"], out["head_drop"])
        for side in sk.legs:
            ft = out["foot_targets"][side]
            foot_w = Gf.Vec3f(float(ft[0]), float(ft[1]), float(ft[2]))
            self._solve_leg_ik(side, foot_w, out["hip_drop"], 0.0, 0.0)
            st = self._foot_state[side]
            st["world"], st["region"], st["plant"], st["from"] = foot_w, None, None, None
        return True

    def _apply_legs(self, dt=None):
        """Procedural walk: synthesise a stepping gait from the avatar root's
        horizontal velocity and solve 2-bone IK per leg so the feet plant on the
        ground instead of sliding with the body. No leg trackers required.

        Anti-skate model: each foot is world-anchored. During its stance half of
        the gait cycle the foot stays locked at the world point where it planted
        (the body glides over it → the leg sweeps backward, as in real walking);
        during the swing half it arcs from that plant to a new plant half a step
        ahead of the hip. When the body is still, both feet ease to the neutral
        stance under the hips and the legs straighten back to rest."""
        sk = self._skel
        if (sk is None or not self._legs_on
                or not getattr(sk, "legs", None) or sk.hip_rest_pos is None):
            return
        dt = self._frame_dt if dt is None else dt

        # Body horizontal velocity from the CLEAN body translation (set by camera
        # follow). Yaw is a separate xform op there, so turning in place gives no
        # phantom forward velocity, and the value is already One-Euro smoothed.
        if self._body_pos is None:
            return
        rxz = Gf.Vec3f(float(self._body_pos[0]), 0.0, float(self._body_pos[2]))

        # Crouch / sit takes over the legs when the pelvis lowers (and suppresses
        # stepping). Keep the gait velocity anchor fresh so walking resumes cleanly.
        if self._apply_crouch_sit(sk, rxz, dt):
            self._gait_root_prev = rxz
            return

        if self._gait_root_prev is None:
            self._gait_root_prev = rxz
            return
        vel = _scale(_sub(rxz, self._gait_root_prev), 1.0 / max(dt, 1e-3))
        self._gait_root_prev = rxz
        # Light extra low-pass on top of the already-smoothed position.
        self._gait_speed += (_len(vel) - self._gait_speed) * (1.0 - math.exp(-6.0 * dt))
        # Hysteresis so head-bob noise can't flicker the gait on and off: start
        # stepping above speed_min, keep stepping until clearly stopped.
        if self._gait_speed > self._gait_speed_min:
            moving = True
        elif self._gait_speed < self._gait_speed_min * 0.5:
            moving = False
        else:
            moving = self._foot_state["L"]["region"] is not None

        # Heading from travel direction while moving; hold the body facing at rest.
        if self._gait_speed > self._gait_speed_min:
            self._gait_heading = _normalize(vel)
        if self._gait_heading is None:
            self._gait_heading = _rot_y(Gf.Vec3f(0, 0, -1), self._body_yaw_live)
        fwd = self._gait_heading

        # Anti-skate: pick the CADENCE first (mild rise with speed), then derive
        # the stride from it — step_len = distance travelled per step = speed /
        # (2·cadence). A foot then plants exactly where the body will be, so a
        # world-locked stance foot doesn't slide. step_len → 0 as the body stops,
        # so there is no overstep/march-in-place at low speed.
        cadence  = (max(0.6, min(1.4, 0.6 + self._gait_speed * 0.45))
                    * self._gait_cadence_mult)
        step_len = min(0.7, self._gait_speed / (2.0 * max(cadence, 1e-3))
                       * self._gait_stride_mult)
        if moving:
            self._gait_phase = (self._gait_phase + cadence * dt) % 1.0

        # PASS 1 — per-foot world target, ground (raycast) and foot-roll phase.
        targets = {}
        for side, leg in sk.legs.items():
            st = self._foot_state[side]
            # Neutral foot anchor = the foot's ACTUAL rest position mapped to the
            # live world (tracks the moving hip). Standing on this reproduces the
            # avatar's real rest pose exactly — no lean, correct fore/aft spread.
            neutral  = self._rest_to_world(leg["ankle_pos"])
            ground_y = float(neutral[1])

            if not moving:
                cur = st["world"] if st["world"] is not None else neutral
                ease = 1.0 - math.exp(-10.0 * dt)
                xz = _add(cur, _scale(_sub(neutral, cur), ease))
                region, p = None, 0.0
            else:
                p = (self._gait_phase + (0.0 if side == "L" else 0.5)) % 1.0
                if p >= 0.5:   # SWING: arc from last plant to a new plant ahead
                    if st["region"] != "swing":
                        st["from"] = st["world"] if st["world"] is not None else neutral
                    swing_to = _add(neutral, _scale(fwd, step_len * 0.5))
                    t = (p - 0.5) / 0.5
                    s = t * t * (3.0 - 2.0 * t)              # smoothstep
                    xz = _add(st["from"], _scale(_sub(swing_to, st["from"]), s))
                    region = "swing"
                else:          # STANCE: foot stays world-locked at its plant
                    if st["region"] != "stance" or st["plant"] is None:
                        w = st["world"] if st["world"] is not None else neutral
                        st["plant"] = Gf.Vec3f(float(w[0]), ground_y, float(w[2]))
                    xz = st["plant"]
                    region = "stance"

            lift = (self._gait_lift * math.sin(math.pi * ((p - 0.5) / 0.5))
                    if (moving and region == "swing") else 0.0)
            pitch, toe = self._foot_roll(p) if moving else (0.0, 0.0)
            # Foot-roll floor lift: the foot pitches about the ANKLE, so an
            # extremity dips below the sole — the heel when toe-up (pitch>0), the
            # toe when toe-down (pitch<0). Raise the ankle target by that dip so the
            # foot pivots on the floor instead of clipping through it.
            foot_len = leg.get("foot_len", 0.12)
            if pitch >= 0.0:
                dip = foot_len * self._gait_heel_frac * math.sin(pitch)
            else:
                dip = foot_len * math.sin(-pitch)
            foot_w = Gf.Vec3f(float(xz[0]), ground_y + lift + dip, float(xz[2]))
            st["world"], st["region"] = foot_w, region
            targets[side] = {"foot": foot_w, "pitch": pitch, "toe": toe}

        # PASS 2 — shared pelvis drop (item 2) so the most-extended leg reaches
        # without overstretching, then solve each leg from the dropped hip.
        drop_target = 0.0
        for side, leg in sk.legs.items():
            thigh_live = self._rest_to_world(leg["thigh_pos"])
            needed     = _len(_sub(targets[side]["foot"], thigh_live))
            max_reach  = (leg["thigh_len"] + leg["calf_len"]) * 0.98
            drop_target = max(drop_target, needed - max_reach)
        drop_target = max(0.0, min(self._gait_drop_max, drop_target))
        self._pelvis_drop += (drop_target - self._pelvis_drop) * (1.0 - math.exp(-8.0 * dt))
        self._apply_pelvis_drop(self._pelvis_drop)

        for side in sk.legs:
            tg = targets[side]
            self._solve_leg_ik(side, tg["foot"], self._pelvis_drop,
                               tg["pitch"], tg["toe"])

    @staticmethod
    def _foot_roll_curve(p, heel, toe):
        """Foot pitch (rad, + = toe up) and toe-joint bend (rad) over a foot's
        gait phase p∈[0,1): heel-strike → flat → toe-off → swing dorsiflexion."""
        if p < 0.12:                       # heel strike easing to flat
            return heel * (1.0 - p / 0.12), 0.0
        if p < 0.38:                       # flat mid-stance
            return 0.0, 0.0
        if p < 0.5:                        # toe-off: heel lifts, toe bends
            f = (p - 0.38) / 0.12
            return -toe * f, toe * f
        if p < 0.8:                        # swing: plantar → dorsi, toe relaxes
            f = (p - 0.5) / 0.3
            return -toe * (1.0 - f) + heel * f, toe * (1.0 - f)
        return heel, 0.0                   # hold dorsiflexed for the next strike

    def _foot_roll(self, p):
        return self._foot_roll_curve(p, math.radians(self._gait_heel_deg),
                                     math.radians(self._gait_toe_deg))

    def _solve_leg_ik(self, side, foot_world, drop, pitch, toe):
        """Solve a leg as a 2-bone chain (thigh→calf) to a world foot target and
        bake thigh/calf/foot(+toe) local rotations.

        Uses _swing_to_local (not _bone_rotation_from_vectors): the bones are
        rotated relative to their AUTHORED rest local rotations, so a foot at the
        rest position reproduces the avatar's exact rest leg pose — no residual
        tilt from world→local rotation reconstruction. `drop` lowers the hip the
        leg solves from (pelvis drop); `pitch`/`toe` apply the foot roll."""
        sk = self._skel
        leg = sk.legs[side]
        # Solve from the (pelvis-)dropped hip; both frames share +Y, so the metric
        # drop maps straight to -Y in the cached rest/solve frame.
        thigh_root = _add(leg["thigh_pos"], Gf.Vec3f(0.0, -drop, 0.0))
        target = self._world_to_rest(foot_world)   # solve in the cached rest frame
        # Pole = the rig's own rest knee-bend direction (see leg discovery).
        knee, ankle, _b, _h, _u, _v = _two_bone_ik_full(
            thigh_root, target, leg["thigh_len"], leg["calf_len"], leg["pole"])

        thigh_dir = _normalize(_sub(knee, thigh_root))
        calf_dir  = _normalize(_sub(ankle, knee))

        # Thigh: parent = Hip (never re-rotated → live world == rest world rot).
        thigh_local, thigh_world = _swing_to_local(
            leg["rest_thigh_dir"], thigh_dir, leg["thigh_q_rest"],
            sk.hip_world_q_rest, sk.hip_world_q_rest,
            leg["rest_local"][leg["thigh_idx"]])
        sk.write_joint_rotation(leg["thigh_idx"], thigh_local)

        # Calf: parent = thigh (its just-computed live world rotation).
        calf_local, calf_world = _swing_to_local(
            leg["rest_calf_dir"], calf_dir, leg["calf_q_rest"],
            leg["thigh_q_rest"], thigh_world,
            leg["rest_local"][leg["calf_idx"]])
        sk.write_joint_rotation(leg["calf_idx"], calf_local)

        # Foot: heel/toe pitch (gait) + ANKLE DORSIFLEXION CLAMP. Holding the foot
        # flat while the shin tilts forward (crouch) over-bends the ankle and makes
        # the knee read as under-bent. C = how much the calf rotated from rest in
        # world; past _crouch_ankle_max_deg the heel lifts (foot follows the shin)
        # so the ankle stays at the limit — a natural squat. Gait legs barely
        # rotate the calf, so the clamp never bites there.
        right   = sk.leg_right_axis
        pitch_q = _quat_axis_angle(right, pitch * self._gait_roll_sign)
        C = _quat_mul(calf_world, _quat_conj(leg["calf_q_rest"]))
        if C.GetReal() < 0.0:                                  # canonical (w >= 0)
            im = C.GetImaginary()
            C = Gf.Quatf(-C.GetReal(), -im[0], -im[1], -im[2])
        cd = _quat_angle(C)
        amax = math.radians(self._crouch_ankle_max_deg)
        heel = (_quat_scale_angle(C, 1.0 - amax / cd)
                if cd > amax + 1e-4 else Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        adj = _quat_mul(pitch_q, heel)
        foot_world_live = _quat_mul(adj, leg["foot_q_rest"])
        foot_local = _quat_mul(
            _quat_mul(_quat_conj(calf_world), _quat_mul(adj, leg["calf_q_rest"])),
            leg["rest_local"][leg["foot_idx"]])
        sk.write_joint_rotation(leg["foot_idx"], foot_local)

        # Toe bend (foot-relative) during toe-off, about the same right axis.
        ti = leg["toe_idx"]
        if ti is not None:
            bend_axis = _normalize(_quat_rotate(_quat_conj(foot_world_live), right))
            bend_q = _quat_axis_angle(bend_axis, toe * self._gait_roll_sign)
            sk.write_joint_rotation(ti, _quat_mul(bend_q, leg["rest_local"][ti]))

    def _apply_stick_locomotion(self, dt=0.016):
        """Smooth glide the PLAYER (XR origin / camera rig) from the RIGHT
        thumbstick each frame; the avatar follows via _apply_camera_follow.
        Camera-relative: stick +y (forward) -> camera -Z (move where you look),
        stick +x -> camera +X (strafe right)."""
        if not self._stick_loco_on or self._xr is None:
            return
        x, y = _stick_xy(self._right_dev)
        # Radial deadzone to ignore noise/drift.
        mag = math.sqrt(x * x + y * y)
        if mag < self._stick_deadz:
            return
        step = self._stick_speed * dt
        try:
            self._xr.schedule_move_space_origin_relative_to_camera(
                x * step, 0.0, -y * step)
        except Exception:
            pass

    def _apply_camera_follow(self):
        """Pin the avatar to the XR camera (VRChat-style): every frame move the
        avatar root so its head bone sits under the HMD's stage-space position,
        and lerp the root yaw toward the HMD heading. The camera rig is the
        authority — locomotion moves the rig, the body follows. XZ only: the
        feet stay on the floor when the user crouches or jumps."""
        sk = self._skel
        if sk is None or not self._follow_on:
            return
        m = _get_pose(self._head_dev)   # virtual-world = stage-space camera pose
        if m is None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        prim = stage.GetPrimAtPath(self._avatar_root_path())
        if not prim.IsValid():
            return

        cam = m.ExtractTranslation()
        # Pitch guard: when the camera looks near-vertically (down at the
        # hands/floor) the horizontal forward degenerates — keep the current
        # body heading instead of feeding atan2 noise into the yaw lerp.
        cam_fwd = m.TransformDir(Gf.Vec3d(0, 0, -1))
        if cam_fwd[0] * cam_fwd[0] + cam_fwd[2] * cam_fwd[2] > 0.04:
            target_yaw = math.atan2(-float(cam_fwd[0]), -float(cam_fwd[2]))
        else:
            target_yaw = self._root_yaw if self._root_yaw is not None else 0.0

        # Wraparound-safe exponential yaw lerp — the body realigns smoothly
        # instead of twitching with every head glance.
        if self._root_yaw is None:
            self._root_yaw = target_yaw
        else:
            diff = (target_yaw - self._root_yaw + math.pi) % (2 * math.pi) - math.pi
            self._root_yaw += diff * (1.0 - math.exp(-self._follow_yaw_rate * self._frame_dt))
        yaw = self._root_yaw

        # EYE anchor offset in WORLD frame (Y-up, metres). The asset's
        # root-LOCAL frame is Z-up, so root-local vectors must never be used
        # here. The follow-yaw op sits directly inside the translate op, so
        # the yaw pivot is the root's translate position:
        #   world_eye = T + R_y(yaw) · (eye_rest_world − T_rest)
        # Eye = head bone + _eye_fwd along avatar rest forward (world -Z).
        off = _rot_y(Gf.Vec3f(
            float(sk.head_rest_world[0] - sk.root_rest_translate[0]),
            0.0,
            float(sk.head_rest_world[2] - sk.root_rest_translate[2]) - self._eye_fwd,
        ), yaw)
        pos = Gf.Vec3f(float(cam[0]) - off[0], 0.0, float(cam[2]) - off[2])
        if self._follow_smooth_on:
            pos = self._filt_root.filter(pos)

        # Stash the clean body XZ position + heading for the procedural walk. This
        # is the true translation (yaw is a SEPARATE op below), so turning in place
        # produces no phantom forward velocity, and it is already One-Euro smoothed.
        self._body_pos = pos
        self._body_yaw_live = yaw

        xform = UsdGeom.Xformable(prim)
        translate_op, follow_op = None, None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and translate_op is None:
                translate_op = op
            elif op.GetOpName() == sk.FOLLOW_YAW_OP:
                follow_op = op
            elif op.GetOpName() == "xformOp:rotateY":
                # Leftover from the first follow implementation — appended
                # innermost it rotated about the asset's Z-up local Y (= a
                # world ROLL). Neutralise; the suffixed op below replaces it.
                op.Set(0.0)
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
        if follow_op is None:
            follow_op = xform.AddRotateYOp(opSuffix="avatarFollow")
            # Order [translate, follow-yaw, <asset ops>]: directly inside the
            # translate the yaw is applied in the parent (= world) frame and
            # turns the avatar about the world up axis. Appended at the end it
            # would rotate in the asset's Z-up local frame and roll the body.
            others = [op for op in xform.GetOrderedXformOps()
                      if op.GetOpName() not in (translate_op.GetOpName(),
                                                follow_op.GetOpName())]
            xform.SetXformOpOrder([translate_op, follow_op] + others,
                                  xform.GetResetXformStack())
        cur = translate_op.Get() or Gf.Vec3d(0, 0, 0)
        translate_op.Set(Gf.Vec3d(float(pos[0]), float(cur[1]), float(pos[2])))
        follow_op.Set(math.degrees(yaw))

        self._follow_tick += 1
        if DEBUG_FILES and self._follow_tick % 60 == 0:
            try:
                # Live eye via the full matrix path — must match cam in XZ.
                eye_rest = Gf.Vec3f(float(sk.head_rest_world[0]),
                                    float(sk.head_rest_world[1]) + self._eye_up,
                                    float(sk.head_rest_world[2]) - self._eye_fwd)
                live_eye = self._rest_to_world(eye_rest)
                with open(_data_path("_follow_debug.txt"),
                          "w", encoding="utf-8") as f:
                    f.write(
                        f"cam=({cam[0]:+.3f},{cam[1]:+.3f},{cam[2]:+.3f})  "
                        f"yaw_target={math.degrees(target_yaw):+.1f}deg\n"
                        f"root=({pos[0]:+.3f},{cur[1]:+.3f},{pos[2]:+.3f})  "
                        f"yaw={math.degrees(yaw):+.1f}deg\n"
                        f"live_eye=({live_eye[0]:+.3f},{live_eye[1]:+.3f},"
                        f"{live_eye[2]:+.3f})  (XZ should equal cam XZ)\n"
                        f"head_rest_world=({sk.head_rest_world[0]:+.3f},"
                        f"{sk.head_rest_world[1]:+.3f},"
                        f"{sk.head_rest_world[2]:+.3f})  "
                        f"root_rest_T=({sk.root_rest_translate[0]:+.3f},"
                        f"{sk.root_rest_translate[1]:+.3f},"
                        f"{sk.root_rest_translate[2]:+.3f})\n")
            except Exception:
                pass

    def _update_pole_lbl(self):
        self._set_lbl("_pole_lbl",
            f"elbow pole: down {self._pole_down:.1f} back {self._pole_back:.1f}")

    def _pole_down_up(self):
        self._pole_down = min(3.0, self._pole_down + 0.25)
        self._update_pole_lbl()

    def _pole_down_dn(self):
        self._pole_down = max(-3.0, self._pole_down - 0.25)
        self._update_pole_lbl()

    def _pole_back_up(self):
        self._pole_back = min(3.0, self._pole_back + 0.25)
        self._update_pole_lbl()

    def _pole_back_dn(self):
        self._pole_back = max(-3.0, self._pole_back - 0.25)
        self._update_pole_lbl()

    # --- Clavicle / shoulder-follow controls ---
    def _update_clav_lbl(self):
        state = "ON" if self._clav_follow else "OFF"
        self._set_lbl("_clav_lbl",
            f"shoulder follow: {state}  (strength {self._clav_weight:.2f})")

    def _toggle_clav_follow(self):
        self._clav_follow = not self._clav_follow
        self._update_clav_lbl()

    def _clav_weight_up(self):
        self._clav_weight = min(1.0, self._clav_weight + 0.1)
        self._update_clav_lbl()

    def _clav_weight_dn(self):
        self._clav_weight = max(0.0, self._clav_weight - 0.1)
        self._update_clav_lbl()

    # --- Phase 1: One Euro smoothing controls ---
    def _update_smooth_lbl(self):
        state = "ON" if self._smooth_on else "OFF"
        self._set_lbl("_smooth_lbl",
            f"smoothing: {state}  cutoff {self._smooth_cutoff:.2f}Hz  beta {self._smooth_beta:.3f}")

    def _push_smooth_params(self):
        for f in (self._filt_head, self._filt_rhand, self._filt_lhand):
            f.min_cutoff = self._smooth_cutoff
            f.beta = self._smooth_beta

    def _toggle_smooth(self):
        self._smooth_on = not self._smooth_on
        # Reset filter state so toggling on doesn't jump from a stale value.
        for f in (self._filt_head, self._filt_rhand, self._filt_lhand,
                  self._filt_rrot, self._filt_lrot):
            f.reset()
        self._update_smooth_lbl()

    def _smooth_cutoff_up(self):  # less jitter reduction (higher cutoff = more responsive)
        self._smooth_cutoff = min(8.0, self._smooth_cutoff + 0.25)
        self._push_smooth_params(); self._update_smooth_lbl()

    def _smooth_cutoff_dn(self):  # more jitter reduction (lower cutoff = smoother)
        self._smooth_cutoff = max(0.25, self._smooth_cutoff - 0.25)
        self._push_smooth_params(); self._update_smooth_lbl()

    def _smooth_beta_up(self):    # "Lag -" : raise beta = less lag in motion
        self._smooth_beta = min(0.5, self._smooth_beta + 0.01)
        self._push_smooth_params(); self._update_smooth_lbl()

    def _smooth_beta_dn(self):    # "Lag +" : lower beta = more lag, smoother
        self._smooth_beta = max(0.0, self._smooth_beta - 0.01)
        self._push_smooth_params(); self._update_smooth_lbl()

    # ------------------------------------------------------------------
    # Input Simulation (for testing without headset)
    # ------------------------------------------------------------------

    def _start_simulation(self):
        """Start synthetic hand/head movement simulation."""
        self._sim_enabled = True
        self._sim_time = 0.0
        self._sim_pose = "tpose"
        self._sim_lbl.text = "simulation: ON"
        self._sim_lbl.style = {"font_size": 11, "color": 0xFF44FF44}
        # Enable IK if not already enabled (calibration not required for simulation)
        self._ik_enabled = True
        # Initialize body yaw for simulation if not set
        if self._body_yaw is None:
            self._body_yaw = 0.0
        # Start tracking if not already running
        if not self._xr_active:
            self._start_tracking()
        self._set_status("Input simulation started", error=False)

    def _stop_simulation(self):
        """Stop synthetic movement simulation."""
        self._sim_enabled = False
        self._sim_lbl.text = "simulation: off"
        self._sim_lbl.style = {"font_size": 11, "color": 0xFFCCCCCC}
        self._set_status("Input simulation stopped", error=False)

    def _set_sim_pose(self, pose_name):
        """Set which test pose to simulate."""
        self._sim_pose = pose_name
        self._sim_time = 0.0
        self._pose_lbl.text = f"pose: {pose_name}"

    # ------------------------------------------------------------------
    # One-click automated follow-capture (streamlined debugging)
    # ------------------------------------------------------------------
    _DEBUG_CAPTURE_PATH = _data_path("_avatar_follow_capture.csv")
    _DEBUG_POSES = [
        "tpose", "reach_fwd", "reach_up", "reach_down", "reach_left", "reach_right",
        "reach_fwd_up", "reach_fwd_down", "reach_left_up", "reach_right_up",
        "reach_fwd_left", "reach_fwd_right", "reach_left_down", "reach_right_down",
    ]

    def _start_debug_capture(self):
        """One click: enable simulation + IK + tracking, step through every test
        pose, and write a CSV comparing the simulated controller target with the
        resulting avatar joint positions — so follow fidelity can be reviewed from
        a file instead of by eye. Does not change the collision/limit toggles
        (the current config is recorded in the file header)."""
        if self._debug_capturing:
            return
        if self._skel is None:
            self._debug_cap_lbl.text = "capture: skeleton not ready"
            return
        asyncio.ensure_future(self._debug_capture_loop())

    async def _debug_capture_loop(self):
        self._debug_capturing = True
        prev_sim, prev_pose = self._sim_enabled, self._sim_pose
        try:
            self._sim_enabled = True
            self._ik_enabled = True
            if self._body_yaw is None:
                self._body_yaw = 0.0
            if not self._xr_active:
                self._start_tracking()
            rows = []
            settle = 90                    # ~1.5 s so smoothing/easing/swivel settle
            for i, pose in enumerate(self._DEBUG_POSES):
                self._sim_pose = pose
                self._sim_time = 0.0
                self._debug_cap_lbl.text = (
                    f"capture: {pose} ({i + 1}/{len(self._DEBUG_POSES)})...")
                for _ in range(settle):
                    await asyncio.sleep(0.016)
                self._collect_capture_rows(rows, pose)

            # --- Wrist-twist (orientation) sweep -------------------------------
            # Drive a synthetic pronation/supination twist about the forearm axis
            # and record how it is distributed (forearm vs hand) and whether it is
            # followed or clipped by the wrist clamp. Tests the forearm-roll fix
            # that the position capture can't reach.
            twist_rows = []
            for pose in ("tpose", "reach_up", "reach_fwd"):
                self._sim_pose = pose
                self._sim_time = 0.0
                for ang in (-90, -45, 0, 45, 90):
                    self._sim_wrist_twist = math.radians(ang)
                    self._debug_cap_lbl.text = f"twist: {pose} {ang:+d}deg..."
                    for _ in range(20):
                        await asyncio.sleep(0.016)
                    for is_right in (True, False):
                        d = self._sim_twist_diag.get(is_right)
                        if d is None:
                            continue
                        twist_rows.append({"pose": pose,
                                           "arm": "R" if is_right else "L", **d})
            self._sim_wrist_twist = 0.0

            self._write_capture(rows, twist_rows)
            self._debug_cap_lbl.text = f"capture done -> {self._DEBUG_CAPTURE_PATH}"
            print(f"[avatar_xr_control] debug capture written: "
                  f"{self._DEBUG_CAPTURE_PATH} ({len(rows)} rows)")
        except Exception as e:
            import traceback
            self._debug_cap_lbl.text = f"capture error: {e}"
            print(f"[avatar_xr_control] debug capture failed: {e}\n"
                  f"{traceback.format_exc()}")
        finally:
            self._sim_pose, self._sim_enabled = prev_pose, prev_sim
            self._debug_capturing = False

    def _collect_capture_rows(self, rows, pose_label):
        """Append one metric row per arm from the CURRENT solved IK state. Shared
        by the synthetic-pose capture and the replay-metrics capture."""
        sk = self._skel
        if sk is None:
            return
        for is_right in (True, False):
            # Synthetic capture: ctrl = the authored controller (avatar frame).
            # Replay capture: ctrl = the 1:1-mapped target (the raw controller is
            # in physical space, so comparing to it directly is meaningless) — the
            # follow error then reads as the reach-cap/limit/push displacement.
            ctrl = (self._map_ideal_world.get(is_right) if self._replay_capturing
                    else self._sim_ctrl_world.get(is_right))
            tgt  = self._sim_hand_world.get(is_right)
            wr   = self._hand_world.get(is_right)
            el   = self._elbow_world.get(is_right)
            solved = self._arm_solved.get(is_right)
            if None in (ctrl, tgt, wr, el) or solved is None:
                continue
            # Reach fraction of the controller request: |ctrl-shoulder|/armlen.
            # >1 means the controller asked beyond the arm's length, so the hand
            # is capped (expected, not a tracking failure).
            sh = self._rest_to_world(solved[0])
            maxr = ((sk.r_upperarm_len + sk.r_forearm_len) if is_right
                    else (sk.l_upperarm_len + sk.l_forearm_len))
            reach_frac = _len(_sub(ctrl, sh)) / max(1e-6, maxr)
            # Body intersection: penetration (m, 0=clear) of the rendered upper
            # arm / elbow / forearm+hand into the analytic torso. Uses the REST-
            # frame solved joints; the upper-arm test skips the shoulder third.
            sh_r, el_r, wr_r = solved
            fdir = _sub(wr_r, el_r)
            flen = _len(fdir)
            hand_end = (_add(wr_r, _scale(fdir, self._HAND_EXT / flen))
                        if flen > 1e-5 else wr_r)
            up_pen = self._seg_torso_pen(sh_r, el_r, self._UPPER_R, t0=0.35)
            fo_pen = self._seg_torso_pen(el_r, hand_end, self._FORE_R)
            el_pen = self._torso_pen_point(el_r, self._UPPER_R)
            # Elbow "down-ness": angle of the elbow's bend direction (perpendicular
            # to the shoulder→wrist axis) from straight-down. 0°=points down,
            # 180°=points up. arm_y = vertical component of the arm direction
            # (<0 ⇒ hand is below the shoulder). Lets us check the elbow-down rule
            # for below-head poses directly from the capture.
            ax = _sub(wr, sh); axl = _len(ax)
            arm_y = (float(ax[1]) / axl) if axl > 1e-6 else 0.0
            elbow_down_deg = 0.0
            if axl > 1e-6:
                axn = _scale(ax, 1.0 / axl)
                bend = _sub(el, sh)
                bend_perp = _sub(bend, _scale(axn, _dot(bend, axn)))
                downv = Gf.Vec3f(0.0, -1.0, 0.0)
                down_perp = _sub(downv, _scale(axn, _dot(downv, axn)))
                bl, dl = _len(bend_perp), _len(down_perp)
                if bl > 1e-5 and dl > 1e-5:
                    c = max(-1.0, min(1.0, _dot(bend_perp, down_perp) / (bl * dl)))
                    elbow_down_deg = math.degrees(math.acos(c))
            rows.append({
                "pose": pose_label, "arm": "R" if is_right else "L",
                "ctrl": ctrl, "target": tgt, "wrist": wr, "elbow": el,
                "err_target_vs_ctrl": _len(_sub(tgt, ctrl)),
                "err_wrist_vs_target": _len(_sub(wr, tgt)),
                "err_wrist_vs_ctrl": _len(_sub(wr, ctrl)),
                "reach_frac": reach_frac,
                "upper_pen": up_pen, "elbow_pen": el_pen, "fore_pen": fo_pen,
                "elbow_down_deg": elbow_down_deg, "arm_y": arm_y,
            })

    def _start_replay_capture(self):
        if self._debug_capturing or self._replay_capturing:
            return
        if self._rec_enabled:
            self._set_track("stop recording before capturing metrics", 0xFFFFAA33)
            return
        if not self._load_recording():
            self._set_track("no recording to capture metrics from", 0xFFFFAA33)
            return
        asyncio.ensure_future(self._replay_capture_loop())

    async def _replay_capture_loop(self):
        """Step through the recorded demo, run the IK each frame, and write the same
        follow/penetration metrics as the synthetic capture — but for REAL motion
        through the loaded calibration. Eases are paced by the recorded timestamps."""
        self._replay_capturing = True
        saved_devs = (self._head_dev, self._left_dev, self._right_dev)
        prev_sim, prev_play = self._sim_enabled, self._play_enabled
        # Pause the background tracking loop so it doesn't race our synchronous
        # frame stepping; resume it afterwards if it was running.
        was_active = self._xr_active
        self._xr_active = False
        if self._tracking_task and not self._tracking_task.done():
            self._tracking_task.cancel()
        await asyncio.sleep(0)         # let the cancellation take effect
        self._sim_enabled = False
        self._play_enabled = False     # we drive the frames ourselves here
        self._ik_enabled = True
        if self._body_yaw is None:
            self._body_yaw = 0.0
        try:
            rows = []
            orient_rows = []
            n = len(self._play_frames)
            prev_t = None
            if self._skel is not None:
                self._skel.set_deferred(True)   # one USD update per frame
            for i, fr in enumerate(self._play_frames):
                t = fr.get("t", 0.0)
                if prev_t is not None:
                    self._frame_dt = max(0.005, min(0.2, t - prev_t))
                prev_t = t
                self._head_dev  = _PlaybackDevice(fr.get("head"))
                self._left_dev  = _PlaybackDevice(fr.get("left"))
                self._right_dev = _PlaybackDevice(fr.get("right"))
                try:
                    self._apply_camera_follow()
                except Exception:
                    pass
                self._apply_head()
                self._apply_upper_body()
                self._apply_hand_tracking()
                if self._skel is not None:
                    self._skel.flush()
                self._collect_capture_rows(rows, f"f{i}")
                for is_right in (True, False):
                    d = self._hand_orient_diag.get(is_right)
                    if d is not None:
                        orient_rows.append({"frame": f"f{i}",
                                            "arm": "R" if is_right else "L", **d})
                if i % 20 == 0:
                    if getattr(self, "_capture_lbl", None) is not None:
                        self._capture_lbl.text = f"capturing metrics {i}/{n}..."
                    await asyncio.sleep(0)   # keep the UI responsive
            self._write_capture(
                rows, None, path=self._replay_capture_path,
                title=f"Replay metrics capture ({n} frames, "
                      f"{os.path.basename(self._rec_path)})",
                orient_rows=orient_rows)
            self._set_track(f"replay metrics -> {self._replay_capture_path} "
                            f"({len(rows)} rows)", 0xFF44FF44)
            print(f"[avatar_xr_control] replay capture written: "
                  f"{self._replay_capture_path} ({len(rows)} rows)")
        except Exception as e:
            import traceback
            self._set_track(f"replay capture error: {e}", 0xFF4444FF)
            print(f"[avatar_xr_control] replay capture failed: {e}\n"
                  f"{traceback.format_exc()}")
        finally:
            self._replay_capturing = False
            self._sim_enabled = prev_sim
            self._play_enabled = prev_play
            self._head_dev, self._left_dev, self._right_dev = saved_devs
            self._restore_live_calib()
            if self._skel is not None:
                self._skel.set_deferred(False)   # flushes pending writes
            # Resume the background tracking loop if it was running before.
            if was_active and not self._xr_active:
                self._start_tracking()
            if getattr(self, "_capture_lbl", None) is not None:
                self._update_capture_lbl()

    def _write_capture(self, rows, twist_rows=None, path=None, title=None,
                       orient_rows=None):
        """Write the captured follow data as CSV with a config + summary header.
        `rows` = position follow; `twist_rows` = synthetic wrist-twist sweep;
        `orient_rows` = per-frame hand-orientation diagnostic (replay)."""
        def mean_max(key):
            vals = [r[key] for r in rows] or [0.0]
            return sum(vals) / len(vals), max(vals)

        lines = []
        lines.append(f"# {title or 'Avatar follow capture'}")
        lines.append(f"# limits_on={self._limits_on} body_push={self._body_push} "
                     f"phys_collision={self._phys_collision} "
                     f"phys_shoulder_push={self._phys_shoulder_push}")
        lines.append(f"# ik_scale={self._ik_scale:.3f} ik_scale_mult={self._ik_scale_mult:.3f} "
                     f"fore_roll={self._fore_roll:.2f} lim_reach={self._lim_reach:.2f}")
        if self._skel is not None:
            lines.append(f"# torso_half_x={self._skel.torso_half_x:.3f} "
                         f"torso_half_z={self._skel.torso_half_z:.3f} "
                         f"torso_fwd={self._torso_fwd:.2f} "
                         f"protract_max={math.degrees(self._protract_max):.0f}deg "
                         f"clav_follow={self._clav_follow}")
        for k, label in (("err_target_vs_ctrl", "target_vs_ctrl (mapping: scale/limits)"),
                         ("err_wrist_vs_target", "wrist_vs_target (IK reach residual)"),
                         ("err_wrist_vs_ctrl",  "wrist_vs_ctrl (TOTAL follow error)")):
            m, mx = mean_max(k)
            lines.append(f"# {label}: mean={m * 100:.1f}cm max={mx * 100:.1f}cm")
        # Body intersection summary: worst limb penetration into the torso and how
        # many pose/arm rows clip it (>1cm). 0 clips = the elbow/arm stays out.
        worst_pen = max([max(r["upper_pen"], r["elbow_pen"], r["fore_pen"])
                         for r in rows] or [0.0])
        n_clip = sum(1 for r in rows
                     if max(r["upper_pen"], r["elbow_pen"], r["fore_pen"]) > 0.01)
        lines.append(f"# body intersection (vs analytic torso): worst="
                     f"{worst_pen * 100:.1f}cm  clipping_rows={n_clip}/{len(rows)}")
        lines.append("pose,arm,"
                     "ctrl_x,ctrl_y,ctrl_z,target_x,target_y,target_z,"
                     "wrist_x,wrist_y,wrist_z,elbow_x,elbow_y,elbow_z,"
                     "err_target_vs_ctrl_cm,err_wrist_vs_target_cm,err_wrist_vs_ctrl_cm,"
                     "reach_frac,upper_pen_cm,elbow_pen_cm,fore_pen_cm,clips,"
                     "elbow_down_deg,arm_y")

        def fv(v):
            return f"{float(v[0]):.4f},{float(v[1]):.4f},{float(v[2]):.4f}"

        for r in rows:
            clips = int(max(r["upper_pen"], r["elbow_pen"], r["fore_pen"]) > 0.01)
            lines.append(
                f"{r['pose']},{r['arm']},{fv(r['ctrl'])},{fv(r['target'])},"
                f"{fv(r['wrist'])},{fv(r['elbow'])},"
                f"{r['err_target_vs_ctrl'] * 100:.2f},"
                f"{r['err_wrist_vs_target'] * 100:.2f},"
                f"{r['err_wrist_vs_ctrl'] * 100:.2f},"
                f"{r['reach_frac']:.2f},"
                f"{r['upper_pen'] * 100:.2f},{r['elbow_pen'] * 100:.2f},"
                f"{r['fore_pen'] * 100:.2f},{clips},"
                f"{r['elbow_down_deg']:.0f},{r['arm_y']:+.2f}")

        # --- Wrist-twist (orientation) section --------------------------------
        if twist_rows:
            # follow_err = |total − requested|: 0 ⇒ the twist was fully followed.
            ferrs = [abs(t["total_deg"] - abs(t["requested_deg"])) for t in twist_rows]
            n_clamped = sum(1 for t in twist_rows if t["clamped"])
            lines.append("")
            lines.append("# --- Wrist twist (forearm-roll) ---")
            lines.append(f"# fore_roll={self._fore_roll:.2f} lim_wrist={math.degrees(self._lim_wrist):.0f}deg")
            lines.append(f"# follow error mean={sum(ferrs) / len(ferrs):.1f}deg "
                         f"max={max(ferrs):.1f}deg  clamped_rows={n_clamped}/{len(twist_rows)}")
            lines.append("twist_pose,arm,requested_deg,forearm_deg,hand_deg,"
                         "total_deg,followed_deg_err,clamped")
            for t in twist_rows:
                lines.append(
                    f"{t['pose']},{t['arm']},{t['requested_deg']:+.1f},"
                    f"{t['forearm_deg']:.1f},{t['hand_deg']:.1f},{t['total_deg']:.1f},"
                    f"{abs(t['total_deg'] - abs(t['requested_deg'])):.1f},"
                    f"{int(t['clamped'])}")

        # --- Hand orientation section (replay) --------------------------------
        if orient_rows:
            reqs = [o["req_deg"] for o in orient_rows]
            fores = [o["forearm_deg"] for o in orient_rows]
            bends = [o["hand_bend_deg"] for o in orient_rows]
            twists = [o["hand_twist_deg"] for o in orient_rows]
            n_cl = sum(1 for o in orient_rows if o["clamped"])
            mean = lambda a: sum(a) / len(a)
            lines.append("")
            lines.append("# --- Hand orientation (real motion) ---")
            lines.append(f"# fore_roll={self._fore_roll:.2f} "
                         f"lim_wrist={math.degrees(self._lim_wrist):.0f}deg")
            lines.append(f"# requested wrist angle: mean={mean(reqs):.0f} "
                         f"max={max(reqs):.0f}deg")
            lines.append(f"# routed to forearm: mean={mean(fores):.0f}deg | "
                         f"residual hand bend mean={mean(bends):.0f} max={max(bends):.0f} | "
                         f"hand twist mean={mean(twists):.0f} max={max(twists):.0f}")
            lines.append(f"# wrist clamp hit: {n_cl}/{len(orient_rows)} frames "
                         f"({100.0 * n_cl / len(orient_rows):.0f}%)")
            lines.append("orient_frame,arm,req_deg,forearm_deg,"
                         "hand_bend_deg,hand_twist_deg,clamped")
            for o in orient_rows:
                lines.append(
                    f"{o['frame']},{o['arm']},{o['req_deg']:.0f},"
                    f"{o['forearm_deg']:.0f},{o['hand_bend_deg']:.0f},"
                    f"{o['hand_twist_deg']:.0f},{int(o['clamped'])}")

        with open(path or self._DEBUG_CAPTURE_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    def _get_simulated_pose(self, is_head: bool, is_right: bool = None):
        """Generate a synthetic pose matrix for testing.

        Returns a Gf.Matrix4d in stage space (world frame).
        """
        sk = self._skel
        if sk is None:
            return None

        self._sim_time += 0.016

        # Base position: head at rest, hands at rest position
        if is_head:
            # Head stays roughly at rest position
            head_pos = sk.head_rest_world
            return Gf.Matrix4d(1.0).SetTranslateOnly(head_pos)

        # Hand poses (right vs left)
        shoulder_pos = sk.r_shoulder_pos if is_right else sk.l_shoulder_pos
        forearm_len = sk.r_forearm_len if is_right else sk.l_forearm_len
        upperarm_len = sk.r_upperarm_len if is_right else sk.l_upperarm_len
        reach = upperarm_len + forearm_len

        pos = Gf.Vec3f(shoulder_pos[0], shoulder_pos[1], shoulder_pos[2])

        # Define poses
        offset = 0.5 if is_right else -0.5  # left/right offset multiplier

        if self._sim_pose == "tpose":
            # T-pose: arms straight out to sides
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * offset,
                shoulder_pos[1],
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_fwd":
            # Reach forward (full extend)
            pos = Gf.Vec3f(
                shoulder_pos[0] + offset * 0.1,
                shoulder_pos[1],
                shoulder_pos[2] - reach * 0.95
            )
        elif self._sim_pose == "reach_left":
            # Reach to the left (both hands go left, no Z offset to avoid crossing)
            pos = Gf.Vec3f(
                shoulder_pos[0] - reach * 0.8,
                shoulder_pos[1],
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_right":
            # Reach to the right (both hands go right, no Z offset to avoid crossing)
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * 0.8,
                shoulder_pos[1],
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_up":
            # Reach up
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * 0.2 * offset,
                shoulder_pos[1] + reach * 0.8,
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_down":
            # Reach down (straight down, no inward pull)
            pos = Gf.Vec3f(
                shoulder_pos[0],
                shoulder_pos[1] - reach * 0.9,
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_fwd_up":
            # Reach forward and up (overhead forward)
            pos = Gf.Vec3f(
                shoulder_pos[0] + offset * 0.1,
                shoulder_pos[1] + reach * 0.6,
                shoulder_pos[2] - reach * 0.85
            )
        elif self._sim_pose == "reach_fwd_down":
            # Reach forward and down (waist-level forward)
            pos = Gf.Vec3f(
                shoulder_pos[0] + offset * 0.1,
                shoulder_pos[1] - reach * 0.3,
                shoulder_pos[2] - reach * 0.95
            )
        elif self._sim_pose == "reach_left_up":
            # Reach left and up
            pos = Gf.Vec3f(
                shoulder_pos[0] - reach * 0.6,
                shoulder_pos[1] + reach * 0.5,
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_right_up":
            # Reach right and up
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * 0.6,
                shoulder_pos[1] + reach * 0.5,
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_fwd_left":
            # Reach forward and left diagonal
            pos = Gf.Vec3f(
                shoulder_pos[0] - reach * 0.3,
                shoulder_pos[1],
                shoulder_pos[2] - reach * 0.95
            )
        elif self._sim_pose == "reach_fwd_right":
            # Reach forward and right diagonal
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * 0.3,
                shoulder_pos[1],
                shoulder_pos[2] - reach * 0.95
            )
        elif self._sim_pose == "reach_left_down":
            # Reach left and down (less extreme lateral reach to avoid crossing body)
            pos = Gf.Vec3f(
                shoulder_pos[0] - reach * 0.4,
                shoulder_pos[1] - reach * 0.8,
                shoulder_pos[2]
            )
        elif self._sim_pose == "reach_right_down":
            # Reach right and down (less extreme lateral reach to avoid crossing body)
            pos = Gf.Vec3f(
                shoulder_pos[0] + reach * 0.4,
                shoulder_pos[1] - reach * 0.8,
                shoulder_pos[2]
            )

        # Apply global forward offset to move all poses closer to avatar's front
        pos = Gf.Vec3f(pos[0], pos[1], pos[2] - 0.25)

        m = Gf.Matrix4d(1.0)
        m.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        return m

    _CALIB_DEBUG_PATH = _data_path("_calib_debug.txt")
    _CALIB_SAVE_PATH  = _data_path("_avatar_calibration.json")

    def _save_calibration(self):
        """Persist the per-user calibration (reach scale + shoulder anchors +
        wrist orientation offsets) so it only has to be measured ONCE, not every
        session. Written after a successful T-pose calibration."""
        def v(o):
            return None if o is None else [float(o[0]), float(o[1]), float(o[2])]
        def q(o):
            return None if o is None else [float(o.GetReal()),
                                           float(o.GetImaginary()[0]),
                                           float(o.GetImaginary()[1]),
                                           float(o.GetImaginary()[2])]
        try:
            data = {
                "ik_scale":       float(self._ik_scale),
                "real_arm_len":   float(self._real_arm_len),
                "r_shoulder_off": v(self._r_shoulder_off),
                "l_shoulder_off": v(self._l_shoulder_off),
                "r_wrist_offset": q(self._r_wrist_offset),
                "l_wrist_offset": q(self._l_wrist_offset),
                # Standing HMD head height — the crouch/sit baseline. Without it
                # the next session falls back to the AVATAR's rest head height,
                # skewing crouch detection for users taller/shorter than the avatar.
                "calib_head_y":   (None if self._calib_head_y is None
                                   else float(self._calib_head_y)),
            }
            with open(self._CALIB_SAVE_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            print(f"[avatar_xr_control] calibration saved → {self._CALIB_SAVE_PATH}")
        except Exception as e:
            print(f"[avatar_xr_control] calibration save failed: {e}")

    def _load_calibration(self):
        """Load a previously-saved calibration. Returns True if a usable one was
        applied (so startup can skip the defaults). Calibrate once → reused forever."""
        if not os.path.exists(self._CALIB_SAVE_PATH):
            return False
        try:
            with open(self._CALIB_SAVE_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            def v(a):
                return None if not a else Gf.Vec3f(float(a[0]), float(a[1]), float(a[2]))
            def q(a):
                return None if not a else Gf.Quatf(float(a[0]), float(a[1]),
                                                   float(a[2]), float(a[3]))
            self._ik_scale       = float(data.get("ik_scale", self._ik_scale))
            self._real_arm_len   = float(data.get("real_arm_len", self._real_arm_len))
            self._r_shoulder_off = v(data.get("r_shoulder_off"))
            self._l_shoulder_off = v(data.get("l_shoulder_off"))
            self._symmetrize_shoulder_offsets()   # fix legacy asymmetric calibs
            self._r_wrist_offset = q(data.get("r_wrist_offset"))
            self._l_wrist_offset = q(data.get("l_wrist_offset"))
            head_y = data.get("calib_head_y")
            if head_y is not None:
                self._calib_head_y = float(head_y)
            ok = (self._r_shoulder_off is not None
                  and self._l_shoulder_off is not None)
            if ok:
                self._ik_enabled = True
                print(f"[avatar_xr_control] calibration loaded ← {self._CALIB_SAVE_PATH}")
            return ok
        except Exception as e:
            print(f"[avatar_xr_control] calibration load failed: {e}")
            return False

    def _apply_default_calibration(self):
        """Sensible defaults so the avatar tracks WITHOUT any calibration: anchor
        each shoulder at the avatar's own head→shoulder offset (average human
        proportions), reach scale 1.0, neutral (identity) wrist offset. An optional
        one-time T-pose calibration then refines these and persists them."""
        sk = self._skel
        if sk is None:
            return
        head = _vec3f(sk.head_rest_world)
        if self._r_shoulder_off is None:
            self._r_shoulder_off = _sub(sk.r_shoulder_pos, head)
        if self._l_shoulder_off is None:
            self._l_shoulder_off = _sub(sk.l_shoulder_pos, head)
        self._ik_enabled = True
        print("[avatar_xr_control] no saved calibration — using default anchors "
              "(optional T-pose calibrate will refine + persist them)")

    _CALIB_INSTRUCTIONS = [
        "Step 1: Arms straight out to your sides",
        "Step 2: Palms facing DOWN, fingers forward",
        "Step 3: FREEZE - do not move",
        "Step 3: FREEZE - do not move",
        "Step 3: FREEZE - do not move",
        "Step 3: FREEZE - do not move",
        "Step 3: FREEZE - do not move",
        "Step 3: FREEZE - do not move",
        "Capturing in 2...",
        "Capturing in 1...",
    ]

    async def _calibrate_countdown(self, seconds: int):
        for i in range(seconds, 0, -1):
            instr = self._CALIB_INSTRUCTIONS[seconds - i]
            self._calib_lbl.text  = f"{i}s - {instr}"
            self._calib_lbl.style = {"font_size": 11, "color": 0xFFFFAA33}
            await asyncio.sleep(1)
        self._do_calibrate()

    def _do_calibrate(self):
        sk = self._skel
        if sk is None:
            self._calib_lbl.text  = "No skeleton"
            self._calib_lbl.style = {"font_size": 11, "color": 0xFF4444FF}
            return

        lines = ["=== Hand Calibration Debug ===", ""]
        calibrated = []

        def fmtv(v):
            return f"({v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f})"

        for dev, is_right in ((self._right_dev, True), (self._left_dev, False)):
            side = "R" if is_right else "L"
            if dev is None:
                lines.append(f"[{side}] device not found")
                continue
            wrist_m = _get_pose(dev)
            if wrist_m is None:
                lines.append(f"[{side}] no pose")
                continue
            q_tpose     = wrist_m.ExtractRotationQuat()
            q_tpose_f   = Gf.Quatf(
                float(q_tpose.GetReal()),
                float(q_tpose.GetImaginary()[0]),
                float(q_tpose.GetImaginary()[1]),
                float(q_tpose.GetImaginary()[2]))
            # Same frame change as the runtime path in _apply_hand_tracking:
            # the stage-space controller quat is brought into the avatar's
            # local frame by undoing the current root yaw (camera follow).
            if self._root_yaw:
                q_tpose_f = _quat_mul(_quat_conj(_yaw_quatf(self._root_yaw)), q_tpose_f)
            fore_rest   = sk.r_fore_world_q_rest if is_right else sk.l_fore_world_q_rest
            local_tpose = _quat_mul(_quat_conj(fore_rest), q_tpose_f)
            rest_local  = sk.r_hand_local_rest_q if is_right else sk.l_hand_local_rest_q
            # RIGHT-multiply convention offset C: runtime does cal = local_q · C, so
            # C must satisfy local_tpose · C = rest_local at the T-pose ⇒
            # C = conj(local_tpose) · rest_local. (Was rest_local·conj(local_tpose),
            # a LEFT-multiply offset — see _apply_hand_tracking.)
            offset      = _quat_mul(_quat_conj(local_tpose), rest_local)
            if is_right:
                self._r_wrist_offset = offset
            else:
                self._l_wrist_offset = offset
            calibrated.append(side)

            chk = _quat_mul(local_tpose, offset)   # right-multiply ⇒ ≈ rest_local

            def fmt(q):
                i = q.GetImaginary()
                return f"w{q.GetReal():+.4f} x{i[0]:+.4f} y{i[1]:+.4f} z{i[2]:+.4f}"

            # Capture XR wrist world pos in T-pose — must be raw so it matches the
            # raw positions used by _apply_arm_ik each frame.
            _raw_wrist = _get_pose_raw(dev)
            xr_wrist_pos = _vec3f(_raw_wrist.ExtractTranslation()
                                  if _raw_wrist is not None
                                  else wrist_m.ExtractTranslation())
            if is_right:
                self._r_wrist_xr_tpose = xr_wrist_pos
            else:
                self._l_wrist_xr_tpose = xr_wrist_pos

            hand_idx = sk.r_hand_idx if is_right else sk.l_hand_idx
            av_wrist_pos = sk._rest_world[hand_idx].ExtractTranslation()

            lines += [
                f"[{side}]",
                f"  offset      (computed)               = {fmt(offset)}",
                f"  verify: matches rest_local?          = {'YES' if abs(chk.GetReal() - rest_local.GetReal()) < 0.01 else 'NO — mismatch!'}",
                f"  XR wrist world pos    = {fmtv(xr_wrist_pos)}",
                f"  avatar wrist rest pos = {fmtv(av_wrist_pos)}",
                "",
            ]

        # Compute IK scale from ARM LENGTH, not wrist span.
        # real_arm_len ≈ (xr_wrist_span − real_shoulder_width) / 2
        # Use a FIXED real-world shoulder width (~0.38 m): xr_span is in real
        # metres, so subtracting the avatar's shoulder span (different space)
        # was the bug that produced a tiny arm length and huge scale.
        REAL_SHOULDER_W = 0.38
        if self._r_wrist_xr_tpose is not None and self._l_wrist_xr_tpose is not None:
            xr_span    = _len(_sub(self._r_wrist_xr_tpose, self._l_wrist_xr_tpose))
            av_arm_len = sk.r_upperarm_len + sk.r_forearm_len
            real_arm_len = max(0.20, (xr_span - REAL_SHOULDER_W) / 2.0)
            self._real_arm_len = real_arm_len
            self._ik_scale = av_arm_len / real_arm_len
            self._ik_enabled = True

            # Capture the HEAD pose first — the shoulder derivation needs the
            # calibration yaw. The T-pose arms point along the USER'S own
            # right/left axis, which in raw space is R_y(calib_yaw)·(±X), NOT
            # raw ±X. (Hardcoded ±X was the bug that produced ~1.5 m shoulder
            # anchors whenever calibration happened while not facing raw -Z,
            # making the arms dead/erratic.)
            head_m = _get_pose_raw(self._head_dev)   # raw: matches IK frame
            if head_m is not None:
                head_pos = _vec3f(head_m.ExtractTranslation())
                self._calib_head_yaw = _yaw_of(head_m)
                cy = self._calib_head_yaw
                # Standing head height (VIRTUAL-WORLD Y — same frame the crouch
                # signal reads each frame) for crouch/sit thresholds. Re-calibrating
                # rescales every crouch/sit threshold automatically.
                hv = _get_pose(self._head_dev)
                if hv is not None:
                    self._calib_head_y = float(hv.ExtractTranslation()[1])
                r_shoulder_xr = _sub(self._r_wrist_xr_tpose,
                                     _rot_y(Gf.Vec3f(+real_arm_len, 0.0, 0.0), cy))
                l_shoulder_xr = _sub(self._l_wrist_xr_tpose,
                                     _rot_y(Gf.Vec3f(-real_arm_len, 0.0, 0.0), cy))
                # Store the head→shoulder offsets YAW-NEUTRAL (user body
                # frame): each frame they are re-rotated by the live body yaw,
                # so the anchors follow both head position AND turning.
                self._r_shoulder_off = _rot_y(_sub(r_shoulder_xr, head_pos), -cy)
                self._l_shoulder_off = _rot_y(_sub(l_shoulder_xr, head_pos), -cy)
                self._symmetrize_shoulder_offsets()   # remove T-pose L/R skew
                self._body_yaw = cy   # seed the live body-yaw tracker
                lines.append(f"head_pos = {fmtv(head_pos)}")
                lines.append(f"calib_head_yaw = {math.degrees(cy):+.1f}deg")
                lines.append(f"r_shoulder_off (body frame) = {fmtv(self._r_shoulder_off)}")
                lines.append(f"l_shoulder_off (body frame) = {fmtv(self._l_shoulder_off)}")
            else:
                lines.append("WARNING: no head pose at calibration — IK disabled")
                self._ik_enabled = False

            lines.append(f"xr_span={xr_span:.4f}  real_shoulder_w={REAL_SHOULDER_W:.4f}")
            lines.append(f"real_arm_len={real_arm_len:.4f}  av_arm_len={av_arm_len:.4f}")
            lines.append(f"IK scale (arm-length based) = {self._ik_scale:.4f}")

        lines.append("calibrated: " + ("+".join(calibrated) if calibrated else "NONE"))

        with open(self._CALIB_DEBUG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        if calibrated:
            self._save_calibration()   # persist so it's not needed next session
            self._calib_lbl.text  = "calibrated: " + "+".join(calibrated) + " (saved)"
            self._calib_lbl.style = {"font_size": 11, "color": 0xFF44FF44}
        else:
            self._calib_lbl.text  = "no XR devices - start tracking first"
            self._calib_lbl.style = {"font_size": 11, "color": 0xFF4444FF}

    # ------------------------------------------------------------------
    # Status labels
    # ------------------------------------------------------------------

    def _set_status(self, text, error=False):
        self._status_lbl.text  = text
        self._status_lbl.style = {"font_size": 11, "color": 0xFF4444FF if error else 0xFF44FF44}

    def _set_lbl(self, attr, text):
        """Set a UI label's text if the label exists. The advanced-tuning
        section was removed from the UI, but its handler methods stay callable
        programmatically — their label updates must no-op, not crash."""
        lbl = getattr(self, attr, None)
        if lbl is not None:
            lbl.text = text

    def _set_track(self, text, color=0xFFCCCCCC):
        self._track_lbl.text  = text
        self._track_lbl.style = {"font_size": 11, "color": color}

    # ------------------------------------------------------------------
    # Tracking
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Combined Start XR / Stop XR — one button each, sequenced with a 10s gap.
    # ------------------------------------------------------------------

    _XR_SEQ_DELAY = 10   # seconds between stream and tracking steps

    def _start_xr_combined(self):
        asyncio.ensure_future(self._run_start_xr())

    def _stop_xr_combined(self):
        asyncio.ensure_future(self._run_stop_xr())

    def _set_xr_buttons_enabled(self, enabled: bool):
        for btn in (self._startxr_btn, self._stopxr_btn):
            try:
                btn.enabled = enabled
            except Exception:
                pass

    async def _run_start_xr(self):
        if self._xr_seq_busy:
            return
        self._xr_seq_busy = True
        self._set_xr_buttons_enabled(False)
        try:
            self._start_xr_session()
            for i in range(self._XR_SEQ_DELAY, 0, -1):
                self._xr_session_lbl.text  = f"Stream aktiv - Tracking startet in {i}s..."
                self._xr_session_lbl.style = {"font_size": 11, "color": 0xFFFFAA33}
                await asyncio.sleep(1)
            self._start_tracking()
        finally:
            self._xr_seq_busy = False
            self._set_xr_buttons_enabled(True)

    async def _run_stop_xr(self):
        if self._xr_seq_busy:
            return
        self._xr_seq_busy = True
        self._set_xr_buttons_enabled(False)
        try:
            self._stop_tracking()
            for i in range(self._XR_SEQ_DELAY, 0, -1):
                self._xr_session_lbl.text  = f"Tracking gestoppt - Stream stoppt in {i}s..."
                self._xr_session_lbl.style = {"font_size": 11, "color": 0xFFFFAA33}
                await asyncio.sleep(1)
            self._stop_xr_session()
        finally:
            self._xr_seq_busy = False
            self._set_xr_buttons_enabled(True)

    def _start_xr_session(self):
        try:
            from omni.kit.xr.core import XRCore
            xr = XRCore.get_singleton()
            all_profiles = xr.get_profile_name_list()
            profiles = [str(p) for p in all_profiles if str(p) not in ("None", "", "null")]
            print(f"[avatar_xr_control] Available XR profiles: {profiles}")
            if profiles:
                # Prefer the VR profile: Isaac Sim lists ['ar', 'vr'] and the
                # AR profile fails xrCreateInstance without an AR runtime.
                name = next((p for p in profiles if p.lower() == "vr"), profiles[0])
                XRCore.request_enable_profile(name)
                self._xr_session_lbl.text  = f"Starting: {name}"
                self._xr_session_lbl.style = {"font_size": 11, "color": 0xFFFFAA33}
                print(f"[avatar_xr_control] Requested XR profile: {name}")
                asyncio.ensure_future(self._set_camera_after_xr_start())
            else:
                self._xr_session_lbl.text  = "No profiles found"
                self._xr_session_lbl.style = {"font_size": 11, "color": 0xFF4444FF}
        except Exception as e:
            self._xr_session_lbl.text  = f"Error: {e}"
            self._xr_session_lbl.style = {"font_size": 11, "color": 0xFF4444FF}

    def _stop_xr_session(self):
        try:
            from omni.kit.xr.core import XRCore
            xr = XRCore.get_singleton()
            xr.request_disable_profile()
            self._xr_session_lbl.text  = "stopped"
            self._xr_session_lbl.style = {"font_size": 11, "color": 0xFF888888}
        except Exception as e:
            self._xr_session_lbl.text  = f"Error: {e}"
            self._xr_session_lbl.style = {"font_size": 11, "color": 0xFF4444FF}

    async def _set_camera_after_xr_start(self):
        # Wait for XR session to initialize, then parent camera to head bone
        for _ in range(100):
            await asyncio.sleep(0.1)
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                continue
            cam_prim = stage.GetPrimAtPath(self._xr_cam_path)
            if not cam_prim.IsValid():
                continue
            self._parent_camera_to_head()
            self._disable_kit_locomotion()
            self._xr_session_lbl.text  = "active - first person"
            self._xr_session_lbl.style = {"font_size": 11, "color": 0xFF44FF44}
            return

    def _disable_kit_locomotion(self):
        """Disable Kit's built-in XR thumbstick locomotion so it doesn't double up
        with _apply_stick_locomotion().  All attempts are best-effort and guarded."""
        # carb settings approach (works on many Kit versions)
        try:
            import carb
            s = carb.settings.get_settings()
            for path in ("/xr/locomotion/enabled",
                         "/persistent/xr/locomotion/enabled"):
                try:
                    s.set(path, False)
                except Exception:
                    pass
        except Exception:
            pass
        # Profile component approach (Kit 104+)
        try:
            from omni.kit.xr.core import XRCore
            profile = XRCore.get_singleton().get_active_profile()
            if profile is not None:
                for attr in ("locomotion", "teleport", "thumbstick_locomotion"):
                    comp = getattr(profile, attr, None)
                    if comp is not None:
                        try:
                            comp.set_enabled(False)
                        except Exception:
                            pass
        except Exception:
            pass
        print("[avatar_xr_control] Kit locomotion disable attempted")

    def _parent_camera_to_head(self):
        """Teleport the XR view to the avatar's EYE position (full immersion).
        The head mesh is hidden via head chop while tracking, so eye level
        doesn't put the camera inside visible geometry. Looks along the
        avatar's current forward (root yaw)."""
        sk = self._skel
        if sk is None:
            return
        try:
            from omni.kit.xr.core import XRCore
            xr = XRCore.get_singleton()
            head_prim_path = self._skel_path + "/" + JOINT_MAP["head"]
            # Eye point in the cached rest frame: slightly above the head-joint
            # origin and forward toward the face (avatar forward = -Z), then
            # mapped through the root delta (the root may have moved/yawed).
            t = sk._rest_world[sk.head_idx].ExtractTranslation()
            eye = self._rest_to_world(Gf.Vec3f(
                float(t[0]),
                float(t[1]) + self._eye_up,
                float(t[2]) - self._eye_fwd,
            ))
            yaw = self._root_yaw or 0.0
            view_pose = Gf.Matrix4d().SetRotate(
                Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(yaw)))
            view_pose.SetTranslateOnly(Gf.Vec3d(eye[0], eye[1], eye[2]))
            xr.schedule_teleport_to_view(head_prim_path, view_pose)
            print(f"[avatar_xr_control] Teleported view to eyes: "
                  f"({eye[0]:.3f}, {eye[1]:.3f}, {eye[2]:.3f}) yaw={math.degrees(yaw):.1f}deg")
        except Exception as e:
            print(f"[avatar_xr_control] Failed to teleport to eyes: {e}")


    def _start_tracking(self):
        if self._skel is None:
            self._set_track("No skeleton - open a stage first", 0xFF4444FF)
            return
        ok, msg = self._init_xr()
        self._set_track(msg, 0xFF44FF44 if ok else 0xFF4444FF)
        # Allow tracking even if XR init fails when we're driving the pipeline
        # ourselves — simulation OR replay of a recording (no headset needed).
        if not ok and not self._sim_enabled and not self._play_enabled:
            return
        # First person: hide the head mesh while tracking (restored on stop).
        if self._hide_head_on:
            self._skel.set_head_hidden(True, self._head_chop_fallback)
        # Static 5° forward waist tilt (natural standing posture). Always
        # derived from the rest rotation — idempotent across restarts.
        fwd5 = Gf.Quatf(math.cos(math.radians(5)), math.sin(math.radians(5)), 0, 0)
        self._skel.write_joint_rotation(
            self._skel.waist_idx,
            _quat_mul(fwd5, self._skel.waist_local_rest_q))
        self._xr_active = True
        if self._tracking_task and not self._tracking_task.done():
            self._tracking_task.cancel()
        self._tracking_task = asyncio.ensure_future(self._tracking_loop())

    def _stop_tracking(self):
        self._xr_active = False
        if self._tracking_task:
            self._tracking_task.cancel()
            self._tracking_task = None
        # Drop elbow-swivel history and shoulder yaw so a later restart doesn't
        # smooth from stale poses.
        self._elbow_bend = {True: None, False: None}
        self._shoulder_yaw = None
        # Drop gait/crouch anchors so a restart doesn't lurch from a stale hip pose.
        self._gait_root_prev = None
        self._gait_speed = 0.0
        self._pelvis_drop = 0.0
        self._crouch_sit.reset()
        for s in self._foot_state.values():
            s["world"] = s["from"] = s["plant"] = s["region"] = None
        if self._coll_debug_on:
            try:
                self._hide_coll_debug(omni.usd.get_context().get_stage())
            except Exception:
                pass
        if self._skel is not None:
            try:
                self._skel.set_head_hidden(False, self._head_chop_fallback)
            except Exception:
                pass
        self._set_track("stopped", 0xFF888888)

    async def _tracking_loop(self):
        try:
            while self._xr_active:
                sk_frame = self._skel   # flush the same object the frame wrote
                try:
                    # Measure the real frame time so the exponential eases below
                    # stay correctly paced at any frame rate (clamped to ignore the
                    # first frame and big hitches).
                    now = time.monotonic()
                    if self._last_frame_t is not None:
                        self._frame_dt = max(0.005, min(0.2, now - self._last_frame_t))
                    self._last_frame_t = now
                    # Batch this frame's joint writes: flushed once per frame in
                    # the finally below instead of a full-array USD Set per joint.
                    if sk_frame is not None:
                        sk_frame.set_deferred(True)
                    # Replay: swap in PlaybackDevices for this frame BEFORE anything
                    # reads them, so the whole pipeline runs off the recording.
                    self._advance_playback()
                    try:
                        self._apply_camera_follow()
                    except Exception:
                        pass  # camera follow must never break tracking
                    self._apply_head()
                    self._apply_upper_body()
                    self._apply_hand_tracking()
                    # Record: snapshot the (real) device inputs this frame used.
                    self._record_frame()
                    try:
                        self._apply_stick_locomotion()
                    except Exception:
                        pass  # locomotion must never break tracking
                    try:
                        self._apply_legs()
                    except Exception:
                        pass  # procedural walk must never break tracking
                    if self._coll_debug_on:
                        try:
                            self._update_coll_debug(omni.usd.get_context().get_stage())
                        except Exception:
                            pass  # debug viz must never break tracking
                except Exception as e:
                    self._set_track(f"Error: {e}", 0xFF4444FF)
                    self._xr_active = False
                    break
                finally:
                    if sk_frame is not None:
                        sk_frame.flush()
                await asyncio.sleep(0.016)
        except asyncio.CancelledError:
            pass
        finally:
            # Restore immediate-write semantics (and push any pending writes)
            # so UI callbacks outside the loop keep working unbatched.
            sk = self._skel
            if sk is not None:
                sk.set_deferred(False)

    # ------------------------------------------------------------------
    # XR capture / replay
    # ------------------------------------------------------------------

    @staticmethod
    def _ser_quat(q):
        if q is None:
            return None
        im = q.GetImaginary()
        return [float(q.GetReal()), float(im[0]), float(im[1]), float(im[2])]

    @staticmethod
    def _ser_vec(v):
        return None if v is None else [float(v[0]), float(v[1]), float(v[2])]

    def _calib_snapshot(self):
        """Serialise the calibration that defines the hand→avatar mapping, so a
        replay reproduces the same motion. Tuning params are NOT captured — they
        stay live so they can be adjusted while replaying."""
        return {
            "r_wrist_offset":   self._ser_quat(self._r_wrist_offset),
            "l_wrist_offset":   self._ser_quat(self._l_wrist_offset),
            "r_shoulder_off":   self._ser_vec(self._r_shoulder_off),
            "l_shoulder_off":   self._ser_vec(self._l_shoulder_off),
            "r_wrist_xr_tpose": self._ser_vec(self._r_wrist_xr_tpose),
            "l_wrist_xr_tpose": self._ser_vec(self._l_wrist_xr_tpose),
            "ik_scale":         float(self._ik_scale),
            "real_arm_len":     float(self._real_arm_len),
            "calib_head_y":     (None if self._calib_head_y is None
                                 else float(self._calib_head_y)),
        }

    def _restore_calib(self, meta):
        def q(v):
            return None if v is None else Gf.Quatf(v[0], v[1], v[2], v[3])
        def vec(v):
            return None if v is None else Gf.Vec3f(v[0], v[1], v[2])
        self._r_wrist_offset   = q(meta.get("r_wrist_offset"))
        self._l_wrist_offset   = q(meta.get("l_wrist_offset"))
        self._r_shoulder_off   = vec(meta.get("r_shoulder_off"))
        self._l_shoulder_off   = vec(meta.get("l_shoulder_off"))
        self._symmetrize_shoulder_offsets()   # so replay/metrics use a clean anchor
        self._r_wrist_xr_tpose = vec(meta.get("r_wrist_xr_tpose"))
        self._l_wrist_xr_tpose = vec(meta.get("l_wrist_xr_tpose"))
        self._ik_scale         = float(meta.get("ik_scale", 1.0))
        self._real_arm_len     = float(meta.get("real_arm_len", 1.0))
        self._calib_head_y     = meta.get("calib_head_y")   # standing head Y (crouch/sit)

    def _snap_device(self, dev, fingers, gestures):
        """Capture one device's raw outputs this frame as plain JSON-able data."""
        if dev is None:
            return None
        out = {}
        try:
            m = dev.get_virtual_world_pose()
            out["vw"] = _mat_to_list(m) if m is not None else None
        except Exception:
            out["vw"] = None
        try:
            poses = dev.get_all_raw_poses()
            desc = poses.get("") if poses else None
            if desc is not None and desc.validity_flags != 0:
                out["raw"] = _mat_to_list(Gf.Matrix4d(desc.pose_matrix))
                out["raw_valid"] = int(desc.validity_flags)
            else:
                out["raw"] = None
                out["raw_valid"] = 0
        except Exception:
            out["raw"] = None
            out["raw_valid"] = 0
        if fingers:
            fd = {}
            # Probe one finger first; if optical hand tracking isn't providing data
            # (controllers held), skip the other 14 reads — saves ~15 device round-
            # trips per hand each frame, a real cost at the recorder's call volume.
            try:
                first = dev.get_virtual_world_pose(FINGER_POSE_MAP[0][0])
            except Exception:
                first = None
            if first is not None:
                fd[FINGER_POSE_MAP[0][0]] = _mat_to_list(first)
                for pose_name, _rk, _lk in FINGER_POSE_MAP[1:]:
                    try:
                        m = dev.get_virtual_world_pose(pose_name)
                        if m is not None:
                            fd[pose_name] = _mat_to_list(m)
                    except Exception:
                        pass
            out["fingers"] = fd
            try:
                out["src"] = dev.get_hand_tracking_data_source()
            except Exception:
                out["src"] = None
        if gestures:
            g = {}
            for comp in ("trigger", "squeeze", "grip", "select", "pinch"):
                for gest in ("value", "click", "force", "touch"):
                    try:
                        if dev.has_input_gesture(comp, gest):
                            g[f"{comp}.{gest}"] = round(float(
                                dev.get_input_gesture_value(comp, gest)), 4)
                    except Exception:
                        pass
            for comp in _STICK_COMPONENTS:
                for ax in ("x", "y"):
                    try:
                        if dev.has_input_gesture(comp, ax):
                            g[f"{comp}.{ax}"] = round(float(
                                dev.get_input_gesture_value(comp, ax)), 4)
                    except Exception:
                        pass
            out["gestures"] = g
        return out

    def _record_frame(self):
        if not self._rec_enabled or self._rec_file is None:
            return
        try:
            frame = {
                "t":     round(time.monotonic() - self._rec_t0, 4),
                "head":  self._snap_device(self._head_dev, False, False),
                "left":  self._snap_device(self._left_dev, True, True),
                "right": self._snap_device(self._right_dev, True, True),
            }
            self._rec_file.write(json.dumps(frame) + "\n")
            self._rec_count += 1
            if self._rec_count % 60 == 0:
                self._rec_file.flush()
                self._update_capture_lbl()
        except Exception:
            pass  # recording must never break tracking

    def _start_recording(self):
        if self._play_enabled:
            self._set_track("stop replay before recording", 0xFFFFAA33)
            return
        if not self._xr_active:
            self._set_track("start XR before recording a live demo", 0xFFFFAA33)
            return
        try:
            self._rec_file = open(self._rec_path, "w", encoding="utf-8")
            self._rec_file.write(json.dumps({"meta": self._calib_snapshot()}) + "\n")
        except Exception as e:
            self._set_track(f"record open failed: {e}", 0xFF4444FF)
            self._rec_file = None
            return
        self._rec_t0 = time.monotonic()
        self._rec_count = 0
        self._rec_enabled = True
        self._update_capture_lbl()

    def _stop_recording(self):
        self._rec_enabled = False
        if self._rec_file is not None:
            try:
                self._rec_file.flush()
                self._rec_file.close()
            except Exception:
                pass
            self._rec_file = None
        self._set_track(f"recorded {self._rec_count} frames -> {self._rec_path}",
                        0xFF44FF44)
        self._update_capture_lbl()

    def _restore_live_calib(self):
        """Undo the recording's calibration snapshot (applied by _load_recording)
        so a live session after a replay runs on the user's own calibration."""
        if self._live_calib is not None:
            self._restore_calib(self._live_calib)
            self._live_calib = None

    def _load_recording(self):
        self._play_frames = []
        # The recording's meta line overwrites the calibration so the replay
        # reproduces the recorded mapping — stash the LIVE calibration first
        # (once; a replay-of-a-replay must not stash the recording's own values).
        if self._live_calib is None:
            self._live_calib = self._calib_snapshot()
        try:
            with open(self._rec_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    obj = json.loads(line)
                    if "meta" in obj:
                        self._restore_calib(obj["meta"])
                        continue
                    self._play_frames.append(obj)
        except Exception as e:
            self._set_track(f"load failed: {e}", 0xFF4444FF)
            return False
        return len(self._play_frames) > 0

    def _start_replay(self):
        if self._rec_enabled:
            self._set_track("stop recording before replay", 0xFFFFAA33)
            return
        if not self._load_recording():
            self._set_track("no frames in recording to replay", 0xFFFFAA33)
            return
        # Save the real devices so they can be restored when replay stops.
        self._play_saved_devs = (self._head_dev, self._left_dev, self._right_dev)
        self._play_idx = 0
        self._play_prev_t = None
        # Clear spike-rejection history so the first replayed frame isn't compared
        # against a stale live position (which would false-reject it).
        self._last_pos = {}
        self._spike_hold = {}
        self._play_enabled = True
        self._ik_enabled = True
        # Run the tracking loop without a real XR session; _advance_playback swaps
        # in PlaybackDevices each frame. Remember if WE started it so Stop can end it.
        self._play_started_loop = not self._xr_active
        if not self._xr_active:
            self._start_tracking()
        self._set_track(f"replaying {len(self._play_frames)} frames", 0xFF44FF44)
        self._update_capture_lbl()

    def _stop_replay(self):
        self._play_enabled = False
        if self._play_saved_devs is not None:
            self._head_dev, self._left_dev, self._right_dev = self._play_saved_devs
            self._play_saved_devs = None
        self._restore_live_calib()
        # If replay started the loop for offline playback, stop it so it doesn't
        # keep spinning on the restored (null) devices.
        if self._play_started_loop:
            self._play_started_loop = False
            self._stop_tracking()
        self._update_capture_lbl()

    def _toggle_play_loop(self):
        self._play_loop = not self._play_loop
        self._update_capture_lbl()

    def _advance_playback(self):
        if not self._play_enabled or not self._play_frames:
            return
        fr = self._play_frames[self._play_idx]
        # Pace the eases / spike rejection by the RECORDED inter-frame dt, not the
        # wall-clock tick. Replay advances one recorded frame per tick (~7x real
        # time), so a wall-clock dt would make every recorded step look like a
        # spike and the despike would false-reject it (jittery "rearranging").
        t = fr.get("t", 0.0)
        if self._play_prev_t is not None:
            self._frame_dt = max(0.005, min(0.2, t - self._play_prev_t))
        self._play_prev_t = t
        self._head_dev  = _PlaybackDevice(fr.get("head"))
        self._left_dev  = _PlaybackDevice(fr.get("left"))
        self._right_dev = _PlaybackDevice(fr.get("right"))
        self._play_idx += 1
        if self._play_idx >= len(self._play_frames):
            if self._play_loop:
                self._play_idx = 0
                self._play_prev_t = None   # don't diff across the loop seam
            else:
                self._stop_replay()
        if self._play_idx % 30 == 0:
            self._update_capture_lbl()

    def _update_capture_lbl(self):
        lbl = getattr(self, "_capture_lbl", None)
        if lbl is None:
            return
        if self._rec_enabled:
            lbl.text = f"recording... {self._rec_count} frames"
        elif self._play_enabled:
            lbl.text = (f"replaying {self._play_idx}/{len(self._play_frames)}"
                        f"  (loop {'on' if self._play_loop else 'off'})")
        else:
            lbl.text = f"idle - {os.path.basename(self._rec_path)}"

    # ------------------------------------------------------------------
    # Pose application
    # ------------------------------------------------------------------

    # Limb half-thickness used for torso self-collision clearance (metres).
    _UPPER_R   = 0.05
    _FORE_R    = 0.045
    # Palm collider radius (metres) — the kinematic hand proxy that pushes objects.
    _PALM_R    = 0.045
    # Length the hand extends past the wrist along the forearm — the forearm
    # penetration test runs out to here so the HAND clears the body too.
    _HAND_EXT  = 0.08
    # Penetration this small (metres) is treated as touching, not clipping — a
    # deadband so arms resting near the body don't constantly re-pose/jitter.
    _PEN_TOL   = 0.015
    # Self-collision swivel selection / smoothing.
    _SWIVEL_RATE   = 10.0   # exponential approach rate toward the target (1/s).
                            # Was 25 (near-instant → snappy reposing near the body);
                            # 10 ≈ 100 ms settle, smooth without noticeable lag. The
                            # hysteresis in the swivel solvers means reposes are rare.
    _SWIVEL_W_CONT = 0.5    # cost weight: stay near the previous frame's swivel
    _SWIVEL_W_UP   = 10.0   # cost weight (per metre) against raising the elbow above
                            # the natural pole. Was 3.0 — too weak vs the continuity
                            # cost, so on forward below-head reaches (down elbow hits
                            # the torso) the swivel flipped the elbow UP and the
                            # continuity bias held it there. 10 makes it strongly
                            # prefer a down/out clearing angle instead.
    _SWIVEL_STEP   = math.radians(12)   # coarse sweep step
    _SWIVEL_MAX    = math.radians(160)  # max swivel magnitude searched

    def _torso_pen_point(self, p, limb_r):
        """Penetration depth (metres, ≥0) of point p into the elliptical torso
        capsule, inflated by the limb half-thickness limb_r. 0 = outside."""
        sk = self._skel
        tb, tt = sk.torso_bottom, sk.torso_top
        hx, hz = sk.torso_half_x, sk.torso_half_z
        axis = _sub(tt, tb)
        a_len2 = _dot(axis, axis)
        if a_len2 < 1e-9:
            return 0.0
        t = max(0.0, min(1.0, _dot(_sub(p, tb), axis) / a_len2))
        c = _add(tb, _scale(axis, t))
        dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
        rad = math.sqrt(dx * dx + dz * dz)          # horizontal radial distance
        if rad < 1e-6:
            r_eff = min(hx, hz)
        else:
            ux, uz = dx / rad, dz / rad             # ellipse radius in this dir
            r_eff = 1.0 / math.sqrt((ux / hx) ** 2 + (uz / hz) ** 2)
        dist = math.sqrt(rad * rad + dy * dy)       # dy ≠ 0 only past the caps
        return max(0.0, (r_eff + limb_r) - dist)

    def _seg_torso_pen(self, p, q, limb_r, t0=0.0, n=5):
        """Worst penetration of segment p→q (sampled from fraction t0 to 1) into
        the torso. t0 lets the shoulder-attached end of the upper arm be skipped
        so a limb anchored at the body surface isn't falsely flagged."""
        worst = 0.0
        d = _sub(q, p)
        for i in range(n + 1):
            f = t0 + (1.0 - t0) * (i / n)
            pen = self._torso_pen_point(_add(p, _scale(d, f)), limb_r)
            if pen > worst:
                worst = pen
        return worst

    def _push_out_torso(self, p, extra_r, margin=0.01):
        """If point p lies inside the torso capsule (inflated by extra_r), move it
        out to the surface (+margin) along the radial normal and return it; else
        return p unchanged. Used to keep the hand target on the body surface."""
        sk = self._skel
        if sk is None:
            return p
        tb, tt = sk.torso_bottom, sk.torso_top
        hx, hz = sk.torso_half_x, sk.torso_half_z
        axis = _sub(tt, tb)
        a_len2 = _dot(axis, axis)
        if a_len2 < 1e-9:
            return p
        t = max(0.0, min(1.0, _dot(_sub(p, tb), axis) / a_len2))
        c = _add(tb, _scale(axis, t))
        dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
        rad = math.sqrt(dx * dx + dz * dz)
        if rad < 1e-6:
            r_eff = min(hx, hz)
        else:
            ux, uz = dx / rad, dz / rad
            r_eff = 1.0 / math.sqrt((ux / hx) ** 2 + (uz / hz) ** 2)
        dist = math.sqrt(rad * rad + dy * dy)
        need = r_eff + extra_r + margin
        if dist >= need:
            return p
        if dist < 1e-6:
            # On the axis — push forward (avatar front) as a sane default.
            n = sk.fwd_dir
        else:
            inv = 1.0 / dist
            n = Gf.Vec3f(dx * inv, dy * inv, dz * inv)
        return _add(c, _scale(n, need))

    def _swivel_clear_torso(self, shoulder, wrist, base, h, u, v, is_right):
        """Pick the elbow swivel angle that keeps the arm out of the torso.

        2-bone IK has one free DOF: the elbow rides a circle around the
        shoulder→wrist axis (centre `base`, radius `h`, in-plane basis u/v).
        Swivel 0 (base + h·u) is the natural pole. If that makes the upper arm,
        forearm or hand pass through the torso capsule, search BOTH ways for the
        swivel that clears it, choosing by a small cost (smallest deviation,
        continuity with last frame, and a bias against raising the elbow). The
        chosen target is then eased toward over time so the elbow never pops when
        the clearing side flips or when contact starts/ends. If nothing clears
        (e.g. the hand is on the chest) the least-penetrating pose is targeted.
        The wrist never moves, so the hand stays on the controller and both bone
        lengths stay exact.
        """
        sk = self._skel
        natural = _add(base, _scale(u, h))
        if sk is None:
            return natural

        # Singularity guard: when the elbow circle radius h is very small (arm
        # nearly straight or folded), the perpendicular basis (u,v) becomes unstable.
        # Snapping to the natural pole (φ=0) is physically harmless since the radius
        # is too small for position to matter visually, and avoids basis-projection
        # noise that would otherwise create unnatural elbow jitter.
        if h < 0.03:
            self._elbow_bend[is_right] = None
            return natural

        def elbow_at(phi):
            return _add(base, _add(_scale(u, math.cos(phi)),
                                   _scale(v, math.sin(phi))))

        def pen_total(elbow):
            # Upper arm: skip the shoulder-anchored third (always near the body).
            up = self._seg_torso_pen(shoulder, elbow, self._UPPER_R, t0=0.35)
            # Forearm extended past the wrist by _HAND_EXT so the hand clears too.
            fdir = _sub(wrist, elbow)
            flen = _len(fdir)
            hand_end = (_add(wrist, _scale(fdir, self._HAND_EXT / flen))
                        if flen > 1e-5 else wrist)
            fo = self._seg_torso_pen(elbow, hand_end, self._FORE_R)
            return up + fo

        # Previous swivel expressed in the CURRENT basis → comparable angle.
        prev_bend = self._elbow_bend.get(is_right)
        phi_prev = (math.atan2(_dot(prev_bend, v), _dot(prev_bend, u))
                    if prev_bend is not None else 0.0)

        tol = self._PEN_TOL
        if pen_total(natural) <= tol:
            phi_target = 0.0
        else:
            step, max_off = self._SWIVEL_STEP, self._SWIVEL_MAX
            last_clear_free = {1.0: 0.0, -1.0: 0.0}  # last NON-clearing magnitude
            best_cost = None
            phi_target = 0.0
            least_pen, least_phi = pen_total(natural), 0.0
            m = step
            while m <= max_off + 1e-6:
                for sgn in (1.0, -1.0):
                    phi = sgn * m
                    p = pen_total(elbow_at(phi))
                    if p <= tol:
                        # Bisect between the last non-clearing magnitude and m to
                        # find the minimal clearing swivel on this side.
                        lo, hi = last_clear_free[sgn], m
                        for _ in range(4):
                            mid = 0.5 * (lo + hi)
                            if pen_total(elbow_at(sgn * mid)) <= tol:
                                hi = mid
                            else:
                                lo = mid
                        phi_c = sgn * hi
                        e_c = elbow_at(phi_c)
                        d_cont = abs((phi_c - phi_prev + math.pi)
                                     % (2 * math.pi) - math.pi)
                        cost = (abs(phi_c)
                                + self._SWIVEL_W_CONT * d_cont
                                + self._SWIVEL_W_UP * max(0.0, e_c[1] - natural[1]))
                        if best_cost is None or cost < best_cost:
                            best_cost, phi_target = cost, phi_c
                    else:
                        last_clear_free[sgn] = m
                        if p < least_pen:
                            least_pen, least_phi = p, phi
                if best_cost is not None:
                    break          # smallest clearing magnitude found
                m += step
            if best_cost is None:
                phi_target = least_phi   # nothing cleared → least penetrating

        # Ease the actual swivel toward the target (exponential, frame ~16 ms) so
        # changes are smooth; snap on the first frame (no history yet).
        if prev_bend is None:
            phi = phi_target
        else:
            d = (phi_target - phi_prev + math.pi) % (2 * math.pi) - math.pi
            phi = phi_prev + d * (1.0 - math.exp(-self._SWIVEL_RATE * self._frame_dt))

        bend = _add(_scale(u, math.cos(phi)), _scale(v, math.sin(phi)))
        self._elbow_bend[is_right] = bend
        return _add(base, _scale(bend, h))

    # ------------------------------------------------------------------
    # Collision-volume debug visualization
    # ------------------------------------------------------------------
    # Marker prims drawn at the SELF-COLLISION test geometry so it can be checked
    # against the rendered mesh: red spheres trace the torso capsule (radius =
    # torso_half_x, the lateral extent), small green spheres mark the shoulder
    # joints, blue spheres mark the live elbow/wrist test points (radius = the
    # limb half-thickness used in the test). All in live world space, so they ride
    # the camera-follow transform with the body.

    _COLL_DEBUG_ROOT = "/World/_xr_collision_debug"

    def _coll_marker(self, stage, name, world_pos, scale, color):
        """Create/position one unit-sphere marker at world_pos. `scale` is a
        float (uniform) or a 3-sequence (ellipsoid: x,y,z half-extents)."""
        try:
            sx, sy, sz = float(scale[0]), float(scale[1]), float(scale[2])
        except (TypeError, IndexError):
            sx = sy = sz = float(scale)
        path = self._COLL_DEBUG_ROOT + "/" + name
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            sphere = UsdGeom.Sphere.Define(stage, path)
            sphere.CreateRadiusAttr().Set(1.0)   # unit sphere; size via scale op
            sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
            sphere.CreateDisplayOpacityAttr().Set([0.35])
            prim = sphere.GetPrim()
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp()
            xf.AddScaleOp()
        UsdGeom.Imageable(prim).MakeVisible()
        t_op = s_op = None
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                t_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
                s_op = op
        if t_op:
            t_op.Set(Gf.Vec3d(float(world_pos[0]), float(world_pos[1]),
                              float(world_pos[2])))
        if s_op:
            s_op.Set(Gf.Vec3d(sx, sy, sz))

    def _update_coll_debug(self, stage):
        """Draw/refresh the IK target-position markers (Input Simulation only)."""
        if stage is None:
            return
        if not stage.GetPrimAtPath(self._COLL_DEBUG_ROOT).IsValid():
            UsdGeom.Scope.Define(stage, self._COLL_DEBUG_ROOT)
        # Simulated hand targets — yellow/gold circles showing where the IK
        # is trying to reach (Input Simulation only).
        for is_right, tag in ((True, "R"), (False, "L")):
            sim = self._sim_hand_world.get(is_right)
            if sim is not None:
                self._coll_marker(stage, f"sim_target_{tag}", sim, 0.04,
                                  (0.95, 0.85, 0.3))

    def _hide_coll_debug(self, stage):
        if stage is None:
            return
        prim = stage.GetPrimAtPath(self._COLL_DEBUG_ROOT)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()

    def _toggle_coll_debug(self):
        self._coll_debug_on = not self._coll_debug_on
        self._set_lbl("_coll_lbl",
            f"collision shapes: {'ON' if self._coll_debug_on else 'off'}")
        stage = omni.usd.get_context().get_stage()
        if self._coll_debug_on:
            self._update_coll_debug(stage)
        else:
            self._hide_coll_debug(stage)

    # --- Torso volume tuning ---
    def _update_torso_lbl(self):
        self._set_lbl("_torso_lbl",
            f"torso fwd {self._torso_fwd:+.02f}  width {self._torso_half_x:.02f}"
            f"  depth {self._torso_half_z:.02f} m")

    def _apply_torso_change(self):
        self._rebuild_torso()
        self._update_torso_lbl()
        if self._coll_debug_on:
            try:
                self._update_coll_debug(omni.usd.get_context().get_stage())
            except Exception:
                pass

    def _torso_fwd_up(self):
        self._torso_fwd = min(0.30, self._torso_fwd + 0.01)
        self._apply_torso_change()

    def _torso_fwd_dn(self):
        self._torso_fwd = max(-0.30, self._torso_fwd - 0.01)
        self._apply_torso_change()

    def _torso_wide_up(self):
        self._torso_half_x = min(0.30, self._torso_half_x + 0.01)
        self._apply_torso_change()

    def _torso_wide_dn(self):
        self._torso_half_x = max(0.05, self._torso_half_x - 0.01)
        self._apply_torso_change()

    def _torso_deep_up(self):
        self._torso_half_z = min(0.30, self._torso_half_z + 0.01)
        self._apply_torso_change()

    def _torso_deep_dn(self):
        self._torso_half_z = max(0.05, self._torso_half_z - 0.01)
        self._apply_torso_change()

    def _update_body_push_lbl(self):
        self._set_lbl("_body_push_lbl",
            f"hand body-collision: {'ON' if self._body_push else 'off'}")

    def _toggle_body_push(self):
        self._body_push = not self._body_push
        self._update_body_push_lbl()

    def _update_phys_lbl(self):
        if not _PHYS_AVAILABLE:
            self._set_lbl("_phys_lbl",
                          "physics collision: UNAVAILABLE (omni.physx not loaded)")
            return
        env = "env+self" if self._phys_env else "self only"
        sh = ", shoulder-push" if self._phys_shoulder_push else ""
        state = (f"ON ({env}){sh}, probe r={self._phys_probe_r * 100:.0f}mm"
                 if self._phys_collision else "off")
        ready = "" if (not self._phys_collision or self._phys_scene_ready) \
                else " — scene not ready"
        self._set_lbl("_phys_lbl", f"physics collision: {state}{ready}")

    def _toggle_phys_collision(self):
        if not _PHYS_AVAILABLE:
            print("[avatar_xr_control] Physics toggle clicked but omni.physx is "
                  "NOT available (_PHYS_AVAILABLE=False) — nothing will happen.")
            self._update_phys_lbl()
            return
        self._phys_collision = not self._phys_collision
        print(f"[avatar_xr_control] Physics collision toggled "
              f"{'ON' if self._phys_collision else 'off'} "
              f"(scene_ready={self._phys_scene_ready}, skel={self._skel is not None})")
        if self._phys_collision:
            if not self._phys_scene_ready:
                # Build scene + proxies on first enable (runs a diagnostic query).
                self._phys_setup(omni.usd.get_context().get_stage())
            else:
                self._phys_start_sim()
            print(f"[avatar_xr_control] After enable: scene_ready="
                  f"{self._phys_scene_ready}, sim_running={self._phys_sim_running}")
        else:
            self._phys_stop_sim()
        self._update_phys_lbl()

    def _toggle_phys_env(self):
        self._phys_env = not self._phys_env
        self._update_phys_lbl()

    def _phys_probe_dn(self):
        self._phys_probe_r = max(0.0, self._phys_probe_r - 0.005)
        self._update_phys_lbl()

    def _phys_probe_up(self):
        self._phys_probe_r = min(0.10, self._phys_probe_r + 0.005)
        self._update_phys_lbl()

    def _toggle_phys_shoulder(self):
        self._phys_shoulder_push = not self._phys_shoulder_push
        self._update_phys_lbl()

    def _update_fore_roll_lbl(self):
        self._set_lbl("_fore_roll_lbl",
            f"wrist twist -> forearm: {self._fore_roll * 100:.0f}%")

    def _fore_roll_dn(self):
        self._fore_roll = max(0.0, self._fore_roll - 0.1)
        self._update_fore_roll_lbl()

    def _fore_roll_up(self):
        self._fore_roll = min(1.0, self._fore_roll + 0.1)
        self._update_fore_roll_lbl()

    def _update_elbow_roll_lbl(self):
        self._set_lbl("_elbow_roll_lbl",
            f"controller roll -> elbow: {self._elbow_roll_weight * 100:.0f}%")

    def _elbow_roll_dn(self):
        # Allow negative weights so the swing direction can be inverted in-headset
        # if palm-up/palm-down map to the opposite elbow motion on this rig.
        self._elbow_roll_weight = max(-1.5, self._elbow_roll_weight - 0.1)
        self._update_elbow_roll_lbl()

    def _elbow_roll_up(self):
        self._elbow_roll_weight = min(1.5, self._elbow_roll_weight + 0.1)
        self._update_elbow_roll_lbl()

    def _update_finger_curl_lbl(self):
        state = "ON" if self._finger_curl_on else "off"
        self._set_lbl("_finger_curl_lbl",
            f"finger curl (controllers): {state} @ {self._finger_curl_deg:.0f}deg")

    def _toggle_finger_curl(self):
        self._finger_curl_on = not self._finger_curl_on
        self._update_finger_curl_lbl()

    def _finger_curl_dn(self):
        self._finger_curl_deg = max(0.0, self._finger_curl_deg - 10.0)
        self._update_finger_curl_lbl()

    def _finger_curl_up(self):
        self._finger_curl_deg = min(110.0, self._finger_curl_deg + 10.0)
        self._update_finger_curl_lbl()

    def _update_limits_lbl(self):
        self._set_lbl("_limits_lbl",
            f"joint limits: {'ON' if self._limits_on else 'off'}")

    def _toggle_limits(self):
        self._limits_on = not self._limits_on
        self._update_limits_lbl()

    def _clamp_dir_to_cone(self, d, is_right):
        """Clamp a unit arm direction (avatar-local: +X right, +Y up, -Z fwd) to
        the anatomical workspace. Caps backward reach (+Z) and how far the hand
        may cross the body midline, with the cross cap tighter the further back
        the arm points (you can reach across in FRONT, not BEHIND). Returns a
        unit vector."""
        x, y, z = float(d[0]), float(d[1]), float(d[2])
        band = self._lim_soft         # soft-compression band (0 → hard clamp)
        # 1) Backward cap (soft).
        z = _soft_cap_max(z, self._lim_back, band)
        # 2) Cross-midline cap, interpolated by how far back the arm points.
        t = max(0.0, min(1.0, z / self._lim_back)) if self._lim_back > 1e-3 else 0.0
        floor = self._lim_cross_f + (self._lim_cross_b - self._lim_cross_f) * t
        if is_right:                  # x must stay ≥ floor (soft)
            x = -_soft_cap_max(-x, -floor, band)
        else:                         # mirror: x must stay ≤ -floor (soft)
            x = _soft_cap_max(x, -floor, band)
        n = math.sqrt(x * x + y * y + z * z)
        if n < 1e-6:
            return d
        return Gf.Vec3f(x / n, y / n, z / n)

    def _apply_upper_body(self):
        # The OSC-driven waist lean was removed (the sideways tilt felt
        # unnatural); the waist keeps a static 5° forward tilt set once at
        # tracking start. Only the arm IK runs per frame.
        sk = self._skel
        if sk is None:
            return
        if self._ik_enabled:
            # Physics is on by default, but there is no UI toggle to build the
            # scene, so do it once lazily on the first tracked frame (needs the
            # skeleton + stage, both ready here). The guard ensures a single try.
            if (self._phys_collision and not self._phys_scene_ready
                    and not self._phys_setup_tried and _PHYS_AVAILABLE):
                self._phys_setup_tried = True
                try:
                    self._phys_setup(omni.usd.get_context().get_stage())
                except Exception as e:
                    print(f"[avatar_xr_control] phys auto-setup failed: {e}")
            # Refresh the body-collider proxies from last frame's solved poses
            # before solving, so the PhysX self/env queries see the avatar where
            # it currently is (a 1-frame lag on the opposite arm is harmless).
            if self._phys_collision and self._phys_scene_ready:
                try:
                    self._update_phys_proxies(omni.usd.get_context().get_stage())
                    self._phys_step(0.016)   # refresh the query scene
                except Exception:
                    pass  # physics correction must never break tracking
            self._apply_arm_ik(is_right=True)
            self._apply_arm_ik(is_right=False)

    def _symmetrize_shoulder_offsets(self):
        """Force the two shoulder anchors into a mirror pair. Human shoulders are
        symmetric, so any L/R difference in height (Y) or depth (Z) is T-pose
        measurement error — and a too-far-back anchor inflates that arm's reach,
        causing extra clamping/penetration. Averages |X|, Y, Z and mirrors X."""
        r, l = self._r_shoulder_off, self._l_shoulder_off
        if r is None or l is None:
            return
        ax = (abs(float(r[0])) + abs(float(l[0]))) * 0.5
        ay = (float(r[1]) + float(l[1])) * 0.5
        az = (float(r[2]) + float(l[2])) * 0.5
        self._r_shoulder_off = Gf.Vec3f(+ax, ay, az)
        self._l_shoulder_off = Gf.Vec3f(-ax, ay, az)

    def _despike(self, key, pos):
        """Reject a single-frame tracking teleport: if `pos` moved from the last
        accepted position faster than _spike_vmax, hold the previous position (up
        to _spike_max_hold frames, then accept so real relocations aren't frozen)."""
        prev = self._last_pos.get(key)
        if prev is not None:
            if _len(_sub(pos, prev)) > self._spike_vmax * max(self._frame_dt, 1e-3):
                if self._spike_hold.get(key, 0) < self._spike_max_hold:
                    self._spike_hold[key] = self._spike_hold.get(key, 0) + 1
                    return prev
        self._spike_hold[key] = 0
        self._last_pos[key] = pos
        return pos

    def _apply_arm_ik(self, is_right: bool):
        sk = self._skel
        dev = self._right_dev if is_right else self._left_dev
        if dev is None and not self._sim_enabled:
            return

        # Use simulated poses if simulation is enabled
        if self._sim_enabled:
            wrist_m = self._get_simulated_pose(is_head=False, is_right=is_right)
        else:
            wrist_m = _get_pose_raw(dev)   # position only — physical space, no loco jump
        if wrist_m is None:
            return
        # Record the raw controller world position (the IK input) for the debug
        # capture — mapped through _rest_to_world so it shares the frame of the
        # recorded IK target (_sim_hand_world) and rendered wrist (_hand_world).
        self._sim_ctrl_world[is_right] = self._rest_to_world(
            _vec3f(wrist_m.ExtractTranslation()))

        # Simulated hand target will be stored after transforms, just before IK solves.

        # Live shoulder anchor = current head position + calibrated head→shoulder
        # offset. Follows the user as they move/turn (not frozen at calibration).
        # For simulation, use a default shoulder offset if not calibrated.
        shoulder_off = self._r_shoulder_off if is_right else self._l_shoulder_off
        if shoulder_off is None:
            if self._sim_enabled:
                # Default sim anchor = the avatar's ACTUAL shoulder relative to the
                # head (rest frame, yaw 0), so the IK maps around the real shoulder
                # instead of a guessed offset. Without this the sim anchor sat ~8cm
                # off and the debug capture showed a false follow error.
                sh = sk.r_shoulder_pos if is_right else sk.l_shoulder_pos
                shoulder_off = _sub(sh, _vec3f(sk.head_rest_world))
            else:
                return

        # Use simulated head pose if simulation is enabled
        if self._sim_enabled:
            head_m = self._get_simulated_pose(is_head=True)
        else:
            head_m = _get_pose_raw(self._head_dev)  # position only
        if head_m is None:
            return
        head_pos = self._despike("head", _vec3f(head_m.ExtractTranslation()))
        if self._smooth_on:
            head_pos = self._filt_head.filter(head_pos)
        # Live body yaw from the raw head pose, ABSOLUTE (the avatar's local
        # frame corresponds to raw yaw 0, not to the calibration heading —
        # the calibration yaw is already baked into the yaw-neutral shoulder
        # offsets). Hold the last yaw while the head pitches near vertical
        # (looking down at the hands), where the horizontal forward direction
        # degenerates and atan2 becomes noise.
        fwd = head_m.TransformDir(Gf.Vec3d(0, 0, -1))
        if fwd[0] * fwd[0] + fwd[2] * fwd[2] > 0.04:
            self._body_yaw = math.atan2(-float(fwd[0]), -float(fwd[2]))
        yaw_now = self._body_yaw if self._body_yaw is not None else 0.0

        # Shoulder yaw (separate from body yaw): low-pass filter to track large
        # sustained turns while rejecting momentary head glances/nods.
        if self._shoulder_yaw is None:
            self._shoulder_yaw = yaw_now
        else:
            diff = (yaw_now - self._shoulder_yaw + math.pi) % (2 * math.pi) - math.pi
            self._shoulder_yaw += diff * (1.0 - math.exp(-3.0 * self._frame_dt))

        # Shoulders rotate with the body around the head (offsets are stored
        # in the user's body frame by calibration). Use the filtered shoulder yaw,
        # not the instantaneous head yaw, so shoulders don't wobble on head glances.
        # Also adjust shoulder height dynamically: when reaching below head level,
        # the shoulder naturally rotates downward (a biomechanical constraint).
        live_xr_temp = _vec3f(wrist_m.ExtractTranslation())
        reach_below_head = min(0.0, float(live_xr_temp[1] - head_pos[1]))  # negative if below
        # Drop the shoulder anchor a fraction of the downward reach (some natural
        # shoulder lowering). Reduced 0.3→0.15: at 0.3 it shortened down-reaches by
        # ~13cm (the anchor dropped but the target origin didn't); 0.15 halves that.
        shoulder_height_adj = reach_below_head * 0.15
        shoulder_off_adj = Gf.Vec3f(shoulder_off[0], shoulder_off[1] + shoulder_height_adj, shoulder_off[2])
        shoulder_xr = _add(head_pos, _rot_y(shoulder_off_adj, self._shoulder_yaw))

        shoulder = sk.r_shoulder_pos if is_right else sk.l_shoulder_pos
        len1 = sk.r_upperarm_len if is_right else sk.l_upperarm_len
        len2 = sk.r_forearm_len  if is_right else sk.l_forearm_len
        maxr = len1 + len2

        # Proportional shoulder-anchored mapping. Both the XR controller input
        # and the avatar rest data are now in the SAME true-world frame (rest
        # data via UsdSkelSkeletonQuery.ComputeJointWorldTransforms), so the
        # real arm vector maps directly — no frame transform, no axis negations.
        live_xr = self._despike(is_right, _vec3f(wrist_m.ExtractTranslation()))
        if self._smooth_on:
            live_xr = (self._filt_rhand if is_right else self._filt_lhand).filter(live_xr)
        # Counter-rotate by the ABSOLUTE body yaw: maps the raw-space arm
        # vector into the avatar's rest/local frame (user forward ↔ avatar
        # -Z); the root prim adds the world yaw via camera follow.
        arm_vec = _rot_y(_sub(live_xr, shoulder_xr), -yaw_now)
        s = self._ik_scale * self._ik_scale_mult
        arm_stage = _scale(arm_vec, s)
        target = _add(shoulder, arm_stage)
        # The 1:1-mapped (unclamped) target — where the controller maps before any
        # reach cap / cone limit / body push. Captured for the replay-metrics
        # follow error, which measures how far those corrections move the hand.
        if self._replay_capturing:
            self._map_ideal_world[is_right] = self._rest_to_world(target)

        # Anatomical reach limit: clamp the arm direction to a human workspace
        # (no reaching behind-and-across) and cap the reach so the elbow keeps a
        # slight bend. Done in the avatar-local frame, before the body push and
        # IK. When a target is clamped the hand deviates from the controller;
        # Phase-2 body-turn (if on) reduces how often that happens by rotating
        # the torso toward the reach.
        if self._limits_on:
            d = _sub(target, shoulder)
            dist = _len(d)
            if dist > 1e-5:
                cdir = self._clamp_dir_to_cone(_scale(d, 1.0 / dist), is_right)
                # Soft reach cap: compress the last ~20% of the range toward the
                # limit so the hand keeps extending instead of hitting a wall.
                # #1 Reach extension: when shoulder-follow is on, the clavicle
                # protracts toward the hand, so the reachable distance is the arm
                # length PLUS that protraction — raise the cap by _clav_reach_bonus
                # so far/forward reaches use it instead of clamping short.
                cap = self._lim_reach * maxr
                if self._clav_follow:
                    cap += self._clav_reach_bonus
                dist = _soft_cap_max(dist, cap, 0.2 * cap)
                target = _add(shoulder, _scale(cdir, dist))

        # Soft body collision: if the hand target would land inside the torso,
        # push it out to the capsule surface so the hand/forearm rest ON the body
        # instead of clipping through it. The hand only deviates from the
        # controller while it would otherwise be inside the body; everywhere else
        # it stays exactly 1:1. The elbow swivel below then clears the rest.
        # Both body pushes (analytic + physics) are applied to a copy, then the
        # resulting CORRECTION is eased over a few frames so the hand glides onto
        # and off the body instead of snapping when the push turns on/off.
        raw_target = target
        pushed = target
        if self._body_push:
            pushed = self._push_out_torso(pushed, self._FORE_R)
        if self._phys_collision and self._phys_scene_ready:
            try:
                pushed = self._phys_push_point_out_torso(
                    pushed, self._FORE_R + self._phys_probe_r)
            except Exception:
                pass  # physics correction must never break tracking
        new_corr = _sub(pushed, raw_target)
        prev_corr = self._push_corr.get(is_right, Gf.Vec3f(0, 0, 0))
        a = 1.0 - math.exp(-self._push_rate * self._frame_dt)
        corr = _add(prev_corr, _scale(_sub(new_corr, prev_corr), a))
        self._push_corr[is_right] = corr
        target = _add(raw_target, corr)

        # PhysX environment collision: stop the hand at the surface of any scene
        # object it would otherwise move into this frame (analytic model can't
        # see external geometry). Body proxies and the avatar mesh are ignored.
        if self._phys_collision and self._phys_scene_ready and self._phys_env:
            try:
                target = self._phys_env_clamp(target, is_right)
            except Exception:
                pass  # physics correction must never break tracking

        # Store the final IK target (after all transforms) for debug visualization,
        # so the yellow circles show what the arm is actually trying to reach. Also
        # populated during a replay-metrics capture so it can measure target vs ctrl.
        if self._sim_enabled or self._replay_capturing:
            self._sim_hand_world[is_right] = self._rest_to_world(target)

        # --- Diagnostic: log the RAW arm_vec (XR space) for the right arm once
        # per ~second, so the XR→stage axis mapping can be derived from real
        # reaches instead of guessed. Remove once mapping is confirmed.
        if is_right and DEBUG_FILES:
            self._armvec_tick = getattr(self, "_armvec_tick", 0) + 1
            if self._armvec_tick % 60 == 0:
                try:
                    with open(_data_path("_armvec_debug.txt"),
                              "a", encoding="utf-8") as f:
                        f.write(
                            f"arm_vec=({arm_vec[0]:+.3f},{arm_vec[1]:+.3f},"
                            f"{arm_vec[2]:+.3f}) len={_len(arm_vec):.2f}  "
                            f"wrist=({live_xr[0]:+.3f},{live_xr[1]:+.3f},"
                            f"{live_xr[2]:+.3f})  head=({head_pos[0]:+.3f},"
                            f"{head_pos[1]:+.3f},{head_pos[2]:+.3f})  "
                            f"shoulder=({shoulder_xr[0]:+.3f},"
                            f"{shoulder_xr[1]:+.3f},{shoulder_xr[2]:+.3f})  "
                            f"off=({shoulder_off[0]:+.3f},{shoulder_off[1]:+.3f},"
                            f"{shoulder_off[2]:+.3f})  "
                            f"body_yaw={math.degrees(yaw_now):+.1f}deg\n")
                except Exception:
                    pass

        # --- Clavicle / shoulder follow (VRIK-style) ---------------------------
        # Rotate the clavicle a FRACTION of the way toward the hand target so the
        # shoulder lifts/protracts on high, forward and cross-body reaches,
        # instead of the arm digging into the torso. The hand target is
        # unchanged; only the arm ROOT moves, then the 2-bone IK solves to the
        # SAME target from the moved shoulder. `shoulder` stays the REST shoulder
        # (it pairs with the rest upper-arm rotation below); the IK uses
        # `shoulder_ik`, and the upper arm's parent becomes the new clavicle
        # world rotation `clav_world`.
        clav_idx    = sk.r_clav_idx if is_right else sk.l_clav_idx
        clav_pos    = sk.r_clav_pos if is_right else sk.l_clav_pos
        clav_rest_q = sk.r_clav_world_q_rest if is_right else sk.l_clav_world_q_rest
        shoulder_ik = shoulder
        clav_world  = clav_rest_q
        if self._clav_follow:
            rest_ref = _normalize(_sub(shoulder, clav_pos))   # clavicle bone at rest
            targ_dir = _normalize(_sub(target, clav_pos))      # clavicle→hand now
            q_clav   = _quat_scale_angle(_quat_from_to(rest_ref, targ_dir),
                                         self._clav_weight, self._clav_max)
            # Cross-body forward protraction: when the hand crosses the body
            # midline, swing the shoulder FORWARD (rotate the clavicle about the
            # avatar up axis) so the upper arm passes in FRONT of the torso
            # instead of cutting through it — what a real shoulder does reaching
            # across. The hand stays on target (IK re-solves from the moved root);
            # only the upper-arm chord moves off the chest. Scales 0→1 as the arm
            # direction crosses to the opposite side.
            arm_dir = _normalize(_sub(target, shoulder))
            cross   = max(0.0, min(1.0,
                          ((-arm_dir[0]) if is_right else arm_dir[0]) / 0.5))
            q_total = q_clav
            if cross > 0.0:
                sign = 1.0 if is_right else -1.0
                q_total = _quat_mul(_yaw_quatf(sign * cross * self._protract_max),
                                    q_clav)
            shoulder_ik = _add(clav_pos, _quat_rotate(q_total, _sub(shoulder, clav_pos)))
            clav_world  = _quat_mul(q_total, clav_rest_q)
            clav_local  = _quat_mul(_quat_conj(sk.clav_parent_world_q), clav_world)
            sk.write_joint_rotation(clav_idx, clav_local)

        # The rendered upper arm is rooted at the clavicle end (= shoulder_ik
        # BEFORE the physics push below). Keep this for placing the arm collider
        # proxies so they sit on the actual mesh, not on the pushed IK root.
        shoulder_render = shoulder_ik

        # PhysX shoulder push (optional, off by default): nudge the upper-arm ROOT
        # outside the real torso box. It perturbs the IK shoulder without moving
        # the rendered (clavicle-rooted) shoulder, so it can add snap for little
        # visible gain — gated behind _phys_shoulder_push.
        if (self._phys_collision and self._phys_scene_ready
                and self._phys_shoulder_push):
            try:
                shoulder_ik = self._phys_push_point_out_torso(
                    shoulder_ik, self._UPPER_R + self._phys_probe_r)
            except Exception:
                pass  # physics correction must never break tracking

        # Adaptive pole: elbow bias adapts to arm direction so it hangs naturally
        # across all poses (down for relaxed arms, down+back when reaching forward,
        # down+outward when reaching up). This mimics VRIK/FinalIK behavior.
        # Compute arm direction (shoulder_ik → target) to blend the pole components.
        arm_dir = _normalize(_sub(target, shoulder_ik))
        fwd_blend = max(0.0, -float(arm_dir[2]))   # how much arm points forward (-Z)
        up_blend = max(0.0, float(arm_dir[1]))     # how much arm points up (+Y)
        # Down preference engages once the hand is below ~0.4 ABOVE the shoulder
        # (not just below it): the residual elbow up-flips happened in this
        # transition zone (arm_y ~0.13–0.3) coming down from an overhead reach,
        # where back/out used to win. Above ~0.4 (a clear overhead reach) the elbow
        # is left free to rise.
        down_blend = max(0.0, 0.4 - float(arm_dir[1]))

        # Blend the pole components based on arm pose:
        # - Down bias dominates, and STRENGTHENS as the arm drops below horizontal
        #   so the elbow swings back to pointing down when the hand comes from an
        #   up-reach to below the head (back/out from the up pose would otherwise
        #   linger via the swivel continuity bias).
        # - Back bias increases when reaching forward
        # - Outward bias reduces when reaching forward (arm is already separated)
        #   and when reaching up (shoulder lifts)
        out_scaled = (-1.0 if is_right else 1.0) * self._pole_out * (1.0 - up_blend * 0.4) * (1.0 - fwd_blend * 0.8)
        # Back bias keeps the elbow off the chest on forward reaches, but FADES as
        # the arm drops below horizontal — there, pointing the elbow DOWN already
        # clears the chest, so back must not out-weigh down (the bug: on a forward
        # below-head reach back=4.0 beat down=3.2, so the elbow stayed back/up).
        back_scaled = (self._pole_back * (1.0 + fwd_blend * 0.8)
                       * max(0.0, 1.0 - 0.7 * down_blend))
        down_scaled = -self._pole_down * (1.0 + self._pole_down_below * down_blend)

        # Normalize and scale to maintain proper magnitude
        pole_raw = Gf.Vec3f(out_scaled, down_scaled, back_scaled)
        pole_len = _len(pole_raw)
        if pole_len > 1e-5:
            pole = _scale(pole_raw, 1.0 / pole_len)
        else:
            pole = Gf.Vec3f(0, -1, 0)
        # Scale to the weighted magnitude (down bias is the reference)
        pole = _scale(pole, self._pole_down)

        # VRIK-style bend goal: rotate the natural pole about the limb axis by the
        # controller roll, so the elbow swings out when the palm turns up and drops
        # in when it turns down — the swivel a real arm picks for a given wrist
        # twist. Uses the wrist roll measured in _apply_hand_tracking (prev frame);
        # the downstream torso/PhysX swivel still clears collisions from here.
        if self._elbow_roll_weight != 0.0:
            roll = self._wrist_roll.get(is_right, 0.0)
            if roll:
                sign = 1.0 if is_right else -1.0
                pole = _quat_rotate(
                    _quat_axis_angle(arm_dir, sign * self._elbow_roll_weight * roll),
                    pole)

        elbow, wrist, base, h, u, v = _two_bone_ik_full(
            shoulder_ik, target, len1, len2, pole)

        # Self-collision: swivel the elbow around the shoulder→wrist axis to keep
        # the upper arm and forearm out of the torso, staying as close as
        # possible to the natural pole. The wrist is untouched, so the hand
        # stays exactly on the controller and both bone lengths are preserved.
        elbow = self._swivel_clear_torso(shoulder_ik, wrist, base, h, u, v, is_right)

        # PhysX self-collision refine: catch body parts the analytic ellipse
        # misses (head, hips, the opposite arm) by swiveling the elbow further
        # until the real body proxies are clear. Wrist untouched ⇒ hand stays on
        # the controller and bone lengths are preserved.
        if self._phys_collision and self._phys_scene_ready:
            try:
                elbow = self._phys_refine_elbow(
                    shoulder_ik, wrist, base, h, u, v, is_right, elbow)
            except Exception as e:
                # Log once per ~2s so a recurring failure is visible instead of
                # silently degrading to analytic-only behaviour.
                self._phys_err_tick = getattr(self, "_phys_err_tick", 0) + 1
                if self._phys_err_tick % 120 == 1:
                    print(f"[avatar_xr_control] _phys_refine_elbow error: {e}")

        if is_right:
            reach = _len(_sub(target, shoulder_ik))
            clamp = "CLAMP" if reach > maxr else "ok"
            self._set_track(
                f"arm=({arm_stage[0]:+.2f},{arm_stage[1]:+.2f},{arm_stage[2]:+.2f}) "
                f"reach={reach:.2f}/{maxr:.2f} {clamp}",
                0xFF44FF44)

        wrist_rest = sk.r_wrist_pos if is_right else sk.l_wrist_pos
        # Rest bone directions (stage space) from the cached rest positions.
        rest_upper_dir = _normalize(_sub(
            sk.r_elbow_pos if is_right else sk.l_elbow_pos, shoulder))
        rest_fore_dir = _normalize(_sub(
            wrist_rest, sk.r_elbow_pos if is_right else sk.l_elbow_pos))

        # Live bone directions from the IK solution (from the MOVED shoulder).
        upper_dir = _normalize(_sub(elbow, shoulder_ik))
        fore_dir  = _normalize(_sub(wrist, elbow))

        upper_idx = sk.r_upper_idx if is_right else sk.l_upper_idx
        fore_idx  = sk.r_fore_idx  if is_right else sk.l_fore_idx
        upper_rest_q   = sk.r_upper_world_q_rest if is_right else sk.l_upper_world_q_rest
        # Parent is the NEW clavicle world rotation (shoulder follow), so the
        # upper-arm local rotation is taken relative to the moved clavicle.
        upper_parent_q = clav_world
        fore_rest_q    = sk.r_fore_world_q_rest if is_right else sk.l_fore_world_q_rest

        # Upper arm: rest_upper_dir → upper_dir, on top of its rest world rotation.
        upper_local, upper_world = _bone_rotation_from_vectors(
            rest_upper_dir, upper_dir, upper_rest_q, upper_parent_q)
        sk.write_joint_rotation(upper_idx, upper_local)

        # Forearm: rest_fore_dir → fore_dir. Parent is the upper arm's NEW world
        # rotation so the elbow bend composes correctly. Keep the forearm's LIVE
        # world rotation so the hand can use it as its parent frame (otherwise the
        # hand orientation only matches in T-pose, drifting as the arm moves).
        fore_local, fore_world = _bone_rotation_from_vectors(
            rest_fore_dir, fore_dir, fore_rest_q, upper_world)
        sk.write_joint_rotation(fore_idx, fore_local)
        self._fore_world_live[is_right] = fore_world
        # Stash the forearm pointing dir + local rotation so _apply_hand_tracking
        # can route wrist twist onto the forearm roll (swing-twist).
        self._fore_dir_live[is_right]   = fore_dir
        self._fore_local_live[is_right] = fore_local

        # The IK solves in the CACHED rest frame; the root prim has moved/yawed
        # since (camera follow), so map the wrist through the root delta to get
        # the live world position for grab distance checks.
        self._hand_world[is_right] = self._rest_to_world(wrist)

        # Cache the RENDERED arm (rest frame) so the body-collider proxies sit on
        # the actual mesh (which is rooted at the clavicle end, not the pushed IK
        # shoulder). Rebuild the chain from shoulder_render along the live bone
        # directions at the true bone lengths — exactly what the skeleton renders.
        render_elbow = _add(shoulder_render, _scale(upper_dir, len1))
        render_wrist = _add(render_elbow, _scale(fore_dir, len2))
        self._arm_solved[is_right] = (shoulder_render, render_elbow, render_wrist)
        self._elbow_world[is_right] = self._rest_to_world(render_elbow)

    # ------------------------------------------------------------------
    # Grab (level-2 object interaction)
    # ------------------------------------------------------------------

    def _rest_to_world(self, p):
        """Cached-rest-frame point → live world. _AvatarSkel caches all rest
        data with the root transform at init time; camera follow moves the
        root afterwards, so live world = current_root ∘ root_rest⁻¹ ∘ p."""
        sk = self._skel
        stage = omni.usd.get_context().get_stage()
        if sk is None or stage is None:
            return p
        prim = stage.GetPrimAtPath(self._avatar_root_path())
        if not prim.IsValid():
            return p
        cur = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        m = sk.root_rest_xform_inv * cur   # Gf row-vector convention: rest⁻¹ then cur
        wp = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
        return Gf.Vec3f(float(wp[0]), float(wp[1]), float(wp[2]))

    def _world_to_rest(self, wp):
        """Inverse of _rest_to_world: live world point → cached-rest frame.
        Used to bring a physics-clamped world target back into the frame the IK
        solves in."""
        sk = self._skel
        stage = omni.usd.get_context().get_stage()
        if sk is None or stage is None:
            return wp
        prim = stage.GetPrimAtPath(self._avatar_root_path())
        if not prim.IsValid():
            return wp
        cur = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        m = (sk.root_rest_xform_inv * cur).GetInverse()
        rp = m.Transform(Gf.Vec3d(wp[0], wp[1], wp[2]))
        return Gf.Vec3f(float(rp[0]), float(rp[1]), float(rp[2]))

    def _skel_local_to_world(self, p):
        """Transform a point from skeleton-local space to stage world space."""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return p
        prim = stage.GetPrimAtPath(self._skel_path)
        if not prim.IsValid():
            return p
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        wp = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
        return Gf.Vec3f(float(wp[0]), float(wp[1]), float(wp[2]))

    # ------------------------------------------------------------------
    # Finger-tracking diagnostic (read-only — writes _finger_diag.txt)
    # ------------------------------------------------------------------

    _FINGER_DIAG_PATH = _data_path("_finger_diag.txt")

    def _diagnose_fingers(self):
        """Probe whether optical hand-tracking finger joints are arriving from
        ALVR/OpenXR. Writes a report so the cause (controllers active vs ALVR/
        SteamVR not forwarding XR_EXT_hand_tracking) can be read without guessing.
        Drop the controllers before pressing, so Quest is in hand-tracking mode."""
        lines = ["=== Finger Tracking Diagnostic ===", ""]
        try:
            from omni.kit.xr.core import XRCore
            xr = XRCore.get_singleton()
        except Exception as e:
            lines.append(f"XRCore unavailable: {e}  (is the XR session running?)")
            self._write_finger_diag(lines)
            return

        finger_names = [row[0] for row in FINGER_POSE_MAP]
        any_fingers = False
        for path, label in (("/user/hand/right", "RIGHT"),
                            ("/user/hand/left", "LEFT")):
            lines.append(f"--- {label} ({path}) ---")
            dev = None
            try:
                dev = xr.get_input_device(path)
            except Exception as e:
                lines.append(f"  get_input_device failed: {e}")
            if dev is None:
                lines.append("  device is None (no XR session / device not bound)")
                lines.append("")
                continue
            # 1) data source: "hand" = optical tracking, "controller" = holding a
            #    controller (then no fingers), "" = not a hand / inactive.
            try:
                src = str(dev.get_hand_tracking_data_source())
            except Exception as e:
                src = f"<error: {e}>"
            lines.append(f"  hand_tracking_data_source = '{src}'  "
                         f"(want 'hand'; 'controller' = put controller down)")
            # 2) all pose names the device currently exposes.
            try:
                names = [str(p) for p in dev.get_pose_names()]
            except Exception as e:
                names = []
                lines.append(f"  get_pose_names failed: {e}")
            lines.append(f"  pose_names ({len(names)}): {names}")
            # 3) per-finger presence + validity.
            present = 0
            for fn in finger_names:
                try:
                    has = dev.has_pose(fn)
                except Exception:
                    has = False
                flags = ""
                if has:
                    present += 1
                    try:
                        d = dev.get_raw_pose_desc(fn)
                        flags = f" validity={int(d.validity_flags)}"
                    except Exception:
                        pass
                lines.append(f"    {fn:22} has_pose={has}{flags}")
            lines.append(f"  finger joints present: {present}/{len(finger_names)}")

            # 4) Pinch/poke gesture probe — how to read "is pinching" for grab.
            #    Try input-gesture values AND pose validity. PINCH NOW while pressing.
            try:
                inputs = [str(n) for n in dev.get_input_names()]
            except Exception:
                inputs = []
            lines.append(f"  input_names: {inputs}")
            for comp in ("pinch", "poke", "grip", "trigger", "squeeze",
                         "thumbstick", "joystick", "trackpad", "stick"):
                try:
                    if not dev.has_input(comp):
                        continue
                except Exception:
                    continue
                gestures = []
                try:
                    gestures = [str(g) for g in dev.get_input_gesture_names(comp)]
                except Exception:
                    pass
                vals = {}
                for g in gestures:
                    try:
                        vals[g] = round(float(dev.get_input_gesture_value(comp, g)), 3)
                    except Exception:
                        vals[g] = "?"
                lines.append(f"    input '{comp}': gestures={gestures} values={vals}")
            # pinch/poke pose validity (alternative detection)
            for pn in ("pinch", "poke"):
                try:
                    if dev.has_pose(pn):
                        d = dev.get_raw_pose_desc(pn)
                        lines.append(f"    pose '{pn}' validity={int(d.validity_flags)}")
                except Exception:
                    pass
            lines.append("")
            if present > 0:
                any_fingers = True

        verdict = ("FINGERS DETECTED - tracking data is arriving."
                   if any_fingers else
                   "NO finger joints. Check: controllers down? Quest hand-tracking "
                   "ON? ALVR 'Hand skeleton' ON? SteamVR forwarding XR_EXT_hand_tracking?")
        lines.append(f"VERDICT: {verdict}")
        self._write_finger_diag(lines)
        self._set_track(verdict, 0xFF44FF44 if any_fingers else 0xFFFFAA33)

    def _write_finger_diag(self, lines):
        try:
            with open(self._FINGER_DIAG_PATH, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        except Exception as e:
            print(f"[avatar_xr_control] finger diag write failed: {e}")

    def _apply_head(self):
        m = _get_pose(self._head_dev)
        if m is None:
            self._set_track("No head pose - is XR session active?", 0xFFFFAA33)
            return
        quatf = _correct_xr_quat(m.ExtractRotationQuat())
        # Undo the root yaw (camera follow): the joint expects an avatar-local
        # frame orientation, otherwise the head double-rotates when the body
        # turns. Yaw rotations commute with the axis flip in _correct_xr_quat.
        if self._root_yaw:
            quatf = _quat_mul(_quat_conj(_yaw_quatf(self._root_yaw)), quatf)
        self._skel.write_joint_rotation(self._skel.head_idx, quatf)
        pos = m.ExtractTranslation()
        self._set_track(
            f"head pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
            0xFF44FF44,
        )

    # Per-segment curl multipliers (proximal/intermediate/distal) for a fist.
    _CURL_SEG = {"1": 0.7, "2": 1.0, "3": 0.85}

    def _apply_finger_curl(self, sk, is_right, grip, trigger):
        """Procedural finger curl when no optical hand tracking is present
        (controllers held). Trigger curls the index; grip/squeeze curls
        middle/ring/pinky and a softer thumb. Each finger joint rotates about
        its cached knuckle-line axis on top of its rest pose. SteamVR-Skeletal-
        Input-style controller fallback; values are pre-eased by the caller."""
        curl = getattr(sk, "finger_curl", None)
        if not curl:
            return
        base = math.radians(self._finger_curl_deg)
        pre = "r_" if is_right else "l_"
        for key, fc in curl.items():
            if not key.startswith(pre):
                continue
            if "index" in key:
                drive = trigger
            elif fc["thumb"]:
                drive = grip * self._finger_thumb
            else:
                drive = grip
            drive = max(0.0, min(1.0, drive))
            ang = base * self._CURL_SEG.get(fc["seg"], 1.0) * drive
            q = _quat_mul(_quat_axis_angle(fc["axis"], ang), fc["rest"])
            sk.write_joint_rotation(fc["idx"], q)

    def _apply_hand_tracking(self):
        sk = self._skel
        if sk is None:
            return

        for dev, is_right in ((self._right_dev, True), (self._left_dev, False)):
            # Drive hand orientation from the controller, OR — when there is no
            # device but Input Simulation is on — from a SYNTHETIC wrist twist so
            # the forearm-roll path is exercised/testable without a headset.
            sim_orient = (dev is None and self._sim_enabled)
            if dev is None and not sim_orient:
                continue
            fore_parent = self._fore_world_live.get(is_right)
            if fore_parent is None:   # IK not run yet → fall back to rest
                fore_parent = sk.r_fore_world_q_rest if is_right else sk.l_fore_world_q_rest
            fore_dir   = self._fore_dir_live.get(is_right)
            fore_local = self._fore_local_live.get(is_right)
            fore_idx   = sk.r_fore_idx if is_right else sk.l_fore_idx
            a_local = (_normalize(_quat_rotate(_quat_conj(fore_parent), fore_dir))
                       if fore_dir is not None else None)

            # Build local_q = wrist orientation relative to the live forearm frame.
            local_q = None
            if sim_orient:
                # Synthetic: a pure twist (pronation/supination) about the forearm
                # axis by _sim_wrist_twist — the worst case for the wrist clamp.
                if a_local is not None:
                    local_q = _quat_axis_angle(a_local, self._sim_wrist_twist)
            else:
                wrist_m = _get_pose(dev)
                if wrist_m is not None:
                    q_wrist    = wrist_m.ExtractRotationQuat()
                    q_wrist_f  = Gf.Quatf(float(q_wrist.GetReal()),
                                          float(q_wrist.GetImaginary()[0]),
                                          float(q_wrist.GetImaginary()[1]),
                                          float(q_wrist.GetImaginary()[2]))
                    # Stage-space controller quat → avatar-local frame: undo the
                    # root yaw (camera follow). The IK forearm frame lives in the
                    # avatar's rest/local frame.
                    if self._root_yaw:
                        q_wrist_f = _quat_mul(_quat_conj(_yaw_quatf(self._root_yaw)),
                                              q_wrist_f)
                    # Orientation low-pass: kill controller rotation jitter before
                    # it reaches the hand/forearm (positions are filtered upstream,
                    # rotation was not). Adaptive, so fast wrist turns stay crisp.
                    if self._smooth_on:
                        q_wrist_f = (self._filt_rrot if is_right
                                     else self._filt_lrot).filter(q_wrist_f)
                    local_q = _quat_mul(_quat_conj(fore_parent), q_wrist_f)

            if local_q is not None:
                # Total wrist orientation requested by the controller (relative to
                # the forearm), before any routing/offset/clamp — for the diag.
                req_total_deg = math.degrees(_quat_angle(local_q))
                # Stash the SIGNED wrist roll (twist about the forearm axis) for
                # the VRIK bend-goal in _apply_arm_ik next frame: palm-up vs
                # palm-down should swing the elbow. Measured before the forearm-
                # roll routing below alters local_q.
                if a_local is not None:
                    _sw_r, _tw_r = _swing_twist(local_q, a_local)
                    _ti = _tw_r.GetImaginary()
                    _s = (float(_ti[0]) * a_local[0] + float(_ti[1]) * a_local[1]
                          + float(_ti[2]) * a_local[2])
                    self._wrist_roll[is_right] = 2.0 * math.atan2(
                        _s, float(_tw_r.GetReal()))
                # Calibration offset FIRST → the hand-joint-local orientation
                # (≈ the rest hand pose at neutral). Then work RELATIVE to that
                # neutral, so the forearm-roll routing and the wrist clamp see only
                # the user's real deviation — not the constant controller↔forearm
                # convention offset. Routing that convention twist onto the forearm
                # is what over-twisted it once _fore_roll was raised.
                rest_local = (sk.r_hand_local_rest_q if is_right
                              else sk.l_hand_local_rest_q)
                offset = self._r_wrist_offset if is_right else self._l_wrist_offset
                # Apply the convention offset in the CONTROLLER frame (RIGHT-multiply):
                # hand_world = forearm · (local_q · C) = controller · C, so the hand
                # tracks the controller exactly, independent of the (guessed) IK
                # forearm. LEFT-multiplying applied C in the forearm frame, giving a
                # pose-dependent tilt that killed pitch and inverted yaw. (Needs the
                # matching right-handed C from _do_calibrate → recalibrate once.)
                cal = _quat_mul(local_q, offset) if offset is not None else local_q
                # Deviation from the neutral hand pose, expressed in the FOREARM-
                # local frame (= cal·rest⁻¹), so a_local is the correct twist axis
                # and hdev ≈ identity at neutral. NOTE: named hdev, NOT dev — `dev`
                # is the loop's XR device, still needed below for the finger reads.
                hdev = _quat_mul(cal, _quat_conj(rest_local))

                # Forearm roll: route a fraction of the deviation's TWIST (roll
                # about the forearm axis) onto the forearm bone — natural pronation.
                # At neutral hdev≈identity ⇒ ZERO forearm twist (no constant offset).
                fore_deg = hand_resid_deg = 0.0
                if (self._fore_roll > 0.0 and a_local is not None
                        and fore_local is not None):
                    _swing, twist = _swing_twist(hdev, a_local)
                    tf = _quat_scale_angle(twist, self._fore_roll)
                    sk.write_joint_rotation(fore_idx, _quat_mul(fore_local, tf))
                    hdev = _quat_mul(_quat_conj(tf), hdev)
                    fore_deg = math.degrees(_quat_angle(tf))

                # Wrist clamp REMOVED from the controller path. It capped the
                # deviation measured against the IK-inferred forearm, but in 3-point
                # tracking that forearm is a guess, so a neutral wrist already reads
                # as a large "bend" — the clamp then saturated and FROZE wrist pitch
                # (roll survived because it routes to the forearm separately). The
                # controller orientation is always a valid human hand pose, and the
                # orientation low-pass handles jitter, so no clamp is needed.
                # pre_clamp_deg is kept only for the diagnostic.
                pre_clamp_deg = math.degrees(_quat_angle(hdev))
                # Record how the synthetic twist was distributed (for the capture).
                if sim_orient and a_local is not None:
                    _sw, hand_tw = _swing_twist(hdev, a_local)
                    hand_resid_deg = math.degrees(_quat_angle(hand_tw))
                    self._sim_twist_diag[is_right] = {
                        "requested_deg": math.degrees(self._sim_wrist_twist),
                        "forearm_deg": fore_deg,
                        "hand_deg": hand_resid_deg,
                        "total_deg": fore_deg + hand_resid_deg,
                        "clamped": pre_clamp_deg > math.degrees(self._lim_wrist) + 0.5,
                    }
                # Hand-orientation diagnostic for real motion (replay capture):
                # how the requested wrist rotation splits into forearm roll vs the
                # residual hand bend/twist, and whether the wrist clamp cut it.
                if self._replay_capturing and a_local is not None:
                    _swh, _twh = _swing_twist(hdev, a_local)
                    self._hand_orient_diag[is_right] = {
                        "req_deg": req_total_deg,
                        "forearm_deg": fore_deg,
                        "hand_bend_deg": math.degrees(_quat_angle(_swh)),
                        "hand_twist_deg": math.degrees(_quat_angle(_twh)),
                        "clamped": pre_clamp_deg > math.degrees(self._lim_wrist) + 0.5,
                    }
                # Recompose the hand-joint-local rotation (cal = dev·rest) and write
                # it. No _apply_hand_axes: that was a fixed REFLECTION (det −1) that
                # mirrored the hand (the source of the inverted-yaw / dead-pitch);
                # the calibrated right-handed offset C now carries the full
                # controller→hand-joint convention, so the remap is not needed.
                final      = _quat_mul(hdev, rest_local)
                hand_idx   = sk.r_hand_idx if is_right else sk.l_hand_idx
                sk.write_joint_rotation(hand_idx, final)
            if dev is None:
                continue   # no finger poses to read in simulation
            optical_any = False
            for xr_pose, r_key, l_key in FINGER_POSE_MAP:
                key = r_key if is_right else l_key
                idx = sk.finger_idx.get(key)
                if idx is None:
                    continue
                try:
                    m = dev.get_virtual_world_pose(xr_pose)
                    if m is None:
                        continue
                except Exception:
                    continue
                optical_any = True
                quatd = m.ExtractRotationQuat()
                q = Gf.Quatf(
                    float(quatd.GetReal()),
                    float(quatd.GetImaginary()[0]),
                    float(quatd.GetImaginary()[1]),
                    float(quatd.GetImaginary()[2]),
                )
                sk.write_joint_rotation(idx, q)

            # Controller fallback: no optical finger poses arrived this frame, so
            # curl the fingers procedurally from the grip/trigger inputs. Eased
            # per hand so the fist closes/opens smoothly instead of popping.
            if not optical_any and self._finger_curl_on:
                grip = _squeeze_value(dev)
                trig = _trigger_value(dev)
                st = self._finger_drive[is_right]
                a = 1.0 - math.exp(-self._finger_ease_rate * self._frame_dt)
                st[0] += (grip - st[0]) * a
                st[1] += (trig - st[1]) * a
                self._apply_finger_curl(sk, is_right, st[0], st[1])
