import asyncio
import math
import socket
import struct
import threading
import time

import omni.ext
import omni.ui as ui
import omni.usd
from omni.usd import StageEventType
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

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


def _solve_two_bone_ik(root, target, len1, len2, pole):
    """Analytic 2-bone IK.

    root   : shoulder world position (Gf.Vec3f)
    target : desired wrist world position
    len1   : upper-arm length, len2 : forearm length
    pole   : a world-space hint vector for which way the elbow points

    Returns (elbow_pos, wrist_pos) world positions. wrist_pos == clamped target.
    The caller turns these into bone direction vectors.
    """
    to_target = _sub(target, root)
    dist = _len(to_target)
    # Clamp the reach to [|len1-len2|, len1+len2] so the triangle is solvable.
    max_reach = len1 + len2
    min_reach = abs(len1 - len2)
    eps = 1e-5
    dist = max(min_reach + eps, min(dist, max_reach - eps))
    dir_t = _scale(to_target, 1.0 / max(_len(to_target), eps))
    # Distance from root to the elbow's projection on the root→target line
    # (law of cosines).
    a = (len1*len1 - len2*len2 + dist*dist) / (2.0 * dist)
    h = math.sqrt(max(0.0, len1*len1 - a*a))
    # Point on the root→target line below the elbow
    base = _add(root, _scale(dir_t, a))
    # Bend direction: component of pole perpendicular to dir_t
    pole_perp = _sub(pole, _scale(dir_t, _dot(pole, dir_t)))
    pl = _len(pole_perp)
    if pl < eps:
        # Pole parallel to limb; pick an arbitrary perpendicular
        ref = Gf.Vec3f(0, 1, 0) if abs(dir_t[1]) < 0.9 else Gf.Vec3f(1, 0, 0)
        pole_perp = _sub(ref, _scale(dir_t, _dot(ref, dir_t)))
        pl = _len(pole_perp)
    bend = _scale(pole_perp, 1.0 / pl)
    elbow = _add(base, _scale(bend, h))
    wrist = _add(root, _scale(dir_t, dist))
    return elbow, wrist


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
# Body OSC receiver (pure socket, no dependencies)
# ---------------------------------------------------------------------------

# Tracker IDs 1-8 from ALVR VRChat OSC sink
_BODY_TRACKER_MAP = {
    "head":  "head",
    "1":     "hip",
    "2":     "chest",
    "3":     "left_foot",
    "4":     "right_foot",
    "5":     "left_knee",
    "6":     "right_knee",
    "7":     "left_elbow",
    "8":     "right_elbow",
}


def _parse_osc(data: bytes):
    """Parse a minimal OSC message. Returns (address, [float, ...]) or None."""
    try:
        addr_end = data.index(b'\x00')
        address  = data[:addr_end].decode('utf-8')
        tag_start = (addr_end + 4) & ~3
        tag_end   = data.index(b'\x00', tag_start)
        tags      = data[tag_start + 1:tag_end].decode('utf-8')
        val_start = (tag_end + 4) & ~3
        values = []
        for t in tags:
            if t == 'f':
                values.append(struct.unpack('>f', data[val_start:val_start + 4])[0])
                val_start += 4
        return address, values
    except Exception:
        return None, None


class BodyOscReceiver:
    def __init__(self, port: int = 9000):
        self._positions = {name: Gf.Vec3f(0, 0, 0) for name in _BODY_TRACKER_MAP.values()}
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"[BodyOscReceiver] Listening on UDP port {port}")

    def _run(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                address, values = _parse_osc(data)
                if address is None or not address.endswith("/position") or len(values) < 3:
                    continue
                parts = address.split("/")
                if len(parts) < 5:
                    continue
                token = parts[3]
                joint = _BODY_TRACKER_MAP.get(token)
                if joint is None:
                    continue
                # ALVR OSC: X+ right, Y+ up, Z+ forward (Unity).
                # Avatar faces -Z in stage, so ALVR forward (+Z) = stage backward.
                # Negate Z to convert ALVR forward into stage forward (-Z).
                # X stays the same: ALVR right (+X) = stage right (+X).
                x  =  values[0]
                y  =  values[1]
                z  = -values[2]
                with self._lock:
                    self._positions[joint] = Gf.Vec3f(x, y, z)
            except socket.timeout:
                continue
            except Exception:
                continue

    def get_position(self, joint: str) -> Gf.Vec3f:
        with self._lock:
            return self._positions.get(joint, Gf.Vec3f(0, 0, 0))

    def stop(self):
        self._running = False
        self._sock.close()


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

        self._rotations = rotations
        self._scales    = scales

        # TRUE-WORLD joint transforms via the canonical UsdSkel query. This is
        # render-faithful — it composes the full transform stack (rest pose +
        # every prim above the skeleton), unlike the old hand-rolled walk which
        # was skeleton-local and disagreed with what the mesh actually renders.
        # Pin the cache on self so the query stays valid.
        self._skel_cache = UsdSkel.Cache()
        query = self._skel_cache.GetSkelQuery(skel)
        if not query:
            raise RuntimeError(f"UsdSkelSkeletonQuery invalid for {skel_path}")
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

        # Parent world rotations for world→local conversion of arm joints
        self.r_upper_parent_world_q = world_rot(self.r_clav_idx)
        self.l_upper_parent_world_q = world_rot(self.l_clav_idx)

        def _dist(a, b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

        # Bone lengths (constant) for the 2-bone IK solver
        self.r_upperarm_len = _dist(self.r_shoulder_pos, self.r_elbow_pos)
        self.r_forearm_len  = _dist(self.r_elbow_pos, self.r_wrist_pos)
        self.l_upperarm_len = _dist(self.l_shoulder_pos, self.l_elbow_pos)
        self.l_forearm_len  = _dist(self.l_elbow_pos, self.l_wrist_pos)

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

        print(f"[avatar_xr_control] Setup OK — {len(joints)} joints")

    def write_joint_rotation(self, idx: int, quatf: Gf.Quatf):
        self._rotations[idx] = quatf
        self.anim.GetRotationsAttr().Set(Vt.QuatfArray(self._rotations))

    def set_head_hidden(self, hidden: bool):
        """First-person 'head chop' (VRChat technique): scale the head joint to
        ~0 so the head mesh collapses and never blocks the eye-level camera.
        Joint scales inherit down the hierarchy, so eye/jaw joints under Head
        collapse too. The body stays fully visible when looking down."""
        s = 0.001 if hidden else 1.0
        self._scales[self.head_idx] = Gf.Vec3h(s, s, s)
        self.anim.GetScalesAttr().Set(Vt.Vec3hArray(self._scales))


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


_GRIP_DIAG_DONE = False

def _dump_grip_gestures(device):
    """One-shot: write all available input gestures on the device to a file."""
    global _GRIP_DIAG_DONE
    if _GRIP_DIAG_DONE or device is None:
        return
    _GRIP_DIAG_DONE = True
    _COMPONENTS = (
        "trigger", "squeeze", "grip", "select", "pinch",
        "thumbstick", "joystick", "trackpad", "stick",
        "a", "b", "x", "y", "menu", "system",
    )
    _GESTURES = ("value", "click", "touch", "force", "ready", "pose")
    lines = ["=== Available input gestures (right controller) ==="]
    for comp in _COMPONENTS:
        found = []
        for gest in _GESTURES:
            try:
                if device.has_input_gesture(comp, gest):
                    try:
                        val = device.get_input_gesture_value(comp, gest)
                        found.append(f"{gest}={val:.3f}")
                    except Exception:
                        found.append(gest)
            except Exception:
                pass
        if found:
            lines.append(f"  {comp}: {', '.join(found)}")
    try:
        with open(r"c:\World\Institut_Setup3\_grip_gestures.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
    except Exception:
        pass


def _grip_value(device):
    """Return the strongest 'grab' input on an XR controller as 0..1."""
    if device is None:
        return 0.0
    _dump_grip_gestures(device)
    best = 0.0
    _COMPS   = ("trigger", "squeeze", "grip", "select", "pinch")
    _GESTS   = ("value", "click", "force", "touch")
    for comp in _COMPS:
        for gesture in _GESTS:
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
        self._ik_scale_mult = 1.1 # manual fine-tune multiplier (tuned default)
        self._ik_enabled = False
        self._armvec_tick = 0
        try:
            open(r"c:\World\Institut_Setup3\_armvec_debug.txt", "w").close()
        except Exception:
            pass
        # Elbow pole bias. Down dominates so elbows hang down like a relaxed
        # human arm for the common reaches; outward (per-arm) + back keep the
        # elbow off the torso. All live-tunable via the UI buttons.
        self._pole_down = 2.0     # weight of downward component (dominant)
        self._pole_back = 1.0     # weight of backward (+Z) component
        self._pole_out  = 0.6     # weight of outward (sideways) component

        # Phase 1: One Euro Filter on the 3 tracked positions (head + 2 hands).
        self._smooth_on    = True
        self._smooth_cutoff = 1.0   # min_cutoff Hz: lower = less jitter, more lag
        self._smooth_beta   = 0.02  # higher = less lag during fast motion
        self._filt_head  = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)
        self._filt_rhand = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)
        self._filt_lhand = Vec3OneEuro(self._smooth_cutoff, self._smooth_beta)

        self._body_osc       = BodyOscReceiver(port=9000)
        self._xr_cam_path    = "/_xr/stage/xrCamera"

        # Locomotion: the right thumbstick moves the XR ORIGIN (camera rig);
        # the avatar follows the camera via _apply_camera_follow. Step in metres.
        self._move_step = 0.25
        # Right thumbstick locomotion: smooth glide per frame.
        self._stick_loco_on = True
        self._stick_speed   = 1.5    # metres per second at full deflection
        self._stick_deadz   = 0.15   # ignore small stick noise

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
        self._eye_fwd  = 0.10   # metres toward the face (avatar forward = -Z)
        self._hide_head_on = True

        # --- Grab (level-2 object interaction) ---
        # Button grab: hold the controller trigger (or grip) while the hand is
        # near an object to pick it up; release the button to set it down. This
        # allows precise placement (unlike the old flick-release).
        self._grab_enabled   = False
        self._grab_radius    = 0.15   # metres: hand must be this close to grab
        self._grab_threshold = 0.5    # trigger/grip value above which = "pressed"
        self._grab_candidates = []    # list of prim paths typed in UI
        self._grabbed = {True: None, False: None}   # is_right -> prim path or None
        self._grab_offset = {True: None, False: None}  # hand→object offset at grab
        self._hand_world  = {True: None, False: None}  # current hand world pos
        # Live forearm world rotation from the IK each frame — used as the hand's
        # parent frame so hand orientation tracks the arm (not just the T-pose).
        self._fore_world_live = {True: None, False: None}

        self._build_ui()

        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="avatar_xr_control.stage"
        )
        asyncio.ensure_future(self._deferred_init())

    def on_shutdown(self):
        self._stop_tracking()
        if self._body_osc:
            self._body_osc.stop()
        self._stage_sub = None
        self._skel      = None
        if self._window:
            self._window.destroy()
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
        for _ in range(300):
            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self._skel_path).IsValid():
                self._try_init(stage)
                return
            await asyncio.sleep(0.016)
        self._set_status("Timeout: Skeleton not found", error=True)

    def _try_init(self, stage):
        if self._skel is not None:
            return
        try:
            self._sanitize_camera_ops(stage)
        except Exception:
            pass  # repair is best-effort, must never block init
        try:
            self._skel = _AvatarSkel(stage, self._skel_path)
            self._set_status("Ready", error=False)
            self._dump_rest_world()
        except Exception as e:
            self._set_status(str(e), error=True)

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
        if sk is None:
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
            with open(r"c:\World\Institut_Setup3\_rest_world_debug.txt",
                      "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        except Exception as e:
            print(f"[avatar_xr_control] rest-world dump failed: {e}")

    def _apply_skel_path(self):
        self._skel_path = self._skel_path_field.model.get_value_as_string().strip()
        self._skel = None
        self._set_status("Reinitialising…", error=False)
        stage = omni.usd.get_context().get_stage()
        if stage:
            self._try_init(stage)
        else:
            asyncio.ensure_future(self._deferred_init())

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
                                      tooltip="Kopf-Mesh ausblenden (First-Person, 'head chop')")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Eye height -", clicked_fn=self._eye_up_dn, style=BTN)
                            ui.Button("Eye height +", clicked_fn=self._eye_up_up, style=BTN)
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

                # --- GRAB ------------------------------------------------------
                with ui.CollapsableFrame("Grab", collapsed=False, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        ui.Label("Grabbable prim paths (comma-separated):",
                                 height=14, style=S_LABEL)
                        self._grab_field = ui.StringField(height=22, style={"font_size": 10})
                        self._grab_field.model.set_value("/World/Cube")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Enable grab", clicked_fn=self._enable_grab, style=BTN)
                            ui.Button("Disable grab", clicked_fn=self._disable_grab, style=BTN)
                        self._grab_lbl = ui.Label(
                            "grab: off (hold trigger near object)", height=16, style=S_LABEL)

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

                # --- TUNING (advanced, collapsed) ------------------------------
                with ui.CollapsableFrame("Tuning (advanced)", collapsed=True, style=FRAME):
                    with ui.VStack(spacing=4, height=0):
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("IK reach -", clicked_fn=self._ik_scale_down, style=BTN)
                            ui.Button("IK reach +", clicked_fn=self._ik_scale_up, style=BTN)
                        self._ik_scale_lbl = ui.Label("IK reach mult: 1.10", height=16, style=S_LABEL)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Elbow down -", clicked_fn=self._pole_down_dn, style=BTN)
                            ui.Button("Elbow down +", clicked_fn=self._pole_down_up, style=BTN)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Elbow back -", clicked_fn=self._pole_back_dn, style=BTN)
                            ui.Button("Elbow back +", clicked_fn=self._pole_back_up, style=BTN)
                        self._pole_lbl = ui.Label("elbow pole: down 1.0 back 0.0",
                                                  height=16, style=S_LABEL)
                        ui.Line(style={"color": 0xFF333333})
                        ui.Button("Smoothing on/off", clicked_fn=self._toggle_smooth, style=BTN)
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Jitter -", clicked_fn=self._smooth_cutoff_dn, style=BTN,
                                      tooltip="less smoothing (more responsive)")
                            ui.Button("Jitter +", clicked_fn=self._smooth_cutoff_up, style=BTN,
                                      tooltip="more smoothing (less shaking)")
                        with ui.HStack(spacing=6, height=24):
                            ui.Button("Lag -", clicked_fn=self._smooth_beta_up, style=BTN,
                                      tooltip="less lag during fast motion")
                            ui.Button("Lag +", clicked_fn=self._smooth_beta_dn, style=BTN,
                                      tooltip="more lag, smoother")
                        self._smooth_lbl = ui.Label("", height=16, style=S_LABEL)
                        self._update_smooth_lbl()

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
        self._ik_scale_lbl.text = f"IK reach mult: {self._ik_scale_mult:.2f}"

    def _ik_scale_down(self):
        self._ik_scale_mult = max(0.1, self._ik_scale_mult - 0.1)
        self._ik_scale_lbl.text = f"IK reach mult: {self._ik_scale_mult:.2f}"

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

    def _enable_grab(self):
        raw = self._grab_field.model.get_value_as_string()
        self._grab_candidates = [p.strip() for p in raw.split(",") if p.strip()]
        self._grab_enabled = True
        self._grab_lbl.text = f"grab: ON ({len(self._grab_candidates)} objs)"
        self._grab_lbl.style = {"font_size": 11, "color": 0xFF44FF44}

    def _disable_grab(self):
        self._grab_enabled = False
        self._grabbed = {True: None, False: None}
        self._grab_offset = {True: None, False: None}
        self._grab_lbl.text = "grab: off"
        self._grab_lbl.style = {"font_size": 11, "color": 0xFFCCCCCC}

    def _step_origin(self, dx=0.0, dz=0.0):
        """Manual step: move the XR origin (camera rig) camera-relative.
        The avatar follows the camera via _apply_camera_follow."""
        if self._xr is None:
            ok, _ = self._init_xr()
            if not ok:
                self._set_track("no XR session — manual step needs XR", 0xFFFFAA33)
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
        self._eye_lbl.text = (
            f"eye height +{self._eye_up:.2f}m  fwd {self._eye_fwd:.2f}m  head: {head}")

    def _toggle_hide_head(self):
        self._hide_head_on = not self._hide_head_on
        if self._skel is not None:
            self._skel.set_head_hidden(self._hide_head_on)
        self._update_eye_lbl()

    def _eye_up_up(self):
        self._eye_up = min(0.40, self._eye_up + 0.02)
        self._update_eye_lbl()
        self._parent_camera_to_head()   # re-teleport so the change is felt now

    def _eye_up_dn(self):
        self._eye_up = max(-0.20, self._eye_up - 0.02)
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
            self._root_yaw += diff * (1.0 - math.exp(-self._follow_yaw_rate * 0.016))
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
        if self._follow_tick % 60 == 0:
            try:
                # Live eye via the full matrix path — must match cam in XZ.
                eye_rest = Gf.Vec3f(float(sk.head_rest_world[0]),
                                    float(sk.head_rest_world[1]) + self._eye_up,
                                    float(sk.head_rest_world[2]) - self._eye_fwd)
                live_eye = self._rest_to_world(eye_rest)
                with open(r"c:\World\Institut_Setup3\_follow_debug.txt",
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
        self._pole_lbl.text = (
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

    # --- Phase 1: One Euro smoothing controls ---
    def _update_smooth_lbl(self):
        state = "ON" if self._smooth_on else "OFF"
        self._smooth_lbl.text = (
            f"smoothing: {state}  cutoff {self._smooth_cutoff:.2f}Hz  beta {self._smooth_beta:.3f}")

    def _push_smooth_params(self):
        for f in (self._filt_head, self._filt_rhand, self._filt_lhand):
            f.min_cutoff = self._smooth_cutoff
            f.beta = self._smooth_beta

    def _toggle_smooth(self):
        self._smooth_on = not self._smooth_on
        # Reset filter state so toggling on doesn't jump from a stale value.
        for f in (self._filt_head, self._filt_rhand, self._filt_lhand):
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

    _CALIB_DEBUG_PATH = r"c:\World\Institut_Setup3\_calib_debug.txt"

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
            offset      = _quat_mul(rest_local, _quat_conj(local_tpose))
            if is_right:
                self._r_wrist_offset = offset
            else:
                self._l_wrist_offset = offset
            calibrated.append(side)

            chk = _quat_mul(offset, local_tpose)

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
                r_shoulder_xr = _sub(self._r_wrist_xr_tpose,
                                     _rot_y(Gf.Vec3f(+real_arm_len, 0.0, 0.0), cy))
                l_shoulder_xr = _sub(self._l_wrist_xr_tpose,
                                     _rot_y(Gf.Vec3f(-real_arm_len, 0.0, 0.0), cy))
                # Store the head→shoulder offsets YAW-NEUTRAL (user body
                # frame): each frame they are re-rotated by the live body yaw,
                # so the anchors follow both head position AND turning.
                self._r_shoulder_off = _rot_y(_sub(r_shoulder_xr, head_pos), -cy)
                self._l_shoulder_off = _rot_y(_sub(l_shoulder_xr, head_pos), -cy)
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
            self._calib_lbl.text  = "calibrated: " + "+".join(calibrated)
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
            self._set_track("No skeleton – open a stage first", 0xFF4444FF)
            return
        ok, msg = self._init_xr()
        self._set_track(msg, 0xFF44FF44 if ok else 0xFF4444FF)
        if not ok:
            return
        # First person: hide the head mesh while tracking (restored on stop).
        if self._hide_head_on:
            self._skel.set_head_hidden(True)
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
        if self._skel is not None:
            try:
                self._skel.set_head_hidden(False)
            except Exception:
                pass
        self._set_track("stopped", 0xFF888888)

    async def _tracking_loop(self):
        try:
            while self._xr_active:
                try:
                    try:
                        self._apply_camera_follow()
                    except Exception:
                        pass  # camera follow must never break tracking
                    self._apply_head()
                    self._apply_upper_body()
                    self._apply_hand_tracking()
                    try:
                        self._update_grab()
                    except Exception:
                        pass  # grab must never break tracking
                    try:
                        self._apply_stick_locomotion()
                    except Exception:
                        pass  # locomotion must never break tracking
                except Exception as e:
                    self._set_track(f"Error: {e}", 0xFF4444FF)
                    self._xr_active = False
                    break
                await asyncio.sleep(0.016)
        except asyncio.CancelledError:
            pass

    # ------------------------------------------------------------------
    # Pose application
    # ------------------------------------------------------------------


    def _apply_upper_body(self):
        # The OSC-driven waist lean was removed (the sideways tilt felt
        # unnatural); the waist keeps a static 5° forward tilt set once at
        # tracking start. Only the arm IK runs per frame.
        sk = self._skel
        if sk is None:
            return
        if self._ik_enabled:
            self._apply_arm_ik(is_right=True)
            self._apply_arm_ik(is_right=False)

    def _apply_arm_ik(self, is_right: bool):
        sk = self._skel
        dev = self._right_dev if is_right else self._left_dev
        if dev is None:
            return
        wrist_m = _get_pose_raw(dev)   # position only — physical space, no loco jump
        if wrist_m is None:
            return

        # Live shoulder anchor = current head position + calibrated head→shoulder
        # offset. Follows the user as they move/turn (not frozen at calibration).
        shoulder_off = self._r_shoulder_off if is_right else self._l_shoulder_off
        if shoulder_off is None:
            return
        head_m = _get_pose_raw(self._head_dev)  # position only
        if head_m is None:
            return
        head_pos = _vec3f(head_m.ExtractTranslation())
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
        # Shoulders rotate with the body around the head (offsets are stored
        # in the user's body frame by calibration).
        shoulder_xr = _add(head_pos, _rot_y(shoulder_off, yaw_now))

        shoulder = sk.r_shoulder_pos if is_right else sk.l_shoulder_pos
        len1 = sk.r_upperarm_len if is_right else sk.l_upperarm_len
        len2 = sk.r_forearm_len  if is_right else sk.l_forearm_len
        maxr = len1 + len2

        # Proportional shoulder-anchored mapping. Both the XR controller input
        # and the avatar rest data are now in the SAME true-world frame (rest
        # data via UsdSkelSkeletonQuery.ComputeJointWorldTransforms), so the
        # real arm vector maps directly — no frame transform, no axis negations.
        live_xr = _vec3f(wrist_m.ExtractTranslation())
        if self._smooth_on:
            live_xr = (self._filt_rhand if is_right else self._filt_lhand).filter(live_xr)
        # Counter-rotate by the ABSOLUTE body yaw: maps the raw-space arm
        # vector into the avatar's rest/local frame (user forward ↔ avatar
        # -Z); the root prim adds the world yaw via camera follow.
        arm_vec = _rot_y(_sub(live_xr, shoulder_xr), -yaw_now)
        s = self._ik_scale * self._ik_scale_mult
        arm_stage = _scale(arm_vec, s)
        target = _add(shoulder, arm_stage)

        # --- Diagnostic: log the RAW arm_vec (XR space) for the right arm once
        # per ~second, so the XR→stage axis mapping can be derived from real
        # reaches instead of guessed. Remove once mapping is confirmed.
        if is_right:
            self._armvec_tick = getattr(self, "_armvec_tick", 0) + 1
            if self._armvec_tick % 60 == 0:
                try:
                    with open(r"c:\World\Institut_Setup3\_armvec_debug.txt",
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

        # Pole hint: elbow points mostly DOWN (gravity-like), with a smaller
        # OUTWARD (away from body centre, per-arm sign) and BACK bias so it stays
        # off the torso. Down dominance makes elbows hang naturally for common
        # reaches. All weights live-tunable. Stage: right wrist at -X → out = -X.
        outward = (-1.0 if is_right else 1.0) * self._pole_out
        pole = Gf.Vec3f(outward, -self._pole_down, self._pole_back)
        elbow, wrist = _solve_two_bone_ik(shoulder, target, len1, len2, pole)

        if is_right:
            reach = _len(_sub(target, shoulder))
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

        # Live bone directions from the IK solution.
        upper_dir = _normalize(_sub(elbow, shoulder))
        fore_dir  = _normalize(_sub(wrist, elbow))

        upper_idx = sk.r_upper_idx if is_right else sk.l_upper_idx
        fore_idx  = sk.r_fore_idx  if is_right else sk.l_fore_idx
        upper_rest_q   = sk.r_upper_world_q_rest if is_right else sk.l_upper_world_q_rest
        upper_parent_q = sk.r_upper_parent_world_q if is_right else sk.l_upper_parent_world_q
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

        # The IK solves in the CACHED rest frame; the root prim has moved/yawed
        # since (camera follow), so map the wrist through the root delta to get
        # the live world position for grab distance checks.
        self._hand_world[is_right] = self._rest_to_world(wrist)

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

    @staticmethod
    def _prim_world_pos(prim):
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        return Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))

    def _set_prim_world_pos(self, prim, world_pos):
        """Set a prim's translate op so it lands at world_pos (accounts for parent)."""
        xform = UsdGeom.Xformable(prim)
        parent = prim.GetParent()
        local = world_pos
        if parent and parent.IsValid():
            pm = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            inv = pm.GetInverse()
            lp = inv.Transform(Gf.Vec3d(world_pos[0], world_pos[1], world_pos[2]))
            local = Gf.Vec3d(lp[0], lp[1], lp[2])
        translate_op = None
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break
        if translate_op is None:
            translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(local[0], local[1], local[2]))

    def _update_grab(self):
        if not self._grab_enabled:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        self._grab_diag_tick = getattr(self, "_grab_diag_tick", 0) + 1
        write_diag = (self._grab_diag_tick % 60 == 0)

        for is_right in (True, False):
            hand = self._hand_world.get(is_right)
            if hand is None:
                if write_diag and is_right:
                    try:
                        with open(r"c:\World\Institut_Setup3\_grab_debug.txt", "w", encoding="utf-8") as f:
                            f.write("hand_world is None — IK not running or not calibrated\n")
                    except Exception:
                        pass
                continue

            dev = self._right_dev if is_right else self._left_dev
            grip = _grip_value(dev)
            held = self._grabbed[is_right]
            # Hysteresis: grab needs grip >= threshold, release needs grip < 0.1.
            # Prevents one-frame flicker when the XR poll reads 0 between presses.
            if held is not None:
                pressed = grip >= 0.1
            else:
                pressed = grip >= self._grab_threshold

            if write_diag and is_right:
                try:
                    lines = [
                        f"grip_R={grip:.3f}  threshold={self._grab_threshold:.2f}  pressed={pressed}",
                        f"hand=({hand[0]:.3f}, {hand[1]:.3f}, {hand[2]:.3f})",
                    ]
                    for path in self._grab_candidates:
                        prim = stage.GetPrimAtPath(path)
                        if not prim.IsValid():
                            lines.append(f"  {path}: PRIM NOT FOUND")
                            continue
                        pos = self._prim_world_pos(prim)
                        d = _len(_sub(pos, hand))
                        lines.append(
                            f"  {path}: pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"
                            f"  dist={d:.3f}  radius={self._grab_radius:.3f}"
                            f"  {'IN RANGE' if d < self._grab_radius else 'too far'}"
                        )
                    with open(r"c:\World\Institut_Setup3\_grab_debug.txt", "w", encoding="utf-8") as f:
                        f.write("\n".join(lines) + "\n")
                except Exception:
                    pass

            if held is not None:
                # Holding: release the moment the trigger/grip is let go (precise
                # placement — the object stays exactly where it was set down).
                if not pressed:
                    self._grabbed[is_right] = None
                    self._grab_offset[is_right] = None
                    continue
                prim = stage.GetPrimAtPath(held)
                if not prim.IsValid():
                    self._grabbed[is_right] = None
                    continue
                off = self._grab_offset[is_right] or Gf.Vec3f(0, 0, 0)
                self._set_prim_world_pos(prim, _add(hand, off))
            else:
                # Not holding: only grab while the button is pressed AND the hand
                # is within radius of the nearest candidate.
                if not pressed:
                    continue
                best, best_d = None, self._grab_radius
                for path in self._grab_candidates:
                    # Skip if already held by the other hand.
                    if path == self._grabbed[not is_right]:
                        continue
                    prim = stage.GetPrimAtPath(path)
                    if not prim.IsValid():
                        continue
                    d = _len(_sub(self._prim_world_pos(prim), hand))
                    if d < best_d:
                        best, best_d = path, d
                if best is not None:
                    prim = stage.GetPrimAtPath(best)
                    self._grabbed[is_right] = best
                    # Preserve the hand→object offset at grab time so it doesn't snap.
                    self._grab_offset[is_right] = _sub(self._prim_world_pos(prim), hand)

    # ------------------------------------------------------------------
    # Finger-tracking diagnostic (read-only — writes _finger_diag.txt)
    # ------------------------------------------------------------------

    _FINGER_DIAG_PATH = r"c:\World\Institut_Setup3\_finger_diag.txt"

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

        verdict = ("FINGERS DETECTED — tracking data is arriving."
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
            self._set_track("No head pose — is XR session active?", 0xFFFFAA33)
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

    def _apply_hand_tracking(self):
        sk = self._skel
        if sk is None:
            return

        for dev, is_right in ((self._right_dev, True), (self._left_dev, False)):
            if dev is None:
                continue
            # Wrist: world→local via the forearm's LIVE world rotation (from the
            # IK this frame), then calibration offset, then axis remap. Using the
            # live forearm frame — not the frozen T-pose rest — keeps the hand
            # orientation matching the controller as the arm moves front/back.
            wrist_m = _get_pose(dev)
            if wrist_m is not None:
                q_wrist    = wrist_m.ExtractRotationQuat()
                q_wrist_f  = Gf.Quatf(float(q_wrist.GetReal()),
                                      float(q_wrist.GetImaginary()[0]),
                                      float(q_wrist.GetImaginary()[1]),
                                      float(q_wrist.GetImaginary()[2]))
                # Stage-space controller quat → avatar-local frame: undo the
                # root yaw (camera follow). The IK forearm frame below lives
                # in the avatar's rest/local frame.
                if self._root_yaw:
                    q_wrist_f = _quat_mul(_quat_conj(_yaw_quatf(self._root_yaw)),
                                          q_wrist_f)
                fore_parent = self._fore_world_live.get(is_right)
                if fore_parent is None:   # IK not run yet → fall back to rest
                    fore_parent = sk.r_fore_world_q_rest if is_right else sk.l_fore_world_q_rest
                local_q    = _quat_mul(_quat_conj(fore_parent), q_wrist_f)
                offset     = self._r_wrist_offset if is_right else self._l_wrist_offset
                if offset is not None:
                    local_q = _quat_mul(offset, local_q)
                final      = self._apply_hand_axes(local_q)
                hand_idx   = sk.r_hand_idx if is_right else sk.l_hand_idx
                sk.write_joint_rotation(hand_idx, final)
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
                quatd = m.ExtractRotationQuat()
                q = Gf.Quatf(
                    float(quatd.GetReal()),
                    float(quatd.GetImaginary()[0]),
                    float(quatd.GetImaginary()[1]),
                    float(quatd.GetImaginary()[2]),
                )
                sk.write_joint_rotation(idx, q)
