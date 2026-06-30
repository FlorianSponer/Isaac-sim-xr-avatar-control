"""Standalone extension: avatar finger-press interactions on machine parts.

A "switch" mesh is pressed with the avatar's CURLED index fingertip; that toggles
a set of target meshes (a sliding door) and rotates the switch as feedback. The
live avatar pose is read via the UsdSkel query (the Avatar XR Control extension
drives the skeleton) — there is no code dependency on that extension.

Add more interactions by appending dicts to INTERACTIONS below.
"""

import math

import omni.ext
import omni.usd
import omni.kit.app
from pxr import UsdGeom, UsdSkel, Gf, Usd

# ====================================================================
# CONFIGURATION
# ====================================================================
SKEL_PATH = ("/Root/female_adult_business_02/ManRoot/female_adult_business_02"
             "/female_adult_business_02/female_adult_business_02")

_INJ = ("/Root/Machine/injection_molding/tn__25260000000118_3D_VBA_XV08BE"
        "/tn__1317892_/tn__DES002691036_zH")

INTERACTIONS = [
    {
        "name": "injection_door",
        "switch": _INJ + "/Mesh_3152",       # press with a curled index fingertip
        "targets": [                          # these slide together
            _INJ + "/Mesh_1508",
            _INJ + "/Mesh_3815",
            _INJ + "/Mesh_2188",
            _INJ + "/Mesh_2167",
        ],
        "world_dist": -1.0,                   # door travel in metres
        "switch_axis": (0.0, 0.0, -1.0),      # switch rotates about -Z
        "switch_deg": 45.0,                   # switch rotation at full open
        "finger_radius": 0.06,                # fingertip proximity (m)
        "curl_deg": 35.0,                     # min index bend to count as a press
        "speed": 2.0,                         # door open/close ease rate
        "switch_speed": 25.0,                 # switch flip rate (high = near-instant)
    },
]


def _deinstance(prim):
    """Walk up and clear any instanceable flag so the prim's ops are editable."""
    a = prim
    while a and a.IsValid():
        if a.IsInstanceable():
            a.SetInstanceable(False)
        a = a.GetParent()


class _Switch:
    """One press-to-toggle interaction (switch mesh -> sliding targets)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ready = False
        self.phase = 0.0       # door slide 0 = closed, 1 = open (eased)
        self.sphase = 0.0      # switch flip phase (eased faster than the door)
        self.target = 0.0
        self.active = False    # was a press detected last frame (edge guard)
        self._targets = []     # [(translate_op, open_vec)]
        self._btn_op = None
        self._C = None
        self._pl2w = None
        self._pl2w_inv = None
        self._Tc = None
        self._Tnc = None

    def setup(self, stage):
        """Author the direct-driven ops. Returns True once all prims exist."""
        cfg = self.cfg
        switch = stage.GetPrimAtPath(cfg["switch"])
        if not switch.IsValid():
            return False
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())

        # --- target meshes: parent-space slide op (scale-aware) ---
        targets = []
        for p in cfg["targets"]:
            prim = stage.GetPrimAtPath(p)
            if not prim.IsValid():
                return False
            _deinstance(prim)
            xf = UsdGeom.Xformable(prim)
            op = next((o for o in xf.GetOrderedXformOps()
                       if o.GetOpName() == "xformOp:translate:door"), None)
            if op is None:
                op = xf.AddTranslateOp(opSuffix="door")
            xf.SetXformOpOrder([op] + [o for o in xf.GetOrderedXformOps()
                                       if o.GetOpName() != "xformOp:translate:door"])
            op.GetAttr().Clear()
            factor = xc.GetLocalToWorldTransform(prim).TransformDir(
                Gf.Vec3d(1, 0, 0)).GetLength()
            d = cfg["world_dist"] / max(factor, 1e-9)
            targets.append((op, Gf.Vec3d(d, 0, 0)))
            op.Set(Gf.Vec3d(0, 0, 0))
        self._targets = targets

        # --- switch: pure rotation about its world centre (op PREPENDED) ---
        _deinstance(switch)
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        self._C = Gf.Vec3d(
            bb.ComputeWorldBound(switch).ComputeAlignedRange().GetMidpoint())
        self._pl2w = xc.GetLocalToWorldTransform(switch.GetParent())
        self._pl2w_inv = self._pl2w.GetInverse()
        self._Tc = Gf.Matrix4d().SetTranslate(self._C)
        self._Tnc = Gf.Matrix4d().SetTranslate(-self._C)
        bxf = UsdGeom.Xformable(switch)
        bop = next((o for o in bxf.GetOrderedXformOps()
                    if o.GetOpName() == "xformOp:transform:btn"), None)
        if bop is None:
            bop = bxf.AddTransformOp(opSuffix="btn")
        bxf.SetXformOpOrder([bop] + [o for o in bxf.GetOrderedXformOps()
                                     if o.GetOpName() != "xformOp:transform:btn"])
        bop.Set(Gf.Matrix4d(1.0))
        self._btn_op = bop

        self.ready = True
        print(f"[machine.interaction] '{cfg['name']}' ready")
        return True

    def _pressed(self, xforms, hands):
        """True if a curled index fingertip is within range of the switch."""
        cfg = self.cfg
        r = cfg["finger_radius"]
        for i1, i2, i3 in hands:
            tip = Gf.Vec3d(xforms[i3].ExtractTranslation())
            if (tip - self._C).GetLength() >= r:
                continue
            if i1 is None or i2 is None:           # no curl data -> proximity only
                return True
            p1 = Gf.Vec3d(xforms[i1].ExtractTranslation())
            p2 = Gf.Vec3d(xforms[i2].ExtractTranslation())
            p3 = Gf.Vec3d(xforms[i3].ExtractTranslation())
            v1, v2 = (p2 - p1), (p3 - p2)
            if v1.GetLength() < 1e-6 or v2.GetLength() < 1e-6:
                continue
            bend = math.degrees(math.acos(max(-1.0, min(1.0,
                   Gf.Dot(v1.GetNormalized(), v2.GetNormalized())))))
            if bend >= cfg["curl_deg"]:
                return True
        return False

    def update(self, xforms, hands, dt):
        cfg = self.cfg
        a = self._pressed(xforms, hands)
        if a and not self.active:               # rising edge -> toggle
            self.target = 1.0 - self.target
            print(f"[machine.interaction] '{cfg['name']}' ->",
                  "OPEN" if self.target > 0.5 else "CLOSE")
        self.active = a
        # door slides at the slow rate; the switch flips at its own fast rate
        self.phase  += (self.target - self.phase)  * min(1.0, cfg["speed"] * dt)
        self.sphase += (self.target - self.sphase) * min(1.0,
                        cfg.get("switch_speed", 25.0) * dt)
        for op, ov in self._targets:
            op.Set(ov * self.phase)
        axis = Gf.Vec3d(*cfg["switch_axis"])
        R = Gf.Matrix4d().SetRotate(Gf.Rotation(axis, cfg["switch_deg"] * self.sphase))
        self._btn_op.Set(self._pl2w * (self._Tnc * R * self._Tc) * self._pl2w_inv)

    def reset(self):
        """Snap closed + un-rotate (on shutdown)."""
        try:
            for op, _ in self._targets:
                op.Set(Gf.Vec3d(0, 0, 0))
            if self._btn_op is not None:
                self._btn_op.Set(Gf.Matrix4d(1.0))
        except Exception:
            pass


class MachineInteractionExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        self._switches = [_Switch(c) for c in INTERACTIONS]
        self._cache = None
        self._skq = None
        self._hands = []
        self._err = False
        self._sub = omni.kit.app.get_app().get_update_event_stream() \
            .create_subscription_to_pop(self._on_update, name="machine_interaction")
        print("[machine.interaction] started")

    def on_shutdown(self):
        if getattr(self, "_sub", None) is not None:
            self._sub.unsubscribe()
            self._sub = None
        for s in getattr(self, "_switches", []):
            s.reset()
        print("[machine.interaction] stopped")

    def _build_skel(self, stage):
        prim = stage.GetPrimAtPath(SKEL_PATH)
        if not prim.IsValid():
            return False
        self._cache = UsdSkel.Cache()
        skq = self._cache.GetSkelQuery(UsdSkel.Skeleton(prim))
        if not skq:
            return False
        order = [str(j) for j in skq.GetJointOrder()]

        def fidx(suf):
            return next((i for i, j in enumerate(order) if j.endswith(suf)), None)

        hands = []
        for s in ("R", "L"):
            i3 = fidx(s + "_Index3")
            if i3 is not None:
                hands.append((fidx(s + "_Index1"), fidx(s + "_Index2"), i3))
        if not hands:
            return False
        self._skq = skq
        self._hands = hands
        print(f"[machine.interaction] skeleton ready, index chains: {hands}")
        return True

    def _on_update(self, e):
        if self._err:
            return
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            # (re)build the skeleton query if missing or the stage changed
            if self._skq is None or not stage.GetPrimAtPath(SKEL_PATH).IsValid():
                self._skq = None
                for s in self._switches:
                    s.ready = False
                if not self._build_skel(stage):
                    return
            # lazily set up switches once their prims are loaded
            for s in self._switches:
                if not s.ready:
                    s.setup(stage)
            if not any(s.ready for s in self._switches):
                return
            dt = e.payload.get("dt", 0.016)
            xforms = self._skq.ComputeJointWorldTransforms(
                UsdGeom.XformCache(Usd.TimeCode.Default()))
            for s in self._switches:
                if s.ready:
                    s.update(xforms, self._hands, dt)
        except Exception as ex:
            self._err = True
            print(f"[machine.interaction] disabled after error: {ex}")
