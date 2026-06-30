# Avatar XR Control — Project Documentation & Report Source

> **Purpose of this file.** It serves two roles:
> 1. A living progress/status log for ongoing development (Part A).
> 2. A structured knowledge base from which a 6–10 page university project
>    report can be written in a later session (Part B contains the report
>    outline, all technical content, figures-to-make, and references).
>
> A future session should be able to open this file and produce the full
> report without needing the running headset — every logic, formula, design
> decision, dead-end, and resource is recorded here.

---

## ⚠ KEY INSIGHT (root cause of the long IK axis saga)
The `_AvatarSkel._rest_world` transforms are in **skeleton-local space** — they
do NOT include the transform of the root prims ABOVE the skeleton
(`/Root/female_adult_business_02/ManRoot/...`). Those root prims carry a
rotation. Measured skeleton→world axis images:
```
skel local +X -> world (-1.00, +0.06,  0.00)
skel local +Y -> world ( 0.00,  0.00, +1.00)
skel local +Z -> world (+0.06, +1.00,  0.00)
```
So skel +Y = world +Z, skel +Z = world +Y, skel +X = world −X.
All the IK math runs in skeleton-local space, but the XR controller input is in
world space → ~90°+flip mismatch that smeared every commanded direction. Months
of "swap X/Y, negate Z" patches were compensating for THIS, never fixing it.
**Fix:** transform the world-space XR arm vector into skeleton-local via the
cached `world_to_skel` matrix before building the IK target. One transform at
the boundary; rest data stays in its native (working) frame.

---

## ⚠ KEY INSIGHT #2 (collider placement — the physics-layer saga)
The avatar is posed through a bound **`SkelAnimation`** (`write_joint_rotation`
sets the anim's rotation array). The skinned **mesh** deforms from that, but the
joint **prim** transforms (the `OmniJoint` prims) stay at **bind pose** — they
never move. So `ComputeLocalToWorldTransform` on a bone prim only reflects the
ROOT motion, not the live pose. The body-collider proxies must therefore be
driven from `UsdSkelSkeletonQuery.ComputeJointWorldTransforms(..., atRest=False)`
(render-faithful posed joints), **not** from the bone prims and **not** from the
cached-rest reconstruction (which ignores the live spine/clavicle pose and
drifts up to ~0.5 m at the hand). Same lesson as insight #1: drive everything
from the one frame the mesh actually uses.

---

# PART A — CURRENT PROJECT STATUS

## A.1 One-line summary
An NVIDIA Omniverse / Isaac Sim Kit extension (`avatar.xr.control`) that drives a
UsdSkel-rigged avatar in real time from a Meta Quest headset (OpenXR). Head,
arms (analytic two-bone IK), and hand orientation track the HMD + controllers;
the legs are animated by a procedural walking gait; and the body interacts with
the scene through a kinematic PhysX collider layer (push and grab dynamic
objects). Per-frame joint rotations go to a bound `SkelAnimation` prim. Now
~6,400 LOC across the main module + `crouch_sit.py` + tests.

## A.2 What works right now
| Capability | Input source | Method | Status |
|---|---|---|---|
| Head (yaw/pitch/roll) | OpenXR HMD pose | corrected XR quat → direct local write | ✅ working |
| Arms (shoulder/elbow/wrist position) | OpenXR controller position + 2-bone IK | analytic IK in skeleton-local frame, clavicle follow, adaptive elbow pole/swivel | ✅ validated |
| Hand / wrist orientation | OpenXR controller pose | right-multiplied controller-frame offset + forearm-roll routing (no wrist clamp) | ✅ validated 2026-06-22 |
| Fingers | controller trigger/grip | procedural curl (optical finger data unavailable with controllers) | ✅ (procedural) |
| Legs / walking | avatar root velocity | procedural Route-B gait: foot-placement IK, pelvis drop, heel/toe roll + floor-clearance lift | ✅ on by default |
| Crouch & sit | HMD height vs calibrated standing height | `CrouchSitController` + leg bend / pelvis drop | ✅ |
| First-person view | OpenXR HMD | eye-point camera follow + non-deforming head-region hide | ✅ |
| Physics interaction | kinematic collider proxies tracking posed joints | torso/head/arm/palm colliders push dynamic rigid bodies; trigger-grab (dynamic RB only) | ✅ on by default |
| Torso lean via OSC | ALVR VRChat Body-OSC | UDP receiver retained but reserved/optional | 🟡 optional |
| Locomotion | right thumbstick | glide XR rig; avatar follows via camera-follow | ✅ |

## A.3 Recently completed / active work
- **Coordinate-frame fix (insight #1):** arm IK now solves in skeleton-local
  space via the cached `world_to_skel` transform — ended the long axis saga.
- **Validated hand feel** (follow ~1.7 cm; wrist = right-multiplied
  controller-frame offset, forearm-roll routing, no wrist clamp).
- **Procedural walking** (Route B gait IK) with foot-placement IK, pelvis drop,
  and foot roll; foot target now lifted during the roll so heel/toe don't clip
  the floor.
- **PhysX collision layer** on by default: kinematic proxies (torso elliptic
  cylinder, head, arm capsules, **palm spheres**) driven from the live SkelQuery
  joints (insight #2) so they match the skinned mesh and push dynamic objects;
  torso/head proxies follow the crouch.
- **Grab** reworked: kinematic handoff while held; only **dynamic rigid bodies**
  are grabbable (instance proxies / static scenery skipped).
- **First-person head hide** (region cutout default, head-chop fallback) and an
  eye forward-offset so the near plane clears the face.

## A.4 Possible next steps
1. Optical hand-tracking mode for real fingers (controllers off).
2. Real lower-body tracking via SlimeVR/Vive trackers (the OSC path is stubbed).
3. Automatic rig retargeting (remove hard-coded `JOINT_MAP` assumptions).
4. Networked multi-user embodiment.

## A.5 Known hard constraints (hardware)
- Quest 2 + controllers = **3 tracked points** (head + 2 hands); lower body is
  inferred (procedural walk / crouch), not tracked.
- Optical finger tracking and controllers are **mutually exclusive**; with
  controllers in hand there is no finger-pose data (observed 0/15 poses), hence
  the procedural finger curl.

---

# PART B — PROJECT REPORT SOURCE MATERIAL

*(Everything below is written so a later session can expand it into a 6–10 page
report. Target length per section is noted. Suggested final structure follows a
standard technical report: Abstract → Introduction → Background → System
Architecture → Implementation → Evaluation → Discussion → Conclusion →
References.)*

## B.0 Report metadata / front matter
- **Working title:** *Real-Time VR Avatar Embodiment in NVIDIA Omniverse:
  Driving a UsdSkel Character from Consumer Headset Tracking.*
- **Author:** Florian Sponer.
- **Context:** University project (TU — Institut Setup series).
- **Artifact:** `avatar.xr.control` Omniverse Kit extension (~6,400 LOC; main
  module + `crouch_sit.py` + tests), plus an optional `machine_interaction`
  extension.
- **Suggested figures:** (F1) system data-flow diagram; (F2) coordinate-frame
  comparison (OpenXR vs OSC vs Stage); (F3) UsdSkel joint hierarchy excerpt;
  (F4) 2-bone IK geometry diagram; (F5) screenshots of avatar following user;
  (F6) calibration UI panel.

## B.1 Abstract (target: ½ page)
Summarise: goal (embody a user in a high-fidelity Omniverse avatar with
consumer VR), approach (OpenXR for head/hands, OSC for torso, analytic IK for
arms, SkelAnimation for output), key findings (what 3-point tracking can and
cannot drive; coordinate-frame reconciliation as the central engineering
problem), and outcome (working head/torso/hand-rotation + an IK arm prototype).

## B.2 Introduction & Motivation (target: 1 page)
- **Problem:** Social VR / telepresence / digital-twin avatars need believable,
  low-latency full-body motion, but consumer headsets expose only sparse
  tracking. Bridging sparse input to a fully-rigged production avatar is
  non-trivial.
- **Goal:** Take a photorealistic Reallusion Character-Creator avatar
  (`F_Business_02`) inside NVIDIA Omniverse and make it mirror the user's real
  movements in real time, viewed first-person in VR.
- **Scope:** Full-body embodiment from 3-point tracking — head, arms (IK), hand
  orientation, procedural finger curl, procedural walking + crouch for the legs,
  and a kinematic physics layer for object interaction. Real finger and true
  lower-body *tracking* remain out of scope (hardware limits).
- **Contributions:**
  1. A self-contained Kit extension requiring no third-party Python packages.
  2. A documented, reproducible coordinate-frame reconciliation between OpenXR,
     ALVR VRChat-OSC, and the USD stage.
  3. An analysis of what consumer 3-point tracking can faithfully drive, with a
     pivot from (failed) OSC elbow tracking to (working) OpenXR-wrist 2-bone IK.

## B.3 Background & Related Technologies (target: 1–1.5 pages)
Explain each building block (cite in B.10):
- **NVIDIA Omniverse / Kit SDK:** extension architecture (`omni.ext.IExt`,
  `extension.toml`, hot-reload), USD as scene description.
- **OpenUSD & UsdSkel:** `Skeleton`, joint token paths, `restTransforms`,
  `BindingAPI`, `SkelAnimation` (joints / translations / rotations / scales
  arrays). Why animation is written to a bound `SkelAnimation` prim rather than
  mutating joints directly.
- **OpenXR:** the device/pose model; grip pose conventions; how `omni.kit.xr.core`
  surfaces input devices (`/user/head`, `/user/hand/left|right`) and
  per-finger virtual poses.
- **ALVR:** streams Quest↔PC over Wi-Fi; exposes the headset to SteamVR/OpenXR;
  separately offers a **VRChat Body-OSC sink** that emits estimated tracker
  positions over UDP.
- **OSC (Open Sound Control):** minimal binary message format (address string,
  type tag, big-endian float args) — relevant because a hand-rolled parser was
  written (`_parse_osc`).
- **Reallusion Character Creator rig (`RL_BoneRoot`):** bone naming and the
  joint hierarchy used by the avatar.
- **Quaternions & rotations:** brief primer (Hamilton product, conjugate,
  shortest-arc between vectors) since the whole system is quaternion-based.
- **Inverse Kinematics:** forward vs inverse kinematics; the analytic two-bone
  (law-of-cosines) solution vs iterative solvers (FABRIK/CCD); pole vectors.

## B.4 System Architecture (target: 1–1.5 pages) — see Figure F1
**Two independent input channels feed one per-frame update loop (16 ms /
`asyncio.sleep(0.016)`):**

1. **OpenXR channel** (`omni.kit.xr.core`): head pose, hand/controller poses,
   per-finger poses. Pulled synchronously each tick.
2. **OSC channel** (`BodyOscReceiver`): a background daemon thread binds a UDP
   socket on `0.0.0.0:9000`, parses ALVR VRChat-OSC `/position` messages, and
   stores the latest position per tracker in a lock-protected dict.

**Per-tick pipeline** (`_tracking_loop` → `_apply_head`, `_apply_upper_body`
[+ `_apply_arm_ik`], `_apply_hand_tracking`): each function computes joint
quaternions and calls `_AvatarSkel.write_joint_rotation`, which overwrites the
full `QuatfArray` on the `SkelAnimation` prim. Rest pose is cached at init;
each frame replaces only the driven joints' rotations.

**Output:** the bound `SkelAnimation` ("xr_anim") prim → USD evaluates the
skinned mesh → rendered to the VR view (first-person camera teleported to the
avatar head).

**Class/function map:**
- `_AvatarSkel` — wraps the USD skeleton; resolves joint indices, caches rest
  world transforms/positions, bone lengths, parent rotations; owns the
  `SkelAnimation` writes.
- `BodyOscReceiver` — threaded UDP OSC receiver + `_parse_osc`.
- `AvatarXRControlExtension(omni.ext.IExt)` — lifecycle, UI panel, XR session
  control, calibration, the three per-frame apply methods, IK.
- Math helpers — `_quat_mul`, `_quat_conj`, `_quat_from_to`,
  `_bone_rotation_from_vectors`, `_correct_xr_quat`, `_solve_two_bone_ik`,
  vector ops (`_sub/_add/_scale/_dot/_len/_normalize/_vec3f`).

## B.5 The Central Problem — Coordinate-Frame Reconciliation (target: 1.5 pages) — Figure F2
This is the engineering heart of the report. **Three frames had to be aligned:**

| Frame | Convention | Origin |
|---|---|---|
| Avatar **Stage** (USD, viewed from behind) | Y+ up, X+ right, Z+ **backward** (avatar faces −Z) | stage origin |
| **ALVR OSC** (Unity-style) | Y+ up, X+ right, Z+ **forward** | player playspace |
| **OpenXR** device pose | right-handed; mapped to stage via correction | XR/stage origin (large offset) |

Key reconciliations (each is a concrete, documented result):
- **Avatar faces −Z**, so the first-person camera uses identity rotation with a
  +Z/+Y offset from the head joint (`_parent_camera_to_head`).
- **OSC → Stage:** negate Z in `BodyOscReceiver` (ALVR forward = stage backward).
  (Earlier, when the avatar faced +Z, X was negated instead — a documented
  pivot when the avatar was rotated 180°.)
- **OpenXR quaternion → Stage:** `_correct_xr_quat` negates X and Z (equivalent
  to 180° about Y) so head & wrist orientations match.
- **Why the waist tolerated mistakes the arms didn't:** the spine direction is
  Y-dominant, so a 180°-about-Y error is nearly invisible there; arm directions
  are X-dominant, so the same error flipped them — this explains a long
  debugging arc and is a good teaching point in the report.
- **IK position mapping:** OpenXR wrist delta from the T-pose anchor is mapped to
  stage by (−x, −y, −z) × scale.

**Lesson to articulate:** most "bugs" were not math errors but unstated frame
assumptions; the fix pattern was to make every transform explicit and apply each
correction *once, at the boundary* (e.g. the single Z-flip in the receiver)
rather than scattering per-call negations.

## B.6 Implementation Details (target: 1.5–2 pages)
Break into subsections; pull formulas/snippets from the code.

### B.6.1 Skeleton wrapper & SkelAnimation output
- Resolve joints via `JOINT_MAP` (logical name → `RL_BoneRoot/...` token path).
- Cache `restTransforms`; compute world transforms by walking parents
  (`_compute_world_transforms`, `_build_parents`).
- Bind a `SkelAnimation` and write rotations each frame.

### B.6.2 OSC receiver & parser
- Minimal OSC parsing: address string (null-terminated, 4-byte aligned), type
  tag string, big-endian `>f` floats. `_BODY_TRACKER_MAP` maps tracker IDs 1–8.
- Threading + lock; non-blocking via `settimeout`.

### B.6.3 Head & torso
- Head: corrected XR quat written directly to the head joint.
- Waist: `spine_dir = hip − chest` (Z-corrected) compared to rest spine
  direction via shortest-arc; small fixed 5° tilt correction.

### B.6.4 Hand rotation + T-pose calibration
- Pipeline: raw XR quat → `_correct_xr_quat` → world→local via forearm rest
  (`conj(fore_rest)·q`) → calibration offset → fixed axis remap.
- **Calibration math:** capture `local_tpose` in T-pose; compute
  `offset = rest_local · conj(local_tpose)` so the T-pose maps exactly to the
  skeleton rest; verified by `offset·local_tpose == rest_local`.
- 10-second guided countdown UI so the user can don the headset and hold pose.

### B.6.5 Analytic 2-bone IK (arms) — Figure F4
- Inputs: shoulder (root), wrist target, upper-arm & forearm lengths, pole hint.
- **Law of cosines:** with `d` = root→target distance clamped to
  `[|l1−l2|, l1+l2]`, the elbow's projection along the target axis is
  `a = (l1² − l2² + d²) / (2d)`, and its perpendicular offset is
  `h = √(l1² − a²)`. Elbow = base + h·(bend direction), where bend is the pole
  vector projected perpendicular to the limb axis.
- Resulting upper-arm and forearm direction vectors are turned into joint
  rotations via `_bone_rotation_from_vectors`, with the forearm parented to the
  upper arm's *new* world rotation so the bend composes correctly.
- **Target construction:** `target = wrist_rest + scale·(−dx,−dy,−dz)` from the
  XR wrist delta; `scale = (avatar span / xr span) × manual multiplier`.

### B.6.6 User interface
Kit `omni.ui` panel: skeleton path field, XR session start/stop, tracking
start/stop, live status readout, IK reach −/+ tuning, hand calibration with
countdown. Useful as a debugging/operator console — worth a screenshot (F6).

## B.7 Evaluation / Results (target: 1 page)
- **Qualitative:** head and torso track convincingly; hand rotation matches
  after calibration; arm IK follows hand position once reach is scaled (report
  the final multiplier and residual issues).
- **Quantitative options:** measured wrist spans (XR ≈1.81 m vs avatar ≈1.32 m
  → scale ≈0.73); arm reach 0.48 m; loop rate 16 ms target (~60 Hz); latency
  qualitative.
- **Failure analysis (important, honest):**
  - OSC elbow/knee/foot trackers are IK estimates → barely move → unusable.
  - Finger tracking yields 0/15 poses with controllers (mutual exclusivity).
  - IK reach clamping when scale derived from span not arm length.

## B.8 Discussion (target: ½–1 page)
- **What sparse tracking can faithfully drive:** anything well-constrained by
  the 3 real points (head, torso-from-HMD, hand rotation, arm-via-IK); not
  free limbs (elbow, knees, fingers without optical tracking).
- **Design trade-offs:** controllers (stable wrist, no fingers) vs hand-tracking
  (fingers, unstable wrist when out of FOV).
- **Generalisation:** the frame-reconciliation methodology and SkelAnimation
  output approach transfer to any UsdSkel avatar / other OpenXR runtimes.
- **Limitations:** single avatar rig hard-coded paths; no leg IK; calibration
  assumes an accurate T-pose.

## B.9 Conclusion & Future Work (target: ½ page)
- Recap contributions.
- Future: leg IK / locomotion, body-tracker hardware (SlimeVR/Vive) for true
  limbs, hand-tracking mode for fingers, automatic rig retargeting, smoothing/
  filtering for jitter, networked multi-user.

## B.10 Resources & References (to cite properly in the report)
**Software / SDKs used**
- NVIDIA Omniverse Kit SDK & USD Composer (`my_company.my_usd_composer.kit`).
- OpenUSD / `pxr` (Usd, UsdSkel, Gf, Vt).
- `omni.kit.xr.core`, `omni.ui`, `omni.usd`, `omni.ext`.
- ALVR (streamer v20.14.0) — Quest streaming + VRChat Body-OSC sink.
- SteamVR / OpenXR runtime.
- Python 3.11 stdlib: `asyncio`, `socket`, `struct`, `threading`, `math`.

**Hardware**
- Meta Quest 2 (head + 2 controllers; 3-point tracking).
- PC w/ NVIDIA RTX 4070 Laptop GPU (from stats screenshot).

**Assets**
- `F_Business_02` — Reallusion Character Creator female business avatar
  (USDC + BaseColor/Normal/ORM PBR textures), collected from NVIDIA asset S3.

**Concepts / external references to cite**
- OpenXR specification — semantic paths & grip pose conventions (Khronos).
- OpenUSD UsdSkel schema documentation.
- Open Sound Control 1.0 specification.
- Two-bone analytic IK (law of cosines); pole-vector elbow control.
- Quaternion rotation mathematics.

## B.11 Appendices (optional)
- A: full `JOINT_MAP` and the RL bone hierarchy.
- B: the coordinate-correction table and the empirical axis-mapping log.
- C: calibration debug output sample (`_calib_debug.txt`).
- D: key code listings (IK solver, calibration, OSC parser).

---

## Development changelog (chronological, for the "debugging journey" narrative)
1. Base extension: head + waist + hand-rotation + fingers via SkelAnimation.
2. Added OSC elbow→upper-arm aiming (delta + orthonormal-basis methods);
   fought L/R asymmetry, axis swaps, 90°/180° offsets.
3. Discovered avatar faced wrong way; rotated avatar 180°, re-derived all
   corrections; centralised OSC X/Z flips in the receiver.
4. Hand orientation dialed in (axis remap + T-pose calibration offset).
5. Diagnosed that OSC elbow data is unusable (IK guesses, ~frozen); confirmed
   only 3 real tracking points exist; removed OSC arm tracking.
6. Confirmed fingers need optical hand tracking (0/15 with controllers).
7. Pivoted arms to OpenXR-wrist analytic 2-bone IK (current work): added arm
   rest data + bone lengths, IK solver, T-pose anchor + scale calibration,
   per-frame `_apply_arm_ik`, live reach-multiplier tuning.
8. Cleanups: removed dead basis/preset/discovery scaffolding between milestones.
9. Solved the arm-axis saga (insight #1): solve IK in skeleton-local space via
   `world_to_skel`; recomputed scale from arm length; validated hand feel.
10. Added first-person head hide (region cutout, head-chop fallback) + eye offset.
11. Added the PhysX self-collision layer (analytic torso ellipse + scene-query
    proxies) to keep arms/hands off the body during IK.
12. Added procedural walking (Route B gait IK): foot-placement IK, pelvis drop,
    heel/toe roll, plus a floor-clearance lift so the feet don't clip the ground.
13. Added crouch & sit (`CrouchSitController`, unit-tested) from HMD height.
14. Object interaction: kinematic body colliders push dynamic rigid bodies; added
    per-hand palm colliders; trigger-grab with kinematic handoff.
15. Collider placement moved to the live SkelQuery joints (insight #2) so the
    proxies track the skinned mesh; torso/head follow the crouch.
16. Grab restricted to dynamic rigid bodies (skips instanced/static scenery).
17. Published to GitHub (`FlorianSponer/Isaac-sim-xr-avatar-control`); bundled an
    optional example avatar under `examples/avatar/newAva.usd`.

## File / code reference (for whoever writes the report)
- Main module: `exts/avatar_xr_control/avatar/xr/control/extension.py` (~6,400 LOC).
- Crouch/sit logic: `exts/avatar_xr_control/avatar/xr/control/crouch_sit.py`
  (+ `tests/test_crouch_sit.py`).
- Manifest: `exts/avatar_xr_control/config/extension.toml`
  (deps: `omni.kit.uiapp`, `omni.ui`, `omni.usd`, `omni.kit.xr.core`, `omni.physx`).
- Optional second extension: `exts/machine_interaction/` (finger-press triggers
  machine animations).
- GitHub repo: https://github.com/FlorianSponer/Isaac-sim-xr-avatar-control
- Example avatar: `examples/avatar/newAva.usd` (UV-grid placeholder material).
- Working avatar asset: `Collected_F_Business_02/F_Business_02.usd` (+ textures;
  not committed — large).
- Backups: `backups/extension_*.py`.
- Debug dumps: `_calib_debug.txt`, `_grab_debug.txt`, `_follow_debug.txt`, etc.
