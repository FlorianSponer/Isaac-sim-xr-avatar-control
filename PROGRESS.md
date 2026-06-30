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

# PART A — CURRENT PROJECT STATUS

## A.1 One-line summary
An NVIDIA Omniverse Kit extension (`avatar.xr.control`) that drives a
UsdSkel-rigged avatar in real time from a Meta Quest VR headset (via ALVR +
OpenXR) and ALVR's VRChat Body-OSC stream, writing per-frame joint rotations
to a bound `SkelAnimation` prim.

## A.2 What works right now
| Body part | Input source | Method | Status |
|---|---|---|---|
| Head (yaw/pitch/roll) | OpenXR HMD pose | `_correct_xr_quat` → direct local write | ✅ working |
| Waist / torso lean | ALVR OSC hip + chest positions | shortest-arc `_bone_rotation_from_vectors` + 5° X tilt | ✅ working (when OSC stream live) |
| Hand rotation (wrist) | OpenXR controller/hand pose | world→local via forearm-rest, T-pose calibration offset, fixed axis remap | ✅ working (user pre-corrected incoming data) |
| Arm position (shoulder+elbow) | OpenXR wrist position + 2-bone IK | analytic IK, T-pose anchored, scaled | 🟡 in tuning (reach scale + pole) |
| Fingers (30 joints) | OpenXR hand-tracking joint poses | direct world-quat write | ❌ no data on Quest 2 w/ controllers |
| Legs / feet / knees | — | — | ❌ not implemented |

## A.3 Active work — 2-bone arm IK (in tuning)
- **Implemented:** analytic 2-bone IK (`_solve_two_bone_ik`), shoulder/elbow/
  wrist rest data + bone lengths in `_AvatarSkel`, T-pose calibration capture of
  the XR wrist anchor and a span-derived scale, per-frame `_apply_arm_ik`.
- **Axis mapping found empirically:** stage = (−x, −y, −z) × scale relative to
  the OpenXR wrist delta from the T-pose anchor (180°-about-Y plus a Y flip
  discovered in testing).
- **Open tuning issues:**
  1. **Reach over-scales → constant CLAMP.** Calibration scale (≈0.73) came from
     wrist-to-wrist *span*, which includes shoulder width and overshoots arm
     length. Live data: a 0.98 m hand move maps past the 0.48 m avatar arm and
     clamps. A live **IK reach multiplier** (−/+ buttons, `_ik_scale_mult`) was
     added to dial it down (~0.5–0.6 expected).
  2. **Elbow pole** hint is a fixed guess `(0,−1,0.5)`; may need tuning.
  3. **Wrist-rotation/forearm coupling:** wrist rotation uses the forearm rest
     world rotation as parent; now that the forearm moves via IK, hand rotation
     can drift when the arm bends — to revisit.

## A.4 Immediate next steps
1. Lock the IK reach multiplier value, bake it as the default.
2. Tune elbow pole direction.
3. Recompute calibration scale from arm length (shoulder→wrist) not wrist span.
4. Re-couple wrist rotation to the IK-updated forearm world rotation.

## A.5 Known hard constraints (hardware)
- Quest 2 + controllers = **3 tracked points** (head + 2 hands). Hips/chest are
  HMD-derived and usable; **elbows/knees/feet from OSC are IK guesses that barely
  move** — unusable, which is why arms are now driven by OpenXR wrist IK instead.
- Optical finger tracking and controllers are **mutually exclusive**; with
  controllers in hand there is no finger-pose data (observed: 0/15 poses).

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
- **Artifact:** `avatar.xr.control` Omniverse Kit extension (~1070 LOC, single
  Python module).
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
- **Scope:** Upper-body embodiment (head, torso lean, arms, hands); fingers and
  legs scoped out due to hardware limits.
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

## File / code reference (for whoever writes the report)
- Main module: `exts/avatar_xr_control/avatar/xr/control/extension.py` (~1070 LOC).
- Manifest: `exts/avatar_xr_control/config/extension.toml`
  (deps: `omni.kit.uiapp`, `omni.ui`, `omni.usd`, `omni.kit.xr.core`).
- Avatar: `Collected_F_Business_02/F_Business_02.usd` (+ textures).
- Backups: `backups/extension_*_pre_ik.py` (pre-IK working snapshot).
- Calibration debug output: `_calib_debug.txt`.
