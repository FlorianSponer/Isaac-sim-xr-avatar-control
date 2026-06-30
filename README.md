# Isaac Sim XR Avatar Control

Embodied VR avatar for NVIDIA Isaac Sim. The extension drives a UsdSkel avatar
in real time from a VR headset and its controllers:

- **Head tracking** — the avatar's head follows the HMD orientation.
- **Arm tracking** — analytic two-bone inverse kinematics positions both arms
  from the controller poses, anchored to calibrated shoulder positions.
- **Hand rotation** — controller orientation drives the wrists, aligned via a
  one-time T-pose calibration (facing direction does not matter).
- **First-person camera coupling** — the avatar is pinned to the headset every
  frame (camera position and view direction), at eye level. The head region is
  hidden while tracking (non-deforming mesh/material cutout, with a joint-scale
  "head chop" fallback) so it never blocks the view. Looking down shows your
  own body.
- **Locomotion** — the right thumbstick moves the XR camera rig (move where you
  look); the avatar follows automatically. Physical walking and turning work
  the same way.
- **Procedural walking** — when the avatar travels, a synthesized stepping gait
  drives the legs (foot-placement IK with ground contact, pelvis drop, and
  heel/toe roll). No leg trackers required.
- **Crouch & sit** — inferred from headset height; the legs bend and the body
  lowers to follow, again with no leg trackers.
- **Physics interaction** — kinematic collider proxies track the posed avatar
  (torso, head, arms, palms) so the body and hands push dynamic rigid-body
  objects in the scene.
- **Grabbing** — hold the trigger near a configured **dynamic rigid-body**
  object to pick it up, release the trigger to place it precisely. Static
  scenery is never grabbed.
- **Smoothing** — One Euro filtering on all tracked positions, tunable at
  runtime.

## Repository contents

| Path | Description |
|---|---|
| `exts/avatar_xr_control/` | The main Kit extension (`avatar.xr.control`) |
| `exts/machine_interaction/` | Optional extension: finger-press triggers machine animations |
| `examples/avatar/` | Optional bundled sample avatar (`newAva.usd`) |

A small example avatar is bundled under `examples/avatar/newAva.usd` (a sample
UsdSkel character with a UV-grid placeholder material). The extension also works
with any standard Reallusion-rigged USD character such as those shipped with
NVIDIA's asset packs (see step 5).

## Requirements

- Windows 10/11 with a VR-ready NVIDIA GPU (RTX recommended)
- NVIDIA Isaac Sim 4.5 or newer
- A Meta Quest headset (Quest 2/3/Pro) connected via:
  - **Meta Quest Link** (cable or Air Link), or
  - **SteamVR + ALVR** for wireless streaming
- The matching OpenXR runtime set as the system default (see step 4)

## Installation on a fresh Isaac Sim setup

### 1. Install Isaac Sim

Download and install Isaac Sim from the
[NVIDIA Isaac Sim page](https://developer.nvidia.com/isaac/sim) and launch it
once so it finishes its first-run setup.

### 2. Clone this repository

```
git clone https://github.com/FlorianSponer/Isaac-sim-xr-avatar-control.git
```

Any location works; the repository path is referred to as `<repo>` below.

### 3. Register and enable the extension

1. In Isaac Sim open **Window → Extensions**.
2. Open the extension manager settings (hamburger/gear icon) and add
   `<repo>/exts` to the **Extension Search Paths**.
3. Search for **Avatar XR Control** and enable it (optionally tick
   *Autoload*). The required `omni.kit.xr.*` extensions load automatically as
   dependencies.

A window titled **Avatar XR Control** appears.

### 4. Set the OpenXR runtime

- **Quest Link**: in the Meta Quest Link desktop app, go to
  *Settings → General* and click **Set Meta Quest Link as active OpenXR
  runtime**.
- **SteamVR/ALVR**: in SteamVR, *Settings → OpenXR → Set SteamVR as OpenXR
  runtime*, then connect the headset through ALVR.

The extension automatically requests the **VR** profile when starting.

### 5. Add a standard USD avatar to a stage

To try it quickly, reference the bundled `examples/avatar/newAva.usd` into a
stage. For your own character, the extension drives any **Reallusion Character
Creator** rig (joint naming `RL_BoneRoot/Hip/Waist/Spine01/...`). The characters from NVIDIA's standard
asset/template packs use exactly this rig — for example **F_Business_02**
("female adult business"), found in the Isaac Sim asset browser under
*People/Characters* (the characters also used by `omni.anim.people`).

1. Create or open a stage (meters as stage units, Y-up).
2. Reference the character USD (e.g. `F_Business_02.usd`) into the stage —
   drag it from the asset browser or use *Add Reference*. A root prim like
   `/Root/female_adult_business_02` is created.
3. Locate the character's **Skeleton** prim in the Stage panel (prim type
   `Skeleton`, nested under `ManRoot`). For F_Business_02 referenced at
   `/Root/female_adult_business_02` this is the extension's default:

   ```
   /Root/female_adult_business_02/ManRoot/female_adult_business_02/female_adult_business_02/female_adult_business_02
   ```

4. If your character sits at a different path (or is a different Reallusion
   character), paste its Skeleton prim path into the **Avatar Path** panel and
   click *Apply path & reinitialise*.
5. Orient the avatar so it faces stage **−Z** (the standard facing of these
   assets); the extension keeps the feet on the floor and takes over root
   position and yaw while tracking runs.

The status line shows **Ready** once the skeleton is found.

Using a non-Reallusion rig: the logical-name → joint-path mapping lives in
`JOINT_MAP` at the top of
`exts/avatar_xr_control/avatar/xr/control/extension.py`; adapt the paths there
to your skeleton and the rest of the pipeline works unchanged.

Optional: enter the prim paths of objects you want to grab (comma-separated)
in the **Grab** panel.

### 6. Start

1. Put the headset on standby, then click **Start XR** — the VR stream starts
   and tracking begins automatically after 10 seconds. The camera teleports to
   the avatar's eye level.
2. Stand upright, press **Calibrate hands (T-pose)** and follow the 10-second
   countdown: arms straight out to the sides, palms down. This calibrates hand
   rotation, IK scale, and shoulder anchors. You can face any direction.

## Controls

| Input | Action |
|---|---|
| Move/turn physically | Avatar follows the headset |
| Right thumbstick | Glide the camera rig (move where you look) |
| Trigger/grip near an object | Grab while held, release to place |

## Runtime tuning (UI panels)

- **XR Session** — re-center the view to the avatar's eyes, toggle head
  visibility, adjust eye height and forward offset.
- **Locomotion** — camera follow on/off, body turn rate, stick glide on/off,
  procedural walk toggle.
- **Physics** — body-collision layer on/off, grab enable + grabbable prim
  paths, torso collider size/placement, collision-shape debug view.
- **Tuning (advanced)** — IK reach multiplier, elbow pole weights, smoothing
  (jitter/lag).

## Troubleshooting

- **"No XR devices found"** — the XR session is not running yet; click
  *Start XR* first and keep the headset awake (proximity sensor active).
- **Camera not at eye level** — click *Reset view to eyes* in the XR Session
  panel after the session has started.
- **Arms misaligned after moving around** — recalibrate with
  *Calibrate hands (T-pose)*; calibration is direction-independent.
- **Wrong runtime starts (AR/passthrough)** — the extension prefers the VR
  profile automatically; make sure the correct OpenXR runtime is set as system
  default (step 4).
- **Viewport camera errors about `xformOp:scale`** — stale camera overrides in
  the stage; the extension repairs them automatically at initialisation (watch
  the console for `Repaired missing xformOp:...`), then save the stage once.

## Optional: body trackers via ALVR OSC

The extension contains a UDP/OSC receiver (port 9000) for the ALVR *VRChat
body OSC* sink. It is not required for any current feature and is reserved for
future lower-body tracking.
