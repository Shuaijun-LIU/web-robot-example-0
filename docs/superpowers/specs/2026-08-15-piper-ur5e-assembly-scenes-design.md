# PiPER and UR5e Assembly1 Scenes Design

Date: 2026-08-15

## Goal

Add two independent scenes derived from the published `Franka Assembly1`
workcell:

- `Piper Assembly1`: four AgileX PiPER arms with their original parallel
  grippers.
- `UR5e Assembly1`: four Universal Robots UR5e arms, each fitted with a
  Robotiq 2F-85 gripper.

The existing Franka scenes, default selection, workcell implementation, and
Assembly1 automation remain unchanged. The new scenes initially support manual
per-arm IK, keyboard/mouse control, and gripper operation only; they do not
inherit the Franka-specific automated assembly sequence.

## Asset choice and provenance

All robot assets come from the local MuJoCo Menagerie checkout so the deployed
GitHub Pages build does not depend on runtime network access:

- `agilex_piper`: MIT-licensed MJCF, meshes, collisions, inertials, six arm
  actuators, and one coupled gripper actuator.
- `universal_robots_ur5e`: BSD-3-Clause MJCF, meshes, collisions, inertials,
  and six arm actuators.
- `robotiq_2f85`: BSD-2-Clause MJCF, meshes, coupled linkage constraints,
  pad collisions, and one gripper actuator.

Each scene directory retains the upstream licenses and a notice identifying
the source model. No unlicensed reference source or model is copied.

## Approaches considered

1. **Local modular MJCF composition (selected).** Vendor the three Menagerie
   models. PiPER is attached directly; Robotiq is attached to the UR5e wrist
   inside a composite model using MuJoCo's `asset/model` and `body/attach`
   elements. Attach the resulting robot four times with prefixes.
2. **Remote source plus runtime XML patches.** This resembles the original
   Franka scene but makes Pages loading depend on GitHub raw URLs and makes the
   nested UR5e/gripper composition fragile. Rejected.
3. **Convert combined URDFs from RoboTwin/RoboSplat.** Combined files exist but
   introduce another conversion and provenance chain, and their actuator setup
   is less compatible with the current browser controller. Rejected.

## Scene composition

The visible workcell is copied from the stable Assembly1 baseline: platform,
inset surface, supported aluminum frame, staged cross-member, mounting plate,
four fasteners and trays, handover pad, manual screwdriver, torque driver, and
double-face hammer. Names, scales, masses, friction, receivers, and tool detail
remain the same unless a robot base would physically overlap a station.

Both scenes place four fixed robot bases at 90-degree intervals, all facing the
central frame. The base mounting plane is exactly the `0.10 m` platform top.

- PiPER uses a smaller ring radius than Franka because of its shorter reach.
  The chosen radius must let each arm reach its nearest frame grip/receiver
  while leaving visible clearance between base, hammer shelf, and trays. Only
  the hammer shelf may shift slightly inward if required by collision checks.
- UR5e uses approximately the Franka ring radius. The longer Robotiq tool
  center is included when checking frame and neighbor clearance.

Camera positions may differ by scene to keep all four bases and the complete
workcell visible. The global page remains defaulted to `Franka Assembly1`.

## Robot model integration

### PiPER

The PiPER model is loaded as a model asset and attached four times from
`base_link`, with prefixes `r0_` through `r3_`. Its upstream keyframe is
removed before replication so the application supplies one 7-actuator home
target per arm. A `tcp` site is added on the gripper centerline at the physical
pinch position.

Each control target maps:

- IK joints: `joint1` through `joint6`.
- IK actuators: the first six actuators in the selected instance.
- Gripper actuator: `gripper`, with explicit open and closed values inside its
  inherited control range.

### UR5e + Robotiq 2F-85

The UR5e model declares the Robotiq model as an asset. A frame at UR5e's
`attachment_site` orientation attaches the Robotiq `base_mount` subtree with a
`gripper_` prefix. This copies the gripper's referenced assets, tendons,
constraints, contacts, and actuator into the composite model. The composite is
then attached four times with `r0_` through `r3_` prefixes.

The Robotiq `pinch` site is the TCP. Each control target maps:

- IK joints: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
  `wrist_1_joint`, `wrist_2_joint`, and `wrist_3_joint`.
- IK actuators: the six UR5e actuators in the selected instance.
- Gripper actuator: `gripper_fingers_actuator`, with `0` as open and `255` as
  closed according to its scaled tendon actuator.

The nested attachment is compiled and verified before UI integration. If the
browser MuJoCo build cannot replicate the nested attachment reliably, the
fallback is a generated, flattened composite MJCF produced from the same
licensed inputs; changing to a different robot asset is not required.

## Control architecture

The existing selected-instance IK solver is reused because it already resolves
sites, joints, and actuator indices from `ControlTarget`. New target factories
describe each robot's joint order, actuator stride, TCP, and gripper binding.

Gripper keyboard control becomes profile-driven rather than Franka-specific:
the target supplies its actuator name plus open/closed values. Existing Franka
behavior remains byte-for-byte equivalent. PiPER and UR5e use the same keyboard
key and selection rules, so switching `Arm 1` through `Arm 4` changes both the
active IK gizmo and the gripper that receives input.

The new scenes use a generic industrial-arm control family only where needed;
Franka-only Assembly sequence components remain gated to `frankaAssembly1`.
No new automation controller writes to either new scene.

## Failure handling

- A scene reports load error if any robot model, nested gripper attachment,
  mesh, TCP site, joint, or actuator cannot be resolved.
- IK is disabled for an unresolved target rather than writing incorrect
  actuator indices.
- Gripper bindings validate finite open/closed values and actuator existence.
- Switching scenes unmounts the old IK/gripper controller and resets target
  selection to the first arm.
- No corrective invisible support or root transform is introduced to hide a
  bad initial pose.

## Verification

Tests are added before implementation and establish:

1. Both new scene keys are additive and `frankaAssembly1` remains the default.
2. Each layout declares four instances and four control targets labelled
   `Arm 1` through `Arm 4`.
3. PiPER targets resolve six IK joints and one gripper actuator per instance;
   UR5e targets resolve six IK joints and one Robotiq actuator per instance.
4. Vendored licenses and notices exist; runtime URLs for the two scenes are
   local.
5. Offline/browser compilation reports exactly four PiPER base roots or four
   UR5e base roots, four TCP sites, and the expected actuator count.
6. Every home target is finite and within the compiled actuator control range.
7. A browser smoke test switches through all four arms, enables each IK gizmo,
   changes at least one joint target, and opens/closes only the selected
   gripper.
8. Initial browser state has no robot-platform penetration, robot-frame
   penetration, robot-tool penetration, or neighbor-arm collision.
9. The complete Node suite and production build pass.

Final visual artifacts are one screenshot per new scene showing all four arms,
the full Assembly1 workcell, clear base mounting, and non-overlapping initial
poses.
