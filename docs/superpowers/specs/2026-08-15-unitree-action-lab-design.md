# Unitree Action Lab Design

Date: 2026-08-15

## Goal

Add a new, isolated browser scene containing only a Unitree G1, a Unitree
Go2 carrying an Airbot Play arm, and a collision-enabled floor. Both robots
must use articulated MuJoCo dynamics and execute one deterministic,
continuous action sequence without an ONNX policy or scripted root motion.

## Source audit and selected assets

- G1 comes from the local MuJoCo Menagerie mirror at
  `google-deepmind__mujoco_menagerie/unitree_g1`. The selected 29-DoF model
  has a floating base, inertials, collision geometry, foot contacts, position
  actuators, and a stable `stand` keyframe. Its BSD-3-Clause license is copied
  with the vendored model.
- Go2 + Airbot comes from
  `/data/private/user2/workspace/14.unilab/code/src/unilab/assets/robots/go2_arm`.
  The model has one floating base, 12 driven leg joints, six driven arm
  joints, inertials, collision geometry, foot contacts, and a `home`
  keyframe. The vendored files retain Apache-2.0 attribution.
- The Go2 leg targets for the stand-up and stand-down phases are adapted from
  `unitreerobotics__unitree_mujoco/example/python/stand_go2.py`.
- `Axellwppr__humanoid-policy-viewer` contains useful G1 walking and running
  reference clips, but those clips are tracked by an ONNX policy. They are
  not used as raw actions in this policy-free first scene.

Local MuJoCo probes establish the starting envelope: the G1 stand keyframe
remains upright for 10 seconds, and its proposed upper-body gesture keeps
pelvis tilt below 0.6 degrees. The Go2 + Airbot home keyframe remains standing
for 10 seconds; the proposed stand, arm-scan, lower, and recover sequence
returns to the home height under gravity and contact.

## Scene architecture

The new scene key is `unitreeActionLab`, labelled `Unitree Action Lab`. It is
additive: the eight existing scene entries, default Assembly1 selection, and
their manual controls remain unchanged.

All model dependencies live below
`public/assets/unitree-action-lab/`. A root `scene.xml` supplies the floor,
lighting, solver options, and two separated robot placements. Namespaced child
models retain their free joints, actuators, inertials, collisions, and
sensors. No furniture, pedestal, invisible support, planar slide joint, mocap
body, equality weld, or task object is present.

The scene configuration supplies one combined `homeJoints` vector in compiled
actuator order. The standard loader uses it only during load/reset to seed the
stable articulated pose; runtime action phases do not write joint state.

The application adds an action-only control family. The scene does not expose
IK or keyboard locomotion controls because those would imply kinematic control
that this demonstration is meant to replace. Its right-side selector has one
target, `G1 + Go2 action`, while a dedicated action panel provides execution
controls and phase feedback.

## Action representation and execution

`unitreeActionSequence.js` owns data only: actuator-name arrays, home targets,
phase durations, and target joint vectors. It exposes a pure sampler that maps
elapsed sequence time to a pair of actuator target vectors using clamped
smoothstep interpolation. Literal target arrays use the same joint ordering as
the vendored MJCF.

`UnitreeActionController.tsx` resolves actuator names once after scene load and
registers a before-physics-step callback. During execution it writes only the
resolved entries in `data.ctrl`. It never writes `data.qpos`, `data.qvel`,
body transforms, root-joint state, contact state, or applied forces. Reset uses
the existing MuJoCo reset API, which reapplies the scene configuration's
combined `homeJoints` vector.

The 10-second choreography is:

1. **Settle:** hold both home targets for one second.
2. **Rise and greet:** Go2 moves from home toward the Unitree stand target
   while G1 smoothly raises its right arm.
3. **Scan and wave:** G1 oscillates its right wrist; Airbot moves to a raised
   scan pose and sweeps its first joint.
4. **Lower:** G1 returns its arm while Go2 follows the official lower-body
   target and the Airbot returns home.
5. **Recover:** Go2 returns to its stable home pose.
6. **Final hold:** both robots hold their home targets for settling and final
   state measurement.

The phase durations are exactly `1.0`, `1.5`, `3.0`, `1.5`, `1.5`, and `1.5`
seconds respectively.

This is an open-loop actuator action clip, not a learned locomotion controller.
The demonstration intentionally uses stable stationary whole-body motions;
walking is excluded because robust G1/Go2 walking would require a balance
controller or policy and would obscure the action-only objective.

## User interface and diagnostics

The action panel shows the current phase, elapsed time, and state
(`idle`, `running`, `paused`, `complete`, or `error`). It provides:

- `执行完整动作`
- `暂停` / `继续`
- `重新开始`

Browser diagnostics expose action state plus G1 pelvis and Go2 base poses for
automated checks. Existing `window.robotDemo` diagnostics remain available in
other scenes.

## Safety and failure behavior

- Actuator lookup is name-based; a missing or duplicated actuator aborts the
  action before any target is written.
- Every sampled target must be finite and within the MJCF actuator control
  range. Invalid targets transition the panel to `error` and release action
  ownership.
- Pausing holds the last action target while normal MuJoCo stepping continues,
  so gravity and contact never freeze.
- Switching scenes unmounts the controller and removes all callbacks.

## Verification

Tests are added before implementation and cover observable behavior:

1. The compiled scene contains exactly two floating roots, 29 G1 actuators,
   12 Go2 leg actuators, six Airbot actuators, collision geometry, gravity,
   and no planar or equality-based movement shortcut.
2. The pure sampler is continuous at phase boundaries, stays within literal
   actuator limits, returns the expected home/gesture/scan/lower targets, and
   ends in the home pose.
3. Controller source and runtime diagnostics demonstrate actuator-only action
   ownership; a browser run confirms nonzero articulated joint motion, foot
   contacts, finite state, completion, and stable final roots. At completion,
   G1 pelvis height must be within `0.75–0.85 m` with tilt no greater than
   `5°`; Go2 base height must be within `0.22–0.34 m` with tilt no greater than
   `10°`.
4. The existing full test suite, TypeScript check, production build, offline
   MuJoCo compile, screenshot capture, and a full-sequence browser check pass.

The final visual artifacts are a screenshot of the simultaneous G1 wave and
Airbot scan phase and a complete browser-recorded action video at
`artifacts/videos/unitree-action-lab.mp4`.
