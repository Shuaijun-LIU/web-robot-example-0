# Assembly1 Step 2 Physical Clamp Design

## Goal

Continue from the verified Step 1 pre-grasp state and establish four real, contact-based grasps:

- Arm 1 grips the south rail of the assembly frame.
- Arm 2 grips the handle of the side-laid torque driver.
- Arms 3 and 4 grip the cross-member at two balance points separated by 0.24 m.

Step 2 ends after the grasps are verified and held. It does not lift, transfer, align, fasten, or otherwise move a task object to a new station.

## Non-goals and Physical Integrity

Step 2 must not:

- write task-object `qpos` values;
- add equality welds, magnetic grasping, proximity attachment, or scripted following;
- close one cross-member gripper before the other;
- lift any task object;
- change `Franka Assembly2`;
- start a later beam-transfer or fastening action automatically.

All object motion must arise from MuJoCo contacts, actuator forces, gravity, and the existing robot-joint-only gravity compensation.

## Preconditions

The Step 2 button is enabled only when:

- the current scene is `Franka Assembly1`;
- Step 1 has reached its verified `complete` state;
- all four Step 1 TCP targets remain within 0.03 m and 8 degrees of their contract;
- all four gripper controls are still open at `255`;
- no task object has drifted more than 0.003 m since Step 1 completed.

Reset returns both step states, all controls, and all task objects to the original scene state.

## Contact Geometry

The injected Panda `tcp` site is colocated with the fingertip-pad center to within a few millimetres. Each contact target therefore places the TCP at the vertical center of the intended grasp section while the fingers close laterally around it.

| Arm | Target body | World contact TCP | Closing axis | Descent from Step 1 |
|---|---|---:|---|---:|
| Arm 1 / `r0` | `assembly_frame` south rail | `(0.000, -0.230, 0.235)` | world Y | 0.095 m |
| Arm 2 / `r1` | `torque_driver` handle | `(0.559, -0.421, 0.160)` | yaw 162 degrees, equivalent to the undirected -18-degree axis | 0.120 m |
| Arm 3 / `r2` | `cross_member` north balance point | `(-0.490, 0.560, 0.140)` | world X | 0.120 m |
| Arm 4 / `r3` | `cross_member` south balance point | `(-0.490, 0.320, 0.140)` | world X | 0.120 m |

The torque-driver height uses its production-browser settled handle center (approximately 0.160 m), rather than its nominal pre-settling body height of 0.166 m. The first slow-descent waypoints are 0.015 m above the four contact centers. A non-mutating offline feasibility check with the current Panda MJCF found all four contact targets inside joint limits, with maximum position error below 0.00075 m and maximum orientation error below 0.12 degrees.

## Recommended Choreography

The sequence is intentionally staged so a frame-grasp failure cannot be hidden by the later cross-member motion, and the cross-member never receives one-sided gripping torque.

1. **Coarse approach — 1.4 s.** All four arms move from the Step 1 targets to their contact-center-plus-0.015-m waypoints. All grippers remain at `255`.
2. **Slow descent — 0.8 s.** All four arms descend the last 0.015 m while retaining their verified top-down orientations. All grippers remain open.
3. **Frame clamp — 0.8 s.** Arm 1 gripper control eases from `255` to `0`. Arms 2–4 remain open.
4. **Frame verification — at least 0.25 s.** Both Arm 1 finger bodies must continuously contact `assembly_frame`.
5. **Cross-member clamp — 1.0 s.** Arm 3 and Arm 4 gripper controls ease from `255` to `0` in the same simulation frames. Arm 1 remains closed; Arm 2 remains open.
6. **Cross-member verification — at least 0.25 s.** Both fingers of both arms must continuously contact `cross_member` while its pose remains stable.
7. **Torque-driver clamp — 0.8 s.** Arm 2 gripper control eases from `255` to `0`; the other three grasps remain held.
8. **Tool verification — at least 0.25 s.** Both Arm 2 fingers must continuously contact `torque_driver`.
9. **Clamped hold.** The four arm targets, four close commands, and gravity compensation on the 28 Panda arm-joint DOFs remain active. No lift follows automatically.

Joint and gripper interpolation use the same clamped smoothstep convention as Step 1. Gripper control `0` requests physical closure through the existing force-limited actuator; it is not interpreted as an attachment command.

## Contact Verification

Finger collision geoms are not individually named in the source Panda model, so verification resolves contacts through body ownership:

- Arm N left finger body: `rN_left_finger`;
- Arm N right finger body: `rN_right_finger`;
- contact geom ownership: `model.geom_bodyid`;
- active contact pairs: current MuJoCo `data.contact` records.

A grasp passes only if all of the following hold:

- both finger bodies contact only the assigned target body for at least 0.25 continuous seconds;
- the summed two-finger joint aperture remains above 0.020 m after a `0` close command, guarding against an empty fully closed gripper;
- no finger body contacts another task object, a tray, a cradle edge, or the work platform;
- the assigned object moves no more than 0.005 m from its pre-Step-2 pose;
- the assigned object's orientation changes no more than 5 degrees;
- vertical object displacement remains no more than 0.003 m, proving Step 2 did not lift it.

The cross-member verification is atomic: all four Arm 3/4 finger contacts must satisfy the continuous-contact window at the same time.

## Control Ownership

During Step 2 and the final clamped-hold state, the sequence controller owns:

- all 28 Panda arm actuators;
- all four gripper actuators;
- gravity compensation for only the 28 Panda arm joint DOFs.

IK gizmos, drag commands, keyboard arm commands, and keyboard gripper toggles are disabled while this ownership is active. This prevents the normal single-arm controller from reopening one gripper immediately after Step 2 reports completion. Reset releases sequence ownership. A later Step 3 controller may explicitly take ownership from the clamped state.

Step 1 retains its hold until Step 2 has resolved and validated every model name and joint target. Step 1 then releases arm/gravity-compensation ownership in the same physics frame in which Step 2 takes ownership, so compensation is never absent and is never applied twice.

Task-object free joints and generalized forces remain untouched.

## UI

The upper-left Assembly1 action panel becomes a two-step sequence panel:

- Step 1 remains visible as `第一步已完成` after staging.
- A second button appears below it as `执行第二步：下降并物理夹持`.
- During execution, the status line identifies the current phase: approach, slow descent, frame clamp, dual cross-member clamp, torque-driver clamp, or verification.
- Success reads `第二步已完成：四处物理夹持已建立`.
- Failure reads the specific arm and missing/forbidden contact, followed by `请 Reset 后重试`.

The Step 2 button cannot be clicked twice and cannot be used from another scene.

## Failure Handling

On any timeout, forbidden contact, excessive drift, excessive rotation, aperture failure, missing model name, or out-of-limit joint target:

- stop advancing the phase machine;
- retain the current arm targets and gripper controls instead of making an abrupt release;
- keep robot-joint gravity compensation active;
- expose a structured failure reason to the panel and browser diagnostics;
- require Reset before another attempt.

Because Step 2 never lifts an object, this hold-on-failure policy leaves every object supported by its original surface.

## Architecture

Keep the Step 2 behavior isolated from the proven Step 1 implementation:

- `src/assemblyStep2.js` and `.d.ts`: immutable arm/contact contract, timing constants, pure phase selection, smooth gripper interpolation, and pure contact-verdict helpers;
- `src/AssemblyStep2Controller.tsx`: model-name resolution, actuator ownership, trajectory execution, contact sampling, pose-drift checks, and clamped hold;
- `src/AssemblySequencePanel.tsx`: replaces the Step-1-only panel and renders both buttons plus phase/failure status without owning simulation logic;
- `scripts/solve-assembly-step2-waypoints.mjs`: generates and validates approach/contact joint arrays from the same Panda MJCF;
- `scripts/verify-assembly-step2.mjs`: runs the real production browser scene and records contacts, apertures, object drift, object rotation, controls, and screenshot evidence.

Step 1 remains independently runnable and independently testable. Step 2 consumes only the public Step 1 completion state and current simulator controls.

## Test and Acceptance Plan

### Unit and source tests

- exact contact targets, axes, roles, body names, and 0.24-m cross-member separation;
- phase boundaries and simultaneous Arm 3/4 close interpolation;
- contact verdict requires both fingers and rejects wrong-body contacts;
- aperture, pose-drift, and vertical-lift thresholds;
- Step 2 never exposes a task-object position write or attachment shortcut;
- Assembly2 remains unchanged.

### Offline IK

- eight arrays: four 0.015-m approach targets and four contact targets;
- every joint inside the MJCF range;
- position error at most 0.01 m and orientation error at most 5 degrees offline, with the stricter generated results recorded.

### Production-browser acceptance

- Step 2 is unavailable before Step 1 completion and available afterward;
- all four arms visibly descend and all four grippers physically close;
- required bilateral contacts remain continuous for at least 0.25 s;
- cross-member grasps become valid in the same verification window;
- total object drift is at most 0.005 m, rotation at most 5 degrees, and vertical displacement at most 0.003 m;
- no forbidden contact, object `qpos` write, weld, magnet, attachment, or scripted follow occurs;
- clamped hold remains stable for at least 2 s after success;
- Reset returns the scene to the original open-gripper state;
- save a final screenshot under `artifacts/screenshots/` and record exact measured tolerances in `project/`.

## Boundary for Step 3

Step 3 may begin only after this physical clamp state is visually reviewed. Its likely responsibility is lifting and transferring the cross-member with Arms 3/4 while Arm 1 stabilizes the frame and Arm 2 holds the torque driver. Step 2 itself contains no lift or placement behavior.
