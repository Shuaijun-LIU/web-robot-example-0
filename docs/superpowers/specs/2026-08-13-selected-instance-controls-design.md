# Selected-Instance Multi-Robot Controls Design

## Goal

Restore the interaction behavior of the original single-robot scenes while keeping the current physical multi-robot layouts:

- every Franka Panda and SO101 arm can be selected and controlled independently;
- either complete XLeRobot can be selected, with its original base, head, and two-arm keyboard controls;
- the robot selector keeps the original names without count suffixes;
- the active target is explicit in the control panel;
- scene-level performance statistics and keyboard help are easier to understand and read.

## Diagnosed regression

The multi-robot layout added namespaced MuJoCo bodies and actuators, but the controller layer remained bound to the first namespace block:

- the IK site is fixed to `r0_tcp`;
- the built-in generic IK controller reads the first arm's `qpos` values and writes the first actuator block;
- Franka's gripper binding names `r0_gripper` directly;
- SO101 and XLeRobot keyboard controllers use actuator indices starting at zero;
- the scene now starts paused with the gizmo hidden, whereas the original interactive scene started running with the gizmo visible.

Consequently, later instances cannot be controlled, and a paused scene makes even the first instance appear unresponsive.

## Interaction model

The existing `Robot` selector displays only these labels:

- `Franka Panda`
- `SO101`
- `XLeRobot`

A second `Control target` selector changes with the chosen scene:

- Franka Panda: `Arm 1`, `Arm 2`, `Arm 3`, `Arm 4`;
- SO101: `Arm 1`, `Arm 2`, `Arm 3`, `Arm 4`;
- XLeRobot: `Robot 1`, `Robot 2`.

Franka and SO101 display exactly one IK gizmo at the selected arm's TCP. Switching the target synchronizes the gizmo to the new TCP and preserves every arm's current pose. Keyboard commands and gripper commands write only to the selected instance. XLeRobot retains the original single-robot key map for both arms, the base, and the head; the target selector offsets that complete control group to Robot 1 or Robot 2.

The simulation starts running and the IK gizmo starts visible, matching the original working example. Pause remains available. Mouse body dragging and double-click selection remain scene-wide rather than being restricted by the active control target.

## Controller architecture

### Target metadata

Each `RobotEntry` exposes target metadata rather than one hard-coded primary target. A target records the namespace prefix and the offsets needed by that robot type:

- Franka: four targets, eight actuators per target, seven IK joints, and a namespaced TCP and gripper;
- SO101: four targets, six actuators per target, five IK joints, and a namespaced TCP;
- XLeRobot: two targets, sixteen actuators per complete robot.

Pure helpers generate selector options, namespaced resource names, and shifted actuator indices. These helpers are independently testable without a browser.

### Selection-aware IK

The current upstream generic IK implementation assumes `qpos[0..n)` and `ctrl[0..n)`. The demo will use a local selection-aware controller that performs the same damped least-squares solve against explicit selected joint addresses and writes the solution to explicit actuator indices. It owns one gizmo target, resolves the selected TCP after scene load, and resynchronizes whenever the target changes or the simulation resets.

Only the selected arm participates in the IK solve. This avoids four simultaneous solvers, overlapping gizmos, and accidental motion of non-selected arms.

### Keyboard controllers

Franka resolves the selected namespaced gripper actuator. SO101 shifts its six actuator indices by the selected arm's block offset. XLeRobot shifts the existing sixteen-index base/two-arm/head controller by the selected robot's block offset. Changing selection clears held-key state and initializes controller state from the newly selected control block so there is no pose jump.

## Overlay changes

The three top-left panels report whole-scene rendering statistics: FPS, frame time, and renderer memory. They are not measurements for one arm, so they remain a single set. A `Scene performance` label will make that scope explicit.

The bottom-left Keyboard panel will use a dedicated CSS class with larger horizontal and vertical padding, a minimum width, improved line height, and a larger gap between its title and key list. The help text will also identify the current control target so keyboard scope is unambiguous.

## State and observability

The selected target resets to the first valid target when the robot scene changes. The application exposes the active target and resolved IK site in document data attributes for deterministic browser tests. An invalid target is clamped to the first valid entry instead of leaving a controller attached to a stale namespace.

## Verification

Automated checks will cover:

- unchanged robot selector labels;
- target counts and namespace/offset mappings;
- shifted keyboard actuator indices;
- default running simulation and visible gizmo;
- the scene-level performance label and expanded keyboard panel styling;
- MJCF compilation with all expected TCP and actuator names;
- browser switching through every Franka arm, every SO101 arm, and both XLeRobots;
- keyboard input changing only the selected actuator block;
- IK target resolution following the selected TCP without moving a non-selected block.

The final verification includes the full Node test suite, TypeScript checking, production and Pages builds, browser smoke checks, refreshed screenshots, and a non-force SSH push to `origin/main`.

## Scope boundaries

This change restores and generalizes the existing controls; it does not add coordinated multi-arm trajectories, simultaneous multi-gizmo manipulation, collision-aware motion planning, or a new task policy. Those can build on the explicit target model later without changing the scene layout.
