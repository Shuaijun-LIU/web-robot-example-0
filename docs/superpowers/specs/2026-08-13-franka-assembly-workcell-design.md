# Franka Assembly Workcell Design

## Objective

Add a fourth selectable web scene named `Franka Assembly`. It is a static staging scene for a later four-arm collaboration demo: two arms can stabilize a central frame while the other two retrieve parts, exchange tools, place a cross-member, and tighten fasteners. This iteration creates no scripted motion.

## Isolation

The existing `Franka Panda`, `SO101`, and `XLeRobot` entries, layouts, controls, screenshots, and labels remain unchanged. The new entry reuses the Franka source model, four-arm control targets, selected-instance IK, keyboard controller, and mouse interaction through a separate `FRANKA_ASSEMBLY_LAYOUT`.

## Workcell Geometry

- Four Panda bases face inward at cardinal directions on a 0.90 m radius, compared with 0.72 m in the original Franka scene.
- A 2.2 m square low industrial platform supports all four bases and the work area at one common height.
- The center contains a rectangular aluminum frame with four side rails, corner brackets, visible fastener heads, and an open interior.
- A loose cross-member and mounting plate are staged in a parts tray rather than already installed.
- Two tool stations occupy different sides: a powered torque driver at one side and a manual screwdriver at the other.
- A fastener tray contains individually recognizable bolts, while a central handover pad is intentionally kept clear.
- Components use muted industrial colors: anodized aluminum, charcoal fixtures, amber tool handles, and blue-grey trays. Bright cyan is avoided.

## Physical Semantics

All visible task-relevant objects are represented by MuJoCo geometry. The central frame begins supported on assembly blocks; tools and loose components begin on trays or cradles with collision spacing. Compound objects use multiple named geoms under one body where future task motion will need them to move together. Small loose fasteners remain separate bodies. No object begins intersecting a robot, table, tray wall, or neighboring prop.

## Future Task Roles

The scene does not hard-code arm roles, but its layout supports this later sequence:

1. Opposing Arms 1 and 3 grasp and stabilize the frame.
2. Arm 2 retrieves and inserts the staged cross-member or mounting plate.
3. Arm 4 retrieves the torque driver and tightens accessible fasteners.
4. A tool or part can be transferred through the clear handover pad when a target lies outside one arm's convenient approach region.

## Asset Decision

Local RoboTwin screwdriver and hammer GLBs were inspected. The current web loader resolves one MJCF asset root and cannot safely combine those GLBs with the remote Franka model while preserving MuJoCo physics. This iteration therefore uses compound MuJoCo primitives informed by the RoboTwin tool proportions and grasp semantics. This avoids a visual-only overlay and keeps the scene deployable and ready for later physical interaction.

## Verification

- Automated tests assert a fourth config entry, four independent Franka targets, 0.90 m placement, and named assembly components.
- Existing tests protect the previous three layouts and controls.
- Browser smoke verification compiles four Franka instances and records body count, scene readiness, and screenshot.
- Visual review checks base/platform alignment, object scale and orientation, clear handover space, separated tools and parts, and absence of initial overlap.

## Success Criteria

The selector exposes exactly four scenes, with the original three unchanged. `Franka Assembly` opens as a coherent industrial workcell with a visibly larger four-arm ring and enough staged objects to explain a later collaborative assembly task without running any automatic motion.
