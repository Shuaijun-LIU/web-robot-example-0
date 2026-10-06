# Two new cooperative Franka workcells

## Delivery and boundaries

The user approved the two proposals in `docs/progress/2026-10-07-demo2-polish.md` and requested publication of Demo2's optimization before starting them. Keep Demo1/Assembly1 and Demo2 unchanged. Add separate **Franka Demo3** (inspection and packing) and **Franka Demo4** (cooperative pot loading); the default page remains Demo1. English UI, four independently controllable Panda arms, Reset and clear task progress.

Build and inspect both physical layouts before scheduling complete motions. Each scene must be usable for manual inspection even while its automatic sequence is under development. A scene is not a finished automatic demo until its complete actuator-only sequence passes native and browser-engine physics checks.

## Demo3: product inspection and packing

Use RoboTwin `024_scanner`, `112_tea-box`, `113_coffee-box`, and `008_tray` or `062_plasticbox`. Preserve source textures and measured dimensions; record any uniform rescaling. First cycle uses two different product boxes, not a large batch. The scanner is a real graspable body, not attached to the wrist.

1. Arm 1 lifts a product and presents a face toward the inspection region.
2. Arm 2 picks the scanner and positions it relative to the product. Inspection is a **geometric pose gate**, not barcode decoding or a learned vision system.
3. Arm 1 releases the inspected product onto a supported handoff pad and clears it. Arm 3 then places it into the designated order tray.
4. After both products are packed and Arm 3 clears the shared region, Arm 4 pushes the loaded tray along a continuous support bed into dispatch using two separated fingertips. This is explicitly a supported push, not a pinch grasp or airborne carry. Then all arms return home.

Final success: both products inspected once, packed in their intended destinations, physically supported, scanner and tray supported, all grippers released and arms home. Scanner/object orientation and standoff, handoff ownership and tray occupancy must be observable in diagnostics.

## Demo4: two-arm pot support and solid ingredient loading

Use a sourced open pot with two opposed handles and sourced solid food. Retain the actual cavity in both visual and collision geometry. A source lid may be omitted as a separate source part; do not create a replacement pot. Arms 1/3 hold opposite handles; Arms 2/4 load from separated preparation areas, taking turns at the opening.

1. The holding pair approaches, physically grips both handles and lifts the pot slightly while preserving its level orientation.
2. The loading pair picks two solid ingredients from separate trays and sequentially releases them inside the pot.
3. Both loading arms clear the opening. The holding pair lowers the loaded pot to a pad, verifies support, opens and retreats. All arms return home.

Final success: two ingredients remain inside the real cavity, pot is level and supported, all grippers released and arms home. No liquids, cutting, heating or soft-body claims.

## Physical and implementation rules

- Sourced visual objects; primitive tables/pads are fixtures only. Audit units, node transforms, texture orientation, scale and attribution.
- Actuator commands only during execution. No free-body qpos assignment, object following, welds, hidden grasp constraints or gravity disabling.
- All four robot collisions remain enabled. Continuous monitoring distinguishes legitimate finger contacts from robot/object and robot/robot collisions.
- Grasp verification needs actual bilateral contacts and a carried object; TCP proximity alone never passes a grasp.
- Hollow vessels use source convex pieces or derived decompositions, never a solid convex hull over the cavity.
- Each scene owns its MJCF, motion data and task state. Existing controllers and accepted motion assets are not retuned.
- Reuse the fixed 0.002 s control clock. Planning is offline; pressing Play must not run blocking IK in the browser.
- New asset package records sources, hashes, conversion, physical assumptions and relevant licenses. No machine-specific cluster/account configuration in repository documentation.

## Verification

First verify mesh/collision alignment, static settling, handle/finger clearance and all four manual control targets. Then validate each motion phase in CPU MuJoCo and the actual browser WASM engine; inspect rendered screenshots at grasp, transfer/load and final states. Record scope and failures honestly. Reset must cancel playback, and switching away must remove the controller. The full pre-existing test suite, type check and production build remain required.

## Sources

- [RoboTwin scan task](https://robotwin-platform.github.io/doc/tasks/scan_object.html) and [lift-pot task](https://robotwin-platform.github.io/doc/tasks/lift_pot.html): geometry and functional-point references, not ready-made four-arm plans.
- [RoboTwin asset distribution](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0): asset pack used by the project's official download script; dataset card declares MIT. Preserve additional source-specific notices where present.
- [MuJoCo Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda): robot meshes and dynamics.
