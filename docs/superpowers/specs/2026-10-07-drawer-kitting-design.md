# Franka Demo5 — drawer access and cooperative kitting

## Brief and authorization

The user approved starting the proposed new demo after the three-task design discussion. Implement candidate A first: a complete four-Panda drawer-access/kitting demonstration, not a training pipeline or all three proposed task families at once. Preserve existing scenes and Demo1 default. Continue without intermediate approval prompts, as requested in the project history.

## Scene and action

- Add `Franka Demo5` / internal scene `drawer` to the existing selector and cooperative runtime.
- Two compact drawers at X=±320 mm face south/north. Use RoboCasa drawer topology/dimensioning, source panel and handle assets; retain genuine slide joints and real cavity collisions.
- Arm 1 opens/closes the southern drawer; Arm 3 opens/closes the northern drawer. Neither drawer has a motor or invented self-closing force.
- Arms 2/4 retrieve tea/coffee from their respective open drawers and place them into distinct slots of one central order tray. Drawer operators release/clear after opening. Central entry is sequenced to avoid wrist overlap; drawer operations may overlap where safe.
- Drawer extraction requires the opener to physically grasp and pull the handle. Items move only from contacts and gravity. Closing is physical handle manipulation, followed by gripper release and clearance.
- Complete when both correct products are in contact with the order tray, both drawers are closed, fingers are clear and all robots return home.
- Use existing English Play/Pause/Reset controls and independent manual arm targets. Keep existing scenes unchanged.

## Architecture

Reuse the current Panda/tea/coffee/tray package without rebuilding or changing previous scene assets. A new asset builder imports RoboCasa drawer housing topology, the source decorative front and handle mesh/collision geoms, emits `drawer.xml`, a provenance manifest and license/source snapshots. Source resizing is recorded explicitly; no newly invented tool geometry.

Extend cooperative gate schema with passive `joint-range` observations and optional initial fixture-joint conditions. This gate checks actual qpos/qvel; it never writes drawer state. No drawer actuator, weld, attachment or object force override. Native solver subclass uses the same conditions; old scan/pot programs retain their existing semantics.

## Acceptance

1. Loadable self-contained browser assets: four robots/32 actuators; two passive limited slide joints; objects supported inside closed drawers; collision/visual geometry aligned.
2. Full native and WASM physical rollout: bilateral handle/item grasps, drawers open sufficiently before retrieval, no unintended robot penetration above the existing 1 mm safety threshold; aim for zero unintended robot contact on nominal route.
3. Both drawers physically close and products are supported in destination after release; all arms home.
4. Browser complete cycle, initial/open/packing/final screenshots, Reset and scene switching; no blank scene or page errors.
5. All tests/typecheck/build pass. Failed paths and important design adjustments are recorded honestly.

## Boundaries

This is a deterministic demo and seed for later benchmark conversion, not yet a randomized task or learned policy. One-arm solutions remain legal future baselines; four arms demonstrate articulated access, division of roles and safe sequencing, not a claim that four are mathematically necessary. No deployment or push without the user's next publication instruction.
