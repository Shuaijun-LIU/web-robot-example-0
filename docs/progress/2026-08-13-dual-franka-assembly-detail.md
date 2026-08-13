# Dual Franka Assembly Progress — 2026-08-13

## Confirmed scope

- Preserve the original `Franka Panda`, `SO101`, and `XLeRobot` scenes.
- Split the fourth assembly workcell into `Franka Assembly1` and `Franka Assembly2`.
- Keep both variants static in this iteration; no automatic robot sequence is added.
- Share robot placement, task stations, frame dimensions, and exact assembly interfaces.

## Implemented

- Added two selector entries with four independently controllable Panda instances each.
- Rebuilt the center frame from paired aluminum-colored flanges and dark T-slots.
- Added four frame receiver sites and four cross-member hole sites. Their positions coincide exactly at target pose `[0, 0, 0.235]`.
- Rebuilt the staged cross-member with slotted extrusion detail and open end-tab regions around four visible hole markers.
- Assembly1 now includes:
  - a generated eight-sided screwdriver handle with repeated grip recesses; its high-detail visible mesh is non-colliding, while a damped free joint and flat box proxy prevent spontaneous rocking;
  - a cordless torque driver with housing, gearbox, selector, chuck, bit, angled grip, trigger, vents, battery, and battery foot;
  - a free claw hammer rebuilt around a longitudinal handle and transverse steel head, with the round striking face and the two downward-curving claw tines on opposite sides.
- Assembly2 now includes selected RoboTwin screwdriver, drill, and hammer meshes. A standard-library converter applies GLB node transforms, normalizes orientation, exports OBJ UV coordinates and extracts the original base-color textures. It also bakes source texture regions into separate primary, dark, and metal triangle meshes because the current web geometry builder does not forward mesh UVs.
- Assembly2 uses transparent primitive collision proxies instead of high-detail mesh collision, keeping its physical representation stable and inexpensive.
- Added source attribution and licenses alongside the local Assembly2 assets.

## Verification completed so far

- Structural and numeric interface tests pass.
- TypeScript compilation passes.
- Production Vite build passes.
- Offline MuJoCo 3.3.8 compilation passes for both variants:
- Assembly1: 66 bodies, 413 geoms, 32 actuators, 106 qpos.
- Assembly2: 66 bodies, 400 geoms, 32 actuators, 106 qpos.
- Browser loading confirms four physical Panda roots and 66 bodies in each scene.
- Separate screenshots are stored at:
  - `artifacts/screenshots/franka-assembly1.png`
  - `artifacts/screenshots/franka-assembly2.png`

## Focused tool correction — 2026-08-13

- Root cause of the Assembly1 screwdriver motion: the recessed high-face handle mesh was also its contact surface, so it kept alternating between tiny contact points. The visible mesh now has contact disabled; a flat invisible box carries handle collision and the free joint has damping `0.08`.
- Browser sampling from 5 s to 20 s after loading measured only `0.00000393 m` translation and `0.0000973 rad` rotation, which is visually stationary.
- The Assembly1 hammer head was replaced with a conventional eye/cheek, round striking face on one side, and a separated two-tine claw on the other.
- The three Assembly2 RoboTwin tools now render three explicit color groups each instead of one uniform material. Original tool geometry and simple collision proxies remain unchanged.
- Updated offline MuJoCo compilation: Assembly1 has 66 bodies / 413 geoms; Assembly2 has 66 bodies / 400 geoms. Both retain 32 actuators and 106 qpos.
- Focused screenshots were regenerated and visually inspected at the two existing artifact paths above.
- A fresh full browser control run passed all 18 selectable targets, including keyboard and IK isolation for all four arms in both Assembly1 and Assembly2.

## Final verification

- Final visual inspection passed after raising the hammer onto a low tool shelf and adjusting the external drill/hammer scale.
- All 28 Node tests pass.
- TypeScript `--noEmit`, production build, and GitHub Pages build pass.
- Full browser control isolation passes for 18 selectable targets:
  - original Franka: four keyboard + IK targets;
  - SO101: four keyboard + IK targets;
  - XLeRobot: two keyboard targets;
  - Assembly1: four keyboard + IK targets;
  - Assembly2: four keyboard + IK targets.
- Each control check changed only its selected actuator block.
- The deployed GitHub Pages URL was opened in a fresh headless browser; online `Franka Assembly2` loaded four physical Panda roots and 66 bodies successfully. Remote smoke scripts now allow up to 120 seconds for the 12 MB application bundle to reach `DOMContentLoaded` on slower links.

## Publication state

The implementation, generated Pages bundle, source attribution, and comparison screenshots were published to `main` through the configured personal GitHub SSH remote in implementation commit `6d31a75`.
