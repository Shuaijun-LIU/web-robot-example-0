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
  - a generated eight-sided screwdriver handle with repeated grip recesses;
  - a cordless torque driver with housing, gearbox, selector, chuck, bit, angled grip, trigger, vents, battery, and battery foot;
  - a free claw hammer with striking face, split claw, and non-slip handle, staged on a low supported shelf so it remains visible and graspable.
- Assembly2 now includes selected RoboTwin screwdriver, drill, and hammer meshes. A standard-library converter applies GLB node transforms, normalizes orientation, exports OBJ UV coordinates, and extracts the original base-color textures.
- Assembly2 uses transparent primitive collision proxies instead of high-detail mesh collision, keeping its physical representation stable and inexpensive.
- Added source attribution and licenses alongside the local Assembly2 assets.

## Verification completed so far

- Structural and numeric interface tests pass.
- TypeScript compilation passes.
- Production Vite build passes.
- Offline MuJoCo 3.3.8 compilation passes for both variants:
- Assembly1: 66 bodies, 408 geoms, 32 actuators, 106 qpos.
- Assembly2: 66 bodies, 394 geoms, 32 actuators, 106 qpos.
- Browser loading confirms four physical Panda roots and 66 bodies in each scene.
- Separate screenshots are stored at:
  - `artifacts/screenshots/franka-assembly1.png`
  - `artifacts/screenshots/franka-assembly2.png`

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

## Publication state

The implementation, generated Pages bundle, source attribution, and comparison screenshots are ready for the final commit and push through the configured personal GitHub SSH remote.
