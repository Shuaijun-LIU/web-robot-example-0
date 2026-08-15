# PiPER and UR5e Assembly1 Scenes — Progress Record

Date: 2026-08-15  
Branch: `feature/piper-ur5e-assembly`

## Delivered

- Added `Piper Assembly1` as four AgileX PiPER arms with native coupled grippers around the procedural Assembly1 workcell.
- Added `UR5e Assembly1` as four UR5e arms, each composed with a Robotiq 2F-85 gripper, around the same workcell.
- Kept `Franka Assembly1`, its default selection, and its three-step automation unchanged.
- Added independent Arm 1–4 selection, one IK gizmo per selected arm, and robot-specific physical gripper ranges.
- Vendored all new robot meshes and MJCF files locally for deterministic GitHub Pages loading, with upstream licenses and third-party notices.

## Measured MuJoCo Results

| Scene | Bodies | Geoms | Actuators | Qpos | Named sites | Initial penetration |
|---|---:|---:|---:|---:|---:|---|
| Piper Assembly1 | 60 | 448 | 28 | 102 | 17 | `0` penetrating contacts |
| UR5e Assembly1 | 108 | 324 | 28 | 126 | 21 | `0` penetrating contacts |

Both models compile with four prefixed physical roots and four finite TCP poses. PiPER uses a `0.78 m` ring and moves the complete hammer station to `x=0.57 m` to clear the east arm. UR5e uses a `0.90 m` ring.

## Browser Control Evidence

`scripts/verify-alternate-assembly-browser.mjs` loaded both local scenes and checked all eight scene/arm combinations:

- selected target and resolved TCP matched (`r0_` through `r3_`);
- pressing `V` changed exactly the selected arm's gripper actuator;
- a `12 mm` IK target displacement changed at least one selected arm joint command;
- IK changed no actuator outside the selected six-joint block;
- each scene reported exactly four physical instances with no page, request, or simulation error.

## Visual Artifacts

- `artifacts/screenshots/piper-assembly1.png` — 1440 × 900
- `artifacts/screenshots/ur5e-assembly1.png` — 1440 × 900

Both screenshots were inspected at original resolution. All four bases sit on the common platform, the arms remain separated, grippers face the workcell, and the frame, cross-member, tools, trays, and handover regions remain readable. No visible base/platform or robot/tool penetration was found.

## Verification

- `npm test` — 129/129 passed.
- `npx tsc --noEmit` — passed.
- Vite production build with local Node 24 — passed, 703 modules transformed.
- Strict offline MuJoCo compile/contact reports — passed for both scenes.
- Browser Arm 1–4 gripper and IK verification — passed for both scenes.
- Focused scene capture — passed with 4 instances / 60 bodies and 4 instances / 108 bodies.

The host default `/usr/bin/node` is Node 18, while the repository requires Node `>=22.12`. Browser and production-build verification therefore used the existing local Node 24 executable without changing dependencies or repository engine requirements.
