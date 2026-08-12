# Multi-Robot MuJoCo Web Example

An interactive browser simulation built with React, Three.js, `mujoco-react`, and MuJoCo WASM. The three selectable scenes use independent physical robot instances—not visual-only clones.

[Open the live demo](https://shuaijun-liu.github.io/web-robot-example-0/)

## Verified scenes

| Franka Panda — 4 arms | SO101 — 4 arms | XLeRobot — 2 robots |
|---|---|---|
| [![Four Franka Panda arms around graspable cubes](artifacts/screenshots/franka.png)](artifacts/screenshots/franka.png) | [![Four SO101 arms on a shared work table](artifacts/screenshots/so101.png)](artifacts/screenshots/so101.png) | [![Two XLeRobots facing across an arm-height table](artifacts/screenshots/xlerobot.png)](artifacts/screenshots/xlerobot.png) |

| Scene | Physical layout | Shared workspace |
|---|---|---|
| Franka Panda | Four 7-DOF arms at 90° intervals, facing the center | Three free-joint, graspable cubes |
| SO101 | Four 6-actuator arms at 90° intervals, facing the center | One table with a `0.800 m` top and three graspable cubes |
| XLeRobot | Two complete dual-arm mobile robots, facing one another | One four-leg table; its top is exactly `0.775 m`, matching the arm mounting height |

The app starts paused so the requested layout remains stable for inspection. Uncheck **paused** to run physics. The IK gizmo is available from the control panel but starts hidden for an unobstructed overview.

## Run locally

Node.js 22 is recommended and is also used by the deployment workflow.

```bash
nvm use
npm ci
npm run dev
```

Open `http://localhost:3000`. The robot mesh files are loaded from their upstream model repositories, so the first visit may take longer than later cached visits.

## Controls

The current milestone keeps keyboard and IK control on the primary instance (`r0_`) while every displayed robot remains an independent MuJoCo model with its own bodies, joints, actuators, tendons, and collision geometry.

| Key | Franka | SO101 | XLeRobot primary instance |
|---|---|---|---|
| WASD | — | End-effector forward/back/up/down | Drive base |
| Q/E | — | End-effector left/right | — |
| R/F | — | Wrist pitch | Head pan |
| Z/C | — | Wrist roll | — |
| V | Toggle gripper | Toggle gripper | Left gripper |
| B | — | — | Right gripper |
| 7–0, Y/U/I/O | — | — | Left arm |
| H–L, N–/ | — | — | Right arm |

The panel also provides pause, speed, gravity compensation, reset, IK gizmo, contacts, sites, and joints controls. Double-click selects a body; Ctrl/Cmd-click drags a body.

## Implementation

The layouts are centralized in [`src/sceneLayouts.js`](src/sceneLayouts.js). Each upstream robot is loaded as an MJCF model asset and inserted into a parent scene with MuJoCo `attach` elements and per-instance prefixes such as `r0_`, `r1_`, and so on. This keeps cross-references namespaced and produces independent physics for every instance.

Runtime selection and cameras are defined in [`src/configs.ts`](src/configs.ts). Browser smoke tests read the compiled model through the simulation API and reject a scene unless it contains exactly 4 Franka roots, 4 SO101 roots, or 2 XLeRobot roots.

## Verification

```bash
npm test
npx tsc --noEmit
npm run build
npm run build:pages
```

For reproducible screenshots, run the Vite server and then:

```bash
npm run capture:scenes
```

`FRANKA_ASSET_DIR` and `XLEROBOT_ASSET_DIR` can point the capture script at local copies of the upstream assets to avoid repeated network downloads. The offline compiler helper is available as `node scripts/validate-mjcf.mjs <scene> <asset-directory>`.

## GitHub Pages

Every push to `main` runs [`.github/workflows/pages.yml`](.github/workflows/pages.yml), executes tests and type checking, builds with the `/web-robot-example-0/` base path, and deploys the generated site through GitHub Pages Actions.

## Model sources

- Franka Panda: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
- SO101 and XLeRobot: [MuJoCo-GS-Web](https://github.com/Vector-Wangel/MuJoCo-GS-Web/tree/main/assets/robots/xlerobot)
