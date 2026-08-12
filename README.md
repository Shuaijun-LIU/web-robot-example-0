# mujoco-react example

Interactive example app for [mujoco-react](https://www.npmjs.com/package/mujoco-react) — showcasing composable robot simulation with multiple robots, controllers, and debug tools.

## Robots

| Robot | Controller | Features |
|-------|-----------|----------|
| **Franka Panda** | IK gizmo + gripper toggle (V) | 7-DOF arm, gizmo drag, click-to-select, graspable cubes |
| **SO101** | Keyboard IK + gizmo | 2-link IK (WASD/QE), wrist (R/F, Z/C), gripper (V) |
| **XLeRobot** | Keyboard + dual arms | Mobile base (WASD), dual-arm IK, head pan/tilt, grippers (V/B) |

## Getting Started

```bash
npm install
npm run dev
```

## Architecture

The app demonstrates the **composable children** pattern — each feature is an independent R3F child you drop in or remove:

```tsx
<MujocoProvider>
  <MujocoCanvas config={entry.config} paused={sim.paused} speed={sim.speed}>
    <OrbitControls />
    <SceneRenderer />

    {/* Opt-in IK controller plugin — wraps IkGizmo */}
    {entry.hasIk && sim.gizmo && (
      <IkController config={{ siteName: entry.config.tcpSiteName!, numJoints: entry.config.numArmJoints! }}>
        <IkGizmo scale={entry.gizmoScale} />
      </IkController>
    )}
    <DragInteraction />
    <ClickSelectOverlay />

    {/* Per-robot controllers — swap in your own */}
    {robotKey === 'franka' && <FrankaController />}
    {robotKey === 'so101' && <SO101Controller />}
    {robotKey === 'xlerobot' && <XLeRobotController />}

    {/* Debug overlays */}
    <ContactMarkers visible={debug.contacts} />
    <Debug showSites={debug.sites} showJoints={debug.joints} />

    {/* Scene decoration */}
    <ambientLight />
    <directionalLight castShadow />
  </MujocoCanvas>
</MujocoProvider>
```

## Controllers

Controllers are thin wrappers around library hooks. Each robot's controller is a static config object passed to a generic hook — no per-robot logic.

### FrankaController

The simplest possible controller — one `useKeyboardTeleop` call:

```tsx
import { useKeyboardTeleop } from 'mujoco-react';

export function FrankaController() {
  useKeyboardTeleop({
    bindings: { v: { actuator: 'gripper', toggle: [0, 255] } },
  });
  return null;
}
```

Arm positioning comes from `<IkGizmo />` inside `<IkController>` — the controller only adds the gripper toggle.

### SO101Controller / XLeRobotController

Pure config objects passed to `useArmController`:

```tsx
import { useArmController } from './useArmController';

const config: ArmControllerConfig = {
  numActuators: 6,
  arms: [{
    indices: [0, 1, 2, 3, 4, 5],
    keys: ['KeyD', 'KeyA', 'KeyW', 'KeyS', 'KeyQ', 'KeyE',
           'KeyR', 'KeyF', 'KeyZ', 'KeyC', 'KeyV'],
    initialJoints: [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745],
  }],
};

export function SO101Controller() {
  useArmController(config);
  return null;
}
```

### useArmController

Generic hook for keyboard-driven arm control with 2-link IK. Accepts a static config:

```ts
interface ArmControllerConfig {
  numActuators: number;
  base?: BaseConfig;     // Mobile base (WASD drive)
  arms: ArmConfig[];     // One or more arms with IK
  head?: HeadConfig;     // Pan/tilt head
}

interface ArmConfig {
  indices: number[];       // Actuator indices
  keys: string[];          // 11 key bindings (rotation, EE, pitch, roll, gripper)
  initialJoints?: number[];  // Start position (uses FK to sync IK state)
  initialRotation?: number;
  initialRoll?: number;
}
```

Features:
- **Gizmo coexistence** — uses `useIk({ optional: true })` to detect if an IkController is active. When arm keys are pressed, automatically disables the library's IK and syncs from the current arm position so there's no jump
- **Per-arm IK** — each arm has independent 2-link IK state
- **Config-driven** — swap robots by changing the config object, no code changes

### Click-to-Select

A hook + component pattern:

```tsx
function ClickSelectOverlay() {
  const selectedBodyId = useClickSelect(); // double-click raycasting hook
  return <SelectionHighlight bodyId={selectedBodyId} />;
}
```

## Graspable Objects

For objects that need to be picked up by grippers, set these MuJoCo contact parameters:

```tsx
sceneObjects: [{
  name: 'cube',
  type: 'box',
  size: [0.025, 0.025, 0.025],
  position: [0.4, 0, 0.025],
  rgba: [0.9, 0.2, 0.15, 1],
  mass: 0.05,
  freejoint: true,
  friction: '1.5 0.3 0.1',            // high static friction
  solref: '0.01 1',                    // stiff contact solver
  solimp: '0.95 0.99 0.001 0.5 2',    // tight impedance
  condim: 4,                           // 4D contact (friction cone)
}]
```

Without `condim: 4` and high friction, blocks will slide out of the gripper when lifted.

## Control Panel

The [Leva](https://github.com/pmndrs/leva) panel provides runtime controls:

- **Robot** — switch between Franka, SO101, XLeRobot
- **Simulation** — pause, speed, gravity compensation, IK gizmo toggle, reset
- **Debug** — contacts, sites, joints visualization

## Key Bindings

| Key | Franka | SO101 | XLeRobot |
|-----|--------|-------|----------|
| WASD | -- | EE forward/back/up/down | Base drive |
| Q/E | -- | EE left/right | -- |
| R/F | -- | Wrist pitch | Head pan |
| Z/C | -- | Wrist roll | -- |
| V | Gripper toggle | Gripper toggle | Left gripper |
| B | -- | -- | Right gripper |
| 7-0, Y/U/I/O | -- | -- | Left arm |
| H-L, N-/ | -- | -- | Right arm |

## Building

```bash
npm run build     # Production build
npx tsc --noEmit  # Type check
```

## GitHub Pages

Build the static site for this repository's GitHub Pages URL:

```bash
npm run build:pages
```

Commit the generated `docs/` directory. In GitHub, open **Settings → Pages** and set the source to **Deploy from a branch**, with branch **main** and folder **/docs**. The site will be available at `https://shuaijun-liu.github.io/web-robot-example-0/`.
