# PiPER and UR5e Assembly1 Scenes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two local, four-arm Assembly1-derived scenes using AgileX PiPER and UR5e + Robotiq 2F-85, with independently selectable IK and gripper control.

**Architecture:** Vendor licensed Menagerie robot packages under each scene, compose UR5e and 2F-85 through nested MJCF attachment, and keep the Assembly1 workcell snapshot in a new alternate-layout module so Franka automation is untouched. Extend the existing data-driven control-target and selected-IK path with robot-specific target factories and a generic gripper profile.

**Tech Stack:** React 19, TypeScript, Vite, `mujoco-react`, `mujoco-js`/MuJoCo WASM, Node test runner, Playwright.

## Global Constraints

- Existing `frankaAssembly1`, its three-step automation, and its default page selection must remain unchanged.
- Add exactly two new scene keys: `piperAssembly1` and `ur5eAssembly1`.
- Each new scene contains exactly four fixed robot instances facing the Assembly1 workcell.
- PiPER uses the MIT Menagerie model and original gripper; UR5e uses BSD-3-Clause Menagerie plus BSD-2-Clause Robotiq 2F-85.
- Runtime scene assets must be local beneath `public/assets/`; do not depend on GitHub raw URLs.
- New scenes are manual-only: selected-arm IK, keyboard/mouse interaction, and selected gripper control; no Franka automation controller.
- Do not modify or stage files in the canonical dirty checkout.

---

### Task 1: Vendor and compose the licensed robot packages

**Files:**
- Create: `public/assets/piper-assembly1/scene.xml`
- Create: `public/assets/piper-assembly1/THIRD_PARTY_NOTICES.md`
- Create: `public/assets/piper-assembly1/robots/piper/{piper.xml,LICENSE,README.md,assets/**}`
- Create: `public/assets/ur5e-assembly1/scene.xml`
- Create: `public/assets/ur5e-assembly1/THIRD_PARTY_NOTICES.md`
- Create: `public/assets/ur5e-assembly1/robots/ur5e/{ur5e.xml,LICENSE,README.md,assets/**}`
- Create: `public/assets/ur5e-assembly1/robots/robotiq_2f85/{2f85.xml,LICENSE,README.md,assets/**}`
- Test: `test/alternate-assembly-assets.test.mjs`

**Interfaces:**
- Consumes: local Menagerie directories `agilex_piper`, `universal_robots_ur5e`, and `robotiq_2f85`.
- Produces: `piper_model` rooted at `base_link`; `ur5e_model` rooted at `base`, containing a `gripper_`-prefixed Robotiq subtree and `gripper_pinch` site.

- [ ] **Step 1: Write the failing provenance and composition test**

```js
test('alternate assembly assets are local, licensed, and composable', async () => {
  const piper = await readFile(asset('piper-assembly1/robots/piper/piper.xml'), 'utf8');
  const ur5e = await readFile(asset('ur5e-assembly1/robots/ur5e/ur5e.xml'), 'utf8');
  assert.match(piper, /<body name="base_link"/);
  assert.match(piper, /<site name="tcp"/);
  assert.doesNotMatch(piper, /<keyframe>/);
  assert.match(ur5e, /<model name="robotiq_model" file="\.\.\/robotiq_2f85\/2f85\.xml"\/>/);
  assert.match(ur5e, /<attach model="robotiq_model" body="base_mount" prefix="gripper_"\/>/);
  assert.doesNotMatch(ur5e, /<keyframe>/);
  assert.match(await readFile(asset('piper-assembly1/robots/piper/LICENSE'), 'utf8'), /MIT License/);
  assert.match(await readFile(asset('ur5e-assembly1/robots/ur5e/LICENSE'), 'utf8'), /Redistribution and use/);
  assert.match(await readFile(asset('ur5e-assembly1/robots/robotiq_2f85/LICENSE'), 'utf8'), /Redistribution and use/);
});
```

- [ ] **Step 2: Run the asset test and verify it fails because the scene packages do not exist**

Run: `node --test test/alternate-assembly-assets.test.mjs`

Expected: FAIL with `ENOENT` below `public/assets/piper-assembly1`.

- [ ] **Step 3: Copy the upstream packages mechanically**

```bash
mkdir -p public/assets/piper-assembly1/robots/piper
mkdir -p public/assets/ur5e-assembly1/robots/ur5e
mkdir -p public/assets/ur5e-assembly1/robots/robotiq_2f85
cp -a "$MENAGERIE/agilex_piper/assets" public/assets/piper-assembly1/robots/piper/
cp "$MENAGERIE/agilex_piper/piper.xml" "$MENAGERIE/agilex_piper/LICENSE" "$MENAGERIE/agilex_piper/README.md" public/assets/piper-assembly1/robots/piper/
cp -a "$MENAGERIE/universal_robots_ur5e/assets" public/assets/ur5e-assembly1/robots/ur5e/
cp "$MENAGERIE/universal_robots_ur5e/ur5e.xml" "$MENAGERIE/universal_robots_ur5e/LICENSE" "$MENAGERIE/universal_robots_ur5e/README.md" public/assets/ur5e-assembly1/robots/ur5e/
cp -a "$MENAGERIE/robotiq_2f85/assets" public/assets/ur5e-assembly1/robots/robotiq_2f85/
cp "$MENAGERIE/robotiq_2f85/2f85.xml" "$MENAGERIE/robotiq_2f85/LICENSE" "$MENAGERIE/robotiq_2f85/README.md" public/assets/ur5e-assembly1/robots/robotiq_2f85/
```

Set `MENAGERIE=/data/private/user2/workspace/7.web-robot/1.source-repos/google-deepmind__mujoco_menagerie` before running.

- [ ] **Step 4: Patch PiPER for replication and IK**

Remove the upstream `<keyframe>` block and insert this site as a child of `link6`, beside the two finger bodies:

```xml
<site name="tcp" pos="0 0 0.18" size="0.006" rgba="0.75 0.18 0.12 0.7" group="1"/>
```

- [ ] **Step 5: Patch UR5e to attach Robotiq and remove the keyframe**

Add to the UR5e `<asset>` block:

```xml
<model name="robotiq_model" file="../robotiq_2f85/2f85.xml"/>
```

Add under `wrist_3_link`, matching the existing attachment site transform:

```xml
<frame pos="0 0.1 0" quat="-1 1 0 0">
  <attach model="robotiq_model" body="base_mount" prefix="gripper_"/>
</frame>
```

Remove the UR5e `<keyframe>` block so four outer attachments cannot lose nested keyframe data.

- [ ] **Step 6: Create minimal local scene roots and notices**

PiPER root:

```xml
<mujoco model="piper assembly1">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <asset><model name="piper_model" file="robots/piper/piper.xml"/></asset>
  <worldbody><light pos="0 -2 4" dir="0 .4 -1" directional="true"/></worldbody>
</mujoco>
```

UR5e root is identical except for model/name:

```xml
<asset><model name="ur5e_model" file="robots/ur5e/ur5e.xml"/></asset>
```

Each notice records the source Menagerie directory, robot name, license name, and retained license path.

- [ ] **Step 7: Run the asset test**

Run: `node --test test/alternate-assembly-assets.test.mjs`

Expected: PASS.

- [ ] **Step 8: Commit the licensed assets**

```bash
git add public/assets/piper-assembly1 public/assets/ur5e-assembly1 test/alternate-assembly-assets.test.mjs
git commit -m "feat(assets): vendor PiPER and UR5e assembly robots"
```

### Task 2: Define the two four-arm layouts and workcell snapshot

**Files:**
- Create: `src/alternateAssemblyLayouts.js`
- Create: `src/alternateAssemblyLayouts.d.ts`
- Test: `test/alternate-assembly-layouts.test.mjs`

**Interfaces:**
- Consumes: `fixedBox()` and `repeatPose()` from `src/sceneLayouts.js`; local scene roots from Task 1.
- Produces: `PIPER_ASSEMBLY1_LAYOUT`, `UR5E_ASSEMBLY1_LAYOUT`, `PIPER_HOME`, and `UR5E_HOME`.

- [ ] **Step 1: Write failing layout-contract tests**

```js
for (const [layout, stride] of [
  [PIPER_ASSEMBLY1_LAYOUT, 7],
  [UR5E_ASSEMBLY1_LAYOUT, 7],
]) {
  assert.equal(layout.instanceCount, 4);
  assert.equal(layout.homeJoints.length, stride * 4);
  assert.equal(layout.workSurfaceHeight, 0.1);
  assert.deepEqual(layout.taskStations.frame, [0, 0, 0.275]);
  assert.match(layoutXml(layout), /name="assembly_frame"/);
  assert.match(layoutXml(layout), /name="manual_screwdriver"/);
  assert.match(layoutXml(layout), /name="torque_driver"/);
  assert.match(layoutXml(layout), /name="double_face_hammer"/);
}
assert.match(layoutXml(PIPER_ASSEMBLY1_LAYOUT), /attach model="piper_model" body="base_link" prefix="r0_"/);
assert.match(layoutXml(UR5E_ASSEMBLY1_LAYOUT), /attach model="ur5e_model" body="base" prefix="r0_"/);
```

- [ ] **Step 2: Run the layout test and verify missing-module failure**

Run: `node --test test/alternate-assembly-layouts.test.mjs`

Expected: FAIL with `ERR_MODULE_NOT_FOUND` for `alternateAssemblyLayouts.js`.

- [ ] **Step 3: Implement focused alternate layout definitions**

Define:

```js
export const PIPER_HOME = [0, 1.57, -1.3485, 0, 0, 0, 0.035];
export const UR5E_HOME = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0, 0];
```

Use one `createAssemblyLayout({ model, body, radius, home, tcp, gripper })`
factory. It must inject four frames at yaw `0`, `90`, `180`, and `-90`, all at
z=`0.1`; inject the Assembly1 tool mesh, workcell bodies, and detailed tools;
and return the copied platform/inset/pads/cradles as `sceneObjects`.

Use a `0.78 m` initial PiPER ring radius and `0.90 m` initial UR5e ring radius.
Keep all Assembly1 task coordinates except allow only the PiPER hammer mat and
shelf x coordinate to move from `0.65` to `0.58` if the strict contact report
detects base overlap.

- [ ] **Step 4: Add exact TypeScript declarations**

```ts
export interface AlternateAssemblyLayout extends SceneLayout {
  ringRadius: number;
  workSurfaceHeight: number;
  primaryTcpSite: string;
  primaryGripperActuator: string;
  taskStations: Record<string, [number, number, number]>;
}
export const PIPER_HOME: number[];
export const UR5E_HOME: number[];
export const PIPER_ASSEMBLY1_LAYOUT: AlternateAssemblyLayout;
export const UR5E_ASSEMBLY1_LAYOUT: AlternateAssemblyLayout;
```

- [ ] **Step 5: Run the layout tests**

Run: `node --test test/alternate-assembly-layouts.test.mjs`

Expected: PASS.

- [ ] **Step 6: Commit the layouts**

```bash
git add src/alternateAssemblyLayouts.js src/alternateAssemblyLayouts.d.ts test/alternate-assembly-layouts.test.mjs
git commit -m "feat(scene): define alternate Assembly1 layouts"
```

### Task 3: Add selected-arm IK and generic gripper profiles

**Files:**
- Modify: `src/controlTargets.js`
- Modify: `src/controlTargets.d.ts`
- Modify: `src/controllers/controllerConfigs.js`
- Modify: `src/controllers/controllerConfigs.d.ts`
- Create: `src/controllers/IndustrialArmController.tsx`
- Test: `test/control-targets.test.mjs`
- Test: `test/controller-configs.test.mjs`

**Interfaces:**
- Consumes: existing `ControlTarget`, `shiftIndices`, `useKeyboardTeleop`, and selected-IK controller.
- Produces: `createPiperTargets()`, `createUR5eTargets()`, `GripperControlDefinition`, and `getIndustrialGripperBinding()`.

- [ ] **Step 1: Write failing target and binding tests**

```js
assert.deepEqual(createPiperTargets()[0].ik.jointNames, [
  'r0_joint1', 'r0_joint2', 'r0_joint3', 'r0_joint4', 'r0_joint5', 'r0_joint6',
]);
assert.equal(createPiperTargets()[3].gripperControl.actuator, 'r3_gripper');
assert.deepEqual(createUR5eTargets()[0].ik.jointNames, [
  'r0_shoulder_pan_joint', 'r0_shoulder_lift_joint', 'r0_elbow_joint',
  'r0_wrist_1_joint', 'r0_wrist_2_joint', 'r0_wrist_3_joint',
]);
assert.equal(createUR5eTargets()[2].ik.siteName, 'r2_gripper_pinch');
assert.deepEqual(
  getIndustrialGripperBinding(createUR5eTargets()[0]),
  { v: { actuator: 'r0_gripper_fingers_actuator', toggle: [0, 255] } },
);
```

- [ ] **Step 2: Run the focused tests and verify missing exports**

Run: `node --test test/control-targets.test.mjs test/controller-configs.test.mjs`

Expected: FAIL because the two factories and industrial binding do not exist.

- [ ] **Step 3: Extend the target data contract**

Add:

```ts
export interface GripperControlDefinition {
  actuator: string;
  openValue: number;
  closedValue: number;
}
```

Add optional `gripperControl?: GripperControlDefinition` to `ControlTarget`.
PiPER target stride is seven, site is `${prefix}tcp`, six arm actuator indices
start at the instance offset, and gripper values are `0.035` open / `0` closed.
UR5e target stride is seven, site is `${prefix}gripper_pinch`, six arm actuator
indices start at the instance offset, and gripper values are `0` open / `255`
closed.

- [ ] **Step 4: Implement and type the generic binding**

```js
export function getIndustrialGripperBinding(target) {
  const gripper = target.gripperControl;
  if (!gripper || !Number.isFinite(gripper.openValue) || !Number.isFinite(gripper.closedValue)) {
    throw new Error(`Control target ${target.key} has no valid gripper control`);
  }
  return {
    v: {
      actuator: gripper.actuator,
      toggle: [gripper.openValue, gripper.closedValue],
    },
  };
}
```

- [ ] **Step 5: Add `IndustrialArmController`**

```tsx
export function IndustrialArmController({ target }: { target: ControlTarget }) {
  useKeyboardTeleop({ bindings: getIndustrialGripperBinding(target), enabled: true });
  return null;
}
```

- [ ] **Step 6: Run target/config tests**

Run: `node --test test/control-targets.test.mjs test/controller-configs.test.mjs`

Expected: PASS, including four unique target prefixes and correct per-instance actuator offsets.

- [ ] **Step 7: Commit control support**

```bash
git add src/controlTargets.* src/controllers/controllerConfigs.* src/controllers/IndustrialArmController.tsx test/control-targets.test.mjs test/controller-configs.test.mjs
git commit -m "feat(control): support alternate industrial arms"
```

### Task 4: Register both scenes and extend offline validation

**Files:**
- Modify: `src/configs.ts`
- Modify: `src/App.tsx`
- Modify: `src/KeyboardHelp.tsx`
- Modify: `scripts/validate-mjcf.mjs`
- Test: `test/alternate-assembly-scenes.test.mjs`
- Modify: `test/ui-contract.test.mjs`

**Interfaces:**
- Consumes: both layouts, target factories, `IndustrialArmController`, and local asset roots.
- Produces: selectable `piperAssembly1` and `ur5eAssembly1` `RobotEntry` records plus four-instance lifecycle diagnostics.

- [ ] **Step 1: Write failing catalog/UI/compile tests**

```js
const configSource = await readFile(new URL('../src/configs.ts', import.meta.url), 'utf8');
assert.match(configSource, /piperAssembly1:\s*\{[\s\S]*label: 'Piper Assembly1'/);
assert.match(configSource, /ur5eAssembly1:\s*\{[\s\S]*label: 'UR5e Assembly1'/);
assert.match(configSource, /assets\/piper-assembly1\//);
assert.match(configSource, /assets\/ur5e-assembly1\//);
assert.match(configSource, /controlTargets: createPiperTargets\(\)/);
assert.match(configSource, /controlTargets: createUR5eTargets\(\)/);
```

The subprocess assertions must match successful `validate-mjcf.mjs` output and
run with `LIST_NAMES=1`, `POSE_REPORT=1`, and `INITIAL_CONTACT_REPORT=1`.

- [ ] **Step 2: Run the scene/UI tests and verify missing scene failures**

Run: `node --test test/alternate-assembly-scenes.test.mjs test/ui-contract.test.mjs`

Expected: FAIL because the catalog has neither key and the validator rejects them.

- [ ] **Step 3: Register two local scene entries**

Add local bases:

```ts
const PIPER_ASSEMBLY1_BASE = `${import.meta.env.BASE_URL}assets/piper-assembly1/`;
const UR5E_ASSEMBLY1_BASE = `${import.meta.env.BASE_URL}assets/ur5e-assembly1/`;
```

Both entries use `controlFamily: 'industrialArm'`, their respective layout,
`scene.xml`, and target factory. Extend the `RobotEntry` and `SceneChildren`
control-family unions with `industrialArm`.

- [ ] **Step 4: Wire manual control without Franka automation**

Render:

```tsx
{controlFamily === 'industrialArm' && (
  <IndustrialArmController key={`industrial-${target.key}`} target={target} />
)}
```

Add root patterns:

```ts
piperAssembly1: /^r\d+_base_link$/,
ur5eAssembly1: /^r\d+_base$/,
```

Do not broaden any `robotKey === 'frankaAssembly1'` automation condition.
Add `industrialArm` keyboard help with the existing `V`, IK-gizmo, drag, and
selection copy.

- [ ] **Step 5: Extend the validator**

Import both layouts, add the two `definitions`, add root-name mappings
`base_link` and `base`, and preserve the generic `rN_tcp` convention by using
`layout.primaryTcpSite` with the selected instance prefix instead of assuming
the literal suffix `tcp`.

- [ ] **Step 6: Run offline compile and strict initial-contact reports**

```bash
LIST_NAMES=1 POSE_REPORT=1 INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs piperAssembly1 public/assets/piper-assembly1
LIST_NAMES=1 POSE_REPORT=1 INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs ur5eAssembly1 public/assets/ur5e-assembly1
```

Expected: both compile with four roots, four TCP sites, 28 actuators, finite
poses, and no penetration deeper than 5 mm. If PiPER overlaps the hammer shelf,
apply only the specified x shift to `0.58` and rerun.

- [ ] **Step 7: Run catalog/UI tests**

Run: `node --test test/alternate-assembly-scenes.test.mjs test/ui-contract.test.mjs`

Expected: PASS.

- [ ] **Step 8: Commit scene registration**

```bash
git add src/configs.ts src/App.tsx src/KeyboardHelp.tsx scripts/validate-mjcf.mjs test/alternate-assembly-scenes.test.mjs test/ui-contract.test.mjs
git commit -m "feat(scene): register PiPER and UR5e Assembly1"
```

### Task 5: Browser verification, visual artifacts, and handoff

**Files:**
- Modify: `scripts/capture-scenes.mjs`
- Create: `scripts/verify-alternate-assembly-browser.mjs`
- Create: `artifacts/screenshots/piper-assembly1.png`
- Create: `artifacts/screenshots/ur5e-assembly1.png`
- Create: `docs/progress/2026-08-15-piper-ur5e-assembly-scenes.md`
- Modify: `.codex/project/STATUS.md`
- Modify: `.codex/project/TASKS.md`
- Create: `.codex/project/HANDOFFS/2026-08-15-piper-ur5e-assembly.md`

**Interfaces:**
- Consumes: browser scene diagnostics, Leva selectors, `window.robotDemo`, and both compiled scene entries.
- Produces: repeatable browser proof, screenshots, measured scene counts/TCP control results, and pickup-ready handoff.

- [ ] **Step 1: Add both scenes to screenshot capture metadata**

```js
{ key: 'piperAssembly1', label: 'Piper Assembly1', instances: 4, screenshotName: 'piper-assembly1' },
{ key: 'ur5eAssembly1', label: 'UR5e Assembly1', instances: 4, screenshotName: 'ur5e-assembly1' },
```

- [ ] **Step 2: Write browser verification before relying on screenshots**

For each scene, the script must:

1. Select the scene by label and wait for `sceneStatus === 'ready'`.
2. Assert `sceneInstances === '4'` and no page/console/request error.
3. Switch the control-target selector through `Arm 1`–`Arm 4`.
4. Assert `dataset.ikSite` equals the target's expected TCP name.
5. Press `v`, read `window.robotDemo.getCtrl()`, and assert only the selected
   gripper actuator changes.
6. Enable/move the selected IK target through the exposed controller path and
   assert one selected-arm joint target changes while other arm groups remain
   unchanged.

- [ ] **Step 3: Install dependencies and run the complete non-browser gate**

```bash
npm install
npm test
npx tsc --noEmit
npm run build
```

Expected: all commands exit 0.

- [ ] **Step 4: Run the app and browser verification**

Start `npm run dev -- --host 127.0.0.1` in a persistent exec session, then run:

```bash
node scripts/verify-alternate-assembly-browser.mjs
SCENES=piperAssembly1,ur5eAssembly1 node scripts/capture-scenes.mjs
```

Expected: both scenes report four controllable arms and write both PNG files.

- [ ] **Step 5: Inspect both screenshots at original detail**

Check base/platform contact, arm-neighbor clearance, frame/tool clearance,
gripper orientation, camera framing, and workcell readability. Any visual fix
must be followed by offline strict-contact, browser verification, and capture.

- [ ] **Step 6: Write progress and coordination handoff**

Record exact actuator/body/site counts, strict-contact result, all test/build
commands, screenshots, branch and commit. Set T001 to `Needs review` and link
the handoff.

- [ ] **Step 7: Run final verification and commit**

```bash
git diff --check
npm test
npx tsc --noEmit
npm run build
git status --short
git add scripts/capture-scenes.mjs scripts/verify-alternate-assembly-browser.mjs artifacts/screenshots/piper-assembly1.png artifacts/screenshots/ur5e-assembly1.png docs/progress/2026-08-15-piper-ur5e-assembly-scenes.md .codex/project
git commit -m "test(scene): verify alternate Assembly1 arms"
```

Expected: verification passes and only intentional branch files are committed.
