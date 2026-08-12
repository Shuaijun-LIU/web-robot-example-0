# Multi-Robot Ring Layout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build physically shared Franka, SO-101, and XLeRobot multi-robot layouts in the existing browser MuJoCo example and deploy the verified result to GitHub Pages source.

**Architecture:** Keep one MuJoCo model and one canvas per robot selection. A small JavaScript layout module owns pure layout data, pose repetition, MJCF `replicate` patches, and table geometry so Node tests can exercise the exact production data; `configs.ts` maps those values into `SceneConfig`. MuJoCo namespaces every replicated body, joint, actuator, tendon, equality, and contact reference.

**Tech Stack:** React 19, TypeScript, Vite, Three.js/R3F, mujoco-react, mujoco-js WASM, Node test runner, browser screenshot automation.

## Global Constraints

- Franka and SO-101 each contain exactly four physical robot instances facing the center.
- XLeRobot contains exactly two physical robot instances facing one another.
- XLeRobot table-top height is exactly `0.775 m`.
- Existing upstream meshes remain remote; do not vendor large robot asset trees.
- Existing primary-instance controls remain functional.
- Build output for GitHub Pages remains in `docs/` with base `/web-robot-example-0/`.
- Push through the configured SSH remote without force-pushing or rewriting history.

---

### Task 1: Pure multi-robot layout model

**Files:**
- Create: `src/sceneLayouts.js`
- Create: `src/sceneLayouts.d.ts`
- Create: `test/scene-layouts.test.mjs`
- Modify: `package.json`

**Interfaces:**
- Produces: `repeatPose(pose: number[], count: number): number[]`.
- Produces: `FRANKA_LAYOUT`, `SO101_LAYOUT`, and `XLEROBOT_LAYOUT` objects containing counts, transforms, home controls, XML patches, central objects, camera, and orbit target.
- Consumes: no application state or browser APIs.

- [ ] **Step 1: Write the failing layout tests**

```js
import assert from 'node:assert/strict';
import test from 'node:test';
import { FRANKA_LAYOUT, SO101_LAYOUT, XLEROBOT_LAYOUT, repeatPose } from '../src/sceneLayouts.js';

test('repeatPose creates independent actuator homes for every robot', () => {
  assert.deepEqual(repeatPose([1, 2], 3), [1, 2, 1, 2, 1, 2]);
  assert.equal(FRANKA_LAYOUT.homeJoints.length, 32);
  assert.equal(SO101_LAYOUT.homeJoints.length, 24);
  assert.equal(XLEROBOT_LAYOUT.homeJoints.length, 32);
});

test('arm layouts replicate inward around one center', () => {
  assert.equal(FRANKA_LAYOUT.instanceCount, 4);
  assert.equal(FRANKA_LAYOUT.yawStepDegrees, 90);
  assert.equal(SO101_LAYOUT.instanceCount, 4);
  assert.equal(SO101_LAYOUT.yawStepDegrees, 90);
});

test('XLeRobot table top aligns with arm mounting bases', () => {
  assert.equal(XLEROBOT_LAYOUT.instanceCount, 2);
  assert.equal(XLEROBOT_LAYOUT.yawStepDegrees, 180);
  assert.equal(XLEROBOT_LAYOUT.armBaseHeight, 0.775);
  assert.equal(XLEROBOT_LAYOUT.tableTopHeight, XLEROBOT_LAYOUT.armBaseHeight);
  assert.equal(XLEROBOT_LAYOUT.tableObjects.filter((object) => object.name.startsWith('table_leg_')).length, 4);
});
```

- [ ] **Step 2: Run the test and verify RED**

Run: `node --test test/scene-layouts.test.mjs`

Expected: FAIL with `ERR_MODULE_NOT_FOUND` for `src/sceneLayouts.js`.

- [ ] **Step 3: Implement the pure layout module and declarations**

Implement exact ring transforms using MJCF patches:

```js
export function repeatPose(pose, count) {
  return Array.from({ length: count }, () => pose).flat();
}

export const FRANKA_LAYOUT = {
  instanceCount: 4,
  yawStepDegrees: 90,
  ringRadius: 0.72,
  primaryTcpSite: 'tcp_0',
  primaryGripperActuator: 'gripper_0',
};
```

The complete objects must encode SO-101's `0.80 m` surface and XLeRobot's top plus four legs at `0.775 m`.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `node --test test/scene-layouts.test.mjs`

Expected: all layout tests PASS.

- [ ] **Step 5: Add the aggregate test script and commit**

Add `"test": "node --test test/*.test.mjs"`, run `npm test`, then commit:

```bash
git add package.json src/sceneLayouts.js src/sceneLayouts.d.ts test/scene-layouts.test.mjs
git commit -m "feat: define multi-robot layouts"
```

### Task 2: Connect layouts to runtime scenes

**Files:**
- Modify: `src/configs.ts`
- Modify: `src/controllers/FrankaController.tsx`
- Modify: `test/scene-layouts.test.mjs`

**Interfaces:**
- Consumes: `FRANKA_LAYOUT`, `SO101_LAYOUT`, `XLEROBOT_LAYOUT` from Task 1.
- Produces: the existing `robots: Record<string, RobotEntry>` API with multi-robot `SceneConfig` values.

- [ ] **Step 1: Add failing integration assertions**

Read `src/configs.ts` and `src/controllers/FrankaController.tsx` in the Node test and assert that configs consume all three layout objects, Franka IK uses `primaryTcpSite`, and the gripper binding uses `primaryGripperActuator`.

- [ ] **Step 2: Run the test and verify RED**

Run: `npm test`

Expected: FAIL because `configs.ts` still contains single-robot inline layouts and `FrankaController` still names actuator `gripper`.

- [ ] **Step 3: Replace inline single-robot geometry with layout data**

Import the three layout objects, assign their XML patches, repeated home arrays, objects, camera, and target, and keep public robot keys unchanged. Set Franka's first IK site to `tcp_0`; preserve SO-101 and XLeRobot primary actuator indices at the first namespace block.

- [ ] **Step 4: Update the primary Franka gripper binding**

Bind `V` to `FRANKA_LAYOUT.primaryGripperActuator`, whose generated MuJoCo name is `gripper_0`.

- [ ] **Step 5: Verify tests and type checking**

Run: `npm test && npx tsc --noEmit`

Expected: all tests PASS and TypeScript exits `0` without diagnostics.

- [ ] **Step 6: Commit runtime integration**

```bash
git add src/configs.ts src/controllers/FrankaController.tsx test/scene-layouts.test.mjs
git commit -m "feat: render shared multi-robot scenes"
```

### Task 3: Compile and validate the MuJoCo scenes in a browser

**Files:**
- Create: `scripts/capture-scenes.mjs`
- Create: `artifacts/screenshots/.gitkeep`
- Modify: `package.json`
- Modify: `package-lock.json`

**Interfaces:**
- Consumes: the running Vite URL and Leva's robot selector.
- Produces: `artifacts/screenshots/franka.png`, `so101.png`, and `xlerobot.png` plus browser-console/model-load failure reporting.

- [ ] **Step 1: Write the screenshot smoke runner**

The runner starts Chromium at `1280×800`, opens the local page, waits for `Loading model...` to disappear, captures the default Franka scene, selects SO-101 and XLeRobot through Leva, and captures each settled scene. It must throw on page errors, `Simulation error`, or a loading timeout.

- [ ] **Step 2: Start Vite and run the smoke runner**

Run in separate processes:

```bash
npm run dev -- --host 127.0.0.1
npm run capture:scenes
```

Expected: three nonempty PNG files and no MuJoCo XML compilation errors.

- [ ] **Step 3: Inspect all screenshots**

Verify Franka count `4`, SO-101 count `4`, XLeRobot count `2`; all complete models are upright; central items do not intersect robot bases; all robots point inward; XLeRobot table top intersects neither robot and visually aligns with arm bases.

- [ ] **Step 4: Tune only layout transforms when inspection fails**

Change ring radius, camera position, orbit target, or table X/Y dimensions in `sceneLayouts.js`; do not replace physical instances with visual clones. Re-run `npm test`, type checking, and all three screenshots after every adjustment.

- [ ] **Step 5: Commit reproducible visual verification**

```bash
git add package.json package-lock.json scripts/capture-scenes.mjs artifacts/screenshots
git commit -m "test: add multi-scene visual smoke check"
```

### Task 4: Documentation and GitHub Pages build

**Files:**
- Modify: `README.md`
- Modify: `docs/index.html`
- Modify: `docs/assets/*`

**Interfaces:**
- Consumes: verified scene behavior from Tasks 1–3.
- Produces: current usage documentation and deployable static output.

- [ ] **Step 1: Update the README scene descriptions**

Document four-arm Franka, four-arm SO-101, two-robot XLeRobot, the primary-instance control limitation for this layout milestone, and the table-height invariant.

- [ ] **Step 2: Run full verification**

Run:

```bash
npm test
npx tsc --noEmit
npm run build
npm run build:pages
git diff --check
```

Expected: every command exits `0`; `docs/index.html` references generated hashed assets under `/web-robot-example-0/`.

- [ ] **Step 3: Commit documentation and deployment output**

```bash
git add README.md docs
git commit -m "docs: publish multi-robot scene build"
```

### Task 5: Push and confirm remote deployment source

**Files:**
- No source files added in this task.

**Interfaces:**
- Consumes: clean verified `main` branch.
- Produces: updated `origin/main` and a reported commit SHA.

- [ ] **Step 1: Inspect account and remote routing**

Run: `gh auth status`, `git remote -v`, `git branch --show-current`, `git status --short`.

Expected: current branch is `main`; remote uses the configured SSH alias for `Shuaijun-LIU`; worktree is clean.

- [ ] **Step 2: Fetch and confirm fast-forward safety**

Run: `git fetch origin --prune` and compare `git rev-list --left-right --count origin/main...main`.

Expected: local branch is not behind the remote. If it is behind, stop the push and reconcile without force.

- [ ] **Step 3: Push the verified commits**

Run: `git push origin main`.

Expected: normal fast-forward push succeeds.

- [ ] **Step 4: Report final evidence**

Report the final commit SHA, pushed remote branch, verification commands, and absolute links to the three screenshots, design record, implementation plan, and principal source files.
