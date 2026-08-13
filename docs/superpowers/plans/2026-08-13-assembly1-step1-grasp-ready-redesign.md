# Assembly1 Step 1 Grasp-Ready Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Assembly1 Step 1 place all four Panda grippers over physically meaningful grasp regions: one frame rail, one side-laid torque-driver handle, and two balanced cross-member grip points.

**Architecture:** Keep the existing two-phase actuator choreography, but replace the scene pose and waypoint contract with explicit grasp geometry and top-down TCP orientations. Generate the eight joint targets offline from the same Panda MJCF, transforming both world positions and world orientations into each robot's local frame. Extend browser diagnostics so acceptance tests measure both TCP position and orientation while proving task objects remain unscripted.

**Tech Stack:** React 19, TypeScript, Three.js, `mujoco-react`, MuJoCo WASM, Node test runner, Playwright.

## Global Constraints

- Implement only revised Step 1 pre-grasp staging; do not close a gripper or move the cross-member.
- Use actuator controls only during runtime; never write task-object `qpos`.
- Keep all four grippers open at control value `255`.
- Do not add proximity attachment, equality weld, magnet, or scripted object following.
- Keep `Franka Assembly2` unchanged.
- Final TCP position error must be at most `0.03 m`; final TCP orientation error must be at most `8°`.
- Arm 3 and Arm 4 cross-member grasp points must be separated by `0.21–0.27 m`.
- Do not push the implementation without a new explicit Git instruction.

---

### Task 1: Side-Laid Torque Driver Scene Contract

**Files:**
- Modify: `test/franka-assembly-layouts.test.mjs`
- Modify: `src/frankaAssemblyLayouts.js`

**Interfaces:**
- Consumes: `FRANKA_ASSEMBLY1_LAYOUT.xmlPatches`, `sceneObjects()`.
- Produces: Assembly1 `torque_driver` at `pos=".53 -.42 .166" euler="90 0 0"` and named fixed safety edges `torque_driver_cradle_south` / `torque_driver_cradle_north`.

- [x] **Step 1: Add a failing scene test for the side-laid tool**

Add to the Assembly1 tool-detail test:

```js
assert.match(
  xml,
  /<body name="torque_driver" pos="\.53 -\.42 \.166" euler="90 0 0">/,
);
assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
  ({ name }) => name === 'torque_driver_cradle_south',
));
assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
  ({ name }) => name === 'torque_driver_cradle_north',
));
assert.doesNotMatch(
  layoutXml(FRANKA_ASSEMBLY2_LAYOUT),
  /<body name="torque_driver" pos="\.53 -\.42 \.166" euler="90 0 0">/,
);
```

- [x] **Step 2: Run the focused scene test and verify RED**

Run: `node --test test/franka-assembly-layouts.test.mjs`

Expected: FAIL because Assembly1 still uses the upright pose and has no two cradle edges.

- [x] **Step 3: Implement the stable side-laid pose**

Change only `ASSEMBLY1_TOOL_XML` to:

```xml
<body name="torque_driver" pos=".53 -.42 .166" euler="90 0 0">
```

Add two fixed low edges to `sceneObjects()`:

```js
fixedBox('torque_driver_cradle_south', [.20, .008, .012], [.53, -.54, .130], [.17, .18, .19, 1]),
fixedBox('torque_driver_cradle_north', [.20, .008, .012], [.53, -.30, .130], [.17, .18, .19, 1]),
```

The edges remain outside the rotated tool envelope and only prevent large lateral escape.

- [x] **Step 4: Run the focused test and verify GREEN**

Run: `node --test test/franka-assembly-layouts.test.mjs`

Expected: all Assembly layout tests pass and Assembly2 remains unchanged.

---

### Task 2: Grasp Region and TCP Orientation Contract

**Files:**
- Modify: `test/assembly-step1.test.mjs`
- Modify: `src/assemblyStep1.js`
- Modify: `src/assemblyStep1.d.ts`

**Interfaces:**
- Produces: `topDownTcpQuaternion(closingAxisYawDegrees): [number, number, number, number]` and revised `ASSEMBLY1_STEP1_ARMS` entries with `closingAxisYawDegrees`, `tcpQuaternion`, `highWaypoint`, and `finalWaypoint`.
- Quaternion convention: `[x, y, z, w]`; local TCP +Z maps to world -Z; local finger slide axis +Y maps to the requested world closing axis.

- [x] **Step 1: Replace the old waypoint expectations with grasp-ready targets**

Assert this role/target contract:

```js
assert.deepEqual(
  ASSEMBLY1_STEP1_ARMS.map(({ role, highWaypoint, finalWaypoint, closingAxisYawDegrees }) => ({
    role, highWaypoint, finalWaypoint, closingAxisYawDegrees,
  })),
  [
    {
      role: 'south frame rail',
      highWaypoint: [0, -0.23, 0.50],
      finalWaypoint: [0, -0.23, 0.33],
      closingAxisYawDegrees: 90,
    },
    {
      role: 'side-laid torque driver handle',
      highWaypoint: [0.559, -0.421, 0.48],
      finalWaypoint: [0.559, -0.421, 0.28],
      closingAxisYawDegrees: 162,
    },
    {
      role: 'cross member north balance point',
      highWaypoint: [-0.49, 0.56, 0.48],
      finalWaypoint: [-0.49, 0.56, 0.26],
      closingAxisYawDegrees: 0,
    },
    {
      role: 'cross member south balance point',
      highWaypoint: [-0.49, 0.32, 0.48],
      finalWaypoint: [-0.49, 0.32, 0.26],
      closingAxisYawDegrees: 0,
    },
  ],
);
assert.ok(Math.abs(
  ASSEMBLY1_STEP1_ARMS[2].finalWaypoint[1]
  - ASSEMBLY1_STEP1_ARMS[3].finalWaypoint[1],
) >= 0.21);
```

Also add pure quaternion checks:

```js
assert.deepEqual(topDownTcpQuaternion(90), [0, 1, 0, 0]);
assert.deepEqual(
  topDownTcpQuaternion(0).map((value) => Number(value.toFixed(6))),
  [0.707107, 0.707107, 0, 0],
);
```

- [x] **Step 2: Run the focused choreography test and verify RED**

Run: `node --test test/assembly-step1.test.mjs`

Expected: FAIL because the old roles and targets remain and the quaternion helper is absent.

- [x] **Step 3: Implement the pure top-down orientation helper and revised targets**

Use a normalized analytic quaternion derived from the rotation whose columns are local X/Y/Z in world coordinates:

```js
export function topDownTcpQuaternion(closingAxisYawDegrees) {
  const halfTurn = (closingAxisYawDegrees + 90) * Math.PI / 360;
  return [Math.cos(halfTurn), Math.sin(halfTurn), 0, 0]
    .map((value) => Math.abs(value) < 1e-12 ? 0 : value);
}
```

Attach `tcpQuaternion: topDownTcpQuaternion(closingAxisYawDegrees)` to every arm entry and declare both additions in `assemblyStep1.d.ts`.

- [x] **Step 4: Run the focused choreography test and verify GREEN**

Run: `node --test test/assembly-step1.test.mjs`

Expected: all choreography contract tests pass.

---

### Task 3: Offline Position-and-Orientation IK Generation

**Files:**
- Modify: `scripts/solve-assembly-step1-waypoints.mjs`
- Modify: `src/assemblyStep1.js`
- Test: `test/assembly-step1.test.mjs`

**Interfaces:**
- Consumes: each arm's world `tcpQuaternion` and attachment-frame yaw.
- Produces: eight in-limit seven-joint arrays stored as `highJointTargets` and `finalJointTargets`, plus JSON evidence containing `highPositionError`, `finalPositionError`, `highOrientationErrorDegrees`, and `finalOrientationErrorDegrees`.

- [x] **Step 1: Add failing assertions that old joint targets are no longer accepted**

Add a stable solver-version marker to every arm entry:

```js
assert.equal(ASSEMBLY1_STEP1_IK_VERSION, 'grasp-ready-v2');
for (const arm of ASSEMBLY1_STEP1_ARMS) {
  assert.equal(arm.highJointTargets.length, 7);
  assert.equal(arm.finalJointTargets.length, 7);
}
```

Run: `node --test test/assembly-step1.test.mjs`

Expected: FAIL because the new IK version is not defined.

- [x] **Step 2: Transform world TCP orientations into robot-local targets**

In the solver loop construct:

```js
const worldQuaternion = new THREE.Quaternion(...arm.tcpQuaternion);
const baseQuaternion = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(0, 0, 1),
  attachmentFrames[index].yaw,
);
const targetQuaternion = baseQuaternion.clone().invert().multiply(worldQuaternion);
```

Use this target for both high and final IK solves. Compute orientation error from the solved `site_xmat` using the absolute quaternion dot product:

```js
const dot = Math.min(1, Math.abs(actualQuaternion.dot(targetQuaternion)));
const errorDegrees = THREE.MathUtils.radToDeg(2 * Math.acos(dot));
```

Fail the script when either position error exceeds `0.03 m`, orientation error exceeds `8°`, or any target exceeds `model.jnt_range`.

- [x] **Step 3: Generate all eight solutions**

Run:

```bash
node scripts/solve-assembly-step1-waypoints.mjs \
  /data/private/user2/workspace/7.web-robot/1.source-repos/google-deepmind__mujoco_menagerie/franka_emika_panda
```

Expected: JSON contains four arms with `withinLimits: true`, both position errors `<=0.03`, and both orientation errors `<=8`.

- [x] **Step 4: Replace the joint target arrays and mark the solver version**

Copy the exact six-decimal `high` and `final` arrays from the successful output into `src/assemblyStep1.js`, then export:

```js
export const ASSEMBLY1_STEP1_IK_VERSION = 'grasp-ready-v2';
```

- [x] **Step 5: Re-run generation and unit tests**

Run:

```bash
node scripts/solve-assembly-step1-waypoints.mjs > /tmp/assembly1-grasp-ready-waypoints.json
node --test test/assembly-step1.test.mjs
```

Expected: generation exits `0`, the JSON proves all tolerances, and the focused tests pass.

---

### Task 4: Browser Orientation Diagnostics and Real-Scene Acceptance

**Files:**
- Modify: `src/App.tsx`
- Modify: `src/global.d.ts`
- Modify: `scripts/verify-assembly-step1.mjs`
- Create: `artifacts/screenshots/franka-assembly1-step1-grasp-ready.png`

**Interfaces:**
- Produces: `window.robotDemo.getSiteOrientations(names)` returning row-major nine-value site rotation matrices.
- Browser verifier consumes named TCP matrices, body positions, all four gripper controls, and the exact revised target contract.

- [x] **Step 1: Extend the verifier before the browser API exists**

Add target positions and closing-axis expectations:

```js
const tcpTargets = {
  r0_tcp: { position: [0, -0.23, 0.33], closingAxis: [0, 1, 0] },
  r1_tcp: { position: [0.559, -0.421, 0.28], closingAxis: [-0.951057, 0.309017, 0] },
  r2_tcp: { position: [-0.49, 0.56, 0.26], closingAxis: [1, 0, 0] },
  r3_tcp: { position: [-0.49, 0.32, 0.26], closingAxis: [1, 0, 0] },
};
```

For each row-major site matrix, use column 1 (`[matrix[1], matrix[4], matrix[7]]`) as local +Y closing axis and column 2 (`[matrix[2], matrix[5], matrix[8]]`) as local +Z approach axis. Accept axis direction sign symmetry for the closing axis via `Math.abs(dot)`, but require approach axis dot `[0,0,-1]` to meet the `8°` bound.

Run: `node scripts/verify-assembly-step1.mjs`

Expected: FAIL because `getSiteOrientations` is absent or the current pose violates the revised targets.

- [x] **Step 2: Implement the diagnostic API**

Add a collector parallel to `getSitePositions`:

```ts
getSiteOrientations: (names: string[]) => Object.fromEntries(names.map((name) => {
  const id = findSiteByName(model, name);
  if (id < 0) throw new Error(`Could not resolve diagnostic orientation: ${name}`);
  return [name, Array.from(data.site_xmat.slice(id * 9, id * 9 + 9))];
})),
```

Declare the return type as `Record<string, number[]>` in `src/global.d.ts`.

- [x] **Step 3: Verify real browser motion and object stability**

Build and start the production preview with Node 22, then run:

```bash
SCENE_URL=http://127.0.0.1:4174 \
FRANKA_ASSET_DIR=/data/private/user2/workspace/7.web-robot/1.source-repos/google-deepmind__mujoco_menagerie/franka_emika_panda \
node scripts/verify-assembly-step1.mjs
```

Require:

- all four actuator blocks change by at least two visible joint controls;
- all four gripper controls equal `255` at completion;
- position/orientation tolerances pass;
- frame, cross-member, torque driver, screwdriver, hammer, mounting plate, and fasteners drift at most `0.003 m` between the settled pre-click snapshot and completion;
- keyboard `V` still closes and reopens the selected gripper after completion.

- [x] **Step 4: Capture and inspect grasp-ready evidence**

Save the successful browser view as:

```text
artifacts/screenshots/franka-assembly1-step1-grasp-ready.png
```

Inspect that Arm 1 is over the south rail, Arm 2 fingers straddle the side-laid handle, and Arm 3/4 are visibly separated over the cross-member.

---

### Task 5: Regression and Records

**Files:**
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`
- Modify: `project/artifacts.jsonl`
- Modify: `project/artifacts.md`
- Modify: `docs/superpowers/plans/2026-08-13-assembly1-step1-grasp-ready-redesign.md`

**Interfaces:**
- Records the final verified screenshot, exact IK tolerances, test commands, and Step 2 boundary.

- [x] **Step 1: Run complete source verification with Node 22**

Run:

```bash
export PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH
npm test
npx tsc --noEmit
npm run build
git diff --check
```

Expected: every command exits `0`.

- [x] **Step 2: Update tracking files**

Record:

- adopted dual-arm cross-member strategy;
- side-laid torque driver pose;
- final TCP positions and orientation errors;
- browser verification result and screenshot path;
- next action limited to physical descent and gripper closure after visual review.

- [x] **Step 3: Mark this plan's completed checkboxes and inspect repository scope**

Run:

```bash
git status --short
git diff --stat
git diff --check
```

Expected: only the grasp-ready redesign, its generated artifact, and tracking updates are present. Do not push without a new explicit Git instruction.
