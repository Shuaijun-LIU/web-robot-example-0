# Assembly1 Step 2 Physical Clamp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Continue from Assembly1 Step 1 and make all four Panda grippers descend onto their assigned objects, establish verified MuJoCo contact grasps, and hold the physical clamp state without lifting or attaching any object.

**Architecture:** A pure `assemblyStep2.js` module owns the immutable geometry, generated joint targets, phase reducer, interpolation, and contact-verdict rules. `AssemblyStep2Controller.tsx` resolves the live MuJoCo model, validates the Step 1 baseline, atomically takes actuator/gravity-compensation ownership, executes the phase machine, samples physical contacts, and holds the final controls. `AssemblySequencePanel.tsx` and `App.tsx` only own requests and display state; the production verifier reads structured runtime diagnostics rather than inferring success from button text.

**Tech Stack:** React 19, TypeScript 5.7, Vite 7, `mujoco-react`, `mujoco-js`, Three.js, Node.js built-in test runner, Playwright.

## Global Constraints

- Work only in `Franka Assembly1`; `Franka Assembly2` geometry and behavior must remain unchanged.
- Step 2 must not write task-object `qpos`, create equality welds, use magnetic/proximity attachment, or make an object follow a scripted pose.
- All task-object motion must arise from MuJoCo contacts, force-limited actuators, gravity, and gravity compensation applied only to the 28 Panda arm-joint DOFs.
- Step 2 establishes contact and holds; it must not lift, transfer, align, fasten, or automatically start Step 3.
- Contact targets are `r0=(0,-0.23,0.235)`, `r1=(0.559,-0.421,0.160)`, `r2=(-0.49,0.56,0.140)`, and `r3=(-0.49,0.32,0.140)` metres, with approach points exactly `0.015 m` higher.
- Arm 1 closes on world Y around `assembly_frame`; Arm 2 closes at yaw `162°` around `torque_driver`; Arms 3 and 4 close on world X around `cross_member` at points separated by `0.24 m`.
- Arms 3 and 4 must receive identical-frame gripper commands and pass one atomic four-finger verification window.
- A valid grasp requires bilateral target contact for at least `0.25 s`, summed finger aperture above `0.020 m`, translation at most `0.005 m`, rotation at most `5°`, and vertical displacement at most `0.003 m`.
- Failure retains current arm and gripper controls, retains robot-only gravity compensation, reports a structured reason, and requires Reset.
- While Step 2 is planning, running, complete, or failed, manual IK, gizmo drag, keyboard arm control, and keyboard gripper control must not overwrite sequence controls.
- Do not commit or push this shared worktree; the user will request the next unified commit explicitly.

---

## File Structure

- Create `src/assemblyStep2.js`: Step 2 constants, arm contracts, phase reducer, interpolation, pose math, and pure grasp verdict.
- Create `src/assemblyStep2.d.ts`: exact public types and function signatures for the JavaScript domain module.
- Create `src/AssemblyStep2Controller.tsx`: model resolution, preconditions, atomic ownership handoff, trajectories, contact sampling, failure hold, and diagnostics.
- Create `src/AssemblySequencePanel.tsx`: combined Step 1/Step 2 panel with phase-specific Chinese copy and accessible buttons.
- Delete `src/AssemblyStep1Panel.tsx` after `App.tsx` uses the combined panel.
- Modify `src/AssemblyStep1Controller.tsx`: accept the shared ownership ref and expose the exact completion-frame Step 1 baseline.
- Modify `src/App.tsx`: own Step 2 request/state, share the ownership token and baseline, disable manual controllers, expose diagnostics, and coordinate Reset.
- Modify `src/global.d.ts`: type the Step 2 browser diagnostics and run entry point.
- Modify `src/styles.css`: rename the panel selectors and add two-button/phase/error spacing.
- Create `scripts/solve-assembly-step2-waypoints.mjs`: derive approach/contact joint arrays from the current Panda MJCF and verify them.
- Create `scripts/verify-assembly-step2.mjs`: production-browser acceptance with contact, aperture, drift, hold, Reset, and screenshot evidence.
- Create `test/assembly-step2.test.mjs`: pure contract, reducer, simultaneous closure, verdict, threshold, and anti-shortcut tests.
- Modify `test/ui-contract.test.mjs`: combined-panel and browser-diagnostics source contract.
- Modify `package.json`: add `solve:assembly-step2` and `verify:assembly-step2` scripts.
- Modify `project/task_plan.md`, `project/decision_log.md`, `project/standup_log.md`, and `project/next_actions.md`: record the implementation result and measured production tolerances without touching concurrent scene work.

### Task 1: Pure Step 2 Contract, Phase Reducer, and Contact Verdict

**Files:**
- Create: `src/assemblyStep2.js`
- Create: `src/assemblyStep2.d.ts`
- Create: `test/assembly-step2.test.mjs`

**Interfaces:**
- Consumes: `smoothstep01(value: number)` and `interpolateJointTargets(from, to, progress)` from `src/assemblyStep1.js`.
- Produces: `ASSEMBLY1_STEP2_ARMS`, `ASSEMBLY1_STEP2_DURATIONS`, `ASSEMBLY1_STEP2_LIMITS`, `createAssemblyStep2Machine()`, `advanceAssemblyStep2Machine(machine, dt, evidence)`, `interpolateAssemblyStep2Gripper(from, to, progress)`, `quaternionAngularDistanceDegrees(a, b)`, and `evaluateAssemblyStep2Grasp(input)`.

- [ ] **Step 1: Write the failing geometry and timing test**

```js
import assert from 'node:assert/strict';
import test from 'node:test';
import {
  ASSEMBLY1_STEP2_ARMS,
  ASSEMBLY1_STEP2_DURATIONS,
  ASSEMBLY1_STEP2_LIMITS,
} from '../src/assemblyStep2.js';

test('Step 2 assigns exact physical contact geometry to all four arms', () => {
  assert.deepEqual(ASSEMBLY1_STEP2_ARMS.map((arm) => ({
    key: arm.key,
    targetBody: arm.targetBody,
    contactWaypoint: arm.contactWaypoint,
    approachWaypoint: arm.approachWaypoint,
    closingAxisYawDegrees: arm.closingAxisYawDegrees,
    leftFingerBody: arm.leftFingerBody,
    rightFingerBody: arm.rightFingerBody,
  })), [
    { key: 'r0', targetBody: 'assembly_frame', contactWaypoint: [0, -.23, .235], approachWaypoint: [0, -.23, .25], closingAxisYawDegrees: 90, leftFingerBody: 'r0_left_finger', rightFingerBody: 'r0_right_finger' },
    { key: 'r1', targetBody: 'torque_driver', contactWaypoint: [.559, -.421, .16], approachWaypoint: [.559, -.421, .175], closingAxisYawDegrees: 162, leftFingerBody: 'r1_left_finger', rightFingerBody: 'r1_right_finger' },
    { key: 'r2', targetBody: 'cross_member', contactWaypoint: [-.49, .56, .14], approachWaypoint: [-.49, .56, .155], closingAxisYawDegrees: 0, leftFingerBody: 'r2_left_finger', rightFingerBody: 'r2_right_finger' },
    { key: 'r3', targetBody: 'cross_member', contactWaypoint: [-.49, .32, .14], approachWaypoint: [-.49, .32, .155], closingAxisYawDegrees: 0, leftFingerBody: 'r3_left_finger', rightFingerBody: 'r3_right_finger' },
  ]);
  assert.equal(Math.abs(ASSEMBLY1_STEP2_ARMS[2].contactWaypoint[1] - ASSEMBLY1_STEP2_ARMS[3].contactWaypoint[1]), .24);
  assert.deepEqual(ASSEMBLY1_STEP2_DURATIONS, {
    approach: 1.4,
    slowDescent: .8,
    frameClamp: .8,
    crossMemberClamp: 1,
    torqueDriverClamp: .8,
    contactWindow: .25,
    verificationTimeout: 1.5,
    stableHold: 2,
  });
  assert.deepEqual(ASSEMBLY1_STEP2_LIMITS, {
    tcpPosition: .03,
    tcpOrientationDegrees: 8,
    preStepObjectDrift: .003,
    objectTranslation: .005,
    objectRotationDegrees: 5,
    verticalDisplacement: .003,
    minimumAperture: .02,
  });
});
```

- [ ] **Step 2: Run the test and confirm the module is missing**

Run: `node --test test/assembly-step2.test.mjs`

Expected: FAIL with `ERR_MODULE_NOT_FOUND` for `src/assemblyStep2.js`.

- [ ] **Step 3: Implement the immutable arm contracts and declarations**

Create four frozen arm entries. Derive `siteName`, seven `jointNames`, two `fingerJointNames`, seven `actuatorIndices`, and `gripperActuatorIndex` using the existing eight-actuator-per-Panda layout. Reuse `topDownTcpQuaternion()` so the runtime and offline solver share orientation semantics. Include these exact TypeScript shapes:

```ts
export type AssemblyStep2Phase =
  | 'idle' | 'planning' | 'approach' | 'slow-descent'
  | 'frame-clamp' | 'frame-verification'
  | 'cross-member-clamp' | 'cross-member-verification'
  | 'torque-driver-clamp' | 'tool-verification'
  | 'clamped-hold' | 'complete' | 'error';

export interface AssemblyStep2Evidence {
  frame?: AssemblyStep2GraspVerdict;
  crossMember?: AssemblyStep2GraspVerdict;
  tool?: AssemblyStep2GraspVerdict;
  all?: AssemblyStep2GraspVerdict;
}

export interface AssemblyStep2Machine {
  phase: AssemblyStep2Phase;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep2Failure | null;
}
```

- [ ] **Step 4: Write failing reducer, interpolation, quaternion, and verdict tests**

Cover these exact behaviors:

```js
test('cross-member grippers close with one shared interpolation value', () => {
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 0), 255);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, .5), 127.5);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 1), 0);
});

test('verification requires a continuous quarter-second window', () => {
  let state = { phase: 'frame-verification', phaseElapsed: 0, continuousValidSeconds: 0, failure: null };
  state = advanceAssemblyStep2Machine(state, .15, { frame: { ok: true } });
  state = advanceAssemblyStep2Machine(state, .05, { frame: { ok: false, code: 'missing-left-contact' } });
  assert.equal(state.continuousValidSeconds, 0);
  state = advanceAssemblyStep2Machine(state, .25, { frame: { ok: true } });
  assert.equal(state.phase, 'cross-member-clamp');
});

test('grasp verdict rejects empty closure, wrong contact, drift, rotation, and lift', () => {
  const valid = {
    targetBody: 'assembly_frame',
    leftContactBodies: ['assembly_frame'],
    rightContactBodies: ['assembly_frame'],
    forbiddenBodies: [],
    aperture: .03,
    translation: .001,
    rotationDegrees: 1,
    verticalDisplacement: .001,
  };
  assert.deepEqual(evaluateAssemblyStep2Grasp(valid), { ok: true });
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, aperture: .02 }).code, 'empty-closure');
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, leftContactBodies: [] }).code, 'missing-left-contact');
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, forbiddenBodies: ['work_platform'] }).code, 'forbidden-contact');
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, translation: .0051 }).code, 'object-drift');
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, rotationDegrees: 5.1 }).code, 'object-rotation');
  assert.equal(evaluateAssemblyStep2Grasp({ ...valid, verticalDisplacement: .0031 }).code, 'object-lift');
});
```

- [ ] **Step 5: Implement the minimal pure reducer and verdict logic**

Timed motion phases advance only after their exact durations. Verification phases reset `continuousValidSeconds` on a failed sample, advance after `0.25 s` of uninterrupted valid evidence, and enter `error` with the latest verdict after `1.5 s`. `clamped-hold` continuously validates `evidence.all` for `2 s` before `complete`. The complete and error states are terminal until Reset.

- [ ] **Step 6: Run the focused unit test**

Run: `node --test test/assembly-step2.test.mjs`

Expected: PASS for geometry, timing, interpolation, continuous contact, threshold boundaries, quaternion sign symmetry, and structured failure codes.

### Task 2: Generate and Validate the Eight IK Waypoints

**Files:**
- Create: `scripts/solve-assembly-step2-waypoints.mjs`
- Modify: `src/assemblyStep2.js`
- Modify: `src/assemblyStep2.d.ts`
- Modify: `test/assembly-step2.test.mjs`
- Modify: `package.json`

**Interfaces:**
- Consumes: `ASSEMBLY1_STEP1_ARMS[*].finalJointTargets`, Step 2 world approach/contact points and quaternions, `solveSelectedIk()`, and `fitJointAngleToRange()`.
- Produces: finite seven-value `approachJointTargets` and `contactJointTargets` for every `ASSEMBLY1_STEP2_ARMS` entry.

- [ ] **Step 1: Extend the test to require generated joint arrays**

```js
test('every Step 2 waypoint contains a complete generated Panda solution', () => {
  for (const arm of ASSEMBLY1_STEP2_ARMS) {
    assert.equal(arm.approachJointTargets.length, 7);
    assert.equal(arm.contactJointTargets.length, 7);
    assert.ok(arm.approachJointTargets.every(Number.isFinite));
    assert.ok(arm.contactJointTargets.every(Number.isFinite));
  }
});
```

- [ ] **Step 2: Run the test and confirm missing target arrays fail**

Run: `node --test test/assembly-step2.test.mjs`

Expected: FAIL because `approachJointTargets` and `contactJointTargets` do not yet exist.

- [ ] **Step 3: Implement the solver by adapting the proven Step 1 solver**

The script must load the same Menagerie Panda, inject the same `tcp` site, use the same four attachment frames, transform each world target into the corresponding robot frame, and seed Arm N with `ASSEMBLY1_STEP1_ARMS[N].finalJointTargets`. Solve approach first and contact second, fit every angle into `jnt_range`, then forward-evaluate each solution.

For every arm, throw unless:

```js
approachError.position <= .01
&& contactError.position <= .01
&& approachError.orientationDegrees <= 5
&& contactError.orientationDegrees <= 5
&& [...boundedApproach, ...boundedContact].every(withinJointRange)
```

Print deterministic JSON containing `key`, the two targets, the two rounded six-decimal joint arrays, four measured errors, and `withinLimits`.

- [ ] **Step 4: Add and run the package solver command**

Add:

```json
"solve:assembly-step2": "node scripts/solve-assembly-step2-waypoints.mjs"
```

Run: `npm run solve:assembly-step2`

Expected: four results, eight valid solutions, all `withinLimits: true`, position errors at most `0.01 m`, orientation errors at most `5°`.

- [ ] **Step 5: Copy the generated arrays into the immutable arm contract**

Paste the solver's rounded arrays as `approachJointTargets` and `contactJointTargets`; do not manually tune a target without rerunning the solver and preserving its measured error output.

- [ ] **Step 6: Re-run the solver and focused tests against the checked-in constants**

Run: `npm run solve:assembly-step2 && node --test test/assembly-step2.test.mjs`

Expected: both commands PASS and the script output matches the arrays in `ASSEMBLY1_STEP2_ARMS` to six decimals.

### Task 3: Runtime Preconditions and Atomic Ownership Handoff

**Files:**
- Modify: `src/AssemblyStep1Controller.tsx`
- Create: `src/AssemblyStep2Controller.tsx`
- Modify: `src/App.tsx`
- Modify: `src/global.d.ts`
- Modify: `test/assembly-step2.test.mjs`

**Interfaces:**
- Consumes: shared `MutableRefObject<'manual' | 'step1' | 'step2'>`, Step 1 completion state, live MuJoCo model/data, and Step 1 final targets.
- Produces: `AssemblyStep1CompletionSnapshot`, `AssemblyStep2RuntimeDiagnostics`, and a Step 2 controller that takes ownership only after every precondition and name resolution succeeds.

- [ ] **Step 1: Write a source-contract test for ownership and forbidden writes**

Read `AssemblyStep1Controller.tsx` and `AssemblyStep2Controller.tsx` as text and assert that both consume `ownershipRef`, that Step 1 compensation is guarded by `ownershipRef.current === 'step1'`, and Step 2 assigns `ownershipRef.current = 'step2'` only after plan validation. Reject these patterns from the Step 2 source:

```js
/data\.qpos\s*\[[^\]]+\]\s*=/
/mjEQ_WELD|equality|magnet|attach|proximity/i
/taskObject.*follow|scripted.*pose/i
```

Allow `data.qpos[address]` reads for finger aperture.

- [ ] **Step 2: Run the source-contract test and confirm failure**

Run: `node --test test/assembly-step2.test.mjs`

Expected: FAIL because the runtime controller and ownership handoff are not implemented.

- [ ] **Step 3: Expose the exact Step 1 completion-frame baseline**

Change `AssemblyStep1Controller.onMotionComplete` to receive `(model, data)`. In `SceneChildren`, capture:

```ts
interface AssemblyStep1CompletionSnapshot {
  tcpPositions: Record<string, [number, number, number]>;
  tcpOrientations: Record<string, number[]>;
  objectPoses: Record<string, { position: [number, number, number]; quaternion: [number, number, number, number] }>;
  gripperControls: [number, number, number, number];
}
```

Capture `assembly_frame`, `torque_driver`, and `cross_member` from `data.xpos/xquat`, all four TCPs from `data.site_xpos/site_xmat`, and controls `[7, 15, 23, 31]` before reporting Step 1 complete. Clear this ref on Reset and scene change.

- [ ] **Step 4: Add the shared ownership token**

Create `const assemblyOwnershipRef = useRef<'manual' | 'step1' | 'step2'>('manual')` in `SceneChildren`. Step 1 sets it to `step1` only after its complete four-arm plan exists and retains it through Step 1 complete. Step 1's physics callback applies compensation and holds its arm controls only while it owns the token. Reset returns it to `manual`.

- [ ] **Step 5: Implement Step 2 model resolution and precondition validation**

Resolve, without mutation:

- four `rN_tcp` sites;
- 28 `rN_joint1..7` joints and DOF addresses;
- eight `rN_finger_joint1/2` joints and qpos addresses;
- eight `rN_left_finger/right_finger` body IDs;
- three assigned target body IDs;
- forbidden task/support body IDs that exist in the scene;
- actuator indices `0..31` according to each arm contract.

Reject planning with a structured error unless the snapshot exists, every current TCP is within `0.03 m` and `8°` of its Step 1 target, every gripper control equals `255` within `1e-6`, every task object remains within `0.003 m` of the Step 1 completion snapshot, and all generated joint arrays are finite and inside model limits.

- [ ] **Step 6: Perform the atomic ownership transfer**

After all resolution and validation succeeds, initialize the Step 2 runtime refs first, then assign `ownershipRef.current = 'step2'`, then report phase `approach`. Both physics callbacks consult the same token, so the frame after assignment receives exactly one gravity-compensation contribution. A planning failure leaves the token at `step1`; an execution failure leaves it at `step2`.

- [ ] **Step 7: Run focused tests and TypeScript**

Run: `node --test test/assembly-step2.test.mjs && npx tsc --noEmit`

Expected: PASS with no task-object pose assignment and no ownership type mismatch.

### Task 4: Execute Physical Descent, Closure, Contact Verification, and Failure Hold

**Files:**
- Modify: `src/AssemblyStep2Controller.tsx`
- Modify: `src/assemblyStep2.js`
- Modify: `src/assemblyStep2.d.ts`
- Modify: `test/assembly-step2.test.mjs`

**Interfaces:**
- Consumes: validated runtime plan, pure phase reducer, generated joint targets, model body/geom/joint addresses, and shared ownership token.
- Produces: four physical contact grasps, terminal hold/error behavior, and per-frame `AssemblyStep2RuntimeDiagnostics`.

- [ ] **Step 1: Add tests for exact actuator choreography**

Test a pure `step2ControlFrame(machine, plans)` helper or equivalent exported function and assert:

- approach interpolates Step 1 final joints to `approachJointTargets` over `1.4 s`, all grippers `255`;
- slow descent interpolates approach to contact over `0.8 s`, all grippers `255`;
- frame clamp changes only actuator `7` from `255` to `0` over `0.8 s`;
- cross-member clamp produces exactly equal values for actuator `23` and actuator `31` on every progress sample;
- torque-driver clamp changes only actuator `15` over `0.8 s`;
- complete/error preserve the latest 28 joint controls and four gripper commands.

- [ ] **Step 2: Run the choreography test and confirm failure**

Run: `node --test test/assembly-step2.test.mjs`

Expected: FAIL until the exported control-frame helper exists.

- [ ] **Step 3: Implement physics-frame trajectory writes**

Within `useBeforePhysicsStep`, return immediately unless `ownershipRef.current === 'step2'`. Add `qfrc_bias` only at the resolved 28 arm DOF addresses. Write only the 28 arm actuator controls and four gripper actuator controls. Use the phase reducer's smoothstep progress and keep all prior grasps closed while later phases run.

- [ ] **Step 4: Implement live contact and pose evidence**

For each active `data.contact.get(i)`, map `geom1/geom2` through `model.geom_bodyid`. When one side is a resolved left/right finger body, record the opposite body. Build the phase evidence using:

```ts
evaluateAssemblyStep2Grasp({
  targetBody,
  leftContactBodies,
  rightContactBodies,
  forbiddenBodies,
  aperture: data.qpos[leftFingerQposAdr] + data.qpos[rightFingerQposAdr],
  translation: distance(currentPosition, preStep2Position),
  rotationDegrees: quaternionAngularDistanceDegrees(currentQuaternion, preStep2Quaternion),
  verticalDisplacement: Math.abs(currentPosition[2] - preStep2Position[2]),
})
```

For Arms 3/4, `crossMember.ok` is true only if both individual verdicts are true in the same physics frame. The hold evidence is true only when all four arm verdicts are simultaneously true.

- [ ] **Step 5: Implement failure hold and structured diagnostics**

On verification timeout, forbidden contact, aperture failure, excessive movement/rotation/lift, or a non-finite runtime value, transition to `error`, freeze the last controls, keep ownership and gravity compensation, and stop advancing. Diagnostics must include phase, phase elapsed time, failure code/arm/detail, each finger's opposite-body names, apertures, object deltas, continuous valid seconds, gripper controls, and simulation time.

- [ ] **Step 6: Hold success without auto-starting Step 3**

After all contacts remain valid through the `2 s` clamped-hold acceptance window, report `complete` but keep the Step 2 runtime plan active indefinitely. Do not fire any lift or placement callback. Reset is the only release path in this task.

- [ ] **Step 7: Run pure tests and TypeScript**

Run: `node --test test/assembly-step2.test.mjs && npx tsc --noEmit`

Expected: PASS for simultaneous Arm 3/4 controls, evidence thresholds, terminal hold, and type safety.

### Task 5: Integrate the Two-Step Panel, Manual-Control Lockout, Reset, and Diagnostics

**Files:**
- Create: `src/AssemblySequencePanel.tsx`
- Delete: `src/AssemblyStep1Panel.tsx`
- Modify: `src/App.tsx`
- Modify: `src/global.d.ts`
- Modify: `src/styles.css`
- Modify: `test/ui-contract.test.mjs`

**Interfaces:**
- Consumes: `AssemblyStep1Status`, `AssemblyStep2State`, Step 1/2 request handlers, runtime diagnostics ref.
- Produces: a single accessible upper-left action panel and `window.robotDemo.runAssemblyStep2/getAssemblyStep2Diagnostics`.

- [ ] **Step 1: Write failing UI source-contract tests**

Assert that `App.tsx` imports `AssemblySequencePanel`, exposes `dataset.assemblyStep2Status`, exposes both browser functions, and defines automation as:

```ts
const assemblyAutomationActive =
  assemblyStep1Status === 'planning'
  || assemblyStep1Status === 'running'
  || !['idle'].includes(assemblyStep2State.phase);
```

Assert the panel contains `执行第二步：下降并物理夹持`, `第一步已完成`, `第二步已完成：四处物理夹持已建立`, and `请 Reset 后重试`.

- [ ] **Step 2: Run UI tests and confirm failure**

Run: `node --test test/ui-contract.test.mjs`

Expected: FAIL because only the Step-1-only panel exists.

- [ ] **Step 3: Build the combined sequence panel**

Render Step 1 status/button first and Step 2 status/button second. Disable Step 2 unless scene is Assembly1, Step 1 is complete, Step 2 is idle, and the completion snapshot exists. Use phase-specific Chinese text for approach, slow descent, frame clamp/verification, cross-member clamp/verification, torque-driver clamp/verification, and clamped hold. Render structured failures with the arm and failure code followed by `请 Reset 后重试`.

- [ ] **Step 4: Integrate state and manual-control lockout in App**

Add Step 2 request ID/state, guarded run callback, DOM dataset, controller props, and diagnostics ref. Pass `enabled={!assemblyAutomationActive}` to `FrankaController`; show the IK gizmo only when `!assemblyAutomationActive`; pass the same disable flag to `useSelectedIkController`; and prevent drag/keyboard gripper overwrites throughout Step 2 complete/error hold.

- [ ] **Step 5: Make Reset a complete sequence reset**

Both the Leva Reset and `window.robotDemo.reset()` must call the same callback that invokes `simulation.api.reset()`, sets both step states to idle, clears the completion snapshot and diagnostics, returns ownership to manual, disables/synchronizes IK, and increments `resetGeneration`.

- [ ] **Step 6: Expose deterministic browser diagnostics**

Extend `Window.robotDemo` with:

```ts
runAssemblyStep2(): boolean;
getAssemblyStep2Diagnostics(): AssemblyStep2RuntimeDiagnostics | null;
getBodyOrientations(names: string[]): Record<string, [number, number, number, number]>;
getJointPositions(names: string[]): Record<string, number>;
getContacts(): Array<{ geom1: number; geom2: number; body1: string; body2: string }>;
```

- [ ] **Step 7: Rename and extend panel styling**

Replace `.assembly-step1-panel*` selectors with `.assembly-sequence-panel*`, retain the current muted palette, add `display:grid; gap:.55rem`, provide space between step groups, and distinguish errors without introducing saturated cyan.

- [ ] **Step 8: Run focused UI tests and build**

Run: `node --test test/ui-contract.test.mjs && npx tsc --noEmit && npm run build`

Expected: PASS; Vite emits the production bundle and the combined panel has no missing imports.

### Task 6: Production-Browser Physical Acceptance and Evidence

**Files:**
- Create: `scripts/verify-assembly-step2.mjs`
- Modify: `package.json`
- Create: `artifacts/screenshots/franka-assembly1-step2-physical-clamp.png`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`

**Interfaces:**
- Consumes: production Vite scene, combined panel, DOM statuses, generic diagnostics, and Step 2 runtime diagnostics.
- Produces: pass/fail JSON with exact measured tolerances and a visual screenshot of the four-arm clamped state.

- [ ] **Step 1: Implement the Playwright verifier**

Follow the existing local Menagerie asset routing and failure capture pattern. Verify in this order:

1. Assembly1 is ready and Step 2 is disabled before Step 1.
2. Run Step 1 and wait for exact `complete`; Step 2 becomes enabled.
3. Capture task-object poses and confirm controls `[7,15,23,31]` are `255`.
4. Run Step 2 and poll structured diagnostics, failing immediately on `error`.
5. Assert each arm visibly changed at least two joint controls during descent.
6. Assert gripper controls end at `0`, Arm 3/4 closure start times differ by no more than one simulation timestep, and all required bilateral contacts lasted at least `0.25 s`.
7. Assert each aperture is above `0.020 m`, object translation at most `0.005 m`, rotation at most `5°`, and vertical displacement at most `0.003 m`.
8. Wait another `2 s`; assert phase remains complete, contacts remain bilateral, and object poses remain inside thresholds.
9. Save `artifacts/screenshots/franka-assembly1-step2-physical-clamp.png`.
10. Invoke Reset and assert both statuses return to idle, all gripper controls return to the scene's open state, and Step 2 is disabled again.

- [ ] **Step 2: Add the production verification command**

Add:

```json
"verify:assembly-step2": "node scripts/verify-assembly-step2.mjs"
```

- [ ] **Step 3: Run the focused suite and production server**

Terminal A:

```bash
npm run dev -- --host 127.0.0.1
```

Terminal B:

```bash
FRANKA_ASSET_DIR=../google-deepmind__mujoco_menagerie/franka_emika_panda npm run verify:assembly-step2
```

Expected: verifier prints `status: PASS`, the exact four apertures/contact windows/pose deltas, and the screenshot path.

- [ ] **Step 4: Run the complete regression gate**

Run:

```bash
npm test
npx tsc --noEmit
npm run build
npm run solve:assembly-step1
npm run solve:assembly-step2
```

Expected: all commands PASS. If a concurrent agent has introduced an unrelated failing test, record the exact unrelated file and still require all Step 1, Step 2, scene-layout, UI, TypeScript, build, solver, and browser checks owned by this work to pass.

- [ ] **Step 5: Inspect the screenshot at original resolution**

Open `artifacts/screenshots/franka-assembly1-step2-physical-clamp.png` and verify visually that all four TCPs are down at the assigned grasp sections, fingers surround rather than penetrate the objects, the cross-member remains supported and level, and no object has lifted.

- [ ] **Step 6: Record measured completion state**

Update only the Step 2 sections of project tracking files with the executed commands, maximum measured TCP/contact/aperture/drift/rotation/lift values, screenshot path, no-shortcut confirmation, and Step 3's boundary. Leave other agents' scene progress intact. Do not commit or push.

---

## Self-Review Result

- Spec coverage: every geometry, timing, precondition, ownership, physical-contact, threshold, UI, failure, Reset, production-browser, and Step 3 boundary requirement maps to Tasks 1–6.
- Placeholder scan: no `TBD`, `TODO`, “implement later”, generic error-handling step, or undefined “similar to” instruction remains.
- Type consistency: `AssemblyStep2Phase`, `AssemblyStep2Machine`, `AssemblyStep2Evidence`, `AssemblyStep2RuntimeDiagnostics`, ownership token values, browser method names, and generated joint-target property names are consistent across producers and consumers.
- Shared-worktree safety: the plan explicitly avoids commits and limits tracking-file edits to Step 2 sections.
