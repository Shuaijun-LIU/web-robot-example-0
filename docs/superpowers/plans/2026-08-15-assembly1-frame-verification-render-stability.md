# Assembly1 Frame Verification Render Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the Franka Assembly1 workcell rendered during Step 2 frame-contact verification without changing motion, gripper, contact-verdict, or scene behavior.

**Architecture:** Capture all four arms' measured robot-joint positions once at the transition into `frame-verification`. During only that phase, substitute the immutable snapshot for the generated robot-joint targets while preserving the existing gripper targets and state machine; clear it on every exit path.

**Tech Stack:** React 19, TypeScript, MuJoCo via `mujoco-react`, Node test runner, Vite 7, Playwright.

## Global Constraints

- Do not write `data.qpos` or `data.qvel`.
- Do not modify task-object poses, weld constraints, proximity attachments, scene geometry, waypoints, gripper targets, contact thresholds, or phase durations.
- Limit runtime changes to Franka Assembly1 Step 2 `frame-verification`.
- Use Node 22.12.0 for Vite builds.

---

### Task 1: Immutable measured-joint snapshot

**Files:**
- Modify: `src/assemblyStep2.js`
- Modify: `src/assemblyStep2.d.ts`
- Test: `test/assembly-step2.test.mjs`

**Interfaces:**
- Consumes: a `Float64Array` of generalized positions and arms containing `qposAddresses`.
- Produces: `captureAssemblyStep2JointTargets(positions, arms): number[][]`, with one copied seven-joint target array per arm.

- [ ] **Step 1: Write the failing snapshot test**

Add the import and test:

```js
import { captureAssemblyStep2JointTargets } from '../src/assemblyStep2.js';

test('frame verification captures an immutable measured joint target per arm', () => {
  const positions = new Float64Array([0.1, 0.2, 0.3, 9, 1.1, 1.2, 1.3]);
  const result = captureAssemblyStep2JointTargets(positions, [
    { qposAddresses: [0, 1, 2] },
    { qposAddresses: [4, 5, 6] },
  ]);
  positions[0] = 8;
  assert.deepEqual(result, [[0.1, 0.2, 0.3], [1.1, 1.2, 1.3]]);
});
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
node --test test/assembly-step2.test.mjs
```

Expected: failure because `captureAssemblyStep2JointTargets` is not exported.

- [ ] **Step 3: Implement the minimal pure helper**

Add to `src/assemblyStep2.js`:

```js
export function captureAssemblyStep2JointTargets(positions, arms) {
  return arms.map((arm) => arm.qposAddresses.map((address) => positions[address]));
}
```

Add the matching declaration to `src/assemblyStep2.d.ts`:

```ts
export function captureAssemblyStep2JointTargets(
  positions: Float64Array,
  arms: Array<{ qposAddresses: number[] }>,
): number[][];
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run `node --test test/assembly-step2.test.mjs`.

Expected: all focused tests pass.

- [ ] **Step 5: Commit the pure helper**

```bash
git add src/assemblyStep2.js src/assemblyStep2.d.ts test/assembly-step2.test.mjs
git commit -m "test(assembly): define fixed joint snapshot"
```

### Task 2: Apply the snapshot only during frame verification

**Files:**
- Modify: `src/AssemblyStep2Controller.tsx`
- Test: `test/assembly-step2.test.mjs`

**Interfaces:**
- Consumes: `captureAssemblyStep2JointTargets(data.qpos, runtime.arms)` from Task 1.
- Produces: a controller-owned `number[][] | null` snapshot used only when `nextMachine.phase === 'frame-verification'`.

- [ ] **Step 1: Write the failing controller contract test**

Extend the existing controller source contract:

```js
assert.match(step2Source, /frameVerificationJointTargetsRef/);
assert.match(step2Source, /captureAssemblyStep2JointTargets\(data\.qpos, runtime\.arms\)/);
assert.match(step2Source, /frameVerificationJointTargetsRef\.current\?\.\[index\]/);
assert.doesNotMatch(step2Source, /data\.qvel\s*\[[^\]]+\]\s*=/);
```

- [ ] **Step 2: Run the focused test and verify RED**

Run `node --test test/assembly-step2.test.mjs`.

Expected: failure because the controller does not yet own or apply the snapshot.

- [ ] **Step 3: Add the controller-owned phase snapshot**

Import `captureAssemblyStep2JointTargets`, add:

```ts
const frameVerificationJointTargetsRef = useRef<number[][] | null>(null);
```

Clear it in the reset effect and terminal error path. Immediately after advancing the machine, capture or clear it with:

```ts
if (machine.phase !== 'frame-verification' && nextMachine.phase === 'frame-verification') {
  frameVerificationJointTargetsRef.current = captureAssemblyStep2JointTargets(
    data.qpos,
    runtime.arms,
  );
} else if (nextMachine.phase !== 'frame-verification') {
  frameVerificationJointTargetsRef.current = null;
}
```

When writing the seven joint actuator controls, select targets without changing the gripper target:

```ts
const jointTargets = frameVerificationJointTargetsRef.current?.[index]
  ?? controls.jointTargets;
for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
  data.ctrl[arm.actuatorIndices[joint]] = jointTargets[joint];
}
data.ctrl[arm.gripperActuatorIndex] = controls.gripperTarget;
```

- [ ] **Step 4: Run focused and full tests**

Run:

```bash
node --test test/assembly-step2.test.mjs
npm test
```

Expected: focused tests and the full suite pass with zero failures.

- [ ] **Step 5: Commit controller integration**

```bash
git add src/AssemblyStep2Controller.tsx test/assembly-step2.test.mjs
git commit -m "fix(assembly): hold measured frame pose"
```

### Task 3: Production and browser verification

**Files:**
- No repository files modified.
- Build output: `/tmp/web-robot-frame-hold-build`
- Browser evidence: `/tmp/web-robot-frame-hold-check/`

**Interfaces:**
- Consumes: the completed source tree and local Franka assets.
- Produces: a Pages-compatible production build, phase diagnostics, console-error report, and screenshot evidence.

- [ ] **Step 1: Build with the cached Node 22.12.0 binary**

Run:

```bash
/home/shuaijun/.npm/_npx/f6c81a5e22bed22a/node_modules/node/bin/node \
  node_modules/vite/bin/vite.js build \
  --base=/web-robot-example-0/ \
  --outDir /tmp/web-robot-frame-hold-build \
  --emptyOutDir
```

Expected: Vite exits 0 and writes `index.html` plus hashed CSS/JS assets.

- [ ] **Step 2: Serve the build under the Pages base path**

Create `/tmp/web-robot-frame-hold-serve/web-robot-example-0` as a symlink to the build and serve the parent on `127.0.0.1:4175`.

- [ ] **Step 3: Run the real Step 1/Step 2 browser sequence**

Use Playwright with local routing for:

```text
https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/
```

The browser driver must record every `assemblyStep2Status`, wait at least three wall-clock seconds after observing `frame-verification`, and assert:

```text
document.documentElement.dataset.sceneStatus === "ready"
document.querySelectorAll("canvas").length > 0
window.robotDemo is present
no pageerror
no console error matching Simulation error|XML Error|Aborted(
```

Save a screenshot after the observation window under `/tmp/web-robot-frame-hold-check/`.

- [ ] **Step 4: Inspect the screenshot and repository status**

Confirm the screenshot contains the workcell, all four arms, table, frame, and overlays. Run `git diff --check` and `git status --short`; only the pre-existing untracked `artifacts/diagnostics/` may remain.
