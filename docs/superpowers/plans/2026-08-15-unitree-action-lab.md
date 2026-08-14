# Unitree Action Lab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ninth browser scene in which dynamic Unitree G1 and Go2 + Airbot models execute a deterministic 10-second actuator-only action sequence under MuJoCo gravity and contact.

**Architecture:** Vendor the two licensed dynamic MJCF packages into one local asset root and attach them with prefixes to a minimal shared floor scene. Keep the choreography in a pure JavaScript sampler; a small React physics-step adapter resolves named actuators and writes only `data.ctrl`, while a dedicated panel owns play/pause/restart state. Offline MuJoCo and production-browser runners verify the same sampler against the compiled model.

**Tech Stack:** MuJoCo 3.3.8 / `mujoco-js`, React 19, TypeScript, `mujoco-react`, Node test runner, Playwright, Vite, ffmpeg.

## Global Constraints

- Preserve all eight existing scenes, the Assembly1 page default, and their controls.
- The new scene contains only G1, Go2 + Airbot, and a collision-enabled floor.
- Both robot roots are free joints; no planar slides, mocap roots, equality welds, invisible supports, scripted transforms, or runtime root-state writes are allowed.
- Runtime action code writes only named actuator targets in `data.ctrl`; it never writes `qpos`, `qvel`, applied forces, contacts, or body transforms.
- Use no ONNX policy. The action duration is exactly 10 seconds with phase durations `1.0`, `1.5`, `3.0`, `1.5`, `1.5`, and `1.5` seconds.
- Vendor BSD-3-Clause G1 attribution and Apache-2.0 Go2 + Airbot attribution with the copied assets.
- At completion, require G1 pelvis height `0.75–0.85 m`, G1 tilt `<=5°`, Go2 base height `0.22–0.34 m`, Go2 tilt `<=10°`, finite state, and ground contacts.
- Leave the unrelated untracked `artifacts/diagnostics/` directory untouched.

---

### Task 1: Dynamic asset bundle and scene contract

**Files:**
- Create: `public/assets/unitree-action-lab/scene.xml`
- Create: `public/assets/unitree-action-lab/robots/g1/g1.xml`
- Create: `public/assets/unitree-action-lab/robots/g1/assets/*`
- Create: `public/assets/unitree-action-lab/robots/g1/LICENSE`
- Create: `public/assets/unitree-action-lab/robots/go2_arm/go2_with_arm.xml`
- Create: `public/assets/unitree-action-lab/robots/go2_arm/assets/*`
- Create: `public/assets/unitree-action-lab/robots/go2_arm/LICENSE`
- Create: `public/assets/unitree-action-lab/robots/go2/assets/*`
- Create: `src/unitreeActionLab.js`
- Create: `src/unitreeActionLab.d.ts`
- Modify: `scripts/validate-mjcf.mjs`
- Create: `test/unitree-action-lab-scene.test.mjs`

**Interfaces:**
- Produces: `UNITREE_ACTION_LAB_LAYOUT` with `homeJoints`, `camera`, `orbitTarget`, `instanceCount`, and `dynamicRoots`.
- Produces: compiled actuator names prefixed `g1_` and `go2_`, including explicitly named `go2_joint1` through `go2_joint6` arm actuators.
- Consumes: Menagerie G1 asset package and UniLab Go2 + Airbot asset package listed in the design spec.

- [x] **Step 1: Write the failing compiled-scene test.** The test imports `UNITREE_ACTION_LAB_LAYOUT`, invokes the validator, and independently parses its report:

```js
test('Unitree Action Lab compiles two floating articulated robots', () => {
  const report = execFileSync(process.execPath, [
    'scripts/validate-mjcf.mjs',
    'unitreeActionLab',
    'public/assets/unitree-action-lab',
  ], { cwd: repoRoot, encoding: 'utf8' });
  assert.match(report, /unitreeActionLab: .* 47 actuators, 61 qpos/);
  assert.match(report, /floating roots: g1_floating_base_joint, go2_floating_base_joint/);
  assert.equal(UNITREE_ACTION_LAB_LAYOUT.homeJoints.length, 47);
  assert.deepEqual(UNITREE_ACTION_LAB_LAYOUT.dynamicRoots, ['g1_pelvis', 'go2_base']);
});
```

- [x] **Step 2: Run the test and verify RED.**

Run: `node --test test/unitree-action-lab-scene.test.mjs`

Expected: FAIL because `src/unitreeActionLab.js` and the local action-lab asset root do not exist.

- [x] **Step 3: Vendor the two source packages mechanically.** Copy the G1 XML/assets/license from the Menagerie mirror and the Go2 + Airbot XML/assets from UniLab. Copy Go2 leg meshes from UniLab's sibling `go2/assets` directory. Rename only the vendored Go2 + Airbot XML to `go2_with_arm.xml`.

```bash
mkdir -p public/assets/unitree-action-lab/robots/{g1,go2_arm,go2}
cp -a ../google-deepmind__mujoco_menagerie/unitree_g1/assets public/assets/unitree-action-lab/robots/g1/
cp ../google-deepmind__mujoco_menagerie/unitree_g1/g1.xml public/assets/unitree-action-lab/robots/g1/g1.xml
cp ../google-deepmind__mujoco_menagerie/unitree_g1/LICENSE public/assets/unitree-action-lab/robots/g1/LICENSE
cp -a /data/private/user2/workspace/14.unilab/code/src/unilab/assets/robots/go2_arm/assets public/assets/unitree-action-lab/robots/go2_arm/
cp /data/private/user2/workspace/14.unilab/code/src/unilab/assets/robots/go2_arm/go2_with_arm_mjx_full_collision.xml public/assets/unitree-action-lab/robots/go2_arm/go2_with_arm.xml
cp /data/private/user2/workspace/14.unilab/code/LICENCE public/assets/unitree-action-lab/robots/go2_arm/LICENSE
cp -a /data/private/user2/workspace/14.unilab/code/src/unilab/assets/robots/go2/assets public/assets/unitree-action-lab/robots/go2/
```

- [x] **Step 4: Adapt the vendored Go2 model with `apply_patch`.** Keep all free-joint, inertial, collision, mesh-reference, and actuator properties. Name the free joint `floating_base_joint`, give the six existing Airbot `<position>` actuators names `joint1` through `joint6`, and add an XML comment declaring the file modified for this scene. No geometric or dynamic parameter is removed.

- [x] **Step 5: Add the root scene and layout.** `scene.xml` must use local child models and separate them by 2.2 m:

```xml
<asset>
  <model name="g1_dynamic" file="robots/g1/g1.xml"/>
  <model name="go2_arm_dynamic" file="robots/go2_arm/go2_with_arm.xml"/>
</asset>
<worldbody>
  <geom name="floor" type="plane" size="0 0 0.05" friction="1 0.01 0.001"/>
  <frame pos="-1.10 0 0"><attach model="g1_dynamic" body="pelvis" prefix="g1_"/></frame>
  <frame pos="1.10 0 0"><attach model="go2_arm_dynamic" body="base" prefix="go2_"/></frame>
</worldbody>
```

`UNITREE_ACTION_LAB_LAYOUT.homeJoints` is the literal concatenation of the 29-value G1 `stand` control vector and the 18-value Go2 + Airbot `home` control vector recorded in the spec audit.

- [x] **Step 6: Extend the offline validator.** Add a `unitreeActionLab` definition with no XML patches or scene objects, allow a direct layout definition, and print floating joint names when the scene key is `unitreeActionLab`.

- [x] **Step 7: Run the focused test and compile until GREEN.**

Run:

```bash
node --test test/unitree-action-lab-scene.test.mjs
INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 LIST_NAMES=1 \
  node scripts/validate-mjcf.mjs unitreeActionLab public/assets/unitree-action-lab
```

Expected: 47 actuators, 61 qpos, exactly two named free roots, no penetration deeper than 5 mm, and PASS.

- [ ] **Step 8: Commit the asset-scene slice.**

```bash
git add public/assets/unitree-action-lab src/unitreeActionLab.* scripts/validate-mjcf.mjs test/unitree-action-lab-scene.test.mjs
git commit -m "feat(scene): add dynamic Unitree models"
```

---

### Task 2: Pure action clip and actuator-only writer

**Files:**
- Create: `src/unitreeActionSequence.js`
- Create: `src/unitreeActionSequence.d.ts`
- Create: `test/unitree-action-sequence.test.mjs`

**Interfaces:**
- Produces: `sampleUnitreeAction(elapsedSeconds): UnitreeActionSample`.
- Produces: `applyUnitreeActionTargets(ctrl, actuatorIds, sample): void`, where `ctrl` is the only mutable argument.
- Produces: named exports `G1_ACTUATORS`, `GO2_ACTUATORS`, `UNITREE_ACTION_PHASES`, `UNITREE_ACTION_DURATION`, `G1_HOME`, and `GO2_HOME`.

- [ ] **Step 1: Write failing sampler tests with hand-derived expectations.** Cover home at `t=0`, continuous equality at all six phase boundaries, a non-home G1 wrist target and non-home Airbot target inside `scan-wave`, the official 12-value Go2 lower target at `t=7.0`, and exact home targets at `t=10.0`. Assert all samples are finite and inside literal control limits.

```js
for (const boundary of [1, 2.5, 5.5, 7, 8.5, 10]) {
  const left = sampleUnitreeAction(boundary - 1e-7);
  const right = sampleUnitreeAction(boundary + 1e-7);
  assert.ok(maxAbsDiff(left.g1Targets, right.g1Targets) < 1e-5);
  assert.ok(maxAbsDiff(left.go2Targets, right.go2Targets) < 1e-5);
}
```

- [ ] **Step 2: Write a failing writer-isolation test.** Use a 52-entry sentinel control array, write the 47 mapped actuator IDs in non-contiguous order, and assert the five unmapped entries remain unchanged. The production change this catches is accidental control of unrelated actuators.

- [ ] **Step 3: Run the tests and verify RED.**

Run: `node --test test/unitree-action-sequence.test.mjs`

Expected: FAIL because `unitreeActionSequence.js` is absent.

- [ ] **Step 4: Implement the minimal sampler.** Use `smoothstep(t) = t²(3−2t)` for pose transitions. During `scan-wave`, use a `sin(πp)` envelope so Airbot starts and ends at its home arm pose, and multiply the G1 wrist oscillation by the same envelope so the next phase is continuous. Clamp time to `[0, 10]` and return copied arrays.

- [ ] **Step 5: Implement the isolated writer.** Its signature accepts only `Float64Array | number[]`, actuator IDs, and the sampled vectors; reject non-finite target values and out-of-range IDs before mutating any element.

- [ ] **Step 6: Run the focused tests until GREEN.**

Run: `node --test test/unitree-action-sequence.test.mjs`

Expected: all sampler, boundary, range, and writer-isolation tests pass.

- [ ] **Step 7: Commit the action-data slice.**

```bash
git add src/unitreeActionSequence.* test/unitree-action-sequence.test.mjs
git commit -m "feat(action): add Unitree action clip"
```

---

### Task 3: Offline full-dynamics execution verifier

**Files:**
- Create: `scripts/verify-unitree-action-dynamics.mjs`
- Create: `test/unitree-action-dynamics.test.mjs`

**Interfaces:**
- Consumes: `sampleUnitreeAction`, named actuator arrays, action-lab scene assets, and `UNITREE_ACTION_LAB_LAYOUT.homeJoints`.
- Produces: one JSON summary containing phase visits, maximum joint motion, contact counts, final root heights/tilts, and finite-state status.

- [ ] **Step 1: Write the failing executable test.** Execute the verifier and assert the literal JSON contract:

```js
assert.equal(result.completed, true);
assert.deepEqual(result.visitedPhases, ['settle', 'rise-greet', 'scan-wave', 'lower', 'recover', 'final-hold', 'complete']);
assert.ok(result.g1.maxJointDelta > 0.25);
assert.ok(result.go2.maxLegJointDelta > 0.25);
assert.ok(result.go2.maxArmJointDelta > 0.25);
assert.ok(result.g1.finalHeight >= 0.75 && result.g1.finalHeight <= 0.85);
assert.ok(result.go2.finalHeight >= 0.22 && result.go2.finalHeight <= 0.34);
```

- [ ] **Step 2: Run the test and verify RED.**

Run: `node --test test/unitree-action-dynamics.test.mjs`

Expected: FAIL because the executable verifier does not exist.

- [ ] **Step 3: Implement the verifier.** Mount every action-lab asset into `mujoco-js`, load `scene.xml`, seed `ctrl` plus only actuator-transmitted scalar joint positions from `homeJoints`, and call `mj_forward`. Resolve all 47 actuators by name, sample and apply action targets before each physics step, then simulate 10.0 action seconds plus 1.5 final settling seconds. Never write either floating-root qpos during the rollout.

- [ ] **Step 4: Calculate physical metrics from the compiled model.** Resolve `g1_pelvis`, `go2_base`, floor-contact body names, joint qpos addresses, and root quaternions. Fail on non-finite state, missing phases, no articulated movement, no final foot contact, or any final stability threshold violation.

- [ ] **Step 5: Run the focused test until GREEN.**

Run:

```bash
node --test test/unitree-action-dynamics.test.mjs
node scripts/verify-unitree-action-dynamics.mjs
```

Expected: PASS and a JSON report satisfying every global stability threshold.

- [ ] **Step 6: Commit the physics-verification slice.**

```bash
git add scripts/verify-unitree-action-dynamics.mjs test/unitree-action-dynamics.test.mjs
git commit -m "test(action): verify Unitree dynamics"
```

---

### Task 4: Browser scene integration, action controller, and panel

**Files:**
- Create: `src/UnitreeActionController.tsx`
- Create: `src/UnitreeActionPanel.tsx`
- Create: `src/unitreeActionState.js`
- Create: `src/unitreeActionState.d.ts`
- Modify: `src/configs.ts`
- Modify: `src/controlTargets.js`
- Modify: `src/controlTargets.d.ts`
- Modify: `src/App.tsx`
- Modify: `src/global.d.ts`
- Modify: `src/KeyboardHelp.tsx`
- Modify: `src/styles.css`
- Modify: `test/control-targets.test.mjs`
- Modify: `test/collaborative-scene-integration.test.mjs`
- Create: `test/unitree-action-state.test.mjs`

**Interfaces:**
- Produces: `UnitreeActionState = { status, phase, elapsed, error }` and pure state transitions `startAction`, `pauseAction`, `resumeAction`, `advanceAction`, `completeAction`, and `failAction`.
- Produces: `createUnitreeActionTargets(): ControlTarget[]` with one `controlMode: 'action-sequence'` target.
- Produces: `window.robotDemo.runUnitreeAction()`, `pauseUnitreeAction()`, `resumeUnitreeAction()`, and `getUnitreeActionState()` while the scene is active.

- [ ] **Step 1: Write failing pure state tests.** Verify idle-to-running, pause/resume without elapsed-time advance, monotonic running advance, completion at 10 seconds, reset to idle, and terminal error with the original failure message.

- [ ] **Step 2: Extend existing integration expectations before production code.** Require a ninth `unitreeActionLab` config with local assets, 47 home joints, two physical instances, action-only target, no IK site, and the existing Assembly1 default unchanged.

- [ ] **Step 3: Run focused tests and verify RED.**

Run:

```bash
node --test test/unitree-action-state.test.mjs test/control-targets.test.mjs test/collaborative-scene-integration.test.mjs
```

Expected: FAIL on missing state module, config, and action target.

- [ ] **Step 4: Implement the pure state module and action target.** Add `'action-sequence'` to `ControlTarget.controlMode` and `'unitreeAction'` to `RobotEntry.controlFamily`. Keep the target list non-empty so the existing Leva selector hook remains structurally unchanged.

- [ ] **Step 5: Register `unitreeActionLab` in `configs.ts`.** Use `${import.meta.env.BASE_URL}assets/unitree-action-lab/`, `scene.xml`, the 47-value home vector, camera `[4.2, -5.4, 2.7]`, orbit target `[0, 0, 0.75]`, grid size 6, and one action target.

- [ ] **Step 6: Implement `UnitreeActionController`.** On a new request, resolve all named actuators through `findActuatorByName`, verify uniqueness and MJCF control ranges, and report an error before control writes if resolution fails. In `useBeforePhysicsStep`, sample using accumulated MuJoCo timestep only while running and call `applyUnitreeActionTargets(data.ctrl, ids, sample)`. While paused, reapply the last sampled controls without advancing elapsed time. Notify React only on phase changes and at 10 Hz elapsed-time updates.

- [ ] **Step 7: Implement `UnitreeActionPanel`.** Render the exact Chinese controls from the spec, current phase copy, status, and elapsed time. Disable Execute outside `idle`/`complete`, show Pause only while running, show Resume only while paused, and keep Restart enabled outside loading.

- [ ] **Step 8: Integrate action ownership in `App.tsx`.** Mount the controller/panel only for `unitreeActionLab`; exclude IK gizmo, drag interaction, arm keyboard controllers, and `KeyboardHelp` in this scene. Extend scene diagnostics and `replicatedRootPatterns` to count `g1_pelvis` and `go2_base` as the two physical instances. Reset and scene switching must clear action state and callbacks.

- [ ] **Step 9: Run focused tests until GREEN, then type-check.**

Run:

```bash
node --test test/unitree-action-state.test.mjs test/control-targets.test.mjs test/collaborative-scene-integration.test.mjs
npx tsc --noEmit
```

Expected: all focused tests and TypeScript pass.

- [ ] **Step 10: Commit the browser integration slice.**

```bash
git add src/UnitreeActionController.tsx src/UnitreeActionPanel.tsx src/unitreeActionState.* src/configs.ts src/controlTargets.* src/App.tsx src/global.d.ts src/KeyboardHelp.tsx src/styles.css test/unitree-action-state.test.mjs test/control-targets.test.mjs test/collaborative-scene-integration.test.mjs
git commit -m "feat(action): run Unitree choreography"
```

---

### Task 5: Production-browser verification and visual artifacts

**Files:**
- Create: `scripts/verify-unitree-action-browser.mjs`
- Create: `scripts/capture-unitree-action-video.mjs`
- Modify: `scripts/capture-scenes.mjs`
- Modify: `package.json`
- Create: `artifacts/screenshots/unitree-action-lab.png`
- Create: `artifacts/videos/unitree-action-lab.mp4`
- Modify: `test/browser-verification-contract.test.mjs`

**Interfaces:**
- Produces: deterministic production-browser verification through the action methods on `window.robotDemo`.
- Produces: mid-action screenshot and complete MP4 recording.

- [ ] **Step 1: Write the failing browser-runner contract.** Require `unitreeActionLab` in scene capture, the new verifier/video npm scripts, and a verifier that checks two instances, executes the action, observes `scan-wave`, and waits for `complete`.

- [ ] **Step 2: Run the contract test and verify RED.**

Run: `node --test test/browser-verification-contract.test.mjs`

Expected: FAIL because the new runners and scene entries are absent.

- [ ] **Step 3: Implement the production-browser verifier.** Select `Unitree Action Lab`, wait for two instances and diagnostics, save home joint/root states, call `runUnitreeAction`, capture the screenshot when phase is `scan-wave`, and wait for complete. Assert G1 wrist, Go2 leg, and Airbot joint motion; nonzero final contacts; finite root poses; final height/tilt thresholds; and no page, request, WebGL, or MuJoCo console errors.

- [ ] **Step 4: Implement video capture.** Use Playwright `recordVideo` for a 1440×900 context, run the same action, save the resulting WebM in a temporary directory, and invoke ffmpeg with `libx264 -pix_fmt yuv420p -movflags +faststart` to write `artifacts/videos/unitree-action-lab.mp4`. Remove only the temporary WebM after ffmpeg exits successfully.

- [ ] **Step 5: Extend general capture and npm scripts.** Add the ninth scene with `instances: 2`; add `verify:unitree-action`, `verify:unitree-action-browser`, and `capture:unitree-action-video` scripts.

- [ ] **Step 6: Build and run the production server.**

Run:

```bash
PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH npm run build
PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH npm run preview -- --host 127.0.0.1 --port 3011
```

Run preview in a PTY session and keep its session ID for cleanup.

- [ ] **Step 7: Run browser verification and capture.**

Run:

```bash
SCENE_URL=http://127.0.0.1:3011 npm run verify:unitree-action-browser
SCENE_URL=http://127.0.0.1:3011 npm run capture:unitree-action-video
```

Expected: action reaches `complete`, screenshot and MP4 exist, and all physical thresholds pass.

- [ ] **Step 8: Inspect the screenshot and video.** Use the local image viewer for the screenshot and `ffprobe` for duration, resolution, codec, and frame count. Extract three video frames with ffmpeg to a temporary directory and inspect them for floor penetration, separation, visible G1 gesture, visible Airbot scan, and recovery.

- [ ] **Step 9: Commit production verification and artifacts.**

```bash
git add scripts/verify-unitree-action-browser.mjs scripts/capture-unitree-action-video.mjs scripts/capture-scenes.mjs package.json test/browser-verification-contract.test.mjs artifacts/screenshots/unitree-action-lab.png artifacts/videos/unitree-action-lab.mp4
git commit -m "test(action): capture Unitree sequence"
```

---

### Task 6: Documentation, project tracking, complete gate, and publication

**Files:**
- Create: `docs/progress/2026-08-15-unitree-action-lab.md`
- Modify: `README.md`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`
- Modify: `project/artifacts.jsonl`
- Regenerate: `project/artifacts.md`
- Modify: `test/deployment-contract.test.mjs`

**Interfaces:**
- Produces: source attribution, action semantics, physical verification metrics, visual links, and a ninth-scene Pages contract.

- [ ] **Step 1: Write the failing documentation/deployment expectation.** Update the deployment test to require nine documented scenes and the Unitree screenshot/video links.

- [ ] **Step 2: Run the test and verify RED.**

Run: `node --test test/deployment-contract.test.mjs`

Expected: FAIL because README and generated Pages documentation do not yet describe Unitree Action Lab.

- [ ] **Step 3: Write the progress report and README entry.** Record exact source paths/licenses, model counts, action phase targets, the no-policy/no-qpos-write contract, offline/browser metrics, screenshot, and MP4. Describe it as a stationary whole-body action demonstration, not learned walking.

- [ ] **Step 4: Update project tracking.** Add a completed Unitree Action Lab phase and its decisions, append a standup entry, set next actions to user visual review, register both visual artifacts, and regenerate `project/artifacts.md` with the project-flow-manager script.

- [ ] **Step 5: Run the complete fresh gate.**

Run:

```bash
PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH npm test
PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH npx tsc --noEmit
PATH=/home/shuaijun/.local/node-v22.22.0-linux-x64/bin:$PATH npm run build
INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs unitreeActionLab public/assets/unitree-action-lab
node scripts/verify-unitree-action-dynamics.mjs
git diff --check
```

Expected: zero failed tests, clean TypeScript/build, stable physics report, no strict penetration failure, and clean diff.

- [ ] **Step 6: Commit the documentation slice.**

```bash
git add README.md docs/progress/2026-08-15-unitree-action-lab.md project test/deployment-contract.test.mjs docs/superpowers/plans/2026-08-15-unitree-action-lab.md
git commit -m "docs(scene): record Unitree action lab"
```

- [ ] **Step 7: Merge and push after verification.** Fetch `origin/main`, confirm no divergence, merge the feature branch into `main` without rewriting shared history, and push through `git@github-Shuaijun-LIU:Shuaijun-LIU/web-robot-example-0.git`. Verify local `main` and `origin/main` resolve to the same commit.
