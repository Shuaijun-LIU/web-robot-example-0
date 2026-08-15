# Unitree Locomotion Suite Implementation Plan

> **For Codex:** Execute this plan task by task with tests first. Do not weaken
> the free-root, actuator-only, displacement, or final-stability requirements to
> make a test pass.

**Goal:** Add a selectable 25-second G1/Go2 locomotion action program to the
existing Unitree Action Lab while preserving the verified 10-second greeting
program.

**Architecture:** Pure modules define program timing, nominal squat/gait poses,
attitude feedback, validation, and diagnostics. The React controller resolves
MuJoCo addresses once, reads free-root state before each physics step, and
writes one validated 47-actuator target vector to `data.ctrl`. The existing
scene, models, reset path, and original action program remain compatible.

**Tech stack:** React 19, TypeScript, JavaScript ES modules, mujoco-react,
MuJoCo WASM/native Python probes, Node test runner, Playwright, Vite.

---

## Task 1: Lock program/state compatibility with failing tests

**Files:**

- Modify: `test/unitree-action-sequence.test.mjs`
- Modify: `test/unitree-action-state.test.mjs`
- Modify: `src/unitreeActionSequence.js`
- Modify: `src/unitreeActionSequence.d.ts`
- Modify: `src/unitreeActionState.js`
- Modify: `src/unitreeActionState.d.ts`

1. Add tests proving the original six phases, samples, and 10-second duration
   remain unchanged under a `greeting` program id.
2. Add failing tests for a `locomotion` program with the nine approved phases,
   exact 25-second duration, phase-boundary continuity, and program-aware state
   transitions.
3. Run `node --test test/unitree-action-sequence.test.mjs
   test/unitree-action-state.test.mjs` and confirm the new assertions fail for
   missing program support.
4. Implement the smallest program registry, program-aware sampler entry point,
   and state shape needed to pass while keeping the legacy sampler exports.
5. Re-run the focused tests and commit.

## Task 2: Implement pure G1 and Go2 action controllers test-first

**Files:**

- Create: `test/unitree-locomotion-controller.test.mjs`
- Create: `src/unitreeLocomotionController.js`
- Create: `src/unitreeLocomotionController.d.ts`

1. Add failing tests for smooth squat depth/return, alternating G1 leg phases,
   G1 counter-swinging arms, Go2 diagonal-pair phasing, entry/exit envelopes,
   feedback signs, feedback clamps, target finiteness, and actuator limits.
2. Run the focused test and verify RED failures are caused by the absent module.
3. Implement pure quaternion conversion, bounded attitude feedback, nominal G1
   squat/walk targets, nominal Go2 trot targets, safe Airbot carry targets, and
   final greeting sampling.
4. Re-run the focused test until GREEN without broadening tolerances.
5. Run the original sequence tests to catch behavioral regressions and commit.

## Task 3: Add runtime address/state extraction and safety guards test-first

**Files:**

- Create: `test/unitree-dynamics-adapter.test.mjs`
- Create: `src/unitreeDynamicsAdapter.js`
- Create: `src/unitreeDynamicsAdapter.d.ts`
- Modify: `src/UnitreeActionController.tsx`
- Modify: `test/unitree-action-dynamics.test.mjs`

1. Add failing pure tests for free-joint address extraction, quaternion
   roll/pitch conversion, displacement calculation, non-finite state rejection,
   G1/Go2 fall thresholds, and atomic 47-target validation.
2. Add a source audit asserting the runtime controller contains no assignment to
   `qpos`, `qvel`, `xpos`, `xquat`, `mocap_*`, `xfrc_applied`, or free-root state.
3. Implement the adapter and extend `UnitreeActionController` to resolve joint
   addresses, read current state, compute feedback, write only `data.ctrl`,
   publish diagnostics, and fail safe to home actuator targets.
4. Run the focused adapter/dynamics tests and commit.

## Task 4: Add program selection and panel state test-first

**Files:**

- Modify: `src/UnitreeActionPanel.tsx`
- Modify: `src/App.tsx`
- Modify: `src/App.css`
- Modify: `test/unitree-action-state.test.mjs`
- Modify: `test/unitree-action-lab-scene.test.mjs`

1. Add failing tests for the two labels, greeting default, disabled selection
   while active, selection-triggered reset, and program-specific duration/copy.
2. Implement the selector and pass `programId` through app state and controller.
3. Keep existing buttons and the original default program behavior intact.
4. Run focused tests plus `npx tsc --noEmit` and commit.

## Task 5: Tune locomotion in native MuJoCo without control shortcuts

**Files:**

- Create: `scripts/simulate-unitree-locomotion.mjs`
- Modify: `public/assets/unitree-action-lab/scene.xml` only if bounded physical
  tuning is required
- Modify: `public/assets/unitree-action-lab/robots/g1/g1.xml` only if bounded
  actuator/contact tuning is required
- Modify: `public/assets/unitree-action-lab/robots/go2_arm/go2_with_arm.xml` only
  if bounded actuator/contact tuning is required
- Modify: `test/unitree-action-dynamics.test.mjs`

1. Build a deterministic offline simulation harness using the same pure action
   controller and compiled scene.
2. Record initial/final roots, minimum heights, maximum tilt, contacts, clamp
   counts, and non-finite state for every phase.
3. Tune cadence, stride amplitude, balance gains, floor friction, and existing
   position actuator gains only within realistic bounded ranges.
4. Require G1 x displacement >= `0.30 m`, Go2 x displacement >= `0.40 m`, no
   fall, no NaN, and approved final height/tilt bounds.
5. Save deterministic metrics as JSON under `artifacts/metrics/` and commit the
   controller/tuning only after the offline gate passes.

## Task 6: Extend browser diagnostics and end-to-end verification

**Files:**

- Modify: `scripts/verify-unitree-action-browser.mjs`
- Modify: `scripts/verify-unitree-action-dynamics.mjs`
- Modify: `src/UnitreeActionController.tsx`
- Modify: `src/App.tsx`
- Modify: `package.json`

1. Add failing browser assertions for program selection, all nine locomotion
   phases, live root diagnostics, real root displacement, no error/NaN, and
   final stability.
2. Expose narrowly scoped read-only locomotion diagnostics on the existing
   browser diagnostic object.
3. Make both offline and browser verification scripts execute the original and
   locomotion programs.
4. Run focused verification, full `npm test`, `npx tsc --noEmit`, and
   `npm run build`; fix only evidenced failures.
5. Commit the passing verification integration.

## Task 7: Produce visual artifacts and record measured progress

**Files:**

- Modify: `scripts/capture-unitree-action-video.mjs`
- Modify: `artifacts/screenshots/unitree-action-lab.png`
- Create: `artifacts/videos/unitree-locomotion-suite.mp4`
- Create: `docs/progress/2026-08-15-unitree-locomotion-suite.md`
- Modify: `project/next_actions.md`

1. Capture a screenshot during a clearly visible locomotion phase.
2. Record the complete 25-second browser action, verify duration/codec/frames,
   and visually inspect representative early, middle, and final frames.
3. Record the exact measured displacement, height, tilt, contacts, clamp counts,
   tests, build result, limitations if any, and artifact paths.
4. Run the complete verification matrix once more from a clean tree.
5. Review the final diff for unrelated changes and commit the verified artifacts
   and progress documentation. Do not push until explicitly requested.
