# Cooperative Workcells Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two independent four-Panda workcells with benchmark assets and physically verified cooperative tasks.

**Architecture:** A self-contained `franka-cooperative` package shares Panda meshes but has separate scan/pot MJCF files and motion programs. A small actuator player consumes offline planned phases and evaluates scene-specific physical gates. React owns only selection, Reset, Play and progress; no online planning.

**Tech Stack:** React/TypeScript, MuJoCo WASM, CPU Python MuJoCo/trimesh for asset conversion and trajectory validation.

**Spec:** `docs/superpowers/specs/2026-10-07-cooperative-workcells.md`.

## Global Constraints

- Preserve accepted Demo1/Assembly1/Demo2 motions and assets; keep Demo1 as default.
- English UI, four independently controllable Panda arms, 0.002 s control clock.
- Source objects and textures from benchmarks; no fake grasps or object-state animation.
- Preserve hollow collision cavities and real finger/object contact.
- Prove static scenes before automatic tasks; never label a static scene as completed automation.

## Review Focus

- Loading either page directly or switching mid-play must not retain the other controller.
- Missing/malformed motion JSON must disable Play with a readable status, not blank the scene.
- Reset/pause must not leave task time running or retain old object references.
- Source GLB node transforms must affect visual and collision geometry consistently.
- A carried object losing contact or a wrong item reaching a destination must not pass success.

### Task 1: sourced physical layouts and independent pages

**Files:** create `scripts/build-cooperative-assets.py`, `public/assets/franka-cooperative/`, `src/cooperativeWorkcells.js`, `src/cooperativeWorkcells.d.ts`, `src/CooperativeWorkcellPanel.tsx`, `test/cooperative-workcells.test.mjs`, `scripts/verify-cooperative-assets.py`; modify `src/configs.ts`, scoped sections of `src/App.tsx`/`src/styles.css`.

**Interfaces:** produce `COOPERATIVE_WORKCELLS.scan/pot` with `sceneFile`, `title`, `roles`, `homeJoints`, camera/target; MJCF names `r0..r3_joint1..7`, `rN_gripper`, `rN_tcp`, separate named free bodies and supports. Asset manifest contains source hashes, transformed bounds and body start poses.

- [ ] Write consumer tests that fail for absent scene entries and missing package. Assert four available control targets, referenced asset closure, supported/free task objects and separate scene files.
- [ ] Run `node --test test/cooperative-workcells.test.mjs`. Expected: FAIL (new implementation absent).
- [ ] Implement converter preserving textures, Panda controls, sourced collisions and fixture layouts. Add both page entries, English role/status panel, warm backdrop and independent Reset/manual control.
- [ ] Run asset verification in CPU MuJoCo and the node test. Expected: both scenes compile, 32 actuators, four TCPs, finite static settling, no robot penetration, real open pot cavity and visual/collision bounds aligned.
- [ ] Capture both browser layouts and inspect images. Expected: upright robots, table-level bases, visible textured objects, sensible clearance and no load/console errors.
- [ ] Record evidence and commit `feat(workcells): add scanning and pot layouts`.

### Task 2: shared guarded actuator playback

**Files:** create `src/cooperativeMotion.js`, `src/cooperativeMotion.d.ts`, `src/CooperativeMotionRuntime.js`, `src/CooperativeMotionRuntime.d.ts`, `src/CooperativeMotionController.tsx`, `test/cooperative-motion.test.mjs`; extend panel and scoped App state.

**Interfaces:** consume Task 1 scene names. Produce validated program `{version:1,scene,initialJoints,phases}`; each phase has duration, per-arm joint keyframes/gripper targets, allowed contact body pairs and physical end gates. Runtime exposes immutable `{phase,label,stage,reason?}`, time and contact/gate diagnostics. Live writes limited to `ctrl`.

- [ ] Write tests for finite/ranged programs, smooth interpolation, stale/reset clock, malformed fetch handling and contact/gate rejection (including missing bilateral grasp and wrong destination). Expected initial run: FAIL (module absent).
- [ ] Implement fixed-physics player, defensive asset loading and scene-local control ownership; no controller mounted on legacy scenes.
- [ ] Run `node --test test/cooperative-motion.test.mjs` and type check. Expected: all tests pass; invalid program cannot start.
- [ ] Record evidence and commit `feat(workcells): add guarded motion playback`.

### Task 3: complete inspection and packing cycle

**Files:** create `scripts/solve-cooperative-tasks.py`, scan motion JSON, `test/cooperative-wasm.test.mjs`; use a separate planning MjData and reuse tested Jacobian/minimum-jerk patterns without editing Demo2 code.

**Interfaces:** consume Task 1 scene/manifest and Task 2 phase program. Produce a native-verified scan program and independent WASM report.

- [ ] Add a real-WASM test requiring two inspected/packed boxes, real carried contacts, no forbidden collisions and all arms home. Expected: FAIL (program absent).
- [ ] Solve and execute physical approach, grasp, scan, supported handoff, packing and tray dispatch. Export only after native gates pass. Adjust new-scene layout/poses on evidence, recording changes.
- [ ] Run native solver and `node --test --test-name-pattern=scan test/cooperative-wasm.test.mjs`. Expected: all physical gates pass; no scripted object transforms.
- [ ] Inspect rendered critical stages and completion. Expected: actual visible grasps and supported final objects; record screenshot/report paths.
- [ ] Record evidence and commit `feat(demo3): complete inspection and packing`.

### Task 4: complete pot loading and final regression

**Files:** extend new solver/WASM test for pot motion; add browser verification script, progress notes, asset README and main README entries.

**Interfaces:** consume Tasks 1–2; produce pot motion and independent reports/screenshots. No dependency on scan runtime state.

- [ ] Add WASM test requiring two-arm supported lift, two ingredients inside cavity, supported release and all arms home. Expected: FAIL (pot program absent).
- [ ] Solve physically grasped pot lift/hold, alternating loading, lowering and retreat. Export only after native gates pass.
- [ ] Run native solver, `node --test test/cooperative-wasm.test.mjs`, rendered browser checks (both pages, pause/reset/switch), `npm test`, type check and production build. Expected: complete gates and old regressions pass with no console errors.
- [ ] Update progress with precise completion scope, source attribution, screenshots and any remaining limitations; commit `feat(demo4): complete cooperative pot loading`.
- [ ] Run whole-branch fresh review under the execution skill; fix important findings with reproducing tests, then hand off without publishing the new branch unless requested.
