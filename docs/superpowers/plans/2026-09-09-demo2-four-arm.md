# Demo2 Four-arm Sorting Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for the tightly coupled physics/controller work; complete all tasks without intermediate user approval pauses, as requested.

**Goal:** Physically validated two-arm coordination followed by four-arm continuous sorting of all 16 eggs.

**Architecture:** Generate contact-verified per-egg actuator paths, derive a safe concurrent schedule, and replay through a separate scoped controller with real-state gates. Preserve accepted single-egg recipes and every other scene.

**Tech Stack:** Native MuJoCo 3.3.7, browser MuJoCo 3.3.8, Python/NumPy, React/TypeScript, Node tests and Playwright.

**Spec:** `docs/superpowers/specs/2026-09-09-demo2-four-arm-design.md`

## Global Constraints

- No object pose/velocity writes after initialization, welds or invisible supports.
- Preserve Demo1/Assembly1, all other scenes, default entry and existing scene physics.
- Forbidden penetration below 0.1 mm and finger/egg penetration below 1 mm.
- Bilateral contact before lift, lift above 60 mm, no sustained grasp loss over 0.12 s.
- Final support, no finger contact, tilt below 20 degrees, cell error below 12 mm.
- All 16 eggs reach their class tray, four each, then all arms return home.

## Task 1: Native per-egg grasp/place and physical audit

Files: create `scripts/solve-egg-sorting.py`, `test/egg-sorting-physics.py`; consume unchanged `scripts/solve-egg-transfer.py` and scene assets. Export `sorting-motion.json` only after successful dynamic verification.

- [x] Red test calls `build_and_verify(root, arms=2)` and asserts two distinct arms, completed class-owned cells, zero forbidden collisions and real contact before lift.
- [x] Implement a shared-world four-arm actuator driver, reusable seeded IK, per-egg state and class/cell assignment. First validate each independent pick/place in the occupied scene.
- [x] Add full-16 acceptance assertions, run native physical tests and record trajectory failures with named contacts/stages.

```python
result = build_and_verify(ROOT, arms=4, rounds=4)
assert result['success']
assert result['counts'] == [4, 4, 4, 4]
assert result['metrics']['maxForbiddenPenetration'] < .0001
```

## Task 2: Checked coordination and web-engine replay

Files: create `src/eggSorting.js`, matching declarations, `test/egg-sorting-coordination.test.mjs`, `test/egg-sorting-wasm.test.mjs`. Preserve the existing static `test/egg-sorting.test.mjs`. Extend native generator with physically checked concurrency.

- [x] Red tests: reject conflicting reservations, allow independent paths, release waiting requests deterministically, reject duplicate eggs/cells and incomplete final counts.
- [x] Define a versioned motion interface with 4-arm actuator paths, per-egg identity/class/cell and explicit stage gates; schedule conflict waits without browser IK.
- [x] Replay two-arm and four-arm programs using real `mujoco-js`, actual contacts and per-tick safety checks; export reports only after terminal success.

```js
assert.equal(sourcePathsConflict({arm:0,eggIndex:0},{arm:1,eggIndex:4}), true);
assert.equal(sourcePathsConflict({arm:0,eggIndex:0},{arm:2,eggIndex:8}), false);
assert.deepEqual(report.state.counts, [4,4,4,4]);
```

## Task 3: Continuous UI, browser evidence and regression

Files: create `src/EggSortingController.tsx`, `scripts/verify-egg-sorting-runtime.mjs`; preserve the existing static scene verifier; modify only Demo2 integration in `src/App.tsx`; update progress and project tracking.

- [x] Browser acceptance starts the new four-arm button, observes real overlap and a wait, and requires completion with 16 sorted eggs. Reset cancels actuator ownership before any stale target write.
- [x] Implement controller using physics time, per-arm feedback and strict contact gates; lock competing controls only while active/error. Retain original buttons and default scene.
- [x] Capture actual overview, simultaneous grasp/transport and final occupied trays. Run original transfer/reseat regressions, all Node tests, TypeScript and production build.
- [x] Independent read-only code review, fix material findings and update measured progress.
- Publication follows verified acceptance; confirm the push and Pages workflow separately in the delivery report.

## Execution ledger

- Baseline: `284d045` committed and pushed; 226 tests, type check and build pass.
- Approach: execute inline on the feature branch because path generation, occupancy and runtime physics are tightly coupled. No changes to the accepted asset geometry are planned.
- Final acceptance: native, shared WASM runtime and actual-page full-16 replay all pass; 236 Node tests, focused exception checks, TypeScript, production build and original reseating regression pass. Reports and screenshots are recorded in the progress document.
