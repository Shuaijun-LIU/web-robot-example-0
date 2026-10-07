# Drawer Kitting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Complete a physically actuated four-Panda drawer-opening, retrieval, shared-tray kitting and closing demo.
**Architecture:** Extend the existing cooperative runtime only for passive joint observation. Reuse prior scene assets and solver; isolate drawer conversion and motion generation in new scripts.
**Tech Stack:** React/TypeScript, MuJoCo WASM 3.3.8, native MuJoCo 3.3.7, Python, existing RoboCasa/RoboTwin assets.
**Spec:** `docs/superpowers/specs/2026-10-07-drawer-kitting-design.md`

## Global Constraints

- Preserve existing scenes and Demo1 default.
- Four Panda robots, 32 robot actuators, two passive slide joints; no object-following or drawer actuator.
- Sourced drawer/front/handle geometry with transformation and license records.
- English interface. No commit/push required for this local implementation.

## Review Focus

- A manually moved drawer must reject stale playback preconditions rather than snap back.
- Arm clearance during closing includes front panel, handle and gripper fingers, not just TCP.
- Destination checks require actual support and correct slot, not center distance alone.
- Reset/Pause/scene switching must retain normal ownership and never leave stale motion callbacks.
- Altered shared gate validation must not change scan/pot behavior or accept missing/non-finite joint readings.

### Task 1: Physical drawer assets and observed-joint interface

**Files:** create `scripts/build-drawer-assets.py`, `test/drawer-workcell.test.mjs`, `test/drawer-gates.test.mjs`; modify `src/cooperative{Motion,Workcells}.{js,d.ts}`, `src/CooperativeMotionRuntime.js`, `src/configs.ts`, `src/App.tsx`.
**Interfaces:** scene `drawer`; optional `initialFixtureJoints:[{joint,position,tolerance}]`; gate `{type:'joint-range',joint,min,max,maxSpeed}`; observations `{position,speed}`. The builder emits `public/assets/franka-cooperative/drawer.xml` and `drawer-manifest.json` without overwriting old scene XML.

- [x] Write/run failing schema tests for finite passive joint bounds and stale initial conditions; real-WASM scene loading confirms passive joints, initial support, source dependencies and no new actuators.
- [x] Implement builder, scene registration and passive joint observation. Add real-model coverage for joint state reads and preservation of qpos/forces.
- [x] Run focused tests and inspect generated static layout before planning full motion.

### Task 2: Complete actuator-only motion

**Files:** create `scripts/solve-drawer-task.py`, `test/drawer-wasm.test.mjs`; generated `drawer-motion.json` and native/WASM reports.
**Consumes:** drawer scene and joint gates from Task 1; existing native Workcell IK/phase/pick/place and JS runtime.
**Produces:** complete valid motion plan with contact, opening, tray support, closing, release and HOME gates.

- [x] Write/run failing end-to-end test requiring two physical open/close sequences and both products released in separate tray slots.
- [x] Implement native solver subclass and arm choreography; export only after all gates pass. Log diagnostic failures; do not loosen physical conditions to hide failed grasps.
- [x] Run native and WASM complete cycles, including delayed start and negative precondition tests.

### Task 3: Browser verification and recorded delivery

**Files:** create `scripts/verify-drawer-browser.mjs`; add screenshots, report and `docs/progress/2026-10-07-drawer-kitting.md`; update project tracking.
**Consumes:** exported assets/program and existing page controls; produces actual rendered end-to-end evidence and reproducible source/design records.

- [x] Verify scene loading, complete playback, Pause, Reset and switching in actual browser with software rendering; capture closed/open/packing/final states.
- [x] Run complete Node suite, TypeScript and production build; review full diff with a fresh reviewer and fix important findings with reproducing tests.
- [x] Record actual results, design adjustments and deferred limitations. Hand off local scene and evidence without claiming deployment.
