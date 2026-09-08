# Demo2 first physical egg transfer implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. User authorized continuation after publishing the static scene; execute inline without another approval checkpoint.

**Goal:** Deliver a reviewable Arm 1 contact grasp, lift, transfer and release of one ivory egg before expanding concurrency.

**Architecture:** Isolated Demo2 controller drives actuator commands only. A CPU MuJoCo trajectory builder solves arm waypoints on separate planning data and validates them through dynamic simulation. Browser uses the same checked targets with real-contact transition gates, not object animation.

**Tech Stack:** MuJoCo native + WASM, NumPy, React/TypeScript, Node tests.

**Spec:** `project/demo2-egg-sorting-proposal.md`, stage B. Stages C/D follow the successful physical gate and are not claimed by this increment.

## Global constraints

- Preserve Demo1/Assembly scenes, targets, default entry and shared physics.
- No runtime writes to egg pose/velocity; no weld, proximity attachment or invisible grip supports.
- Keep all UI English. Reset must cancel motion and restore original scene.
- Whole-arm/environment collision checks; reject missing two-sided finger contact or drops.
- Source UMI fingers are rigid mesh approximations, not validated soft-body hardware.

### Task 1: Physical grasp and trajectory validation

Files: `scripts/solve-egg-transfer.py`, `test/egg-transfer-physics.py`, `public/assets/franka-egg-sorting/first-egg-motion.json`.

- [x] Add a native integration test calling `build_and_verify()`; require egg lift > 60 mm, two distinct finger contacts, final real tray support and < 2 mm post-release drift. With implementation absent this must fail.
- [x] Solve `r0_tcp` poses using Jacobians on separate `MjData`; dynamically replay position-actuator controls on untouched free eggs. Diagnose geometry/contact failures before adjusting gains.
- [x] Export joint phases only after integration test passes; include measured contact, penetration and placement evidence. No successful export on failed verification.

### Task 2: Browser controller and review button

Files: `src/eggTransfer.js`, `.d.ts`, `src/EggTransferController.tsx`, `src/App.tsx`, `test/egg-transfer.test.mjs`.

- [x] Red tests: early phase cannot skip closure contact; a lost carried egg fails; controls interpolate continuously; final support verification does not accept airborne egg.
- [x] Implement pure state/controls functions and a thin `useBeforePhysicsStep` adapter. Lock keyboard/IK and object dragging while this isolated controller owns the scene.
- [x] Add `Run first egg` and status text to Demo2; reuse Reset, export diagnostics for browser tests.
- [x] Run Node tests, TypeScript and production build.

### Task 3: Visual and physical acceptance

Files: `scripts/verify-egg-transfer.mjs`, `docs/progress/2026-09-08-demo2-first-egg.md`, `project/next_actions.md`.

- [x] Browser test actual source egg lift, carried contacts, supported release, no errors/warnings; record carry/completion screenshots alongside static box/tray/gripper views.
- [x] Run existing static Demo2 manual-control/Reset test and keep Demo1 assets unchanged.
- [x] Record measured outcome and any unresolved condition without claiming stages C/D complete. Expand to multi-arm scheduling only when this gate is genuinely passed.
