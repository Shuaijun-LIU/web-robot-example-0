# Demo2 Static Workcell Implementation Plan

> **For agentic workers:** Use inline execution with checkpoints and test-first verification. A bounded read-only reviewer is used for the required final code review; implementation stays inline. Do not publish without authorization.

**Goal:** Deliver a separate inspectable four-Panda egg-sorting scene, without automatic manipulation or changes to accepted assembly scenes.

**Architecture:** A self-contained local MJCF package holds robot/gripper and converted benchmark assets. A small scene-layout module registers the package with existing controls. Conversion retains source geometry and records transformations and licenses. Physical contacts remain enabled; no attachments or scripted egg motion.

**Tech Stack:** Existing React/Three.js/MuJoCo WASM, Node tests, CPU USD/trimesh conversion.

**Spec:** `project/demo2-egg-sorting-proposal.md`, stage A, approved by the user on 2026-09-08.

## Global Constraints

- Preserve Demo1 / Assembly1 at accepted baseline behavior and preserve the default page entry.
- Keep all imported task and gripper assets local and attributed.
- Four Pandas, 16 mixed eggs in one source box, four empty four-cell destination trays.
- Static milestone only; no claims of validated gripping, softness or multi-arm collision avoidance.
- No automatic commits or pushes in this turn.

## Tasks

### 1. Resource package and compilable scene

Files: create `scripts/build-egg-sorting-assets.py`, `public/assets/franka-egg-sorting/`, `test/egg-sorting.test.mjs`.

- [x] Write a failing test that loads the real MJCF using MuJoCo WASM; assert four robot roots, 32 actuators, 16 free eggs, empty output trays, finite poses and aligned tabletop/base height.
- [x] Run `node --test test/egg-sorting.test.mjs` and observe the missing-package failure.
- [x] Convert RoboDojo egg/holder meshes with source hashes; use the open holder base as reusable four-cell inserts. Preserve cavities with convex decomposition, never one solid hull across the holes. Use natural appearance classes with documented scale/color adaptations, not unsupported species labels.
- [x] Import Menagerie Panda and UMI assets and licenses. Keep original finger geometry and opposing-jaw mechanics; align mount/TCP, and explicitly record the simulated mount as an adaptation rather than an engineering-qualified hardware adapter.
- [x] Generate the common box and worktable as simple physical fixtures; use sourced egg-holder surfaces for shaped inserts/output trays.
- [x] Run the real-model test, inspect CPU contact settling, correct pose/collider mismatches and rerun.

### 2. Page registration and inspection

Files: create `src/eggSortingLayout.js`, `src/eggSortingLayout.d.ts`; modify only registration/scene-specific branches in `src/configs.ts`, `src/App.tsx`; create `scripts/verify-egg-sorting.mjs`.

- [x] Register `frankaDemo2` / `Franka Demo2` using `createFrankaTargets()`; preserve the existing default key.
- [x] Use existing per-arm selection, joint diagnostics, keyboard/IK and Reset. Provide a concise English static-review panel describing class assignments, not disabled future-action buttons.
- [x] Browser-test scene loading, four selectable arms, Reset, error-free rendering and local asset loading. Capture overview, box/tray closeup and gripper closeup.
- [x] Run all Node tests, TypeScript and production build; inspect the resulting screenshots. Test switching back to Demo1 without playing/changing its actions.

### 3. Handoff

Files: `docs/progress/2026-09-08-demo2-static.md`, existing `project/` tracking files.

- [x] Record exact achieved status, asset provenance, measurements and remaining grasp/mount validation.
- [x] Provide local preview access and screenshots. Do not change the default page, commit or push until requested.
