# Franka Assembly Workcell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated fourth static four-Panda assembly workcell without changing the existing three scenes.

**Architecture:** Define a focused `FRANKA_ASSEMBLY_LAYOUT` next to existing layouts and expose it through a fourth `robots` config entry that reuses existing Franka controllers and target definitions. Generate physically aligned compound assembly components through MJCF XML injected by layout patches, while using ordinary fixed scene objects for the shared platform.

**Tech Stack:** React 19, TypeScript, Vite, mujoco-react, MuJoCo MJCF, Node test runner, Playwright.

## Global Constraints

- Preserve `Franka Panda`, `SO101`, and `XLeRobot` behavior and labels.
- Add `Franka Assembly` as the fourth scene rather than replacing a scene.
- Build only a static initial scene; do not add scripted task motion.
- Use a 0.90 m four-Panda ring with all bases aligned to the shared platform top.
- Keep every visible task object physically represented and initially non-overlapping.

---

### Task 1: Fourth-scene contract

**Files:**
- Modify: `test/scene-layouts.test.mjs`
- Modify: `test/deployment-contract.test.mjs`

**Interfaces:**
- Consumes: existing layout exports and `robots` record.
- Produces: tests requiring `FRANKA_ASSEMBLY_LAYOUT`, exact component names, fourth selector entry, and screenshot documentation.

- [ ] **Step 1: Write failing tests** that import `FRANKA_ASSEMBLY_LAYOUT`, assert four instances at 0.90 m, verify named frame/tool/tray components, and require `artifacts/screenshots/franka-assembly.png` in the README.
- [ ] **Step 2: Run `npm test`** and verify failure occurs because the new layout/export and README content do not exist.
- [ ] **Step 3: Commit the red tests** with `test: define Franka assembly workcell contract`.

### Task 2: Assembly layout and runtime entry

**Files:**
- Modify: `src/sceneLayouts.js`
- Modify: `src/sceneLayouts.d.ts`
- Modify: `src/configs.ts`
- Modify: `src/App.tsx`
- Modify: `scripts/capture-scenes.mjs`

**Interfaces:**
- Consumes: `repeatPose`, Franka home pose, attachment-frame generation, `createFrankaTargets`, and selected-instance controller routing.
- Produces: `FRANKA_ASSEMBLY_LAYOUT` and the `frankaAssembly` robot entry with four target controls.

- [ ] **Step 1: Implement the minimal fourth layout** with four cardinal attachments at ±0.90 m, shared platform at the Panda base height, compound central frame, staged components, separate tool stations, fastener tray, and clear handover pad.
- [ ] **Step 2: Add the `Franka Assembly` runtime config** and route its controller family to the existing Franka controller without changing the original entry.
- [ ] **Step 3: Run `npm test` and `npx tsc --noEmit`** and verify all contracts pass.
- [ ] **Step 4: Commit** with `feat: add four-arm assembly workcell`.

### Task 3: Browser and visual verification

**Files:**
- Modify: `scripts/capture-scenes.mjs`
- Create: `artifacts/screenshots/franka-assembly.png`

**Interfaces:**
- Consumes: browser lifecycle datasets and the new robot label.
- Produces: a verified 1440×900 screenshot and reported physical instance/body counts.

- [ ] **Step 1: Start Vite** with local Franka asset routing available to Playwright.
- [ ] **Step 2: Run the fourth-scene capture** and require ready state plus exactly four physical roots.
- [ ] **Step 3: Inspect the screenshot** for object scale, orientation, spacing, platform alignment, and initial intersections; revise only the fourth layout if needed.
- [ ] **Step 4: Re-run control verification** for the original three scenes and the fourth Franka target set.

### Task 4: Documentation and deployment

**Files:**
- Modify: `README.md`
- Modify: `project/task_plan.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`
- Modify: `docs/index.html`
- Replace: `docs/assets/index-*.js`
- Replace: `docs/assets/index-*.css`

**Interfaces:**
- Consumes: verified scene screenshot and successful source tests.
- Produces: published documentation, deterministic Pages bundle, and updated progress record.

- [ ] **Step 1: Document the fourth scene** and embed its screenshot in the README.
- [ ] **Step 2: Update project progress** with implementation and verification results.
- [ ] **Step 3: Run full tests, type checking, production build, and two identical Pages builds** to verify deterministic output.
- [ ] **Step 4: Commit** with `docs: publish Franka assembly workcell`.
- [ ] **Step 5: Push `main` through `github-Shuaijun-LIU`** and verify GitHub Pages serves the same `docs/index.html` hash and referenced assets return HTTP 200.
