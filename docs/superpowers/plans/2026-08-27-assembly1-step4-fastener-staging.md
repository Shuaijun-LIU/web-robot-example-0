# Assembly1 Step 4 Fastener Staging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robustly finish Steps 2/3 and add a physical four-arm first-fastener staging action.

**Architecture:** Keep deterministic arm motion in pure state/control-frame modules and isolate MuJoCo resource lookup, contacts, and actuator writes in React controllers. Step 2 gains recent-contact evidence; Step 3 gains split alignment, one reseat, and physical release; Step 4 follows the same pure-machine plus runtime-controller boundary.

**Tech Stack:** JavaScript/TypeScript, React 19, mujoco-react, MuJoCo WASM, Node test runner, Playwright, Vite.

**Spec:** `docs/superpowers/specs/2026-08-27-assembly1-step4-fastener-staging-design.md`

## Global Constraints

- No free-body `qpos` writes, welds, magnets, proximity attachment, or scripted object poses.
- Step 1–3 successful trajectories remain unchanged before Step 3 alignment verification.
- Step 2 contact grace is `0.20 s`; verification deadline is `4.0 s`.
- Step 3 planar alignment is `0.030 m`; vertical tolerance is `0.025 m` before release and `0.020 m` after physical seating.
- Step 3 completes with gripper controls `[48, 96, 255, 255]`.
- Reset returns all four steps to idle.

---

### Task 1: Robust Step 2 Evidence

**Files:**
- Modify: `test/assembly-step2.test.mjs`
- Modify: `src/assemblyStep2.js`
- Modify: `src/assemblyStep2.d.ts`
- Modify: `src/AssemblyStep2Controller.tsx`

**Interfaces:**
- Produces: `advanceAssemblyStep2ContactMemory(memory, dt, left, right)` and a state machine that retains `lastInvalidVerdict`.

- [ ] Add tests proving a one-frame left-contact gap inside `0.20 s` remains valid, a longer gap fails, and timeout reports the retained specific verdict.
- [ ] Run `node --test test/assembly-step2.test.mjs` and confirm the new assertions fail because recent-contact memory is absent.
- [ ] Implement contact-memory aging in the pure module and feed it from the runtime controller without changing other grasp checks.
- [ ] Run `node --test test/assembly-step2.test.mjs` and confirm it passes.

### Task 2: Step 3 Split Alignment and Physical Release

**Files:**
- Modify: `test/assembly-step3.test.mjs`
- Modify: `src/assemblyStep3.js`
- Modify: `src/assemblyStep3.d.ts`
- Modify: `src/AssemblyStep3Controller.tsx`
- Modify: `src/AssemblySequencePanel.tsx`

**Interfaces:**
- Consumes: existing Step 3 aligned and hover joint targets.
- Produces: phases `reseat-lift`, `reseat-descent`, `release`, `release-settle`, `retreat`, and `placed-verification`; diagnostics expose planar and vertical hole errors.

- [ ] Add tests for planar/vertical separation, exactly one reseat, gripper opening interpolation, retreat targets, and final placed verification.
- [ ] Run `node --test test/assembly-step3.test.mjs` and confirm failures identify the missing phases and metric fields.
- [ ] Implement the state machine and runtime sampling, keeping object pose fully physics-driven.
- [ ] Run `node --test test/assembly-step3.test.mjs` and confirm it passes.

### Task 3: Step 4 Pure Contract and Reachable Layout

**Files:**
- Create: `test/assembly-step4.test.mjs`
- Create: `src/assemblyStep4.js`
- Create: `src/assemblyStep4.d.ts`
- Modify: `src/frankaAssemblyLayouts.js`
- Create: `scripts/solve-assembly-step4-waypoints.mjs`

**Interfaces:**
- Produces: `createAssemblyStep4Machine`, `advanceAssemblyStep4Machine`, `createAssemblyStep4ControlFrame`, recorded four-arm waypoint contracts, and the inward fastener station.

- [x] Add tests for reachable tray coordinates, ordered Step 4 phases, independent arm roles, gripper commands, and absence of attachment APIs.
- [ ] Run `node --test test/assembly-step4.test.mjs` and confirm it fails because the module and layout do not exist.
- [x] Implement the pure state/control contract and offline IK generator; run it against the local Menagerie assets until all targets meet limits and pose tolerances.
- [x] Run `node --test test/assembly-step4.test.mjs` and confirm it passes.

### Task 4: Step 4 Runtime and UI Integration

**Files:**
- Create: `src/AssemblyStep4Controller.tsx`
- Modify: `src/App.tsx`
- Modify: `src/AssemblySequencePanel.tsx`
- Modify: `src/types.d.ts`
- Modify: `test/ui-contract.test.mjs`

**Interfaces:**
- Consumes: Step 3 completion ownership and Step 4 pure control frames.
- Produces: fourth request/state/diagnostics, `runAssemblyStep4`, `getAssemblyStep4Diagnostics`, dataset status, button gating, and reset behavior.

- [ ] Add UI contract tests for fourth-step gating, diagnostics, ownership, and reset.
- [ ] Run the focused tests and confirm the missing controller/UI contracts fail.
- [x] Implement runtime resource resolution, physical sampling, actuator writes, App state, diagnostics API, and panel copy.
- [x] Run the focused tests and TypeScript checking until both pass.

### Task 5: End-to-End Verification and Records

**Files:**
- Create: `scripts/verify-assembly-step4.mjs`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`

**Interfaces:**
- Produces: repeatable 1→2→3→4 browser evidence and current project handoff records.

- [x] Run `npm test`, `npx tsc --noEmit`, `npm run build`, and the offline IK solver.
- [ ] Run multiple browser sequences and assert no Step 2 timeout/contact false negative, Step 3 ends open and stable, and Step 4 stages the fastener without direct object-state writes.
- [ ] Record exact pass/fail evidence, artifacts, remaining risk, and next action in `project/`.
