# SO101 Gearbox and XLeRobot Kitting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two independent static collaboration scenes—`SO101 Gearbox` and `XLeRobot Kitting`—while preserving every existing scene unchanged.

**Architecture:** Put both new layout generators in `src/collaborativeSceneLayouts.js`, reusing only exported layout helpers and upstream robot assets. Register them as additional runtime entries with the existing SO101 and XLeRobot controller families. Keep all task objects as MJCF geometry injected only into the copied scene layouts.

**Tech Stack:** React 19, TypeScript, Vite, MuJoCo MJCF/WASM, Node test runner, Playwright.

## Global Constraints

- Do not modify `SO101_LAYOUT` or `XLEROBOT_LAYOUT` behavior, geometry, labels, or positions.
- Add new selector labels `SO101 Gearbox` and `XLeRobot Kitting`.
- Keep this iteration static: no scripted robot motion or task automation.
- Do not add proximity attachment, magnetic grasp, equality weld for payloads, or automatic object following.
- Do not commit or push any files until the user explicitly authorizes it.
- Preserve other agents' unrelated working-tree changes.

---

### Task 1: Independent layout contracts

**Files:**
- Create: `test/collaborative-scene-layouts.test.mjs`
- Create: `src/collaborativeSceneLayouts.js`
- Create: `src/collaborativeSceneLayouts.d.ts`

**Interfaces:**
- Consumes: `fixedBox`, `repeatPose`, upstream SO101/XLeRobot MJCF anchors.
- Produces: `SO101_GEARBOX_LAYOUT`, `XLEROBOT_KITTING_LAYOUT`.

- [x] Write failing tests requiring 4/2 instances, exact station coordinates, named task bodies, free task parts, full-height XLeRobot chassis collision, and no weld/magnet/attach shortcut.
- [x] Run `node --test test/collaborative-scene-layouts.test.mjs`; expect module-not-found failure.
- [x] Implement shared attachment and tray helpers plus the two layout exports in the new module.
- [x] Run the focused test and keep the original layout tests green.

### Task 2: SO101 precision gearbox workcell

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: four radial SO101 attachments and `objects_SO101.xml`/`SO101.xml` patch anchors.
- Produces: central fixture, housing/cover/shaft/gear/spacer/pin stations, named alignment sites, and a four-target SO101 layout.

- [x] Add failing assertions for station placement, two open-bore gear collision rings, two shaft racks, four pins, movable housing/cover, and central fixture clearances.
- [x] Run the focused test and verify it fails because gearbox geometry is absent.
- [x] Add the minimal procedural MJCF workcell and stable table objects.
- [x] Compile with `node scripts/validate-mjcf.mjs so101Gearbox <xlerobot-asset-directory>` and fix only compilation/initial-contact issues exposed by MuJoCo.

### Task 3: XLeRobot kitting workcell

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: two complete XLeRobot attachments and existing full-height rack collision patch.
- Produces: narrow table, source tote, four goods, handoff cradles, scanner/dock, divided order tray, barcode sites, and scene-specific chassis payload decks.

- [x] Add failing assertions for `(-1.10,0)`/`(1.10,0)` starts, `0.72 × 1.00 m` table, exact station coordinates, four product types, two handoff cradles, scanner dock, tray dividers/handles, and physical free bodies.
- [x] Run the focused test and verify it fails because kitting geometry is absent.
- [x] Add the minimal MJCF kitting workcell and scene-specific chassis deck patch without changing base XLeRobot.
- [x] Compile with `node scripts/validate-mjcf.mjs xlerobotKitting <xlerobot-asset-directory>` and verify two physical roots.

### Task 4: Runtime integration and controls

**Files:**
- Modify: `src/configs.ts`
- Modify: `src/App.tsx`
- Modify: `scripts/validate-mjcf.mjs`
- Modify: `scripts/capture-scenes.mjs`
- Modify: `scripts/verify-controls.mjs`
- Modify: `test/scene-layouts.test.mjs`
- Modify: `test/deployment-contract.test.mjs`

**Interfaces:**
- Consumes: new layouts.
- Produces: runtime keys `so101Gearbox` and `xlerobotKitting`, 4 SO101 targets, 2 XLeRobot targets, offline/browser verification support.

- [x] Add failing integration tests for both runtime entries and root diagnostic patterns.
- [x] Run focused integration tests and verify missing runtime entries fail.
- [x] Register both configurations and extend validation/capture/control scripts.
- [x] Run focused tests, TypeScript, and independent control regression for the two new scenes.

### Task 5: Visual artifacts and final verification

**Files:**
- Create: `artifacts/screenshots/so101-gearbox.png`
- Create: `artifacts/screenshots/xlerobot-kitting.png`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/artifacts.md`
- Modify: `project/artifacts.jsonl`

**Interfaces:**
- Consumes: compiled browser scenes.
- Produces: inspectable screenshots and local progress records; no Git commit/push.

- [x] Capture both scenes at `1440×900` after MuJoCo reaches ready state.
- [x] Inspect screenshots for scale, orientation, separation, readability, and initial penetration; correct issues through test-first changes.
- [x] Run `npm test`, `npx tsc --noEmit`, both offline compiles, `npm run build`, and new-scene control verification.
- [x] Record exact verification evidence and leave all changes uncommitted and unpushed.
