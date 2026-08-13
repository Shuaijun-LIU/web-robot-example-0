# Collaboration Scene Detail and Home Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refine both new collaboration scenes and turn XLeRobot Kitting into a physically reachable open-plan kitchen/dining environment.

**Architecture:** Keep all changes inside the independent collaborative layout module and its contracts. Add procedural MJCF detail helpers for annular components and fixed home furniture, reshape only the XLeRobot Kitting table objects, and preserve runtime/controller integration.

**Tech Stack:** JavaScript, TypeScript declarations, MuJoCo MJCF/WASM, Node test runner, Vite, Playwright.

## Global Constraints

- Do not modify the existing `SO101_LAYOUT` or `XLEROBOT_LAYOUT` scene behavior.
- Do not add weld, magnet, proximity attachment, scripted following, or automatic object motion.
- Keep task objects as collision-driven free bodies and furniture as fixed geometry.
- Keep SO101 bases at the collision-validated radius `0.42 m`.
- Give XLeRobot a `0.05 m` default chassis/table clearance and shared-center arm reach.
- Maintain north/south circulation gaps of at least `0.65 m`.
- Use restrained domestic/industrial colors with no bright cyan.
- Do not commit or push until explicitly authorized by the user.

---

### Task 1: Distance and detail contracts

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`
- Modify: `src/collaborativeSceneLayouts.d.ts`

**Interfaces:**
- Consumes: `SO101_GEARBOX_LAYOUT`, `XLEROBOT_KITTING_LAYOUT`.
- Produces: assertions for `reachEnvelope`, unique scene-object names, mechanical detail names, domestic furniture names, and physical-free-body contracts.

- [x] Add assertions that SO101 assigned-station distances are `0.21 m`, the `0.42 m` center distance remains inside its nominal chain envelope, and all fixed scene-object names are unique.
- [x] Add assertions for housing/cover bearing seats, shaft keys, open spacer rings, housing ribs, and cover pin sockets.
- [x] Add assertions for XLeRobot centers at `±0.50 m`, `0.05 m` chassis/table clearance, `0.41 m` center reach, and three-piece recessed tabletop dimensions.
- [x] Add assertions for kitchen sink, faucet, prep cabinet, stove, oven, pantry, dining table, storage shelf, produce crate, and product detail geometry.
- [x] Run `node --test test/collaborative-scene-layouts.test.mjs` and verify failures report the missing details and old robot frames.

### Task 2: SO101 mechanical detail refinement

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Test: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: current procedural gearbox bodies and fixed work-surface objects.
- Produces: `annularGeometry(name, innerRadius, outerRadius, halfHeight, rgba, mass)` and refined `SO101_GEARBOX_LAYOUT.reachEnvelope`.

- [x] Add a reusable annular collision helper using 12 radial box segments so spacers and bearing seats retain real central clearance.
- [x] Replace both solid spacer cylinders with annular collision geometry and preserve their free joints.
- [x] Add named bearing-seat segments, shaft keys/shoulders, housing ribs, gear keyway sites, and four cover pin sockets.
- [x] Preserve unique table-leg names and expose exact SO101 reach distances.
- [x] Run the focused layout test until all SO101 contracts pass.
- [x] Compile `so101Gearbox` with `node scripts/validate-mjcf.mjs so101Gearbox <asset-directory>`.

### Task 3: Reachable recessed island and detailed kitting objects

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `src/collaborativeSceneLayouts.d.ts`
- Test: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: XLeRobot parent attachment patch and chassis payload-deck patch.
- Produces: `XLEROBOT_KITTING_LAYOUT.reachEnvelope`, three fixed tabletop components, `±0.50 m` attachment frames, and rear payload decks.

- [x] Replace the single table slab with a `0.24 × 0.50 m` half-size spine and two `0.36 × 0.115 m` half-size end wings at `y=±0.385 m`.
- [x] Move chassis frames to `x=±0.50 m`, move the payload deck to local rear `x=+0.255 m`, and move table legs clear of the docking recesses.
- [x] Reposition tote, cradles, scanner, and order tray to the approved station coordinates.
- [x] Add tote ribs/divider/label, product label and closure details, scanner controls/speaker/battery seam, and tray corner bumpers.
- [x] Run the focused layout test until distance, table, and object-detail contracts pass.
- [x] Compile `xlerobotKitting` and confirm exactly two chassis roots.

### Task 4: Open-plan kitchen and dining environment

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Test: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: the reachable island geometry from Task 3.
- Produces: fixed `XLEROBOT_HOME_ENVIRONMENT_XML` and free `produce_crate`, `tomato`, `cucumber`, and `bell_pepper` bodies.

- [x] Add the north kitchen line with a physically recessed sink basin, segmented faucet, prep drawers, cutting board, four stove burners, oven door/handle, and tall pantry doors/shelves.
- [x] Add the south dining table and free produce crate contents at `y≈-1.45 m`.
- [x] Add the west storage shelf and storage bins at `x≈-1.65 m`.
- [x] Add a back wall and expand the fixed floor while preserving at least `0.65 m` circulation gaps.
- [x] Update the camera to frame the complete room and run the focused test.
- [x] Compile the scene and correct MuJoCo errors and initial physical overlaps until both scenes report zero penetrating contacts.

### Task 5: Browser inspection and complete verification

**Files:**
- Replace: `artifacts/screenshots/so101-gearbox.png`
- Replace: `artifacts/screenshots/xlerobot-kitting.png`
- Create: `docs/progress/2026-08-13-collaboration-scene-detail-home.md`

**Interfaces:**
- Consumes: compiled refined scenes.
- Produces: inspectable screenshots and exact local verification evidence.

- [x] Capture both scenes at `1440×900` after their scene status is `ready`.
- [x] Inspect object scale, tabletop support, rear-deck clearance, furniture orientation, circulation lanes, and free-object stability; correct visual/physical defects through new failing assertions where applicable.
- [x] Run SO101 four-arm and XLeRobot two-device control regression.
- [x] Run `npm test`, `npx tsc --noEmit`, both offline MuJoCo compiles, a production Vite build, and `git diff --check`.
- [x] Record counts, screenshots, distance evidence, and the intentional uncommitted/unpushed repository state.
