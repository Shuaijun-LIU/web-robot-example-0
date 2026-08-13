# XLeRobot Kitchen Prop Refinement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Repair the faucet and saucepan silhouettes and add a trash bin, kitchen scale, and recessed preparation bowl to XLeRobot Kitting.

**Architecture:** Keep the complete change inside the independent procedural MJCF home-environment string in `src/collaborativeSceneLayouts.js`. Reuse the existing annular-segment helper for true open cookware/bowl cavities, add fixed scene furniture only, and protect all visible components with focused layout contracts and the existing strict contact validator.

**Tech Stack:** JavaScript, TypeScript declarations, MuJoCo MJCF/WASM, Node test runner, Vite, Playwright.

## Global Constraints

- Do not change XLeRobot attachment frames, control mappings, island geometry, task-station coordinates, or other scenes.
- Do not add weld, magnet, proximity attachment, automatic following, or scripted object motion.
- Preserve the existing `0.05 m` chassis/island clearance and recorded navigation clearances.
- New components are fixed geometry and must produce zero initial penetrating contacts.
- Preserve unrelated local changes from other agents.
- Do not commit or push the implementation unless separately requested.

---

### Task 1: Focused visual and physical contracts

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: `XLEROBOT_KITTING_LAYOUT.xmlPatches` through the existing `patchText(layout)` helper.
- Produces: contracts for connected faucet geometry, open cookware/bowl geometry, trash-bin inventory, scale inventory, and unchanged task/navigation layout.

- [x] Add a focused test requiring `kitchen_faucet_lever_pivot`, `saucepan_bottom`, `saucepan_wall_segment_0`, `saucepan_rim_segment_0`, `kitchen_trash_bin`, `trash_bin_foot_pedal`, `produce_table_scale`, `produce_scale_display`, `produce_prep_bowl`, `prep_bowl_wall_segment_0`, and `prep_bowl_rim_segment_0`.
- [x] Assert that the obsolete solid `saucepan_body` cylinder is absent and that task stations/navigation clearances retain their exact current literal values.
- [x] Run `node --test --test-name-pattern='connected faucet' test/collaborative-scene-layouts.test.mjs`; failure correctly reported the missing faucet pivot.

### Task 2: Faucet and recessed cookware

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Test: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: `annularGeometry(name, innerRadius, outerRadius, halfHeight, rgba, mass, center)`.
- Produces: continuous fixed faucet lever and fixed open-cavity saucepan geometry.

- [x] Add a horizontal cylindrical `kitchen_faucet_lever_pivot` seated against the riser and start `kitchen_faucet_lever` at the pivot's outer face.
- [x] Replace `saucepan_body` with a thin bottom cylinder plus `annularGeometry` wall and rim segments; retain and detail the connected handle.
- [x] Run the focused test and retain RED only for the not-yet-added trash-bin and produce-table objects.
- [x] Run `INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs xlerobotKitting /data/shared/user2/web-robot-assets/MuJoCo-GS-Web/assets/robots/xlerobot`; compile succeeded with zero penetrating contacts.

### Task 3: Southeast trash bin and produce-table tools

**Files:**
- Modify: `src/collaborativeSceneLayouts.js`
- Test: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: fixed home-environment MJCF and `annularGeometry`.
- Produces: fixed southeast trash-bin assembly, fixed digital scale, and fixed recessed preparation bowl.

- [x] Add `kitchen_trash_bin` around `(1.55, -1.25)` with base, four walls, upper rim, recessed dark opening, rear hinge, lid detail, and front foot pedal.
- [x] Add `produce_table_scale` on the right side of the dining table with a low base, top platform, angled display housing, and named display.
- [x] Add `produce_prep_bowl` beside the scale using a bottom plus annular wall and rim segments, keeping clear of the crate and loose produce.
- [x] Run the focused test until it passes.
- [x] Run strict offline compilation and require `initial penetrating contacts: 0`.

### Task 4: Browser and full verification

**Files:**
- Replace: `artifacts/screenshots/xlerobot-kitting.png`
- Create: `docs/progress/2026-08-13-xlerobot-kitchen-prop-refinement.md`
- Modify: `docs/superpowers/plans/2026-08-13-xlerobot-kitchen-prop-refinement.md`

**Interfaces:**
- Consumes: compiled `XLeRobot Kitting` scene.
- Produces: refreshed visual artifact and exact verification record.

- [x] Capture only `xlerobotKitting` at `1440×900` after `sceneStatus=ready` and visually inspect faucet continuity, saucepan/bowl cavities, trash-bin placement, scale, table occupancy, robot clearances, and navigation lanes.
- [x] No additional visual defect remained that required another production adjustment.
- [x] Run `SCENES=xlerobotKitting XLEROBOT_ASSET_DIR=/data/shared/user2/web-robot-assets/MuJoCo-GS-Web/assets/robots/xlerobot npm run verify:controls`; Robot 1/2 keyboard checks passed.
- [x] Run `npm test`, `npx tsc --noEmit`, a production Vite build, strict MuJoCo compilation, and `git diff --check`.
- [x] Record exact geometry/physics counts, contact result, screenshot path, verification commands, and the intentionally uncommitted/unpushed state.
