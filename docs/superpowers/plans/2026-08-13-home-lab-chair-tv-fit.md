# Home Lab Chair and TV Fit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the Home Lab office chair complete armrests and place the TV and speakers correctly on their console.

**Architecture:** Keep all visual geometry in `src/so101HomeLabEnvironment.js`. Extend the existing Home Lab layout test with hand-derived surface and footprint checks against the emitted MJCF, then refresh the existing screenshot and progress record.

**Tech Stack:** JavaScript, MuJoCo MJCF, Node test runner, Playwright, Vite.

## Global Constraints

- Modify only `SO101 Home Lab` furniture and its documentation/test artifacts.
- Preserve all robot, workcell, control, and physics mappings.
- Require positive TV-to-speaker clearance and exact cabinet-top support heights.

---

### Task 1: Furniture geometry contracts

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Consumes: `SO101_HOME_LAB_LAYOUT.xmlPatches`.
- Produces: regression checks for armrest construction and TV-console fit.

- [x] **Step 1: Write failing assertions** for four named arm supports, two
  padded rails, common cabinet-top support height, contained footprints, and
  positive TV-to-speaker gaps.
- [x] **Step 2: Run the Home Lab detail test** and confirm it fails against the
  current two-stick chair and intersecting media furniture.

### Task 2: Chair and media-console implementation

**Files:**
- Modify: `src/so101HomeLabEnvironment.js`
- Modify: `docs/progress/2026-08-13-so101-home-lab-layout-mobility.md`
- Replace: `artifacts/screenshots/so101-home-lab.png`

**Interfaces:**
- Produces: corrected fixed MJCF bodies consumed by the existing Home Lab
  layout; no new runtime interface.

- [x] **Step 1: Replace chair arm sticks** with front/rear supports and padded
  longitudinal rails on both sides.
- [x] **Step 2: Resize and refit the media group** by widening the console,
  expanding its doors, centering/raising the TV, and seating both speakers on
  the console outside the screen.
- [x] **Step 3: Run the focused test** and confirm all geometry contracts pass.
- [x] **Step 4: Compile and capture** Home Lab, inspect the screenshot, and
  record final body/geom/actuator/contact counts.
- [x] **Step 5: Run the complete gate:** `npm test`, `npx tsc --noEmit`,
  `npm run build`, and `git diff --check`.
