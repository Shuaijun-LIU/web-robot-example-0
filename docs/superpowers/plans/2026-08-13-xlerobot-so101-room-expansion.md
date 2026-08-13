# XLeRobot Kitting and SO101 Gearbox Room Expansion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expand XLeRobot Kitting into a detailed three-wall kitchen and SO101 Gearbox into a large furnished room containing static high-fidelity G1 and Go2+Arm models, without moving the existing SO101 workcell or introducing initial penetrations.

**Architecture:** Keep the existing layout-patch architecture and add explicit room metadata/XML to `collaborativeSceneLayouts.js`. Give SO101 Gearbox a self-contained local asset root containing its original SO101 files plus static, actuator-free G1 and Go2+Arm child models attached with unique prefixes; keep XLeRobot Kitting on its current source and change only its patched worldbody and camera.

**Tech Stack:** JavaScript MJCF builders, MuJoCo 3.1 WASM via `mujoco-js`/`mujoco-react`, React 19, Vite 7, Node test runner, Playwright.

## Global Constraints

- Do not move or resize the existing SO101 Gearbox arms, central table, task stations, fixture, housing, shafts, gears, cover, or pins.
- G1 and Go2+Arm are static visual scene robots in this iteration; do not add control selectors or motion.
- Do not add weld, magnet, proximity attachment, or automatic carrying behavior.
- Preserve unrelated dirty worktree changes and do not push without a later user instruction.
- Strict initial penetration threshold is `5 mm`; target zero negative-distance contacts.

---

### Task 1: Specify room geometry and preserved workcell contracts

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`
- Modify: `src/collaborativeSceneLayouts.js`

**Interfaces:**
- Consumes: existing `SO101_GEARBOX_LAYOUT`, `XLEROBOT_KITTING_LAYOUT`, and `patchText(layout)` test helper.
- Produces: `roomBounds`, `protectedWorkcellRadius`, `roomZones`, and the XLe three-wall/cabinet XML names used by later integration checks.

- [ ] **Step 1: Write failing layout tests**

Add literal assertions that the SO room is at least `9.2 × 7.2 m`, its protected radius is `1.4 m`, all existing `taskStations` retain their current values, and the XLe XML contains west/east walls, a west window, east wall art, and an east-wall cabinet.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `node --test test/collaborative-scene-layouts.test.mjs`  
Expected: FAIL because room metadata and new XML names do not exist.

- [ ] **Step 3: Implement XLe walls and room metadata**

Add fixed MJCF bodies named `kitchen_west_wall`, `kitchen_east_wall`, `kitchen_west_window`, `kitchen_east_wall_art_1`, `kitchen_east_wall_art_2`, and `kitchen_side_cabinet`. Move the XLe camera to the open south side. Add SO room metadata without changing existing task data.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `node --test test/collaborative-scene-layouts.test.mjs`  
Expected: PASS.

### Task 2: Add SO101 room furniture while preserving the central workcell

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`
- Modify: `src/collaborativeSceneLayouts.js`

**Interfaces:**
- Consumes: Task 1 room bounds and protected radius.
- Produces: fixed room XML bodies for lounge and office zones; enlarged `gearbox_floor` scene object.

- [ ] **Step 1: Write failing furniture and clearance tests**

Assert the patch contains `gearbox_room_west_wall`, `gearbox_room_east_wall`, `gearbox_room_back_wall`, `gearbox_sofa`, `gearbox_tv`, `gearbox_desk`, both monitors, and that each declared zone center lies outside the literal `1.4 m` protected radius.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `node --test test/collaborative-scene-layouts.test.mjs`  
Expected: FAIL on the missing furniture bodies.

- [ ] **Step 3: Implement fixed room and furniture MJCF**

Build the west lounge and northeast office from fixed geoms, expand only `gearbox_floor`, retain all original workcell scene objects and positions, and set a south-open room camera framing the entire scene.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `node --test test/collaborative-scene-layouts.test.mjs`  
Expected: PASS.

### Task 3: Vendor and statically attach G1 and Go2+Arm

**Files:**
- Create: `public/assets/so101-gearbox-room/objects_SO101.xml`
- Create: `public/assets/so101-gearbox-room/SO101.xml`
- Create: `public/assets/so101-gearbox-room/*.stl`
- Create: `public/assets/so101-gearbox-room/robots/g1/g1_static.xml`
- Create: `public/assets/so101-gearbox-room/robots/g1/assets/*`
- Create: `public/assets/so101-gearbox-room/robots/go2_arm/go2_arm_static.xml`
- Create: `public/assets/so101-gearbox-room/robots/go2_arm/assets/*`
- Create: `public/assets/so101-gearbox-room/THIRD_PARTY_NOTICES.md`
- Modify: `test/collaborative-scene-layouts.test.mjs`
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `src/configs.ts`

**Interfaces:**
- Consumes: MuJoCo Menagerie G1 assets and UniLab Go2+Arm/Go2 assets already present locally.
- Produces: `g1_room_model`, `go2_arm_room_model`, attached bodies prefixed `room_g1_` and `room_go2_`, and local `SO101_GEARBOX_BASE` in `configs.ts`.

- [ ] **Step 1: Write failing attachment contract tests**

Assert the SO scene patch declares both model files, attaches `pelvis` and `base` with unique prefixes, and the asset root is local. Add a filesystem test that the static XML files exist and contain neither `<freejoint` nor `<actuator>`.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `node --test test/collaborative-scene-layouts.test.mjs test/collaborative-scene-integration.test.mjs`  
Expected: FAIL because static robot assets and local source do not exist.

- [ ] **Step 3: Prepare static assets and connect them**

Copy only referenced G1 meshes, Go2 visual meshes, arm meshes, and original SO101 assets. Generate static MJCF variants by removing body joints/freejoints and actuator blocks and disabling collision proxy rendering/contact. Declare and attach both child models outside the protected workcell.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `node --test test/collaborative-scene-layouts.test.mjs test/collaborative-scene-integration.test.mjs`  
Expected: PASS.

### Task 4: Compile scenes and eliminate penetrations

**Files:**
- Modify if required: `src/collaborativeSceneLayouts.js`
- Modify if required: `public/assets/so101-gearbox-room/robots/g1/g1_static.xml`
- Modify if required: `public/assets/so101-gearbox-room/robots/go2_arm/go2_arm_static.xml`

**Interfaces:**
- Consumes: completed local SO asset directory and existing XLe asset directory used by the validator.
- Produces: compile evidence, 24-actuator SO scene, and zero severe initial penetration.

- [ ] **Step 1: Compile SO101 Gearbox with names and contacts**

Run: `INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 LIST_NAMES=1 node scripts/validate-mjcf.mjs so101Gearbox public/assets/so101-gearbox-room`  
Expected: compile succeeds, four `r*_Base` bodies and both static robot roots are present, `24 actuators`, and no penetration below `-0.005 m`.

- [ ] **Step 2: Compile XLeRobot Kitting with contacts**

Run: `INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs xlerobotKitting <local-xlerobot-asset-directory>`  
Expected: compile succeeds with two XLeRobot instances and no penetration below `-0.005 m`.

- [ ] **Step 3: Correct only measured geometry defects**

Use reported geom/body pairs to adjust the smallest relevant position or clearance. Re-run the corresponding strict validator after each correction until it exits zero.

### Task 5: Browser, controls, screenshots, and full verification

**Files:**
- Modify: `artifacts/screenshots/so101-gearbox.png`
- Modify: `artifacts/screenshots/xlerobot-kitting.png`
- Create: `docs/progress/2026-08-13-xlerobot-so101-room-expansion.md`

**Interfaces:**
- Consumes: compiled room layouts and local SO assets.
- Produces: browser-load evidence, refreshed visual artifacts, and a progress record.

- [ ] **Step 1: Run automated checks**

Run: `npm test && npx tsc --noEmit && npm run build`  
Expected: all commands exit zero.

- [ ] **Step 2: Start Vite and capture the two scenes**

Run Vite on a free local port, then run: `SCENES=so101Gearbox,xlerobotKitting node scripts/capture-scenes.mjs`.  
Expected: both scenes report expected instance counts and write their PNGs.

- [ ] **Step 3: Verify existing SO/XLe controls**

Run: `node scripts/verify-controls.mjs` against the same server.  
Expected: all existing SO101 and XLeRobot targets resolve and move as before; no G1/Go2 control target is added.

- [ ] **Step 4: Inspect screenshots at original resolution**

Check wall continuity, window/painting orientation, cabinet-wall gap, sofa/TV facing, desk screens, robot scale, floor contact, central clearance, and camera occlusion. If a visual defect is found, correct it and repeat compile/contact/capture checks.

- [ ] **Step 5: Record exact evidence**

Write the changed files, asset sources, test counts, model body/geom/actuator/qpos counts, contact report, screenshot paths, and any deliberately deferred dynamic-control work to the progress document.

