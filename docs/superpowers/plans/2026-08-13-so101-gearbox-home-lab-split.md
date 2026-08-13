# SO101 Gearbox / Home Lab Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the compact SO101 Gearbox scene and add a separate, detailed SO101 Home Lab scene with a full home/office layout, static G1, and static Go2 with arm.

**Architecture:** Keep the shared four-arm gearbox task in `collaborativeSceneLayouts.js`, move the large Home Lab environment XML to a focused `so101HomeLabEnvironment.js`, and export two independent layout objects. Register both layouts through the same local asset base and SO101 control-target factory.

**Tech Stack:** JavaScript, TypeScript declarations, React, MuJoCo MJCF/WASM, Node test runner, Playwright, Vite.

## Global Constraints

- `SO101 Gearbox` restores the exact compact floor `[2, 2, 0.005]`, camera `[1.15, -1.15, 1.28]`, FOV `40`, and orbit target `[0, 0, 0.81]`.
- `SO101 Home Lab` uses a 10 m × 8.4 m floor, a protected central radius of 1.6 m, and the south side remains open.
- Both scenes expose four SO101 control targets and exactly 24 actuators.
- G1 and Go2 remain static, collision-free visual room models without control targets.
- Do not change gearbox task coordinates, loose-part physics, or SO101 actuator mappings.
- Do not add automatic attachment, object following, or external runtime downloads.
- Preserve unrelated repository work.

---

### Task 1: Scene split contracts and runtime registration

**Files:**
- Modify: `test/collaborative-scene-layouts.test.mjs`
- Modify: `test/collaborative-scene-integration.test.mjs`
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `src/collaborativeSceneLayouts.d.ts`
- Modify: `src/configs.ts`
- Modify: `src/App.tsx`
- Modify: `scripts/validate-mjcf.mjs`
- Modify: `scripts/capture-scenes.mjs`
- Modify: `scripts/verify-controls.mjs`

**Interfaces:**
- Consumes: existing `SO101_GEARBOX_WORKCELL_XML`, SO101 model patches, and `createSO101Targets()`.
- Produces: `SO101_GEARBOX_LAYOUT`, `SO101_HOME_LAB_LAYOUT`, and runtime key `so101HomeLab` labeled `SO101 Home Lab`.

- [x] **Step 1: Write failing split tests**

Add literal expectations proving that the compact layout has no room metadata,
restores the compact floor/camera, and that `SO101_HOME_LAB_LAYOUT` is a distinct
export registered in config, App diagnostics, validator, capture, and controls.

```js
assert.deepEqual(compactObjects.gearbox_floor.size, [2, 2, 0.005]);
assert.deepEqual(SO101_GEARBOX_LAYOUT.camera, {
  position: [1.15, -1.15, 1.28],
  fov: 40,
});
assert.doesNotMatch(patchText(SO101_GEARBOX_LAYOUT), /g1_room_model|gearbox_room_back_wall/);
assert.notStrictEqual(SO101_HOME_LAB_LAYOUT, SO101_GEARBOX_LAYOUT);
assert.match(configSource, /so101HomeLab:[\s\S]*label: 'SO101 Home Lab'/);
```

- [x] **Step 2: Verify RED**

Run:

```bash
node --test test/collaborative-scene-layouts.test.mjs test/collaborative-scene-integration.test.mjs
```

Expected: FAIL because `SO101_HOME_LAB_LAYOUT` and `so101HomeLab` do not exist,
and the compact layout still contains the expanded room.

- [x] **Step 3: Implement the split and registrations**

Restore `SO101_GEARBOX_LAYOUT` to the `93e6333` compact worldbody/floor/camera.
Create `SO101_HOME_LAB_LAYOUT` using shared gearbox metadata and local G1/Go2
asset injections. Register `so101HomeLab` with the same four SO101 targets and
add it to every diagnostic/verification scene map.

- [x] **Step 4: Verify GREEN**

Run the Step 2 command and require all split/registration tests to pass.

### Task 2: Detailed Home Lab environment

**Files:**
- Create: `src/so101HomeLabEnvironment.js`
- Modify: `src/collaborativeSceneLayouts.js`
- Modify: `test/collaborative-scene-layouts.test.mjs`

**Interfaces:**
- Produces: `SO101_HOME_LAB_ROOM_XML` and `SO101_HOME_LAB_STATIC_ROBOTS_XML`.
- Consumes: those exports from `SO101_HOME_LAB_LAYOUT.xmlPatches`.

- [x] **Step 1: Write failing zoning/detail tests**

Require the 5 m × 4.2 m half-bounds, 1.6 m protected radius, lounge/office/G1/
Go2 zone centers outside that radius, and representative named details:

```js
for (const name of [
  'home_lab_coffee_table', 'home_lab_sofa_seam_1', 'home_lab_tv_left_speaker',
  'home_lab_mouse', 'home_lab_pc_tower', 'home_lab_cable_tray',
  'home_lab_g1_display_pad', 'home_lab_go2_charging_pad',
  'home_lab_maintenance_cabinet', 'home_lab_tool_board',
  'home_lab_west_window', 'home_lab_east_door',
]) assert.match(homeXml, new RegExp(`name="${name}"`));
```

- [x] **Step 2: Verify RED**

Run:

```bash
node --test --test-name-pattern='Home Lab' test/collaborative-scene-layouts.test.mjs
```

Expected: FAIL because the new environment module and detailed names are absent.

- [x] **Step 3: Build the environment**

Implement explicit fixed MJCF bodies for the three-wall envelope, windows and
door treatment, detailed west lounge, northeast office, southeast robot
gallery/maintenance zone, warm light fixtures, and circulation markings. Place
the G1 at `[2.45, -0.9]` with yaw `155` and Go2 at `[3.55, -2.35]` with yaw
`150`; place all large furniture outside the protected 1.6 m radius.

- [x] **Step 4: Verify GREEN and strict MJCF**

Run:

```bash
node --test --test-name-pattern='Home Lab' test/collaborative-scene-layouts.test.mjs
INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs so101HomeLab public/assets/so101-gearbox-room
```

Expected: tests pass, 24 actuators compile, and no contact is deeper than 5 mm.

### Task 3: Compact and Home Lab browser/control verification

**Files:**
- Replace: `artifacts/screenshots/so101-gearbox.png`
- Create: `artifacts/screenshots/so101-home-lab.png`

**Interfaces:**
- Consumes: runtime keys `so101Gearbox` and `so101HomeLab`.
- Produces: two 1440 × 900 visual checkpoints and per-arm keyboard/IK evidence.

- [x] **Step 1: Compile both scenes offline**

```bash
INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs so101Gearbox public/assets/so101-gearbox-room
INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1 node scripts/validate-mjcf.mjs so101HomeLab public/assets/so101-gearbox-room
```

- [x] **Step 2: Capture both scenes**

Run a local Vite server and then:

```bash
SCENES=so101Gearbox,so101HomeLab SCENE_URL=http://127.0.0.1:3015 node scripts/capture-scenes.mjs
```

Inspect the compact scene for absence of furnishings and Home Lab for clear
central circulation, unobscured SO101 arms, readable furniture details, stable
G1/Go2 placement, and no wall clipping.

- [x] **Step 3: Verify all eight control blocks**

```bash
SCENES=so101Gearbox,so101HomeLab SCENE_URL=http://127.0.0.1:3015 node scripts/verify-controls.mjs
```

Expected: four keyboard+IK PASS lines per scene.

### Task 4: Full regression and progress record

**Files:**
- Create: `docs/progress/2026-08-13-so101-gearbox-home-lab-split.md`
- Modify: `docs/superpowers/plans/2026-08-13-so101-gearbox-home-lab-split.md`

**Interfaces:**
- Produces: exact scene counts, contact results, control results, screenshot
  paths, asset policy, and implementation status.

- [x] **Step 1: Run the full verification gate**

```bash
git diff --check
npm test
npx tsc --noEmit
npm run build
```

If system Node is below Vite's required version, use the existing local crypto
compatibility bootstrap without changing project dependencies and require the
Vite process to exit successfully.

- [x] **Step 2: Record exact evidence**

Write the final body/geom/actuator/qpos counts and penetration results for both
scenes, all control results, build/test totals, screenshots, and confirmation
that no G1/Go2 controls or automatic attachment were added.

- [x] **Step 3: Mark completed plan items**

Change each executed checkbox in this document from `[ ]` to `[x]` only after
its command has produced the required evidence.
