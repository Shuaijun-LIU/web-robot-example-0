# Dual Franka Assembly Detail Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single assembly scene with `Franka Assembly1` and `Franka Assembly2`, sharing exact frame/cross-member interfaces while comparing procedural physical tools against converted RoboTwin tool meshes.

**Architecture:** Move assembly-specific MJCF generation into `src/frankaAssemblyLayouts.js`. It exports a numeric installation contract and two layouts: Assembly1 injects procedural tool geometry; Assembly2 references local converted OBJ meshes plus stable collision proxies. Both layouts reuse the existing four-Panda controllers and the same shared frame, parts, trays, platform, and camera.

**Tech Stack:** React 19, TypeScript, Vite public assets, MuJoCo MJCF/WASM, Node test runner, Playwright, Python 3 asset conversion with `trimesh`.

## Global Constraints

- Preserve `Franka Panda`, `SO101`, and `XLeRobot` behavior and labels.
- Expose exact selector labels `Franka Assembly1` and `Franka Assembly2`.
- Keep both assembly scenes static; add no scripted task motion.
- Keep both scenes on the same `0.90 m` four-Panda ring and identical station coordinates.
- Represent every visible task object with MuJoCo physics and avoid initial penetration.
- Make all four cross-member hole sites coincide with their paired frame receiver sites at the documented target pose.
- Commit only selected converted tool assets, not the RoboTwin dataset or unused variants.

---

### Task 1: Dual-layout and interface contract

**Files:**
- Create: `test/franka-assembly-layouts.test.mjs`
- Modify: `test/scene-layouts.test.mjs`
- Modify: `test/deployment-contract.test.mjs`

**Interfaces:**
- Consumes: existing `FRANKA_ASSEMBLY_LAYOUT` behavior and five-scene design specification.
- Produces: required exports `FRANKA_ASSEMBLY1_LAYOUT`, `FRANKA_ASSEMBLY2_LAYOUT`, `FRANKA_ASSEMBLY_INTERFACE`, and `applyAssemblyTargetPose(points, targetPose)`.

- [ ] **Step 1: Write the failing layout tests.** Require two four-instance layouts with `ringRadius === 0.9`; require Assembly1 XML names `manual_screwdriver_octagonal_handle`, `torque_driver_trigger`, and `claw_hammer`; require Assembly2 XML mesh names `robotwin_screwdriver_visual`, `robotwin_drill_visual`, and `robotwin_hammer_visual`.

```js
assert.equal(FRANKA_ASSEMBLY1_LAYOUT.instanceCount, 4);
assert.equal(FRANKA_ASSEMBLY2_LAYOUT.instanceCount, 4);
assert.equal(FRANKA_ASSEMBLY1_LAYOUT.ringRadius, 0.9);
assert.match(xml(FRANKA_ASSEMBLY1_LAYOUT), /manual_screwdriver_octagonal_handle/);
assert.match(xml(FRANKA_ASSEMBLY1_LAYOUT), /torque_driver_trigger/);
assert.match(xml(FRANKA_ASSEMBLY1_LAYOUT), /name="claw_hammer"/);
assert.match(xml(FRANKA_ASSEMBLY2_LAYOUT), /mesh="robotwin_screwdriver_visual"/);
assert.match(xml(FRANKA_ASSEMBLY2_LAYOUT), /mesh="robotwin_drill_visual"/);
assert.match(xml(FRANKA_ASSEMBLY2_LAYOUT), /mesh="robotwin_hammer_visual"/);
```
- [ ] **Step 2: Write the failing numeric alignment test.** Use literal receiver coordinates `[-0.04, 0.215, 0.275]`, `[0.04, 0.215, 0.275]`, `[-0.04, -0.215, 0.275]`, and `[0.04, -0.215, 0.275]`; transform the four cross-member local sites by target pose `[0, 0, 0.235]`; require exact coordinate equality to `1e-9`.

```js
const expected = [
  [-0.04, 0.215, 0.275], [0.04, 0.215, 0.275],
  [-0.04, -0.215, 0.275], [0.04, -0.215, 0.275],
];
const actual = applyAssemblyTargetPose(
  FRANKA_ASSEMBLY_INTERFACE.crossMemberHoleLocalPositions,
  FRANKA_ASSEMBLY_INTERFACE.crossMemberTargetPose,
);
actual.forEach((point, i) => point.forEach((value, axis) => {
  assert.ok(Math.abs(value - expected[i][axis]) < 1e-9);
}));
```
- [ ] **Step 3: Run `node --test test/franka-assembly-layouts.test.mjs`.** Expected result: failure because the new module and exports do not exist.
- [ ] **Step 4: Update the existing deployment expectation.** Require both `artifacts/screenshots/franka-assembly1.png` and `artifacts/screenshots/franka-assembly2.png` and five verified scenes.
- [ ] **Step 5: Commit red tests.** Use `test: define dual assembly detail contract`.

### Task 2: Shared frame interfaces and Assembly1 procedural tools

**Files:**
- Create: `src/frankaAssemblyLayouts.js`
- Create: `src/frankaAssemblyLayouts.d.ts`
- Modify: `src/sceneLayouts.js`
- Modify: `src/sceneLayouts.d.ts`
- Modify: `src/configs.ts`
- Modify: `src/App.tsx`

**Interfaces:**
- Consumes: exported `FRANKA_HOME`, `repeatPose`, `fixedBox`, and four-arm control targets.
- Produces: `FRANKA_ASSEMBLY_INTERFACE`, `applyAssemblyTargetPose`, `FRANKA_ASSEMBLY1_LAYOUT`, and the `frankaAssembly1` runtime entry.

- [ ] **Step 1: Export shared Franka primitives.** Export `FRANKA_HOME` and `fixedBox` from `src/sceneLayouts.js` and declare them in `src/sceneLayouts.d.ts`; remove the old monolithic assembly export after the new module is ready.

```js
export const FRANKA_HOME = [1.707, -1.754, 0.003, -2.702, 0.003, 0.951, 2.49, 0];
export const fixedBox = (name, size, position, rgba) => ({
  name, type: 'box', size, position, rgba,
});
```
- [ ] **Step 2: Implement the installation contract.** Define the four receiver coordinates, four cross-member local hole coordinates at `z=0.04`, and target pose `[0, 0, 0.235]`; implement `applyAssemblyTargetPose` as literal point-wise translation.

```js
export const FRANKA_ASSEMBLY_INTERFACE = {
  crossMemberTargetPose: [0, 0, 0.235],
  frameReceiverPositions: [
    [-0.04, 0.215, 0.275], [0.04, 0.215, 0.275],
    [-0.04, -0.215, 0.275], [0.04, -0.215, 0.275],
  ],
  crossMemberHoleLocalPositions: [
    [-0.04, 0.215, 0.04], [0.04, 0.215, 0.04],
    [-0.04, -0.215, 0.04], [0.04, -0.215, 0.04],
  ],
};
export const applyAssemblyTargetPose = (points, target) =>
  points.map((point) => point.map((value, axis) => value + target[axis]));
```
- [ ] **Step 3: Build the shared detailed frame XML.** Replace each solid rail with paired flanges, recessed slot strips, gussets, receiver recesses, and named receiver sites. Build perforated cross-member end tabs from strips around four true gaps and add paired hole sites.

```xml
<geom name="frame_rail_north_outer" type="box" pos="0 0.244 0" size="0.34 0.009 0.025"/>
<geom name="frame_rail_north_inner" type="box" pos="0 0.216 0" size="0.34 0.009 0.025"/>
<geom name="frame_rail_north_slot" type="box" pos="0 0.23 0.021" size="0.34 0.005 0.003" contype="0" conaffinity="0"/>
<site name="frame_receiver_nw" pos="-0.04 0.215 0.04" size="0.006"/>
```
- [ ] **Step 4: Build the octagonal grooved handle.** Generate one inline mesh with eight vertices per axial ring, alternating radii `[0.026, 0.023, 0.026, 0.023, 0.026]`, triangulated side faces and closed caps; use it for the free screwdriver body.

```js
const rings = [
  [-0.09, 0.026], [-0.06, 0.026], [-0.055, 0.023],
  [-0.02, 0.026], [-0.015, 0.023], [0.02, 0.026], [0.035, 0.023],
];
const vertices = rings.flatMap(([x, radius]) =>
  Array.from({ length: 8 }, (_, i) => [x, radius * Math.cos(i * Math.PI / 4), radius * Math.sin(i * Math.PI / 4)]));
```
- [ ] **Step 5: Build the detailed cordless driver.** Add named housing, gearbox, selector, chuck, bit, angled grip, trigger, vents, and battery geoms with the battery bottom at or above the tool mat top.

```xml
<geom name="torque_driver_housing" type="capsule" fromto="-0.055 0 0.08 0.055 0 0.08" size="0.042"/>
<geom name="torque_driver_selector" type="cylinder" fromto="-0.08 0 0.08 -0.055 0 0.08" size="0.025"/>
<geom name="torque_driver_grip" type="capsule" fromto="0.02 0 0.065 0.045 0 -0.025" size="0.022"/>
<geom name="torque_driver_trigger" type="box" pos="-0.006 0 0.035" size="0.012 0.026 0.012"/>
```
- [ ] **Step 6: Build the procedural claw hammer.** Add a horizontal free body with striking face, cheek, split rear claws, and faceted handle on a new hammer mat.

```xml
<body name="claw_hammer" pos="0.65 0 0.145" euler="0 0 90">
  <freejoint/>
  <geom name="hammer_handle" type="box" pos="0 -0.11 0" size="0.018 0.14 0.014"/>
  <geom name="hammer_face" type="cylinder" fromto="-0.07 0 0 -0.025 0 0" size="0.027"/>
  <geom name="hammer_claw_left" type="capsule" fromto="0.035 0.012 0 0.10 0.026 -0.018" size="0.009"/>
  <geom name="hammer_claw_right" type="capsule" fromto="0.035 -0.012 0 0.10 -0.026 -0.018" size="0.009"/>
</body>
```
- [ ] **Step 7: Register `Franka Assembly1`.** Route it through `controlFamily: 'franka'`, four existing Franka control targets, and `frankaAssembly1` root diagnostics.

```ts
frankaAssembly1: {
  label: 'Franka Assembly1',
  controlFamily: 'franka',
  config: createAssemblyConfig(FRANKA_ASSEMBLY1_LAYOUT, FRANKA_REMOTE_BASE),
  controlTargets: createFrankaTargets(),
}
```
- [ ] **Step 8: Run the focused tests and TypeScript.** Run `node --test test/franka-assembly-layouts.test.mjs test/scene-layouts.test.mjs` and `npx tsc --noEmit`; expected: Assembly1 and alignment tests pass while Assembly2 asset expectations remain pending only until Task 3 implementation.
- [ ] **Step 9: Commit.** Use `feat: refine procedural assembly workcell`.

### Task 3: RoboTwin conversion and Assembly2 mesh integration

**Files:**
- Create: `scripts/convert-robotwin-tools.py`
- Create: `public/assets/franka-assembly2/THIRD_PARTY_NOTICES.md`
- Create: `public/assets/franka-assembly2/tools/screwdriver_visual.obj`
- Create: `public/assets/franka-assembly2/tools/screwdriver_collision.obj`
- Create: `public/assets/franka-assembly2/tools/drill_visual.obj`
- Create: `public/assets/franka-assembly2/tools/drill_collision.obj`
- Create: `public/assets/franka-assembly2/tools/hammer_visual.obj`
- Create: `public/assets/franka-assembly2/tools/hammer_collision.obj`
- Copy selected required Franka XML/mesh files to: `public/assets/franka-assembly2/`
- Modify: `src/frankaAssemblyLayouts.js`
- Modify: `src/configs.ts`

**Interfaces:**
- Consumes: local RoboTwin object directories `032_screwdriver`, `030_drill` variant `base6`, and `020_hammer`, plus MuJoCo Menagerie Franka files.
- Produces: a Vite-served local asset root and `FRANKA_ASSEMBLY2_LAYOUT` referencing OBJ meshes and stable physical proxies.

- [ ] **Step 1: Implement a deterministic GLB-to-OBJ converter.** `scripts/convert-robotwin-tools.py` accepts `--robotwin-root`, `--output`, loads the exact visual/collision GLBs with `trimesh`, concatenates scene geometry after applying node transforms, and exports named OBJ files without modifying scale.

```py
def convert(source: Path, destination: Path) -> None:
    loaded = trimesh.load(source, force="scene", process=False)
    meshes = [geometry.copy().apply_transform(transform)
              for _name, geometry, transform in loaded.graph.to_flattened()]
    merged = trimesh.util.concatenate(meshes)
    destination.write_bytes(trimesh.exchange.obj.export_obj(merged).encode("utf-8"))
```
- [ ] **Step 2: Convert only six selected meshes.** Use visual/collision `base0` for screwdriver, visual `base6` plus collision `base6` for drill, and visual/collision `base0` for hammer.

```bash
python3 scripts/convert-robotwin-tools.py \
  --robotwin-root /data/private/user2/workspace/benchmarks/RoboTwin/assets/objects \
  --output public/assets/franka-assembly2/tools
```
- [ ] **Step 3: Copy the 36 MB Franka model directory into the Assembly2 public root.** Exclude `.git`, caches, and unrelated menagerie models.

```bash
rsync -a --delete --exclude '.git' --exclude '__pycache__' \
  /data/private/user2/workspace/7.web-robot/1.source-repos/google-deepmind__mujoco_menagerie/franka_emika_panda/ \
  public/assets/franka-assembly2/
```
- [ ] **Step 4: Add attribution.** Record RoboTwin repository, MIT license notice, exact source object paths/variants, conversion method, and MuJoCo Menagerie source/license in `THIRD_PARTY_NOTICES.md`.
- [ ] **Step 5: Inject external mesh assets.** Define six `<mesh>` assets with source scales `0.095`, `0.08`, and `0.079`; Assembly2 tool bodies use non-colliding visual mesh geoms and colliding converted meshes or conservative box/capsule proxies when offline compilation rejects a source collision mesh.

```xml
<mesh name="robotwin_screwdriver_visual" file="tools/screwdriver_visual.obj" scale="0.095 0.095 0.095"/>
<mesh name="robotwin_drill_visual" file="tools/drill_visual.obj" scale="0.08 0.08 0.08"/>
<mesh name="robotwin_hammer_visual" file="tools/hammer_visual.obj" scale="0.079 0.079 0.079"/>
```
- [ ] **Step 6: Register `Franka Assembly2`.** Set its `src` to `${import.meta.env.BASE_URL}assets/franka-assembly2/`, reuse shared workcell coordinates/interfaces, and expose four Franka targets.

```ts
const ASSEMBLY2_BASE = `${import.meta.env.BASE_URL}assets/franka-assembly2/`;
frankaAssembly2: {
  label: 'Franka Assembly2',
  controlFamily: 'franka',
  config: createAssemblyConfig(FRANKA_ASSEMBLY2_LAYOUT, ASSEMBLY2_BASE),
  controlTargets: createFrankaTargets(),
}
```
- [ ] **Step 7: Run focused tests, TypeScript, and offline compilation.** Compile Assembly1 against Menagerie and Assembly2 against `public/assets/franka-assembly2`; both must report four Panda roots, 32 actuators, and all eight named interface sites.
- [ ] **Step 8: Commit.** Use `feat: add RoboTwin assembly comparison`.

### Task 4: Browser capture and stability verification

**Files:**
- Modify: `scripts/capture-scenes.mjs`
- Modify: `scripts/verify-controls.mjs`
- Modify: `scripts/validate-mjcf.mjs`
- Create: `artifacts/screenshots/franka-assembly1.png`
- Create: `artifacts/screenshots/franka-assembly2.png`

**Interfaces:**
- Consumes: the two runtime keys and existing browser lifecycle diagnostics.
- Produces: separate screenshots, physical root/body counts, independent four-arm control evidence, and settled tool/frame pose checks.

- [ ] **Step 1: Add both scenes to browser scripts.** Capture exact labels and filenames; verify controls for four targets in each scene.

```js
{ key: 'frankaAssembly1', label: 'Franka Assembly1', instances: 4, screenshotName: 'franka-assembly1' },
{ key: 'frankaAssembly2', label: 'Franka Assembly2', instances: 4, screenshotName: 'franka-assembly2' },
```
- [ ] **Step 2: Extend offline validation.** Accept `frankaAssembly1` and `frankaAssembly2`; under `ASSEMBLY_REPORT=1`, print the eight named site positions before and after placing the cross-member at its target pose.
- [ ] **Step 3: Capture both scenes at 1440×900.** Require ready state and four `rN_link0` roots in each.
- [ ] **Step 4: Inspect screenshots.** Confirm Assembly1 grooves/facets and procedural silhouettes, Assembly2 real mesh silhouettes, frame extrusion slots, open end-tab holes, receiver alignment, realistic scale/orientation, and no overlaps.
- [ ] **Step 5: Run all 18 control targets.** Verify 4 original Franka + 4 SO101 + 2 XLeRobot + 4 Assembly1 + 4 Assembly2 targets write only their selected actuator block.
- [ ] **Step 6: Commit.** Use `test: verify dual assembly workcells`.

### Task 5: Documentation, deterministic Pages build, and deployment

**Files:**
- Modify: `README.md`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`
- Modify: `project/artifacts.jsonl`
- Modify: `project/artifacts.md`
- Modify: `docs/index.html`
- Replace generated files under: `docs/assets/`
- Copy generated public assets under: `docs/assets/franka-assembly2/`

**Interfaces:**
- Consumes: verified screenshots, source assets, tests, compilation, and browser regression.
- Produces: a five-scene README comparison and GitHub Pages deployment from `main`.

- [ ] **Step 1: Document the comparison.** Embed both screenshots and describe which visual/physics strategy each scene uses, the shared interface contract, and RoboTwin attribution.
- [ ] **Step 2: Update project records.** Mark phases 6–9 complete and register both screenshots plus the Assembly2 asset notice.
- [ ] **Step 3: Run fresh final verification.** Run `npm test`, `npx tsc --noEmit`, offline compile for both scenes, `npm run build`, and two consecutive `npm run build:pages` commands with identical hashes.
- [ ] **Step 4: Commit generated deployment files.** Use `docs: publish dual assembly comparison` after deleting stale hashed bundles.
- [ ] **Step 5: Push `main` over `git@github-Shuaijun-LIU:Shuaijun-LIU/web-robot-example-0.git`.** Do not force-push.
- [ ] **Step 6: Verify deployment.** Require the matching-head GitHub Pages workflow to complete successfully, the live `index.html` hash to match local, and every referenced JS/CSS/OBJ asset to return HTTP 200.
