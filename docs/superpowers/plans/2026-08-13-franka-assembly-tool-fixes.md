# Franka Assembly Tool Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the Assembly1 screwdriver from moving at rest, correct the procedural claw hammer topology, and restore visibly varied tool textures in Assembly2.

**Architecture:** Keep the existing two scene layouts and all station coordinates. Separate the Assembly1 screwdriver's detailed visual mesh from a flat stable collision proxy; rebuild only the Assembly1 hammer geoms around a conventional handle/head/claw axis. Because the current `mujoco-react` renderer ignores MuJoCo texture IDs and mesh UV buffers, bake each RoboTwin texture into three mesh partitions (`primary`, `dark`, and `metal`) and render those partitions with distinct MJCF materials.

**Tech Stack:** MuJoCo MJCF/WASM, JavaScript layout generation, Python standard-library GLB conversion, Node test runner, Playwright.

## Global Constraints

- Modify only the Assembly1 screwdriver, Assembly1 hammer, and Assembly2 screwdriver/drill/hammer appearances.
- Preserve all Panda, platform, frame, parts, and task-station coordinates.
- Keep every tool as one movable free body.
- Keep the RoboTwin visual meshes non-colliding and retain primitive collision proxies.

---

### Task 1: Stable Assembly1 screwdriver

**Files:**
- Modify: `test/franka-assembly-layouts.test.mjs`
- Modify: `src/frankaAssemblyLayouts.js`

**Interfaces:**
- Consumes: `FRANKA_ASSEMBLY1_LAYOUT.xmlPatches`.
- Produces: a visual-only `manual_screwdriver_handle` and a flat `manual_screwdriver_handle_collision` geom on a damped free body.

- [x] Add a failing structural test that requires the visual mesh to have contacts disabled, an invisible box collision proxy, and free-joint damping.
- [x] Run `node --test test/franka-assembly-layouts.test.mjs` and confirm the new assertion fails.
- [x] Add the collision proxy and damping without changing the octagonal visual mesh.
- [x] Re-run the focused test and offline MuJoCo compilation.

### Task 2: Correct Assembly1 claw hammer

**Files:**
- Modify: `test/franka-assembly-layouts.test.mjs`
- Modify: `src/frankaAssemblyLayouts.js`

**Interfaces:**
- Consumes: the existing `claw_hammer` free body and shelf.
- Produces: a longitudinal handle terminating at a transverse head, a round striking face on one side, and two rearward/downward split claw capsules on the opposite side.

- [x] Add failing assertions for `hammer_eye`, `hammer_cheek`, `hammer_striking_face`, and two capsule claws expressed with explicit `fromto` endpoints.
- [x] Run the focused test and confirm failure.
- [x] Replace only the hammer's procedural geoms, preserving its world pose and shelf.
- [x] Re-run the focused test and inspect a new Assembly1 screenshot.

### Task 3: Restore Assembly2 texture variation

**Files:**
- Modify: `scripts/convert-robotwin-tools.py`
- Modify: `src/frankaAssemblyLayouts.js`
- Regenerate: `public/assets/franka-assembly2/tools/*.obj`
- Regenerate: `public/assets/franka-assembly2/tools/*.png`
- Modify: `docs/progress/2026-08-13-dual-franka-assembly-detail.md`

**Interfaces:**
- Consumes: RoboTwin GLB `TEXCOORD_0`, indices, material texture references, and embedded PNGs.
- Produces: three palette-baked OBJ partitions per tool whose distinct MJCF materials preserve the primary, dark, and metallic regions of the source texture in the current web renderer.

- [x] Add a layout/asset test requiring non-empty `primary`, `dark`, and `metal` OBJ partitions and three distinct materials for all three external tools.
- [x] Run the test and confirm it fails because the palette-baked OBJ partitions do not exist.
- [x] Decode the embedded RGB PNG with the standard library, classify each source triangle from its UV centroid, and export three reindexed OBJ partitions per tool.
- [x] Replace each single visual geom with three colored non-colliding mesh geoms and verify every partition is non-empty.
- [x] Compile Assembly2 offline and inspect a new Assembly2 screenshot.

### Task 4: Regression, documentation, and publication

**Files:**
- Modify: `artifacts/screenshots/franka-assembly1.png`
- Modify: `artifacts/screenshots/franka-assembly2.png`
- Modify: `docs/progress/2026-08-13-dual-franka-assembly-detail.md`

- [x] Run all Node tests and TypeScript checking.
- [x] Run production and Pages builds.
- [x] Capture only Assembly1 and Assembly2 and inspect both images.
- [x] Run all eight Assembly1/2 keyboard and IK control-isolation checks.
- [x] Record root causes and evidence in the progress log.
- [ ] Commit, push `main` through `github-Shuaijun-LIU`, and monitor Pages CI.
