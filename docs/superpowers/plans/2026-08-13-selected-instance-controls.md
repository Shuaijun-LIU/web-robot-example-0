# Selected-Instance Multi-Robot Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the original interactive controls and allow every physical robot instance to be selected and controlled independently.

**Architecture:** Add pure target metadata and index-shifting helpers, then connect them to a local selection-aware IK hook and the existing keyboard controllers. Keep one active gizmo/controller target at a time, preserve scene-wide mouse interactions, and expose deterministic browser diagnostics.

**Tech Stack:** React 19, TypeScript, Leva, Three.js/R3F, `mujoco-react`, MuJoCo WASM, Node test runner, Playwright, Vite, GitHub Actions Pages.

## Global Constraints

- Robot labels are exactly `Franka Panda`, `SO101`, and `XLeRobot`.
- Franka and SO101 expose four independently controllable arms.
- XLeRobot exposes two independently controllable complete robots, each retaining two-arm/base/head keys.
- Only the selected Franka/SO101 target displays and drives one IK gizmo.
- Switching targets does not reset or move non-selected instances.
- Simulation starts running and the IK gizmo starts visible.
- Top-left statistics remain one scene-level FPS/time/memory group.
- Pages base remains `/web-robot-example-0/`; push is a normal SSH fast-forward to `origin/main`.

---

### Task 1: Pure control-target metadata

**Files:**
- Create: `src/controlTargets.js`
- Create: `src/controlTargets.d.ts`
- Modify: `src/configs.ts`
- Test: `test/control-targets.test.mjs`

**Interfaces:**
- Produces: `makeArmTargets(count, options)`, `shiftIndices(indices, offset)`, and target arrays on every `RobotEntry`.
- Consumes: existing `r0_` through `r3_` MJCF namespaces and fixed per-instance actuator block sizes.

- [ ] **Step 1: Write failing target tests**

Assert that Franka/SO101 targets have labels `Arm 1` through `Arm 4`, XLeRobot has `Robot 1` and `Robot 2`, namespaced TCP/gripper/joint names are correct, offsets are `0/8/16/24`, `0/6/12/18`, and `0/16`, and `shiftIndices([0,2,5], 6)` returns `[6,8,11]`.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `node --test test/control-targets.test.mjs`

Expected: FAIL because `src/controlTargets.js` does not exist.

- [ ] **Step 3: Implement metadata helpers and exact selector labels**

Create serializable target objects containing `key`, `label`, `prefix`, `actuatorOffset`, and optional `ik` fields with `siteName`, `jointNames`, `actuatorIndices`, and `qpos` resolution inputs. Extend `RobotEntry` with `controlTargets`. Restore the three original `label` strings.

- [ ] **Step 4: Run focused and aggregate tests**

Run: `npm test`

Expected: target tests and all prior tests PASS.

- [ ] **Step 5: Commit the metadata layer**

```bash
git add src/controlTargets.js src/controlTargets.d.ts src/configs.ts test/control-targets.test.mjs
git commit -m "feat: define selectable control targets"
```

### Task 2: Selection-aware IK and keyboard routing

**Files:**
- Create: `src/controllers/selectedIkSolver.ts`
- Create: `src/controllers/useSelectedIkController.ts`
- Modify: `src/controllers/FrankaController.tsx`
- Modify: `src/controllers/SO101Controller.tsx`
- Modify: `src/controllers/XLeRobotController.tsx`
- Modify: `src/controllers/useArmController.ts`
- Test: `test/control-targets.test.mjs`

**Interfaces:**
- Consumes: a selected target from Task 1.
- Produces: an `IkContextValue` compatible with `IkGizmo`, plus keyboard controllers whose actuator indices are shifted to the selected block.

- [ ] **Step 1: Add failing routing tests**

Assert that each controller accepts a target/offset, Franka derives the selected gripper name, SO101 shifts `[0,1,2,3,4,5]`, XLeRobot shifts all sixteen indices, and the selected IK implementation uses explicit `qposAddresses` and `actuatorIndices` instead of starting from zero.

- [ ] **Step 2: Run tests and verify RED**

Run: `npm test`

Expected: FAIL because controllers still hard-code the first instance.

- [ ] **Step 3: Implement the offset-capable DLS solver**

Adapt the existing six-dimensional damped least-squares algorithm to perturb only `qposAddresses[j]`, restore the full `qpos` array after every solve, and return one solution value per selected arm joint.

- [ ] **Step 4: Implement `useSelectedIkController(target)`**

Resolve the selected site and joint addresses after model readiness, synchronize one `THREE.Group` target to that site, solve in `useBeforePhysicsStep`, and write results only to the selected actuator indices. Resynchronize and disable stale IK when target or reset generation changes.

- [ ] **Step 5: Route keyboard controllers**

Make Franka's gripper name selection-aware. Shift SO101's six indices and XLeRobot's base, both arms, and head indices. Clear held keys and rebuild internal state on target changes.

- [ ] **Step 6: Verify tests and TypeScript**

Run: `npm test && npx tsc --noEmit`

Expected: all tests PASS and TypeScript exits zero.

- [ ] **Step 7: Commit controller routing**

```bash
git add src/controllers test/control-targets.test.mjs
git commit -m "feat: control selected robot instances"
```

### Task 3: Active-target UI and overlay polish

**Files:**
- Modify: `src/App.tsx`
- Modify: `src/KeyboardHelp.tsx`
- Modify: `src/styles.css`
- Test: `test/ui-contract.test.mjs`

**Interfaces:**
- Consumes: `entry.controlTargets` and `useSelectedIkController`.
- Produces: Leva `Control target`, active-target help text, scene-performance label, and browser data attributes.

- [ ] **Step 1: Replace the old UI assertions with failing desired behavior**

Assert exact robot labels without count suffixes, a dynamic `Control target` selector, default `paused: false`, default gizmo `true`, `dataset.controlTarget`, `dataset.ikSite`, a `Scene performance` label, and a `.keyboard-help` class with minimum width and padding.

- [ ] **Step 2: Run the UI test and verify RED**

Run: `node --test test/ui-contract.test.mjs`

Expected: FAIL on old defaults and missing selector/overlay behavior.

- [ ] **Step 3: Implement the target selector and controller wiring**

Reset selection to the first target when the robot scene changes. Pass the selected target and reset generation into scene children. Keep `DragInteraction` and click selection scene-wide. Update data attributes after target resolution.

- [ ] **Step 4: Implement overlay changes**

Wrap the three Stats panels with a visible `Scene performance` title. Move Keyboard sizing into `.keyboard-help` with at least `1rem` horizontal padding, `0.875rem` vertical padding, `min-width: 15rem`, and readable line height; display the active target under the Keyboard heading.

- [ ] **Step 5: Verify UI, full tests, and TypeScript**

Run: `npm test && npx tsc --noEmit`

Expected: all tests PASS and TypeScript exits zero.

- [ ] **Step 6: Commit UI integration**

```bash
git add src/App.tsx src/KeyboardHelp.tsx src/styles.css test/ui-contract.test.mjs
git commit -m "feat: select active robot controls"
```

### Task 4: Browser interaction verification and documentation

**Files:**
- Modify: `scripts/capture-scenes.mjs`
- Create: `scripts/verify-controls.mjs`
- Modify: `package.json`
- Modify: `README.md`
- Modify: `artifacts/screenshots/*.png`

**Interfaces:**
- Consumes: document target diagnostics and live MuJoCo API state.
- Produces: repeatable verification for all ten selectable target states and refreshed visual artifacts.

- [ ] **Step 1: Add browser control verification**

For every Franka/SO101 arm and both XLeRobots, select the target, confirm its resolved namespace, send a target-specific key, and assert only the expected actuator block changes. Move the selected Franka/SO101 IK target and assert only that arm block responds.

- [ ] **Step 2: Run the browser check and diagnose any failure**

Run Vite with Node 22 and local asset routing, then run `npm run verify:controls`.

Expected: all target rows report PASS with no page, request, XML, or simulation error.

- [ ] **Step 3: Refresh screenshots and inspect overlays**

Run `npm run capture:scenes`. Inspect all PNGs for one active gizmo, clean selector labels, visible performance scope, larger keyboard background, and unchanged valid physical layouts.

- [ ] **Step 4: Update documentation**

Document target selection, per-instance routing, scene-wide mouse behavior, global performance statistics, and verification commands. Remove the old primary-instance limitation.

- [ ] **Step 5: Commit verified behavior**

```bash
git add scripts package.json package-lock.json README.md artifacts/screenshots
git commit -m "test: verify selected-instance controls"
```

### Task 5: Pages build and GitHub deployment

**Files:**
- Modify: `docs/index.html`
- Modify: `docs/assets/*`

**Interfaces:**
- Consumes: the verified source tree.
- Produces: deployed `origin/main` and a successful GitHub Pages run.

- [ ] **Step 1: Run final verification**

Run: `npm test && npx tsc --noEmit && npm run build && npm run build:pages && git diff --check`

Expected: every command exits zero; Pages asset URLs use `/web-robot-example-0/`.

- [ ] **Step 2: Commit Pages output**

```bash
git add docs
git commit -m "docs: publish selectable robot controls"
```

- [ ] **Step 3: Confirm fast-forward safety and push**

Run `git fetch origin --prune`, confirm `origin/main...main` has zero commits on the remote-only side, then run `git push origin main` through `github-Shuaijun-LIU`. Never force-push.

- [ ] **Step 4: Verify deployment**

Wait for the custom Pages workflow to succeed, fetch the live page and hashed assets, and compare the live `index.html` hash with the local Pages build.
