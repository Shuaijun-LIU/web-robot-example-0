# Assembly1 Step 3 Cross-Member Placement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a third Assembly1 action that physically lifts the cross-member with Arms 3/4, transfers it into the frame, verifies all four installation interfaces, and holds the aligned state.

**Architecture:** A pure `assemblyStep3.js` module owns waypoints, timing, limits, interpolation, the phase reducer, and pure contact/alignment verdicts. `AssemblyStep3Controller.tsx` resolves live MuJoCo resources, captures stationary-arm targets, executes only actuator commands, samples contact and site evidence, and exposes diagnostics. `App.tsx` owns Step 3 request/state and `AssemblySequencePanel.tsx` renders the third action without embedding simulation behavior.

**Tech Stack:** React 19, TypeScript 5.7, Vite 7, `mujoco-react`, `mujoco-js`, Three.js, Node.js built-in test runner, Playwright.

## Global Constraints

- Work only in `Franka Assembly1`; do not alter Assembly2 geometry or behavior.
- Step 3 starts only from Step 2 `complete` and ends with Arms 3/4 still gripping the aligned cross-member.
- Arm 1 holds the frame; Arm 2 holds the torque driver; neither receives a new motion trajectory.
- Cross-member motion must result only from physical finger contact and Panda `data.ctrl` commands.
- Never write task-object `qpos`/`qvel`, apply task-object forces, create welds, or implement magnet/proximity/scripted-follow behavior.
- Final cross-member body target is `[0, 0, 0.278]`; final compensated transport TCPs are `[0.006, 0.1275, 0.278]` and `[0.002, -0.1275, 0.278]`.
- Transport height is `0.34 m`; horizontal transfer is split into two generated IK segments.
- Completion requires four hole-to-receiver distances `<= 0.008 m`, frame drift `<= 0.008 m`, cross-member rotation `<= 5°`, bilateral contact on every active grasp, and a stable `1 s` final hold.
- Reset is the only retry path after an error.

---

### Task 1: Pure Step 3 Contract and Phase Machine

**Files:**
- Create: `test/assembly-step3.test.mjs`
- Create: `src/assemblyStep3.js`
- Create: `src/assemblyStep3.d.ts`

**Interfaces:**
- Consumes: `interpolateJointTargets()` and Step 2 clamp commands.
- Produces: `ASSEMBLY1_STEP3_WAYPOINTS`, `ASSEMBLY1_STEP3_DURATIONS`, `ASSEMBLY1_STEP3_LIMITS`, `createAssemblyStep3Machine()`, `advanceAssemblyStep3Machine()`, `createAssemblyStep3ControlFrame()`, `evaluateAssemblyStep3Transport()`, and `evaluateAssemblyStep3Alignment()`.

- [x] Write failing tests for exact world waypoints, phase ordering, synchronized Arm 3/4 progress, unchanged Arm 1/2 targets, retained gripper commands, bilateral-contact rejection, and four-site alignment thresholds.
- [x] Run `node --test test/assembly-step3.test.mjs`; verify failure is caused by the missing Step 3 module.
- [x] Implement the smallest pure module and declarations that satisfy those contracts.
- [x] Re-run the focused test and keep it green before integration work.

### Task 2: Generate Panda Transport Targets

**Files:**
- Create: `scripts/solve-assembly-step3-waypoints.mjs`
- Modify: `src/assemblyStep3.js`
- Modify: `src/assemblyStep3.d.ts`
- Modify: `package.json`

**Interfaces:**
- Consumes: Step 2 Arm 3/4 contact joint targets, Panda attachment frames, the selected-IK solver, and the Step 3 world TCP waypoints.
- Produces: finite, joint-range-safe seven-value targets for lift, transfer midpoint, hover, and aligned descent for both transport arms.

- [x] Extend the focused test to require every transport waypoint to contain two complete seven-joint arrays.
- [x] Run the test and observe the missing-array failure.
- [x] Implement the offline solver, seeded sequentially from each previous waypoint, and forward-check position `<= 0.01 m` plus orientation `<= 5°`.
- [x] Run `ALLOW_UNRECORDED_IK=1 node scripts/solve-assembly-step3-waypoints.mjs`, copy the generated arrays into the pure contract, then rerun without the environment override to prove the checked-in arrays match.

### Task 3: Runtime Controller

**Files:**
- Create: `src/AssemblyStep3Controller.tsx`
- Modify: `test/assembly-step3.test.mjs`

**Interfaces:**
- Consumes: Step 2 completion state, shared ownership ref, live MuJoCo model/data, Step 3 pure phase/control helpers.
- Produces: Step 3 state callbacks and structured runtime diagnostics.

- [x] Add a failing source-contract test requiring actuator-only control, immutable entry snapshots for Arms 1/2, shared phase timing for Arms 3/4, and rejecting object `qpos`/`qvel`, weld, magnet, attachment, or scripted-follow patterns.
- [x] Run the focused test and observe the missing controller failure.
- [x] Implement model resolution, precondition validation, control ownership, contact/site sampling, phase advancement, safe failure hold, and diagnostics.
- [x] Re-run the focused test.

### Task 4: App and Three-Step UI Integration

**Files:**
- Modify: `src/App.tsx`
- Modify: `src/AssemblySequencePanel.tsx`
- Modify: `src/global.d.ts`
- Modify: `test/ui-contract.test.mjs`

**Interfaces:**
- Consumes: `AssemblyStep3Controller`, Step 2 `complete`, and shared assembly ownership.
- Produces: third-step request/button/status, `data-assembly-step3-status`, `window.robotDemo.runAssemblyStep3()`, and `getAssemblyStep3Diagnostics()`.

- [x] Add failing UI/source tests for the third button, Step 2 completion gate, automation lockout, Reset, dataset status, and browser diagnostics.
- [x] Run the UI and Step 3 tests and verify the new assertions fail.
- [x] Wire Step 3 state/request/diagnostics through `SceneChildren` and `App`, extend ownership to `'step3'`, and add concise Chinese phase/failure copy.
- [x] Re-run the focused tests.

### Task 5: Production Verification and Progress Record

**Files:**
- Create: `scripts/verify-assembly-step3.mjs`
- Modify: `package.json`
- Modify: `project/task_plan.md`
- Modify: `project/decision_log.md`
- Modify: `project/standup_log.md`
- Modify: `project/next_actions.md`

**Interfaces:**
- Consumes: production Vite scene and `window.robotDemo` diagnostics.
- Produces: measured acceptance result and `artifacts/screenshots/franka-assembly1-step3-aligned-hold.png`.

- [x] Add a verifier that runs Steps 1–3, checks the Step 3 gate, physical contact, vertical clearance before transfer, final four-site distances, frame drift, finite runtime values, scene readiness, and Reset.
- [x] Run the focused Step 3 tests, complete test suite, IK solver, TypeScript/Vite build, and production browser verifier.
- [x] Review the final diff against every global constraint and record measured results plus remaining Step 4 boundary in the project logs.

## Verification Result

- Two consecutive production-browser runs completed all three actions without a scene crash.
- Run 1: four hole distances `2.44–2.61 mm`, frame drift `4.47 mm`, cross-member rotation `0.66°`.
- Run 2: four hole distances `4.90–6.38 mm`, frame drift `4.88 mm`, cross-member rotation `0.30°`.
- The cross-member is moved only by the two physical Panda grasps. Step 3 ends with Arms 3/4 still clamped; fastening remains outside this step.
