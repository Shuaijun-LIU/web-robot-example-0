# Demo2: focused optimization and next-scene candidates

## Scope

User request: another optimization round, plus 2–3 suggestions for subsequent scenes. This round preserves the accepted four-arm task. It does not build a new scene or publish changes without a separate push request.

Unchanged: all models and collision meshes, object masses, gripper forces, motion assets, reservation timing, contact thresholds, 2 ms physics/control step, Demo1 / Assembly1 and the default page.

## Changes

- Contact classification now looks up only the current phase, instead of unnecessarily interpolating joint targets a second time. Actuator interpolation is unchanged. Tests compare both lookups at every phase boundary, including the source-lease epsilon.
- Progress snapshots are immutable and reused while their content is unchanged. React publishes on identity changes; it no longer serializes progress to JSON at every physics tick. Physical observations, safety checks and diagnostic updates still run every tick.
- Demo2 has a narrower, collapsible panel with total progress and four per-arm cards. Shared-path waiting and the existing simulation pause are explained explicitly. The original single-egg / correction checks remain available. No decorative labels were added to physical objects.
- Actual-page validation exposed an existing development-only clock issue: Vite imports `mujoco-react/dist/index.js?v=...`, while the fixed-control plugin matched only paths ending in `.js`. The served development module therefore lacked the fixed control clock even though the production bundle contained it. Normalize the cache query before matching. This enables the already-intended fixed clock in development; it does not change the clock algorithm, timestep or production motion.

## Verification

- Red tests reproduced unnecessary state rebuilding in real two-arm WASM playback and the missing compact view in the actual page before implementation.
- Stage-boundary and existing coordination tests: 9 passing.
- Initial full suite: 237 passing; TypeScript and production build pass. The first actual-page pause/resume run stopped on `unexpected-collision` during transfer; investigation found the development module was unpatched as described above. A new regression fails on the original query-ID matcher and passes on the fix. Re-fetching the served module confirms both physical stepping branches now use the fixed clock.
- Final suite: **238/238 passing**, including complete two-arm and four-arm physical WASM replay, all 16 eggs sorted and all arms home, and immutable/stable UI snapshot assertions throughout playback. Fresh WASM reports match the checked-in baseline; no motion/model assets changed.
- Final TypeScript check and production build pass. The build retains existing large-chunk / MuJoCo browser-module warnings; they are not failures and bundle restructuring is outside this round.
- Actual development-page check passes panel expand/collapse, total progress, 800×600 bounds, pause/resume, 16.078 simulated seconds of physical startup and Reset. At that checkpoint four arms have moved simultaneously, forbidden penetration is zero and browser errors are empty. This is a startup/interaction check, **not** a new full-length rendered replay; the complete 16-egg run is covered by the separate WASM regression above.
- [Browser check report](../../artifacts/reports/demo2-polish-browser-2026-10-07.json) and [actual paused inspection screenshot](../../artifacts/screenshots/demo2-polish-2026-10-07.png). Rendering used software mode; the screenshot's FPS is not a measurement of hardware-accelerated browser performance.
- Independent read-only review found no critical or important issues in the runtime/UI changes or the cache-query fix.
- A local stage-lookup microbenchmark (52,800 calls per pass, median of five passes) measured 83.00 ms for interpolating targets versus 0.88 ms for stage-only lookup. This measures one small helper, **not** total simulation speed or browser FPS.

## Suggested next scenes — designs, not implemented features

These continue the existing browser/MuJoCo direction. Benchmark tasks supply reusable models and contact-point references; their original controllers are not assumed portable to four Panda arms. Check model-specific redistribution terms and visual/collision alignment before importing.

### 1. Cooperative pot handling and ingredient loading — recommended for a distinct demo

- Arm 1 and Arm 2 grasp the two handles, lift together and bring the pot to the loading position while keeping it level.
- Arm 3 and Arm 4 pick solid ingredients from separate trays, take turns entering the pot opening, and release inside it.
- The loading arms withdraw before Arms 1/2 lower the filled pot onto its destination support, release and return home.
- Goal: all specified ingredients contained, pot level and supported, no spilled objects, all arms clear.
- Coordination: load sharing, maintaining a common object pose, hand-clearance through a shared opening and phase-dependent access. First version uses solid ingredients only, not fluid simulation or cutting.
- Local assets confirmed in `workspace/benchmarks/RoboTwin/assets/objects`: `060_kitchenpot/100038/mobility.urdf` with sourced meshes and lid/body structure; `069_vagetable`, `103_fruit`, `008_tray`. The pot/lid opening and handle contact geometry need an import feasibility check before layout design.
- Reference: [RoboTwin Lift Pot](https://robotwin-platform.github.io/doc/tasks/lift_pot.html). The four-arm loading sequence above is our proposed extension, not an existing verified benchmark task.

### 2. Product presentation, scanning and order packing — recommended for a clear workflow

- Arm 1 picks a product box and rotates its label toward a fixed inspection region.
- Arm 2 brings a handheld scanner into the correct viewing direction and standoff, then withdraws.
- Arm 1 places the inspected item on a shared handoff pad; Arm 3 moves it into the assigned order tray.
- Arm 4 prepares an empty order tray and moves completed trays to dispatch, waiting until Arm 3 is clear.
- Goal: every required item passes the inspection gate and reaches its assigned tray, with no missing or duplicated items.
- Coordination: relative tool/object pose, a physical handoff pad and staged ownership of packing/dispatch space. The first version should expose inspection as a geometric simulation gate; do not claim actual barcode decoding unless it is separately implemented from rendered camera images.
- Local assets confirmed: `024_scanner/visual/base0.glb`, `112_tea-box/visual/base0.glb`, plus `113_coffee-box`, `008_tray`, `062_plasticbox` and model-data files under RoboTwin objects.
- Reference: [RoboTwin Scan Object](https://robotwin-platform.github.io/doc/tasks/scan_object.html). Its scanner/object model and functional-point definitions provide the starting geometry; packing is our proposed extension.

Both proposals have explicit reasons for all four arms to participate and avoid merely reskinning the existing egg-sorting task.

## Delivery status

Implemented and verified locally. Publication was requested on 2026-10-07; this change is the Demo2 optimization release, separate from the subsequent scanning/packing and cooperative pot-loading scenes. The release procedure repeats the full test suite, type check and production build before pushing, then checks the matching Pages deployment.
