# Demo2: two-arm coordination to four-arm sorting

## Published checkpoint

The accepted physical reseating trial was committed as `284d045` and pushed. Both Pages workflows completed successfully for that revision. Demo1/Assembly1 remain frozen.

## Implementation and measured progress

- Added a separate multi-arm actuator runtime and an English continuous-start panel, preserving both single-egg checks.
- Every egg has one class owner and one tray cell. Existing models, masses, grippers and collision meshes are unchanged.
- Independent native physical generation now covers all 16 eggs. Far-row reach uses a tilted wrist, mesh-distance checked approach yaw/depth, and seeded IK. Tray placement yaw accounts for already filled cells.
- A tilted release can meet the tray rim. A short 12 mm upward opening motion clears it; this is physical gripper motion, not an egg pose edit. Gate checks still require supported, released placement.
- Starting adjacent arms together was rejected by actual `r0_hand/r1_hand` contact (0.452 mm penetration). A 15 s source-corridor delay passed in native MuJoCo and the actual browser WASM runtime, with 12.968 s of simultaneous movement and zero forbidden penetration.
- Four arms / one egg each passed fresh native concurrent replay: four simultaneous moving arms, 40.938 s of parallel movement, counts `[1,1,1,1]`, zero forbidden penetration. This is not the final 16-egg result.
- Full-16 **fresh fixed-path native replay** now passes: `[4,4,4,4]`, 216 s total, 169.660 s parallel movement, maximum four arms moving, zero sustained grip loss, 0.0523 mm maximum forbidden penetration, 0.4802 mm grip penetration and 1.6859 mm unpicked-egg drift. All eight engine warning counters are zero; all arms returned HOME. The final native report has `refinedDuringReplay: false`.
- The shared browser WASM runtime also passes all 16 eggs: 169.660 s parallel, maximum four moving arms, 0.0552 mm forbidden penetration, 0.5045 mm grip penetration, 1.6857 mm unpicked-egg drift, all engine warning counters zero and all arms HOME. The full Node suite passes 236 tests.
- A separate fresh replay audits the contact points against the sourced finger mesh triangles at all 80 grasp/carry/support gates. Maximum surface distance is **0.1473 mm**, below the 1 mm limit; all 16 placements still complete. This is a gate-sampled visual/collision consistency audit, in addition to continuous runtime contact checks.
- Actual page acceptance passes: counts `[4,4,4,4]`, all arms HOME, 216 simulated seconds, 169.660 seconds of overlapping movement and maximum four moving arms. Maximum forbidden penetration is 0.0544 mm, grip penetration 0.3406 mm, and unpicked-egg drift 1.6908 mm. All eight engine warning counters are zero and browser errors are empty. Idle manual-close Reset and actual bilateral-grasp carrying-state Reset both pass. The original physical correction trial also passes its actual-browser regression.

## Scheduling and runtime review

Source leases include approach, grasp, lift and the outward transfer. Only opposite arms serving opposite near-side quadrants may share these leases. Opposite far-side reaches were rejected by a real hand/hand collision in the final round and are serialized. Other requests wait at HOME; tray work can overlap. A full-world physical replay remains mandatory because this corridor rule is not a general collision planner.

Plan validation rejects duplicate objects/cells, incorrect class/tray assignment, overlapping source leases, nonchronological per-arm paths and wrong HOME poses. Startup additionally checks the actual eight finger joints are open. Checked dwell durations are explicit in the paths; unexpected physical gate failure causes an actuator safety stop rather than silently delaying the whole timeline. Reset also retries a failed asset fetch.

Unpicked eggs have small passive settling drift even without arm motion (e.g. egg 2: 0.824 mm after 51 s, 1.700 mm after 101 s). Long serial candidate generation is not the acceptance measurement for neighbor movement. Final runtime checks displacement from the fresh replay's initial scene and retains the 2 mm threshold.

Serial pickup offsets did not exactly reproduce the concurrent world: the last brown egg's landing path needed a few-millimetre correction. Offline landing refinement now uses the concurrent physical grasp offset, then a **separate fresh fixed-path replay** must pass before export. No browser IK or object following is introduced.

Landing checks distinguish load transfer from carrying: once a real tray contact supports an egg within **3 mm** of its cell, below **20°** tilt and below **8 mm/s**, one jaw may legitimately lose contact. The in-flight bilateral-contact requirement is unchanged. For example, the previously rejected landing had real tray support, 0.269 mm cell error, 1.823° tilt and 0.625 mm/s speed. Opening/withdrawal still require zero finger contacts afterward.

Reset remounts the Demo2 teleop toggle so an idle manual close cannot survive Reset. Asset-load failures do not own actuators. The generic all-body gravity-compensation switch is disabled in Demo2 because it would also compensate free eggs; robot gravity feed-forward remains in the checked actuator controller.

## Reproduction

Use Python with MuJoCo 3.3.7, NumPy and trimesh for the native scripts; the web runtime uses MuJoCo 3.3.8. JavaScript build tooling requires Node 22.12 or newer.

```bash
python scripts/solve-egg-sorting.py --replay --arms 4 --input public/assets/franka-egg-sorting/sorting-motion.json
node --test test/egg-sorting-coordination.test.mjs test/egg-sorting-wasm.test.mjs
node scripts/verify-egg-sorting-runtime.mjs
python scripts/verify-egg-sorting-surfaces.py
```

The committed motion is the canonical checked result. To search new candidate paths, run `--arms 4 --rounds 4`, then `--replay --arms 4 --refine`, and finally a fresh `--replay --arms 4 --input artifacts/reports/demo2-four-arm-native.json --export`. Generation is a gated search, not a guarantee for modified layouts; do not replace the checked asset unless native, WASM and browser validation pass.

The checkpoint file is only an offline planning restart artifact, not browser playback state. Runtime code drives actuators; it does not write egg poses, add welds, disable gravity, or disable collisions.

## References used

- [MoveIt simultaneous trajectory execution](https://moveit.ai/moveit/google/2023/01/12/gsoc-simultaneous-trajectory-execution.html): validate interacting whole-arm trajectories, not TCP separation alone.
- [MuJoCo collision computation](https://mujoco.readthedocs.io/en/latest/computation/): contact-based physical checks.
- [MuJoCo engine support implementation](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_support.c): exact geometry-distance queries for approach and tray clearance.

## Final acceptance evidence

- [Actual-page report](../../artifacts/reports/demo2-four-arm-browser.json), including physical diagnostics, phase transitions and carrying-state Reset.
- [Four-arm parallel motion](../../artifacts/screenshots/demo2-four-arm-parallel.png).
- [All 16 eggs placed and four arms HOME](../../artifacts/screenshots/demo2-four-arm-complete.png).
- Full Node suite: **236/236 passing**; focused coordination/exception tests: **8/8 passing**; TypeScript and production build pass.
- Screenshots use CPU software rendering for verification; their displayed FPS is not a measurement of ordinary hardware-accelerated browser performance.
- Delivery keeps Demo1/Assembly1, the default page and physical asset geometry unchanged. Select **Franka Demo2**, then **Run four-arm sorting**. Classification uses known scene metadata; this is a checked fixed-layout task, not a camera-recognition or general online planning system.
