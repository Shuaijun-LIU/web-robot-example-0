# Physical assembly completion

User request: align visible and collision geometry; bring handover grips together; return Arm 4 with its hammer while Arm 3 shallow-inserts and releases the fastener; stabilize with the three other arms, strike once, return all arms home.

## Root causes and implementation plan

- The downloaded hammer has a curved handle; the old invisible 48 mm rectangular grip and added ribs do not follow that surface. Replace collision geometry with small convex pieces clipped directly from the visible OBJ. Keep original mesh/color assets.
- Step 4 aimed at a nominal frame marker rather than the actual cross-member hole, and its old 0.29 m TCP insertion endpoint drives the fingertips below the connector top. Align to the measured hole and held shaft, shallow-insert with force/contact limits, open, then withdraw vertically.
- The previous controller held Arm 4 at its handover pose during insertion. Add a retreat/wait posture and complete the support, single-strike, release and return sequence.
- Verify physical contacts against geometry, actual shaft clearance/depth, beam motion, robot collisions, strike contact and final joint poses. Record actual failures; a handover-only pass is not a full-task pass.

Reference: MuJoCo collision detection uses convex meshes; concave handles/openings require decomposition rather than a single enclosing hull: https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection

Status: full Assembly1 sequence and final production Demo1 continuous playback passed. The intermediate entries below preserve failures and superseded trials, rather than representing the final state. The user approved committing and pushing this implementation; deployment status is tracked by the repository's Pages workflow.

## Additional reproduced root cause

MuJoCo attachment does not inherit a child model's global integrator option. A minimal native `mujoco-js` test returned child integrator 3 (`implicitfast`) and parent integrator 0 (`Euler`). The four-arm scene had no explicit parent option despite high-gain Panda actuators. Explicitly set `implicitfast` in the parent scene; rerun physical regressions rather than accepting previously tuned tolerances. Step 4 also uses gravity-only actuator feedforward calculated on zero-velocity scratch data, preserving actuator force limits and object gravity.

## Intermediate physical evidence

- First three steps pass with the model-derived hammer collision surface, explicit parent integrator and firmer robot contact surfaces.
- Physical handover passed after aligning the receiver closing axis to the actual handle and updating its approach from measured object pose. At donor release, receiver TCP in hammer coordinates was approximately `[-0.0469, -0.0033, -0.0099]` m, compared with the grip center `[-0.045, 0, -0.008]` m. Grasp error was 4.2 mm; both fingers retained physical contact after donor clearance.
- Returning the loaded Arm 4 to the exact folded home posture produced hammer-head contact with `r3_link2`. This is a real self-collision, not a heavy-object issue. Use a waiting pose in front of the west base, then return the hammer to a two-block west-side cradle before all four empty hands return home.
- Full insertion, support, strike and tool-return validation remains in progress. Intermediate handover success must not be described as full-task success.

## Fixed simulation control clock

The upstream renderer invokes `beforeStep` once per display frame, then executes multiple MuJoCo steps. Added a version-checked Vite adaptation, opt-in via Assembly1/Demo1's `controlTimestep=0.01`, to update controllers on the simulation clock. Other scenes retain their previous loop. Dev and production use the same adaptation; unit tests check frame-grouping invariance and fail on an unmatched upstream implementation.

First fixed-clock physical run: Steps 1–3 passed without reset; handover passed with 4.2 mm TCP-to-grip-center offset and bilateral contact. Safe retreat, pin pickup, lift and transfer passed. At the hole approach, shaft radial error was 0.85 mm and tilt 0.026 degrees, without workpiece or arm collisions. The run correctly stopped because the waiting hammer slid longitudinally by about 15 mm; receiver clamp force is being increased, not relaxing the grasp-zone check. Insertion/strike/full return remain unverified.

Further physical tests confirmed shallow insertion and release: seated shaft radial error 0.085 mm, axis tilt 0.010 degrees, beam displacement 0.24 mm, pin/plate contact penetration 0.071 mm. This run still failed on the waiting hammer's longitudinal slip; it is not a completed task.

The full-sequence contact audit also detected up to 2.8 mm finger/beam-shoulder penetration during Step 3 descent. The beam contact impedance was brought into line with the stiffer finger contacts. Handover now approaches from above and must stay within 10 mm of the measured handle center for 0.3 s with OPEN fingers before closure. Both tool gripper servos are strengthened within their existing actuator force limits, with original calibration restored on reset/error/completion.

Official reference for the further friction-solver test: https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip . In particular, slow contact creep is a property of the soft-contact model; elliptic friction cones with increased `impratio` reduce it. Testing `cone="elliptic" impratio="10"`; no welds, attachments, contact masks disabling required collisions, or object-pose forcing are used.

## Current approach and reproduced solver-memory failure

- The elliptic-cone trial affected the frame's existing contacts; it was superseded by two NoSlip post-processing iterations during Step 4 only, restored on reset/error/completion. This operates on actual contact friction, not object attachments.
- Present the hammer level, planning the donor wrist from its measured grasp transform. A wrist-only tilt tilted the object too and made the receiver approach almost along the handle. The current level exchange, retreat, pin insertion/release and waiting grasp have passed repeated physical runs. Waiting grasp center error is about 3.3 mm; pin-to-hole radial error about 0.05 mm; beam displacement about 0.17 mm.
- Dense IK paths now have one global smooth acceleration/deceleration envelope instead of stopping at every sample. A regression test reproduced the artificial stops before the change.
- Support centers use the actual opposite rail faces: east x=0.3175, north y=0.232. Parallel-gripper 180-degree symmetry avoids the north arm's wrist limit. All three supports established bilateral contact.
- A subsequent simultaneous contact collapse was traced to the physics engine, not a heavy hammer: `narena=18,874,368` bytes, `maxuse_arena=18,510,352`, warning index 2 (`CNSTRFULL`) counted 24 times with `lastInfo=18,874,368`. MuJoCo disables constraint solving when constraint allocation fails. Added an explicit 64 MiB arena and zero-warning final validation. Full tap/tool-return/home validation is still pending at this log entry.

## Full physical sequence: first passing Assembly1 run

`artifacts/reports/assembly-physical-audit.json` records a complete run with no prerequisite resets:

- Actual bilateral hammer contacts through handover and loaded waiting; typical receiver grip-center error approximately 3.3 mm.
- Measured shallow pin insertion followed by opening and vertical finger withdrawal; the pin subsequently settles under gravity. Final radial error 0.042 mm, depth 50.067 mm, tilt 0.017 degrees.
- Three supporting arms close on real frame rails; hammer strike terminates on actual head/pin contact.
- Return the hammer along the reverse safe staging path and set it down on two visible west-side blocks, then return all four empty arms to their original joint positions. Folding the arm while still holding the hammer would collide with its own elbow, so the loaded waiting posture and empty final home are intentionally different.
- Frame displacement 0.193 mm, beam displacement 0.188 mm / rotation 0.027 degrees. Pin/beam contact penetration 0.072 mm.
- No cross-arm penetration above 1 mm or finger/held-object penetration above 2.5 mm in the full sampled sequence. These are explicit soft-contact audit limits, not a claim of mathematically zero penetration.
- All engine warning counters zero. Peak constraint arena use 25,463,776 bytes within the new 64 MiB allocation.
- Final hammer contacts both return blocks; all 28 arm joints are within 0.04 rad of home.

## Production continuous-play regression

The first two production Demo1 runs exposed a marginal donor path that the earlier Assembly1 run had passed: its diagonal crossed Arm 1's wrist. Both runs stopped on real hammer/`r0_link6` contact; gripper penetration was also recorded during the resulting collision. Those failed runs are retained locally and are not presented as successful evidence.

The donor now carries the measured hammer through a high, north-side clearance waypoint before descending to the same exchange pose. Travel time increases from 4 to 6 seconds to accommodate that extra path. Collision checks remain unchanged. The complete production continuous-play regression passed after this change, without resetting or skipping a stage.

Validation commands: `npm test`, `tsc --noEmit`, `vite build`, `node scripts/verify-assembly-step4.mjs`, and `node scripts/verify-franka-demo1-physical.mjs` against the production preview. The continuous-play verifier checks the English-only two-button UI, all four stages, actual support contacts at impact, contact violations, engine warnings, final seated fastener, tool setdown and all home joints.

## Final production evidence

- Full recording: [Franka Demo1 continuous playback](../../artifacts/videos/franka-demo1-complete.webm), 5 min 54 s wall-clock recording, approximately 139 simulated seconds including startup. It is an uncut browser recording; rendering and planning affect wall-clock playback speed.
- Machine-readable checks: [production physical audit](../../artifacts/reports/franka-demo1-physical-audit.json).
- Final image: [four arms home](../../artifacts/screenshots/franka-demo1-complete.png).
- Receiver grip-center error in monitored held phases: maximum 4.02 mm. Actual bilateral contact is required for the handover; there are no object attachments or simulated-object pose edits.
- Maximum pin/beam penetration in the sampled sequence: 0.076 mm. Final shaft radial error: 0.105 mm.
- At the actual hammer-head/pin contact, all three support pairs were `[true, true]`.
- Whole-flow audit violations: none; all eight MuJoCo warning counters: zero. Peak arena use: 27,907,600 bytes / 64 MiB.
- Final tool rests on both return supports; all 28 joints pass the home-position check.
- Fresh unit tests: 198 passed, zero failed. Type checking and production build passed; existing bundle-size advisory remains.

This evidence covers the scripted initial scene and tested browser executions, not every possible manually edited starting pose. The audit uses stated soft-contact penetration thresholds, not a guarantee of zero numerical penetration or universal reliability across browsers.
