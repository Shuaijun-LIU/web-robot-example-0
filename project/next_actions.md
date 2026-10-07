# Next Actions

## Current handoff — 2026-10-07

1. Preserve accepted Demo1/Assembly1/Demo2 and the verified Demo3/4 trajectories. Detailed current state: [workcell design record](cooperative-workcells-design-record.md).
2. Demo2 optimization `0c18da6` was pushed; Demo3/4 remain on `feat/franka-scan-and-pot` at recorded baseline `54ada17`. Demo5 is a new local increment. Do not describe these new scenes as deployed; no fresh publication instruction is part of this implementation turn.
3. User inspection of [Demo5 drawer kitting](../docs/progress/2026-10-07-drawer-kitting.md): select Franka Demo5 and Play task. The full browser cycle, Pause/Reset/switch, 261 tests and build pass. Keep this 49-phase real-contact baseline stable; publish only on user instruction.
4. Further benchmark work should add seeded task resets and dense synchronized observation/action/recovery export before collecting a training pilot; waypoint JSON is not a dataset. Retain legal single-/two-arm solutions as baselines. Other [task candidates](benchmark-task-proposals-2026-10-07.md) remain planned.
5. Carry forward the optional explicit Demo4 world-up/tilt gate separately. Do not change successful old motions as part of Demo5.

## Current blockers / qualifications

- No implementation blocker; requested local Demo5 implementation and acceptance are complete.
- Demo5 uses RoboCasa drawer, panel and handle assets with source snapshots, hashes and license attribution. The alternative RoboTwin cabinet was not imported. Other candidates' assets still need qualification when selected.
- Randomized task API, dense episode export and learned-policy evaluation are future work, not existing functionality. Physical hardware mounts are not qualified by simulation.

## Historical priorities — 2026-09-08 (superseded, preserved)

1. Keep Demo1 / Assembly1 at the user-accepted `019afd2` baseline; do not reopen their old action-tuning priorities. Existing dated diagnostics remain archived, not erased.
2. Preserve the user-accepted `f6d45ac` first-egg transfer. A separate local `Run correction trial` now demonstrates actual post-release regrasp/reseat; [before/after screenshots and verification](../docs/progress/2026-09-08-demo2-reseat.md). It is not yet committed/pushed.
3. Asset publication check is complete: the separately published RoboDojo dataset declares Apache-2.0 and both USDZ hashes match; see `public/assets/franka-egg-sorting/licenses/SOURCE-AUDIT.md`. Evaluate the UMI simulation assembly's egg access during single-arm manipulation. The first version uses four appearance classes, not four verified species.
4. The controlled post-release correction milestone passes (24.385 → 0.241 degrees). Next generalize to other classes and occupied tray cells, retaining real contacts, conservative forces and no object attachments/pose scripting. The current offline path only accepts a checked 4 mm / 5 degree inspection-pose envelope.
5. Add two-arm conflict tests, then parallel four-arm sorting with shared-space reservations and meaningful tilted-egg correction.
6. Add continuous Demo2 playback only after these gates pass; preserve existing page behavior and default entry.

## Historical blocked note — 2026-09-08
- Asset publication provenance is resolved. One-egg simulation grasp/transfer is verified; full sorting and a real hardware mounting design are not yet qualified.
