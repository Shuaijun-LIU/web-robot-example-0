# Next Actions

## Top priorities
1. Keep Demo1 / Assembly1 at the user-accepted `019afd2` baseline; do not reopen their old action-tuning priorities. Existing dated diagnostics remain archived, not erased.
2. Review the isolated `Franka Demo2` first-egg transfer using `Run first egg`. Native and WASM physical replay pass; the release's final browser/Reset evidence is tracked in [the first-egg report](../docs/progress/2026-09-08-demo2-first-egg.md). Static scene `49f9be7` has already been published.
3. Asset publication check is complete: the separately published RoboDojo dataset declares Apache-2.0 and both USDZ hashes match; see `public/assets/franka-egg-sorting/licenses/SOURCE-AUDIT.md`. Evaluate the UMI simulation assembly's egg access during single-arm manipulation. The first version uses four appearance classes, not four verified species.
4. Next physical milestone: detect tilt after release and perform a real regrasp/reseat, then generalize to other classes and occupied tray cells. The first transfer already corrects tilt in free space; this is not the same as post-placement correction. Keep contact-aligned geometry, no high-force inheritance and no object attachments/pose scripting.
5. Add two-arm conflict tests, then parallel four-arm sorting with shared-space reservations and meaningful tilted-egg correction.
6. Add continuous Demo2 playback only after these gates pass; preserve existing page behavior and default entry.

## Blocked
- Asset publication provenance is resolved. One-egg simulation grasp/transfer is verified; full sorting and a real hardware mounting design are not yet qualified.
