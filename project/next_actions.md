# Next Actions

## Top priorities
1. Keep Demo1 / Assembly1 at the user-accepted `019afd2` baseline; do not reopen their old action-tuning priorities. Existing dated diagnostics remain archived, not erased.
2. User review of the implemented static `Franka Demo2`: inspect [overview](../artifacts/screenshots/demo2-eggs-overview.png), [box](../artifacts/screenshots/demo2-eggs-box.png), [tray](../artifacts/screenshots/demo2-eggs-tray.png) and [gripper](../artifacts/screenshots/demo2-eggs-gripper.png).
3. Asset publication check is complete: the separately published RoboDojo dataset declares Apache-2.0 and both USDZ hashes match; see `public/assets/franka-egg-sorting/licenses/SOURCE-AUDIT.md`. Evaluate the UMI simulation assembly's egg access during single-arm manipulation. The first version uses four appearance classes, not four verified species.
4. Prove one-egg physical grasp/place/reseat with contact-aligned visual and collision geometry. Do not inherit Assembly1's high-force settings or use object attachments/pose scripting.
5. Add two-arm conflict tests, then parallel four-arm sorting with shared-space reservations and meaningful tilted-egg correction.
6. Add continuous Demo2 playback only after these gates pass; preserve existing page behavior and default entry.

## Blocked
- Asset publication provenance is resolved for this release. Physically stable gripping and real hardware mounting are not yet validated.
