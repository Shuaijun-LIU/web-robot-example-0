# Next Actions

## Top priorities
1. Let the user run through Step 3, enter manual positioning, and save compatible Arm 2/Arm 4 `handover-approach`, `handover-clamp`, and `donor-clear` keyframes. GitHub Pages now downloads each JSON; local Vite additionally writes it to `artifacts/manual-poses/`.
2. Read the downloaded or server-saved snapshots and replace the incompatible handover joint targets with the user-approved poses.
3. Add collision-safe interpolation between the captured keyframes while retaining the 2.3 mm penetration/no-attachment gates.
4. Re-run handover-only and complete Step 1→2→3→4 browser gates before full tests/build/MJCF validation.

## Blocked
- None.
