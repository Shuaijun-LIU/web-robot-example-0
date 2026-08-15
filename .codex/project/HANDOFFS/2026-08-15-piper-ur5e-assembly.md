# Handoff: PiPER and UR5e Assembly1 Scenes

Status: Needs review  
Owner: S-PIPER-UR5E  
Branch: `feature/piper-ur5e-assembly`

## Scope

Two additive manual scenes are complete:

- `piperAssembly1` / `Piper Assembly1`
- `ur5eAssembly1` / `UR5e Assembly1`

They reuse the detailed procedural Assembly1 workcell but do not use or widen Franka automation conditions.

## Key Files

- `src/alternateAssemblyLayouts.js`
- `src/controlTargets.js`
- `src/controllers/IndustrialArmController.tsx`
- `src/configs.ts`
- `public/assets/piper-assembly1/`
- `public/assets/ur5e-assembly1/`
- `scripts/verify-alternate-assembly-browser.mjs`
- `docs/progress/2026-08-15-piper-ur5e-assembly-scenes.md`

## Verification Snapshot

- Tests: 129 passed, 0 failed.
- TypeScript: passed.
- Production build: passed under the local Node 24 executable.
- Offline strict contact: passed for both scenes.
- Browser: all four PiPER arms and all four UR5e arms passed isolated gripper + IK checks.
- Screenshots: `artifacts/screenshots/piper-assembly1.png` and `artifacts/screenshots/ur5e-assembly1.png`.

## Integration Notes

- Implementation commits begin after `origin/main` at `8b8b7e1`; scene implementation is complete through `e3abe44`, followed by the verification/documentation commit containing this handoff.
- The canonical checkout was not modified by this workstream.
- Review should preserve exact `robotKey === 'frankaAssembly1'` automation gates.
- PiPER's hammer station is intentionally at `x=0.58 m`; UR5e and Franka remain at `x=0.65 m`.
- All new runtime assets are local and their licenses are retained.
