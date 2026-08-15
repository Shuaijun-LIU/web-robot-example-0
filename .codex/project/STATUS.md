# Project Status

## Goal
- Build two additive Assembly1-derived scenes using four AgileX PiPER arms and four UR5e + Robotiq 2F-85 arms.

## Current State (last updated: 2026-08-15)
- `origin/main` contains the published Franka Assembly1 scene.
- The canonical main checkout is occupied by separate Assembly Step 3 work and is not touched by this workstream.
- This workstream is isolated in `.worktrees/piper-ur5e-assembly` on `feature/piper-ur5e-assembly`.

## Active Workstreams
- S-PIPER-UR5E: two new manual industrial-arm assembly scenes.
- S-ASSEMBLY-STEP3: observed in the canonical checkout; ownership and changes remain external to this worktree.

## Key Paths
- `src/configs.ts`
- `src/controlTargets.js`
- `src/alternateAssemblyLayouts.js`
- `public/assets/piper-assembly1/`
- `public/assets/ur5e-assembly1/`

## How to Run / Test
- `npm test`
- `npm run build`
- `npm run dev`
