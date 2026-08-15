# Project Status

## Goal
- Build two additive Assembly1-derived scenes using four AgileX PiPER arms and four UR5e + Robotiq 2F-85 arms.

## Current State (last updated: 2026-08-15)
- Assembly Step 3 is already present in `main`.
- The verified PiPER and UR5e Assembly1 history is integrated into `main`.
- The verified Unitree locomotion history is integrated into `main`.
- Previously untracked Assembly diagnostic screenshots are archived in Git.
- The integrated branch passes 151 tests, TypeScript checking, and a production build.

## Active Workstreams
- None. The unified `main` is ready for remote synchronization.

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
- `node scripts/verify-alternate-assembly-browser.mjs`
- `SCENES=piperAssembly1,ur5eAssembly1 node scripts/capture-scenes.mjs`
