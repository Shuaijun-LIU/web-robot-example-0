# Workstation palette restoration

## Approved scope

Restore the workstation palette from the pre-camera version (`f64a6c6`), except the drill, screwdriver and hammer. Preserve current metalness/roughness, geometry details, cameras, controls and actions. Preserve the separate, uncommitted spare-fastener stability changes.

## Changes

- Frame and cross-member base color: `#b2b8b9`.
- Connector and mounting plate palette: `#777f81`.
- Fasteners: `#aab0b1`.
- Supports/cradles: `#555f61`; trays: `#596466`.
- Metallic reflection and roughness parameters are unchanged, as are tool material/color functions, decals and mesh geometry. The refined connector retains its continuous surface and open bores.
- Background was explicitly compared with the approved baseline: both are already `#d6d8d2`. Ambient intensity .65, directional intensities 1.5/.3 and reflection lighting are also identical. They are retained rather than replaced with an unrelated lighting design.

## Verification

- Updated material regression checks failed against the old working palette and passed after the color changes.
- Geometry-preservation, open-bore, tool-color and material-isolation tests pass.
- TypeScript check, production build with the Pages base path, and all 211 unit tests passed.
- Production browser: Demo1 and Assembly1 loaded without page errors; screenshots captured for both pages. The earlier development-server navigation timed out while loading; verification was completed against the production preview instead.
- [Demo1 screenshot](../../artifacts/screenshots/assembly-palette-restored.png) and [Assembly1 screenshot](../../artifacts/screenshots/assembly1-palette-restored.png).

The user subsequently requested committing and pushing this change. All 211 unit tests were rerun successfully before submission; the remote branch had no intervening changes.
