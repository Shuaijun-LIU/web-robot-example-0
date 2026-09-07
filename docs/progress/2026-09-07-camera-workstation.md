# Assembly1 / Demo1 cameras and material refinement

## Approved scope

- Leave Step 1–4 trajectories, timing and performance logic unchanged.
- Remove workstation text labels, retain physical fixtures and non-text mounting detail.
- Add five RGB simulation views: one fixed overview and four wrist-mounted RealSense D435 cameras. These are simulated RGB images, not a claim of calibrated RealSense depth/noise simulation.
- Reuse camera, Panda mount and camera-mast meshes; do not invent camera housings.
- Yellow/black drill; lighter silver-white frame/beam/pins; darker matte trays and supports. No wear.
- Refine only the beam's insertion interface; preserve the bore and existing contact geometry.

## Design

Camera views use the existing Three.js scene and one renderer. A collapsible English-language panel selects Global / Arm 1–4 or all five. Low-resolution previews refresh at 10 Hz; no duplicate physics worlds. The free inspection view remains available. The RGB optical transform is fixed to each camera housing, never aimed dynamically through occluders.

Integrated sources: RealSense ROS D435 mesh (Apache-2.0), ViSP Panda D435 holder (GPL-2.0), ALOHA Menagerie extrusion/mount (BSD-3-Clause). Original assets, licenses, transforms and conversion details are recorded in [asset notices](../../public/assets/assembly-cameras/THIRD_PARTY_NOTICES.md).

## Implementation checklist

- [x] Tests first: label removal, material contrast, unchanged tool vertices, camera transforms and selection.
- [x] Material and insertion-interface visual changes in `assemblyPresentation.js`.
- [x] Camera assets, source notices, fixed mounting transforms and optical projection.
- [x] Camera scene component and compact panel shared by Assembly1 / Demo1.
- [x] Browser checks: five distinct live feeds, switching/collapse/scene remount, source assets load with Pages base path.
- [x] Full physical assembly regression; record actual outcome and screenshots.

## Installation and presentation decisions

- Use D435 for all five cameras. Wrist mounts are fixed to each simulated hand, with 72 g camera mass and an estimated 15 g holder mass; simplified casing/holder collision envelopes remain enabled.
- Rotate the reused Panda flange holder 180 degrees to the outside of the handover gap. The first same-side installation caused an Arm 2/4 camera collision; the outside installation passed the unchanged handover trajectory. Camera rear and holder seating surfaces meet rather than float apart.
- Fixed overview camera sits on the reused ALOHA extrusion, foot and mounting bracket. The camera housing and wrist holder are downloaded meshes, not procedural replacements.
- Sensor previews refresh at 10 Hz, 384 × 216. The panel supports one selected feed, all five, and collapse. Only UI overlays (IK gizmos/contact debug markers) are excluded from sensor passes; real geometry is not hidden to clear the view.
- Remove added tool/station text decals. Retain non-text rings, physical supports and mounting details. Drill motor casing is yellow, grip/battery/chuck black and nose metallic, using the existing scanned mesh.
- Lighten frame/beam/connectors/fasteners to silver-white; supports and trays are darker matte surfaces. No weathering added.
- Replace only the connector's segmented visible face with one continuous face containing open round/square bores. Existing bore dimensions, seating height and contact geometry remain unchanged.
- No edits to the Step 1–4 controllers, trajectories, timing or existing performance logic. Accessory mass/collisions are the only additions to physics.

## Verification results

- TypeScript: `tsc --noEmit` passed.
- Unit suite: **209/209 passed**, no skipped tests.
- Production build: Vite build with `/web-robot-example-0/` base passed; existing bundle-size warning remains.
- Production browser: `scripts/verify-assembly-cameras.mjs` passed against the prefixed preview URL. Assets loaded, all five views rendered, selection/collapse/scene switching worked, and Step 1 visibly animated. No browser errors. Final screenshots were inspected, including drill paint and UI-free sensor images.
- Full production physical playback: **PASS / Demo complete**, simulation time 139.47 s. Hammer strike observed with all three support arms in bilateral contact. Existing audit recorded no violations and all eight MuJoCo warning counters remained zero. Peak audited grip error 4.09 mm; maximum audited pin penetration 0.0847 mm. These are measured solver/audit results, not a claim of mathematically zero penetration or calibrated sensor physics.
- Full physical audit used the final camera mass, collision envelopes and mounting transforms. Two subsequent presentation-only changes excluded IK overlays from sensor views and corrected drill paint; the final build, all tests and camera browser check were rerun after those changes.

## Review artifacts

- [Five working views](../../artifacts/screenshots/assembly-cameras-working.png)
- [Overview](../../artifacts/screenshots/assembly-cameras-overview.png)
- [Wrist mounting close-up](../../artifacts/screenshots/assembly-camera-installation.png)
- [Fixed mast close-up](../../artifacts/screenshots/assembly-camera-mast-final.png)
- [Final drill material](../../artifacts/screenshots/assembly-camera-tools.png)
- [Full physical audit](../../artifacts/reports/assembly-camera-physical-audit.json)
- [Complete playback recording](../../artifacts/videos/assembly-camera-complete.webm) — recorded before the final presentation-only drill paint/IK-overlay changes described above.

Intermediate screenshots were moved to the ignored `artifacts/debug/camera-installation/` directory, not deleted.

The user subsequently authorized committing and pushing this complete camera/material update to `main`.
