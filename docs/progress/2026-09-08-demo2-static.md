# Demo2 static workcell — 2026-09-08

Release follow-up: the user authorized commit/push and subsequent manipulation development. The asset-rights follow-up below is resolved: the RoboDojo asset dataset explicitly declares Apache-2.0 and remote source hashes match both local USDZ files. The separate code repository's license text is not treated as the asset license; see [source audit](../../public/assets/franka-egg-sorting/licenses/SOURCE-AUDIT.md).

## Delivered locally

New scene selector entry: **Franka Demo2** (`frankaDemo2`). Default entry remains **Franka Demo1**. Existing Assembly assets and automatic motion code are unchanged. Work remains uncommitted on `feat/demo2-egg-sorting`; no push or deployment was performed.

- Four independent Panda arms, bases at the platform top (`z=0.10 m`), on a 0.78 m ring.
- One central shared box containing 16 interleaved eggs: ivory, brown, pale-green and cream, four of each. Every source insert contains all four classes.
- Four empty four-cavity destination trays on class-colored mats; one assigned to each arm.
- Existing Menagerie UMI base, long fingers, holders and GoPro meshes, rigidly adapted at the Panda wrist for this simulation review. The original finger shape is not stretched or hand-modelled.
- Existing arm selection, IK gizmo/target controls, keyboard gripper toggle and Reset. English scene-review panel; no automatic sorting button or hidden manipulation controller.

## Inspectable artifacts

- [Overview](../../artifacts/screenshots/demo2-eggs-overview.png)
- [Mixed source box](../../artifacts/screenshots/demo2-eggs-box.png)
- [Empty destination tray](../../artifacts/screenshots/demo2-eggs-tray.png)
- [Sourced UMI gripper adaptation](../../artifacts/screenshots/demo2-eggs-gripper.png)
- [Browser control/contact report](../../artifacts/reports/demo2-eggs-browser.json)
- [Asset dimensions, transformations and initial-state manifest](../../public/assets/franka-egg-sorting/manifest.json)
- [Sources, licenses and adaptation notes](../../public/assets/franka-egg-sorting/README.md)

Local production preview: `http://127.0.0.1:4174/web-robot-example-0/`; choose **Franka Demo2** in the Robot selector. This address is on the server; remote viewing requires an existing port-forward. The GitHub page has not been updated in this turn.

## Verification

- All **216 Node tests passed** (215 pre-existing tests plus the real-MuJoCo egg-workcell test).
- TypeScript `tsc --noEmit` passed; production Vite build passed. Existing large-chunk / Node-module externalization warnings remain.
- MuJoCo WASM compiles 32 actuators and 148 position coordinates: four seven-joint arms with two sliding jaws each, plus 16 free eggs.
- Regression test uses page-equivalent initialization, including jaws starting at zero before opening under actuator control. Confirms matching table/base height, independently named TCPs/gripper actuators, real egg support, mixed categories and under-2-mm egg displacement over its five-second stability observation.
- Browser check: all four control targets switch correctly; each V-key toggle changes only the selected gripper command; all four IK target interfaces accept a 2-mm test adjustment. This is a manual-control smoke test, not a collision-free full workspace guarantee.
- Browser Reset succeeds, then screenshots/contact readings are taken. All 16 eggs have physical support. No penetrating robot contacts were found at the inspected idle state; maximum sampled contact penetration overall was approximately **0.0023 mm**. No JavaScript errors, failed HTTP responses or physics warnings were recorded.
- Browser successfully switches back to Demo1 and displays its Play button. The complete Demo1 action sequence was not rerun in this static-scene turn; no claim of a new full-playback regression run is made.
- Screenshots were captured with explicit CPU SwiftShader rendering. Their low FPS readout is not a benchmark of the user's Mac or GPU performance.

## Issues found and corrected in this milestone

1. The old deeply folded home caused UMI fingers to contact Panda link 1, with approximately 1.9 mm penetration. A failing physical regression test reproduced this. Demo2 now uses an unfolded hover home; the test passes without disabling those collisions.
2. Fresh upright egg placement contained a metastable egg that rolled later. The builder now performs 60 seconds of passive gravity/contact settling, saves the resulting initial poses, then reloads the exported scene with zero velocities for a Reset-like stability check. The maximum five-second displacement in that native check is approximately **0.834 mm**, recorded in the manifest. This removes the large delayed roll, but does not claim mathematically zero rigid-body drift.
3. Tray mats originally coincided with the platform top. They now sit on top of the platform, with the trays supported above them.
4. Read-only review prompted collision-cache keys based on actual converted geometry/settings and a page-equivalent reset test. No critical or important old-scene regression was identified.

## Deliberate boundaries before the next stage

- **Static layout only.** Egg grasping, placement correction, shared-region scheduling and four-arm automatic collision avoidance are not implemented or validated yet.
- Four appearance classes reuse one sourced shell with documented scale/color changes; they are not four independently sourced biological species.
- Source holder shape is widened and lowered, and its matte material represents a packing insert. It is not a validated deformable foam/soft-body model.
- UMI's sourced handheld assembly has a simulation wrist adaptation. A qualified physical robot adapter and the final grasp contact envelope must be checked before claiming hardware-realistic use.
- Existing repository licenses are included. Separate upstream publication rights for the RoboDojo assets remain to be checked before public release; local availability does not establish redistribution permission.
- Next milestone, after scene feedback: prove one real-contact egg grasp, lift, tray placement, release and correction. Do not inherit Assembly1's high gripper forces or introduce proximity attachment/object pose animation.

## Workflow

The approved design was implemented with an explicit plan, failing-then-passing model tests, browser verification and an independent read-only review. These checks led to the isolated home-pose correction, passive initial-state settling and cache/reset-test safeguards above.
