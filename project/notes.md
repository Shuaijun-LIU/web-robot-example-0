# Notes

## Current design index — 2026-10-07

- [Demo6 supported insertion](../docs/progress/2026-10-07-supported-insertion.md): sourced robosuite stand/frame, four-arm supply/stabilize/insert/dispatch, real cavity and continuous retention checks. New local feature after the Demo3/4/5 release.
- [Demo5 drawer-kitting implementation](../docs/progress/2026-10-07-drawer-kitting.md): two passive RoboCasa drawers, two source product boxes, one shared order tray, real handle manipulation and physical pack/release/close. Includes source/geometry decisions, failed approach diagnoses, verification artifacts and rebuild commands.
- [Demo3/4 detailed design record](cooperative-workcells-design-record.md): robot layout, arm roles, task dependencies, source assets, contact/release decisions, executable interfaces, prior validation and conversion to training tasks.
- [Benchmark candidates](benchmark-task-proposals-2026-10-07.md): three proposed task families, asset locations, cooperation shortcuts, randomized variables, data schema, baseline/split/evaluation design and independent agent critique.
- Demo2 four-arm work supersedes the old single-egg pending list. Demo3/4/5 were published at `f3bc480`, Pages deployment succeeded. Candidate B is now Demo6 locally; candidate C remains planned.
- Source inspection found: RoboCasa drawer damping is not a self-closing mechanism; robosuite TwoArmPegInHole attaches objects to robot bodies; ToolHang uses a deliberately heavy stand configuration. These reference assumptions must not silently enter a real-contact cooperation benchmark.
- The older context/findings below are historical checkpoints, not current unresolved failure claims. Use the dated progress reports and the current design index for latest status.

## Context
- The fourth scene is a separate option cloned from the Franka model configuration, not a replacement for the existing three scenes.
- It began as a static scene-design iteration and now has a staged physical action sequence.

## Links
- RoboTwin local asset index: `/data/private/user2/workspace/benchmarks/RoboTwin/assets/objects`
- Franka source: `google-deepmind/mujoco_menagerie/franka_emika_panda`

## Findings
- RoboTwin includes screwdriver and hammer GLB assets with semantic grasp/contact metadata.
- The current `mujoco-react` loader resolves all MJCF dependencies from one base URL and supports primitive `sceneObjects`; directly mixing a second GLB asset root would either break physical loading or require vendoring the full Franka pack.
- Compound MuJoCo primitives preserve physical/visual alignment and are sufficient for this workcell.
- Step 2 currently resets its complete contact window on any missing contact-manifold sample.
- Step 3 currently combines horizontal alignment and seated height into one three-dimensional hole distance.
- Arm 3 cannot reliably reach the original northeast fastener tray; the approved center `(0.18, 0.48)` is reachable and clear of the prior paths.
- Step 4 browser trials established real bilateral r2 contact and, in one candidate, a physical fastener lift from roughly `z=0.136 m` to `z=0.290 m`. Contact was not retained through the end of lift, so the final browser verifier remains red with `missing-finger-contact`.
- Static evidence is green: 163/163 unit tests, TypeScript, production build, and offline Step 4 IK. This does not supersede the failed dynamics gate.
- GitHub Pages currently reports `build_type=legacy` and `source=main:/docs`. A successful Actions deployment alone does not update the user-visible page; the committed `docs/index.html` had continued to reference the pre-Step-4 `index-CewyqVpw.js` bundle until the 2026-08-28 rebuild.
