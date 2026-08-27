# Notes

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
