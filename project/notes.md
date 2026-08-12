# Notes

## Context
- The fourth scene is a separate option cloned from the Franka model configuration, not a replacement for the existing three scenes.
- It is a static scene-design iteration only; no scripted task motion is in scope.

## Links
- RoboTwin local asset index: `/data/private/user2/workspace/benchmarks/RoboTwin/assets/objects`
- Franka source: `google-deepmind/mujoco_menagerie/franka_emika_panda`

## Findings
- RoboTwin includes screwdriver and hammer GLB assets with semantic grasp/contact metadata.
- The current `mujoco-react` loader resolves all MJCF dependencies from one base URL and supports primitive `sceneObjects`; directly mixing a second GLB asset root would either break physical loading or require vendoring the full Franka pack.
- Compound MuJoCo primitives preserve physical/visual alignment and are sufficient for this static workcell iteration.
