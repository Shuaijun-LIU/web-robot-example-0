# Restore the pre-September-6 warm palette

## Corrected baseline and scope

The previous palette change used `f64a6c6` (September 6, 23:48), which already replaced the beige background with gray-green. The user identified the desired appearance as the version visible around September 5. The latest committed baseline before that date is `ea84ab2` (September 3); its background is `#d8d2b5`.

- Assembly1 and Demo1 now share that warm beige background.
- Non-tool meshes retain the source model's colors rather than the later presentation layer's uniform color overrides. Existing named non-tool geometry RGB values were compared against `ea84ab2` and had no differences. Platform and mat colors were also checked against that version.
- The refined, continuous connector uses the original connector plate's linear RGB `.24 .27 .29`, preserving its current geometry and open bores.
- Current metalness, roughness, ambient/directional/reflection lighting, tool paint, cameras, details, physics, trajectories and spare-fastener stabilization are unchanged. This is a palette restoration, not a rollback of the scene or renderer.

## Verification

- Material tests failed on the previous uniform color overrides, then passed after restoring source colors. They also check inner/outer rail contrast, mat colors, tool isolation and connector geometry.
- All 212 unit tests, TypeScript checking and production build passed.
- Production browser verification passed for Demo1 and Assembly1: four robot instances loaded, no page errors, and the rendered background pixel was exactly RGB `216,210,181` (`#d8d2b5`) on both pages. The first test attempt selected the camera dropdown instead of the page selector; narrowing the test selector resolved that test-harness issue without application changes.
- Regression script: `scripts/verify-assembly-warm-palette.mjs`.
- Screenshots: [Demo1](../../artifacts/screenshots/demo1-warm-palette.png), [Assembly1](../../artifacts/screenshots/assembly1-warm-palette.png).

The user authorized committing and pushing this change.
