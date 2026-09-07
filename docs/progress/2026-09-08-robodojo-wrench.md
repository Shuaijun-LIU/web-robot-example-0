# Add a benchmark wrench to the unused Arm 1 pad

## Scope

The user approved adding RoboDojo's existing 17 cm open-end wrench to the empty
pad in front of Arm 1. Assembly1 and Demo1 share the addition; Assembly2 and all
existing robot actions remain unchanged. The user subsequently authorized
committing and pushing the addition.

## Asset and placement

- Source: RoboDojo `Assets/Object/RoboDojo/Rigid/wrench/00001/object.usdz`, UUID `308fd1d1-7f33-43c4-ad67-3ac5ac56ddcd`.
- The model is a combination wrench: one open end and one ring end. Its complete 1,399 vertices and 876 triangles are retained, without resizing or custom visual modeling.
- Source USD transforms are applied, the mesh is centered and rotated 180 degrees about vertical. The open end points toward the frame, the ring end toward Arm 1; the handle follows that radial direction.
- Final dimensions: 169.85 mm long, 27.26 mm wide, 6.16 mm thick. It lies within the 320 × 220 mm pad at `[0, -0.48]`, with the mesh bottom at the pad's .118 m top.
- The body remains free with the source's .15 kg mass. Its original collision mesh is decomposed into 18 convex parts, consistent with the source USD's convex-decomposition setting. Voxel preprocessing is disabled to retain the thin mesh envelope. Contact friction is `1 .005 .001`, condim 6; no freezing, attachment, attraction, pose tracking or disabled tool collisions.
- Source gray and geometry details are preserved; render metalness .65 / roughness .32 matches the workcell's metallic finish. No new labels or support geometry.
- This is scene placement, not a newly implemented wrench grasp or fastening action. Finger clearance for a future pickup still needs a dedicated grasp test.

## Reproducibility and validation

- Converter: `scripts/convert-robodojo-wrench.py SOURCE_USDZ public/assets/franka-assembly2/tools src/robodojoWrench.js` (usd-core, numpy, trimesh, coacd).
- Source checksum, exact bounds, XML and collision-part count: `public/assets/franka-assembly2/tools/robodojo-wrench.json`.
- Source notices and the accompanying RoboDojo license are included with the assets.
- New tests failed before adding the asset/layout and passed afterward. All 215 unit tests, TypeScript checking and production build passed on the final conversion.
- Production browser initial placement passed: the free tool has actual contacts with the pad and its height matches the visible mesh bottom. Overview and close-up were visually inspected.
- Initial full existing Demo1 playback passed: task completed, no sampled wrench contacts except with its pad, maximum wrench displacement 6.64 nanometers during playback, no physics warnings. Measurements are in `artifacts/reports/assembly-wrench.json`.
- Subsequent mesh-envelope audit identified .934 mm of bottom inflation from CoACD's default voxel preprocessing. Added a regression test that failed on that inflation, then disabled preprocessing and regenerated directly from the original collision surface. Final collision vertices stay within the visible model's bounding envelope (20 micrometer tolerance). This correction does not change the visual geometry or any robot action.
- Final corrected-collider browser placement verification passed: body Z .12107598 m, visible bottom .11799785 m versus pad top .118 m (about 2 micrometers contact compression); pad contacts exist, no page errors or physics warnings. Final screenshots were refreshed; measurements are in `artifacts/reports/assembly-wrench-idle.json`. The full playback report above belongs to the initial 15-piece collision variant, not the final 18-piece conversion; full playback was not repeated after removing collider inflation.
- [Overview](../../artifacts/screenshots/assembly-wrench-overview.png) / [close-up](../../artifacts/screenshots/assembly-wrench-closeup.png).
