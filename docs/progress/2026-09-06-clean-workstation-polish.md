# Clean workstation polish

Approved scope: refine the successful Assembly1 / Demo1 presentation. No wear,
scratches, dirt, rust or aged textures. Baseline: `fe99a0f`.

This iteration keeps the verified physical XML, collision surfaces, grasp sites,
actuator settings and all motion paths unchanged. The presentation layer may
change render materials and add flat surface markings, but must not replace a
support surface, fill an opening or create an apparent floating attachment.

Work items:

1. Distinguish clean aluminium, machined steel, rubber and coated surfaces.
2. Add flush station labels, tool parking outlines and arm identification.
3. Detail existing mounting/support surfaces without changing their envelope.
4. Refine lighting and the surrounding grid while retaining the same camera and
   manual controls; apply only to Assembly1 and Demo1.
5. Test material isolation/restoration and run the full physical sequence with
   the same collision limits. Capture initial/close/final images and full video.

Changes to pin-hole mechanical fit, chamfers on contact edges, action overlap,
and simulation-thread architecture are deferred to separately validated rounds.
This iteration does not claim to implement mechanical locking.

## Implemented

- Separate, removable Three.js presentation layer shared by Assembly1 / Demo1.
  Other pages keep their existing presentation. Render resources are released on
  reset/page change; decals do not intercept manual picking.
- Clean aluminium, steel, rubber, coated supports and worktop finishes with a
  locally generated softbox reflection environment (no remote texture download).
- The inherited tool mesh partitions followed thresholded scan texture colors,
  producing paint-chip-like patches. Replaced their rendering with continuous
  body-coordinate color/roughness/metalness fields. Vertex positions, triangles,
  object pose and physical collision meshes remain unchanged.
- Flush arm IDs, mounting outlines/recess graphics, station names and pin-tray
  positioning marks. Corner fastener surface graphics follow the real frame.
  Markings are printed/etched surface details, not new solid support geometry.
- A quieter neutral background/grid; no wear, dirt, scratches or aged textures.

## Checks

- New material-isolation, hidden-collision visibility, marking and tool-vertex
  preservation tests were observed failing before implementation, then passing.
- 202 unit tests passed; TypeScript and production build passed.
- Demo1 and Assembly1 page-switch rendering smoke test passed.
- Verified a zero diff against `fe99a0f` for physical layout, hammer collision
  mesh, all step controllers, step definitions and measured motion planning.
- Full production Demo1 physical playback passed, without reset or skipping any
  stage. The previous passing report and video are retained; this run uses the
  separate `assembly-clean` prefix.
- All three supporting arms had bilateral contact at the actual hammer strike;
  the hammer was returned onto both support blocks and all 28 joints returned
  home. Whole-sequence contact audit violations: zero; engine warning counters:
  all zero. Maximum held hammer grip-center error: 4.019 mm. Maximum pin/beam
  penetration: 0.079 mm; final shaft radial error: 0.048 mm. Audit thresholds are
  unchanged from the baseline, not relaxed for the new presentation.

Images: [overview](../../artifacts/screenshots/assembly-clean-overview.png),
[detail](../../artifacts/screenshots/assembly-clean-detail.png),
[tools](../../artifacts/screenshots/assembly-clean-tools.png),
[Assembly1 editor](../../artifacts/screenshots/assembly-clean-editor.png).

Evidence: [complete video](../../artifacts/videos/assembly-clean-complete.webm),
[physical report](../../artifacts/reports/assembly-clean-physical-audit.json),
[completed scene](../../artifacts/screenshots/assembly-clean-complete.png).

Status: first, non-contact presentation round completed and physically verified.
The user approved committing and pushing this round. Deployment status is
tracked by the repository's GitHub Pages workflow.
