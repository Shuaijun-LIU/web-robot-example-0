# Demo2 — Four-arm mixed-egg sorting

Date: 2026-09-08
Status: stage A approved by the user and implemented on 2026-09-08; static scene available locally for review. No automatic manipulation controller has been implemented. See [implementation result](../docs/progress/2026-09-08-demo2-static.md). Sections below retain the design rationale; the progress report supersedes the original resource-investigation status.

## 1. Previous demonstration closure

The user has marked Demo1 and Assembly as temporarily complete. Freeze Demo1 / Assembly1 at `019afd25d2f1f93aa08866330634f936b29f2818` (`feat(assembly): add RoboDojo wrench to Arm 1 pad`). Leave Assembly2 and every other existing scene unchanged as well. This is a user-accepted milestone, not a claim that all historical diagnostic caveats disappeared. Detailed verification remains in `docs/progress/`, including `2026-09-08-robodojo-wrench.md`.

Demo2 must have its own scene assets, manipulation parameters and controller state. Reuse established loaders, controls, cameras and rendering infrastructure where safe, but do not modify the accepted assembly motion targets or shared gripper physics indiscriminately. Keep the current default landing page until explicitly changed.

## 2. User requirement and proposed task

Four Franka Panda arms surround one shared box containing multiple types of eggs. Each arm selects its assigned type and places it in its own destination tray. Incorrectly seated/tilted eggs are corrected. Longer, egg-appropriate fingertips should come from existing hardware/model resources. The demonstration must show coordinated simultaneous work and collision avoidance in a restricted shared space.

Proposed first layout:

- One central shallow box with cushioning/low separators; begin with a full single layer, tentatively 16 eggs (four appearance classes, four each). Final count and dimensions follow verified asset size, reach and finger clearance, not an arbitrary fixed box size.
- Interleave all classes throughout the source box. Do not give each arm a private pre-sorted quarter of the box.
- Put one destination tray in each arm's accessible outer work sector, clear of neighboring elbows and source-box approaches. Match tray capacity and cavity size to its eggs.
- Keep four natural-looking classes distinguishable through existing shape/texture resources. Exact species are not yet selected; do not label recolored identical meshes as verified chicken/duck/quail assets.
- Support eggs physically while leaving enough exposed side surface and approach space for the sourced fingers. Padding must not hide an impossible grasp or be an invisible collider.

Each arm owns one type. Selection considers reach, neighboring eggs and current reservations; within-type nearest feasible picks reduce unnecessary crossings. Outside-box transfers and independent tray placements can proceed in parallel.

## 3. Approaches considered

| Approach | Benefit | Limitation | Decision |
| --- | --- | --- | --- |
| Four fixed private picking quadrants | Easiest first motion | Removes most shared-box contention and may make assigned types unreachable | Not the main demonstration |
| One global lock for all picking | Simple safe fallback | Makes box work entirely sequential | Recovery/debug fallback only |
| Mixed box plus local time/space reservations | Shows real parallel work with explicit yielding where paths conflict | Requires whole-arm path checks and tested waiting poses | Recommended target |

Start with deterministic geometry/state-based control. The first version need not introduce a vision model or claim visual classification: use known asset class metadata and simulator poses explicitly. Wrist/global cameras can display the scene; perception-based classification may be a separate later extension.

## 4. Action and coordination contract

1. Select an accessible egg of the assigned class; reserve its approach/grasp/exit corridor and an empty destination cell.
2. Move from safe hover into the box, align fingers to the egg and close with conservative force. Require physical two-sided contact and adequate opening; do not accept an empty-air grasp.
3. Lift clear of neighboring eggs, padding and box walls before moving sideways. Confirm the egg is actually carried through contact.
4. Transport to the assigned tray and align the egg's long axis to the cell. Lower gently until supported; open and withdraw through a checked path.
5. If the egg is tilted or perched on a rim, regrasp, lift enough to clear the rim, correct orientation and lower again. Do not force rotation while wedged in the tray. For an approximately axisymmetric egg, yaw about its long axis is not itself an orientation error.
6. Verify class, cell occupancy, seating height, tilt and post-release stability; then release the reservations and select the next egg.

Collision checks must include links/elbows, wrist-camera hardware if installed, fingers, held eggs, neighboring eggs, the box and trays—not only TCP distances. Allow concurrent picks only when their swept volumes do not conflict. Conflicts trigger a visible hold/yield at verified hover positions, not last-second interpenetration. Include a deterministic conflict case and at least one visibly parallel non-conflicting interval in the later acceptance run.

Demonstrate correction via a documented initial condition or physically executed placement perturbation, never by teleporting/rotating a released egg to manufacture an error. Rigid-body contact with a soft-contact approximation is not shell fracture or a validated deformable-finger simulation; any damage metric would initially be a clearly labeled force proxy.

## 5. Resource investigation: verified vs pending

### Local eggs and containers

Asset paths below are relative to the named benchmark's root so the proposal remains portable.

| Source | Verified local resource | What is established | Still to check |
| --- | --- | --- | --- |
| RoboDojo | `Assets/Object/RoboDojo/Rigid/egg/00000/object.usdz` | Egg asset exists; metadata bounding box approximately 41.1 × 42.2 × 57.6 mm | Shell appearance, collision shape, provenance/license and suitable mass |
| RoboDojo | `Assets/Object/RoboDojo/Articulation/egg_holder/00000/object.usdz` | Articulated holder exists; bounding box approximately 90.4 × 120.9 × 125.6 mm | Actual open configuration, cavity count/spacing and finger access; not yet confirmed as a suitable output tray |
| RoboDojo | `Assets/Object/RoboDojo/Geometry/egg_basket/00000/object.usdz` | Asset exists; description identifies a shallow woven natural-fiber basket | Not the requested padded rectangular source box; only an alternative visual reference |
| RoboCasa | `robocasa/models/assets/objects/objaverse/egg/egg_*` and `aigen_objs/egg/egg_*` | Multiple local egg models with XML, visual meshes/textures and collision meshes | Select intact natural eggs, audit dimensions/species claims/licenses and collider complexity |

RoboDojo's `task/RoboDojo/config/fill_egg_holder.yml` and corresponding task code provide an existing egg-to-holder example. Its four eggs and articulated-holder completion condition do not constitute our four-arm sorting task. Reuse assets and placement references, not an unsupported claim of a ready-made controller. Official task overview: [RoboDojo](https://robodojo-benchmark.com/).

Important physical audit: the RoboDojo egg metadata declares mass `0.3 kg`. Do not copy that setting automatically. Egg mass/inertia, contact stiffness, friction and grip force must be justified together for the selected geometry. Some RoboCasa eggs have many collision pieces; inspect for clean collision surfaces and manageable contact cost before importing a full box.

The padded source box/liner is still an asset gap. Search existing benchmark container/packing assets first; importing a basket is not completion of this requirement. Verify each imported asset's redistribution terms and preserve attribution before publishing.

### Gripper candidates

Primary candidate: existing UMI soft fingers and the WSG50 robot-deployment assembly. The [official UMI hardware guide](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU/edit) provides these actual CAD references:

- [Soft finger CAD](https://cad.onshape.com/documents/c8d06048fc2e425772f2749c).
- [UMI handheld gripper CAD](https://cad.onshape.com/documents/a8a98201374f5cc9c6538790).
- [WSG50–Franka mounting CAD](https://cad.onshape.com/documents/10e70b8dbf6f9fe17c716739).
- [Official Franka deployment instructions](https://github.com/real-stanford/universal_manipulation_interface/blob/main/franka_instruction.md).

The guide specifies TPU fingers and a separate WSG50 robot setup. The handheld teaching device is not the same assembly as the robot end effector. Finding these links does not yet establish downloaded mesh availability, publication license, compatibility with the current Panda hand, or a validated egg grasp.

Evaluate two routes: replace fingertips only if a real compatible mount is available, or import the complete supported gripper plus Franka adapter. Do not simply stretch the Panda's existing visual mesh while leaving short collision fingers behind. Existing local Robotiq 2F85 meshes are a fallback candidate, but their bulk and contact force may make them unsuitable for dense egg packing; test rather than assume.

First asset gate: inspect fingertip dimensions, aperture, mount offsets, TCP, pad contact geometry, box entry clearance and camera clearance. Only then choose final box spacing. A longer finger alone does not guarantee stable or gentle grasping.

## 6. Incremental delivery and acceptance

### A. Static scene — next proposed implementation

Deliver one isolated development scene, source attribution and screenshots from overhead and grasp-height viewpoints. Four arms, mounted candidate grippers, a mixed source box with padding and four destination trays. No full automatic action sequence yet.

Gate: visually/physically aligned geometry; no initial penetration/floating supports; realistic scale and orientation; reachable grasp/withdrawal spaces; user scene review before motion expansion.

### B. One arm, one egg

Implement approach, contact grasp, lift, transfer, seating, opening/retreat and one meaningful orientation correction. Measure actual finger/egg contacts, grip aperture, slip, neighbor disturbance, object support and stability after release.

Gate: no welds, magnetic/proximity attachments, hidden grip bodies, scripted object following or post-spawn object pose writes (except explicit Reset). Do not transplant Assembly1's high-force settings. Fix real geometry/contact defects before tuning acceptance thresholds.

### C. Two arms, then four

Validate conflict detection, waiting and release with two arms first. Expand to four-arm sorting only after both parallel and conflicting paths work. Maintain per-egg identity/class/cell records and log collisions, drops, corrective attempts and completion counts.

Gate: no forbidden inter-arm/environment collisions; clear parallel work where feasible; bounded waits without deadlock; correct sorted counts; stable supported eggs; responsive interaction during planning.

### D. Continuous Demo2

Add a minimal English presentation page with Play, Reset and scene switching, analogous to Demo1. Keep the development scene's diagnostic/manual-pose facilities separate. Preserve the accepted Demo1/Assembly scenes and current default entry, and regression-check them before release.

## 7. Current result and next decision

Original design-turn result: closure record, local resource inventory, official gripper/CAD references and this proposed task/layout/validation breakdown. Subsequent user approval authorized stage A; the resulting scene is now `Franka Demo2`. Grasping and avoidance are still not implemented or claimed.

Next review: inspect the static result before commissioning the manipulation sequence. The initial model uses source-derived appearance classes and a UMI simulation wrist adaptation; confirm layout/asset suitability before physical grasp development.
