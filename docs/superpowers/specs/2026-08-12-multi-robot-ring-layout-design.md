# Multi-Robot Ring Layout Design

## Goal

Convert each selectable scene in `0.our-example` from a single-robot showcase into a physically shared multi-robot workspace:

- Franka Panda: four arms surrounding central graspable cubes.
- SO-101: four arms surrounding central graspable cubes on a shared work surface.
- XLeRobot: two complete robots facing one another across a table whose top is level with the arm mounting bases.

## Chosen architecture

Each scene remains one `MujocoCanvas` and one compiled MuJoCo model. Robot instances are duplicated inside MJCF with MuJoCo's `replicate` meta-element so body, joint, actuator, equality, sensor, and asset references receive unique namespaces automatically. Three.js-only clones and separate canvases are excluded because they would not share contacts or a physical workspace.

The upstream robot files remain remote assets. `SceneConfig.xmlPatches` wraps each upstream root body in a `replicate`/`frame` transform, avoiding checked-in copies of large meshes. Layout values and patch strings live in focused scene-layout helpers; `configs.ts` assembles those helpers with camera, IK, and object settings.

## Coordinate convention

All scenes use MuJoCo's Z-up convention and place the collaboration center at world `(0, 0)`.

### Franka Panda

- Four fixed Panda bases lie on a ring of radius `0.72 m`.
- The first base is at `(0, -0.72, 0)` and faces `+Y` toward the center.
- Three additional instances are generated with cumulative yaw increments of `90°`.
- Three graspable cubes are centered around `(0, 0)` at floor height.
- Camera framing shows the complete four-arm ring and central cubes on initial load.

### SO-101

- The shared square work surface is centered at `(0, 0)` and has a top height of `0.80 m`.
- Four fixed SO-101 bases lie on the work surface at a radius of `0.34 m`.
- The first base is at `(0, 0.34, 0.80)` and its local forward direction points toward the center.
- Three additional instances are generated with cumulative yaw increments of `90°`.
- Graspable cubes sit near `(0, 0)` with their bottoms supported by the work surface.
- Camera framing shows all four arms, the complete work surface, and the cubes.

### XLeRobot

- Two complete XLeRobot bodies are positioned symmetrically at `(0.85, 0, 0.38)` and `(-0.85, 0, 0.38)`.
- Their yaw angles differ by `180°`, so they face one another along the X axis.
- The arm mounting base height is `0.38 + 0.395 = 0.775 m`.
- The central table top is exactly `0.775 m` high, with a thin top and four visible legs rather than a solid block.
- The table is centered at `(0, 0)` and leaves clearance for both mobile bases.
- Camera framing shows both complete robots and the central table.

## Runtime behavior

All replicated robots are genuine MuJoCo bodies with independent namespaced joints and actuators. Existing controls continue to operate the primary instance in this layout milestone; the namespace structure deliberately preserves a clean path for later per-arm selection and coordinated controllers without rebuilding the scene.

The first IK target is renamed to the first replicated TCP site. Home-control arrays are repeated for every physical instance so all robots load in a stable, deliberate pose instead of leaving later instances at raw defaults.

## Tests and visual acceptance

Automated contract tests verify:

- Franka and SO-101 each use `count="4"` replication with `90°` yaw increments.
- XLeRobot uses `count="2"` replication with `180°` yaw increments.
- home-control arrays cover every replicated actuator group.
- graspable objects are centered for Franka and SO-101.
- XLeRobot table-top geometry reaches exactly `0.775 m` and has four legs.
- first-instance TCP/gripper names match the generated namespace.

Verification also includes TypeScript checking, a production Vite build, and browser screenshots of all three robot selections. A visual pass must confirm complete robots, inward-facing orientations, uncluttered center objects, and table/base height alignment.

## Deployment

After verification, rebuild the checked-in `docs/` GitHub Pages output, commit source, tests, design/plan records, and generated deployment assets, then push the current `main` branch through its configured SSH remote. No force push or history rewrite is permitted.
