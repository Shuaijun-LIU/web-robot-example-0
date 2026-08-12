# Dual Franka Assembly Detail Design

## Objective

Replace the single `Franka Assembly` selector entry with two independently selectable comparison scenes:

- `Franka Assembly1`: physically detailed, self-contained MuJoCo geometry.
- `Franka Assembly2`: the same workcell and assembly interfaces, but with real external tool meshes from the local RoboTwin asset pack.

The original `Franka Panda`, `SO101`, and `XLeRobot` scenes remain unchanged. Both assembly scenes remain static staging scenes; no scripted robot motion is added.

## Shared Workcell Contract

Both scenes use the same four Panda poses, `0.90 m` ring radius, platform, tool-zone coordinates, frame dimensions, loose cross-member, mounting plate, fastener tray, and handover pad. They also share an explicit installation contract:

- The loose cross-member is intended to span the north and south frame rails at the center of the frame.
- Each cross-member end has two open mounting holes, for four holes total.
- The frame has four matching recessed receiver positions.
- Named MuJoCo sites `frame_receiver_nw`, `frame_receiver_ne`, `frame_receiver_sw`, and `frame_receiver_se` identify the target interfaces.
- Corresponding sites `cross_member_hole_nw`, `cross_member_hole_ne`, `cross_member_hole_sw`, and `cross_member_hole_se` are attached to the loose cross-member.
- At the documented target pose, each cross-member site coincides with its matching frame receiver. This gives later grasping, insertion, stabilization, and fastening code an exact geometric contract.

## Shared Frame Detail

The rectangular frame changes from four plain solid bars to a recognizable aluminum extrusion assembly:

- Each rail has separated silver flanges and dark recessed center channels.
- Side slot strips suggest T-slot aluminum extrusion rather than painted rectangular tubing.
- Corner gussets and installed corner fasteners remain visible.
- Four cross-member receiver recesses are visible and aligned with the interface sites.
- The cross-member uses the same extrusion language and has perforated end tabs built around actual openings rather than painted circles.

The frame remains one free rigid body supported by four blocks. The loose cross-member remains one free rigid body in the parts tray.

## Franka Assembly1: Procedural Physical Detail

Assembly1 uses only MJCF primitives and inline procedural meshes, so it has no additional runtime asset downloads beyond Franka itself.

### Manual screwdriver

- A true eight-sided handle mesh replaces the round cylinder.
- Alternating octagonal radii create several shallow axial grip recesses.
- The collision hull retains eight flat faces, reducing the tendency to roll on the tool mat.
- A collar, steel shaft, and tip remain distinct physical parts of the same free body.

### Powered torque driver

The current blocky upright tool becomes a recognizable cordless driver with a rounded motor housing, gearbox, torque selector ring, chuck, bit, angled pistol grip, trigger, side vents, and a broad battery foot. The battery rests above the mat without initial penetration and provides a stable base.

### Hammer

A movable claw hammer is added on its own low tool shelf. It has a steel striking face, head, neck, split curved claw, and faceted non-slip handle. Its initial pose is horizontal and fully supported, avoiding a top-heavy standing configuration while keeping the third tool visible beside the right-hand arm.

## Franka Assembly2: External RoboTwin Tool Assets

Assembly2 reuses the shared detailed frame and interfaces but replaces the three procedural tool appearances with locally available RoboTwin meshes:

- `assets/objects/032_screwdriver/visual/base0.glb`
- `assets/objects/030_drill/visual/base6.glb`
- `assets/objects/020_hammer/visual/base0.glb`

The selected drill variant is the compact `base6` model. Its RoboTwin task scale gives an envelope of approximately `0.046 × 0.156 × 0.131 m`; the web workcell enlarges it to approximately `0.056 × 0.199 × 0.165 m` so its cordless-driver silhouette reads clearly beside a full-size Panda while remaining graspable.

MuJoCo WASM does not load GLB directly. The chosen visual GLBs are converted once into repository-owned OBJ assets with UV coordinates, and their embedded base-color textures are extracted as PNG. Each tool body uses a non-colliding textured visual mesh plus a conservative primitive collision proxy. RoboTwin scale metadata is the baseline; the drill and hammer receive one documented display-scale adjustment for realistic workcell proportions.

Assembly2 uses a local Vite public asset root containing the required Franka files plus the converted tool meshes. A third-party notice records the upstream RoboTwin MIT license and source paths. No large dataset, unused object variant, or original GLB archive is committed.

## Code Boundaries

Assembly-specific generation moves into `src/frankaAssemblyLayouts.js` with a matching declaration file. The module owns the shared workcell geometry, installation interface data, and the two layout exports. `src/sceneLayouts.js` continues to own only the original three layouts and exposes the Franka home pose needed by the assembly module. `src/configs.ts` registers the two comparison scenes and reuses the existing Franka control targets and controller family.

## Stability and Error Handling

- Free tools start with a positive clearance above their mat or cradle; no body begins in penetration.
- Assembly1 uses flat contact faces or broad bases. Assembly2 pairs each detailed visual mesh with a simple, stable physical shape.
- Missing local Assembly2 assets must surface as the existing scene loading error rather than silently falling back to visual-only geometry.
- A scene is not accepted if MuJoCo cannot compile every mesh, body, actuator, and named interface site.

## Verification

- Test-first structural contracts require two assembly layout exports, exact selector labels, four targets per scene, and paired interface sites.
- A numeric alignment test applies the documented cross-member target pose and verifies all four local hole coordinates match the frame receiver coordinates.
- Offline MuJoCo validation compiles both scenes and reports named bodies/sites.
- Browser captures verify four physical Panda roots in each scene and produce separate screenshots.
- Control regression covers all four arms in both assembly scenes in addition to the original three scenes.
- Visual inspection checks tool scale/orientation, screwdriver facets and grooves, drill silhouette, hammer silhouette, extrusion slots, actual cross-member openings, and absence of initial overlaps.

## Success Criteria

The selector exposes five scenes in total. `Franka Assembly1` and `Franka Assembly2` have identical robot placement and assembly interfaces but visibly different tool-asset strategies. Both compile, remain stable at rest, preserve independent four-arm control, and are documented with separate screenshots suitable for direct comparison.
