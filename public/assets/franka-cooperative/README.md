# Cooperative workcell assets

This self-contained package serves Franka Demo3 (product inspection/packing)
and Franka Demo4 (two-arm pot support with solid ingredient loading).

## Attribution and licenses

- **Panda:** [MuJoCo Menagerie / Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda), Apache-2.0; copy in `licenses/Panda.txt`. Original meshes and arm inertias retained. Changes to the MJCF: named TCP, removed source keyframe, renamed gripper actuator, finite 40 N gripper force limit with 1000 N/m position stiffness and friction 1.0 on fingertip collision surfaces. These are simulation control parameters, not a hardware performance claim.
- **Scanner, tea box, coffee box and tray:** RoboTwin Team's [official asset distribution](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0), categories `024_scanner`, `112_tea-box`, `113_coffee-box`, `008_tray`, instance 0. The distribution card declares MIT; local asset card and code license are included in `licenses/`. Source visuals, textures and separate supplied collision pieces are reused. Source GLB node transforms are applied, then Y-up becomes Z-up; original metadata scales are retained. No PartNet object is included.
- **Pot:** RoboCasa / Lightwheel AI, `lightwheel/pot/Pot054`, from [RoboCasa assets](https://huggingface.co/datasets/robocasa/robocasa-assets). **CC BY 4.0** under the [official project asset license](https://github.com/robocasa/robocasa#license). Visual geometry, texture and all 38 source convex collision pieces retained; uniformly scaled to 65% and recentered on the bottom. No lid or replacement geometry is added.
- **Carrot and tomato:** RoboCasa AIGen, `carrot/carrot_0` and `tomato/tomato_0`, **CC BY 4.0**, same source/license as above. Original source scale/reference rotation and 32 convex pieces per object retained. These are existing benchmark meshes, not assets generated for this project.

CC BY 4.0: [license deed](https://creativecommons.org/licenses/by/4.0/) and [legal code](https://creativecommons.org/licenses/by/4.0/legalcode.en). Attribution does not imply endorsement.

## Conversion and physical assumptions

`scripts/build-cooperative-assets.py` exports source geometry to OBJ for MuJoCo,
binds the original textures explicitly, and downsamples textures to at most
1024 pixels per side. No object shape is remodeled. `manifest.json` records
source identifiers/hashes, bounds, scales, collision counts and initial poses.
The same transform is applied to visuals and collisions. Existing source
collision approximations are measured independently in the native report.

All task objects are free rigid bodies. Masses are task assumptions, not
measured physical properties: scanner 130 g; tea/coffee boxes 90/100 g;
tray 120 g; pot 550 g; carrot/tomato 55/70 g. Inertias use conservative
bounding-box estimates. Only fixtures (table, work pads) are simple primitives.
No weld, magnet, object-following constraint or scripted free-body pose is used.

The pot's collision cavity is tested by dropping a physical probe and requiring
it to rest on the inner floor below the rim. A whole-pot convex hull is never
used. Barcode inspection will be a geometric pose gate, not image recognition.

## Rebuild

Use the existing MuJoCo Menagerie, RoboTwin and RoboCasa asset checkouts:

```bash
uv run --with trimesh --with pillow python scripts/build-cooperative-assets.py \
  --menagerie PATH_TO_MENAGERIE --robotwin PATH_TO_ROBOTWIN \
  --robocasa-objects PATH_TO_ROBOCASA_OBJECTS
uv run --with 'mujoco==3.3.7' --with trimesh --with pillow \
  python scripts/verify-cooperative-assets.py
```

Automatic task readiness is reported by each page; displaying these assets
alone does not establish successful task execution.
