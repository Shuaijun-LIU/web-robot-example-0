# Third-party assets

This directory combines the Franka Emika Panda model from MuJoCo Menagerie with
three converted tool meshes from RoboTwin.

- `scene.xml`, `panda.xml`, and `assets/`: copied from
  `google-deepmind/mujoco_menagerie/franka_emika_panda`. See the upstream
  notices copied with those files.
- `tools/robotwin-screwdriver.obj`: converted from
  `RoboTwin/assets/objects/032_screwdriver/visual/base0.glb`.
- `tools/robotwin-drill.obj`: converted from
  `RoboTwin/assets/objects/030_drill/visual/base6.glb`.
- `tools/robotwin-hammer.obj`: converted from
  `RoboTwin/assets/objects/020_hammer/visual/base0.glb`.

The matching PNG files are the base-color textures embedded in those same GLB
files. `scripts/convert-robotwin-tools.py` applies GLB node transforms, rotates
the authored +Y tool axis onto local +X, centers each mesh, exports the complete
OBJ geometry and source texture, then classifies its textured triangles into
`primary`, `dark`, and `metal` OBJ partitions. These partitions preserve visible
color blocking in the current web renderer, which does not consume mesh UVs.
The drill and hammer are enlarged slightly from RoboTwin's task scale to match a
realistic Panda workcell tool envelope; MuJoCo collision uses documented simple
primitive proxies rather than these high-detail visual meshes.

RoboTwin is licensed under the MIT License, Copyright (c) 2025 Tianxing Chen.
The complete license text is included as `ROBOWTWIN_LICENSE` in this directory.
