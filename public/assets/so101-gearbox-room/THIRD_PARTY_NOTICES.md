# Third-party assets

This directory contains scene assets redistributed for the browser-based
SO101 Gearbox room.

## SO101 model

- Source: `Vector-Wangel/MuJoCo-GS-Web`,
  `assets/robots/xlerobot/SO101.xml` and its referenced meshes.
- Upstream MIT license is reproduced in `LICENSE_MUJOCO_GS_WEB`.
- Local changes: none to the mesh files; the model is patched at runtime to
  replicate four SO101 arms and add the TCP site used by this demo.

## Unitree G1

- Source: `google-deepmind/mujoco_menagerie`, `unitree_g1`.
- Upstream asset copyright and license are reproduced in
  `robots/g1/LICENSE`.
- Local changes: `g1_static.xml` removes joints, actuators, sensors, collision
  proxies, and the keyframe so the high-fidelity mesh hierarchy is a fixed
  room-display model. Mesh files are unmodified.

## UniLab Go2 with arm

- Source: `unilabsim/UniLab`,
  `src/unilab/assets/robots/go2_arm` and the referenced Go2 meshes.
- Upstream Apache-2.0 license is reproduced in
  `robots/go2_arm/LICENSE`.
- Local changes: `go2_arm_static.xml` removes joints, actuators, sensors,
  contact exclusions, and collision proxies; Go2 mesh paths are relocated
  beneath `assets/go2`. Mesh files are unmodified.
