# SO101 Home Lab Layout and Mobility Progress

Date: 2026-08-13

## Implemented

- Removed both loose sofa pillows that clipped the upholstery and rebuilt the
  sofa with separated structural layers and integrated paired cushions.
- Opened the lounge spacing by moving the side table, coffee table, TV console,
  TV, speakers, rug, and lamp as a coordinated layout.
- Extended the rendered grid from 4 m to 10 m for Home Lab, with 100 divisions.
- Moved the full four-arm gearbox island and its table to southwest center
  `[-2.25, -1.85]`, preserving all relative arm/task geometry.
- Added MuJoCo-backed X/Y/yaw mobility for G1 and Go2-with-arm. Both appear in
  the existing control-target selector and use W/S plus A/D after selection.

## Verification evidence

- MuJoCo 3.3.8: 150 bodies, 621 geoms, 30 actuators, 121 qpos.
- Initial penetration: 0 / 0 contacts.
- Four SO101 roots compile around the translated center and retain their TCP
  positions relative to the task island.
- G1 and Go2 + Arm browser W/S movement checks: PASS.
- Screenshot: `artifacts/screenshots/so101-home-lab.png`.

The mobility layer is simulated planar whole-body motion for the fixed-pose
visual models. It deliberately does not claim articulated walking or gait
control.
