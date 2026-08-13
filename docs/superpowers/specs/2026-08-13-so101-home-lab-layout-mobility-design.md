# SO101 Home Lab Layout and Mobility Design

Date: 2026-08-13

## Goal

Refine only the expanded `SO101 Home Lab` scene: remove the clipping sofa props,
improve the sofa construction, restore readable lounge clearances, extend the
floor grid across the full room, move the complete four-SO101 workcell into the
open southwest zone, and make the G1 and Go2-with-arm selectable mobile devices.
The compact `SO101 Gearbox` scene remains unchanged.

## Spatial decisions

- The workcell center moves from `[0, 0]` to `[-2.25, -1.85]`. One MJCF frame
  translates all four arms, fixtures, and loose gearbox parts together; the
  table top and legs receive the identical translation through scene-object
  coordinates.
- The lounge uses a sofa at `[-4.30, 1.35]`, coffee table at
  `[-2.95, 1.35]`, and TV console at `[-1.82, 1.35]`. The side table moves to
  `[-4.30, 2.85]`, leaving visible clearance from the sofa arm.
- Loose sofa pillows are removed. The sofa is rebuilt from a base, apron, seat
  deck, paired seat cushions, paired integrated back cushions, arms, seams,
  back shell, and feet.
- The WebGL grid is scene-configurable. Home Lab uses a 10 m grid with 100
  subdivisions, covering the room rather than only its former center.

## Mobility decisions

The vendored G1 and Go2-with-arm visual hierarchies remain fixed-pose assets,
but each is attached beneath a three-degree-of-freedom parent body: world X
slide, world Y slide, and yaw hinge. Six MuJoCo velocity actuators provide real
simulated planar motion. This is a browser-friendly whole-body mobility proxy;
it does not fake locomotion by teleporting Three.js objects and does not add
scripted trajectories, automatic following, magnetic attachment, or gait
animation.

`SO101 Home Lab` exposes six control targets: Arm 1–4, G1, and Go2 + Arm.
Selecting either mobile robot switches W/S to forward/reverse and A/D to
left/right turning. Forward velocity is resolved through the current heading,
so the robots travel in their own facing direction. The original four-arm IK
and keyboard controls are retained with actuator indices shifted by the six
new mobile actuators in the compiled MJCF.

## Verification contract

- Home Lab compiles with 30 actuators and four SO101 instances.
- The four SO101 roots and task coordinates are translated together.
- Initial penetrating-contact count remains zero.
- Arm 1–4 retain isolated keyboard and IK control.
- G1 and Go2 + Arm each change only their own three-actuator block; W/S changes
  planar position and A/D changes the corresponding yaw joint.
- A 1440 × 900 screenshot shows full-room grid coverage, readable lounge
  spacing, and the workcell in the southwest zone.
