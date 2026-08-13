# SO101 Home Lab Layout and Mobility Implementation Plan

Date: 2026-08-13

## Tasks

- [x] Add failing contracts for the translated workcell, sofa cleanup and
  details, lounge coordinates, full-room grid, mobile targets, and planar
  velocity math.
- [x] Rebuild the sofa and move the side table, coffee table, TV console,
  screen, speakers, rug, and floor lamp to preserve visible clearances.
- [x] Remove both under-robot pads and rebuild the service cart with four
  floor-contacting wheels.
- [x] Translate the four SO101 arms, complete gearbox task, table, and task
  metadata together to `[-2.25, -1.85]`.
- [x] Add scene-specific grid size/division configuration and set Home Lab to
  10 m / 100 divisions.
- [x] Wrap G1 and Go2-with-arm in planar X/Y/yaw roots and add six velocity
  actuators.
- [x] Add selectable G1 and Go2 + Arm control targets, heading-aware keyboard
  control, and target-specific help text.
- [x] Update browser control diagnostics for non-uniform actuator blocks and
  physical position/yaw checks.
- [x] Compile the MJCF, run initial-contact checks, build the app, run TypeScript
  and regression tests, verify browser controls, and inspect the new screenshot.
