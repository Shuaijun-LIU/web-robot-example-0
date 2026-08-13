# SO101 Gearbox / Home Lab Split Design

Date: 2026-08-13

## Goal

Split the current expanded `SO101 Gearbox` entry into two independently
selectable scenes:

1. `SO101 Gearbox`: the exact compact four-arm gearbox workcell envelope that
   existed before the room expansion.
2. `SO101 Home Lab`: a substantially redesigned full-room scene containing the
   same four-arm gearbox island, detailed home/office furnishings, a static G1,
   and a static Go2 with arm.

The split must preserve all existing SO101 controls and task geometry and must
not introduce automatic attachment or scripted object motion.

## Scene Boundary

The two scenes share only the four SO101 arm attachments, TCP patch, gearbox
task assembly, home joints, task-station metadata, and reach metadata.

`SO101 Gearbox` injects only the SO101 model. Its floor is restored to
half-size `[2, 2, 0.005]`, its camera is restored to `[1.15, -1.15, 1.28]` with
FOV `40`, and its orbit target is `[0, 0, 0.81]`. It contains no room walls,
furniture, G1, or Go2.

`SO101 Home Lab` additionally injects the local static G1 and Go2-with-arm
models. It uses a 10 m × 8.4 m floor, three enclosing walls with the south side
open, and a protected 1.6 m central radius around the gearbox island.

## Home Lab Spatial Plan

- Central task island: unchanged gearbox table, four SO101 arms, fixtures, and
  loose assembly parts.
- West/northwest lounge: sofa against the west side, TV and console facing it,
  a coffee table between them, rug, lamp, speakers, books, mug, and remote.
- Northeast office: full desk, ergonomic chair, two monitors, keyboard, mouse
  and pad, PC tower, cable tray, books, desk lamp, and wall organization board.
- Southeast robot gallery: G1 and Go2-with-arm on distinct low display/charging
  pads, floor safety markings, status pedestals, a detailed maintenance cabinet,
  tool board, charging contact, and service accessories.
- Envelope: north/west/east walls, baseboards, a west-wall window, a north-wall
  window, a detailed east-side door treatment, artwork, and warm ceiling-light
  fixtures. The south side remains open for the overview camera.

Furniture and robot-zone centers remain outside the 1.6 m central protected
radius. A visible circulation band remains between the central workcell and
all perimeter furnishings.

## Visual Language

Use a muted warm residential palette with light oak, off-white, sage, warm
grey, and charcoal accents. Avoid saturated cyan/blue. Large forms must have
secondary construction details so they read as real objects rather than single
primitive blocks:

- upholstery seams, cushions, legs, and pillows;
- TV bezel, screen inset, speakers, stand, remote, and console doors;
- desk edge, drawers, monitor bezels/stands, input devices, PC vents, books,
  cable tray, and lamp;
- display-pad edge trim, warning bands, charging contacts, status lamps,
  maintenance cabinet doors/drawers, handles, tool silhouettes, and labels;
- window frames, glass, sill, curtain rail, and curtains;
- door frame, panel, handle, and threshold.

## Controls and Runtime Registration

Both scene entries use control family `so101`, expose exactly four targets
(`Arm 1` through `Arm 4`), use the local `so101-gearbox-room` asset base, and
retain 24 actuators. G1 and Go2 remain static visual models and do not appear in
the control-target selector.

Add `so101HomeLab` to runtime configuration, replicated-root diagnostics,
offline MJCF validation, browser capture, and browser control verification.
Its screenshot is `artifacts/screenshots/so101-home-lab.png`.

## Verification Contract

- Compact Gearbox compiles without `gearbox_room_*`, `room_g1_*`, or
  `room_go2_*` bodies and restores the exact compact floor/camera values.
- Home Lab compiles with all four SO101 instances plus named G1 and Go2 roots,
  retains exactly 24 actuators, and contains every required spatial zone.
- Both scenes have no initial penetration deeper than 5 mm; target outcome is
  zero penetrating contacts.
- Keyboard and IK checks pass independently for all four SO101 arms in both
  scenes.
- Browser screenshots at 1440 × 900 show clear task, circulation, lounge,
  office, and robot-gallery zones without wall or furniture occlusion.
- Full Node tests, TypeScript, and production Vite build pass.

## Scope Exclusions

- No animation or control for G1 or Go2 in this pass.
- No change to gearbox task coordinates, loose-part physics, or SO101 actuator
  mappings.
- No new external runtime downloads and no automatic grasp/attachment logic.
