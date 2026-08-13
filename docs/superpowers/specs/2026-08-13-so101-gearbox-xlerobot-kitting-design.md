# SO101 Gearbox and XLeRobot Kitting Scene Design

## Objective

Add two independent selectable scenes without changing the existing `SO101` or `XLeRobot` layouts:

- `SO101 Gearbox`: four SO101 arms surround a precision two-stage gearbox assembly fixture.
- `XLeRobot Kitting`: two complete mobile dual-arm XLeRobots face a narrow kitting table for tote unloading, handoff, handheld scanning, order packing, and shared tray transport.

Both scenes are static staging scenes in this iteration. They must already contain the complete task geometry needed by later continuous-motion demos, but no scripted object motion, attachment, or automatic task execution is added.

## Isolation Contract

- Existing exports `SO101_LAYOUT` and `XLEROBOT_LAYOUT` remain unchanged.
- New geometry and XML patches live in a separate `src/collaborativeSceneLayouts.js` module with matching declarations.
- New runtime keys are `so101Gearbox` and `xlerobotKitting`.
- Existing five scene entries, their labels, their physical placements, and their controllers remain unchanged.
- This implementation remains uncommitted and unpushed until the user explicitly authorizes Git operations.

## SO101 Gearbox Scene

The scene keeps a `1.04 × 1.04 m` work surface at `z=0.80 m` and four SO101 bases at north, east, south, and west positions `0.34 m` from the table center. A fixed `0.18 × 0.18 m` fixture occupies the center.

The north station contains a movable open gearbox lower housing. The east station contains two horizontal shafts and two spacers in anti-roll V racks. The south station contains three movable gears in separate shallow pockets. The west station contains a movable top cover and four upright press-fit pins. The lower housing, cover, gears, shafts, spacers, and pins have free joints and stable primitive/compound collision geometry.

Gear visuals use procedural teeth, while collision geometry is split around an open central bore so a shaft can physically pass through it. The housing and cover expose named alignment sites. The intended later collaboration is: one arm holds the housing; a second arm holds a gear bore aligned; a third inserts the shaft through housing and gear; the fourth positions the cover; two arms press the cover while two arms insert pins.

## XLeRobot Kitting Scene

The scene uses two complete XLeRobots facing one another along the world X axis. Their scene-specific initial centers are `(-1.10, 0, 0)` and `(1.10, 0, 0)` so later motion can drive them to docking positions near `x=±0.59 m`. A narrow table occupies the center with a `0.72 × 1.00 m` top at the existing arm base height `z=0.775 m`.

The table contains a source-tote staging bay at `(-0.18, -0.30)`, north and south handoff cradles near `(0, ±0.09)`, a handheld scanner dock at `(0.22, 0)`, and a divided order tray at `(0.10, 0.32)`. The source tote contains a pill bottle, tea box, drink carton, and bottle. Scene-specific payload decks with low safety rails are attached to both chassis models, but the tote and order tray remain ordinary free bodies supported by contact and friction.

The object appearances are recognizable procedural approximations in this first web scene, with dimensions based on local RoboTwin object categories `062_plasticbox`, `008_tray`, `024_scanner`, `080_pillbottle`, `112_tea-box`, `068_boxdrink`, and `114_bottle`. Named barcode and scanner sites establish a later deterministic scan contract. No VLM is required to decide whether a scan pose is valid.

The intended later collaboration is: both robots drive to their docks; Robot A unloads the source tote with both arms; Robot B unloads the order tray; A transfers individual goods through two anti-roll handoff cradles; one B arm presents each item while the other holds the scanner; both robots finally grasp opposite tray handles and share the loaded tray before Robot B seats it on its payload deck.

## Visual and Physical Constraints

- Use restrained industrial colors; no bright cyan centerpiece.
- No initial overlaps between free objects, trays, robot bodies, or table geometry.
- Round objects start in V cradles or tote pockets rather than unsupported on a flat surface.
- Every task-relevant visible object has collision geometry.
- No proximity attachment, magnetic grasp, equality weld for carried goods, scripted following, or automatic object movement.
- Scanner and barcode markers are sites or non-colliding details only; they do not move objects.

## Verification

- Test-first contracts prove two new exports, independent runtime entries, unchanged base layouts, correct instance counts, task stations, movable task parts, and absence of attachment shortcuts.
- Offline MuJoCo compilation must succeed for both new scenes with four SO101 roots or two XLeRobot roots.
- Browser smoke tests capture separate screenshots and verify the expected physical instance counts.
- Existing keyboard/IK control targets remain available for every new robot instance.
- Screenshot review checks object scale, orientation, station separation, central task readability, table/chassis clearance, and absence of initial penetration.
