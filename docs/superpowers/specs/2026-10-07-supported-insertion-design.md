# Franka Demo6 — supported insertion and dispatch

## Brief

After publishing the verified Demo3–5 increments, the user requested the next
demo. Implement candidate B from the recorded proposals, in a separate English
`Franka Demo6` page (`insertion`). Preserve all old scenes and Demo1 default.
Continue inline without another approval loop, following the user's standing
request. This is a physical demonstration and benchmark seed, not model training.

## Chosen assets and alternatives

Use robosuite's existing `StandWithMount` and `HookFrame` constructors and their
real hollow mount geometry. Export their generated MJCF, preserve source code,
MIT license, source hashes and constructor parameters. This is reuse of the
benchmark's parameterized assets, not newly invented prop geometry.

Alternative robot-attached peg/hole examples omit real grasping and are not
used. A new industrial CAD search would add licensing/conversion work without
first validating insertion; defer that visual upgrade. Do not inherit ToolHang's
50000 kg/m³ base density. Use explicit aluminium-equivalent 2700 kg/m³ geometry
and polymer-equivalent 1000 kg/m³ grip density as simulation assumptions.

Start with a graspable narrow base, a real four-wall socket and an L-shaped
insert with its source grip. Source dimensions may be parameter-adjusted for
Panda clearance; record every value. No invisible handle, fixed stand, weld,
attachment, scripted object following, or object force override.

## Layout and complete task

- Four existing Panda robots on a 0.75 m radius ring, table Z=0.1 m. This new
  workcell's 30 mm inward base adjustment provides horizontal-wrist overlap;
  prior workcells retain their original 0.78 m ring.
- Free installation stand near the center/south, resting on a visible narrow
  support pad that leaves both base grasp edges accessible.
- Incoming insert lies flat near Arm 3; an east staging pad is reachable by
  Arms 3/2. The west output support is reachable by Arm 4.
- Arm 3 physically picks and stages the insert, releases and clears.
- Arm 1 grips the stand's accessible base region to stabilize it without
  blocking the socket mouth.
- Arm 2 picks the staged insert, reorients it upright, aligns its actual tip
  to the actual socket and lowers until the source grip shoulder is supported.
  Check depth, lateral alignment, axes, actual contact and settled speed.
- Arm 2 opens and retracts along a clear direction; Arm 1 then releases and
  retreats. A settling interval verifies the assembly remains inserted.
- Arm 4 grips the stand, lifts/transfers the assembled component to the west
  output pad, releases, and returns HOME. The insert stays supported by its
  socket through gravity and contacts, never by a hidden connection.
- Finish only when assembly is still seated, the stand is on the output
  support, all fingers clear and all four arms HOME.

First validate one controlled layout. If a grip or path is blocked, diagnose
contact pairs and geometry before changing source dimensions or choreography;
never relax gates to hide penetration or false grasps. Table support is legal:
the demonstration does not assert that four arms are strictly necessary.

## Interfaces and physical gates

Add `insertion` scene and an `inserted` gate with `object`, `socket`, local
`tip`, local `mouth`, local unit `axis` and `socketAxis`, `minDepth`, `maxDepth`,
`maxLateral`, `minAlignment`, and `maxSpeed`. Observation is actual tip/mouth
relative displacement projected onto the socket axis, axis alignment, real
object/socket contact and body speed. All inputs/observations must be finite;
being near the hole without contact, above it, off-axis or moving is failure.
Mirror the same geometry in native/WASM. Add optional initial object quaternion
checks to reject stale manually rotated insertion parts even at the same center.
No orientation reset during playback.

Optional plan `contactLimits:[{a,b,maxPenetration}]` enforces the socket/insert
pair throughout all phases, including approach and transport, not just at gates.
Record its peak separately from robot contacts. Old plans omit this field.

Retain existing robot-only actuator control, contact/carry checks, safety stop,
Play/Pause/Reset, manual arm selection and scene lifecycle. Source collisions
must match the visible hollow socket. Check insert/socket penetration separately
through the rollout, not only robot penetration.

## Acceptance

1. CPU-native and WASM compile a self-contained scene with two independent free
   task bodies, 32 Panda actuators and a physical open socket.
2. Full actuator-driven cycle passes real grasp, insertion, release, assembly
   transport, final support and HOME checks. No unintended robot penetration
   over the existing 1 mm gate; insertion pair penetration stays below 1 mm.
3. Native/WASM reject off-axis, shallow, unsupported and stale-orientation cases.
4. Rendered scene and complete browser playback, Pause/Reset/scene switching;
   screenshots at initial, insertion and complete states; no browser errors.
5. Full regression, typecheck and production build; independent final review.
6. Detailed source/parameter/action/diagnostic records. New Demo6 is local for
   inspection unless the user separately requests its publication.
