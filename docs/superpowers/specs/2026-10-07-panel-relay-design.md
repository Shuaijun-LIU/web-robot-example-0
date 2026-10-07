# Franka Demo7 — panel relay over an obstacle

## Brief and scope

The user requests publishing the accepted Demo6, then completing Demo7. Continue
candidate C (panel/tray obstacle relay), retaining all old scenes and the Demo1
default. Deliver a separate English Franka Demo7 with a complete physical action
cycle, source assets, visible screenshots and reproducible checks. The standing
instruction is autonomous execution without repeated approval prompts.

Subsequent request: optimize and publish Demo7. Retain the approved physical
design; overlap safe opposing-arm travel instead of accelerating contact
motions, compact the Demo7-only panel and inspect its actual default view.

## Assets and alternatives

Use the existing RoboTwin `104_board` instance 3: a solid wooden panel without
the cutting-board handle hole of instance 0. Retain its texture and shape;
uniform scaling is allowed, not new handles or a newly modeled payload.
Official previews: https://robotwin-platform.github.io/doc/objects/104_board.html
Official asset distribution declares MIT:
https://huggingface.co/datasets/TianxingChen/RoboTwin2.0/blob/main/README.md
Preserve source hashes, original metadata and license attribution.

The upstream collision envelope exceeds the visible panel by about 3.5 mm at
0.2 scale. Do not inherit that invisible padding. Derive a convex collision
mesh from the solid panel's visible mesh, record the conversion, and inspect
the top/bottom contact surfaces at the selected grips. Preserve holes when
reusing other objects; never convexify a hollow tray as if it were solid.

Use a sourced RoboTwin wooden block as a fixed, visible workcell obstacle;
table, ordinary start/end supports remain simple physical pads. Alternative
008_tray has an already observed tipping problem at its rim; do not infer its
airborne feasibility from Demo3 pushing. A new CAD tray is unnecessary for
this first empty-panel relay. Loaded trays and narrow-roof corridors are later
tasks, not hidden additions to this milestone.

## Choreography

Start from a supported horizontal panel south of the obstacle. Use opposing
Arms 1/3 as donors and opposing Arms 2/4 as receivers, rather than initially
loading two adjacent edges. This is a documented role adjustment to candidate C
to keep the center of mass between separated grasps.

1. Arms 1/3 approach the actual south/north edge, jaws closing across thickness.
2. Confirm bilateral finger contact on both arms; lift together clear of pads.
3. Move to a shared relay pose; Arms 2/4 approach/enter together at distinct
   east/west edge regions, then close and verify separately.
4. Verify BOTH new grips before Arm 1 releases, holding its open hand in place.
5. With the opposite receiving pair established, Arm 3 releases. Both donor
   arms then withdraw radially and return HOME together.
6. Arms 2/4 transport the panel over the real obstacle with visible clearance,
   lower it onto the north output support, confirm contact, release and retreat.
7. All four arms finish HOME with the free panel supported at the destination.

At least two physically verified grips carry the panel through the relay. A
new receiver's established contact precedes its donor's release. Coupled moves
derive both TCP targets from one panel transform, not independent guesses.
The initially proposed alternating handover (one receiver, then one donor
release) was rejected by simulation: adjacent support permitted board tilt.
Use 4→3→2 support transfer instead. Final isolated layout: 0.85 m robot ring,
start Y=-0.05 m, output Y=0.38 m, support tops Z=0.22 m. The obstacle uses seven
unchanged 50-mm source cubes: two three-cube outer posts and a low middle cube
at Y=0.16 m. Tall posts require lifting; the center leaves the north palm clear.
The real Panda palms need this access. Grip direction is 35° below horizontal,
not the initially attempted horizontal or unstable steep 55° approach.
No free-body pose writes, welds, magnetic grasps or object force overrides.
Source Panda force limits remain unchanged. If a pose is unreachable or a sweep
collides, adjust the isolated layout/path and record why; do not hide it by
weakening the gates. Overhead passage is a legal geometric choice, not a rule
forbidding other physical solutions. A single-arm solution is a future baseline,
not a reason to claim four arms are necessary.

## Interfaces and safeguards

- New scene `relay`, key `frankaDemo7`, existing cooperative provider/UI.
- Existing grasp/carry, support, released, HOME gates and quaternion preconditions.
- Add finite `upright` gate: object local axis against world up, minimum alignment,
  maximum settled linear speed. Initial threshold cos(10°); use the same formula
  in native and WASM. No absolute-speed stop during intended transport.
- Pair contact limits guard panel/obstacle through every phase (1 mm maximum
  numerical penetration); contact-free clearance is additionally measured.
- Tests record every airborne tick's tilt, height and bilateral supporting arms,
  plus receiver-before-donor contact order; endpoint-only success is insufficient.
- Exact zero-force manifold entries occasionally disappear for a physics tick.
  Keep the raw bilateral minimum in reports. A missing entry is accepted only
  for a previously established, still-closed grip when the two actual MATERIAL
  surface witness points from contact within 6 ms remain within 50 micrometres.
  This is observation-only, not contact forces, attachment or a distance grasp.
  Do not trust exact-zero mesh `mj_geomDistance` queries; additionally certify
  obstacle clearance using separating planes of transformed collision vertices.
- Reset, pause, switch, stale manual poses and malformed/non-finite gate inputs
  retain the existing safety behavior. No old trajectory/asset regeneration.

## Acceptance

Source-derived scene compiles with one free payload, fixed visible supports and
obstacle, 32 robot actuators, no payload constraints. Inspect the static layout
before full choreography. Full native and WASM cycles complete with real grasps,
two sequential receiver transfers, obstacle clearance, output support and HOME.
Repeat after 3 seconds additional settling. Software-rendered browser playback,
Pause/Reset/switch and initial/relay/final screenshots pass without page/physics
errors. Run full tests, TypeScript and production build; one independent final
review. Detailed source, geometry, motion failures and measured outcomes are
recorded. The initial handoff stayed local after publishing Demo6. The user's
subsequent instruction now explicitly authorizes optimized Demo7 publication;
verify deployment as well. No benchmark training or randomized generalization
result is claimed.
