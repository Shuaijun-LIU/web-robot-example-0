# Collaboration Scene Detail and Home Environment Design

## Objective

Refine the task-object detail in the independent `SO101 Gearbox` and `XLeRobot Kitting` scenes, verify that their operational distances match the physical arm envelopes, and expand only `XLeRobot Kitting` into an open-plan domestic kitchen/dining environment. Existing `SO101`, `XLeRobot`, Franka, and Franka Assembly scenes remain unchanged.

## SO101 Gearbox Refinement

The four SO101 bases use a radius of `0.42 m`; the five existing workstations remain at their current coordinates. The initial `0.34 m` candidate produced 79 penetrating contacts between bases, jaws, and staged parts, so it was rejected after MuJoCo contact inspection. At `0.42 m`, each arm is `0.21 m` from its assigned outer parts station, the center remains inside the approximately `0.455 m` nominal chain length, and the home TCPs form a collision-free inner ring of radius `0.1366 m`.

The refinement adds mechanically readable details without changing task semantics:

- Annular bearing-seat geometry around the two housing and cover shaft openings.
- Open-bore annular spacers instead of solid cylinders.
- Shaft shoulders and longitudinal key features.
- Gear hub/keyway indicators while preserving physically open bores.
- Housing exterior ribs and four cover pin-socket indicators.
- More legible tray/pocket separation and unique fixed-object names.

All housing, cover, shafts, spacers, gears, and press pins remain ordinary free bodies driven only by collision, friction, and gravity.

## XLeRobot Operational Island

The central table keeps the maximum `0.72 × 1.00 m` footprint but becomes a recessed island rather than a full rectangle. A `0.48 m`-wide central spine runs the full length; two `0.72 m`-wide end wings support the source tote and order tray. This creates side docking recesses for the mobile bases.

The two robot centers move to `x=±0.50 m`. With the chassis half-depth of `0.21 m` and central tabletop half-width of `0.24 m`, the static side clearance is `0.05 m`. The inward arm bases are approximately `x=±0.41 m`; the shared handoff area is therefore within the approximately `0.41 m` kinematic chain length. Payload decks move from the inward face to the rear face of each chassis so they cannot penetrate the island during docking.

The kitting stations are redistributed only as needed for the recessed top:

- Source tote: `(-0.15, -0.36)` on the south wing.
- Handoff cradles: `(0, ±0.10)` on the central spine.
- Scanner dock: `(+0.16, 0)` on the central spine.
- Order tray: `(+0.10, +0.36)` on the north wing.

The source tote gains ribs, a divider, and label plate. Products gain recognizable labels, closures, or package details. The scanner gains a status button, speaker slots, and battery seam. The order tray gains corner bumpers while retaining dividers and bilateral robot-grasp handles.

## Open-Plan Home Environment

The island remains the primary collaboration zone. Fixed furniture forms three additional task zones with circulation lanes no narrower than `0.65 m` around the island:

- North kitchen line at `y≈1.45 m`: sink cabinet with a recessed basin and faucet, preparation drawers and cutting board, four-burner stove and oven, and a tall pantry cabinet.
- South dining/preparation table at `y≈-1.45 m`: a produce crate with individually movable tomato, cucumber, and bell pepper.
- West storage shelf at `x≈-1.65 m`: multiple levels and labeled storage bins for final delivery.

The furniture uses restrained warm wood, off-white, graphite, and muted metal colors. No bright cyan is introduced. The back wall and floor establish a room envelope while leaving the camera-facing side open.

This space supports a later continuous task sequence: dock at the island, unload and scan groceries, exchange objects through the two cradles, drive to the dining table for produce, transfer produce to the sink/prep zone, and place sorted goods on the storage shelf or pantry.

## Physical Constraints

- No equality weld, magnetic grasp, proximity attachment, scripted following, or automatic object motion.
- Task objects remain free bodies; furniture, docking markers, and storage receivers are fixed.
- Table/chassis clearance is at least `0.05 m` at the default docks.
- The island center is within the physical arm-chain envelope from each default dock.
- Navigation gaps north and south of the island are at least `0.65 m`.
- Existing scene layout exports and default page scene remain unchanged.
- No commit or push is performed until the user explicitly requests it.
- Both refined scenes must report zero initial penetrating contacts at their configured home states.

## Verification

- Test-first contracts cover unique names, new mechanical details, recessed island dimensions, docking clearance, reach envelope, furniture inventory, circulation gaps, product details, and absence of attachment shortcuts.
- Both layouts must compile through MuJoCo.
- `INITIAL_CONTACT_REPORT=1 INITIAL_CONTACT_STRICT=1` validates the configured home state and rejects penetration deeper than `5 mm`; the accepted layouts report zero penetrating contacts.
- Browser screenshots are captured at `1440×900` and inspected for object scale, spacing, visibility, and initial stability.
- SO101 Arm 1–4 keep independent keyboard/IK control; XLeRobot Robot 1–2 keep independent mobile/arm control.
- TypeScript, full tests, and the production build must pass.
