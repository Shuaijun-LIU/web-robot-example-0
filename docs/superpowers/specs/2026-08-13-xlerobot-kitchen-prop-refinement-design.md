# XLeRobot Kitchen Prop Refinement Design

Date: 2026-08-13

## Objective

Correct four visual and physical deficiencies in the independent `XLeRobot Kitting` scene without changing either robot pose, the recessed kitting island, existing furniture placement, navigation clearances, or any other scene.

## Faucet Lever

The current lever begins at local `x=0.04 m`, while the faucet riser radius is only `0.018 m`; this leaves a visible gap between the lever and the faucet body. The refined faucet adds a short cylindrical pivot/connector seated against the riser and starts the lever from that connector. The connector, lever, riser, and base form one continuous fixed assembly.

## Recessed Saucepan

The current saucepan is a solid cylinder and therefore reads as a puck rather than cookware. It will be replaced by:

- A thin circular bottom plate.
- Twelve annular wall segments that leave the center physically open.
- A slightly wider upper rim.
- A connected handle with a grip cap.
- A dark interior surface treatment that makes the cavity legible from the scene camera.

The saucepan remains fixed on the existing rear-left burner. No free joint or scripted motion is introduced.

## Southeast Trash Bin

A fixed kitchen trash bin will occupy the unused southeast corner, away from the island-to-kitchen, island-to-dining-table, and island-to-storage-shelf routes. It will use a restrained graphite palette and contain a base, four side walls, an upper rim, a visibly recessed opening, a rear lid hinge, and a front foot pedal. It must not reduce any recorded navigation clearance.

## Produce-Table Additions

The unoccupied right side of the dining/produce table will receive two task-relevant fixtures:

- A fixed digital kitchen scale with platform, body, and display.
- A fixed preparation bowl built from a bottom and annular wall segments so the bowl has a real open cavity rather than a solid-cylinder silhouette.

These additions support later weighing, sorting, and handoff actions while leaving the existing free produce crate, tomato, cucumber, and bell pepper unobstructed.

## Physical and Visual Constraints

- No weld, magnet, proximity attachment, automatic following, or scripted object motion.
- No changes to XLeRobot attachment frames, arm controls, island geometry, or current task-station coordinates.
- All new fixed geometry must compile in MuJoCo and create no initial penetrating contacts.
- The revised scene must remain readable in the existing `1440×900` overview camera.
- Existing unrelated local changes from other agents remain untouched.

## Verification

- Test-first contracts require a faucet connector, annular saucepan walls and rim, southeast trash-bin details, and the scale/bowl details.
- The contracts prohibit a solid cylindrical `saucepan_body` and preserve all existing free-body and no-shortcut guarantees.
- Offline MuJoCo compilation runs with strict initial-contact reporting.
- The browser scene is recaptured and visually inspected for connection, cavity visibility, scale, placement, and navigation clearance.
- The complete Node test suite, TypeScript check, and production build run after the focused verification.
