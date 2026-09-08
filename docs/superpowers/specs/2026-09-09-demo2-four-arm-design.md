# Demo2 four-arm sorting design

The user authorizes continuous execution through two-arm collision avoidance to a four-arm result, after publishing the accepted reseating increment (`284d045`). No intermediate approval pause is required.

## Scope and architecture

Retain the existing 16 mixed eggs, four sourced grippers, four class-owned trays and both single-egg buttons. Add a separate four-arm controller and checked motion package. Preserve Demo1/Assembly1, all other scenes, default entry and existing scene physics.

Generate per-egg actuator trajectories using the existing native IK/contact workflow, then validate concurrent schedules in the web engine. Known simulator class metadata selects eggs; this is not camera-based recognition. Each arm owns its corresponding appearance class (four eggs each). Each egg owns exactly one tray cell. No object pose/velocity writes after initialization, welds or invisible supports.

Use local shared-space reservations derived from checked whole-arm approach/exit paths. Conflicting paths wait at verified poses; outside-box transport and placement overlap where safe. A completely serial global lock is only a diagnostic fallback, not the final concurrent demonstration. Compare against an intentionally conflicting request, prove that the scheduler delays it, and record a physically verified simultaneous interval.

Runtime timing is fixed to the checked schedule, including explicit close/release/settle dwells. Waiting occurs at HOME before a source corridor lease. Only opposite arms approaching opposite near-side quadrants may overlap source leases; far-side crossed reaches must yield. Do not extend one arm's dwell online while others follow the old schedule: if its physical gate fails at the checked deadline, stop safely through actuator targets and report the reason. This is a safety abort, not a routine global scheduling barrier.

## Alternatives and choice

Private quadrants simplify motion but violate the mixed-box task. A global lock is safe but hides collaboration. Local checked path scheduling retains mixed classes and permits measured parallel work, so it is selected. Start with deterministic offline paths plus runtime contact/pose gates, not synchronous browser IK or an unvalidated general planner.

## Acceptance

- Two-arm test: successful real grasps and placements, overlapping motion, explicit conflict wait/release, no deadlock.
- Four-arm test: all four arms perform real work; all 16 eggs reach their class tray, four each, then all arms return home.
- Bilateral contact before lift, lift above 60 mm, no sustained grasp loss over 0.12 s; final support, no finger contact, tilt below 20 degrees, cell error below 12 mm.
- Landing load transfer may remove one jaw contact only after real tray support within 3 mm of the intended cell, tilt below 20 degrees and speed below 8 mm/s. This does not exempt any in-flight grasp from bilateral-contact checks.
- Forbidden penetration below 0.1 mm and finger/egg penetration below 1 mm. Check all robot bodies and held eggs, not TCP separation alone. No disabled collisions to manufacture success.
- Preserve separate post-placement correction trial; automatically stop if a new unverified outcome would need an unknown correction path.
- English UI with continuous start, Reset, per-arm state and sorted count. Existing manual controls remain available when automation is idle.
- Native and WASM replay, actual browser run/screenshots, Reset, original baseline tests and production build. Record failures honestly and fix physical paths before export.

## Delivery

Implement on `feat/demo2-four-arm-sorting` in the existing checkout to retain the user's local workflow. The pushed main branch is the checkpoint. This is a tightly coupled physical-planning/controller extension executed inline; use an independent read-only review at integration. Update dated progress and project tracking before the final handoff.
