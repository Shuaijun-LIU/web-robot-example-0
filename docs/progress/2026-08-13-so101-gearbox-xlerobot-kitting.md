# SO101 Gearbox and XLeRobot Kitting Progress

Date: 2026-08-13

## Result

Two additional selectable static collaboration scenes are implemented without replacing or modifying the existing `SO101` and `XLeRobot` layouts:

- `SO101 Gearbox` (`so101Gearbox`): four independently controllable SO101 arms, a central fixture, movable open lower housing, movable perforated cover, two anti-roll shaft racks, two shafts, two spacers, three open-bore toothed gears, and four individually movable press pins in supported slots.
- `XLeRobot Kitting` (`xlerobotKitting`): two complete independently controllable mobile dual-arm robots, a `0.72 × 1.00 m` table at `z=0.775 m`, two chassis payload decks, a free source tote, four recognizable free products, two handoff cradles, a free handheld scanner and dock, and a free divided order tray with bilateral handles.

All carried/task objects use contact and friction. There is no equality weld, magnetic grasp, proximity attachment, automatic following, or scripted object motion.

## Spatial Contract

SO101 workstations:

- Central fixture: `(0, 0)`
- Housing: `(0, +0.21)`
- Shafts and spacers: `(+0.21, 0)`
- Gears: `(0, -0.21)`
- Cover and pins: `(-0.21, 0)`

XLeRobot workstations:

- Robot starts: `(-1.10, 0)` and `(+1.10, 0)`
- Source tote: `(-0.18, -0.30)`
- Handoff cradles: `(0, -0.09)` and `(0, +0.09)`
- Scanner dock: `(+0.22, 0)`
- Order tray: `(+0.10, +0.32)`

## Verification Evidence

- Test-first RED: collaboration layout test initially failed with `ERR_MODULE_NOT_FOUND`; integration test initially failed because both runtime entries were absent.
- Focused layout contracts: 6/6 passed.
- Focused runtime integration contracts: 4/4 passed.
- Full suite: 44/44 passed.
- TypeScript: `npx tsc --noEmit` passed.
- Offline MuJoCo compile:
  - `so101Gearbox`: 57 bodies, 278 geoms, 24 actuators, 115 qpos.
  - `xlerobotKitting`: 71 bodies, 217 geoms, 32 actuators, 95 qpos.
- Browser scene roots: 4 SO101 bases and 2 XLeRobot chassis roots detected.
- Control regression:
  - `so101Gearbox`: Arm 1–4 keyboard and IK passed independently.
  - `xlerobotKitting`: Robot 1–2 keyboard control passed independently.
- Production Vite build passed with Node 22.
- Screenshots:
  - `artifacts/screenshots/so101-gearbox.png`
  - `artifacts/screenshots/xlerobot-kitting.png`

## Repository State

The implementation, tests, records, and screenshots are intentionally left uncommitted and unpushed. This avoids conflicting with concurrent work and follows the explicit instruction to wait for a later Git operation command.
