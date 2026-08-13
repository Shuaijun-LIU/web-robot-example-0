# Collaboration Scene Detail and Home Environment Progress

Date: 2026-08-13

## Result

The two independent collaboration scenes were refined without changing the original `SO101` or `XLeRobot` layouts.

- `SO101 Gearbox` now uses a collision-validated four-arm radius of `0.42 m`. The movable housing, cover, shafts, gears, spacers, and press pins gained annular bearing seats, true open spacer bores, shaft keys, housing ribs, cover pin sockets, and gear keyway indicators.
- `XLeRobot Kitting` now uses two opposing robots at `x=±0.50 m`, rear payload decks, and a recessed three-piece kitting island. The tote, four products, scanner, handoff cradles, and order tray gained recognizable physical and visual detail.
- The XLeRobot scene is now a complete open-plan home environment: north kitchen line with sink/faucet, preparation counter, stove/oven and pantry; south dining table with a movable produce crate and three movable vegetables; west storage shelf with bins; open camera-facing circulation area.

All movable task objects remain ordinary contact-driven free bodies. No weld, magnet, proximity attachment, automatic following, or scripted object motion was added.

## Distance and Contact Evidence

SO101:

- Base radius: `0.42 m`.
- Assigned outer-station distance: `0.21 m`.
- Nominal kinematic chain length from model offsets: approximately `0.455 m`.
- Home TCP inner-ring radius: `0.1366 m`.
- The rejected `0.34 m` candidate produced 79 initial penetrating contacts; the accepted `0.42 m` layout produces 0.

XLeRobot:

- Robot centers: `x=±0.50 m`.
- Chassis-to-island static clearance: `0.05 m`.
- Inward arm-base-to-center distance: `0.41 m`; nominal arm reach: `0.413 m`.
- North and south island circulation: `0.65 m`; west circulation: `1.11 m`.
- Initial penetrating contacts: 0.

## Verification Evidence

- Full Node suite: 45/45 passed.
- TypeScript: `npx tsc --noEmit` passed.
- Production Vite build passed (688 modules transformed).
- Offline MuJoCo compile and strict initial-contact report:
  - `so101Gearbox`: 57 bodies, 354 geoms, 24 actuators, 115 qpos, 0 initial penetrating contacts.
  - `xlerobotKitting`: 92 bodies, 324 geoms, 32 actuators, 123 qpos, 0 initial penetrating contacts.
- Browser scene roots: 4 SO101 bases and 2 XLeRobot chassis roots detected.
- Control regression:
  - SO101 Gearbox Arm 1–4: independent keyboard and IK passed.
  - XLeRobot Kitting Robot 1–2: independent keyboard control passed.
- Browser screenshots were captured at `1440×900` and visually inspected:
  - `artifacts/screenshots/so101-gearbox.png`
  - `artifacts/screenshots/xlerobot-kitting.png`

## Repository State

The implementation, tests, screenshots, design/plan records, and this progress record are intentionally left uncommitted and unpushed. This follows the instruction to avoid Git conflicts with other agents until a later explicit commit/push command.
