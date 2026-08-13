# SO101 Gearbox / Home Lab Split Progress

Date: 2026-08-13

## Outcome

- Restored `SO101 Gearbox` as the compact pre-room four-arm precision gearbox workcell.
- Added `SO101 Home Lab` as an independently selectable scene; the gearbox workcell and task-part coordinates are shared without moving or enlarging them.
- Added the runtime key `so101HomeLab`, four independent SO101 control targets, offline validation, browser capture, and browser control verification.
- Kept the Home Lab G1 and Go2-with-arm as static visual assets without additional controls, free joints, scripted animation, or runtime downloads.
- Added no magnetic grasp, weld, proximity attachment, object following, or other carrying shortcut.

## Layout record

### SO101 Gearbox

- Floor half-size: `[2, 2, 0.005]`
- Camera: `[1.15, -1.15, 1.28]`, FOV `40`
- Orbit target: `[0, 0, 0.81]`
- Four SO101 bases remain on a `0.42 m` ring around the shared precision workcell.

### SO101 Home Lab

- Floor: 10 m × 8.4 m; three fixed walls; south side open to the camera.
- Protected center radius: `1.6 m`; large furnishings remain outside it.
- West lounge: detailed sofa cushions/seams/pillows, rug, coffee table with lower shelf, mug and remote, TV console, screen, stereo speakers, side table, and floor lamp.
- Northeast office: detailed desk, two display screens, keyboard, mouse and pad, books, PC tower and vents, cable tray, office chair, and wall organizer.
- Southeast service gallery: G1 display pad/status pedestal, Go2 charging pad/contact strips, safety markings, maintenance cabinet/drawers, wall tool board, and wheeled service cart.
- Room treatment: west and north framed windows, curtains and sill, east door/handle, baseboards, wall art, and two ceiling fixtures.
- Static robots: G1 at `[2.45, -0.9, 0]`, yaw `155°`; Go2-with-arm at `[3.55, -2.35, 0]`, yaw `150°`.

The Home Lab environment is isolated in `src/so101HomeLabEnvironment.js` so the compact scene does not inherit room geometry.

## Verification evidence

Strict MuJoCo compile and initial-contact checks:

| Scene | Bodies | Geoms | Actuators | qpos | Initial penetration |
|---|---:|---:|---:|---:|---:|
| SO101 Gearbox | 57 | 354 | 24 | 115 | 0 / 0 contacts |
| SO101 Home Lab | 148 | 619 | 24 | 115 | 0 / 0 contacts |

Browser control checks:

- `SO101 Gearbox`: Arm 1–4 keyboard + IK PASS.
- `SO101 Home Lab`: Arm 1–4 keyboard + IK PASS.

Regression checks:

- `npm test`: 70 / 70 tests passed at the implementation gate.
- `npx tsc --noEmit`: passed.
- production Vite build: passed with 691 transformed modules.
- `git diff --check`: passed.

## Visual checkpoints

- Compact scene: [`artifacts/screenshots/so101-gearbox.png`](../../artifacts/screenshots/so101-gearbox.png)
- Full Home Lab: [`artifacts/screenshots/so101-home-lab.png`](../../artifacts/screenshots/so101-home-lab.png)

The screenshot pass confirmed that the central four-arm workcell remains unobstructed, the lounge and office read as separate zones, and both static robot displays are visible without blocking the south-side circulation path.
