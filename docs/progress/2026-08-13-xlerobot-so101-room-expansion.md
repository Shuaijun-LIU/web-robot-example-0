# XLeRobot / SO101 room expansion progress — 2026-08-13

## Delivered scene changes

### XLeRobot Kitting

- Kept the existing north kitchen wall and added west/east walls while leaving
  the south side open for the two mobile robots.
- Added a framed west-wall window, two east-wall artworks, and a separate
  multi-door cabinet placed against the east wall.
- Preserved the existing kitting table, kitchen equipment, storage props,
  produce, scanner, task goods, and both controllable XLeRobot instances.
- Moved the overview camera to the open south side so all three walls and the
  central mobile-robot circulation area remain visible.

### SO101 Gearbox

- Preserved the central four-arm gearbox workcell, task stations, task parts,
  and all 24 SO101 actuators.
- Expanded the floor to a 9.2 m × 7.2 m room with north/west/east walls and an
  open south side.
- Added a west lounge with sofa, rug, TV, console, side table, and floor lamp.
- Added a northeast office with desk, keyboard, dual monitors, desk lamp, and
  chair, plus wall art.
- Added a high-detail Unitree G1 and a Go2 with manipulator as fixed display
  robots outside the protected central workcell. They intentionally have no
  joints, actuators, collision proxies, or UI control targets in this round.

## Reused assets

- SO101: `Vector-Wangel/MuJoCo-GS-Web` (MIT).
- Unitree G1: `google-deepmind/mujoco_menagerie` (upstream Unitree asset
  license included beside the local model).
- Go2 with arm: `unilabsim/UniLab` (Apache-2.0 license included beside the
  local model).

The SO101 and display-robot files required by SO101 Gearbox are vendored below
`public/assets/so101-gearbox-room/`, so this expanded scene does not depend on
runtime GitHub downloads.

## Verification

- Full Node test suite: 54/54 passed.
- TypeScript: `tsc --noEmit` passed.
- Production Vite bundle: passed.
- SO101 Gearbox MJCF: 130 bodies, 502 geoms, 24 actuators, 115 qpos;
  0 initial penetrating contacts.
- XLeRobot Kitting MJCF: 101 bodies, 426 geoms, 32 actuators, 123 qpos;
  0 initial penetrating contacts.
- SO101 Gearbox control isolation: all four arms passed keyboard and IK checks.
- XLeRobot Kitting control isolation: both robots passed keyboard checks.

## Visual checkpoints

- `artifacts/screenshots/so101-gearbox.png`
- `artifacts/screenshots/xlerobot-kitting.png`

Both checkpoints were captured at 1440 × 900 after successful browser-side
MJCF loading.

## XLeRobot visual follow-up

The subsequent kitchen pass separates the west storage shelf from the window:
the shelf now sits immediately south of the window with its back against the
west wall. The former east-wall cabinet is replaced by a pale refrigerator
with a distinct upper door, freezer drawer, handles, water dispenser, toe kick,
door seam, and badge.

The southeast trash bin is rotated 90 degrees so its front and pedal face west
toward the room. Its lid now opens approximately 58 degrees from the wall-side
hinge. The bin, refrigerator, shelf containers, digital produce scale, and open
preparation bowl use a lighter palette to improve silhouette and part
legibility.

Follow-up verification:

- XLeRobot Kitting MJCF: 101 bodies, 429 geoms, 32 actuators, 123 qpos;
  0 initial penetrating contacts.
- Focused collaboration-scene tests: 18/18 passed.
- TypeScript and production Vite build passed.
- Both XLeRobot keyboard-control blocks passed browser checks.
- Refreshed checkpoint: `artifacts/screenshots/xlerobot-kitting.png`.
