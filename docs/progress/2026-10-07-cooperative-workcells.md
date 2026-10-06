# Cooperative workcells — 2026-10-07

## Requested delivery order

Demo2 optimization was committed and pushed first as `0c18da6` after a fresh
238/238 test run, type check and production build. GitHub's matching build and
artifact upload also passed; deployment status is checked separately.
New work is on `feat/franka-scan-and-pot`, not in that published release.

Design/acceptance: [spec](../superpowers/specs/2026-10-07-cooperative-workcells.md).
Implementation: [plan](../superpowers/plans/2026-10-07-cooperative-workcells.md).

## Stage 1 — physical layouts

Added two separate pages, keeping Demo1 as default:

- **Franka Demo3:** RoboTwin scanner, tea box, coffee box and tray, with incoming,
  inspection/handoff, packing and dispatch regions.
- **Franka Demo4:** RoboCasa/Lightwheel Pot054, a source carrot and tomato, and
  two preparation trays. Opposite arms are assigned the handles; the other two
  own the ingredient areas. Pot uses 65% uniform scale and all 38 original
  convex collision pieces; it is not a solid hull or a newly modeled pot.

The four Panda roots are mounted at tabletop height and retain separate manual
IK/keyboard control targets. Objects are free bodies with gravity and physical
contacts. This stage is explicitly a **layout preview**, not completed automatic
motion; later stages must pass the full grasp/transfer/load gates.

### Evidence

- Native MuJoCo 3.3.7: both scenes compile with 32 actuators and four TCPs.
- Five-second idle test: no meaningful robot penetration (maximum numeric
  residual 2.6e-18 m); all task objects drift less than 0.003 mm in the final
  second of the native check.
- Collision/visual bounding-box difference: 0.45–2.72 mm depending on the
  benchmark's existing approximation; recorded per asset, not assumed exact.
- A free 20 mm sphere drops through the actual pot opening and rests on the
  inner floor 21.84 mm above the pot origin, proving the collision cavity is open.
- Browser loading/switching/Reset test and screenshots exercise actual WASM.
- Early screenshot exposed an upstream renderer limitation: OBJ UVs and
  texture maps were not displayed. A failing UV-retention check reproduced
  export loss, and a scene-scoped texture adapter now supplies the source maps
  without changing geometry, physics or old scenes. A separate test verifies
  per-face UV seams and restoration on cleanup.

Reports: `artifacts/reports/cooperative-assets-native.json`,
`artifacts/reports/cooperative-browser-layouts.json`.

![Demo3 layout](../../artifacts/screenshots/frankaDemo3-layout-2026-10-07.png)

![Demo4 layout](../../artifacts/screenshots/frankaDemo4-layout-2026-10-07.png)

## Next gates recorded at the layout checkpoint

1. Actuator-only runtime with contact, release/support and failure checks.
2. Product presentation → scanner pose → supported handoff → packing/dispatch.
3. Bilateral pot grasp → cooperative lift/hold → alternating loading → set-down.
4. Full native/WASM replay, rendered critical-stage inspection and old-scene regression.

## Stage 2 — complete actuator programs

Both automatic programs now pass native MuJoCo and independent replay through
the actual MuJoCo WASM engine. Browser-rendered checks and the full old-scene
regression run are tracked separately below; these numbers are not a claim
about arbitrary manual perturbations or a learned manipulation policy.

| Task | Phases / simulation time | Native → WASM result | Maximum unintended robot contact penetration (WASM) |
|---|---|---|---|
| Demo3 inspection and packing | 66 / 130.5 s | Pass → Pass | 0 mm |
| Demo4 cooperative pot loading | 27 / 67.1 s | Pass → Pass | 0.452 mm |

### Demo3 actions and design decisions

Arm 1 presents each box; Arm 2 physically holds the scanner and aligns its
source optical-head point with the presented face. The inspection gate checks
distance, off-axis error and facing; it does not decode a barcode. Arm 1
releases onto the handoff pad and clears it before Arm 3 picks and packs.
The tea and coffee boxes use different grasp heights. Placement verifies
support before opening the fingers.

After packing, Arm 4 uses two spread fingertips to push the outside tray wall
along a continuous support bed into dispatch. **This action is a supported
push, not a grasp or airborne carry.** The source rim can be pinched, but trials
showed that airborne cantilever loading tipped the tray and spilled the boxes;
rim pinching during sliding also wedged the rim. The tested outside-wall push
keeps both products supported, without adding a handle, weld or extra force.
Final gates require both products inside/contacting the tray, the tray resting
on dispatch, and all arms returned home. Maximum intended contact penetration
in WASM is 0.977 mm.

### Demo4 actions and design decisions

Arms 1/3 insert partially-open fingers into the opposed handle loops, recenter,
close with bilateral contact and lift about 35 mm. Arms 2/4 pick the carrot and
tomato from separate trays and drop them in from high east/west approach lanes.
Those lanes keep the loader wrists above the holding wrists. Both ingredients
must settle inside the real pot cavity with physical support.

The holding pair then lowers the loaded pot. Release reverses the safe insertion
path: partial opening, radial withdrawal, vertical clearance, then full opening.
Simply opening fully and lifting caught the handle loops in testing; that
unsuccessful route is not exported. Final gates verify both foods inside, pot
support, zero finger contact and all arms home. Maximum intended grip
penetration in WASM is 0.360 mm.

### Reproduce motion verification

```bash
uv run --with 'mujoco==3.3.7' --with numpy python scripts/solve-cooperative-tasks.py --scene scan
uv run --with 'mujoco==3.3.7' --with numpy python scripts/solve-cooperative-tasks.py --scene pot
node --test test/cooperative-wasm.test.mjs
```

Motion reports are `artifacts/reports/cooperative-{scan,pot}-{native,wasm}.json`.
The player writes only actuator controls. Each physics tick checks actual
contacts; lost bilateral grasps or excessive robot contact stop the motion
while leaving the rendered scene available for inspection.
