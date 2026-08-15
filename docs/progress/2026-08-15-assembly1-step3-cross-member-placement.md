# Assembly1 Step 3 Cross-Member Placement — 2026-08-15

## Implemented behavior

- A third Assembly1 action is gated behind the completed physical clamp step.
- Arm 1 stabilizes the frame from an offset south-rail station and Arm 2 retains the side-laid torque driver.
- Arms 3 and 4 keep their physical cross-member clamps, lift together, carry the beam into the center, descend, align all four interfaces, and hold it in place.
- The action writes only Panda actuator controls. It does not write object poses or velocities and uses no weld, magnet, proximity attachment, or scripted following.

## Geometry and stability corrections

- The installed beam now rests on top of the frame: frame top `z=0.260 m`, beam body target `z=0.278 m`, beam bottom `z=0.260 m`.
- Arm 1's frame station moved from world `x=0` to `x=0.18 m`, clearing the center path without changing its stabilizing role.
- Both beam grasp stations have physical stop ribs and top caps to retain the Panda fingers during transport.
- Final load compensation uses TCP targets `[0.006, 0.1275, 0.278]` and `[0.002, -0.1275, 0.278]`; the declared part target remains `[0, 0, 0.278]`.

## Blank-scene root cause

The disappearing scene was traced to leaked embind handles from per-frame `data.contact` reads. Long runs exhausted the fixed 2 GiB MuJoCo WASM heap and aborted rendering. Contact sampling now deletes each temporary contact handle and the vector wrapper in `finally`, and regression tests cover both normal and exceptional cleanup.

## Measured browser verification

- Two consecutive full Step 1 → Step 2 → Step 3 runs reached `complete` with no browser/WASM errors.
- Run 1: hole distances `2.44–2.61 mm`, frame drift `4.47 mm`, beam rotation `0.66°`.
- Run 2: hole distances `4.90–6.38 mm`, frame drift `4.88 mm`, beam rotation `0.30°`.
- The separate Step 2 production-browser regression also passed.
- Visual result: `artifacts/screenshots/franka-assembly1-step3-aligned-hold.png`.

## Boundary

Step 3 intentionally stops with both transport arms holding the aligned beam. Fastener insertion, tool handover, and torque application belong to the next action.
