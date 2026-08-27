# Assembly1 Step 4 Fastener Staging Design

## Goal

Make the existing Step 2 and Step 3 sequence robust to visually harmless contact and
height jitter, finish Step 3 with Arms 3/4 physically releasing the installed cross
member, and add a fourth action that stages the first fastener and powered tool for
tightening.

## Constraints

- Keep all object motion physical. Do not write free-body `qpos`, weld, magnetize,
  teleport, or use proximity attachment.
- Preserve the successful Step 1–3 paths except for the approved verification and
  release phases.
- Arm 1 keeps the frame clamped and Arm 2 keeps the torque driver clamped throughout.
- A failed check freezes safe arm positions and reports concrete evidence.
- Reset restores the original scene and sequence gating.

## Step 2 Contact Verification

The current controller requires simultaneous bilateral contact for an uninterrupted
`0.08 s`. MuJoCo may drop one fingertip from the current contact manifold for a frame
even while the object remains captive. Track the age of the last real target contact
for each fingertip and accept a contact only while that age is at most `0.20 s`.
Both sides must therefore have made real physical contact; they need not appear in
the same simulation frame.

Increase each verification deadline from `2.5 s` to `4.0 s`. Aperture, forbidden
contact, translation, rotation, and lift checks remain unchanged. The state machine
retains the most recent invalid verdict so a timeout cannot erase a useful `r1 /
missing-left-contact` diagnosis merely because the final sample is valid.

## Step 3 Alignment and Release

The existing hole metric is a three-dimensional Euclidean distance. Replace it with:

- maximum planar hole-center error: `0.030 m`;
- pre-release vertical interface offset: `0.025 m`;
- released-and-seated vertical interface offset: `0.020 m`;
- frame translation: `0.008 m`;
- cross-member total rotation remains diagnostic-only; four-hole XY/Z limits are the physical alignment check.

This keeps screw-axis alignment strict while allowing the seated beam height to be
judged independently. If alignment cannot become valid within `4.0 s`, perform one
physical reseat: raise Arms 3/4 to the existing descent-mid pose, descend slowly back
to the aligned pose, and verify again. A second timeout is terminal.

After one second of aligned hold:

1. keep Arms 3/4 at the aligned pose and open both grippers over `0.8 s`;
2. hold open for `0.5 s` so the beam settles on the frame;
3. retreat both arms to the existing hover pose over `1.5 s`;
4. require one continuous second of valid frame, rotation, planar-hole, and vertical
   placement evidence before completing Step 3.

The completed Step 3 state has gripper commands `[48, 96, 255, 255]` and Arms 3/4 at
known collision-free hover poses.

## Step 4 Scene Adjustment

Move the fastener tray center from `(0.56, 0.42)` to `(0.18, 0.48)` and move its four
fasteners by the same offset. This remains clear of the frame and old parts tray and
places the fasteners within Arm 3's reach. No Step 1–3 waypoint crosses this station.

## Step 4 Action: First Fastener Staging

The fourth button unlocks only after Step 3 completes.

1. **Concurrent preparation.** Arm 1 keeps its frame clamp. Arm 2 keeps its tool
   clamp and lifts the side-laid tool to a collision-free staging pose. Arm 3 moves
   above `fastener_1`. Arm 4 moves above the southwest cross-member plate.
2. **Fastener pickup and support.** Arm 3 descends and physically clamps the fastener
   head. Arm 4 descends to a light support pose on the opposite side of the beam.
3. **Transfer.** Arm 3 raises the fastener, transfers it above `frame_receiver_ne`,
   and descends until the shaft is inside the open plate cell. Arm 2 remains clear.
4. **Release.** Arm 3 opens its gripper and retreats. The fastener settles under
   gravity against the frame rail; its pose and nearby frame/beam stability are
   verified.
5. **Tool pre-drive.** Once Arm 3 has cleared the shared workspace, Arm 2 rotates and
   moves the torque driver to a safe pre-drive pose above the staged fastener. Arm 4
   maintains support and Arm 1 maintains the frame clamp.

Step 4 completes with the first fastener physically staged, Arm 2 ready for the next
tightening action, and no hidden attachment state.

## Verification

- Pure unit tests cover contact grace, timeout diagnostics, split alignment,
  one-reseat behavior, release/open/retreat phases, Step 4 phase ordering, control
  ownership, and reset gating.
- The offline MuJoCo IK solver validates every recorded target against joint limits,
  position error, and orientation error.
- Browser verification runs the complete 1→2→3→4 sequence and records final body,
  site, gripper, contact, and status diagnostics.

## 2026-08-27 Implementation Status

The state machine, UI, diagnostics, reachable tray, and offline IK contract are
implemented. Unit, type, and build verification pass. The full browser dynamics
gate is not yet green: r2 has made bilateral physical contact and has lifted the
fastener in trial runs, but contact is not retained reliably to the end of the lift.
No attachment or scripted pose fallback was added. Step 4 remains an implemented
but dynamically incomplete stage until the contact interface is corrected.
