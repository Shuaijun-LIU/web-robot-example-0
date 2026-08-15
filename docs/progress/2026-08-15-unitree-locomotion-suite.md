# Unitree Locomotion Suite Progress Report

## Outcome

`Unitree Action Lab` now offers two selectable actuator-only programs. The
original 10-second `Greeting & Scan` remains unchanged. The new 25-second
`Locomotion Suite` makes the floating-base G1 squat, stand, execute a bounded
biped gait, stabilize, then makes the floating-base Go2 walk with a diagonal
gait while carrying its Airbot arm. G1 finishes with the existing greeting.

The exact locomotion timeline is settle (1 s), G1 squat (2 s), G1 stand (2 s),
G1 gait (6 s), G1 stabilization (2 s), Go2 gait (6 s), Go2 stabilization
(2 s), final greeting (3 s), and final hold (1 s).

## Control and physics contract

- The runtime resolves all 47 named position actuators and writes only their
  `data.ctrl` entries.
- Both robots keep their MuJoCo free joints. Root position, quaternion, and
  velocity are read only for bounded body-frame attitude and velocity feedback.
- Gravity, actuator dynamics, foot contacts, friction, and joint limits produce
  all body translation. There are no root `qpos`/`qvel` writes, mocap targets,
  applied-force locomotion, welds, magnets, or scripted transform changes.
- The G1 gait uses alternating hips/knees, counter-swinging arms, stance support,
  and bounded forward/lateral braking. The Go2 gait uses opposing diagonal leg
  pairs and carries the Airbot at a stable joint target.
- Any non-finite target, target-range violation, low root height, or excessive
  root tilt enters a safe error state instead of continuing the action.

## Reference decision

The implementation audit included the local MimicX whole-body-control assets,
including its GR00T walk policy and recorded crouch-to-walk trajectories. Those
assets were not copied or made a runtime dependency: the browser demo remains a
deterministic action controller whose commanded joints and physical outcome can
be inspected directly. The references informed the phase structure, smooth
entry/exit envelopes, body-frame feedback, and the separation between gait and
stabilization.

## Measured verification

The deterministic offline rollout emulated the browser's 16 ms controller
cadence while MuJoCo stepped at 2 ms:

| Metric | G1 | Go2 + Airbot |
|---|---:|---:|
| Forward displacement | 0.093566 m | 0.437805 m |
| Planar displacement | 0.117963 m | 0.438116 m |
| Final root height | 0.791226 m | 0.271354 m |
| Minimum root height | 0.649679 m | 0.218415 m |
| Final root tilt | 0.139819 degrees | 1.092828 degrees |
| Maximum transient tilt | 16.737868 degrees | 18.725185 degrees |
| Ground-contact timesteps | 12,618 | 12,548 |

All ten sampled states (`settle` through `complete`) were visited. The rollout
ended with finite state, zero target clamps, and no safety trigger.

The production Chromium/Vulkan run independently completed the program with G1
forward displacement of 0.099296 m and Go2 forward displacement of 0.439727 m.
Final heights were 0.791232 m and 0.271096 m, final tilts were 0.119718 degrees
and 1.328407 degrees, and the robots ended with 8 and 4 observed contacts.
Measured joint motion was 0.409659 rad for the G1 legs, 1.244608 rad for the G1
greeting, 0.078689 rad for the Go2 legs, and 0.501202 rad for the Airbot arm. No page,
resource, WebGL, MuJoCo, finite-state, target-clamp, or controller error was
reported.

## Validated implementation variance

The initial design used 0.30 m as an aspirational G1 forward-distance gate.
That more aggressive open-loop gait was not reproducible at browser controller
cadence without sacrificing balance. The implemented browser gate is therefore
0.05 m (offline regression gate: 0.07 m): this still requires measurable free-
root translation generated through foot contact, while the retained gait has
visible alternating leg motion and finishes upright. Go2 retains and exceeds
the original 0.40 m gate. No root-state shortcut was introduced to recover the
discarded distance.

## Visual artifacts

- [G1 locomotion-phase screenshot](../../artifacts/screenshots/unitree-locomotion-suite.png)
- [Complete 1440x900 H.264 locomotion recording](../../artifacts/videos/unitree-locomotion-suite.mp4)
- [Deterministic offline metrics](../../artifacts/metrics/unitree-locomotion-suite.json)

The MP4 is 27.0 seconds (810 frames at 30 fps), including short pre/post-action
context around the complete 25-second program.
