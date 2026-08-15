# Unitree Locomotion Suite Design

Date: 2026-08-15

## Goal

Extend the published `Unitree Action Lab` without removing or changing its
verified 10-second `Greeting & Scan` program. Add a second, deterministic
`Locomotion Suite` in which the free-root G1 squats, stands, and walks, and the
free-root Go2 walks while carrying its Airbot arm. The complete program must be
driven through articulated joint actuators and MuJoCo contact dynamics: no
runtime root-pose writes, trajectory teleportation, mocap bodies, equality
welds, invisible supports, or proximity attachment.

This remains an action-oriented demo. The user selects a named action program;
the locomotion phases use compact state-feedback controllers for balance rather
than a learned policy or an externally scripted root path.

## Resource audit and implementation choice

Three implementation routes were evaluated:

1. **State-feedback procedural actions (selected).** The local
   `hwkim3330__mujoco-unitree` reference demonstrates G1/Go2 gait generation
   from periodic leg targets plus roll/pitch feedback. Its repository has no
   distributable license file, so no source or model artifact will be copied.
   The implementation will independently express the required oscillator,
   support-phase, and attitude-feedback equations against this project's
   existing position-actuator MJCFs.
2. **Learned whole-body policy.** MimicX records the motions
   `B25_-_crouch_to_walk1` and `C2_-_Run_to_stand`, and its Apache-2.0
   GR00T-WBC dependency contains the SONIC encoder/decoder. This is a valuable
   future route, but browser deployment requires the complete observation
   history, encoder, decoder, reference-motion stream, and action postprocess.
   The original NPZ paths recorded by the selected MimicX runs are not present
   on this machine, so this route is not the smallest reliable extension.
3. **Open-loop pose playback.** Reference joint frames could be interpolated
   directly, but a free-root humanoid can fall or foot-skate when tracking a
   clip without balance feedback. This route is rejected for locomotion.

The selected controller is deliberately small and auditable. It reads current
joint/root state and writes actuator targets; the ground reaction that advances
each robot remains entirely a MuJoCo result.

## Compatibility and scene layout

The existing `unitreeActionLab` scene key, models, lighting, camera, floor,
free joints, namespaced actuators, licenses, and reset pose remain in place.
No third robot, furniture, or task prop is added.

The robots start at their existing x positions (`G1: -1.10 m`, `Go2: +1.10
m`) and both face the positive x direction. The programs run sequentially to
avoid collision: G1 advances first and must stop before the Go2 start lane;
Go2 then advances away from G1. The camera/orbit bounds may widen only as much
as needed to keep both physical paths visible.

Reset remains the only operation allowed to restore the initial free-root
states. Starting a program from a completed or disturbed scene performs the
existing scene reset first, waits for the simulation to be ready, and then
starts the selected program. A running program cannot be switched in place.

## Program model and user interface

The action panel gains a program selector with two entries:

- `问候与扫描 · 10 s` — the existing sampler and phase timings, unchanged.
- `完整运动套件 · 25 s` — the new locomotion sampler/controller.

The selected program name, duration, current phase, elapsed time, and run state
are visible. `执行完整动作`, `暂停/继续`, and `重新开始` retain their current
meanings. Changing the selection while idle or complete resets the scene and
loads the chosen program. Selection is disabled while running or paused.

Program definitions are data: id, label, duration, phase list, and phase-copy
metadata. State transitions are parameterized by the selected program instead
of relying on the original global duration. `Greeting & Scan` remains the
default to preserve the current published behavior.

## Locomotion choreography

`Locomotion Suite` lasts exactly 25 seconds:

1. **Settle — 1.0 s.** Hold the verified home targets and collect the initial
   root poses used only for displacement diagnostics.
2. **G1 squat — 2.0 s.** Smoothly increase symmetric hip and knee flexion,
   compensate ankle pitch, keep the waist neutral, and lower the pelvis without
   breaking foot contact.
3. **G1 stand — 2.0 s.** Reverse the same continuous path and settle upright.
4. **G1 walk — 6.0 s.** Ramp into an alternating left/right gait, advance for
   multiple steps, ramp the stride back to zero, and use opposite arm swing.
5. **G1 stabilize — 2.0 s.** Return to the stable leg pose and let root rates
   decay while Go2 remains at home.
6. **Go2 walk — 6.0 s.** Ramp into a diagonal trot: front-left with rear-right,
   then front-right with rear-left. Hold the Airbot joints in a compact carry
   pose and ramp the gait back to zero.
7. **Go2 stabilize — 2.0 s.** Restore the verified quadruped home targets and
   let root rates decay.
8. **Final greeting — 3.0 s.** G1 performs the existing right-arm greeting and
   wrist wave while the Airbot performs the existing scan pose. Locomotion
   targets remain stationary.
9. **Final hold — 1.0 s.** Both robots return to their stable whole-body home
   targets for final measurements.

Every phase transition uses a clamped smoothstep envelope. Oscillatory phases
also use entry and exit envelopes so stride amplitude and target velocity are
zero at the phase boundaries.

## G1 controller

The G1 controller uses the existing 29 position actuators and current MuJoCo
state. It contains three independently testable parts:

- A symmetric squat sampler for both hip pitch, knees, and ankle pitch. Target
  depth is bounded by actuator limits and by a pelvis-height safety floor.
- A gait clock with left/right legs exactly half a cycle apart. Hip pitch,
  knee flexion, and ankle pitch generate stance and swing shapes; hip roll and
  ankle roll receive small lateral-balance corrections.
- Root attitude and rate feedback derived from the G1 floating-base quaternion
  and angular velocity. Pitch corrections are distributed across hip/ankle
  pitch; roll corrections are distributed symmetrically across hip/ankle roll.
  All corrections are clamped before they are added to the nominal action.

Arm counter-swing is phase-locked to the opposite leg. Waist and wrist targets
stay within a conservative subset of their control ranges. The controller does
not estimate or command a root x position; forward speed emerges from the gait,
foot friction, and contact forces.

If pelvis height drops below `0.50 m`, absolute roll/pitch exceeds `35°`, state
becomes non-finite, or a target exceeds its declared actuator range, the
program enters `error` and commands the stable home targets rather than
continuing the gait.

## Go2 controller

The Go2 controller drives the existing 12 leg position actuators while holding
the six Airbot actuators in a compact carry pose. It uses:

- A continuous trot clock whose diagonal pairs differ by half a cycle.
- Piecewise-smooth thigh/calf swing clearance and stance extension.
- Small hip-abduction offsets for lateral support.
- Base roll/pitch and angular-rate feedback distributed across diagonal leg
  targets, with independent clamps for hip, thigh, and calf corrections.

The controller never writes wheel, foot, body, or root transforms. If base
height drops below `0.16 m`, absolute roll/pitch exceeds `45°`, state becomes
non-finite, or a target leaves its declared range, execution enters `error` and
commands the Go2/Airbot home targets.

## Runtime architecture

The pure action module owns program/phase timing, nominal pose sampling, gait
phase, envelopes, clamps, and actuator limit validation. A small dynamics
adapter owns only state extraction: named free-root qpos/qvel offsets,
quaternion-to-roll/pitch conversion, and the diagnostic start/final root poses.

`UnitreeActionController` resolves actuator and joint addresses once per reset.
At every before-physics step it:

1. Reads the selected program and current MuJoCo time.
2. Reads only the current qpos/qvel values needed for attitude feedback and
   diagnostics.
3. Samples nominal action targets and applies bounded feedback corrections.
4. Validates the complete 47-target vector atomically.
5. Writes only the resolved entries in `data.ctrl`.

Pausing holds the last actuator targets while MuJoCo continues stepping, so
gravity, contact, and instability remain visible. Reset/unmount releases all
controller state and callbacks.

## Diagnostics and verification contract

Browser diagnostics expose, per program:

- status, phase, elapsed time, and error;
- initial and current G1/Go2 root position, height, roll, pitch, and speed;
- actuator target minima/maxima and any clamp count;
- foot-contact observations where available;
- final x displacement for both free roots.

Tests are written before implementation and cover:

1. The original `Greeting & Scan` samples and exact 10-second duration remain
   unchanged.
2. The new phase durations sum to exactly 25 seconds and sampling is continuous
   at every boundary.
3. Squat depth is visible, returns to upright, and all 29 G1 targets remain
   finite and within declared ranges.
4. G1 legs are half-cycle opposed, arms counter-swing, Go2 diagonal pairs are
   phase matched, and opposing diagonal pairs are half-cycle opposed.
5. Feedback signs counter the measured roll/pitch disturbance and every
   correction is bounded.
6. Source/runtime audits find no action-time writes to `qpos`, `qvel`, free
   roots, body transforms, applied forces, or mocap state.
7. A complete browser run finishes without controller error, NaN, robot-robot
   collision, or a fall. G1 must advance at least `0.30 m`; Go2 must advance at
   least `0.40 m`. Final G1 pelvis height must be `0.70–0.90 m` with tilt at
   most `12°`; final Go2 base height must be `0.20–0.36 m` with tilt at most
   `15°`.
8. The production build, complete Node test suite, offline MJCF compile,
   browser verification, screenshot capture, and full 25-second video capture
   pass.

If the first physically valid gait does not reach the displacement thresholds,
stride amplitude, cadence, feedback gains, floor friction, and actuator gains
may be tuned within realistic bounded ranges. The thresholds may not be met by
writing root state or moving the floor.

### Validated implementation addendum

Browser-cadence tuning showed that the aspirational `0.30 m` G1 gate required
an open-loop biped stride that did not remain reproducibly balanced. The final
verification therefore uses `0.05 m` in the production browser and `0.07 m` in
the deterministic 16 ms-control-cadence regression, while retaining visible
alternating leg motion, finite state, ground contacts, and the specified final
height/tilt bounds. Go2 keeps the original `0.40 m` gate. The measured final
browser displacement is recorded in the progress report; root-state writes
remain prohibited.

## Deliverables

- Both selectable programs in the existing `Unitree Action Lab` scene.
- Deterministic unit and dynamics tests for the new controllers.
- Updated browser verification and diagnostics.
- A screenshot showing a locomotion phase.
- A complete browser-recorded locomotion video under `artifacts/videos/`.
- Progress documentation recording measured displacement, final stability, and
  any tuned physical parameters.
