# Unitree Action Lab Progress Report

## Outcome

`Unitree Action Lab` is the ninth independent browser scene. It contains one
dynamic Unitree G1 and one dynamic Go2 carrying an Airbot arm on a shared
collision-enabled floor. Both roots remain MuJoCo free joints. The scene runs a
deterministic 10-second action clip with real gravity, contacts, joint limits,
and actuator dynamics.

## Asset provenance

- G1: MuJoCo Menagerie `unitree_g1`, BSD-3-Clause. The local package includes
  its MJCF, meshes, and license under
  `public/assets/unitree-action-lab/robots/g1/`.
- Go2 + Airbot: UniLab `go2_arm`, Apache-2.0. The local package includes its
  MJCF, Go2/Airbot meshes, and license under
  `public/assets/unitree-action-lab/robots/go2_arm/` and
  `public/assets/unitree-action-lab/robots/go2/`.
- The Go2 package was adapted only to name its floating root and six Airbot
  actuators so runtime control can resolve them deterministically.

The combined MJCF compiles to 58 bodies, 155 geoms, 47 actuators, and 61 qpos,
with exactly two floating roots: `g1_floating_base_joint` and
`go2_floating_base_joint`. Strict initial-contact validation found no
penetration deeper than 5 mm.

## Action contract

The exact phases are settle (1.0 s), rise/greet (1.5 s), scan/wave (3.0 s),
lower (1.5 s), recover (1.5 s), and final hold (1.5 s). G1 raises and bends its
right arm into a visible greeting while the Go2 legs change stance and its
six-joint Airbot arm scans. All targets transition continuously and finish at
the home vector.

The browser controller resolves 29 G1 and 18 Go2/Airbot actuator names, samples
the clip from MuJoCo simulation time, and writes only `data.ctrl`. It never
writes free-root state, body transforms, `qpos`, `qvel`, applied force, or
contacts. No policy, mocap body, equality weld, automatic attachment, or
proximity-based movement is used.

## Verification results

Offline rollout simulated 11.5 seconds at a 0.002-second timestep and visited
every action phase. The final results were:

| Metric | G1 | Go2 + Airbot |
|---|---:|---:|
| Maximum articulated joint delta | 2.129614 rad | 1.168040 rad legs / 0.940566 rad arm |
| Final floating-root height | 0.791226 m | 0.269969 m |
| Final root tilt | 0.117259° | 2.429211° |
| Timesteps with ground contact | 5,743 | 5,673 |

The production-browser verifier independently completed the clip and measured
visible G1 wrist motion of 0.584784 rad, Go2 leg motion of 0.169499 rad, and
Airbot motion of 0.601978 rad. Final browser root heights were 0.791226 m and
0.270227 m, with 8 and 4 final contacts respectively. No page, resource,
WebGL, or MuJoCo console error was recorded.

## Visual artifacts

- [Mid-action screenshot](../../artifacts/screenshots/unitree-action-lab.png)
- [Complete 1440×900 H.264 action recording](../../artifacts/videos/unitree-action-lab.mp4)

The video is 20.68 seconds including model loading and pre/post-action hold. A
six-frame inspection confirmed robot separation, visible G1 greeting, Airbot
scan, Go2 stance change, recovery, stable floor contact, and no visual
penetration.
