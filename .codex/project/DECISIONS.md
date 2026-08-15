# Decisions

## 2026-08-15: Isolate alternate-arm scenes
- Context: The canonical checkout contains uncommitted Assembly Step 3 work.
- Options considered: edit the canonical checkout, reuse the Unitree worktree, or create a dedicated worktree.
- Decision: use `feature/piper-ur5e-assembly` in `.worktrees/piper-ur5e-assembly`.
- Rationale: avoids file and staging conflicts with both active workstreams.
- Consequences / follow-ups: integrate by commit after verification; do not stage or alter canonical-checkout files.

## 2026-08-15: Use local Menagerie assets
- Context: PiPER, UR5e, and Robotiq 2F-85 are available locally with redistributable licenses.
- Options considered: runtime GitHub assets, alternate URDF conversion, or local MJCF vendoring.
- Decision: vendor the local Menagerie MJCFs and meshes; compose UR5e and 2F-85 with MuJoCo `attach`.
- Rationale: deterministic GitHub Pages loading, documented provenance, and direct actuator/IK integration.
- Consequences / follow-ups: retain each upstream license and add third-party notices.

## 2026-08-15: Keep outer assembly transforms in degrees
- Context: Robot child models define radian joint ranges, while the copied Assembly1 parent frames and tool poses use literal 90/180-degree Euler values.
- Decision: compile both new parent scenes with `angle="degree"`; attached child models retain their own compiler settings.
- Rationale: preserves the proven Assembly1 orientation contract and prevents parent tool/base transforms from being interpreted as 90 radians.

## 2026-08-15: Move only the PiPER hammer station inward
- Context: At the selected `0.78 m` PiPER ring, the original `x=0.65 m` hammer station intersected the east arm at its home pose.
- Decision: move the complete PiPER hammer station (tool, mat, and both shelf supports) to `x=0.57 m`; keep every other Assembly1 station and all UR5e stations unchanged.
- Rationale: strict contact evidence isolates the overlap to that station, and the adjusted scene reports zero initial penetrating contacts.

## 2026-08-15: Preserve both completed feature histories during integration
- Context: PiPER/UR5e Assembly1 and Unitree locomotion were developed in isolated worktrees and overlap in three UI/test files.
- Options considered: squash one branch, overwrite shared files from one branch, or merge both histories and resolve shared files additively.
- Decision: fast-forward the PiPER/UR5e branch, merge the Unitree branch without rewriting history, and resolve shared files so both scene families and their verification contracts remain present.
- Rationale: this keeps independently verified commits attributable and avoids dropping either completed feature set.
