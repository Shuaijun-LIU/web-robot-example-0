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
