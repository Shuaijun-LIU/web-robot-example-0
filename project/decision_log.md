# Decision Log

- 2026-08-12 18:50 UTC — Initialized tracking.
- 2026-08-13 02:00 CST — Add a fourth `Franka Assembly` scene while preserving the existing three scene entries unchanged.
- 2026-08-13 02:00 CST — Use compound MuJoCo primitives for the first assembly workcell so every visible task object is represented in physics and GitHub Pages remains self-contained.
- 2026-08-13 02:00 CST — Increase the Franka ring radius from 0.72 m in the original scene to 0.90 m only in the fourth scene.
- 2026-08-13 03:15 CST — Split the assembly comparison into `Franka Assembly1` (procedural physical detail) and `Franka Assembly2` (converted RoboTwin tool meshes) while keeping one shared frame/interface contract.
- 2026-08-13 10:30 CST — Replace the Assembly1 claw hammer with a fully symmetric double-face hammer; no claw geometry or claw-named body remains in that scene.
- 2026-08-13 10:30 CST — Keep grasping purely contact-based. Increase Panda gripper stiffness/damping and fingertip friction/contact dimensionality; explicitly reject proximity attachment, welds, magnets, and scripted object following.
- 2026-08-13 10:30 CST — Extend the XLeRobot rack collision proxy to the complete visible rack envelope so its upper blue frame cannot enter the table.
- 2026-08-13 10:30 CST — Select a four-arm precision gearbox assembly for SO101 and two-robot mobile kitting/handover for XLeRobot as the next scene directions.
- 2026-08-13 13:19 CST — Limit the first Assembly1 animation to four-arm pre-grasp staging: no gripper closure, object motion, weld, magnet, proximity attach, or automatic continuation.
- 2026-08-13 13:19 CST — Assign nearby, reachable work zones after full-scene IK rejected the original cross-cell targets: south and west arms stabilize the frame, east arm stages over the torque driver, and north arm stages over the cross-member's north end.
- 2026-08-13 13:19 CST — Generate the eight Panda joint waypoints offline with the same MJCF and existing IK solver, then run only actuator interpolation in the browser. This keeps the click responsive while retaining joint-limit and TCP-error validation and avoiding any direct runtime `qpos` write.
