# Task Plan

## Goal
- Active work (2026-09-08): design an isolated four-Panda mixed-egg sorting Demo2; first deliver a reviewable static workcell, then validate physical manipulation incrementally.
- Demo1 / Assembly1 are temporarily complete and frozen by user acceptance at `019afd25d2f1f93aa08866330634f936b29f2818`. Preserve all existing scenes and the current default entry. The goals below describe the completed/archived assembly work, not a request to reopen it.
- Refine the fourth web scene into `Franka Assembly1` and add `Franka Assembly2` for a direct procedural-versus-external-asset comparison.
- Preserve the existing Franka Panda, SO101, and XLeRobot scenes and controls unchanged.
- Deliver a physically coherent static workcell, browser-verified screenshot, documentation, and GitHub Pages deployment.
- Incrementally animate Assembly1, beginning with a reviewable four-arm pre-grasp staging step before introducing contact or object manipulation.
- Add an isolated dynamic Unitree scene with a real policy-free actuator action for G1 and Go2 + Airbot.
- Continue Assembly1 through robust physical release and first-fastener staging.

## Phases
- [x] Phase 1: Scope, local asset audit, and scene design
- [x] Phase 2: Test-first scene contract
- [x] Phase 3: Static assembly workcell implementation
- [x] Phase 4: Browser, physics, and visual verification
- [x] Phase 5: Documentation and GitHub Pages deployment
- [x] Phase 6: Shared detailed frame and installation interfaces
- [x] Phase 7: Assembly1 procedural tool refinement
- [x] Phase 8: Assembly2 RoboTwin asset integration
- [x] Phase 9: Dual-scene verification and republishing
- [x] Phase 10: Double-face hammer, XLeRobot rack collision, and physical-grasp repair
- [x] Phase 11: SO101 and XLeRobot next-scene concepts
- [x] Phase 12: Assembly1 Step 1 four-arm coordinated pre-grasp staging
- [x] Phase 13: Redesign Step 1 around grasp-ready geometry, dual-arm cross-member handling, and gravity-stable final poses
- [x] Phase 14: Review the grasp-ready result and define the first real contact/grasp transition
- [x] Phase 15: Implement and production-verify Step 2 physical contact and clamped hold
- [x] Phase 16: Unitree Action Lab dynamic models, actuator choreography, browser verification, and visual artifacts
- [x] Phase 17: Implement and production-verify Step 3 dual-arm cross-member placement
- [x] Phase 18: Assembly1 release, handover, insertion/strike and return-home development; subsequently accepted by the user as temporarily complete on 2026-09-08. Historical test caveats remain in dated progress reports.
- [x] Phase 19: Record Demo1 / Assembly1 closure and initial Demo2 resource investigation.
- [x] Phase 20: User approved stage A; imported RoboDojo shell/holder and Menagerie UMI gripper candidates. Physical hardware adapter qualification and upstream asset publication rights remain separate follow-ups.
- [x] Phase 21: Deliver and publish the isolated static Demo2 workcell for visual review (2026-09-08, `49f9be7`, Pages deployment successful).
- [x] Phase 22: Validate one-arm egg grasp, transfer, seating, release and controlled post-placement regrasp/orientation correction.
- [ ] Phase 23: Validate two-arm shared-space conflict handling, then four-arm parallel sorting.
- [ ] Phase 24: Integrate continuous Demo2 playback and regression-check existing scenes.

## Status
- Initialized: 2026-08-12 18:50 UTC
- Updated: 2026-09-08.
- Current phase: First-egg baseline `f6d45ac` published and accepted. Separate post-placement correction trial now passes native, WASM and actual browser checks (24.385 → 0.241 degrees), with screenshots and 226 passing regression tests; see [correction report](../docs/progress/2026-09-08-demo2-reseat.md). Correction changes remain local, not pushed.
- Pending validation: other egg classes/occupied tray cells, two-arm conflict handling and four-arm automation; qualified hardware gripper mount. The current correction recipe is checked for a controlled pose envelope, not arbitrary-pose online recovery. Source asset publication provenance has been resolved and documented.
