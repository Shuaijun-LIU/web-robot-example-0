# Panel Relay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline, task by task. Track checkbox steps.

**Goal:** A complete isolated four-Panda physical panel relay over a sourced obstacle.
**Architecture:** Reuse cooperative asset conversion, Workcell IK and actuator runtime. Add a scene-specific builder/solver, optional finite upright gate and physical acceptance tests; preserve all existing tasks.
**Tech Stack:** React/TypeScript, native MuJoCo 3.3.1 / WASM 3.3.8, Python/trimesh, Node/Playwright.
**Spec:** `docs/superpowers/specs/2026-10-07-panel-relay-design.md`

## Global Constraints

- New scene `relay`, key `frankaDemo7`; old scenes and Demo1 default unchanged.
- One free sourced panel, 32 robot actuators, no attachment/body-state scripting.
- Every receiver must have actual bilateral contact before its donor releases.
- Panel/obstacle numerical penetration limit 1 mm; accepted nominal route has physical clearance.
- Native/WASM/browser full cycle, delayed start, full regression and source records.
- Initially publish accepted Demo6 first and keep new Demo7 local for inspection. The subsequent explicit optimize-and-push request authorizes Demo7 publication after renewed acceptance.

## Review Focus

- Convex collision must not inherit source padding that creates invisible grasps.
- Contact order must follow observations, not labels declaring a handover.
- A dropped/tilted panel must not pass just because the final frame recovers.
- Sweeping palms/forearms during open-gripper retreat count as real collisions.
- Scene switching and manual rotations must not leave stale controller ownership.

### Task 1: Source assets, page and orientation observation

**Files:** create `scripts/build-relay-assets.py`, `test/relay-workcell.test.mjs`, `test/relay-gates.test.mjs`, `test/helpers/relay-engine.mjs`; modify scene/config registration and cooperative gate/runtime declarations.
**Consumes:** existing `build-cooperative-assets.py` Package export, Panda and cooperative provider.
**Produces:** `relay.xml`, `relay-manifest.json`, source snapshots/texture/mesh package; upright gate `{type:'upright',object,axis,minAlignment,maxSpeed}` and observation `{alignment,speed}`.

- [x] RED: missing scene/assets, finite upright cases (aligned pass, tilted/moving/nonfinite fail), bad axis/limits rejection; real model must have free payload/no weld and physically aligned visible/collision surfaces.
- [x] Build sourced board/block assets and isolated layout. Register page and read-only orientation observation; retain initial quaternion guard.
- [x] GREEN: focused tests plus software-rendered static screenshot; inspect grip access, source geometry and obstacle position.

### Task 2: Physical two-pair relay and obstacle transport

**Files:** create `scripts/solve-relay-task.py`, `test/relay-wasm.test.mjs`; generate accepted `relay-motion.json` and native/WASM reports.
**Consumes:** Task 1 assets and gate schema; existing Workcell real-contact phases.
**Produces:** full accepted actuator program with source-panel relative coupled targets and measured relay states.

- [x] RED: complete actual WASM replay requires all four grasps, receiver contact before donor release, at least two surface-qualified supports throughout airborne relay (raw bilateral counts retained; bounded material-witness check for numerical contact flicker), maximum tilt 10°, output support, released fingers, HOME and no obstacle collision.
- [x] Implement solver, run full native cycle, diagnose recorded IK/contact failures before path/layout changes. Export only after actual contact gates pass.
- [x] GREEN: native, WASM and +1500 settling ticks; preserve per-tick tilt/support/clearance evidence and source-grip aperture checks. Independent review additionally required actual contact→visual triangle checks; RED caught the Arm 2 groove, GREEN after 10-mm grip inset and carry height adjustment.

### Task 3: Browser and delivery evidence

**Files:** create `scripts/verify-relay-browser.mjs`, `docs/progress/2026-10-07-panel-relay.md`; update project tracking, screenshots and reports.
**Consumes:** accepted relay program through the real cooperative provider.

- [x] Run full rendered physical cycle, Pause/Reset/switch; inspect initial/relay/final screenshots, zero errors/warnings.
- [x] Run full Node suite, TypeScript/build; one independent source review and RED/GREEN fixes for Important findings. Final 276/276 tests pass; every-2-ms visible contact audit checks 351,589 nominal and 369,511 delayed contact points.
- [x] Record actual performance, source license/hashes, geometry and failed-path decisions, plus deferred benchmark work; preserve local branch, no implicit Demo7 push.

## Subsequent bounded optimization and explicit release

- [x] Test-first paired receiver approach and donor retreat, preserving independent receiver contact and sequential 4→3→2 release. 22 phases/48.9 seconds; no faster contact motions or relaxed physical conditions.
- [x] Compact only Demo7's English panel and improve default view; actual-default-view browser height/screenshot checks pass.
- [x] Repeat native, every-tick nominal/delayed WASM, full browser, 277-test regression/build and independent review; no unresolved findings. Detailed current metrics supersede the initial Task 3 counts above.
- Publication is now expressly requested; fast-forward main and verify deployed files. Do not clean other worktrees or rewrite history.
