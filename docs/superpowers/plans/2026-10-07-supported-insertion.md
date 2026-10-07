# Supported Insertion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Complete an isolated four-Panda physical supply/stabilize/insert/dispatch demonstration.
**Architecture:** Reuse cooperative runtime and offline Workcell; export sourced robosuite objects in a dedicated builder. Add finite insertion geometry and optional stale-orientation checks without changing old gate semantics.
**Tech Stack:** React/TypeScript, MuJoCo native 3.3.7 / WASM 3.3.8, robosuite parameterized assets, Python and Node tests.
**Spec:** `docs/superpowers/specs/2026-10-07-supported-insertion-design.md`

## Global Constraints

- Keep old scenes/default and motion assets unchanged. New key `frankaDemo6`, scene `insertion`.
- Two free task bodies; 32 robot actuators; no attachment, stand anchoring or scripted object state/forces.
- Sourced stand/frame constructors with hashes, parameters and MIT attribution.
- Full contact-based success; nominal insertion pair penetration below 1 mm.
- Implement locally, on a new feature branch after the requested current release push.

## Review Focus

- Visually aligned but unsupported or too-shallow insertion must fail.
- Rotating a part around its center manually must invalidate stale playback.
- Frame must remain seated after both robots release and during stand transport.
- Palm/fingers must clear socket and frame during all approach/release paths.
- Reset, pause and scene switch must retain existing controller ownership.

### Task 1: Asset import and insertion observation contract

**Files:** create `scripts/build-insertion-assets.py`, `test/insertion-gates.test.mjs`, `test/insertion-workcell.test.mjs`, `test/helpers/insertion-engine.mjs`; add scene/config entries and extend `src/cooperativeMotion.{js,d.ts}`, `src/CooperativeMotionRuntime.js`.
**Produces:** `insertion.xml`, manifest/source snapshots, scene mapping; optional `initialObjects[].quaternion` and `orientationTolerance`; `inserted` gate/observation matching the spec.

- [x] Write and run RED tests: independent insertion gate numeric cases (depth .10 m, lateral .002 m, alignment .999, real contact, settled speed); malformed limits/axes rejected; real-model cavity probe cannot pass through a wall; no fixture actuator; same-center object rotation requires Reset.
- [x] Export source constructors with exact dimension/provenance records and matching visual/collision poses. Register isolated scene and finite read-only observations.
- [x] Run gate/workcell tests GREEN and inspect initial software-rendered browser layout before complete motion. Expected: tests pass, four robots and supported source parts visible.

### Task 2: Physical supply, insertion and assembly transport

**Files:** create `scripts/solve-insertion-task.py`, `test/insertion-wasm.test.mjs`; generate `insertion-motion.json` and physical reports.
**Consumes:** Task 1 gate schema/assets; existing Workcell IK, contact checks, pick/place and actuator export.
**Produces:** physically accepted continuous motion with all four roles and final free-standing assembly.

- [x] Write/run RED full-cycle test requiring each role's actual grasp, meaningful insertion depth, real socket contact after release, final output support, all arms HOME, and pair penetration under 1 mm.
- [x] Implement native subclass/gates and choreography. Record failing contacts/IK diagnostics. Export only after full cycle passes; record any geometric/path adjustment.
- [x] Run native, WASM and delayed-start rollouts. Expected: full-cycle tests pass with no scripted task-body motion and stable inserted assembly through dispatch.

### Task 3: Browser acceptance and delivery records

**Files:** create `scripts/verify-insertion-browser.mjs`, `docs/progress/2026-10-07-supported-insertion.md`; update project tracking; save screenshots/reports.
**Consumes:** Task 2 program through the existing cooperative provider.

- [x] Run complete browser physical cycle, Pause/Reset/switch and inspect initial/inserting/final screenshots. Expected: complete, zero page errors/warnings and correct visibly assembled output.
- [x] Run full Node suite, TypeScript and production build; independent source review and any Important findings' reproducing tests. Expected: all pass.
- [x] Record real outcomes and remaining benchmark work; keep new demo local for user inspection. Do not claim the new task is deployed with the preceding release.
