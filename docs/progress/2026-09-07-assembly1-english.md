# Assembly1 English interface

## Scope

Translate Assembly1 interface copy only: all four action steps, phase descriptions, completion/failure/reset messages, manual posing controls, pose validation, download confirmations and local server persistence errors. Translate the legacy first-step panel as well. Other scene languages and all simulation/action logic remain unchanged.

The pose-capture hint now distinguishes browser JSON downloads from the additional server copy available in local development.

## Verification

- New real-component rendering tests failed on the original Chinese text, then passed after translation. They cover every declared Step 1–4 status/phase, failures and both manual control modes.
- Full unit suite: 211/211 passed.
- TypeScript check and production Vite build with the Pages base path: passed.
- Production browser check: English page text, entering/exiting manual mode, invalid pose-name feedback, successful JSON download and Step 1 completion all passed, with no page errors.
- [Browser screenshot](../../artifacts/screenshots/assembly1-english-ui.png).
- Reproduce browser check with `scripts/verify-assembly-english-ui.mjs` and `SCENE_URL` pointing to the production preview.

The user subsequently authorized committing and pushing this English interface update to `main`.
