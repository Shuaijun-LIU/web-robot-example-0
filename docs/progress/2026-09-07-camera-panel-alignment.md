# Five-camera panel alignment

## Cause and fix

The global feed spanned two grid rows but had one external caption, while the wrist feeds had two external captions plus a row gap. Center alignment put the global feed approximately 12.31 px below the upper wrist feeds at a 1600 × 1000 viewport.

Only the all-five CSS layout changed: keep the global feed on the left and four wrist feeds on the right, move captions inside each feed, and use inset separators instead of spacing that alters image dimensions. All five retain native 16:9 proportions. Single-camera layout, camera poses, renderer, physics and action controllers are unchanged. Assembly1 and Demo1 share this fix.

## Verification

- `scripts/verify-camera-panel-layout.mjs` failed before the fix on the measured top-edge mismatch, then passed in both development and production preview.
- Verified image edges, columns, captions, 16:9 ratios and viewport bounds at 1600 × 1000, 1280 × 800, 900 × 700 and 390 × 844.
- Single-camera selection and collapse checks passed.
- TypeScript and production build passed; unit suite 211/211 passed.
- [Final screenshot](../../artifacts/screenshots/assembly-cameras-aligned.png) inspected visually.

The user subsequently authorized committing and pushing this camera-panel alignment fix to `main`.
