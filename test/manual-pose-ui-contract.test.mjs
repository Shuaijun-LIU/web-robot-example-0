import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

test('Assembly1 exposes manual positioning with static-host download and optional local persistence', async () => {
  const [app, panel, vite] = await Promise.all([
    readFile(new URL('../src/App.tsx', import.meta.url), 'utf8'),
    readFile(new URL('../src/Assembly1PoseCapturePanel.tsx', import.meta.url), 'utf8'),
    readFile(new URL('../vite.config.ts', import.meta.url), 'utf8'),
  ]);

  assert.match(app, /manualPoseMode/);
  assert.match(app, /Assembly1PoseCapturePanel/);
  assert.match(app, /assemblyOwnershipRef\.current = 'manual'/);
  assert.match(panel, /进入手动定姿/);
  assert.match(panel, /保存当前姿态/);
  assert.match(panel, /createManualPoseDownload/);
  assert.match(panel, /anchor\.download/);
  assert.match(panel, /import\.meta\.env\.DEV/);
  assert.match(panel, /\/__manual-pose-capture/);
  assert.match(vite, /manualPoseCapturePlugin/);
});
