import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const files = {
  capture: new URL('../scripts/capture-scenes.mjs', import.meta.url),
  verifier: new URL('../scripts/verify-unitree-action-browser.mjs', import.meta.url),
  video: new URL('../scripts/capture-unitree-action-video.mjs', import.meta.url),
  package: new URL('../package.json', import.meta.url),
  controller: new URL('../src/UnitreeActionController.tsx', import.meta.url),
};

test('scene capture includes the two-instance Unitree Action Lab', async () => {
  const source = await readFile(files.capture, 'utf8');
  assert.match(
    source,
    /key:\s*'unitreeActionLab',[\s\S]*?label:\s*'Unitree Action Lab',[\s\S]*?instances:\s*2/,
  );
});

test('browser action timing follows MuJoCo time rather than render-frame count', async () => {
  const source = await readFile(files.controller, 'utf8');
  assert.match(source, /data\.time - lastPhysicsTimeRef\.current/);
  assert.doesNotMatch(source, /elapsedRef\.current \+ model\.opt\.timestep/);
});

test('browser verifier drives the locomotion suite through gait to completion', async () => {
  const source = await readFile(files.verifier, 'utf8');
  assert.match(source, /sceneInstances\)\s*!==\s*2/);
  assert.match(source, /selectUnitreeActionProgram\('locomotion'\)/);
  assert.match(source, /runUnitreeAction\(\)/);
  assert.match(source, /unitreeActionPhase\s*===\s*'g1-walk'/);
  assert.match(source, /getUnitreeActionDiagnostics\(\)/);
  assert.match(source, /displacement\.g1\.x/);
  assert.match(source, /displacement\.go2\.x/);
  assert.match(source, /unitreeActionStatus\s*===\s*'complete'/);
  assert.match(source, /unitree-locomotion-suite\.png/);
});

test('video runner records the full action and package exposes all verification commands', async () => {
  const source = await readFile(files.video, 'utf8');
  const packageJson = JSON.parse(await readFile(files.package, 'utf8'));

  assert.match(source, /recordVideo/);
  assert.match(source, /process\.env\.FFMPEG_PATH/);
  assert.match(source, /libx264/);
  assert.match(source, /selectUnitreeActionProgram\('locomotion'\)/);
  assert.match(source, /unitree-locomotion-suite\.mp4/);
  assert.equal(
    packageJson.scripts['verify:unitree-action'],
    'node scripts/verify-unitree-action-dynamics.mjs',
  );
  assert.equal(
    packageJson.scripts['verify:unitree-action-browser'],
    'node scripts/verify-unitree-action-browser.mjs',
  );
  assert.equal(
    packageJson.scripts['capture:unitree-action-video'],
    'node scripts/capture-unitree-action-video.mjs',
  );
});
