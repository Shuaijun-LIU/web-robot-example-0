import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const files = {
  configs: new URL('../src/configs.ts', import.meta.url),
  app: new URL('../src/App.tsx', import.meta.url),
  validator: new URL('../scripts/validate-mjcf.mjs', import.meta.url),
  capture: new URL('../scripts/capture-scenes.mjs', import.meta.url),
  controls: new URL('../scripts/verify-controls.mjs', import.meta.url),
  g1Static: new URL('../public/assets/so101-gearbox-room/robots/g1/g1_static.xml', import.meta.url),
  go2ArmStatic: new URL('../public/assets/so101-gearbox-room/robots/go2_arm/go2_arm_static.xml', import.meta.url),
};

test('runtime registers both collaboration layouts as additional scenes', async () => {
  const source = await readFile(files.configs, 'utf8');

  assert.match(source, /SO101_GEARBOX_LAYOUT/);
  assert.match(source, /XLEROBOT_KITTING_LAYOUT/);
  assert.match(source, /so101Gearbox:\s*{[\s\S]*?label:\s*'SO101 Gearbox'/);
  assert.match(source, /so101Gearbox:\s*{[\s\S]*?controlFamily:\s*'so101'/);
  assert.match(source, /so101Gearbox:\s*{[\s\S]*?controlTargets:\s*createSO101Targets\(\)/);
  assert.match(source, /xlerobotKitting:\s*{[\s\S]*?label:\s*'XLeRobot Kitting'/);
  assert.match(source, /xlerobotKitting:\s*{[\s\S]*?controlFamily:\s*'xlerobot'/);
  assert.match(source, /xlerobotKitting:\s*{[\s\S]*?controlTargets:\s*createXLeRobotTargets\(\)/);
  assert.match(
    source,
    /const SO101_GEARBOX_BASE = `\$\{import\.meta\.env\.BASE_URL\}assets\/so101-gearbox-room\/`/,
  );
  assert.match(source, /so101Gearbox:\s*{[\s\S]*?src:\s*SO101_GEARBOX_BASE/);
});

test('vendored room robots are visual static models without control or free fall', async () => {
  for (const [name, path, root] of [
    ['G1', files.g1Static, 'pelvis'],
    ['Go2 arm', files.go2ArmStatic, 'base'],
  ]) {
    const source = await readFile(path, 'utf8').catch(() => '');
    assert.match(source, new RegExp(`<body name="${root}"`), `${name} root body must exist`);
    assert.match(source, /<geom[^>]+mesh=/, `${name} must retain its visual meshes`);
    assert.doesNotMatch(source, /<freejoint|<joint\b|<actuator>/, `${name} must remain static`);
  }
});

test('scene diagnostics count the copied physical roots and preserve Assembly1 as default', async () => {
  const source = await readFile(files.app, 'utf8');

  assert.match(source, /so101Gearbox:\s*\/\^r\\d\+_Base\$\//);
  assert.match(source, /xlerobotKitting:\s*\/\^r\\d\+_chassis\$\//);
  assert.match(source, /robot:\s*{\s*value:\s*'frankaAssembly1'/);
});

test('offline compiler knows both collaboration layouts and root body types', async () => {
  const source = await readFile(files.validator, 'utf8');

  assert.match(source, /so101Gearbox:\s*{\s*layout:\s*SO101_GEARBOX_LAYOUT/);
  assert.match(source, /xlerobotKitting:\s*{\s*layout:\s*XLEROBOT_KITTING_LAYOUT/);
  assert.match(source, /so101Gearbox:\s*'Base'/);
  assert.match(source, /xlerobotKitting:\s*'chassis'/);
  assert.match(source, /INITIAL_CONTACT_REPORT/);
  assert.match(source, /initial penetrating contacts:/);
});

test('browser capture and per-device control checks include both new scenes', async () => {
  const capture = await readFile(files.capture, 'utf8');
  const controls = await readFile(files.controls, 'utf8');

  assert.match(capture, /key:\s*'so101Gearbox',[\s\S]*?label:\s*'SO101 Gearbox',[\s\S]*?instances:\s*4/);
  assert.match(capture, /key:\s*'xlerobotKitting',[\s\S]*?label:\s*'XLeRobot Kitting',[\s\S]*?instances:\s*2/);
  assert.match(controls, /key:\s*'so101Gearbox',[\s\S]*?targets:\s*\['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'\]/);
  assert.match(controls, /key:\s*'xlerobotKitting',[\s\S]*?targets:\s*\['Robot 1', 'Robot 2'\]/);
  assert.match(controls, /process\.env\.SCENES/);
});
