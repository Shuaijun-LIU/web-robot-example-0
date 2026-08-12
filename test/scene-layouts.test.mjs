import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import {
  FRANKA_ASSEMBLY_LAYOUT,
  FRANKA_LAYOUT,
  SO101_LAYOUT,
  XLEROBOT_LAYOUT,
  repeatPose,
} from '../src/sceneLayouts.js';

test('repeatPose creates one actuator home block for every robot', () => {
  assert.deepEqual(repeatPose([1, 2], 3), [1, 2, 1, 2, 1, 2]);
  assert.equal(FRANKA_LAYOUT.homeJoints.length, 32);
  assert.equal(SO101_LAYOUT.homeJoints.length, 24);
  assert.equal(XLEROBOT_LAYOUT.homeJoints.length, 32);
});

test('arm layouts replicate four robots inward around one center', () => {
  assert.equal(FRANKA_LAYOUT.instanceCount, 4);
  assert.equal(FRANKA_LAYOUT.yawStepDegrees, 90);
  assert.equal(SO101_LAYOUT.instanceCount, 4);
  assert.equal(SO101_LAYOUT.yawStepDegrees, 90);
});

test('Franka assembly scene enlarges only the new four-arm ring while remaining reachable', () => {
  assert.equal(FRANKA_LAYOUT.ringRadius, 0.72);
  assert.equal(FRANKA_ASSEMBLY_LAYOUT.instanceCount, 4);
  assert.equal(FRANKA_ASSEMBLY_LAYOUT.ringRadius, 0.9);
  assert.equal(FRANKA_ASSEMBLY_LAYOUT.homeJoints.length, 32);

  const attachmentXml = FRANKA_ASSEMBLY_LAYOUT.xmlPatches
    .flatMap((patch) => patch.replace?.[1] ?? [])
    .join('\n');
  assert.match(attachmentXml, /<frame pos="0 -0\.9 0\.1">/);
  assert.match(attachmentXml, /<frame pos="0\.9 0 0\.1" euler="0 0 90">/);
  assert.match(attachmentXml, /<frame pos="0 0\.9 0\.1" euler="0 0 180">/);
  assert.match(attachmentXml, /<frame pos="-0\.9 0 0\.1" euler="0 0 -90">/);
});

test('Franka assembly scene stages a frame, installable parts, separated tools, and handover area', () => {
  assert.deepEqual(FRANKA_ASSEMBLY_LAYOUT.taskStations, {
    frame: [0, 0, 0.275],
    parts: [-0.56, 0.42, 0.125],
    poweredTool: [0.53, -0.42, 0.135],
    manualTool: [-0.53, -0.42, 0.13],
    fasteners: [0.56, 0.42, 0.125],
    handover: [0, -0.48, 0.112],
  });

  const sceneNames = FRANKA_ASSEMBLY_LAYOUT.sceneObjects.map(({ name }) => name);
  assert.ok(sceneNames.includes('assembly_platform'));
  assert.ok(sceneNames.includes('handover_pad'));
  assert.equal(sceneNames.some((name) => name.endsWith('_cube')), false);

  const workcellXml = FRANKA_ASSEMBLY_LAYOUT.xmlPatches
    .map((patch) => patch.replace?.[1] ?? '')
    .join('\n');
  for (const name of [
    'assembly_frame',
    'cross_member',
    'mounting_plate',
    'torque_driver',
    'manual_screwdriver',
    'parts_tray',
    'fastener_tray',
    'fastener_1',
  ]) {
    assert.match(workcellXml, new RegExp(`name="${name}"`));
  }
});

test('each attachment uses an oriented physical frame in parent-scene degrees', () => {
  for (const layout of [FRANKA_LAYOUT, SO101_LAYOUT, XLEROBOT_LAYOUT]) {
    const replacements = layout.xmlPatches
      .filter((patch) => patch.replace)
      .map((patch) => patch.replace[1])
      .join('\n');
    assert.match(replacements, /<frame pos=/);
    assert.match(replacements, /<attach model=/);
  }
  const frankaXml = FRANKA_LAYOUT.xmlPatches.map((patch) => patch.replace?.[1] ?? '').join('\n');
  const xlerobotXml = XLEROBOT_LAYOUT.xmlPatches.map((patch) => patch.replace?.[1] ?? '').join('\n');
  assert.match(frankaXml, /euler="0 0 90"/);
  assert.match(xlerobotXml, /<frame pos="-0\.85 0 0" euler="0 0 180">/);
  assert.match(xlerobotXml, /<frame pos="0\.85 0 0"><attach/);
});

test('XLeRobot uses two opposing robots and an arm-height table', () => {
  assert.equal(XLEROBOT_LAYOUT.instanceCount, 2);
  assert.equal(XLEROBOT_LAYOUT.yawStepDegrees, 180);
  assert.equal(XLEROBOT_LAYOUT.armBaseHeight, 0.775);
  assert.equal(XLEROBOT_LAYOUT.tableTopHeight, XLEROBOT_LAYOUT.armBaseHeight);
  assert.equal(
    XLEROBOT_LAYOUT.tableObjects.filter((object) => object.name.startsWith('table_leg_')).length,
    4,
  );
});

test('runtime configs consume all four shared layout definitions', async () => {
  const source = await readFile(new URL('../src/configs.ts', import.meta.url), 'utf8');

  assert.match(source, /FRANKA_LAYOUT\.xmlPatches/);
  assert.match(source, /FRANKA_ASSEMBLY_LAYOUT\.xmlPatches/);
  assert.match(source, /SO101_LAYOUT\.xmlPatches/);
  assert.match(source, /XLEROBOT_LAYOUT\.xmlPatches/);
  assert.match(source, /controlTargets:\s*createFrankaTargets\(\)/);
  assert.match(source, /frankaAssembly:[\s\S]*controlFamily:\s*'franka'/);
  assert.match(source, /controlTargets:\s*createSO101Targets\(\)/);
  assert.match(source, /controlTargets:\s*createXLeRobotTargets\(\)/);
});
