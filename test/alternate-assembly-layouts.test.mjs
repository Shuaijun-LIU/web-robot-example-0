import assert from 'node:assert/strict';
import test from 'node:test';

import {
  PIPER_ASSEMBLY1_LAYOUT,
  PIPER_HOME,
  UR5E_ASSEMBLY1_LAYOUT,
  UR5E_HOME,
} from '../src/alternateAssemblyLayouts.js';

const layoutXml = (layout) => layout.xmlPatches
  .map((patch) => [patch.inject, patch.replace?.[1]].filter(Boolean).join('\n'))
  .join('\n');

test('both alternate Assembly1 layouts preserve the four-arm workcell contract', () => {
  for (const [layout, home] of [
    [PIPER_ASSEMBLY1_LAYOUT, PIPER_HOME],
    [UR5E_ASSEMBLY1_LAYOUT, UR5E_HOME],
  ]) {
    assert.equal(layout.instanceCount, 4);
    assert.equal(layout.homeJoints.length, home.length * 4);
    assert.equal(layout.workSurfaceHeight, 0.1);
    assert.deepEqual(layout.taskStations.frame, [0, 0, 0.275]);

    const xml = layoutXml(layout);
    for (const name of [
      'assembly_frame',
      'manual_screwdriver',
      'torque_driver',
      'double_face_hammer',
    ]) {
      assert.match(xml, new RegExp(`name="${name}"`));
    }
  }
});

test('PiPER layout replicates four base_link roots around the enlarged workcell', () => {
  const xml = layoutXml(PIPER_ASSEMBLY1_LAYOUT);
  assert.equal(PIPER_ASSEMBLY1_LAYOUT.ringRadius, 0.78);
  assert.equal(PIPER_ASSEMBLY1_LAYOUT.primaryTcpSite, 'r0_tcp');
  assert.equal(PIPER_ASSEMBLY1_LAYOUT.primaryGripperActuator, 'r0_gripper');
  assert.deepEqual(PIPER_HOME, [0, 1.57, -1.3485, 0, 0, 0, 0.035]);
  for (let index = 0; index < 4; index += 1) {
    assert.match(
      xml,
      new RegExp(`attach model="piper_model" body="base_link" prefix="r${index}_"`),
    );
  }
});

test('UR5e layout replicates four base roots and exposes prefixed Robotiq controls', () => {
  const xml = layoutXml(UR5E_ASSEMBLY1_LAYOUT);
  assert.equal(UR5E_ASSEMBLY1_LAYOUT.ringRadius, 0.9);
  assert.equal(UR5E_ASSEMBLY1_LAYOUT.primaryTcpSite, 'r0_gripper_pinch');
  assert.equal(UR5E_ASSEMBLY1_LAYOUT.primaryGripperActuator, 'r0_gripper_fingers_actuator');
  assert.deepEqual(UR5E_HOME, [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0, 0]);
  for (let index = 0; index < 4; index += 1) {
    assert.match(
      xml,
      new RegExp(`attach model="ur5e_model" body="base" prefix="r${index}_"`),
    );
  }
});

test('alternate scenes keep Assembly1 supports, mats, and cradles as scene objects', () => {
  for (const layout of [PIPER_ASSEMBLY1_LAYOUT, UR5E_ASSEMBLY1_LAYOUT]) {
    const objectNames = new Set(layout.sceneObjects.map(({ name }) => name));
    for (const name of [
      'assembly_platform',
      'platform_inset',
      'handover_pad',
      'tool_mat_powered',
      'tool_mat_manual',
      'tool_mat_hammer',
      'torque_driver_cradle_south',
      'torque_driver_cradle_north',
    ]) {
      assert.ok(objectNames.has(name), `${name} is missing`);
    }
  }
});
