import assert from 'node:assert/strict';
import test from 'node:test';

import {
  createFrankaTargets,
  createSO101Targets,
  createXLeRobotTargets,
  shiftIndices,
} from '../src/controlTargets.js';
import {
  createSO101ControllerConfig,
  createXLeRobotControllerConfig,
  getFrankaGripperBinding,
} from '../src/controllers/controllerConfigs.js';

test('Franka exposes four namespaced arm targets with independent control blocks', () => {
  const targets = createFrankaTargets();

  assert.deepEqual(targets.map(({ label }) => label), ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4']);
  assert.deepEqual(targets.map(({ actuatorOffset }) => actuatorOffset), [0, 8, 16, 24]);
  assert.equal(targets[0].ik.siteName, 'r0_tcp');
  assert.equal(targets[3].ik.siteName, 'r3_tcp');
  assert.deepEqual(targets[2].ik.jointNames, [
    'r2_joint1', 'r2_joint2', 'r2_joint3', 'r2_joint4',
    'r2_joint5', 'r2_joint6', 'r2_joint7',
  ]);
  assert.deepEqual(targets[1].ik.actuatorIndices, [8, 9, 10, 11, 12, 13, 14]);
  assert.equal(targets[3].gripperActuator, 'r3_gripper');
});

test('SO101 exposes four namespaced arm targets with six-actuator strides', () => {
  const targets = createSO101Targets();

  assert.deepEqual(targets.map(({ label }) => label), ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4']);
  assert.deepEqual(targets.map(({ actuatorOffset }) => actuatorOffset), [0, 6, 12, 18]);
  assert.equal(targets[2].ik.siteName, 'r2_tcp');
  assert.deepEqual(targets[3].ik.jointNames, [
    'r3_Rotation', 'r3_Pitch', 'r3_Elbow', 'r3_Wrist_Pitch', 'r3_Wrist_Roll',
  ]);
  assert.deepEqual(targets[3].ik.actuatorIndices, [18, 19, 20, 21, 22]);
});

test('XLeRobot selects either complete sixteen-actuator robot', () => {
  const targets = createXLeRobotTargets();

  assert.deepEqual(targets.map(({ label }) => label), ['Robot 1', 'Robot 2']);
  assert.deepEqual(targets.map(({ actuatorOffset }) => actuatorOffset), [0, 16]);
  assert.deepEqual(targets.map(({ prefix }) => prefix), ['r0_', 'r1_']);
  assert.equal(targets.every(({ ik }) => ik === undefined), true);
});

test('shiftIndices moves an existing controller into a selected actuator block', () => {
  assert.deepEqual(shiftIndices([0, 2, 5], 6), [6, 8, 11]);
  assert.deepEqual(shiftIndices([2, 3, 4], 16), [18, 19, 20]);
});

test('Franka gripper binding follows the selected namespace', () => {
  const target = createFrankaTargets()[2];

  assert.deepEqual(getFrankaGripperBinding(target), {
    v: { actuator: 'r2_gripper', toggle: [0, 255] },
  });
  assert.deepEqual(getFrankaGripperBinding(target, true), {
    v: { actuator: 'r2_gripper', toggle: [255, 0] },
  });
});

test('SO101 keyboard controller writes only the selected six-actuator block', () => {
  const config = createSO101ControllerConfig(12);

  assert.equal(config.numActuators, 24);
  assert.deepEqual(config.arms[0].indices, [12, 13, 14, 15, 16, 17]);
});

test('XLeRobot keyboard controller shifts base, arms, and head together', () => {
  const config = createXLeRobotControllerConfig(16);

  assert.equal(config.numActuators, 32);
  assert.deepEqual(config.base.indices, [16, 17]);
  assert.deepEqual(config.arms[0].indices, [18, 19, 20, 21, 22, 23]);
  assert.deepEqual(config.arms[1].indices, [24, 25, 26, 27, 28, 29]);
  assert.deepEqual(config.head.indices, [30, 31]);
});
