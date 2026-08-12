import assert from 'node:assert/strict';
import test from 'node:test';

import {
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
