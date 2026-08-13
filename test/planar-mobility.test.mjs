import assert from 'node:assert/strict';
import test from 'node:test';

import { computePlanarVelocity } from '../src/controllers/planarMobility.js';

test('planar mobility converts forward speed through the robot heading', () => {
  const [vx, vy, yawRate] = computePlanarVelocity({
    forward: 1,
    turn: 0,
    yaw: Math.PI / 2,
    linearSpeed: 0.42,
    turnSpeed: 0.8,
  });

  assert.ok(Math.abs(vx) < 1e-10);
  assert.ok(Math.abs(vy - 0.42) < 1e-10);
  assert.equal(yawRate, 0);
});

test('planar mobility supports reverse and simultaneous turning', () => {
  const [vx, vy, yawRate] = computePlanarVelocity({
    forward: -1,
    turn: 1,
    yaw: Math.PI,
    linearSpeed: 0.42,
    turnSpeed: 0.8,
  });

  assert.ok(Math.abs(vx - 0.42) < 1e-10);
  assert.ok(Math.abs(vy) < 1e-10);
  assert.equal(yawRate, 0.8);
});
