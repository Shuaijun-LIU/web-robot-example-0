import assert from 'node:assert/strict';
import test from 'node:test';

import {
  G1_ACTUATORS,
  G1_HOME,
  GO2_ACTUATORS,
  GO2_HOME,
} from '../src/unitreeActionSequence.js';
import {
  G1_WALK_READY,
  computeBalanceCorrection,
  locomotionEnvelope,
  sampleG1Gait,
  sampleG1Squat,
  sampleGo2Trot,
  sampleUnitreeLocomotionAction,
} from '../src/unitreeLocomotionController.js';

const maxAbsDiff = (left, right) => Math.max(
  ...left.map((value, index) => Math.abs(value - right[index])),
);

const zeroFeedback = {
  g1: { roll: 0, pitch: 0, rollRate: 0, pitchRate: 0, forwardSpeed: 0 },
  go2: { roll: 0, pitch: 0, rollRate: 0, pitchRate: 0, forwardSpeed: 0 },
};

test('locomotion envelope enters and exits with zero value and slope-friendly bounds', () => {
  assert.equal(locomotionEnvelope(0), 0);
  assert.equal(locomotionEnvelope(1), 0);
  assert.equal(locomotionEnvelope(-1), 0);
  assert.equal(locomotionEnvelope(2), 0);
  assert.equal(locomotionEnvelope(0.5), 1);
  for (let index = 0; index <= 100; index += 1) {
    const value = locomotionEnvelope(index / 100);
    assert.ok(value >= 0 && value <= 1);
  }
});

test('G1 squat visibly flexes symmetric legs and returns toward a walk-ready stand', () => {
  assert.deepEqual(sampleG1Squat(0), G1_HOME);
  const bottom = sampleG1Squat(1);
  assert.ok(bottom[3] > G1_HOME[3] + 0.8);
  assert.equal(bottom[3], bottom[9]);
  assert.equal(bottom[0], bottom[6]);
  assert.equal(bottom[4], bottom[10]);
  assert.deepEqual(sampleG1Squat(1, true), G1_WALK_READY);
});

test('G1 gait alternates legs and counter-swings arms', () => {
  const first = sampleG1Gait(0.25, 1, zeroFeedback.g1);
  const second = sampleG1Gait(0.75, 1, zeroFeedback.g1);
  const leftKneeA = first.targets[3] - G1_WALK_READY[3];
  const rightKneeA = first.targets[9] - G1_WALK_READY[9];
  assert.ok(leftKneeA * rightKneeA < 0);
  assert.ok((first.targets[15] - G1_WALK_READY[15])
    * (first.targets[22] - G1_WALK_READY[22]) < 0);
  assert.ok(maxAbsDiff(first.targets, second.targets) > 0.2);
  assert.ok(Math.abs(first.targets[0] - second.targets[6]) < 1e-9);
  assert.ok(Math.abs(first.targets[6] - second.targets[0]) < 1e-9);
});

test('G1 braking anticipates negative local forward speed without changing the root state', () => {
  const coasting = sampleG1Gait(0, 0, zeroFeedback.g1, 1, 0);
  const braking = sampleG1Gait(0, 0, { ...zeroFeedback.g1, forwardSpeed: -0.25 }, 1, 1);
  assert.ok(braking.targets[0] > coasting.targets[0]);
  assert.ok(braking.targets[6] > coasting.targets[6]);
  assert.ok(braking.targets[4] < coasting.targets[4]);
  assert.ok(braking.targets[10] < coasting.targets[10]);
  const lateral = sampleG1Gait(
    0,
    0,
    { ...zeroFeedback.g1, lateralSpeed: 0.2 },
    1,
    1,
  );
  assert.ok(lateral.targets[1] > coasting.targets[1]);
  assert.ok(lateral.targets[7] > coasting.targets[7]);
});

test('Go2 trot phase-locks diagonal pairs and opposes the other diagonal', () => {
  const sample = sampleGo2Trot(0.25, 1, zeroFeedback.go2);
  const offsets = [0, 3, 6, 9].map((index) => sample.targets[index + 1] - GO2_HOME[index + 1]);
  assert.ok(Math.abs(offsets[0] - offsets[3]) < 1e-9, 'FL and RR must match');
  assert.ok(Math.abs(offsets[1] - offsets[2]) < 1e-9, 'FR and RL must match');
  assert.ok(offsets[0] * offsets[1] < 0, 'diagonal groups must oppose one another');
  assert.deepEqual(sample.targets.slice(12), GO2_HOME.slice(12));
});

test('attitude feedback counters disturbances and clamps its output', () => {
  const positive = computeBalanceCorrection({
    roll: 0.1, pitch: 0.2, rollRate: 0.3, pitchRate: 0.4,
  });
  const negative = computeBalanceCorrection({
    roll: -0.1, pitch: -0.2, rollRate: -0.3, pitchRate: -0.4,
  });
  assert.ok(positive.roll < 0 && positive.pitch < 0);
  assert.ok(negative.roll > 0 && negative.pitch > 0);
  const saturated = computeBalanceCorrection({
    roll: 10, pitch: 10, rollRate: 10, pitchRate: 10,
  });
  assert.ok(Math.abs(saturated.roll) <= 0.14);
  assert.ok(Math.abs(saturated.pitch) <= 0.14);
});

test('full locomotion action is continuous at all approved boundaries', () => {
  for (const boundary of [1, 3, 5, 11, 13, 19, 21, 24, 25]) {
    const left = sampleUnitreeLocomotionAction(boundary - 1e-7, zeroFeedback);
    const right = sampleUnitreeLocomotionAction(boundary + 1e-7, zeroFeedback);
    assert.ok(maxAbsDiff(left.g1Targets, right.g1Targets) < 1e-5, `G1 discontinuity at ${boundary}`);
    assert.ok(maxAbsDiff(left.go2Targets, right.go2Targets) < 1e-5, `Go2 discontinuity at ${boundary}`);
  }
  assert.equal(sampleUnitreeLocomotionAction(7, zeroFeedback).phase, 'g1-walk');
  assert.equal(sampleUnitreeLocomotionAction(15, zeroFeedback).phase, 'go2-walk');
  assert.equal(sampleUnitreeLocomotionAction(22, zeroFeedback).phase, 'final-greeting');
});

test('every locomotion target is finite and stays in the declared actuator ranges', () => {
  for (let step = 0; step <= 2_500; step += 1) {
    const sample = sampleUnitreeLocomotionAction(step / 100, {
      g1: { roll: 0.25, pitch: -0.25, rollRate: 0.5, pitchRate: -0.5 },
      go2: { roll: -0.25, pitch: 0.25, rollRate: -0.5, pitchRate: 0.5 },
    });
    assert.ok([...sample.g1Targets, ...sample.go2Targets].every(Number.isFinite));
    sample.g1Targets.forEach((value, index) => {
      assert.ok(value >= G1_ACTUATORS[index].min && value <= G1_ACTUATORS[index].max);
    });
    sample.go2Targets.forEach((value, index) => {
      assert.ok(value >= GO2_ACTUATORS[index].min && value <= GO2_ACTUATORS[index].max);
    });
  }
});
