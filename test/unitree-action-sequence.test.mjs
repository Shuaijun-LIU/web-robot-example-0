import assert from 'node:assert/strict';
import test from 'node:test';

import {
  G1_ACTUATORS,
  G1_HOME,
  GO2_ACTUATORS,
  GO2_HOME,
  GO2_LOWER,
  UNITREE_ACTION_DURATION,
  UNITREE_ACTION_PHASES,
  applyUnitreeActionTargets,
  sampleUnitreeAction,
} from '../src/unitreeActionSequence.js';

const maxAbsDiff = (left, right) => Math.max(
  ...left.map((value, index) => Math.abs(value - right[index])),
);

test('action clip has the exact timeline and home endpoints', () => {
  assert.equal(UNITREE_ACTION_DURATION, 10);
  assert.deepEqual(
    UNITREE_ACTION_PHASES.map(({ name, duration }) => [name, duration]),
    [
      ['settle', 1],
      ['rise-greet', 1.5],
      ['scan-wave', 3],
      ['lower', 1.5],
      ['recover', 1.5],
      ['final-hold', 1.5],
    ],
  );
  assert.deepEqual(sampleUnitreeAction(0).g1Targets, G1_HOME);
  assert.deepEqual(sampleUnitreeAction(0).go2Targets, GO2_HOME);
  assert.deepEqual(sampleUnitreeAction(10).g1Targets, G1_HOME);
  assert.deepEqual(sampleUnitreeAction(10).go2Targets, GO2_HOME);
});

test('action targets remain continuous at every phase boundary', () => {
  for (const boundary of [1, 2.5, 5.5, 7, 8.5, 10]) {
    const left = sampleUnitreeAction(boundary - 1e-7);
    const right = sampleUnitreeAction(boundary + 1e-7);
    assert.ok(maxAbsDiff(left.g1Targets, right.g1Targets) < 1e-5);
    assert.ok(maxAbsDiff(left.go2Targets, right.go2Targets) < 1e-5);
  }
});

test('scan-wave articulates both robots and lower uses the official Go2 pose', () => {
  const scan = sampleUnitreeAction(4);
  assert.equal(scan.phase, 'scan-wave');
  assert.ok(Math.abs(scan.g1Targets[26] - G1_HOME[26]) > 0.25);
  assert.ok(Math.abs(scan.go2Targets[12] - GO2_HOME[12]) > 0.25);
  assert.deepEqual(sampleUnitreeAction(7).go2Targets.slice(0, 12), GO2_LOWER);
});

test('all action samples are finite and remain inside literal control limits', () => {
  assert.equal(G1_ACTUATORS.length, 29);
  assert.equal(GO2_ACTUATORS.length, 18);
  for (let step = 0; step <= 1_000; step += 1) {
    const sample = sampleUnitreeAction(step / 100);
    assert.ok([...sample.g1Targets, ...sample.go2Targets].every(Number.isFinite));
    sample.g1Targets.forEach((value, index) => {
      assert.ok(value >= G1_ACTUATORS[index].min && value <= G1_ACTUATORS[index].max);
    });
    sample.go2Targets.forEach((value, index) => {
      assert.ok(value >= GO2_ACTUATORS[index].min && value <= GO2_ACTUATORS[index].max);
    });
  }
});

test('writer mutates only the explicitly mapped 47 actuator slots', () => {
  const ctrl = new Float64Array(52).fill(123.456);
  const mappedIds = Array.from({ length: 47 }, (_, index) => (index * 7) % 52);
  const actuatorIds = {
    g1: mappedIds.slice(0, 29),
    go2: mappedIds.slice(29),
  };
  const sample = sampleUnitreeAction(4);

  applyUnitreeActionTargets(ctrl, actuatorIds, sample);

  const mapped = new Set(mappedIds);
  ctrl.forEach((value, index) => {
    if (!mapped.has(index)) assert.equal(value, 123.456);
  });
  actuatorIds.g1.forEach((id, index) => assert.equal(ctrl[id], sample.g1Targets[index]));
  actuatorIds.go2.forEach((id, index) => assert.equal(ctrl[id], sample.go2Targets[index]));
});

test('writer validates every target and id before changing ctrl', () => {
  const ctrl = new Float64Array(52).fill(9);
  const ids = {
    g1: Array.from({ length: 29 }, (_, index) => index),
    go2: Array.from({ length: 18 }, (_, index) => index + 29),
  };
  const badSample = sampleUnitreeAction(4);
  badSample.go2Targets[17] = Number.NaN;

  assert.throws(() => applyUnitreeActionTargets(ctrl, ids, badSample), /finite/);
  assert.deepEqual([...ctrl], new Array(52).fill(9));
});
