import assert from 'node:assert/strict';
import test from 'node:test';

import {
  ASSEMBLY1_GRIPPER_OPEN,
  ASSEMBLY1_STEP1_ARMS,
  ASSEMBLY1_STEP1_PHASE_DURATION,
  interpolateJointTargets,
  isCompleteAssemblyStep1Plan,
  selectAssemblyStep1Phase,
  smoothstep01,
} from '../src/assemblyStep1.js';

test('Assembly1 Step 1 assigns all four arms distinct namespaced pre-grasp jobs', () => {
  assert.equal(ASSEMBLY1_STEP1_PHASE_DURATION, 1.5);
  assert.equal(ASSEMBLY1_GRIPPER_OPEN, 255);
  assert.deepEqual(
    ASSEMBLY1_STEP1_ARMS.map(({ key, label, role }) => ({ key, label, role })),
    [
      { key: 'r0', label: 'Arm 1', role: 'south frame grip' },
      { key: 'r1', label: 'Arm 2', role: 'torque driver' },
      { key: 'r2', label: 'Arm 3', role: 'cross member north grip' },
      { key: 'r3', label: 'Arm 4', role: 'west frame grip' },
    ],
  );
  assert.deepEqual(
    ASSEMBLY1_STEP1_ARMS.map(({ highWaypoint, finalWaypoint }) => ({
      highWaypoint,
      finalWaypoint,
    })),
    [
      { highWaypoint: [0, -0.36, 0.50], finalWaypoint: [0, -0.31, 0.34] },
      { highWaypoint: [0.53, -0.42, 0.50], finalWaypoint: [0.53, -0.42, 0.36] },
      { highWaypoint: [-0.49, 0.65, 0.50], finalWaypoint: [-0.49, 0.65, 0.34] },
      { highWaypoint: [-0.48, 0, 0.50], finalWaypoint: [-0.46, 0, 0.34] },
    ],
  );

  for (const [index, arm] of ASSEMBLY1_STEP1_ARMS.entries()) {
    assert.equal(arm.siteName, `r${index}_tcp`);
    assert.deepEqual(arm.jointNames, [1, 2, 3, 4, 5, 6, 7].map(
      (joint) => `r${index}_joint${joint}`,
    ));
    assert.deepEqual(
      arm.actuatorIndices,
      [0, 1, 2, 3, 4, 5, 6].map((offset) => index * 8 + offset),
    );
    assert.equal(arm.gripperActuatorIndex, index * 8 + 7);
    assert.equal(arm.highJointTargets.length, 7);
    assert.equal(arm.finalJointTargets.length, 7);
    assert.ok(arm.highJointTargets.every(Number.isFinite));
    assert.ok(arm.finalJointTargets.every(Number.isFinite));
  }

  assert.deepEqual(ASSEMBLY1_STEP1_ARMS[0].highJointTargets, [
    1.820193, -0.069366, -0.242346, -1.970456, -0.01791, 1.907045, 2.366129,
  ]);
  assert.deepEqual(ASSEMBLY1_STEP1_ARMS[1].finalJointTargets, [
    0.538023, -0.41985, 1.801701, -2.100681, 0.489855, 2.147852, 2.86118,
  ]);
});

test('Assembly1 Step 1 joint interpolation clamps and eases phase progress', () => {
  assert.equal(smoothstep01(-0.5), 0);
  assert.equal(smoothstep01(0), 0);
  assert.equal(smoothstep01(0.5), 0.5);
  assert.equal(smoothstep01(1), 1);
  assert.equal(smoothstep01(1.5), 1);

  assert.deepEqual(interpolateJointTargets([0, 10], [10, 20], -1), [0, 10]);
  assert.deepEqual(interpolateJointTargets([0, 10], [10, 20], 0.5), [5, 15]);
  assert.deepEqual(interpolateJointTargets([0, 10], [10, 20], 2), [10, 20]);
});

test('Assembly1 Step 1 selects two phases and accepts only an atomic four-arm plan', () => {
  assert.deepEqual(selectAssemblyStep1Phase(0), { phase: 'high', progress: 0 });
  assert.deepEqual(selectAssemblyStep1Phase(0.75), { phase: 'high', progress: 0.5 });
  assert.deepEqual(selectAssemblyStep1Phase(1.5), { phase: 'final', progress: 0 });
  assert.deepEqual(selectAssemblyStep1Phase(2.25), { phase: 'final', progress: 0.5 });
  assert.deepEqual(selectAssemblyStep1Phase(3), { phase: 'complete', progress: 1 });

  const completePlan = Array.from({ length: 4 }, (_, arm) => ({
    armKey: `r${arm}`,
    start: Array(7).fill(0),
    high: Array(7).fill(arm + 0.5),
    final: Array(7).fill(arm + 1),
  }));
  assert.equal(isCompleteAssemblyStep1Plan(completePlan), true);
  assert.equal(isCompleteAssemblyStep1Plan(completePlan.slice(0, 3)), false);
  assert.equal(isCompleteAssemblyStep1Plan([
    ...completePlan.slice(0, 2),
    null,
    completePlan[3],
  ]), false);
  assert.equal(isCompleteAssemblyStep1Plan([
    ...completePlan.slice(0, 3),
    { ...completePlan[3], final: [1, 2] },
  ]), false);
});
