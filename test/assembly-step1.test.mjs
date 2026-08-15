import assert from 'node:assert/strict';
import test from 'node:test';

import * as assemblyStep1 from '../src/assemblyStep1.js';

const {
  ASSEMBLY1_GRIPPER_OPEN,
  ASSEMBLY1_STEP1_ARMS,
  ASSEMBLY1_STEP1_PHASE_DURATION,
  ASSEMBLY1_STEP1_SETTLE_DURATION,
  interpolateJointTargets,
  isCompleteAssemblyStep1Plan,
  selectAssemblyStep1Phase,
  smoothstep01,
  topDownTcpQuaternion,
  holdAssemblyJointState,
} = assemblyStep1;

test('Assembly1 Step 1 assigns all four arms grasp-ready pre-grasp targets', () => {
  assert.equal(typeof topDownTcpQuaternion, 'function');
  assert.equal(ASSEMBLY1_STEP1_PHASE_DURATION, 1.5);
  assert.equal(ASSEMBLY1_GRIPPER_OPEN, 255);
  assert.deepEqual(
    ASSEMBLY1_STEP1_ARMS.map(({
      role,
      highWaypoint,
      finalWaypoint,
      closingAxisYawDegrees,
    }) => ({
      role,
      highWaypoint,
      finalWaypoint,
      closingAxisYawDegrees,
    })),
    [
      {
        role: 'south frame rail',
        highWaypoint: [0.18, -0.23, 0.50],
        finalWaypoint: [0.18, -0.23, 0.33],
        closingAxisYawDegrees: 90,
      },
      {
        role: 'side-laid torque driver handle',
        highWaypoint: [0.559, -0.421, 0.48],
        finalWaypoint: [0.559, -0.421, 0.28],
        closingAxisYawDegrees: 162,
      },
      {
        role: 'cross member north balance point',
        highWaypoint: [-0.49, 0.56, 0.48],
        finalWaypoint: [-0.49, 0.56, 0.32],
        closingAxisYawDegrees: 0,
      },
      {
        role: 'cross member south balance point',
        highWaypoint: [-0.49, 0.32, 0.48],
        finalWaypoint: [-0.49, 0.32, 0.32],
        closingAxisYawDegrees: 0,
      },
    ],
  );
  assert.ok(Math.abs(
    ASSEMBLY1_STEP1_ARMS[2].finalWaypoint[1]
    - ASSEMBLY1_STEP1_ARMS[3].finalWaypoint[1],
  ) >= 0.21);
  assert.ok(Math.abs(ASSEMBLY1_STEP1_ARMS[0].finalWaypoint[0]) >= 0.15);

  assert.deepEqual(topDownTcpQuaternion(90), [0, 1, 0, 0]);
  assert.deepEqual(
    topDownTcpQuaternion(0).map((value) => Number(value.toFixed(6))),
    [0.707107, 0.707107, 0, 0],
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

  for (const arm of ASSEMBLY1_STEP1_ARMS) {
    assert.deepEqual(arm.tcpQuaternion, topDownTcpQuaternion(arm.closingAxisYawDegrees));
  }
});

test('Assembly1 Step 1 uses the verified grasp-ready IK generation', () => {
  assert.equal(assemblyStep1.ASSEMBLY1_STEP1_IK_VERSION, 'installation-clearance-v4');
  for (const arm of ASSEMBLY1_STEP1_ARMS) {
    assert.equal(arm.highJointTargets.length, 7);
    assert.equal(arm.finalJointTargets.length, 7);
  }
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

test('Assembly1 can hold only selected robot joints while waiting for Step 2', () => {
  const controls = new Float64Array([9, 9, 9, 9]);
  const positions = new Float64Array([1, 2, 3, 4]);
  const velocities = new Float64Array([1, 2, 3, 4, 5]);
  holdAssemblyJointState(controls, positions, velocities, [{
    actuatorIndices: [0, 2],
    qposAddresses: [1, 3],
    dofAddresses: [1, 3],
    positions: [2.5, 4.5],
  }]);
  assert.deepEqual(Array.from(controls), [2.5, 9, 4.5, 9]);
  assert.deepEqual(Array.from(positions), [1, 2.5, 3, 4.5]);
  assert.deepEqual(Array.from(velocities), [1, 0, 3, 0, 5]);
});

test('Assembly1 Step 1 stages motion, settles, and accepts only an atomic four-arm plan', () => {
  assert.equal(ASSEMBLY1_STEP1_SETTLE_DURATION, 3);
  assert.deepEqual(selectAssemblyStep1Phase(0), { phase: 'high', progress: 0 });
  assert.deepEqual(selectAssemblyStep1Phase(0.75), { phase: 'high', progress: 0.5 });
  assert.deepEqual(selectAssemblyStep1Phase(1.5), { phase: 'final', progress: 0 });
  assert.deepEqual(selectAssemblyStep1Phase(2.25), { phase: 'final', progress: 0.5 });
  assert.deepEqual(selectAssemblyStep1Phase(3), { phase: 'settling', progress: 0 });
  assert.deepEqual(selectAssemblyStep1Phase(4.5), { phase: 'settling', progress: 0.5 });
  assert.deepEqual(selectAssemblyStep1Phase(6), { phase: 'complete', progress: 1 });

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
