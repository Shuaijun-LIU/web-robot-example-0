import assert from 'node:assert/strict';
import test from 'node:test';

import { sampleUnitreeLocomotionAction } from '../src/unitreeLocomotionController.js';
import {
  computeRootDisplacement,
  quaternionToRollPitch,
  readUnitreeRootState,
  resolveUnitreeFreeRootAddresses,
  validateLocomotionTargets,
  validateUnitreeDynamicsState,
} from '../src/unitreeDynamicsAdapter.js';

const closeTo = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
};

test('free-root resolver requires the two namespaced MuJoCo free joints', () => {
  const model = {
    jnt_qposadr: new Int32Array([11, 31]),
    jnt_dofadr: new Int32Array([17, 37]),
    jnt_type: new Int32Array([0, 0]),
  };
  const ids = new Map([
    ['g1_floating_base_joint', 0],
    ['go2_floating_base_joint', 1],
  ]);
  assert.deepEqual(
    resolveUnitreeFreeRootAddresses(model, (name) => ids.get(name) ?? -1),
    {
      g1: { jointId: 0, qposAddress: 11, dofAddress: 17 },
      go2: { jointId: 1, qposAddress: 31, dofAddress: 37 },
    },
  );
  assert.throws(
    () => resolveUnitreeFreeRootAddresses(model, () => -1),
    /free joint g1_floating_base_joint/,
  );
  assert.throws(
    () => resolveUnitreeFreeRootAddresses({ ...model, jnt_type: new Int32Array([3, 0]) }, (name) => ids.get(name) ?? -1),
    /must be a MuJoCo free joint/,
  );
});

test('quaternion conversion recovers roll and pitch in radians', () => {
  const roll = 0.24;
  const pitch = -0.18;
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const quaternion = [cr * cp, sr * cp, cr * sp, -sr * sp];
  const result = quaternionToRollPitch(quaternion);
  closeTo(result.roll, roll);
  closeTo(result.pitch, pitch);
});

test('root reader extracts pose, rates, and planar displacement without mutating state', () => {
  const qpos = new Float64Array(20);
  const qvel = new Float64Array(18);
  qpos.set([1.2, -0.4, 0.81, 1, 0, 0, 0], 3);
  qvel.set([0.3, 0.4, 0.1, 0.2, -0.1, 0.05], 6);
  const beforeQpos = [...qpos];
  const beforeQvel = [...qvel];
  const state = readUnitreeRootState(qpos, qvel, { qposAddress: 3, dofAddress: 6 });
  assert.deepEqual(state.position, [1.2, -0.4, 0.81]);
  assert.deepEqual(state.velocity, [0.3, 0.4, 0.1]);
  assert.deepEqual(state.angularVelocity, [0.2, -0.1, 0.05]);
  closeTo(state.speed, 0.5);
  closeTo(state.forwardSpeed, 0.3);
  closeTo(state.roll, 0);
  closeTo(state.pitch, 0);
  assert.deepEqual([...qpos], beforeQpos);
  assert.deepEqual([...qvel], beforeQvel);

  const displacement = computeRootDisplacement(
    { ...state, position: [-1.1, 0, 0.8] },
    { ...state, position: [-0.65, 0.2, 0.8] },
  );
  closeTo(displacement.x, 0.45);
  closeTo(displacement.y, 0.2);
  closeTo(displacement.planar, Math.hypot(0.45, 0.2));
});

test('root reader expresses forward speed in the robot frame after a yaw turn', () => {
  const qpos = new Float64Array(7);
  const qvel = new Float64Array(6);
  qpos.set([0, 0, 0.8, 0, 0, 0, 1]);
  qvel.set([-0.35, 0, 0, 0, 0, 0]);
  const state = readUnitreeRootState(qpos, qvel, { qposAddress: 0, dofAddress: 0 });
  closeTo(state.forwardSpeed, 0.35);
});

test('dynamics safety distinguishes valid standing state, falls, and NaN', () => {
  const standing = {
    g1: { position: [0, 0, 0.79], roll: 0.1, pitch: -0.1 },
    go2: { position: [0, 0, 0.29], roll: -0.1, pitch: 0.1 },
  };
  assert.deepEqual(validateUnitreeDynamicsState(standing), { safe: true, reason: null });
  assert.match(
    validateUnitreeDynamicsState({ ...standing, g1: { ...standing.g1, position: [0, 0, 0.49] } }).reason,
    /G1 pelvis height/,
  );
  assert.match(
    validateUnitreeDynamicsState({ ...standing, go2: { ...standing.go2, roll: Math.PI } }).reason,
    /Go2 base tilt/,
  );
  assert.match(
    validateUnitreeDynamicsState({ ...standing, g1: { ...standing.g1, pitch: Number.NaN } }).reason,
    /non-finite/,
  );
});

test('47-target validator is atomic and rejects non-finite or out-of-range targets', () => {
  const valid = sampleUnitreeLocomotionAction(7);
  assert.equal(validateLocomotionTargets(valid), valid);
  assert.throws(
    () => validateLocomotionTargets({ ...valid, g1Targets: valid.g1Targets.slice(1) }),
    /29 G1 targets/,
  );
  const nan = { ...valid, go2Targets: [...valid.go2Targets] };
  nan.go2Targets[4] = Number.NaN;
  assert.throws(() => validateLocomotionTargets(nan), /finite/);
  const outOfRange = { ...valid, g1Targets: [...valid.g1Targets] };
  outOfRange.g1Targets[0] = 99;
  assert.throws(() => validateLocomotionTargets(outOfRange), /control range/);
});
