import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import {
  ASSEMBLY1_STEP3_DURATIONS,
  ASSEMBLY1_STEP3_GRIPPER_CLAMPS,
  ASSEMBLY1_STEP3_LIMITS,
  ASSEMBLY1_STEP3_TRANSPORT_ARMS,
  ASSEMBLY1_STEP3_WAYPOINTS,
  advanceAssemblyStep3Machine,
  createAssemblyStep3ControlFrame,
  createAssemblyStep3Machine,
  evaluateAssemblyStep3Alignment,
  evaluateAssemblyStep3Transport,
  holdAssemblyStep3Controls,
} from '../src/assemblyStep3.js';

test('Step 3 preserves the measured dual-grasp span while moving to the interface', () => {
  assert.deepEqual(ASSEMBLY1_STEP3_WAYPOINTS, {
    start: [[-0.49, 0.56, 0.20], [-0.49, 0.32, 0.20]],
    lift: [[-0.49, 0.5675, 0.38], [-0.49, 0.3125, 0.38]],
    transferA: [[-0.245, 0.3475, 0.38], [-0.245, 0.0925, 0.38]],
    transferMid: [[-0.1225, 0.2375, 0.37], [-0.1225, -0.0175, 0.37]],
    hover: [[0, 0.1275, 0.34], [0, -0.1275, 0.34]],
    descentMid: [[0, 0.1275, 0.295], [0, -0.1275, 0.295]],
    aligned: [[0.006, 0.1275, 0.278], [0.002, -0.1275, 0.278]],
  });
  assert.ok(Math.abs(
    ASSEMBLY1_STEP3_WAYPOINTS.start[0][1]
      - ASSEMBLY1_STEP3_WAYPOINTS.start[1][1]
      - 0.24,
  ) < 1e-12);
  for (const pair of Object.values(ASSEMBLY1_STEP3_WAYPOINTS).slice(1)) {
    assert.ok(Math.abs(pair[0][1] - pair[1][1] - 0.255) < 1e-12);
  }
  assert.deepEqual(ASSEMBLY1_STEP3_GRIPPER_CLAMPS, [48, 96, 24, 24]);
  assert.deepEqual(ASSEMBLY1_STEP3_LIMITS, {
    minimumAperture: 0.02,
    frameTranslation: 0.008,
    holePlanarDistance: 0.035,
    holeVerticalOffset: 0.025,
    seatedVerticalOffset: 0.02,
    comparisonEpsilon: 0.001,
  });
});

test('every Step 3 transport waypoint contains a generated Panda joint solution', () => {
  assert.deepEqual(ASSEMBLY1_STEP3_TRANSPORT_ARMS.map(({ key, armIndex, closingAxisYawDegrees }) => ({
    key,
    armIndex,
    closingAxisYawDegrees,
  })), [
    { key: 'r2', armIndex: 2, closingAxisYawDegrees: 0 },
    { key: 'r3', armIndex: 3, closingAxisYawDegrees: 0 },
  ]);
  for (const arm of ASSEMBLY1_STEP3_TRANSPORT_ARMS) {
    for (const name of [
      'liftJointTargets',
      'transferAJointTargets',
      'transferMidJointTargets',
      'hoverJointTargets',
      'descentMidJointTargets',
      'alignedJointTargets',
    ]) {
      assert.equal(arm[name].length, 7, `${arm.key}/${name} must contain seven joints`);
      assert.ok(arm[name].every(Number.isFinite), `${arm.key}/${name} must be finite`);
    }
  }
});

test('Step 3 follows transport, alignment, physical release, retreat, and placed hold', () => {
  let machine = createAssemblyStep3Machine();
  assert.equal(machine.phase, 'grasp-check');

  machine = advanceAssemblyStep3Machine(machine, 0.15, { all: { ok: true } });
  assert.equal(machine.phase, 'grasp-check');
  machine = advanceAssemblyStep3Machine(machine, 0.10, { all: { ok: true } });
  assert.equal(machine.phase, 'lift');

  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.lift, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'lift-settle');
  machine = advanceAssemblyStep3Machine(machine, 0.25, { all: { ok: true } });
  assert.equal(machine.phase, 'transfer-a');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.transferA, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'transfer-b');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.transferB, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'hover-settle');
  machine = advanceAssemblyStep3Machine(machine, 0.25, { all: { ok: true } });
  assert.equal(machine.phase, 'aligned-descent');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.alignedDescent, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'alignment-verification');
  machine = advanceAssemblyStep3Machine(machine, 0.25, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'aligned-hold');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.alignedHold, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'release');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.release, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'release-settle');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.releaseSettle, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'retreat');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.retreat, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'placed-verification');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.placedHold, {
    all: { ok: true },
    alignment: { ok: true },
  });
  assert.equal(machine.phase, 'complete');
});

test('Step 3 performs one physical reseat before reporting persistent misalignment', () => {
  let machine = {
    phase: 'alignment-verification',
    phaseElapsed: 3.99,
    continuousValidSeconds: 0,
    reseatAttempts: 0,
    failure: null,
  };
  machine = advanceAssemblyStep3Machine(machine, 0.02, {
    all: { ok: true },
    alignment: { ok: false, code: 'hole-misalignment', detail: '0.012' },
  });
  assert.equal(machine.phase, 'reseat-lift');
  assert.equal(machine.reseatAttempts, 1);
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.reseatLift, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'reseat-descent');
  machine = advanceAssemblyStep3Machine(machine, ASSEMBLY1_STEP3_DURATIONS.reseatDescent, {
    all: { ok: true },
  });
  assert.equal(machine.phase, 'alignment-verification');
  machine = advanceAssemblyStep3Machine({ ...machine, phaseElapsed: 3.99 }, 0.02, {
    all: { ok: true },
    alignment: { ok: false, code: 'hole-misalignment', detail: '0.011' },
  });
  assert.equal(machine.phase, 'error');
  assert.equal(machine.failure.code, 'hole-misalignment');
});

test('Step 3 control frames leave Arms 1/2 fixed and synchronize Arms 3/4', () => {
  const plans = [0, 1, 2, 3].map((index) => ({
    armKey: `r${index}`,
    hold: Array(7).fill(index * 10),
    lift: Array(7).fill(index * 10 + 2),
    transferA: Array(7).fill(index * 10 + 4),
    transferMid: Array(7).fill(index * 10 + 5),
    hover: Array(7).fill(index * 10 + 6),
    descentMid: Array(7).fill(index * 10 + 7),
    aligned: Array(7).fill(index * 10 + 8),
  }));
  const frame = createAssemblyStep3ControlFrame({
    phase: 'transfer-a',
    phaseElapsed: ASSEMBLY1_STEP3_DURATIONS.transferA / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);

  assert.deepEqual(frame.arms[0].jointTargets, Array(7).fill(0));
  assert.deepEqual(frame.arms[1].jointTargets, Array(7).fill(10));
  assert.deepEqual(frame.arms[2].jointTargets, Array(7).fill(23));
  assert.deepEqual(frame.arms[3].jointTargets, Array(7).fill(33));
  assert.deepEqual(frame.arms.map((arm) => arm.gripperTarget), [48, 96, 24, 24]);

  const transferMidFrame = createAssemblyStep3ControlFrame({
    phase: 'transfer-b',
    phaseElapsed: ASSEMBLY1_STEP3_DURATIONS.transferB / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(transferMidFrame.arms[2].jointTargets, Array(7).fill(25));
  assert.deepEqual(transferMidFrame.arms[3].jointTargets, Array(7).fill(35));

  const descentMidFrame = createAssemblyStep3ControlFrame({
    phase: 'aligned-descent',
    phaseElapsed: ASSEMBLY1_STEP3_DURATIONS.alignedDescent / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(descentMidFrame.arms[2].jointTargets, Array(7).fill(27));
  assert.deepEqual(descentMidFrame.arms[3].jointTargets, Array(7).fill(37));

  const releaseFrame = createAssemblyStep3ControlFrame({
    phase: 'release',
    phaseElapsed: ASSEMBLY1_STEP3_DURATIONS.release / 2,
    continuousValidSeconds: 0,
    reseatAttempts: 0,
    failure: null,
  }, plans);
  assert.deepEqual(releaseFrame.arms.map((arm) => arm.gripperTarget), [48, 96, 139.5, 139.5]);

  const retreatFrame = createAssemblyStep3ControlFrame({
    phase: 'retreat',
    phaseElapsed: ASSEMBLY1_STEP3_DURATIONS.retreat / 2,
    continuousValidSeconds: 0,
    reseatAttempts: 0,
    failure: null,
  }, plans);
  assert.deepEqual(retreatFrame.arms[2].jointTargets, Array(7).fill(27));
  assert.deepEqual(retreatFrame.arms[3].jointTargets, Array(7).fill(37));
  assert.deepEqual(retreatFrame.arms.map((arm) => arm.gripperTarget), [48, 96, 255, 255]);
});

test('Step 3 transport requires bilateral target contact and a non-empty aperture', () => {
  const valid = {
    targetBody: 'cross_member',
    leftContactBodies: ['cross_member'],
    rightContactBodies: ['cross_member'],
    forbiddenBodies: [],
    aperture: 0.03,
  };
  assert.deepEqual(evaluateAssemblyStep3Transport(valid), { ok: true });
  assert.equal(evaluateAssemblyStep3Transport({ ...valid, leftContactBodies: [] }).code,
    'missing-left-contact');
  assert.equal(evaluateAssemblyStep3Transport({ ...valid, rightContactBodies: [] }).code,
    'missing-right-contact');
  assert.equal(evaluateAssemblyStep3Transport({ ...valid, forbiddenBodies: ['parts_tray'] }).code,
    'forbidden-contact');
  assert.equal(evaluateAssemblyStep3Transport({ ...valid, aperture: 0.02 }).code,
    'empty-closure');
  assert.deepEqual(evaluateAssemblyStep3Transport({
    ...valid,
    leftContactBodies: [],
    rightContactBodies: [],
    requireBilateralContact: false,
  }), { ok: true });
});

test('Step 3 alignment separates strict planar error from seated vertical offset', () => {
  const valid = {
    holePlanarDistances: [0.003, 0.004, 0.005, 0.006],
    holeVerticalOffsets: [0.0156, 0.015, 0.014, 0.016],
    frameTranslation: 0.004,
    crossMemberRotationDegrees: 2,
  };
  assert.deepEqual(evaluateAssemblyStep3Alignment(valid), { ok: true });
  assert.deepEqual(evaluateAssemblyStep3Alignment({
    ...valid,
    holePlanarDistances: [0.003, 0.02224061821727346, 0.005, 0.006],
  }), { ok: true });
  assert.equal(evaluateAssemblyStep3Alignment({
    ...valid,
    holePlanarDistances: [0.003, 0.0361, 0.005, 0.006],
  }).code,
    'hole-misalignment');
  assert.equal(evaluateAssemblyStep3Alignment({
    ...valid,
    holeVerticalOffsets: [0.015, 0.0265, 0.014, 0.016],
  }).code, 'hole-height');
  assert.equal(evaluateAssemblyStep3Alignment({ ...valid, frameTranslation: 0.0081 }).code,
    'frame-drift');
  assert.deepEqual(evaluateAssemblyStep3Alignment({
    ...valid,
    crossMemberRotationDegrees: 30,
  }), { ok: true });
  assert.equal(evaluateAssemblyStep3Alignment({
    ...valid,
    verticalTolerance: 0.014,
  }).code, 'hole-height');
  assert.equal(evaluateAssemblyStep3Alignment({
    ...valid,
    holePlanarDistances: [0.003, Number.NaN, 0.005, 0.006],
  }).code,
    'non-finite-runtime');
});

test('Step 3 stops on a lost physical grasp and preserves the current gripper commands', () => {
  const failure = advanceAssemblyStep3Machine({
    phase: 'transfer-b',
    phaseElapsed: 0.4,
    continuousValidSeconds: 0,
    failure: null,
  }, 0.01, {
    all: { ok: false, code: 'missing-right-contact', armKey: 'r3' },
  });
  assert.deepEqual(failure.failure, {
    code: 'missing-right-contact',
    armKey: 'r3',
  });

  const controls = new Float64Array(12).fill(-9);
  const positions = new Float64Array([0.1, 0.2, 0.3, 9, 1.1, 1.2, 1.3]);
  controls[3] = 37;
  controls[11] = 81;
  holdAssemblyStep3Controls(controls, positions, [
    { actuatorIndices: [0, 1, 2], qposAddresses: [0, 1, 2], gripperActuatorIndex: 3 },
    { actuatorIndices: [8, 9, 10], qposAddresses: [4, 5, 6], gripperActuatorIndex: 11 },
  ]);
  assert.deepEqual(Array.from(controls.slice(0, 4)), [0.1, 0.2, 0.3, 37]);
  assert.deepEqual(Array.from(controls.slice(8, 12)), [1.1, 1.2, 1.3, 81]);
});

test('Step 3 release motion allows gravity settling before strict placed verification', () => {
  const releasing = advanceAssemblyStep3Machine({
    phase: 'release',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    reseatAttempts: 0,
    failure: null,
  }, 0.1, {
    all: { ok: true },
    alignment: { ok: false, code: 'hole-height', detail: 'settling' },
  });
  assert.equal(releasing.phase, 'release');
});

test('Step 3 runtime owns only Panda actuators and exposes physical evidence', async () => {
  const source = await readFile(
    new URL('../src/AssemblyStep3Controller.tsx', import.meta.url),
    'utf8',
  ).catch(() => '');
  assert.match(source, /ownershipRef\.current\s*=\s*'step3'/);
  assert.match(source, /createAssemblyStep3ControlFrame/);
  assert.match(source, /evaluateAssemblyStep3Transport/);
  assert.match(source, /evaluateAssemblyStep3Alignment/);
  assert.match(source, /crossMemberQuaternion,\s*\n\s*\[1, 0, 0, 0\]/);
  assert.doesNotMatch(source, /crossMemberBaselineQuaternion/);
  assert.match(source, /holdAssemblyStep3Controls\(data\.ctrl, data\.qpos/);
  assert.doesNotMatch(source, /data\.qpos\s*\[[^\]]+\]\s*=/);
  assert.doesNotMatch(source, /data\.qvel\s*\[[^\]]+\]\s*=/);
  assert.doesNotMatch(source, /qfrc_applied|mjEQ_WELD|magnet|proximity|scripted.*pose/i);
});
