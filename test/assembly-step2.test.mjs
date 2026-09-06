import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import {
  ASSEMBLY1_STEP2_ARMS,
  ASSEMBLY1_STEP2_DURATIONS,
  ASSEMBLY1_STEP2_GRIPPER_CLAMPS,
  ASSEMBLY1_STEP2_LIMITS,
  advanceAssemblyStep2Machine,
  createAssemblyStep2ControlFrame,
  createAssemblyStep2Machine,
  captureAssemblyStep2JointTargets,
  evaluateAssemblyStep2Grasp,
  interpolateAssemblyStep2Gripper,
  quaternionAngularDistanceDegrees,
  releaseAssemblyStep2Controls,
} from '../src/assemblyStep2.js';
import { ASSEMBLY1_STEP1_ARMS } from '../src/assemblyStep1.js';

test('Step 2 assigns exact physical contact geometry to all four arms', () => {
  assert.deepEqual(ASSEMBLY1_STEP2_ARMS.map((arm) => ({
    key: arm.key,
    targetBody: arm.targetBody,
    contactWaypoint: arm.contactWaypoint,
    approachWaypoint: arm.approachWaypoint,
    closingAxisYawDegrees: arm.closingAxisYawDegrees,
    leftFingerBody: arm.leftFingerBody,
    rightFingerBody: arm.rightFingerBody,
  })), [
    {
      key: 'r0',
      targetBody: 'assembly_frame',
      contactWaypoint: [0.18, -0.23, 0.25],
      approachWaypoint: [0.18, -0.23, 0.265],
      closingAxisYawDegrees: -90,
      leftFingerBody: 'r0_left_finger',
      rightFingerBody: 'r0_right_finger',
    },
    {
      key: 'r1',
      targetBody: 'double_face_hammer',
      contactWaypoint: [0.675, -0.421, 0.197],
      approachWaypoint: [0.675, -0.421, 0.212],
      closingAxisYawDegrees: 90,
      leftFingerBody: 'r1_left_finger',
      rightFingerBody: 'r1_right_finger',
    },
    {
      key: 'r2',
      targetBody: 'cross_member',
      contactWaypoint: [-0.489, 0.56, 0.21],
      approachWaypoint: [-0.489, 0.56, 0.225],
      closingAxisYawDegrees: 0,
      leftFingerBody: 'r2_left_finger',
      rightFingerBody: 'r2_right_finger',
    },
    {
      key: 'r3',
      targetBody: 'cross_member',
      contactWaypoint: [-0.482, 0.32, 0.213],
      approachWaypoint: [-0.482, 0.32, 0.228],
      closingAxisYawDegrees: 0,
      leftFingerBody: 'r3_left_finger',
      rightFingerBody: 'r3_right_finger',
    },
  ]);
  assert.ok(
    Math.abs(Math.abs(
      ASSEMBLY1_STEP2_ARMS[2].contactWaypoint[1]
      - ASSEMBLY1_STEP2_ARMS[3].contactWaypoint[1],
    ) - 0.24) < 1e-12,
  );
  assert.deepEqual(ASSEMBLY1_STEP2_DURATIONS, {
    approach: 1.4,
    slowDescent: 0.8,
    contactSettle: 1.5,
    frameClamp: 0.8,
    crossMemberClamp: 1,
    hammerClamp: 0.8,
    contactWindow: 0.08,
    contactGrace: 0.5,
    verificationTimeout: 4,
    stableHold: 2,
  });
  assert.deepEqual(ASSEMBLY1_STEP2_GRIPPER_CLAMPS, [130, 130, 135, 130]);
  assert.deepEqual(ASSEMBLY1_STEP2_LIMITS, {
    tcpPosition: 0.06,
    tcpOrientationDegrees: 8,
    preStepObjectDrift: 0.003,
    objectTranslation: 0.005,
    settlingTranslation: {
      assembly_frame: 0.008,
      double_face_hammer: 0.05,
      cross_member: 0.03,
    },
    objectRotationDegrees: 8,
    hammerRotationDegrees: 10,
    verticalDisplacement: 0.003,
    hammerVerticalDisplacement: 0.005,
    crossMemberVerticalDisplacement: 0.015,
    minimumAperture: 0.035,
    hammerMinimumAperture: 0.03,
    crossMemberMinimumAperture: 0.035,
    maximumContactPenetration: 0.002,
    hammerMaximumContactPenetration: 0.0025,
    contactComparisonEpsilon: 0.00015,
  });
});

test('Step 2 derives all Panda resource names and actuator blocks', () => {
  for (const [index, arm] of ASSEMBLY1_STEP2_ARMS.entries()) {
    assert.equal(arm.siteName, `r${index}_tcp`);
    assert.deepEqual(
      arm.jointNames,
      Array.from({ length: 7 }, (_, joint) => `r${index}_joint${joint + 1}`),
    );
    assert.deepEqual(
      arm.fingerJointNames,
      [`r${index}_finger_joint1`, `r${index}_finger_joint2`],
    );
    assert.deepEqual(
      arm.actuatorIndices,
      Array.from({ length: 7 }, (_, joint) => index * 8 + joint),
    );
    assert.equal(arm.gripperActuatorIndex, index * 8 + 7);
    assert.equal(arm.tcpQuaternion.length, 4);
  }
});

test('every Step 2 waypoint contains a complete generated Panda solution', () => {
  for (const arm of ASSEMBLY1_STEP2_ARMS) {
    assert.equal(arm.approachJointTargets.length, 7);
    assert.equal(arm.contactJointTargets.length, 7);
    assert.ok(arm.approachJointTargets.every(Number.isFinite));
    assert.ok(arm.contactJointTargets.every(Number.isFinite));
  }
});

test('Arm 1 keeps the Step 1 IK branch when Step 2 begins', () => {
  const step1Final = ASSEMBLY1_STEP1_ARMS[0].finalJointTargets;
  const step2Approach = ASSEMBLY1_STEP2_ARMS[0].approachJointTargets;
  const maximumDirectSweep = Math.max(
    ...step1Final.map((value, joint) => Math.abs(step2Approach[joint] - value)),
  );
  assert.equal(ASSEMBLY1_STEP2_ARMS[0].closingAxisYawDegrees, -90);
  assert.ok(maximumDirectSweep < 0.8, `Arm 1 direct sweep is ${maximumDirectSweep} rad`);
});

test('Step 2 gripper interpolation uses clamped smoothstep', () => {
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, -1), 255);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 0), 255);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 0.5), 127.5);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 1), 0);
  assert.equal(interpolateAssemblyStep2Gripper(255, 0, 2), 0);
});

test('Step 2 failure holds each arm at its measured pose and releases every gripper', () => {
  const controls = new Float64Array(16).fill(-9);
  const positions = new Float64Array([0.1, 0.2, 0.3, 0, 0, 0, 1.1, 1.2, 1.3]);
  releaseAssemblyStep2Controls(controls, positions, [
    {
      actuatorIndices: [0, 1, 2],
      qposAddresses: [0, 1, 2],
      gripperActuatorIndex: 3,
    },
    {
      actuatorIndices: [8, 9, 10],
      qposAddresses: [6, 7, 8],
      gripperActuatorIndex: 11,
    },
  ]);
  assert.deepEqual(Array.from(controls.slice(0, 4)), [0.1, 0.2, 0.3, 255]);
  assert.deepEqual(Array.from(controls.slice(8, 12)), [1.1, 1.2, 1.3, 255]);
});

test('frame verification captures an immutable measured joint target per arm', () => {
  const positions = new Float64Array([0.1, 0.2, 0.3, 9, 1.1, 1.2, 1.3]);
  const result = captureAssemblyStep2JointTargets(positions, [
    { qposAddresses: [0, 1, 2] },
    { qposAddresses: [4, 5, 6] },
  ]);
  positions[0] = 8;
  assert.deepEqual(result, [[0.1, 0.2, 0.3], [1.1, 1.2, 1.3]]);
});

test('Step 2 control frames descend all arms and close cross-member grippers together', () => {
  const plans = ASSEMBLY1_STEP2_ARMS.map((arm, index) => ({
    armKey: arm.key,
    start: Array(7).fill(index),
    approach: Array(7).fill(index + 1),
    contact: Array(7).fill(index + 2),
  }));
  const machine = (phase, phaseElapsed) => ({
    phase,
    phaseElapsed,
    continuousValidSeconds: 0,
    failure: null,
  });

  const approach = createAssemblyStep2ControlFrame(machine('approach', 0.7), plans);
  assert.deepEqual(approach.arms[0].jointTargets, Array(7).fill(0.5));
  assert.deepEqual(approach.arms.map((arm) => arm.gripperTarget), [255, 255, 255, 255]);

  const descent = createAssemblyStep2ControlFrame(machine('slow-descent', 0.4), plans);
  assert.deepEqual(descent.arms[2].jointTargets, Array(7).fill(3.5));
  assert.deepEqual(descent.arms.map((arm) => arm.gripperTarget), [255, 255, 255, 255]);

  const frameClamp = createAssemblyStep2ControlFrame(machine('frame-clamp', 0.4), plans);
  assert.deepEqual(frameClamp.arms.map((arm) => arm.gripperTarget), [192.5, 255, 255, 255]);

  const crossClamp = createAssemblyStep2ControlFrame(
    machine('cross-member-clamp', 0.5),
    plans,
  );
  assert.deepEqual(crossClamp.arms.map((arm) => arm.gripperTarget), [130, 255, 195, 195]);

  const toolClamp = createAssemblyStep2ControlFrame(
    machine('hammer-clamp', 0.4),
    plans,
  );
  assert.deepEqual(toolClamp.arms.map((arm) => arm.gripperTarget), [130, 192.5, 135, 130]);
});

test('Step 2 phase machine follows timed motions and starts verification windows', () => {
  let state = createAssemblyStep2Machine();
  assert.equal(state.phase, 'approach');
  state = advanceAssemblyStep2Machine(state, 1.4, {});
  assert.equal(state.phase, 'slow-descent');
  state = advanceAssemblyStep2Machine(state, 0.8, {});
  assert.equal(state.phase, 'contact-settle');
  state = advanceAssemblyStep2Machine(state, 1.5, {});
  assert.equal(state.phase, 'frame-clamp');
  state = advanceAssemblyStep2Machine(state, 0.8, {});
  assert.equal(state.phase, 'frame-verification');
});

test('Step 2 verification requires an uninterrupted 0.08 second window', () => {
  let state = {
    phase: 'frame-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  };
  state = advanceAssemblyStep2Machine(state, 0.05, { frame: { ok: true } });
  assert.equal(state.phase, 'frame-verification');
  assert.equal(state.continuousValidSeconds, 0.05);
  state = advanceAssemblyStep2Machine(state, 0.02, {
    frame: { ok: false, code: 'missing-left-contact' },
  });
  assert.equal(state.continuousValidSeconds, 0);
  state = advanceAssemblyStep2Machine(state, 0.08, { frame: { ok: true } });
  assert.equal(state.phase, 'cross-member-clamp');
});

test('Step 2 verification accepts recent real contact but rejects expired contact memory', () => {
  const valid = {
    targetBody: 'double_face_hammer',
    leftContactBodies: [],
    rightContactBodies: ['double_face_hammer'],
    forbiddenBodies: [],
    aperture: 0.04,
    leftTargetContactDistance: -0.001,
    rightTargetContactDistance: -0.001,
    translation: 0.001,
    rotationDegrees: 1,
    verticalDisplacement: 0.001,
  };
  assert.deepEqual(evaluateAssemblyStep2Grasp({
    ...valid,
    leftTargetContactAge: 0.05,
    rightTargetContactAge: 0,
  }), { ok: true });
  assert.deepEqual(evaluateAssemblyStep2Grasp({
    ...valid,
    targetBody: 'double_face_hammer',
    leftContactBodies: ['double_face_hammer'],
    rightContactBodies: ['double_face_hammer'],
    verticalDisplacement: 0.004,
    maximumVerticalDisplacement: ASSEMBLY1_STEP2_LIMITS.hammerVerticalDisplacement,
  }), { ok: true });
  assert.equal(evaluateAssemblyStep2Grasp({
    ...valid,
    leftTargetContactAge: 0.501,
    rightTargetContactAge: 0,
  }).code, 'missing-left-contact');
});

test('Step 2 verification timeout is terminal and preserves the failure reason', () => {
  let state = {
    phase: 'tool-verification',
    phaseElapsed: 3.9,
    continuousValidSeconds: 0,
    failure: null,
  };
  state = advanceAssemblyStep2Machine(state, 0.11, {
    tool: { ok: false, code: 'missing-right-contact', armKey: 'r1' },
  });
  assert.equal(state.phase, 'error');
  assert.deepEqual(state.failure, {
    code: 'missing-right-contact',
    armKey: 'r1',
  });
  assert.deepEqual(advanceAssemblyStep2Machine(state, 10, {}), state);
});

test('Step 2 timeout reports the retained invalid sample when the final sample is valid', () => {
  const state = advanceAssemblyStep2Machine({
    phase: 'tool-verification',
    phaseElapsed: 3.995,
    continuousValidSeconds: 0,
    lastInvalidVerdict: {
      ok: false,
      code: 'missing-left-contact',
      armKey: 'r1',
    },
    failure: null,
  }, 0.01, { tool: { ok: true } });
  assert.equal(state.phase, 'error');
  assert.deepEqual(state.failure, {
    code: 'missing-left-contact',
    armKey: 'r1',
  });
});

test('Step 2 stable hold completes only while every grasp remains valid', () => {
  let state = {
    phase: 'clamped-hold',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  };
  state = advanceAssemblyStep2Machine(state, 1.5, { all: { ok: true } });
  assert.equal(state.phase, 'clamped-hold');
  state = advanceAssemblyStep2Machine(state, 0.1, {
    all: { ok: false, code: 'forbidden-contact', armKey: 'r3' },
  });
  assert.equal(state.phase, 'error');
  assert.equal(state.failure.code, 'forbidden-contact');
});

test('Step 2 grasp verdict accepts only bilateral physical target contact', () => {
  const valid = {
    targetBody: 'assembly_frame',
    leftContactBodies: ['assembly_frame'],
    rightContactBodies: ['assembly_frame'],
    forbiddenBodies: [],
    aperture: 0.04,
    translation: 0.001,
    rotationDegrees: 1,
    verticalDisplacement: 0.001,
  };
  assert.deepEqual(evaluateAssemblyStep2Grasp(valid), { ok: true });
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, aperture: 0.035 }).code,
    'empty-closure',
  );
  assert.deepEqual(evaluateAssemblyStep2Grasp({
    ...valid,
    targetBody: 'cross_member',
    leftContactBodies: ['cross_member'],
    rightContactBodies: ['cross_member'],
    aperture: 0.04,
    minimumAperture: ASSEMBLY1_STEP2_LIMITS.crossMemberMinimumAperture,
  }), { ok: true });
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, leftContactBodies: [] }).code,
    'missing-left-contact',
  );
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, rightContactBodies: [] }).code,
    'missing-right-contact',
  );
  assert.deepEqual(
    evaluateAssemblyStep2Grasp({
      ...valid,
      leftContactBodies: [],
      rightContactBodies: [],
      requireBilateralContact: false,
    }),
    { ok: true },
  );
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, forbiddenBodies: ['work_platform'] }).code,
    'forbidden-contact',
  );
  assert.deepEqual(
    evaluateAssemblyStep2Grasp({ ...valid, leftTargetContactDistance: -0.0021 }),
    { ok: true },
  );
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, leftTargetContactDistance: -0.0022 }).code,
    'deep-penetration',
  );
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, translation: 0.0051 }).code,
    'object-drift',
  );
  assert.deepEqual(
    evaluateAssemblyStep2Grasp({
      ...valid,
      translation: 0.019,
      maximumTranslation: ASSEMBLY1_STEP2_LIMITS.settlingTranslation.cross_member,
    }),
    { ok: true },
  );
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, rotationDegrees: 8.1 }).code,
    'object-rotation',
  );
  assert.deepEqual(evaluateAssemblyStep2Grasp({
    ...valid,
    targetBody: 'double_face_hammer',
    leftContactBodies: ['double_face_hammer'],
    rightContactBodies: ['double_face_hammer'],
    rotationDegrees: 9,
    maximumRotationDegrees: 10,
  }), { ok: true });
  assert.equal(
    evaluateAssemblyStep2Grasp({ ...valid, verticalDisplacement: 0.0031 }).code,
    'object-lift',
  );
  assert.deepEqual(evaluateAssemblyStep2Grasp({
    ...valid,
    targetBody: 'cross_member',
    leftContactBodies: ['cross_member'],
    rightContactBodies: ['cross_member'],
    verticalDisplacement: 0.004,
    maximumVerticalDisplacement: 0.015,
  }), { ok: true });
});

test('quaternion angle is sign symmetric and reported in degrees', () => {
  assert.equal(quaternionAngularDistanceDegrees([1, 0, 0, 0], [-1, 0, 0, 0]), 0);
  assert.ok(Math.abs(
    quaternionAngularDistanceDegrees([1, 0, 0, 0], [Math.SQRT1_2, 0, 0, Math.SQRT1_2])
    - 90,
  ) < 1e-9);
});

test('Step 1 and Step 2 use one ownership token without object-pose shortcuts', async () => {
  const [step1Source, step2Source] = await Promise.all([
    readFile(new URL('../src/AssemblyStep1Controller.tsx', import.meta.url), 'utf8'),
    readFile(new URL('../src/AssemblyStep2Controller.tsx', import.meta.url), 'utf8')
      .catch(() => ''),
  ]);
  assert.match(step1Source, /ownershipRef/);
  assert.match(step1Source, /ownershipRef\.current\s*===\s*'step1'/);
  assert.match(step1Source, /}, \[ownershipRef, resetGeneration\]\);/);
  assert.doesNotMatch(step1Source, /}, \[ownershipRef, requestId, resetGeneration\]\);/);
  assert.match(step2Source, /ownershipRef/);
  assert.match(step2Source, /ownershipRef\.current\s*=\s*'step2'/);
  assert.match(step2Source, /frameVerificationJointTargetsRef/);
  assert.match(
    step2Source,
    /captureAssemblyStep2JointTargets\(\s*data\.qpos,\s*runtime\.arms,?\s*\)/,
  );
  assert.match(step2Source, /frameVerificationJointTargetsRef\.current\?\.\[index\]/);
  assert.match(
    step2Source,
    /}, \[diagnosticsRef, ownershipRef, resetGeneration\]\);/,
  );
  assert.doesNotMatch(
    step2Source,
    /}, \[diagnosticsRef, ownershipRef, requestId, resetGeneration\]\);/,
  );
  assert.doesNotMatch(step2Source, /data\.qpos\s*\[[^\]]+\]\s*=/);
  assert.doesNotMatch(step2Source, /data\.qvel\s*\[[^\]]+\]\s*=/);
  assert.doesNotMatch(step2Source, /mjEQ_WELD|equality|magnet|proximity/i);
  assert.doesNotMatch(step2Source, /taskObject.*follow|scripted.*pose/i);
});
