import {
  interpolateJointTargets,
  topDownTcpQuaternion,
} from './assemblyStep1.js';

export const ASSEMBLY1_STEP2_DURATIONS = Object.freeze({
  approach: 1.4,
  slowDescent: 0.8,
  contactSettle: 1.5,
  frameClamp: 0.8,
  crossMemberClamp: 1,
  torqueDriverClamp: 0.8,
  contactWindow: 0.25,
  verificationTimeout: 2.5,
  stableHold: 2,
});

export const ASSEMBLY1_STEP2_GRIPPER_CLAMPS = Object.freeze([48, 96, 0, 0]);

export const ASSEMBLY1_STEP2_LIMITS = Object.freeze({
  tcpPosition: 0.06,
  tcpOrientationDegrees: 8,
  preStepObjectDrift: 0.003,
  objectTranslation: 0.005,
  objectRotationDegrees: 5,
  verticalDisplacement: 0.003,
  minimumAperture: 0.02,
});

const roles = [
  {
    role: 'south frame rail',
    targetBody: 'assembly_frame',
    contactWaypoint: [0, -0.23, 0.235],
    closingAxisYawDegrees: 90,
    approachJointTargets: [-2.422463, -0.967332, -2.055864, -1.664503, -0.942029, 2.02085, -0.275333],
    contactJointTargets: [-2.412541, -0.986406, -2.075424, -1.665048, -0.959337, 2.041001, -0.268219],
  },
  {
    role: 'side-laid torque driver handle',
    targetBody: 'torque_driver',
    contactWaypoint: [0.559, -0.421, 0.16],
    closingAxisYawDegrees: 162,
    approachJointTargets: [2.291276, 0.432042, 0.156637, -2.241718, -0.139372, 2.664861, 0.515273],
    contactJointTargets: [2.298379, 0.468452, 0.147356, -2.23421, -0.150268, 2.693066, 0.523317],
  },
  {
    role: 'cross member north balance point',
    targetBody: 'cross_member',
    contactWaypoint: [-0.49, 0.56, 0.14],
    closingAxisYawDegrees: 0,
    approachJointTargets: [2.65007, -1.276525, -1.901844, -1.922017, -1.389913, 1.973555, -2.583203],
    contactJointTargets: [2.662583, -1.294408, -1.92694, -1.910965, -1.410603, 1.990045, -2.579454],
  },
  {
    role: 'cross member south balance point',
    targetBody: 'cross_member',
    contactWaypoint: [-0.49, 0.32, 0.14],
    closingAxisYawDegrees: 0,
    approachJointTargets: [2.605475, 0.464764, -0.32948, -2.29885, 0.354635, 2.717314, -0.372398],
    contactJointTargets: [2.585778, 0.503303, -0.304679, -2.287456, 0.379514, 2.74454, -0.391434],
  },
];

export const ASSEMBLY1_STEP2_ARMS = roles.map((role, index) => {
  const prefix = `r${index}_`;
  const actuatorOffset = index * 8;
  return Object.freeze({
    key: `r${index}`,
    label: `Arm ${index + 1}`,
    ...role,
    contactWaypoint: Object.freeze([...role.contactWaypoint]),
    approachWaypoint: Object.freeze([
      role.contactWaypoint[0],
      role.contactWaypoint[1],
      Number((role.contactWaypoint[2] + 0.015).toFixed(6)),
    ]),
    tcpQuaternion: Object.freeze(topDownTcpQuaternion(role.closingAxisYawDegrees)),
    approachJointTargets: Object.freeze([...role.approachJointTargets]),
    contactJointTargets: Object.freeze([...role.contactJointTargets]),
    siteName: `${prefix}tcp`,
    jointNames: Object.freeze(
      Array.from({ length: 7 }, (_, joint) => `${prefix}joint${joint + 1}`),
    ),
    fingerJointNames: Object.freeze([
      `${prefix}finger_joint1`,
      `${prefix}finger_joint2`,
    ]),
    leftFingerBody: `${prefix}left_finger`,
    rightFingerBody: `${prefix}right_finger`,
    actuatorIndices: Object.freeze(
      Array.from({ length: 7 }, (_, joint) => actuatorOffset + joint),
    ),
    gripperActuatorIndex: actuatorOffset + 7,
  });
});

export function interpolateAssemblyStep2Gripper(from, to, progress) {
  return interpolateJointTargets([from], [to], progress)[0];
}

export function releaseAssemblyStep2Controls(controls, positions, arms) {
  for (const arm of arms) {
    for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
      controls[arm.actuatorIndices[joint]] = positions[arm.qposAddresses[joint]];
    }
    controls[arm.gripperActuatorIndex] = 255;
  }
}

export function captureAssemblyStep2JointTargets(positions, arms) {
  return arms.map((arm) => arm.qposAddresses.map((address) => positions[address]));
}

export function createAssemblyStep2ControlFrame(machine, plans) {
  const progressFor = (duration) => machine.phaseElapsed / duration;
  const clamps = ASSEMBLY1_STEP2_GRIPPER_CLAMPS;
  const gripperTargets = [255, 255, 255, 255];
  if (machine.phase === 'frame-clamp') {
    gripperTargets[0] = interpolateAssemblyStep2Gripper(
      255,
      clamps[0],
      progressFor(ASSEMBLY1_STEP2_DURATIONS.frameClamp),
    );
  } else if (machine.phase === 'frame-verification') {
    gripperTargets[0] = clamps[0];
  } else if (machine.phase === 'cross-member-clamp') {
    gripperTargets[0] = clamps[0];
    const sharedTarget = interpolateAssemblyStep2Gripper(
      255,
      clamps[2],
      progressFor(ASSEMBLY1_STEP2_DURATIONS.crossMemberClamp),
    );
    gripperTargets[2] = sharedTarget;
    gripperTargets[3] = sharedTarget;
  } else if (machine.phase === 'cross-member-verification') {
    gripperTargets[0] = clamps[0];
    gripperTargets[2] = clamps[2];
    gripperTargets[3] = clamps[3];
  } else if (machine.phase === 'torque-driver-clamp') {
    gripperTargets[0] = clamps[0];
    gripperTargets[1] = interpolateAssemblyStep2Gripper(
      255,
      clamps[1],
      progressFor(ASSEMBLY1_STEP2_DURATIONS.torqueDriverClamp),
    );
    gripperTargets[2] = clamps[2];
    gripperTargets[3] = clamps[3];
  } else if (
    machine.phase === 'tool-verification'
    || machine.phase === 'clamped-hold'
    || machine.phase === 'complete'
  ) {
    for (let index = 0; index < gripperTargets.length; index += 1) {
      gripperTargets[index] = clamps[index];
    }
  }

  return {
    arms: plans.map((plan, index) => {
      let jointTargets = plan.contact;
      if (machine.phase === 'approach') {
        jointTargets = interpolateJointTargets(
          plan.start,
          plan.approach,
          progressFor(ASSEMBLY1_STEP2_DURATIONS.approach),
        );
      } else if (machine.phase === 'slow-descent') {
        jointTargets = interpolateJointTargets(
          plan.approach,
          plan.contact,
          progressFor(ASSEMBLY1_STEP2_DURATIONS.slowDescent),
        );
      }
      return {
        armKey: plan.armKey,
        jointTargets,
        gripperTarget: gripperTargets[index],
      };
    }),
  };
}

export function quaternionAngularDistanceDegrees(first, second) {
  const firstNorm = Math.hypot(...first);
  const secondNorm = Math.hypot(...second);
  if (firstNorm === 0 || secondNorm === 0) return Number.POSITIVE_INFINITY;
  const rawDot = first.reduce(
    (sum, value, index) => sum + value * second[index],
    0,
  ) / (firstNorm * secondNorm);
  const symmetricDot = Math.max(-1, Math.min(1, Math.abs(rawDot)));
  return 2 * Math.acos(symmetricDot) * 180 / Math.PI;
}

function failed(code, detail) {
  return detail === undefined ? { ok: false, code } : { ok: false, code, detail };
}

export function evaluateAssemblyStep2Grasp({
  targetBody,
  leftContactBodies,
  rightContactBodies,
  forbiddenBodies,
  aperture,
  translation,
  rotationDegrees,
  verticalDisplacement,
}) {
  if (!leftContactBodies.includes(targetBody)) return failed('missing-left-contact');
  if (!rightContactBodies.includes(targetBody)) return failed('missing-right-contact');
  if (forbiddenBodies.length > 0) return failed('forbidden-contact', forbiddenBodies.join(', '));
  if (!(aperture > ASSEMBLY1_STEP2_LIMITS.minimumAperture)) {
    return failed('empty-closure', String(aperture));
  }
  if (translation > ASSEMBLY1_STEP2_LIMITS.objectTranslation) {
    return failed('object-drift', String(translation));
  }
  if (rotationDegrees > ASSEMBLY1_STEP2_LIMITS.objectRotationDegrees) {
    return failed('object-rotation', String(rotationDegrees));
  }
  if (verticalDisplacement > ASSEMBLY1_STEP2_LIMITS.verticalDisplacement) {
    return failed('object-lift', String(verticalDisplacement));
  }
  return { ok: true };
}

export function createAssemblyStep2Machine() {
  return {
    phase: 'approach',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  };
}

const timedTransitions = {
  approach: [ASSEMBLY1_STEP2_DURATIONS.approach, 'slow-descent'],
  'slow-descent': [ASSEMBLY1_STEP2_DURATIONS.slowDescent, 'contact-settle'],
  'contact-settle': [ASSEMBLY1_STEP2_DURATIONS.contactSettle, 'frame-clamp'],
  'frame-clamp': [ASSEMBLY1_STEP2_DURATIONS.frameClamp, 'frame-verification'],
  'cross-member-clamp': [
    ASSEMBLY1_STEP2_DURATIONS.crossMemberClamp,
    'cross-member-verification',
  ],
  'torque-driver-clamp': [
    ASSEMBLY1_STEP2_DURATIONS.torqueDriverClamp,
    'tool-verification',
  ],
};

const verificationTransitions = {
  'frame-verification': ['frame', 'cross-member-clamp'],
  'cross-member-verification': ['crossMember', 'torque-driver-clamp'],
  'tool-verification': ['tool', 'clamped-hold'],
};

function enterPhase(phase) {
  return {
    phase,
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  };
}

function terminalFailure(verdict) {
  return {
    phase: 'error',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: {
      code: verdict?.code ?? 'verification-timeout',
      ...(verdict?.armKey ? { armKey: verdict.armKey } : {}),
      ...(verdict?.detail ? { detail: verdict.detail } : {}),
    },
  };
}

export function advanceAssemblyStep2Machine(machine, deltaSeconds, evidence) {
  if (machine.phase === 'complete' || machine.phase === 'error') return machine;
  const dt = Math.max(0, deltaSeconds);
  const timedTransition = timedTransitions[machine.phase];
  if (timedTransition) {
    const [duration, nextPhase] = timedTransition;
    const phaseElapsed = machine.phaseElapsed + dt;
    return phaseElapsed >= duration
      ? enterPhase(nextPhase)
      : { ...machine, phaseElapsed };
  }

  const verificationTransition = verificationTransitions[machine.phase];
  if (verificationTransition) {
    const [evidenceKey, nextPhase] = verificationTransition;
    const verdict = evidence[evidenceKey];
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok
      ? machine.continuousValidSeconds + dt
      : 0;
    if (continuousValidSeconds >= ASSEMBLY1_STEP2_DURATIONS.contactWindow) {
      return enterPhase(nextPhase);
    }
    if (phaseElapsed >= ASSEMBLY1_STEP2_DURATIONS.verificationTimeout) {
      return terminalFailure(verdict);
    }
    return {
      ...machine,
      phaseElapsed,
      continuousValidSeconds,
    };
  }

  if (machine.phase === 'clamped-hold') {
    const verdict = evidence.all;
    if (!verdict?.ok) return terminalFailure(verdict);
    const continuousValidSeconds = machine.continuousValidSeconds + dt;
    if (continuousValidSeconds >= ASSEMBLY1_STEP2_DURATIONS.stableHold) {
      return enterPhase('complete');
    }
    return {
      ...machine,
      phaseElapsed: machine.phaseElapsed + dt,
      continuousValidSeconds,
    };
  }

  return machine;
}
