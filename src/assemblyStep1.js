export const ASSEMBLY1_STEP1_PHASE_DURATION = 1.5;
export const ASSEMBLY1_STEP1_SETTLE_DURATION = 3;
export const ASSEMBLY1_GRIPPER_OPEN = 255;
export const ASSEMBLY1_STEP1_IK_VERSION = 'grasp-ready-v2';

export function topDownTcpQuaternion(closingAxisYawDegrees) {
  const halfTurn = (closingAxisYawDegrees + 90) * Math.PI / 360;
  return [Math.cos(halfTurn), Math.sin(halfTurn), 0, 0]
    .map((value) => (Math.abs(value) < 1e-12 ? 0 : value));
}

const roles = [
  {
    role: 'south frame rail',
    highWaypoint: [0, -0.23, 0.50],
    finalWaypoint: [0, -0.23, 0.33],
    closingAxisYawDegrees: 90,
    highJointTargets: [-2.436612, -0.739274, -1.834062, -1.36531, -0.708778, 1.593286, -0.412203],
    finalJointTargets: [-2.462187, -0.877127, -1.957716, -1.627588, -0.85221, 1.901325, -0.316657],
  },
  {
    role: 'side-laid torque driver handle',
    highWaypoint: [0.559, -0.421, 0.48],
    finalWaypoint: [0.559, -0.421, 0.28],
    closingAxisYawDegrees: 162,
    highJointTargets: [2.254901, -0.053876, 0.201647, -2.005079, 0.012022, 1.954481, 0.409568],
    finalJointTargets: [2.262258, 0.200462, 0.19843, -2.241938, -0.058834, 2.43746, 0.459226],
  },
  {
    role: 'cross member north balance point',
    highWaypoint: [-0.49, 0.56, 0.48],
    finalWaypoint: [-0.49, 0.56, 0.26],
    closingAxisYawDegrees: 0,
    highJointTargets: [2.556337, -0.990657, -1.399648, -1.722293, -0.9717, 1.513838, -2.720657],
    finalJointTargets: [2.576151, -1.172598, -1.716567, -1.948272, -1.241199, 1.841582, -2.619852],
  },
  {
    role: 'cross member south balance point',
    highWaypoint: [-0.49, 0.32, 0.48],
    finalWaypoint: [-0.49, 0.32, 0.26],
    closingAxisYawDegrees: 0,
    highJointTargets: [2.717062, -0.131099, -0.462315, -2.077166, -0.06344, 1.96, -0.073787],
    finalJointTargets: [2.700314, 0.204706, -0.45956, -2.326086, 0.148119, 2.505253, -0.225722],
  },
];

export const ASSEMBLY1_STEP1_ARMS = roles.map((role, index) => {
  const prefix = `r${index}_`;
  const actuatorOffset = index * 8;
  return {
    key: `r${index}`,
    label: `Arm ${index + 1}`,
    ...role,
    tcpQuaternion: topDownTcpQuaternion(role.closingAxisYawDegrees),
    siteName: `${prefix}tcp`,
    jointNames: Array.from({ length: 7 }, (_, joint) => `${prefix}joint${joint + 1}`),
    actuatorIndices: Array.from({ length: 7 }, (_, joint) => actuatorOffset + joint),
    gripperActuatorIndex: actuatorOffset + 7,
  };
});

export function smoothstep01(value) {
  const clamped = Math.max(0, Math.min(1, value));
  return clamped * clamped * (3 - 2 * clamped);
}

export function interpolateJointTargets(from, to, progress) {
  const eased = smoothstep01(progress);
  return from.map((value, index) => value + (to[index] - value) * eased);
}

export function holdAssemblyJointState(controls, positions, velocities, arms) {
  for (const arm of arms) {
    for (let joint = 0; joint < arm.positions.length; joint += 1) {
      controls[arm.actuatorIndices[joint]] = arm.positions[joint];
      positions[arm.qposAddresses[joint]] = arm.positions[joint];
      velocities[arm.dofAddresses[joint]] = 0;
    }
  }
}

export function selectAssemblyStep1Phase(elapsed) {
  const motionDuration = ASSEMBLY1_STEP1_PHASE_DURATION * 2;
  if (elapsed >= motionDuration + ASSEMBLY1_STEP1_SETTLE_DURATION) {
    return { phase: 'complete', progress: 1 };
  }
  if (elapsed >= motionDuration) {
    return {
      phase: 'settling',
      progress: (elapsed - motionDuration) / ASSEMBLY1_STEP1_SETTLE_DURATION,
    };
  }
  if (elapsed >= ASSEMBLY1_STEP1_PHASE_DURATION) {
    return {
      phase: 'final',
      progress: (elapsed - ASSEMBLY1_STEP1_PHASE_DURATION) / ASSEMBLY1_STEP1_PHASE_DURATION,
    };
  }
  return {
    phase: 'high',
    progress: Math.max(0, elapsed) / ASSEMBLY1_STEP1_PHASE_DURATION,
  };
}

export function isCompleteAssemblyStep1Plan(plans) {
  const completeJoints = (values) => (
    Array.isArray(values)
    && values.length === 7
    && values.every(Number.isFinite)
  );
  return (
    Array.isArray(plans)
    && plans.length === ASSEMBLY1_STEP1_ARMS.length
    && plans.every((plan) => (
      plan
      && completeJoints(plan.start)
      && completeJoints(plan.high)
      && completeJoints(plan.final)
    ))
  );
}
