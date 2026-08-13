export const ASSEMBLY1_STEP1_PHASE_DURATION = 1.5;
export const ASSEMBLY1_GRIPPER_OPEN = 255;

const roles = [
  {
    role: 'south frame grip',
    highWaypoint: [0, -0.36, 0.50],
    finalWaypoint: [0, -0.31, 0.34],
    highJointTargets: [1.820193, -0.069366, -0.242346, -1.970456, -0.01791, 1.907045, 2.366129],
    finalJointTargets: [1.80813, 0.228846, -0.247302, -2.025619, 0.070489, 2.250253, 2.304969],
  },
  {
    role: 'torque driver',
    highWaypoint: [0.53, -0.42, 0.50],
    finalWaypoint: [0.53, -0.42, 0.36],
    highJointTargets: [0.788514, -0.345604, 1.476793, -1.887358, 0.355544, 1.840629, 2.8973],
    finalJointTargets: [0.538023, -0.41985, 1.801701, -2.100681, 0.489855, 2.147852, 2.86118],
  },
  {
    role: 'cross member north grip',
    highWaypoint: [-0.49, 0.65, 0.50],
    finalWaypoint: [-0.49, 0.65, 0.34],
    highJointTargets: [2.405602, -1.285844, -1.140805, -1.831768, -1.164872, 1.255829, 2.006757],
    finalJointTargets: [2.326314, -1.45842, -1.371126, -2.058079, -1.374876, 1.45049, 2.047761],
  },
  {
    role: 'west frame grip',
    highWaypoint: [-0.48, 0, 0.50],
    finalWaypoint: [-0.46, 0, 0.34],
    highJointTargets: [1.771147, -0.443691, -0.173607, -2.339956, -0.078928, 1.905001, 2.421572],
    finalJointTargets: [1.781726, -0.196218, -0.212141, -2.544417, -0.058976, 2.354537, 2.397102],
  },
];

export const ASSEMBLY1_STEP1_ARMS = roles.map((role, index) => {
  const prefix = `r${index}_`;
  const actuatorOffset = index * 8;
  return {
    key: `r${index}`,
    label: `Arm ${index + 1}`,
    ...role,
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

export function selectAssemblyStep1Phase(elapsed) {
  if (elapsed >= ASSEMBLY1_STEP1_PHASE_DURATION * 2) {
    return { phase: 'complete', progress: 1 };
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
