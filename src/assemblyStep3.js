import { interpolateJointTargets } from './assemblyStep1.js';
import { FRANKA_HOME } from './sceneLayouts.js';

export const ASSEMBLY1_STEP3_DURATIONS = Object.freeze({
  graspCheckWindow: 0.25,
  verificationTimeout: 4,
  lift: 3,
  liftContactGrace: 1.2,
  transferA: 6.5,
  transferB: 6.5,
  alignedDescent: 3,
  alignedHold: 1,
  reseatLift: 1.2,
  reseatDescent: 1.5,
  release: 0.8,
  releaseSettle: 0.5,
  retreat: 3,
  placedHold: 1,
});

export const ASSEMBLY1_STEP3_GRIPPER_CLAMPS = Object.freeze([130, 130, 135, 130]);
export const ASSEMBLY1_STEP3_START_GRIPPER_CLAMPS = Object.freeze([130, 130, 135, 130]);
export const ASSEMBLY1_STEP3_HOME_JOINT_TARGETS = Object.freeze(FRANKA_HOME.slice(0, 7));

export const ASSEMBLY1_STEP3_HAMMER_WAYPOINTS = Object.freeze({
  start: Object.freeze([0.675, -0.421, 0.197]),
  prelift: Object.freeze([0.675, -0.421, 0.207]),
  lift: Object.freeze([0.675, -0.421, 0.257]),
  liftPath: Object.freeze([0.207, 0.217, 0.227, 0.237, 0.247, 0.257].map(
    (z) => Object.freeze([0.675, -0.421, z]),
  )),
  // Step 3 only clears the pickup cradle and holds a compact staging pose.
  // The actual donor-to-receiver transfer belongs to Step 4.
  handover: Object.freeze([0.675, -0.421, 0.257]),
  handoverPath: Object.freeze([Object.freeze([0.675, -0.421, 0.257])]),
});

export const ASSEMBLY1_STEP3_HAMMER_ARM = Object.freeze({
  key: 'r1',
  armIndex: 1,
  closingAxisYawDegrees: 90,
  preliftJointTargets: Object.freeze([
    1.663093, 0.382413, 0.916994, -2.442793, -0.613259, 2.598899, 2.300676,
  ]),
  liftJointTargets: Object.freeze([
    1.47825, 0.235116, 1.141209, -2.458871, -0.37648, 2.524016, 2.135577,
  ]),
  handoverJointTargets: Object.freeze([
    1.47825, 0.235116, 1.141209, -2.458871, -0.37648, 2.524016, 2.135577,
  ]),
});

export const ASSEMBLY1_STEP3_LIMITS = Object.freeze({
  minimumAperture: 0.035,
  hammerMinimumAperture: 0.03,
  crossMemberMinimumAperture: 0.035,
  maximumContactPenetration: 0.002,
  frameMaximumContactPenetration: 0.0025,
  contactComparisonEpsilon: 0.00015,
  frameTranslation: 0.02,
  holePlanarDistance: 0.04,
  holeVerticalOffset: 0.025,
  seatedVerticalOffset: 0.02,
  comparisonEpsilon: 0.001,
});

export const ASSEMBLY1_STEP3_WAYPOINTS = Object.freeze({
  start: Object.freeze([
    Object.freeze([-0.489, 0.56, 0.21]),
    Object.freeze([-0.482, 0.32, 0.213]),
  ]),
  lift: Object.freeze([
    Object.freeze([-0.49, 0.5675, 0.38]),
    Object.freeze([-0.49, 0.3125, 0.38]),
  ]),
  transferA: Object.freeze([
    Object.freeze([-0.245, 0.3475, 0.38]),
    Object.freeze([-0.245, 0.0925, 0.38]),
  ]),
  transferMid: Object.freeze([
    Object.freeze([-0.1225, 0.2375, 0.37]),
    Object.freeze([-0.1225, -0.0175, 0.37]),
  ]),
  hover: Object.freeze([
    Object.freeze([0, 0.1275, 0.34]),
    Object.freeze([0, -0.1275, 0.34]),
  ]),
  descentMid: Object.freeze([
    Object.freeze([0, 0.1275, 0.315]),
    Object.freeze([0, -0.1275, 0.315]),
  ]),
  aligned: Object.freeze([
    Object.freeze([0.006, 0.1275, 0.292]),
    Object.freeze([0.002, -0.1275, 0.292]),
  ]),
});

export const ASSEMBLY1_STEP3_TRANSPORT_ARMS = Object.freeze([
  Object.freeze({
    key: 'r2',
    armIndex: 2,
    closingAxisYawDegrees: 0,
    liftJointTargets: Object.freeze([
      1.512016, -0.682083, -0.537287, -2.76636, -0.388813, 2.122724, -2.636736,
    ]),
    transferAJointTargets: Object.freeze([
      1.92702, -0.068878, -0.244566, -2.288658, -0.021863, 2.223498, -2.230458,
    ]),
    transferMidJointTargets: Object.freeze([
      2.007347, 0.327231, -0.171128, -1.79554, 0.06406, 2.118359, -2.115164,
    ]),
    hoverJointTargets: Object.freeze([
      2.025487, 0.919618, -0.118889, -0.898438, 0.097376, 1.814967, -1.997266,
    ]),
    descentMidJointTargets: Object.freeze([
      2.026892, 0.95688, -0.116543, -0.946158, 0.100488, 1.896209, -1.999844,
    ]),
    alignedJointTargets: Object.freeze([
      2.032727, 0.979879, -0.114387, -0.948012, 0.10142, 1.921416, -1.993605,
    ]),
  }),
  Object.freeze({
    key: 'r3',
    armIndex: 3,
    closingAxisYawDegrees: 0,
    liftJointTargets: Object.freeze([
      2.812157, -0.28508, -0.443897, -2.493134, -0.15349, 2.229172, 0.122219,
    ]),
    transferAJointTargets: Object.freeze([
      2.655534, 0.159851, -0.946985, -2.084786, 0.156593, 2.174215, -0.730454,
    ]),
    transferMidJointTargets: Object.freeze([
      2.237724, 0.623438, -0.875703, -1.586116, 0.507729, 1.968393, -1.102189,
    ]),
    hoverJointTargets: Object.freeze([
      1.448019, 1.162494, -0.127668, -0.466453, 0.11769, 1.644099, -0.96558,
    ]),
    descentMidJointTargets: Object.freeze([
      1.425614, 1.221362, -0.046695, -0.466925, 0.044249, 1.697324, -0.951748,
    ]),
    alignedJointTargets: Object.freeze([
      1.41738, 1.244817, -0.015408, -0.466996, 0.01477, 1.722095, -0.945825,
    ]),
  }),
]);

function failure(code, detail) {
  return detail === undefined ? { ok: false, code } : { ok: false, code, detail };
}

export function evaluateAssemblyStep3Transport({
  targetBody,
  leftContactBodies,
  rightContactBodies,
  forbiddenBodies,
  aperture,
  leftTargetContactDistance = null,
  rightTargetContactDistance = null,
  requireBilateralContact = true,
  minimumAperture = ASSEMBLY1_STEP3_LIMITS.minimumAperture,
  maximumContactPenetration = ASSEMBLY1_STEP3_LIMITS.maximumContactPenetration,
  contactComparisonEpsilon = ASSEMBLY1_STEP3_LIMITS.contactComparisonEpsilon,
}) {
  if (!Number.isFinite(aperture)) return failure('non-finite-runtime');
  if (requireBilateralContact && !leftContactBodies.includes(targetBody)) {
    return failure('missing-left-contact');
  }
  if (requireBilateralContact && !rightContactBodies.includes(targetBody)) {
    return failure('missing-right-contact');
  }
  if (forbiddenBodies.length > 0) return failure('forbidden-contact', forbiddenBodies.join(', '));
  const contactDistances = [leftTargetContactDistance, rightTargetContactDistance]
    .filter((value) => typeof value === 'number' && Number.isFinite(value));
  if (
    contactDistances.length > 0
    && Math.min(...contactDistances)
      < -(maximumContactPenetration + contactComparisonEpsilon)
  ) {
    return failure('deep-penetration', String(Math.min(...contactDistances)));
  }
  if (!(aperture > minimumAperture)) {
    return failure('empty-closure', String(aperture));
  }
  return { ok: true };
}

export function evaluateAssemblyStep3Alignment({
  holePlanarDistances,
  holeVerticalOffsets,
  frameTranslation,
  crossMemberRotationDegrees,
  planarTolerance = ASSEMBLY1_STEP3_LIMITS.holePlanarDistance,
  verticalTolerance = ASSEMBLY1_STEP3_LIMITS.holeVerticalOffset,
}) {
  if (
    holePlanarDistances.length !== 4
    || holeVerticalOffsets.length !== 4
    || !holePlanarDistances.every(Number.isFinite)
    || !holeVerticalOffsets.every(Number.isFinite)
    || !Number.isFinite(frameTranslation)
    || !Number.isFinite(crossMemberRotationDegrees)
  ) {
    return failure('non-finite-runtime');
  }
  const maximumPlanarDistance = Math.max(...holePlanarDistances);
  if (maximumPlanarDistance > planarTolerance + ASSEMBLY1_STEP3_LIMITS.comparisonEpsilon) {
    return failure('hole-misalignment', String(maximumPlanarDistance));
  }
  const maximumVerticalOffset = Math.max(...holeVerticalOffsets);
  if (maximumVerticalOffset > verticalTolerance + ASSEMBLY1_STEP3_LIMITS.comparisonEpsilon) {
    return failure('hole-height', String(maximumVerticalOffset));
  }
  if (frameTranslation > ASSEMBLY1_STEP3_LIMITS.frameTranslation) {
    return failure('frame-drift', String(frameTranslation));
  }
  return { ok: true };
}

export function createAssemblyStep3Machine() {
  return {
    phase: 'grasp-check',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    reseatAttempts: 0,
    failure: null,
  };
}

function enterPhase(phase, reseatAttempts = 0) {
  return {
    phase,
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    reseatAttempts,
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

function combinedEvidence(evidence, includeAlignment = false) {
  if (!evidence.all?.ok) return evidence.all;
  if (includeAlignment && !evidence.alignment?.ok) return evidence.alignment;
  return { ok: true };
}

const motionTransitions = {
  lift: [ASSEMBLY1_STEP3_DURATIONS.lift, 'lift-settle', false],
  'transfer-a': [ASSEMBLY1_STEP3_DURATIONS.transferA, 'transfer-b', false],
  'transfer-b': [ASSEMBLY1_STEP3_DURATIONS.transferB, 'hover-settle', false],
  'aligned-descent': [
    ASSEMBLY1_STEP3_DURATIONS.alignedDescent,
    'alignment-verification',
    false,
  ],
  'reseat-lift': [ASSEMBLY1_STEP3_DURATIONS.reseatLift, 'reseat-descent', false],
  'reseat-descent': [
    ASSEMBLY1_STEP3_DURATIONS.reseatDescent,
    'alignment-verification',
    false,
  ],
  release: [ASSEMBLY1_STEP3_DURATIONS.release, 'release-settle', false],
  'release-settle': [ASSEMBLY1_STEP3_DURATIONS.releaseSettle, 'retreat', false],
  retreat: [ASSEMBLY1_STEP3_DURATIONS.retreat, 'placed-verification', false],
};

const verificationTransitions = {
  'grasp-check': ['lift', false],
  'lift-settle': ['transfer-a', false],
  'hover-settle': ['aligned-descent', false],
  'alignment-verification': ['aligned-hold', true],
};

export function advanceAssemblyStep3Machine(machine, deltaSeconds, evidence) {
  if (machine.phase === 'error') return machine;
  if (machine.phase === 'complete') {
    return evidence.all?.ok ? machine : terminalFailure(evidence.all);
  }
  const dt = Math.max(0, deltaSeconds);
  const motionTransition = motionTransitions[machine.phase];
  if (motionTransition) {
    const [duration, nextPhase, includeAlignment] = motionTransition;
    const releasingCrossMember = ['release', 'release-settle', 'retreat'].includes(machine.phase);
    const verdict = releasingCrossMember
      ? { ok: true }
      : combinedEvidence(evidence, includeAlignment);
    const liftIsRecovering = machine.phase === 'lift'
      && machine.phaseElapsed < ASSEMBLY1_STEP3_DURATIONS.liftContactGrace;
    if (!verdict?.ok && !liftIsRecovering) return terminalFailure(verdict);
    const phaseElapsed = machine.phaseElapsed + dt;
    return phaseElapsed >= duration
      ? enterPhase(nextPhase, machine.reseatAttempts ?? 0)
      : { ...machine, phaseElapsed };
  }

  const verificationTransition = verificationTransitions[machine.phase];
  if (verificationTransition) {
    const [nextPhase, includeAlignment] = verificationTransition;
    const verdict = combinedEvidence(evidence, includeAlignment);
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok
      ? machine.continuousValidSeconds + dt
      : 0;
    if (continuousValidSeconds >= ASSEMBLY1_STEP3_DURATIONS.graspCheckWindow) {
      return enterPhase(nextPhase, machine.reseatAttempts ?? 0);
    }
    if (phaseElapsed >= ASSEMBLY1_STEP3_DURATIONS.verificationTimeout) {
      if (
        machine.phase === 'alignment-verification'
        && (machine.reseatAttempts ?? 0) < 1
        && evidence.all?.ok
      ) {
        return enterPhase('reseat-lift', 1);
      }
      return terminalFailure(verdict);
    }
    return { ...machine, phaseElapsed, continuousValidSeconds };
  }

  if (machine.phase === 'aligned-hold') {
    const verdict = combinedEvidence(evidence, true);
    if (!verdict?.ok) return terminalFailure(verdict);
    const continuousValidSeconds = machine.continuousValidSeconds + dt;
    if (continuousValidSeconds >= ASSEMBLY1_STEP3_DURATIONS.alignedHold) {
      return enterPhase('release', machine.reseatAttempts ?? 0);
    }
    return {
      ...machine,
      phaseElapsed: machine.phaseElapsed + dt,
      continuousValidSeconds,
    };
  }

  if (machine.phase === 'placed-verification') {
    const verdict = combinedEvidence(evidence, true);
    if (!verdict?.ok) return terminalFailure(verdict);
    const continuousValidSeconds = machine.continuousValidSeconds + dt;
    if (continuousValidSeconds >= ASSEMBLY1_STEP3_DURATIONS.placedHold) {
      return enterPhase('complete', machine.reseatAttempts ?? 0);
    }
    return {
      ...machine,
      phaseElapsed: machine.phaseElapsed + dt,
      continuousValidSeconds,
    };
  }

  return machine;
}

export function createAssemblyStep3ControlFrame(machine, plans) {
  const progress = (duration) => machine.phaseElapsed / duration;
  const pathTarget = (path, value) => {
    const clamped = Math.max(0, Math.min(1, value));
    const scaled = clamped * Math.max(0, path.length - 1);
    const index = Math.min(path.length - 1, Math.floor(scaled));
    const next = Math.min(path.length - 1, index + 1);
    return interpolateJointTargets(path[index], path[next], scaled - index);
  };
  return {
    arms: plans.map((plan, index) => {
      let jointTargets = plan.hold;
      if (index === 1) {
        if (machine.phase === 'lift') {
          jointTargets = pathTarget(
            plan.hammerLiftPath ?? [plan.hold, plan.hammerPrelift, plan.hammerLift],
            progress(ASSEMBLY1_STEP3_DURATIONS.lift),
          );
        } else if (machine.phase === 'lift-settle') {
          jointTargets = plan.hammerLift;
        } else if (machine.phase === 'transfer-a') {
          jointTargets = pathTarget(
            plan.hammerHandoverPath ?? [plan.hammerLift, plan.hammerHandover],
            progress(ASSEMBLY1_STEP3_DURATIONS.transferA),
          );
        } else if (machine.phase !== 'grasp-check') {
          jointTargets = plan.hammerHandover;
        }
      } else if (index >= 2) {
        if (machine.phase === 'lift') {
          jointTargets = pathTarget(
            plan.transportLiftPath ?? [plan.hold, plan.lift],
            progress(ASSEMBLY1_STEP3_DURATIONS.lift),
          );
        } else if (machine.phase === 'lift-settle') {
          jointTargets = plan.lift;
        } else if (machine.phase === 'transfer-a') {
          jointTargets = pathTarget(
            plan.transportAPath ?? [plan.lift, plan.transferA],
            progress(ASSEMBLY1_STEP3_DURATIONS.transferA),
          );
        } else if (machine.phase === 'transfer-b') {
          const transferProgress = progress(ASSEMBLY1_STEP3_DURATIONS.transferB);
          jointTargets = plan.transportBPath
            ? pathTarget(plan.transportBPath, transferProgress)
            : transferProgress < 0.5
              ? interpolateJointTargets(plan.transferA, plan.transferMid, transferProgress * 2)
              : interpolateJointTargets(plan.transferMid, plan.hover, (transferProgress - 0.5) * 2);
        } else if (machine.phase === 'hover-settle') {
          jointTargets = plan.hover;
        } else if (machine.phase === 'aligned-descent') {
          const descentProgress = progress(ASSEMBLY1_STEP3_DURATIONS.alignedDescent);
          jointTargets = plan.transportDescentPath
            ? pathTarget(plan.transportDescentPath, descentProgress)
            : descentProgress < 0.5
              ? interpolateJointTargets(plan.hover, plan.descentMid, descentProgress * 2)
              : interpolateJointTargets(plan.descentMid, plan.aligned, (descentProgress - 0.5) * 2);
        } else if (machine.phase === 'reseat-lift') {
          jointTargets = interpolateJointTargets(
            plan.aligned,
            plan.descentMid,
            progress(ASSEMBLY1_STEP3_DURATIONS.reseatLift),
          );
        } else if (machine.phase === 'reseat-descent') {
          jointTargets = interpolateJointTargets(
            plan.descentMid,
            plan.aligned,
            progress(ASSEMBLY1_STEP3_DURATIONS.reseatDescent),
          );
        } else if (machine.phase === 'retreat') {
          jointTargets = interpolateJointTargets(
            plan.aligned,
            plan.home,
            progress(ASSEMBLY1_STEP3_DURATIONS.retreat),
          );
        } else if (
          machine.phase === 'alignment-verification'
          || machine.phase === 'aligned-hold'
          || machine.phase === 'release'
          || machine.phase === 'release-settle'
        ) {
          jointTargets = plan.aligned;
        } else if (machine.phase === 'placed-verification' || machine.phase === 'complete') {
          jointTargets = plan.home;
        }
      }
      let gripperTarget = ASSEMBLY1_STEP3_GRIPPER_CLAMPS[index];
      if (index >= 2 && machine.phase === 'release') {
        gripperTarget = interpolateJointTargets(
          [ASSEMBLY1_STEP3_GRIPPER_CLAMPS[index]],
          [255],
          progress(ASSEMBLY1_STEP3_DURATIONS.release),
        )[0];
      } else if (
        index >= 2
        && ['release-settle', 'retreat', 'placed-verification', 'complete'].includes(machine.phase)
      ) {
        gripperTarget = 255;
      }
      return {
        armKey: plan.armKey,
        jointTargets,
        gripperTarget,
      };
    }),
  };
}

export function holdAssemblyStep3Controls(controls, positions, arms) {
  for (const arm of arms) {
    for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
      controls[arm.actuatorIndices[joint]] = positions[arm.qposAddresses[joint]];
    }
  }
}
