import { interpolateJointTargets } from './assemblyStep1.js';
import { ASSEMBLY1_STEP2_ARMS } from './assemblyStep2.js';
import {
  ASSEMBLY1_STEP3_HAMMER_ARM,
  ASSEMBLY1_STEP3_HOME_JOINT_TARGETS,
} from './assemblyStep3.js';

export const ASSEMBLY1_STEP4_DURATIONS = Object.freeze({
  donorTighten: 1.2,
  prepare: 3,
  engage: 2,
  engageSettle: 3,
  dualClamp: 1.5,
  hammerRelease: 0.8,
  donorClear: 2,
  fastenerTighten: 5,
  verificationWindow: 0.2,
  contactGrace: 0.75,
  verificationTimeout: 4,
  lift: 6,
  transfer: 3,
  transferSettle: 2,
  insert: 5,
  fastenerRelease: 0.8,
  clear: 2,
  placementHold: 1,
  placementTimeout: 4,
  hammerStage: 4,
  hammerStrike: 0.9,
  hammerRecover: 1.2,
});

export const ASSEMBLY1_STEP4_LIMITS = Object.freeze({
  minimumFastenerAperture: 0.005,
  minimumToolAperture: 0.035,
  maximumContactPenetration: 0.002,
  frameTranslation: 0.012,
  crossMemberTranslation: 0.025,
  crossMemberRotationDegrees: 12,
  fastenerPlanarDistance: 0.018,
  fastenerMinimumHeight: 0.275,
  fastenerMaximumHeight: 0.34,
  comparisonEpsilon: 0.0005,
  rotationComparisonEpsilonDegrees: 0.5,
});

export const ASSEMBLY1_STEP4_GRIPPERS = Object.freeze({
  frame: 130,
  donorEntry: 122,
  tool: 122,
  receiverTool: 127,
  // A 110 command leaves about 34.5 mm between the fingers, wider than the
  // 30 mm fastener head.  Close to 80 during the shared clamp phase so both
  // fingers establish contact instead of one finger sweeping the part aside.
  pregrasp: 80,
  fastener: 70,
  open: 255,
});

export const ASSEMBLY1_STEP4_WAYPOINTS = Object.freeze({
  r1: Object.freeze({
    prepare: Object.freeze([0.30, -0.18, 0.45]),
    engage: Object.freeze([0.095, 0, 0.40]),
    clear: Object.freeze([0.46, -0.10, 0.45]),
  }),
  r2: Object.freeze({
    // The Panda TCP is roughly 30 mm behind the physical finger pads in this
    // orientation.  Offset south and slightly west so both pads close around
    // the 30 mm fastener head instead of one pad sweeping it aside.
    prepare: Object.freeze([0.115, 0.39, 0.34]),
    engage: Object.freeze([0.115, 0.39, 0.16]),
    lift: Object.freeze([0.115, 0.39, 0.40]),
    transfer: Object.freeze([-0.04, 0.215, 0.43]),
    insert: Object.freeze([-0.04, 0.215, 0.29]),
    clear: Object.freeze([-0.10, 0.35, 0.44]),
  }),
  r3: Object.freeze({
    prepare: Object.freeze([-0.015, 0.02, 0.44]),
    engage: Object.freeze([-0.015, 0.02, 0.40]),
    clear: Object.freeze([-0.12, 0.08, 0.45]),
    ready: Object.freeze([-0.25, 0.215, 0.41]),
    strike: Object.freeze([-0.25, 0.215, 0.35]),
  }),
});

const holds = [
  [...ASSEMBLY1_STEP2_ARMS[0].contactJointTargets],
  [...ASSEMBLY1_STEP3_HAMMER_ARM.handoverJointTargets],
  [...ASSEMBLY1_STEP3_HOME_JOINT_TARGETS],
  [...ASSEMBLY1_STEP3_HOME_JOINT_TARGETS],
];

function targets(hold, overrides = {}) {
  return Object.freeze({
    hold: Object.freeze([...hold]),
    prepare: Object.freeze([...(overrides.prepare ?? hold)]),
    engage: Object.freeze([...(overrides.engage ?? overrides.prepare ?? hold)]),
    lift: Object.freeze([...(overrides.lift ?? overrides.engage ?? overrides.prepare ?? hold)]),
    transfer: Object.freeze([...(overrides.transfer ?? overrides.lift ?? hold)]),
    insert: Object.freeze([...(overrides.insert ?? overrides.transfer ?? hold)]),
    clear: Object.freeze([...(overrides.clear ?? overrides.insert ?? hold)]),
    ready: Object.freeze([...(overrides.ready ?? overrides.clear ?? hold)]),
    strike: Object.freeze([...(overrides.strike ?? overrides.ready ?? overrides.clear ?? hold)]),
  });
}

// The recorded targets are generated and checked by scripts/solve-assembly-step4-waypoints.mjs.
export const ASSEMBLY1_STEP4_ARMS = Object.freeze([
  Object.freeze({
    key: 'r0',
    armIndex: 0,
    role: 'frame hold',
    jointTargets: targets(holds[0]),
  }),
  Object.freeze({
    key: 'r1',
    armIndex: 1,
    role: 'hammer handover donor',
    closingAxisYawDegrees: 90,
    jointTargets: targets(holds[1], {
      prepare: [1.157142, 0.305927, 0.795861, -1.722408, -0.231751, 1.93218, 1.226666],
      engage: [1.558404, 1.079203, 0.044088, -0.466945, -0.039084, 1.575101, 0.792853],
      clear: [1.698143, -0.304484, 0.089632, -2.348099, 0.030022, 2.043414, 0.984635],
    }),
  }),
  Object.freeze({
    key: 'r2',
    armIndex: 2,
    role: 'fastener pickup and insertion',
    jointTargets: targets(holds[2], {
      prepare: [0.073933, -0.536314, 2.113671, -1.879544, 0.534672, 2.110019, -1.964995],
      engage: [-0.125202, -0.820445, 2.392948, -1.886877, 0.802016, 2.377185, -2.115963],
      lift: [0.112454, -0.451261, 2.05785, -1.820968, 0.438653, 2.006641, -1.906512],
      transfer: [-0.456726, -0.651346, 2.149446, -1.4471, 0.548932, 1.805163, -2.265575],
      insert: [-0.484171, -0.78883, 2.25253, -1.597063, 0.671086, 2.054394, -2.338392],
      clear: [0.181851, 0.136814, 1.809115, -2.079331, -0.148766, 2.04078, -1.866059],
    }),
  }),
  Object.freeze({
    key: 'r3',
    armIndex: 3,
    role: 'hammer handover receiver and strike',
    closingAxisYawDegrees: 90,
    jointTargets: targets(holds[3], {
      prepare: [-1.615043, -1.026006, -2.894244, -0.465278, -0.212472, 1.495639, -2.289955],
      engage: [-1.614529, -1.073949, -2.897069, -0.473746, -0.214538, 1.534482, -2.289185],
      clear: [-1.703553, -0.443765, -2.823195, -1.428873, -0.140335, 1.84872, -2.160632],
      ready: [-1.595525, -0.160565, -2.724549, -1.934765, -0.074376, 2.079056, -1.932282],
      strike: [-1.587963, -0.231146, -2.733246, -2.008228, -0.113806, 2.217128, -1.906121],
    }),
  }),
]);

function enterPhase(phase) {
  return {
    phase,
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    lastInvalidVerdict: null,
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

export function createAssemblyStep4Machine() {
  return enterPhase('donor-tighten');
}

function combinedEvidence(
  evidence,
  needFastenerGrasp = false,
  needPlacement = false,
  needHammerGrasp = false,
) {
  if (!evidence.all?.ok) return evidence.all;
  if (needFastenerGrasp && !evidence.fastenerGrasp?.ok) return evidence.fastenerGrasp;
  if (needHammerGrasp && !evidence.hammerGrasp?.ok) return evidence.hammerGrasp;
  if (needPlacement && !evidence.placement?.ok) return evidence.placement;
  return { ok: true };
}

const timedTransitions = {
  'donor-tighten': [ASSEMBLY1_STEP4_DURATIONS.donorTighten, 'prepare', false, false, false],
  prepare: [ASSEMBLY1_STEP4_DURATIONS.prepare, 'engage', false, false, false],
  engage: [ASSEMBLY1_STEP4_DURATIONS.engage, 'engage-settle', false, false, false],
  'engage-settle': [ASSEMBLY1_STEP4_DURATIONS.engageSettle, 'dual-clamp', false, false, false],
  'dual-clamp': [ASSEMBLY1_STEP4_DURATIONS.dualClamp, 'handover-verification', false, false, false],
  'hammer-release': [ASSEMBLY1_STEP4_DURATIONS.hammerRelease, 'donor-clear', false, false, true],
  'donor-clear': [ASSEMBLY1_STEP4_DURATIONS.donorClear, 'fastener-tighten', false, false, true],
  'fastener-tighten': [ASSEMBLY1_STEP4_DURATIONS.fastenerTighten, 'lift', false, false, true],
  lift: [ASSEMBLY1_STEP4_DURATIONS.lift, 'transfer', true, false, true],
  transfer: [ASSEMBLY1_STEP4_DURATIONS.transfer, 'transfer-settle', true, false, true],
  'transfer-settle': [ASSEMBLY1_STEP4_DURATIONS.transferSettle, 'insert', true, false, true],
  insert: [ASSEMBLY1_STEP4_DURATIONS.insert, 'fastener-release', true, false, true],
  'fastener-release': [
    ASSEMBLY1_STEP4_DURATIONS.fastenerRelease,
    'clear',
    false,
    false,
    true,
  ],
  clear: [ASSEMBLY1_STEP4_DURATIONS.clear, 'placement-verification', false, false, true],
  'hammer-stage': [ASSEMBLY1_STEP4_DURATIONS.hammerStage, 'hammer-strike', false, true, true],
  'hammer-strike': [ASSEMBLY1_STEP4_DURATIONS.hammerStrike, 'hammer-recover', false, true, true],
  'hammer-recover': [ASSEMBLY1_STEP4_DURATIONS.hammerRecover, 'complete', false, true, true],
};

export function advanceAssemblyStep4Machine(machine, deltaSeconds, evidence) {
  if (machine.phase === 'complete' || machine.phase === 'error') return machine;
  const dt = Math.max(0, deltaSeconds);
  const timed = timedTransitions[machine.phase];
  if (timed) {
    const [duration, nextPhase, needFastenerGrasp, needPlacement, needHammerGrasp] = timed;
    const verdict = combinedEvidence(
      evidence,
      needFastenerGrasp,
      needPlacement,
      needHammerGrasp,
    );
    if (!verdict?.ok) return terminalFailure(verdict);
    const phaseElapsed = machine.phaseElapsed + dt;
    return phaseElapsed >= duration ? enterPhase(nextPhase) : { ...machine, phaseElapsed };
  }

  if (machine.phase === 'handover-verification') {
    const verdict = combinedEvidence(evidence, false, false, true);
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok
      ? machine.continuousValidSeconds + dt
      : 0;
    const lastInvalidVerdict = verdict?.ok ? machine.lastInvalidVerdict : verdict;
    if (continuousValidSeconds >= ASSEMBLY1_STEP4_DURATIONS.verificationWindow) {
      return enterPhase('hammer-release');
    }
    if (phaseElapsed >= ASSEMBLY1_STEP4_DURATIONS.verificationTimeout) {
      return terminalFailure(verdict?.ok ? lastInvalidVerdict : verdict);
    }
    return { ...machine, phaseElapsed, continuousValidSeconds, lastInvalidVerdict };
  }

  if (machine.phase === 'placement-verification') {
    const verdict = combinedEvidence(evidence, false, true, true);
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok ? machine.continuousValidSeconds + dt : 0;
    const lastInvalidVerdict = verdict?.ok ? machine.lastInvalidVerdict : verdict;
    if (continuousValidSeconds >= ASSEMBLY1_STEP4_DURATIONS.placementHold) {
      return enterPhase('hammer-stage');
    }
    if (phaseElapsed >= ASSEMBLY1_STEP4_DURATIONS.placementTimeout) {
      return terminalFailure(verdict?.ok ? lastInvalidVerdict : verdict);
    }
    return {
      ...machine,
      phaseElapsed,
      continuousValidSeconds,
      lastInvalidVerdict,
    };
  }

  return machine;
}

function phaseTargets(machine, plan) {
  const progress = (duration) => machine.phaseElapsed / duration;
  if (machine.phase === 'donor-tighten') return plan.hold;
  if (machine.phase === 'prepare') {
    return interpolateJointTargets(plan.hold, plan.prepare, progress(ASSEMBLY1_STEP4_DURATIONS.prepare));
  }
  if (machine.phase === 'engage') {
    return interpolateJointTargets(plan.prepare, plan.engage, progress(ASSEMBLY1_STEP4_DURATIONS.engage));
  }
  if (
    machine.phase === 'engage-settle'
    || machine.phase === 'dual-clamp'
    || machine.phase === 'handover-verification'
    || machine.phase === 'hammer-release'
  ) {
    return plan.engage;
  }
  if (machine.phase === 'donor-clear') {
    if (plan.armKey !== 'r1' && plan.armKey !== 'r3') return plan.engage;
    return interpolateJointTargets(
      plan.engage,
      plan.clear,
      progress(ASSEMBLY1_STEP4_DURATIONS.donorClear),
    );
  }
  if (machine.phase === 'fastener-tighten') {
    return plan.armKey === 'r1' || plan.armKey === 'r3' ? plan.clear : plan.engage;
  }
  if (machine.phase === 'lift') {
    if (plan.armKey === 'r1' || plan.armKey === 'r3') return plan.clear;
    return interpolateJointTargets(plan.engage, plan.lift, progress(ASSEMBLY1_STEP4_DURATIONS.lift));
  }
  if (machine.phase === 'transfer') {
    if (plan.armKey === 'r1' || plan.armKey === 'r3') return plan.clear;
    return interpolateJointTargets(plan.lift, plan.transfer, progress(ASSEMBLY1_STEP4_DURATIONS.transfer));
  }
  if (machine.phase === 'transfer-settle') {
    return plan.armKey === 'r1' || plan.armKey === 'r3' ? plan.clear : plan.transfer;
  }
  if (machine.phase === 'insert') {
    if (plan.armKey === 'r1' || plan.armKey === 'r3') return plan.clear;
    return interpolateJointTargets(plan.transfer, plan.insert, progress(ASSEMBLY1_STEP4_DURATIONS.insert));
  }
  if (machine.phase === 'fastener-release') {
    return plan.armKey === 'r1' || plan.armKey === 'r3' ? plan.clear : plan.insert;
  }
  if (machine.phase === 'clear') {
    if (plan.armKey === 'r1' || plan.armKey === 'r3') return plan.clear;
    return interpolateJointTargets(plan.insert, plan.clear, progress(ASSEMBLY1_STEP4_DURATIONS.clear));
  }
  if (machine.phase === 'placement-verification') return plan.clear;
  if (machine.phase === 'hammer-stage') {
    if (plan.armKey !== 'r3') return plan.clear;
    return interpolateJointTargets(plan.clear, plan.ready, progress(ASSEMBLY1_STEP4_DURATIONS.hammerStage));
  }
  if (machine.phase === 'hammer-strike') {
    if (plan.armKey !== 'r3') return plan.clear;
    return interpolateJointTargets(plan.ready, plan.strike, progress(ASSEMBLY1_STEP4_DURATIONS.hammerStrike));
  }
  if (machine.phase === 'hammer-recover') {
    if (plan.armKey !== 'r3') return plan.clear;
    return interpolateJointTargets(plan.strike, plan.ready, progress(ASSEMBLY1_STEP4_DURATIONS.hammerRecover));
  }
  if (plan.armKey === 'r3') return plan.ready;
  return plan.clear;
}

export function createAssemblyStep4ControlFrame(machine, plans) {
  return {
    arms: plans.map((plan, index) => {
      let gripperTarget = [
        ASSEMBLY1_STEP4_GRIPPERS.frame,
        ASSEMBLY1_STEP4_GRIPPERS.tool,
        ASSEMBLY1_STEP4_GRIPPERS.open,
        ASSEMBLY1_STEP4_GRIPPERS.open,
      ][index];
      if (machine.phase === 'donor-tighten' && index === 1) {
        gripperTarget = interpolateJointTargets(
          [ASSEMBLY1_STEP4_GRIPPERS.donorEntry],
          [ASSEMBLY1_STEP4_GRIPPERS.tool],
          machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.donorTighten,
        )[0];
      } else if (machine.phase === 'dual-clamp') {
        if (index === 2) {
          gripperTarget = interpolateJointTargets(
            [ASSEMBLY1_STEP4_GRIPPERS.open],
            [ASSEMBLY1_STEP4_GRIPPERS.pregrasp],
            machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.dualClamp,
          )[0];
        } else if (index === 3) {
          gripperTarget = interpolateJointTargets(
            [ASSEMBLY1_STEP4_GRIPPERS.open],
            [ASSEMBLY1_STEP4_GRIPPERS.receiverTool],
            machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.dualClamp,
          )[0];
        }
      } else if (machine.phase === 'handover-verification') {
        if (index === 2) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.pregrasp;
        if (index === 3) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.receiverTool;
      } else if (machine.phase === 'hammer-release') {
        if (index === 1) {
          gripperTarget = interpolateJointTargets(
            [ASSEMBLY1_STEP4_GRIPPERS.tool],
            [ASSEMBLY1_STEP4_GRIPPERS.open],
            machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.hammerRelease,
          )[0];
        }
        if (index === 2) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.pregrasp;
        if (index === 3) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.receiverTool;
      } else if (['donor-clear', 'fastener-tighten', 'lift', 'transfer', 'transfer-settle', 'insert', 'fastener-release', 'clear', 'placement-verification', 'hammer-stage', 'hammer-strike', 'hammer-recover', 'complete'].includes(machine.phase)) {
        if (index === 1) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.open;
        if (index === 2 && machine.phase === 'donor-clear') {
          gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.pregrasp;
        }
        if (index === 3) gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.receiverTool;
      }

      if (index === 2 && machine.phase === 'fastener-tighten') {
        gripperTarget = interpolateJointTargets(
          [ASSEMBLY1_STEP4_GRIPPERS.pregrasp],
          [ASSEMBLY1_STEP4_GRIPPERS.fastener],
          machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.fastenerTighten,
        )[0];
      } else if (
        index === 2
        && ['lift', 'transfer', 'transfer-settle', 'insert'].includes(machine.phase)
      ) {
        gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.fastener;
      } else if (index === 2 && machine.phase === 'fastener-release') {
        gripperTarget = interpolateJointTargets(
          [ASSEMBLY1_STEP4_GRIPPERS.fastener],
          [ASSEMBLY1_STEP4_GRIPPERS.open],
          machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.fastenerRelease,
        )[0];
      }
      return {
        armKey: plan.armKey,
        jointTargets: phaseTargets(machine, plan),
        gripperTarget,
      };
    }),
  };
}

export function evaluateAssemblyStep4Stability({
  frameTranslation,
  crossMemberTranslation,
  crossMemberRotationDegrees,
}) {
  if (![frameTranslation, crossMemberTranslation, crossMemberRotationDegrees].every(Number.isFinite)) {
    return { ok: false, code: 'non-finite-runtime' };
  }
  if (frameTranslation > ASSEMBLY1_STEP4_LIMITS.frameTranslation + ASSEMBLY1_STEP4_LIMITS.comparisonEpsilon) {
    return { ok: false, code: 'frame-drift', detail: String(frameTranslation) };
  }
  return { ok: true };
}

export function evaluateAssemblyStep4Placement({ fastenerPlanarDistance, fastenerHeight }) {
  if (![fastenerPlanarDistance, fastenerHeight].every(Number.isFinite)) {
    return { ok: false, code: 'non-finite-runtime' };
  }
  if (fastenerPlanarDistance > ASSEMBLY1_STEP4_LIMITS.fastenerPlanarDistance) {
    return { ok: false, code: 'fastener-misalignment', detail: String(fastenerPlanarDistance) };
  }
  if (
    fastenerHeight < ASSEMBLY1_STEP4_LIMITS.fastenerMinimumHeight
    || fastenerHeight > ASSEMBLY1_STEP4_LIMITS.fastenerMaximumHeight
  ) {
    return { ok: false, code: 'fastener-height', detail: String(fastenerHeight) };
  }
  return { ok: true };
}
