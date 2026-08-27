import { interpolateJointTargets } from './assemblyStep1.js';
import { ASSEMBLY1_STEP2_ARMS } from './assemblyStep2.js';
import { ASSEMBLY1_STEP3_TRANSPORT_ARMS } from './assemblyStep3.js';

export const ASSEMBLY1_STEP4_DURATIONS = Object.freeze({
  prepare: 3,
  engage: 2,
  engageSettle: 3,
  fastenerClamp: 1.5,
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
  toolStage: 4,
});

export const ASSEMBLY1_STEP4_LIMITS = Object.freeze({
  minimumFastenerAperture: 0.005,
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
  frame: 48,
  tool: 96,
  pregrasp: 110,
  fastener: 70,
  open: 255,
});

export const ASSEMBLY1_STEP4_WAYPOINTS = Object.freeze({
  r1: Object.freeze({
    prepare: Object.freeze([0.46, -0.30, 0.36]),
    clear: Object.freeze([0.46, -0.10, 0.42]),
    ready: Object.freeze([0.40, 0.04, 0.43]),
  }),
  r2: Object.freeze({
    prepare: Object.freeze([0.12, 0.42, 0.34]),
    engage: Object.freeze([0.10, 0.39, 0.21]),
    lift: Object.freeze([0.12, 0.42, 0.40]),
    transfer: Object.freeze([0.04, 0.215, 0.43]),
    insert: Object.freeze([0.047, 0.202, 0.29]),
    clear: Object.freeze([-0.02, 0.35, 0.44]),
  }),
  r3: Object.freeze({
    prepare: Object.freeze([0, -0.1275, 0.34]),
    engage: Object.freeze([0, -0.1275, 0.34]),
  }),
});

const step3Transport = Object.fromEntries(
  ASSEMBLY1_STEP3_TRANSPORT_ARMS.map((arm) => [arm.key, arm]),
);
const holds = [
  [...ASSEMBLY1_STEP2_ARMS[0].contactJointTargets],
  [...ASSEMBLY1_STEP2_ARMS[1].contactJointTargets],
  [...step3Transport.r2.hoverJointTargets],
  [...step3Transport.r3.hoverJointTargets],
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
    role: 'torque driver pre-drive',
    jointTargets: targets(holds[1], {
      prepare: [2.028178, 0.037292, 0.141969, -2.219225, -0.007456, 2.258367, 0.132989],
      clear: [1.758575, 0.73779, -0.086946, -1.317063, 0.26433, 0.490352, -0.295778],
      ready: [1.540806, 1.035024, -0.013709, -0.764197, -0.161325, 0.232504, -0.640055],
    }),
  }),
  Object.freeze({
    key: 'r2',
    armIndex: 2,
    role: 'fastener pickup and insertion',
    jointTargets: targets(holds[2], {
      prepare: [2.42386, 0.257459, -0.084009, -1.974555, 0.027637, 2.229177, -1.601559],
      engage: [2.371997, 0.495975, -0.086653, -1.994508, 0.065861, 2.488322, -1.683131],
      lift: [2.436537, 0.184617, -0.097374, -1.914403, 0.020639, 2.100066, -1.596574],
      transfer: [2.204207, 0.574136, -0.200638, -1.262953, 0.112118, 1.828018, -1.920371],
      insert: [2.195823, 0.738371, -0.182853, -1.351784, 0.140278, 2.076907, -1.93544],
      clear: [2.302461, 0.072134, -0.226823, -1.956949, 0.018626, 2.026124, -1.859277],
    }),
  }),
  Object.freeze({
    key: 'r3',
    armIndex: 3,
    role: 'cross-member standby',
    jointTargets: targets(holds[3], {
      prepare: holds[3],
      engage: holds[3],
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
  return enterPhase('prepare');
}

function combinedEvidence(evidence, needFastenerGrasp = false, needPlacement = false) {
  if (!evidence.all?.ok) return evidence.all;
  if (needFastenerGrasp && !evidence.fastenerGrasp?.ok) return evidence.fastenerGrasp;
  if (needPlacement && !evidence.placement?.ok) return evidence.placement;
  return { ok: true };
}

const timedTransitions = {
  prepare: [ASSEMBLY1_STEP4_DURATIONS.prepare, 'engage', false, false],
  engage: [ASSEMBLY1_STEP4_DURATIONS.engage, 'engage-settle', false, false],
  'engage-settle': [ASSEMBLY1_STEP4_DURATIONS.engageSettle, 'fastener-clamp', false, false],
  'fastener-tighten': [ASSEMBLY1_STEP4_DURATIONS.fastenerTighten, 'lift', false, false],
  'fastener-clamp': [
    ASSEMBLY1_STEP4_DURATIONS.fastenerClamp,
    'fastener-verification',
    false,
    false,
  ],
  lift: [ASSEMBLY1_STEP4_DURATIONS.lift, 'transfer', true, false],
  transfer: [ASSEMBLY1_STEP4_DURATIONS.transfer, 'transfer-settle', true, false],
  'transfer-settle': [ASSEMBLY1_STEP4_DURATIONS.transferSettle, 'insert', true, false],
  insert: [ASSEMBLY1_STEP4_DURATIONS.insert, 'fastener-release', true, false],
  'fastener-release': [
    ASSEMBLY1_STEP4_DURATIONS.fastenerRelease,
    'clear',
    false,
    false,
  ],
  clear: [ASSEMBLY1_STEP4_DURATIONS.clear, 'placement-verification', false, false],
  'tool-stage': [ASSEMBLY1_STEP4_DURATIONS.toolStage, 'complete', false, true],
};

export function advanceAssemblyStep4Machine(machine, deltaSeconds, evidence) {
  if (machine.phase === 'complete' || machine.phase === 'error') return machine;
  const dt = Math.max(0, deltaSeconds);
  const timed = timedTransitions[machine.phase];
  if (timed) {
    const [duration, nextPhase, needFastenerGrasp, needPlacement] = timed;
    const verdict = combinedEvidence(evidence, needFastenerGrasp, needPlacement);
    if (!verdict?.ok) return terminalFailure(verdict);
    const phaseElapsed = machine.phaseElapsed + dt;
    return phaseElapsed >= duration ? enterPhase(nextPhase) : { ...machine, phaseElapsed };
  }

  if (machine.phase === 'fastener-verification') {
    const verdict = combinedEvidence(evidence, true, false);
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok
      ? machine.continuousValidSeconds + dt
      : 0;
    const lastInvalidVerdict = verdict?.ok ? machine.lastInvalidVerdict : verdict;
    if (continuousValidSeconds >= ASSEMBLY1_STEP4_DURATIONS.verificationWindow) {
      return enterPhase('fastener-tighten');
    }
    if (phaseElapsed >= ASSEMBLY1_STEP4_DURATIONS.verificationTimeout) {
      return terminalFailure(verdict?.ok ? lastInvalidVerdict : verdict);
    }
    return { ...machine, phaseElapsed, continuousValidSeconds, lastInvalidVerdict };
  }

  if (machine.phase === 'placement-verification') {
    const verdict = combinedEvidence(evidence, false, true);
    const phaseElapsed = machine.phaseElapsed + dt;
    const continuousValidSeconds = verdict?.ok ? machine.continuousValidSeconds + dt : 0;
    const lastInvalidVerdict = verdict?.ok ? machine.lastInvalidVerdict : verdict;
    if (continuousValidSeconds >= ASSEMBLY1_STEP4_DURATIONS.placementHold) {
      return enterPhase('tool-stage');
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
  if (machine.phase === 'prepare') {
    return interpolateJointTargets(plan.hold, plan.prepare, progress(ASSEMBLY1_STEP4_DURATIONS.prepare));
  }
  if (machine.phase === 'engage') {
    return interpolateJointTargets(plan.prepare, plan.engage, progress(ASSEMBLY1_STEP4_DURATIONS.engage));
  }
  if (
    machine.phase === 'engage-settle'
    || machine.phase === 'fastener-clamp'
    || machine.phase === 'fastener-verification'
    || machine.phase === 'fastener-tighten'
  ) {
    return plan.engage;
  }
  if (machine.phase === 'lift') {
    return interpolateJointTargets(plan.engage, plan.lift, progress(ASSEMBLY1_STEP4_DURATIONS.lift));
  }
  if (machine.phase === 'transfer') {
    return interpolateJointTargets(plan.lift, plan.transfer, progress(ASSEMBLY1_STEP4_DURATIONS.transfer));
  }
  if (machine.phase === 'transfer-settle') return plan.transfer;
  if (machine.phase === 'insert') {
    return interpolateJointTargets(plan.transfer, plan.insert, progress(ASSEMBLY1_STEP4_DURATIONS.insert));
  }
  if (machine.phase === 'fastener-release') return plan.insert;
  if (machine.phase === 'clear') {
    return interpolateJointTargets(plan.insert, plan.clear, progress(ASSEMBLY1_STEP4_DURATIONS.clear));
  }
  if (machine.phase === 'placement-verification') return plan.clear;
  if (machine.phase === 'tool-stage') {
    if (plan.armKey !== 'r1') return plan.clear;
    return interpolateJointTargets(plan.clear, plan.ready, progress(ASSEMBLY1_STEP4_DURATIONS.toolStage));
  }
  if (plan.armKey === 'r2') return plan.clear;
  return plan.ready;
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
      if (index === 2 && machine.phase === 'fastener-clamp') {
        gripperTarget = interpolateJointTargets(
          [ASSEMBLY1_STEP4_GRIPPERS.open],
          [ASSEMBLY1_STEP4_GRIPPERS.pregrasp],
          machine.phaseElapsed / ASSEMBLY1_STEP4_DURATIONS.fastenerClamp,
        )[0];
      } else if (
        index === 2
        && machine.phase === 'fastener-verification'
      ) {
        gripperTarget = ASSEMBLY1_STEP4_GRIPPERS.pregrasp;
      } else if (index === 2 && machine.phase === 'fastener-tighten') {
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
