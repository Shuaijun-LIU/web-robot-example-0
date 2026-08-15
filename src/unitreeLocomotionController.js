import {
  G1_ACTUATORS,
  G1_HOME,
  GO2_ACTUATORS,
  GO2_HOME,
} from './unitreeActionSequence.js';

const TAU = Math.PI * 2;
const G1_GAIT_FREQUENCY = 1.35;
const GO2_TROT_FREQUENCY = 2.1;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const clamp01 = (value) => clamp(value, 0, 1);
const smoothstep = (value) => {
  const t = clamp01(value);
  return t * t * (3 - 2 * t);
};
const mix = (from, to, amount) => from.map(
  (value, index) => value + (to[index] - value) * amount,
);

export const G1_WALK_READY = [...G1_HOME];
G1_WALK_READY[0] = -0.2;
G1_WALK_READY[3] = 0.4;
G1_WALK_READY[4] = -0.2;
G1_WALK_READY[6] = -0.2;
G1_WALK_READY[9] = 0.4;
G1_WALK_READY[10] = -0.2;

export const G1_SQUAT = [...G1_HOME];
G1_SQUAT[0] = -0.65;
G1_SQUAT[3] = 1.3;
G1_SQUAT[4] = -0.65;
G1_SQUAT[6] = -0.65;
G1_SQUAT[9] = 1.3;
G1_SQUAT[10] = -0.65;

const G1_FINAL_GREETING = [...G1_HOME];
G1_FINAL_GREETING[23] = -1.45;
G1_FINAL_GREETING[25] = -0.85;
G1_FINAL_GREETING[27] = 0.22;

const GO2_SCAN_ARM = [0.52, -0.72, 1.12, 1.15, -0.38, -1.18];

const ZERO_ATTITUDE = Object.freeze({
  roll: 0,
  pitch: 0,
  rollRate: 0,
  pitchRate: 0,
});

export function locomotionEnvelope(progress, rampFraction = 0.18) {
  const value = Number.isFinite(progress) ? progress : 0;
  if (value <= 0 || value >= 1) return 0;
  const ramp = clamp(rampFraction, 1e-6, 0.5);
  if (value < ramp) return smoothstep(value / ramp);
  if (value > 1 - ramp) return smoothstep((1 - value) / ramp);
  return 1;
}

export function computeBalanceCorrection(
  attitude = ZERO_ATTITUDE,
  {
    rollKp = 1.05,
    pitchKp = 1.05,
    rollKd = 0.12,
    pitchKd = 0.12,
    limit = 0.14,
  } = {},
) {
  const roll = Number.isFinite(attitude.roll) ? attitude.roll : 0;
  const pitch = Number.isFinite(attitude.pitch) ? attitude.pitch : 0;
  const rollRate = Number.isFinite(attitude.rollRate) ? attitude.rollRate : 0;
  const pitchRate = Number.isFinite(attitude.pitchRate) ? attitude.pitchRate : 0;
  const boundedLimit = Math.max(0, Number.isFinite(limit) ? limit : 0.14);
  return {
    roll: clamp(-(rollKp * roll + rollKd * rollRate), -boundedLimit, boundedLimit),
    pitch: clamp(-(pitchKp * pitch + pitchKd * pitchRate), -boundedLimit, boundedLimit),
  };
}

function clampTargets(targets, actuators) {
  let clampCount = 0;
  const bounded = targets.map((target, index) => {
    const value = Number.isFinite(target) ? target : 0;
    const next = clamp(value, actuators[index].min, actuators[index].max);
    if (next !== target) clampCount += 1;
    return next;
  });
  return { targets: bounded, clampCount };
}

export function sampleG1Squat(progress, returningToStand = false) {
  const eased = smoothstep(progress);
  return returningToStand
    ? mix(G1_SQUAT, G1_WALK_READY, eased)
    : mix(G1_HOME, G1_SQUAT, eased);
}

export function sampleG1Gait(cyclePhase, envelope = 1, attitude = ZERO_ATTITUDE) {
  const amount = clamp01(envelope);
  const phase = TAU * (Number.isFinite(cyclePhase) ? cyclePhase : 0);
  const left = Math.sin(phase);
  const right = -left;
  const lateral = Math.cos(phase);
  const targets = [...G1_WALK_READY];

  const applyLeg = (offset, wave, side) => {
    const swing = Math.max(0, wave);
    const stance = Math.max(0, -wave);
    targets[offset] += amount * 0.23 * wave;
    targets[offset + 1] += amount * side * 0.025 * lateral;
    targets[offset + 3] += amount * (0.26 * swing - 0.07 * stance);
    targets[offset + 4] += amount * (-0.11 * wave - 0.07 * swing);
    targets[offset + 5] += amount * side * -0.018 * lateral;
  };
  applyLeg(0, left, 1);
  applyLeg(6, right, -1);

  targets[15] -= amount * 0.28 * left;
  targets[22] -= amount * 0.28 * right;
  targets[18] += amount * 0.08 * Math.max(0, -left);
  targets[25] += amount * 0.08 * Math.max(0, -right);

  const correction = computeBalanceCorrection(attitude);
  const roll = correction.roll * amount;
  const pitch = correction.pitch * amount;
  targets[0] += pitch * 0.55;
  targets[6] += pitch * 0.55;
  targets[4] -= pitch * 0.45;
  targets[10] -= pitch * 0.45;
  targets[1] += roll * 0.55;
  targets[7] += roll * 0.55;
  targets[5] -= roll * 0.45;
  targets[11] -= roll * 0.45;
  targets[13] += roll * 0.15;
  targets[14] += pitch * 0.15;

  return {
    ...clampTargets(targets, G1_ACTUATORS),
    cyclePhase: ((cyclePhase % 1) + 1) % 1,
    envelope: amount,
    correction,
  };
}

export function sampleGo2Trot(cyclePhase, envelope = 1, attitude = ZERO_ATTITUDE) {
  const amount = clamp01(envelope);
  const phase = TAU * (Number.isFinite(cyclePhase) ? cyclePhase : 0);
  const targets = [...GO2_HOME];
  const correction = computeBalanceCorrection(attitude, {
    rollKp: 0.8,
    pitchKp: 0.8,
    rollKd: 0.08,
    pitchKd: 0.08,
    limit: 0.12,
  });
  const legs = [
    { offset: 0, phaseOffset: 0, side: 1, front: 1 },
    { offset: 3, phaseOffset: Math.PI, side: -1, front: 1 },
    { offset: 6, phaseOffset: Math.PI, side: 1, front: -1 },
    { offset: 9, phaseOffset: 0, side: -1, front: -1 },
  ];

  for (const leg of legs) {
    const wave = Math.sin(phase + leg.phaseOffset);
    const swing = Math.max(0, wave);
    const stance = Math.max(0, -wave);
    targets[leg.offset] += amount * (
      leg.side * 0.025 * Math.cos(phase + leg.phaseOffset)
      + leg.side * correction.roll * 0.6
    );
    targets[leg.offset + 1] += amount * (
      0.22 * wave
      + leg.front * correction.pitch * 0.55
    );
    targets[leg.offset + 2] += amount * (-0.2 * swing + 0.09 * stance);
  }

  return {
    ...clampTargets(targets, GO2_ACTUATORS),
    cyclePhase: ((cyclePhase % 1) + 1) % 1,
    envelope: amount,
    correction,
  };
}

function finalGreeting(progress) {
  const p = clamp01(progress);
  const envelope = Math.sin(Math.PI * p) ** 2;
  const g1Targets = mix(G1_HOME, G1_FINAL_GREETING, envelope);
  g1Targets[26] += 0.52 * envelope * Math.cos(4 * Math.PI * p);
  const go2Targets = [
    ...GO2_HOME.slice(0, 12),
    ...mix(GO2_HOME.slice(12), GO2_SCAN_ARM, envelope),
  ];
  return { g1Targets, go2Targets };
}

export function sampleUnitreeLocomotionAction(elapsedSeconds, feedback = {}) {
  const time = clamp(Number.isFinite(elapsedSeconds) ? elapsedSeconds : 0, 0, 25);
  const g1Attitude = feedback.g1 ?? ZERO_ATTITUDE;
  const go2Attitude = feedback.go2 ?? ZERO_ATTITUDE;
  let phase = 'complete';
  let g1Targets = [...G1_HOME];
  let go2Targets = [...GO2_HOME];
  let clampCount = 0;
  let gaitEnvelope = 0;
  let cyclePhase = 0;

  if (time < 1) {
    phase = 'settle';
  } else if (time < 3) {
    phase = 'g1-squat';
    g1Targets = sampleG1Squat((time - 1) / 2);
  } else if (time < 5) {
    phase = 'g1-stand';
    g1Targets = sampleG1Squat((time - 3) / 2, true);
  } else if (time < 11) {
    phase = 'g1-walk';
    const progress = (time - 5) / 6;
    gaitEnvelope = locomotionEnvelope(progress);
    cyclePhase = ((time - 5) * G1_GAIT_FREQUENCY) % 1;
    const gait = sampleG1Gait(cyclePhase, gaitEnvelope, g1Attitude);
    g1Targets = gait.targets;
    clampCount += gait.clampCount;
  } else if (time < 13) {
    phase = 'g1-stabilize';
    g1Targets = mix(G1_WALK_READY, G1_HOME, smoothstep((time - 11) / 2));
  } else if (time < 19) {
    phase = 'go2-walk';
    const progress = (time - 13) / 6;
    gaitEnvelope = locomotionEnvelope(progress);
    cyclePhase = ((time - 13) * GO2_TROT_FREQUENCY) % 1;
    const gait = sampleGo2Trot(cyclePhase, gaitEnvelope, go2Attitude);
    go2Targets = gait.targets;
    clampCount += gait.clampCount;
  } else if (time < 21) {
    phase = 'go2-stabilize';
  } else if (time < 24) {
    phase = 'final-greeting';
    ({ g1Targets, go2Targets } = finalGreeting((time - 21) / 3));
  } else if (time < 25) {
    phase = 'final-hold';
  }

  const boundedG1 = clampTargets(g1Targets, G1_ACTUATORS);
  const boundedGo2 = clampTargets(go2Targets, GO2_ACTUATORS);
  return {
    phase,
    elapsed: time,
    g1Targets: boundedG1.targets,
    go2Targets: boundedGo2.targets,
    diagnostics: {
      cyclePhase,
      envelope: gaitEnvelope,
      clampCount: clampCount + boundedG1.clampCount + boundedGo2.clampCount,
    },
  };
}
