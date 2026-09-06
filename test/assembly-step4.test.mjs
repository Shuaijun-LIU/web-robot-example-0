import assert from 'node:assert/strict';
import test from 'node:test';

test('dense physical paths do not stop and restart at every IK sample', async()=>{
  const {createAssemblyStep4ControlFrame,ASSEMBLY1_STEP4_DURATIONS}=await import('../src/assemblyStep4.js');
  const plan={armKey:'r1',phasePaths:{engage:[Array(7).fill(0),Array(7).fill(1),Array(7).fill(2)]}};
  const duration=ASSEMBLY1_STEP4_DURATIONS.engage;
  const value=p=>createAssemblyStep4ControlFrame({phase:'engage',phaseElapsed:p*duration},[plan]).arms[0].jointTargets[0];
  const centerVelocity=(value(.501)-value(.499))/(.002*duration);
  assert.ok(centerVelocity>1/duration,'interior IK waypoint must not introduce a stop');
});
import { FRANKA_HOME } from '../src/sceneLayouts.js';

let step4 = {};
try {
  step4 = await import('../src/assemblyStep4.js');
} catch {
  // The first RED run intentionally reaches the assertions with no Step 4 module.
}

const valid = {
  all: { ok: true },
  fastenerGrasp: { ok: true },
  fastenerCurrentGrasp: { ok: true },
  hammerGrasp: { ok: true },
  placement: { ok: true },
};

test('Step 4 rejects hidden collision contact outside the visible receiver grasp zone', () => {
  assert.deepEqual(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.04,
    receiverGraspPointDistance: 0.04,
    leftContactDistance: -0.0002,
    rightContactDistance: -0.0002,
  }), { ok: false, code: 'receiver-off-grasp-zone', armKey: 'r3', detail: '0.04' });
  assert.deepEqual(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.04,
    receiverGraspPointDistance: 0.008,
    leftContactDistance: -0.0002,
    rightContactDistance: -0.0002,
  }), { ok: true });
  assert.deepEqual(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.04,
    receiverGraspPointDistance: 0.008,
    leftContactDistance: -0.0022,
    rightContactDistance: -0.0002,
  }), { ok: true });
  assert.equal(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.04,
    receiverGraspPointDistance: 0.008,
    leftContactDistance: -0.0024,
    rightContactDistance: -0.0002,
  }).code, 'deep-penetration');
  assert.equal(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.0472,
    receiverGraspPointDistance: 0.008,
    leftContactDistance: -0.0016,
    rightContactDistance: -0.0012,
  }).ok, true);
  assert.equal(step4.evaluateAssemblyStep4HammerHandover({
    leftContact: true,
    rightContact: true,
    aperture: 0.0535,
    receiverGraspPointDistance: 0.008,
    leftContactDistance: -0.0002,
    rightContactDistance: -0.0002,
  }).code, 'receiver-not-closed');
});

test('Step 4 exposes finite four-arm waypoint contracts for distinct roles', () => {
  assert.equal(typeof step4.createAssemblyStep4Machine, 'function');
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.minimumToolAperture, 0.018);
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.maximumToolAperture, 0.052);
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.maximumContactPenetration, 0.002);
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.contactComparisonEpsilon, 0.0003);
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.maximumHammerGraspPointDistance, 0.016);
  assert.equal(step4.ASSEMBLY1_STEP4_DURATIONS.verificationWindow, 0.5);
  assert.equal(step4.ASSEMBLY1_STEP4_DURATIONS.dualClamp, 4);
  assert.equal(step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease, 2);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.frame, 130);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.donorEntry, 130);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.tool, 80);
  // The curved source mesh supplies the contact surface; the commanded
  // aperture must produce normal force on that grip, without added ribs.
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.receiverTool, 50);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.pregrasp, 80);
  // Grasp the 30 mm head rather than squeezing the 14 mm shaft out of its
  // passive fixture.
  assert.ok(step4.ASSEMBLY1_STEP4_GRIPPERS.fastener >= 60);
  assert.ok(step4.ASSEMBLY1_STEP4_GRIPPERS.fastener <= 90);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.fastener, 80);
  assert.equal(step4.ASSEMBLY1_STEP4_ARMS.length, 4);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_ARMS.map(({ key, role }) => [key, role]), [
    ['r0', 'frame hold'],
    ['r1', 'hammer handover donor'],
    ['r2', 'fastener pickup and insertion'],
    ['r3', 'hammer handover receiver and strike'],
  ]);
  for (const arm of step4.ASSEMBLY1_STEP4_ARMS) {
    for (const target of Object.values(arm.jointTargets)) {
      assert.equal(target.length, 7);
      assert.ok(target.every(Number.isFinite));
    }
  }
  const [, donor, fastener, receiver] = step4.ASSEMBLY1_STEP4_ARMS;
  assert.equal(donor.closingAxisYawDegrees, 90);
  assert.notDeepEqual(donor.jointTargets.hold, donor.jointTargets.clear);
  assert.notDeepEqual(fastener.jointTargets.prepare, fastener.jointTargets.engage);
  assert.notDeepEqual(fastener.jointTargets.engage, fastener.jointTargets.insert);
  assert.equal(fastener.liftPathJointTargets.length, 8);
  assert.ok(fastener.liftPathJointTargets.flat().every(Number.isFinite));
  assert.notDeepEqual(receiver.jointTargets.hold, receiver.jointTargets.engage);
  assert.notDeepEqual(receiver.jointTargets.ready, receiver.jointTargets.strike);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r1.prepare, [0.48, -0.28, 0.56]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r1.engage, [0.088, 0.051, 0.36]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.prepare, [0.095, 0.368, 0.34]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.engage, [0.095, 0.368, 0.195]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.liftPath, [
    0.215, 0.235, 0.26, 0.285, 0.315, 0.345, 0.3725, 0.40,
  ].map((z) => [0.095, 0.368, z]));
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.transfer, [-0.12, 0.215, 0.43]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.insert, [-0.12, 0.215, 0.385]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.prepare, [0.001, 0.022, 0.46]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.engage, [0.02, 0.034, 0.38]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.clear, [-0.08, 0.02, 0.421]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.ready, [-0.33, 0.215, 0.41]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.strike, [-0.33, 0.215, 0.35]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_HANDOVER_TCP_QUATERNIONS, {
    r1: [-0.058574, 0.954457, 0.054171, -0.287482],
    r3: [0.957426, 0.019017, 0.284744, 0.043533],
  });
});

test('Step 4 Arm 4 leaves its Step 3 home pose without a greater-than-180-degree joint sweep', () => {
  const receiver = step4.ASSEMBLY1_STEP4_ARMS[3];
  assert.deepEqual(receiver.jointTargets.hold, FRANKA_HOME.slice(0, 7));
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.prepare, [0.001, 0.022, 0.46]);
  assert.equal(receiver.closingAxisYawDegrees, -90);
  assert.ok(receiver.jointTargets.prepare.every(
    (target, joint) => Math.abs(target - receiver.jointTargets.hold[joint]) < Math.PI,
  ));
});

test('Step 4 receives the hammer while picking the fastener, then inserts and strikes', () => {
  let machine = step4.createAssemblyStep4Machine();
  assert.equal(machine.phase, 'donor-tighten');
  const advance = (duration, expected, evidence = valid) => {
    machine = step4.advanceAssemblyStep4Machine(machine, duration, evidence);
    assert.equal(machine.phase, expected);
  };
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.donorTighten, 'prepare');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.prepare, 'engage');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.engage, 'engage-settle');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.engageSettle, 'receiver-align');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.receiverAlign, 'dual-clamp');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.dualClamp, 'handover-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.verificationWindow, 'hammer-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease, 'donor-clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.donorClear, 'receiver-retreat');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.receiverRetreat, 'fastener-tighten');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerTighten, 'fastener-grip-settle');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerGripSettle, 'fastener-grasp-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerVerificationWindow, 'lift');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.lift, 'transfer');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.transfer, 'transfer-settle');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.transferSettle, 'insert');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.insert, 'fastener-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerRelease, 'clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.clear, 'placement-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.placementHold, 'support-approach');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.supportApproach, 'support-clamp');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.supportClamp, 'hammer-stage');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerStage, 'hammer-strike');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerStrike, 'hammer-recover');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerRecover, 'support-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.supportRelease, 'support-clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.supportClear, 'hammer-return');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerReturn, 'hammer-lower');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerLower, 'tool-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.toolRelease, 'tool-clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.toolClear, 'return-home');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.returnHome, 'complete');
});

test('Step 4 stops if the fastener grasp is lost during transfer', () => {
  const failure = step4.advanceAssemblyStep4Machine({
    phase: 'transfer',
    phaseElapsed: 0.2,
    continuousValidSeconds: 0,
    failure: null,
  }, 0.01, {
    ...valid,
    fastenerGrasp: { ok: false, code: 'fastener-grasp-lost', armKey: 'r2' },
  });
  assert.equal(failure.phase, 'error');
  assert.deepEqual(failure.failure, { code: 'fastener-grasp-lost', armKey: 'r2' });
});

test('Step 4 completes hammer release before requiring the fastener grasp', () => {
  const hammerOnly = {
    ...valid,
    fastenerGrasp: { ok: false, code: 'missing-finger-contact', armKey: 'r2' },
  };
  let machine = {
    phase: 'handover-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    lastInvalidVerdict: null,
    failure: null,
  };
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.verificationWindow,
    hammerOnly,
  );
  assert.equal(machine.phase, 'hammer-release');
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease,
    hammerOnly,
  );
  assert.equal(machine.phase, 'donor-clear');
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.donorClear,
    hammerOnly,
  );
  assert.equal(machine.phase, 'receiver-retreat');
  machine = step4.advanceAssemblyStep4Machine(machine, step4.ASSEMBLY1_STEP4_DURATIONS.receiverRetreat, hammerOnly);
  assert.equal(machine.phase, 'fastener-tighten');
});

test('Step 4 still requires the fastener grasp before lifting it', () => {
  const failure = step4.advanceAssemblyStep4Machine({
    phase: 'lift',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  }, 0.01, {
    ...valid,
    fastenerGrasp: { ok: false, code: 'missing-finger-contact', armKey: 'r2' },
  });
  assert.equal(failure.phase, 'error');
  assert.deepEqual(failure.failure, { code: 'missing-finger-contact', armKey: 'r2' });
});

test('Step 4 holds the pickup pose until a sustained physical fastener grasp is verified', () => {
  const noFastener = {
    ...valid,
    fastenerGrasp: { ok: false, code: 'missing-finger-contact', armKey: 'r2' },
    fastenerCurrentGrasp: { ok: false, code: 'missing-finger-contact', armKey: 'r2' },
  };
  let machine = {
    phase: 'fastener-grip-settle',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    lastInvalidVerdict: null,
    failure: null,
  };
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.fastenerGripSettle,
    noFastener,
  );
  assert.equal(machine.phase, 'fastener-grasp-verification');
  machine = step4.advanceAssemblyStep4Machine(machine, 0.2, noFastener);
  assert.equal(machine.phase, 'fastener-grasp-verification');
  assert.equal(machine.continuousValidSeconds, 0);
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.fastenerVerificationWindow,
    valid,
  );
  assert.equal(machine.phase, 'lift');
});

test('Step 4 does not accept cached fastener contact as a current bilateral grasp', () => {
  const machine = step4.advanceAssemblyStep4Machine({
    phase: 'fastener-grasp-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    lastInvalidVerdict: null,
    failure: null,
  }, step4.ASSEMBLY1_STEP4_DURATIONS.fastenerVerificationWindow, {
    ...valid,
    fastenerGrasp: { ok: true },
    fastenerCurrentGrasp: { ok: false, code: 'missing-right-contact', armKey: 'r2' },
  });
  assert.equal(machine.phase, 'fastener-grasp-verification');
  assert.equal(machine.continuousValidSeconds, 0);
});

test('Step 4 reports the physical fastener grasp reason after verification timeout', () => {
  const failure = step4.advanceAssemblyStep4Machine({
    phase: 'fastener-grasp-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    lastInvalidVerdict: null,
    failure: null,
  }, step4.ASSEMBLY1_STEP4_DURATIONS.fastenerVerificationTimeout, {
    ...valid,
    fastenerGrasp: { ok: false, code: 'missing-right-contact', armKey: 'r2' },
    fastenerCurrentGrasp: { ok: false, code: 'missing-right-contact', armKey: 'r2' },
  });
  assert.equal(failure.phase, 'error');
  assert.deepEqual(failure.failure, { code: 'missing-right-contact', armKey: 'r2' });
});

test('Step 4 placement verification tolerates settling before a continuous valid hold', () => {
  let machine = {
    phase: 'placement-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  };
  machine = step4.advanceAssemblyStep4Machine(machine, 0.25, {
    ...valid,
    placement: { ok: false, code: 'fastener-height', detail: 'settling' },
  });
  assert.equal(machine.phase, 'placement-verification');
  machine = step4.advanceAssemblyStep4Machine(
    machine,
    step4.ASSEMBLY1_STEP4_DURATIONS.placementHold,
    valid,
  );
  assert.equal(machine.phase, 'support-approach');
});

test('Step 4 placement timeout retains the physical failure reason', () => {
  const failure = step4.advanceAssemblyStep4Machine({
    phase: 'placement-verification',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  }, step4.ASSEMBLY1_STEP4_DURATIONS.placementTimeout, {
    ...valid,
    placement: { ok: false, code: 'fastener-misalignment', detail: '0.02' },
  });
  assert.equal(failure.phase, 'error');
  assert.deepEqual(failure.failure, { code: 'fastener-misalignment', detail: '0.02' });
});

test('Step 4 rejects a tipped or displaced beam as well as frame drift', () => {
  assert.equal(step4.evaluateAssemblyStep4Stability({
    frameTranslation: 0.002,
    crossMemberTranslation: 0.08,
    crossMemberRotationDegrees: 30,
  }).code, 'cross-member-disturbed');
  assert.equal(step4.evaluateAssemblyStep4Stability({
    frameTranslation: 0.02,
    crossMemberTranslation: 0,
    crossMemberRotationDegrees: 0,
  }).code, 'frame-drift');
});

test('Step 4 completes a physical hammer handover before descending onto the fastener', () => {
  const plans = [0, 1, 2, 3].map((index) => ({
    armKey: `r${index}`,
    hold: Array(7).fill(index * 10),
    prepare: Array(7).fill(index * 10 + 1),
    engage: Array(7).fill(index * 10 + 2),
    lift: Array(7).fill(index * 10 + 3),
    liftPath: index === 2
      ? [Array(7).fill(22), Array(7).fill(30), Array(7).fill(23)]
      : undefined,
    transfer: Array(7).fill(index * 10 + 4),
    insert: Array(7).fill(index * 10 + 5),
    clear: Array(7).fill(index * 10 + 6),
    ready: Array(7).fill(index * 10 + 7),
    strike: Array(7).fill(index * 10 + 8),
  }));
  const donorTighten = step4.createAssemblyStep4ControlFrame({
    phase: 'donor-tighten',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.donorTighten / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(
    donorTighten.arms.map(({ gripperTarget }) => gripperTarget),
    [130, 105, 255, 255],
  );

  const clamp = step4.createAssemblyStep4ControlFrame({
    phase: 'dual-clamp',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.dualClamp / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(clamp.arms.map(({ gripperTarget }) => gripperTarget), [
    130,
    step4.ASSEMBLY1_STEP4_GRIPPERS.tool,
    255,
    (step4.ASSEMBLY1_STEP4_GRIPPERS.open
      + step4.ASSEMBLY1_STEP4_GRIPPERS.receiverTool) / 2,
  ]);
  assert.deepEqual(clamp.arms[2].jointTargets, Array(7).fill(21));

  const donorRelease = step4.createAssemblyStep4ControlFrame({
    phase: 'hammer-release',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(donorRelease.arms.map(({ gripperTarget }) => gripperTarget), [
    130,
    (step4.ASSEMBLY1_STEP4_GRIPPERS.tool + step4.ASSEMBLY1_STEP4_GRIPPERS.open) / 2,
    255,
    step4.ASSEMBLY1_STEP4_GRIPPERS.receiverTool,
  ]);
  assert.deepEqual(donorRelease.arms[2].jointTargets, Array(7).fill(21));

  const donorClear = step4.createAssemblyStep4ControlFrame({
    phase: 'donor-clear',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.donorClear / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.ok(donorClear.arms[1].jointTargets[0] > 12);
  assert.ok(donorClear.arms[1].jointTargets[0] < 16);
  assert.deepEqual(donorClear.arms[2].jointTargets, Array(7).fill(21));
  assert.deepEqual(donorClear.arms[3].jointTargets, Array(7).fill(32));

  const fastenerApproach = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-tighten',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerTighten * 0.5,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.equal(fastenerApproach.arms[2].gripperTarget, 255);
  assert.ok(fastenerApproach.arms[2].jointTargets[0] > 21);
  assert.ok(fastenerApproach.arms[2].jointTargets[0] < 22);
  assert.deepEqual(fastenerApproach.arms[3].jointTargets, FRANKA_HOME.slice(0,7));

  const fastenerClamp = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-tighten',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerTighten * 0.85,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(fastenerClamp.arms[2].jointTargets, Array(7).fill(22));
  assert.ok(fastenerClamp.arms[2].gripperTarget < 255);
  assert.ok(fastenerClamp.arms[2].gripperTarget > 70);

  const fastenerSettle = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-grip-settle',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerGripSettle / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(fastenerSettle.arms[2].jointTargets, Array(7).fill(22));
  assert.equal(
    fastenerSettle.arms[2].gripperTarget,
    step4.ASSEMBLY1_STEP4_GRIPPERS.fastener,
  );

  const fastenerVerification = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-grasp-verification',
    phaseElapsed: 0.2,
    continuousValidSeconds: 0.1,
    failure: null,
  }, plans);
  assert.deepEqual(fastenerVerification.arms[2].jointTargets, Array(7).fill(22));
  assert.equal(
    fastenerVerification.arms[2].gripperTarget,
    step4.ASSEMBLY1_STEP4_GRIPPERS.fastener,
  );

  const fastenerLift = step4.createAssemblyStep4ControlFrame({
    phase: 'lift',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.lift / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(fastenerLift.arms[2].jointTargets, Array(7).fill(30));
  assert.deepEqual(fastenerLift.arms[3].jointTargets, FRANKA_HOME.slice(0,7));

  const hammerStage = step4.createAssemblyStep4ControlFrame({
    phase: 'hammer-stage',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.hammerStage / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.ok(hammerStage.arms[3].jointTargets[0] > 34);
  assert.ok(hammerStage.arms[3].jointTargets[0] < 35);

  const release = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-release',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerRelease / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(release.arms.map(({ gripperTarget }) => gripperTarget), [
    step4.ASSEMBLY1_STEP4_GRIPPERS.frame,
    255,
    (step4.ASSEMBLY1_STEP4_GRIPPERS.fastener + step4.ASSEMBLY1_STEP4_GRIPPERS.open) / 2,
    step4.ASSEMBLY1_STEP4_GRIPPERS.receiverTool,
  ]);

  const complete = step4.createAssemblyStep4ControlFrame({
    phase: 'complete',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(complete.arms.map(({ gripperTarget }) => gripperTarget), [
    255,
    255,
    255,
    255,
  ]);
  for (const arm of complete.arms) assert.deepEqual(arm.jointTargets, FRANKA_HOME.slice(0,7));
});
