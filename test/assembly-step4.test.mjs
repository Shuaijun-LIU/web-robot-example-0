import assert from 'node:assert/strict';
import test from 'node:test';

let step4 = {};
try {
  step4 = await import('../src/assemblyStep4.js');
} catch {
  // The first RED run intentionally reaches the assertions with no Step 4 module.
}

const valid = {
  all: { ok: true },
  fastenerGrasp: { ok: true },
  hammerGrasp: { ok: true },
  placement: { ok: true },
};

test('Step 4 exposes finite four-arm waypoint contracts for distinct roles', () => {
  assert.equal(typeof step4.createAssemblyStep4Machine, 'function');
  assert.equal(step4.ASSEMBLY1_STEP4_LIMITS.minimumToolAperture, 0.012);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.donorEntry, 96);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.tool, 96);
  assert.equal(step4.ASSEMBLY1_STEP4_GRIPPERS.pregrasp, 80);
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
  assert.notDeepEqual(receiver.jointTargets.hold, receiver.jointTargets.engage);
  assert.notDeepEqual(receiver.jointTargets.ready, receiver.jointTargets.strike);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r1.prepare, [0.30, -0.18, 0.45]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r1.engage, [0.095, 0, 0.40]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r2.engage, [0.115, 0.39, 0.16]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.prepare, [-0.015, 0.02, 0.44]);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_WAYPOINTS.r3.engage, [-0.015, 0.02, 0.40]);
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
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.engageSettle, 'dual-clamp');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.dualClamp, 'handover-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.verificationWindow, 'hammer-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease, 'donor-clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.donorClear, 'fastener-tighten');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerTighten, 'lift');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.lift, 'transfer');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.transfer, 'transfer-settle');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.transferSettle, 'insert');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.insert, 'fastener-release');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerRelease, 'clear');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.clear, 'placement-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.placementHold, 'hammer-stage');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerStage, 'hammer-strike');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerStrike, 'hammer-recover');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.hammerRecover, 'complete');
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
  assert.equal(machine.phase, 'hammer-stage');
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

test('Step 4 stability protects the held frame while treating free beam settling as diagnostic', () => {
  assert.deepEqual(step4.evaluateAssemblyStep4Stability({
    frameTranslation: 0.002,
    crossMemberTranslation: 0.08,
    crossMemberRotationDegrees: 30,
  }), { ok: true });
  assert.equal(step4.evaluateAssemblyStep4Stability({
    frameTranslation: 0.02,
    crossMemberTranslation: 0,
    crossMemberRotationDegrees: 0,
  }).code, 'frame-drift');
});

test('Step 4 closes receiver and fastener grippers together before donor release', () => {
  const plans = [0, 1, 2, 3].map((index) => ({
    armKey: `r${index}`,
    hold: Array(7).fill(index * 10),
    prepare: Array(7).fill(index * 10 + 1),
    engage: Array(7).fill(index * 10 + 2),
    lift: Array(7).fill(index * 10 + 3),
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
    [48, 96, 255, 255],
  );

  const clamp = step4.createAssemblyStep4ControlFrame({
    phase: 'dual-clamp',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.dualClamp / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(clamp.arms.map(({ gripperTarget }) => gripperTarget), [48, 96, 167.5, 175.5]);

  const donorRelease = step4.createAssemblyStep4ControlFrame({
    phase: 'hammer-release',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.hammerRelease / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(donorRelease.arms.map(({ gripperTarget }) => gripperTarget), [48, 175.5, 80, 96]);

  const release = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-release',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerRelease / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(release.arms.map(({ gripperTarget }) => gripperTarget), [48, 255, 162.5, 96]);

  const complete = step4.createAssemblyStep4ControlFrame({
    phase: 'complete',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(complete.arms.map(({ gripperTarget }) => gripperTarget), [48, 255, 255, 96]);
  assert.deepEqual(complete.arms[1].jointTargets, Array(7).fill(16));
  assert.deepEqual(complete.arms[2].jointTargets, Array(7).fill(26));
  assert.deepEqual(complete.arms[3].jointTargets, Array(7).fill(37));
});
