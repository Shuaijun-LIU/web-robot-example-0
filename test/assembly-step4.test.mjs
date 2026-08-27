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
  placement: { ok: true },
};

test('Step 4 exposes finite four-arm waypoint contracts for distinct roles', () => {
  assert.equal(typeof step4.createAssemblyStep4Machine, 'function');
  assert.equal(step4.ASSEMBLY1_STEP4_ARMS.length, 4);
  assert.deepEqual(step4.ASSEMBLY1_STEP4_ARMS.map(({ key, role }) => [key, role]), [
    ['r0', 'frame hold'],
    ['r1', 'hammer pickup and strike'],
    ['r2', 'fastener pickup and insertion'],
    ['r3', 'cross-member standby'],
  ]);
  for (const arm of step4.ASSEMBLY1_STEP4_ARMS) {
    for (const target of Object.values(arm.jointTargets)) {
      assert.equal(target.length, 7);
      assert.ok(target.every(Number.isFinite));
    }
  }
  const [, tool, fastener, support] = step4.ASSEMBLY1_STEP4_ARMS;
  assert.notDeepEqual(tool.jointTargets.hold, tool.jointTargets.ready);
  assert.notDeepEqual(tool.jointTargets.ready, tool.jointTargets.strike);
  assert.notDeepEqual(fastener.jointTargets.prepare, fastener.jointTargets.engage);
  assert.notDeepEqual(fastener.jointTargets.engage, fastener.jointTargets.insert);
  assert.deepEqual(support.jointTargets.hold, support.jointTargets.engage);
});

test('Step 4 orders physical insertion followed by a hammer strike and recovery', () => {
  let machine = step4.createAssemblyStep4Machine();
  assert.equal(machine.phase, 'prepare');
  const advance = (duration, expected, evidence = valid) => {
    machine = step4.advanceAssemblyStep4Machine(machine, duration, evidence);
    assert.equal(machine.phase, expected);
  };
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.prepare, 'engage');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.engage, 'engage-settle');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.engageSettle, 'fastener-clamp');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.fastenerClamp, 'fastener-verification');
  advance(step4.ASSEMBLY1_STEP4_DURATIONS.verificationWindow, 'fastener-tighten');
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

test('Step 4 control frames clamp only Arm 3 and finish with both transport arms open', () => {
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
  const clamp = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-clamp',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerClamp / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(clamp.arms.map(({ gripperTarget }) => gripperTarget), [48, 96, 182.5, 255]);

  const release = step4.createAssemblyStep4ControlFrame({
    phase: 'fastener-release',
    phaseElapsed: step4.ASSEMBLY1_STEP4_DURATIONS.fastenerRelease / 2,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(release.arms.map(({ gripperTarget }) => gripperTarget), [48, 96, 162.5, 255]);

  const complete = step4.createAssemblyStep4ControlFrame({
    phase: 'complete',
    phaseElapsed: 0,
    continuousValidSeconds: 0,
    failure: null,
  }, plans);
  assert.deepEqual(complete.arms.map(({ gripperTarget }) => gripperTarget), [48, 96, 255, 255]);
  assert.deepEqual(complete.arms[1].jointTargets, Array(7).fill(17));
  assert.deepEqual(complete.arms[2].jointTargets, Array(7).fill(26));
});
