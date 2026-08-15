import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import test from 'node:test';

const repoRoot = new URL('..', import.meta.url).pathname;

test('runtime action controller never assigns kinematic or force state', () => {
  const source = readFileSync(new URL('../src/UnitreeActionController.tsx', import.meta.url), 'utf8');
  const forbiddenAssignment = /data\.(?:qpos|qvel|xpos|xquat|mocap_pos|mocap_quat|xfrc_applied)\s*(?:\[[^\]]+\])?\s*=/;
  assert.equal(forbiddenAssignment.test(source), false);
  assert.match(source, /data\.ctrl/);
});

test('the actuator-only clip completes under MuJoCo gravity and contact', () => {
  const output = execFileSync(
    process.execPath,
    ['scripts/verify-unitree-action-dynamics.mjs'],
    { cwd: repoRoot, encoding: 'utf8', maxBuffer: 10 * 1024 * 1024 },
  );
  const result = JSON.parse(output.trim().split('\n').at(-1));

  assert.equal(result.completed, true);
  assert.equal(result.finiteState, true);
  assert.deepEqual(result.visitedPhases, [
    'settle',
    'rise-greet',
    'scan-wave',
    'lower',
    'recover',
    'final-hold',
    'complete',
  ]);
  assert.ok(result.g1.maxJointDelta > 0.25);
  assert.ok(result.go2.maxLegJointDelta > 0.25);
  assert.ok(result.go2.maxArmJointDelta > 0.25);
  assert.ok(result.g1.finalHeight >= 0.75 && result.g1.finalHeight <= 0.85);
  assert.ok(result.g1.finalTiltDegrees <= 5);
  assert.ok(result.go2.finalHeight >= 0.22 && result.go2.finalHeight <= 0.34);
  assert.ok(result.go2.finalTiltDegrees <= 10);
  assert.ok(result.g1.groundContactSteps > 0);
  assert.ok(result.go2.groundContactSteps > 0);
  assert.equal(result.runtimeWrites, 'ctrl-only');
});

test('the locomotion suite produces stable free-root displacement', () => {
  const output = execFileSync(
    process.execPath,
    ['scripts/simulate-unitree-locomotion.mjs', '--control-period=0.016'],
    { cwd: repoRoot, encoding: 'utf8', maxBuffer: 10 * 1024 * 1024 },
  );
  const result = JSON.parse(output.trim().split('\n').at(-1));

  assert.equal(result.completed, true);
  assert.equal(result.finiteState, true);
  assert.equal(result.safety.safe, true);
  assert.equal(result.runtimeWrites, 'ctrl-only');
  assert.deepEqual(result.visitedPhases, [
    'settle',
    'g1-squat',
    'g1-stand',
    'g1-walk',
    'g1-stabilize',
    'go2-walk',
    'go2-stabilize',
    'final-greeting',
    'final-hold',
    'complete',
  ]);
  assert.ok(result.g1.forwardDisplacement >= 0.07);
  assert.ok(result.go2.forwardDisplacement >= 0.4);
  assert.ok(result.g1.finalHeight >= 0.7 && result.g1.finalHeight <= 0.9);
  assert.ok(result.g1.finalTiltDegrees <= 12);
  assert.ok(result.go2.finalHeight >= 0.2 && result.go2.finalHeight <= 0.36);
  assert.ok(result.go2.finalTiltDegrees <= 15);
  assert.ok(result.g1.groundContactSteps > 0);
  assert.ok(result.go2.groundContactSteps > 0);
  assert.equal(result.clampCount, 0);
});
