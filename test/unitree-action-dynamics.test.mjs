import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import test from 'node:test';

const repoRoot = new URL('..', import.meta.url).pathname;

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
