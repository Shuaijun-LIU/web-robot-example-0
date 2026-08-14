import assert from 'node:assert/strict';
import test from 'node:test';

import {
  advanceAction,
  completeAction,
  createInitialUnitreeActionState,
  failAction,
  pauseAction,
  resetAction,
  resumeAction,
  startAction,
} from '../src/unitreeActionState.js';

test('Unitree action state starts, advances monotonically, and completes at ten seconds', () => {
  const idle = createInitialUnitreeActionState();
  assert.deepEqual(idle, { status: 'idle', phase: 'settle', elapsed: 0, error: null });

  const running = startAction(idle);
  assert.equal(running.status, 'running');
  const scan = advanceAction(running, 4);
  assert.deepEqual({ status: scan.status, phase: scan.phase, elapsed: scan.elapsed }, {
    status: 'running', phase: 'scan-wave', elapsed: 4,
  });
  assert.equal(advanceAction(scan, -1).elapsed, 4);

  const complete = advanceAction(scan, 20);
  assert.deepEqual(complete, { status: 'complete', phase: 'complete', elapsed: 10, error: null });
  assert.deepEqual(completeAction(scan), complete);
});

test('pause and resume preserve elapsed action time', () => {
  const running = advanceAction(startAction(createInitialUnitreeActionState()), 2.25);
  const paused = pauseAction(running);
  assert.equal(paused.status, 'paused');
  assert.equal(advanceAction(paused, 2).elapsed, 2.25);
  assert.deepEqual(resumeAction(paused), { ...paused, status: 'running' });
});

test('reset returns to idle and failure keeps the original message', () => {
  const failed = failAction(startAction(createInitialUnitreeActionState()), new Error('missing actuator'));
  assert.deepEqual(failed, {
    status: 'error', phase: 'settle', elapsed: 0, error: 'missing actuator',
  });
  assert.equal(failAction(failed, new Error('replacement')).error, 'missing actuator');
  assert.deepEqual(resetAction(failed), createInitialUnitreeActionState());
});
