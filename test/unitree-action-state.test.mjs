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
  assert.deepEqual(idle, {
    programId: 'greeting', status: 'idle', phase: 'settle', elapsed: 0, error: null,
  });

  const running = startAction(idle);
  assert.equal(running.status, 'running');
  const scan = advanceAction(running, 4);
  assert.deepEqual({ status: scan.status, phase: scan.phase, elapsed: scan.elapsed }, {
    status: 'running', phase: 'scan-wave', elapsed: 4,
  });
  assert.equal(advanceAction(scan, -1).elapsed, 4);

  const complete = advanceAction(scan, 20);
  assert.deepEqual(complete, {
    programId: 'greeting', status: 'complete', phase: 'complete', elapsed: 10, error: null,
  });
  assert.deepEqual(completeAction(scan), complete);
});

test('locomotion state uses its own phase clock and completes at 25 seconds', () => {
  const idle = createInitialUnitreeActionState('locomotion');
  assert.deepEqual(idle, {
    programId: 'locomotion', status: 'idle', phase: 'settle', elapsed: 0, error: null,
  });
  const walking = advanceAction(startAction(idle), 7);
  assert.deepEqual(walking, {
    programId: 'locomotion', status: 'running', phase: 'g1-walk', elapsed: 7, error: null,
  });
  const complete = advanceAction(walking, 100);
  assert.deepEqual(complete, {
    programId: 'locomotion', status: 'complete', phase: 'complete', elapsed: 25, error: null,
  });
  assert.deepEqual(resetAction(complete), createInitialUnitreeActionState('locomotion'));
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
    programId: 'greeting', status: 'error', phase: 'settle', elapsed: 0, error: 'missing actuator',
  });
  assert.equal(failAction(failed, new Error('replacement')).error, 'missing actuator');
  assert.deepEqual(resetAction(failed), createInitialUnitreeActionState());
});
