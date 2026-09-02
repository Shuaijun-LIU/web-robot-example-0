import assert from 'node:assert/strict';
import test from 'node:test';

const demoModule = await import('../src/frankaDemo1.js').catch(() => ({}));

const baseState = {
  active: true,
  sceneReady: true,
  step1: 'idle',
  step2: 'idle',
  step3: 'idle',
  step4: 'idle',
};

test('Franka Demo1 advances the existing four steps in order', () => {
  assert.equal(typeof demoModule.nextFrankaDemo1Action, 'function');
  assert.equal(demoModule.nextFrankaDemo1Action(baseState), 'step1');
  assert.equal(demoModule.nextFrankaDemo1Action({ ...baseState, step1: 'complete' }), 'step2');
  assert.equal(demoModule.nextFrankaDemo1Action({
    ...baseState,
    step1: 'complete',
    step2: 'complete',
  }), 'step3');
  assert.equal(demoModule.nextFrankaDemo1Action({
    ...baseState,
    step1: 'complete',
    step2: 'complete',
    step3: 'complete',
  }), 'step4');
  assert.equal(demoModule.nextFrankaDemo1Action({
    ...baseState,
    step1: 'complete',
    step2: 'complete',
    step3: 'complete',
    step4: 'complete',
  }), 'complete');
});

test('Franka Demo1 waits during motion and stops on an error', () => {
  assert.equal(demoModule.nextFrankaDemo1Action({ ...baseState, sceneReady: false }), null);
  assert.equal(demoModule.nextFrankaDemo1Action({ ...baseState, active: false }), null);
  assert.equal(demoModule.nextFrankaDemo1Action({ ...baseState, step1: 'running' }), null);
  assert.equal(demoModule.nextFrankaDemo1Action({
    ...baseState,
    step1: 'complete',
    step2: 'error',
  }), 'error');
});

test('Franka Demo1 reports the step currently in motion', () => {
  assert.equal(typeof demoModule.frankaDemo1DisplayAction, 'function');
  assert.equal(demoModule.frankaDemo1DisplayAction({ ...baseState, step1: 'running' }), 'step1');
  assert.equal(demoModule.frankaDemo1DisplayAction({
    ...baseState,
    step1: 'complete',
    step2: 'frame-clamp',
  }), 'step2');
  assert.equal(demoModule.frankaDemo1DisplayAction({ ...baseState, active: false }), null);
});
