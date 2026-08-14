import { UNITREE_ACTION_DURATION, sampleUnitreeAction } from './unitreeActionSequence.js';

export function createInitialUnitreeActionState() {
  return { status: 'idle', phase: 'settle', elapsed: 0, error: null };
}

export function startAction(state) {
  if (state.status !== 'idle' && state.status !== 'complete') return state;
  return { status: 'running', phase: 'settle', elapsed: 0, error: null };
}

export function pauseAction(state) {
  return state.status === 'running' ? { ...state, status: 'paused' } : state;
}

export function resumeAction(state) {
  return state.status === 'paused' ? { ...state, status: 'running' } : state;
}

export function completeAction(state) {
  if (state.status === 'error') return state;
  return { status: 'complete', phase: 'complete', elapsed: UNITREE_ACTION_DURATION, error: null };
}

export function advanceAction(state, deltaSeconds) {
  if (state.status !== 'running' || !Number.isFinite(deltaSeconds) || deltaSeconds <= 0) return state;
  const elapsed = Math.min(UNITREE_ACTION_DURATION, state.elapsed + deltaSeconds);
  if (elapsed >= UNITREE_ACTION_DURATION) return completeAction(state);
  return { ...state, phase: sampleUnitreeAction(elapsed).phase, elapsed };
}

export function failAction(state, error) {
  if (state.status === 'error') return state;
  const message = error instanceof Error ? error.message : String(error);
  return { ...state, status: 'error', error: message };
}

export function resetAction() {
  return createInitialUnitreeActionState();
}
