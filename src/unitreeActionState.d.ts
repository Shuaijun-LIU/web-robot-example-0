import type { UnitreeActionPhaseName } from './unitreeActionSequence.js';

export type UnitreeActionStatus = 'idle' | 'running' | 'paused' | 'complete' | 'error';

export interface UnitreeActionState {
  status: UnitreeActionStatus;
  phase: UnitreeActionPhaseName;
  elapsed: number;
  error: string | null;
}

export function createInitialUnitreeActionState(): UnitreeActionState;
export function startAction(state: UnitreeActionState): UnitreeActionState;
export function pauseAction(state: UnitreeActionState): UnitreeActionState;
export function resumeAction(state: UnitreeActionState): UnitreeActionState;
export function advanceAction(state: UnitreeActionState, deltaSeconds: number): UnitreeActionState;
export function completeAction(state: UnitreeActionState): UnitreeActionState;
export function failAction(state: UnitreeActionState, error: unknown): UnitreeActionState;
export function resetAction(state?: UnitreeActionState): UnitreeActionState;
