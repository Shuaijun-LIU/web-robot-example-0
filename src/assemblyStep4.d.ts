export type AssemblyStep4Phase =
  | 'idle' | 'planning' | 'prepare' | 'engage' | 'engage-settle' | 'fastener-clamp'
  | 'fastener-verification' | 'fastener-tighten' | 'lift' | 'transfer' | 'transfer-settle' | 'insert'
  | 'fastener-release' | 'clear' | 'placement-verification' | 'tool-stage'
  | 'complete' | 'error';

export interface AssemblyStep4Failure {
  code: string;
  armKey?: string;
  detail?: string;
}

export type AssemblyStep4Verdict = { ok: true } | ({ ok: false } & AssemblyStep4Failure);

export interface AssemblyStep4Machine {
  phase: Exclude<AssemblyStep4Phase, 'idle' | 'planning'>;
  phaseElapsed: number;
  continuousValidSeconds: number;
  lastInvalidVerdict?: AssemblyStep4Verdict | null;
  failure: AssemblyStep4Failure | null;
}

export interface AssemblyStep4State {
  phase: AssemblyStep4Phase;
  failure: AssemblyStep4Failure | null;
}

export interface AssemblyStep4ArmPlan {
  armKey: string;
  hold: readonly number[];
  prepare: readonly number[];
  engage: readonly number[];
  lift: readonly number[];
  transfer: readonly number[];
  insert: readonly number[];
  clear: readonly number[];
  ready: readonly number[];
}

export interface AssemblyStep4RuntimeDiagnostics {
  phase: AssemblyStep4Phase;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep4Failure | null;
  simulationTime: number;
  frameTranslation: number;
  crossMemberTranslation: number;
  crossMemberRotationDegrees: number;
  fastenerPosition: [number, number, number];
  fastenerPlanarDistance: number;
  fastenerHeight: number;
  fastenerAperture: number;
  fastenerLeftContact: boolean;
  fastenerRightContact: boolean;
}

export const ASSEMBLY1_STEP4_DURATIONS: Readonly<Record<string, number>>;
export const ASSEMBLY1_STEP4_LIMITS: Readonly<Record<string, number>>;
export const ASSEMBLY1_STEP4_GRIPPERS: Readonly<Record<string, number>>;
export const ASSEMBLY1_STEP4_WAYPOINTS: Readonly<Record<string, Readonly<Record<string, readonly number[]>>>>;
export const ASSEMBLY1_STEP4_ARMS: ReadonlyArray<Readonly<{
  key: string;
  armIndex: number;
  role: string;
  jointTargets: Readonly<Record<string, readonly number[]>>;
}>>;

export function createAssemblyStep4Machine(): AssemblyStep4Machine;
export function advanceAssemblyStep4Machine(
  machine: AssemblyStep4Machine,
  deltaSeconds: number,
  evidence: { all?: AssemblyStep4Verdict; fastenerGrasp?: AssemblyStep4Verdict; placement?: AssemblyStep4Verdict },
): AssemblyStep4Machine;
export function createAssemblyStep4ControlFrame(
  machine: AssemblyStep4Machine,
  plans: AssemblyStep4ArmPlan[],
): { arms: Array<{ armKey: string; jointTargets: readonly number[]; gripperTarget: number }> };
export function evaluateAssemblyStep4Stability(input: {
  frameTranslation: number;
  crossMemberTranslation: number;
  crossMemberRotationDegrees: number;
}): AssemblyStep4Verdict;
export function evaluateAssemblyStep4Placement(input: {
  fastenerPlanarDistance: number;
  fastenerHeight: number;
}): AssemblyStep4Verdict;
