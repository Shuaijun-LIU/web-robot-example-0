export type AssemblyStep3Phase =
  | 'idle'
  | 'planning'
  | 'grasp-check'
  | 'lift'
  | 'lift-settle'
  | 'transfer-a'
  | 'transfer-b'
  | 'hover-settle'
  | 'aligned-descent'
  | 'alignment-verification'
  | 'aligned-hold'
  | 'complete'
  | 'error';

export type AssemblyStep3FailureCode =
  | 'missing-left-contact'
  | 'missing-right-contact'
  | 'forbidden-contact'
  | 'empty-closure'
  | 'frame-drift'
  | 'cross-member-rotation'
  | 'hole-misalignment'
  | 'verification-timeout'
  | 'invalid-precondition'
  | 'missing-resource'
  | 'joint-limit'
  | 'non-finite-runtime';

export interface AssemblyStep3Failure {
  code: AssemblyStep3FailureCode;
  armKey?: string;
  detail?: string;
}

export type AssemblyStep3Verdict = { ok: true } | ({ ok: false } & AssemblyStep3Failure);

export interface AssemblyStep3Evidence {
  all?: AssemblyStep3Verdict;
  alignment?: AssemblyStep3Verdict;
}

export interface AssemblyStep3Machine {
  phase: Exclude<AssemblyStep3Phase, 'idle' | 'planning'>;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep3Failure | null;
}

export interface AssemblyStep3ArmPlan {
  armKey: string;
  hold: readonly number[];
  lift: readonly number[];
  transferA: readonly number[];
  transferMid: readonly number[];
  hover: readonly number[];
  descentMid: readonly number[];
  aligned: readonly number[];
}

export interface AssemblyStep3State {
  phase: AssemblyStep3Phase;
  failure: AssemblyStep3Failure | null;
}

export interface AssemblyStep3ArmDiagnostics {
  armKey: string;
  targetBody: string;
  leftContactBodies: string[];
  rightContactBodies: string[];
  aperture: number;
  gripperControl: number;
  verdict: AssemblyStep3Verdict;
}

export interface AssemblyStep3RuntimeDiagnostics {
  phase: AssemblyStep3Phase;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep3Failure | null;
  simulationTime: number;
  crossMemberPosition: [number, number, number];
  crossMemberQuaternion: [number, number, number, number];
  frameTranslation: number;
  crossMemberRotationDegrees: number;
  holeDistances: number[];
  arms: AssemblyStep3ArmDiagnostics[];
}

export const ASSEMBLY1_STEP3_DURATIONS: Readonly<{
  graspCheckWindow: 0.25;
  verificationTimeout: 2.5;
  lift: 3;
  transferA: 4.5;
  transferB: 4.5;
  alignedDescent: 3;
  alignedHold: 1;
}>;

export const ASSEMBLY1_STEP3_GRIPPER_CLAMPS: readonly [48, 96, 24, 24];

export const ASSEMBLY1_STEP3_LIMITS: Readonly<{
  minimumAperture: 0.02;
  frameTranslation: 0.008;
  crossMemberRotationDegrees: 5;
  holeDistance: 0.008;
}>;

export const ASSEMBLY1_STEP3_WAYPOINTS: Readonly<Record<
  'start' | 'lift' | 'transferA' | 'transferMid' | 'hover' | 'descentMid' | 'aligned',
  readonly [readonly [number, number, number], readonly [number, number, number]]
>>;

export const ASSEMBLY1_STEP3_TRANSPORT_ARMS: ReadonlyArray<Readonly<{
  key: 'r2' | 'r3';
  armIndex: 2 | 3;
  closingAxisYawDegrees: number;
  liftJointTargets: readonly number[];
  transferAJointTargets: readonly number[];
  transferMidJointTargets: readonly number[];
  hoverJointTargets: readonly number[];
  descentMidJointTargets: readonly number[];
  alignedJointTargets: readonly number[];
}>>;

export function evaluateAssemblyStep3Transport(input: {
  targetBody: string;
  leftContactBodies: string[];
  rightContactBodies: string[];
  forbiddenBodies: string[];
  aperture: number;
  requireBilateralContact?: boolean;
}): AssemblyStep3Verdict;

export function evaluateAssemblyStep3Alignment(input: {
  holeDistances: number[];
  frameTranslation: number;
  crossMemberRotationDegrees: number;
}): AssemblyStep3Verdict;

export function createAssemblyStep3Machine(): AssemblyStep3Machine;

export function advanceAssemblyStep3Machine(
  machine: AssemblyStep3Machine,
  deltaSeconds: number,
  evidence: AssemblyStep3Evidence,
): AssemblyStep3Machine;

export function createAssemblyStep3ControlFrame(
  machine: AssemblyStep3Machine,
  plans: AssemblyStep3ArmPlan[],
): {
  arms: Array<{
    armKey: string;
    jointTargets: readonly number[];
    gripperTarget: number;
  }>;
};

export function holdAssemblyStep3Controls(
  controls: Float64Array,
  positions: Float64Array,
  arms: Array<{
    actuatorIndices: readonly number[];
    qposAddresses: readonly number[];
    gripperActuatorIndex: number;
  }>,
): void;
