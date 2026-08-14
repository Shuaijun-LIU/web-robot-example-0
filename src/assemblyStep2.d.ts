export type AssemblyStep2Phase =
  | 'idle'
  | 'planning'
  | 'approach'
  | 'slow-descent'
  | 'contact-settle'
  | 'frame-clamp'
  | 'frame-verification'
  | 'cross-member-clamp'
  | 'cross-member-verification'
  | 'torque-driver-clamp'
  | 'tool-verification'
  | 'clamped-hold'
  | 'complete'
  | 'error';

export type AssemblyStep2FailureCode =
  | 'missing-left-contact'
  | 'missing-right-contact'
  | 'forbidden-contact'
  | 'empty-closure'
  | 'object-drift'
  | 'object-rotation'
  | 'object-lift'
  | 'verification-timeout'
  | 'invalid-precondition'
  | 'missing-resource'
  | 'joint-limit'
  | 'non-finite-runtime';

export interface AssemblyStep2Failure {
  code: AssemblyStep2FailureCode;
  armKey?: string;
  detail?: string;
}

export type AssemblyStep2GraspVerdict =
  | { ok: true }
  | ({ ok: false } & AssemblyStep2Failure);

export interface AssemblyStep2Evidence {
  frame?: AssemblyStep2GraspVerdict;
  crossMember?: AssemblyStep2GraspVerdict;
  tool?: AssemblyStep2GraspVerdict;
  all?: AssemblyStep2GraspVerdict;
}

export interface AssemblyStep2Machine {
  phase: Exclude<AssemblyStep2Phase, 'idle' | 'planning'>;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep2Failure | null;
}

export interface AssemblyStep2Arm {
  key: string;
  label: string;
  role: string;
  targetBody: string;
  baselinePosition: [number, number, number];
  currentPosition: [number, number, number];
  tcpPosition: [number, number, number];
  contactWaypoint: readonly [number, number, number];
  approachWaypoint: readonly [number, number, number];
  closingAxisYawDegrees: number;
  tcpQuaternion: readonly [number, number, number, number];
  siteName: string;
  jointNames: readonly string[];
  fingerJointNames: readonly [string, string];
  leftFingerBody: string;
  rightFingerBody: string;
  actuatorIndices: readonly number[];
  gripperActuatorIndex: number;
  approachJointTargets: readonly number[];
  contactJointTargets: readonly number[];
}

export interface AssemblyStep2ArmPlan {
  armKey: string;
  start: number[];
  approach: readonly number[];
  contact: readonly number[];
}

export interface AssemblyStep2ControlFrame {
  arms: Array<{
    armKey: string;
    jointTargets: readonly number[];
    gripperTarget: number;
  }>;
}

export interface AssemblyPose {
  position: [number, number, number];
  quaternion: [number, number, number, number];
}

export interface AssemblyStep1CompletionSnapshot {
  tcpPositions: Record<string, [number, number, number]>;
  tcpOrientations: Record<string, number[]>;
  objectPoses: Record<string, AssemblyPose>;
  gripperControls: [number, number, number, number];
}

export interface AssemblyStep2State {
  phase: AssemblyStep2Phase;
  failure: AssemblyStep2Failure | null;
}

export interface AssemblyStep2ArmDiagnostics {
  armKey: string;
  targetBody: string;
  leftContactBodies: string[];
  rightContactBodies: string[];
  aperture: number;
  translation: number;
  rotationDegrees: number;
  verticalDisplacement: number;
  currentContactSeconds: number;
  maximumContactSeconds: number;
  gripperControl: number;
  closureStartedAt: number | null;
  verdict: AssemblyStep2GraspVerdict;
}

export interface AssemblyStep2RuntimeDiagnostics {
  phase: AssemblyStep2Phase;
  phaseElapsed: number;
  continuousValidSeconds: number;
  failure: AssemblyStep2Failure | null;
  simulationTime: number;
  arms: AssemblyStep2ArmDiagnostics[];
}

export const ASSEMBLY1_STEP2_DURATIONS: Readonly<{
  approach: 1.4;
  slowDescent: 0.8;
  contactSettle: 1.5;
  frameClamp: 0.8;
  crossMemberClamp: 1;
  torqueDriverClamp: 0.8;
  contactWindow: 0.25;
  verificationTimeout: 2.5;
  stableHold: 2;
}>;

export const ASSEMBLY1_STEP2_GRIPPER_CLAMPS: readonly [48, 96, 0, 0];

export const ASSEMBLY1_STEP2_LIMITS: Readonly<{
  tcpPosition: 0.06;
  tcpOrientationDegrees: 8;
  preStepObjectDrift: 0.003;
  objectTranslation: 0.005;
  objectRotationDegrees: 5;
  verticalDisplacement: 0.003;
  minimumAperture: 0.02;
}>;

export const ASSEMBLY1_STEP2_ARMS: AssemblyStep2Arm[];

export function interpolateAssemblyStep2Gripper(
  from: number,
  to: number,
  progress: number,
): number;

export function releaseAssemblyStep2Controls(
  controls: Float64Array,
  positions: Float64Array,
  arms: Array<{
    actuatorIndices: readonly number[];
    qposAddresses: readonly number[];
    gripperActuatorIndex: number;
  }>,
): void;

export function captureAssemblyStep2JointTargets(
  positions: Float64Array,
  arms: Array<{ qposAddresses: number[] }>,
): number[][];

export function createAssemblyStep2ControlFrame(
  machine: AssemblyStep2Machine,
  plans: AssemblyStep2ArmPlan[],
): AssemblyStep2ControlFrame;

export function quaternionAngularDistanceDegrees(
  first: readonly number[],
  second: readonly number[],
): number;

export function evaluateAssemblyStep2Grasp(input: {
  targetBody: string;
  leftContactBodies: string[];
  rightContactBodies: string[];
  forbiddenBodies: string[];
  aperture: number;
  translation: number;
  rotationDegrees: number;
  verticalDisplacement: number;
}): AssemblyStep2GraspVerdict;

export function createAssemblyStep2Machine(): AssemblyStep2Machine;

export function advanceAssemblyStep2Machine(
  machine: AssemblyStep2Machine,
  deltaSeconds: number,
  evidence: AssemblyStep2Evidence,
): AssemblyStep2Machine;
