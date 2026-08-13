export type AssemblyStep1Status = 'idle' | 'planning' | 'running' | 'complete' | 'error';

export interface AssemblyStep1Arm {
  key: string;
  label: string;
  role: string;
  siteName: string;
  jointNames: string[];
  actuatorIndices: number[];
  gripperActuatorIndex: number;
  highWaypoint: [number, number, number];
  finalWaypoint: [number, number, number];
  highJointTargets: number[];
  finalJointTargets: number[];
}

export interface AssemblyStep1ArmPlan {
  armKey: string;
  start: number[];
  high: number[];
  final: number[];
}

export const ASSEMBLY1_STEP1_PHASE_DURATION: number;
export const ASSEMBLY1_GRIPPER_OPEN: number;
export const ASSEMBLY1_STEP1_ARMS: AssemblyStep1Arm[];

export function smoothstep01(value: number): number;
export function interpolateJointTargets(from: number[], to: number[], progress: number): number[];
export function selectAssemblyStep1Phase(elapsed: number): {
  phase: 'high' | 'final' | 'complete';
  progress: number;
};
export function isCompleteAssemblyStep1Plan(
  plans: Array<AssemblyStep1ArmPlan | null>,
): boolean;
