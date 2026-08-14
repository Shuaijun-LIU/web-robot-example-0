export interface NamedActuator {
  name: string;
  min: number;
  max: number;
}

export type UnitreeActionPhaseName =
  | 'settle'
  | 'rise-greet'
  | 'scan-wave'
  | 'lower'
  | 'recover'
  | 'final-hold'
  | 'complete';

export interface UnitreeActionSample {
  phase: UnitreeActionPhaseName;
  elapsed: number;
  g1Targets: number[];
  go2Targets: number[];
}

export interface UnitreeActuatorIds {
  g1: number[];
  go2: number[];
}

export const G1_ACTUATORS: NamedActuator[];
export const GO2_ACTUATORS: NamedActuator[];
export const G1_HOME: number[];
export const GO2_HOME: number[];
export const GO2_STAND: number[];
export const GO2_LOWER: number[];
export const UNITREE_ACTION_PHASES: Array<{
  name: Exclude<UnitreeActionPhaseName, 'complete'>;
  duration: number;
}>;
export const UNITREE_ACTION_DURATION: number;

export function sampleUnitreeAction(elapsedSeconds: number): UnitreeActionSample;
export function applyUnitreeActionTargets(
  ctrl: Float64Array | number[],
  actuatorIds: UnitreeActuatorIds,
  sample: UnitreeActionSample,
): void;
