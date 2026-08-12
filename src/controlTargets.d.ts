export interface IkTargetDefinition {
  siteName: string;
  jointNames: string[];
  actuatorIndices: number[];
}

export interface ControlTarget {
  key: string;
  label: string;
  prefix: string;
  actuatorOffset: number;
  gripperActuator?: string;
  ik?: IkTargetDefinition;
}

export function shiftIndices(indices: number[], offset: number): number[];
export function createFrankaTargets(): ControlTarget[];
export function createSO101Targets(): ControlTarget[];
export function createXLeRobotTargets(): ControlTarget[];
