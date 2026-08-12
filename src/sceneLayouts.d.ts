import type { SceneObject, XmlPatch } from 'mujoco-react';

export interface SceneLayout {
  instanceCount: number;
  yawStepDegrees: number;
  homeJoints: number[];
  xmlPatches: XmlPatch[];
  sceneObjects: SceneObject[];
  camera: { position: [number, number, number]; fov: number };
  orbitTarget: [number, number, number];
  ringRadius?: number;
  spacing?: number;
  workSurfaceHeight?: number;
  armBaseHeight?: number;
  tableTopHeight?: number;
  tableObjects?: SceneObject[];
  primaryTcpSite?: string;
  primaryGripperActuator?: string;
}

export function repeatPose(pose: number[], count: number): number[];

export const FRANKA_LAYOUT: SceneLayout & {
  ringRadius: number;
  primaryTcpSite: string;
  primaryGripperActuator: string;
};

export const SO101_LAYOUT: SceneLayout & {
  ringRadius: number;
  workSurfaceHeight: number;
  primaryTcpSite: string;
};

export const XLEROBOT_LAYOUT: SceneLayout & {
  spacing: number;
  armBaseHeight: number;
  tableTopHeight: number;
  tableObjects: SceneObject[];
};
