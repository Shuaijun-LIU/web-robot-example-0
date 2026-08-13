import type { SceneObject } from 'mujoco-react';

import type { SceneLayout } from './sceneLayouts.js';

export const SO101_GEARBOX_LAYOUT: SceneLayout & {
  ringRadius: number;
  workSurfaceHeight: number;
  primaryTcpSite: string;
  taskStations: {
    fixture: [number, number, number];
    housing: [number, number, number];
    shaftsAndSpacers: [number, number, number];
    gears: [number, number, number];
    coverAndPins: [number, number, number];
  };
};

export const XLEROBOT_KITTING_LAYOUT: SceneLayout & {
  spacing: number;
  armBaseHeight: number;
  tableTopHeight: number;
  chassisCollisionTop: number;
  tableObjects: SceneObject[];
  taskStations: {
    sourceTote: [number, number, number];
    handoffSouth: [number, number, number];
    handoffNorth: [number, number, number];
    scannerDock: [number, number, number];
    orderTray: [number, number, number];
  };
};
