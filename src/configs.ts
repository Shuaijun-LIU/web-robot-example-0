import type { SceneConfig } from 'mujoco-react';

import { FRANKA_LAYOUT, SO101_LAYOUT, XLEROBOT_LAYOUT } from './sceneLayouts.js';

export interface RobotEntry {
  label: string;
  config: SceneConfig;
  camera: { position: [number, number, number]; fov: number };
  orbitTarget: [number, number, number];
  hasIk: boolean;
  ikConfig?: { siteName: string; numJoints: number };
  gizmoScale?: number;
}

const XLEROBOT_BASE =
  'https://raw.githubusercontent.com/Vector-Wangel/MuJoCo-GS-Web/main/assets/robots/xlerobot/';

// The XLeRobot controller intentionally operates the first physical robot.
export const XLEROBOT_HOME_JOINTS = XLEROBOT_LAYOUT.homeJoints.slice(0, 16);

export const robots: Record<string, RobotEntry> = {
  franka: {
    label: 'Franka Panda · 4 arms',
    config: {
      src: 'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/',
      sceneFile: 'scene.xml',
      homeJoints: FRANKA_LAYOUT.homeJoints,
      xmlPatches: FRANKA_LAYOUT.xmlPatches,
      sceneObjects: FRANKA_LAYOUT.sceneObjects,
    },
    camera: FRANKA_LAYOUT.camera,
    orbitTarget: FRANKA_LAYOUT.orbitTarget,
    hasIk: true,
    ikConfig: { siteName: FRANKA_LAYOUT.primaryTcpSite, numJoints: 7 },
  },

  so101: {
    label: 'SO101 · 4 arms',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'objects_SO101.xml',
      homeJoints: SO101_LAYOUT.homeJoints,
      xmlPatches: SO101_LAYOUT.xmlPatches,
      sceneObjects: SO101_LAYOUT.sceneObjects,
    },
    camera: SO101_LAYOUT.camera,
    orbitTarget: SO101_LAYOUT.orbitTarget,
    hasIk: true,
    ikConfig: { siteName: SO101_LAYOUT.primaryTcpSite, numJoints: 5 },
    gizmoScale: 0.08,
  },

  xlerobot: {
    label: 'XLeRobot · 2 robots',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'objects.xml',
      homeJoints: XLEROBOT_LAYOUT.homeJoints,
      xmlPatches: XLEROBOT_LAYOUT.xmlPatches,
      sceneObjects: XLEROBOT_LAYOUT.sceneObjects,
    },
    camera: XLEROBOT_LAYOUT.camera,
    orbitTarget: XLEROBOT_LAYOUT.orbitTarget,
    hasIk: false,
  },
};
