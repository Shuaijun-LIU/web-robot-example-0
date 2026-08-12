import type { SceneConfig } from 'mujoco-react';

import {
  FRANKA_LAYOUT,
  SO101_LAYOUT,
  XLEROBOT_LAYOUT,
} from './sceneLayouts.js';
import {
  FRANKA_ASSEMBLY1_LAYOUT,
  FRANKA_ASSEMBLY2_LAYOUT,
} from './frankaAssemblyLayouts.js';
import {
  createFrankaTargets,
  createSO101Targets,
  createXLeRobotTargets,
} from './controlTargets.js';
import type { ControlTarget } from './controlTargets.js';

export interface RobotEntry {
  label: string;
  controlFamily: 'franka' | 'so101' | 'xlerobot';
  config: SceneConfig;
  camera: { position: [number, number, number]; fov: number };
  orbitTarget: [number, number, number];
  gizmoScale?: number;
  controlTargets: ControlTarget[];
}

const XLEROBOT_BASE =
  'https://raw.githubusercontent.com/Vector-Wangel/MuJoCo-GS-Web/main/assets/robots/xlerobot/';
const FRANKA_REMOTE_BASE =
  'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/';
const FRANKA_ASSEMBLY2_BASE = `${import.meta.env.BASE_URL}assets/franka-assembly2/`;

export const XLEROBOT_HOME_JOINTS = XLEROBOT_LAYOUT.homeJoints.slice(0, 16);

export const robots: Record<string, RobotEntry> = {
  franka: {
    label: 'Franka Panda',
    controlFamily: 'franka',
    config: {
      src: FRANKA_REMOTE_BASE,
      sceneFile: 'scene.xml',
      homeJoints: FRANKA_LAYOUT.homeJoints,
      xmlPatches: FRANKA_LAYOUT.xmlPatches,
      sceneObjects: FRANKA_LAYOUT.sceneObjects,
    },
    camera: FRANKA_LAYOUT.camera,
    orbitTarget: FRANKA_LAYOUT.orbitTarget,
    controlTargets: createFrankaTargets(),
  },

  so101: {
    label: 'SO101',
    controlFamily: 'so101',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'objects_SO101.xml',
      homeJoints: SO101_LAYOUT.homeJoints,
      xmlPatches: SO101_LAYOUT.xmlPatches,
      sceneObjects: SO101_LAYOUT.sceneObjects,
    },
    camera: SO101_LAYOUT.camera,
    orbitTarget: SO101_LAYOUT.orbitTarget,
    gizmoScale: 0.08,
    controlTargets: createSO101Targets(),
  },

  xlerobot: {
    label: 'XLeRobot',
    controlFamily: 'xlerobot',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'objects.xml',
      homeJoints: XLEROBOT_LAYOUT.homeJoints,
      xmlPatches: XLEROBOT_LAYOUT.xmlPatches,
      sceneObjects: XLEROBOT_LAYOUT.sceneObjects,
    },
    camera: XLEROBOT_LAYOUT.camera,
    orbitTarget: XLEROBOT_LAYOUT.orbitTarget,
    controlTargets: createXLeRobotTargets(),
  },

  frankaAssembly1: {
    label: 'Franka Assembly1',
    controlFamily: 'franka',
    config: {
      src: FRANKA_REMOTE_BASE,
      sceneFile: 'scene.xml',
      homeJoints: FRANKA_ASSEMBLY1_LAYOUT.homeJoints,
      xmlPatches: FRANKA_ASSEMBLY1_LAYOUT.xmlPatches,
      sceneObjects: FRANKA_ASSEMBLY1_LAYOUT.sceneObjects,
    },
    camera: FRANKA_ASSEMBLY1_LAYOUT.camera,
    orbitTarget: FRANKA_ASSEMBLY1_LAYOUT.orbitTarget,
    controlTargets: createFrankaTargets(),
  },

  frankaAssembly2: {
    label: 'Franka Assembly2',
    controlFamily: 'franka',
    config: {
      src: FRANKA_ASSEMBLY2_BASE,
      sceneFile: 'scene.xml',
      homeJoints: FRANKA_ASSEMBLY2_LAYOUT.homeJoints,
      xmlPatches: FRANKA_ASSEMBLY2_LAYOUT.xmlPatches,
      sceneObjects: FRANKA_ASSEMBLY2_LAYOUT.sceneObjects,
    },
    camera: FRANKA_ASSEMBLY2_LAYOUT.camera,
    orbitTarget: FRANKA_ASSEMBLY2_LAYOUT.orbitTarget,
    controlTargets: createFrankaTargets(),
  },
};
