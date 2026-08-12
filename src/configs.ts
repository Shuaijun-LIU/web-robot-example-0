import type { SceneConfig } from 'mujoco-react';

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

export const XLEROBOT_HOME_JOINTS = [
  0, 0,
  1.5708, 1.5785, 1.5777, 0.0008, 1.57, -0.25,
  -1.5708, 1.5785, 1.5777, 0.0008, 1.57, -0.25,
  0, 0,
];

export const robots: Record<string, RobotEntry> = {
  franka: {
    label: 'Franka Panda',
    config: {
      src: 'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/',
      sceneFile: 'scene.xml',
      homeJoints: [1.707, -1.754, 0.003, -2.702, 0.003, 0.951, 2.49],
      xmlPatches: [
        {
          target: 'panda.xml',
          replace: ['name="actuator8"', 'name="gripper"'],
          inject:
            '<site name="tcp" pos="0 0 0.1" size="0.01" rgba="1 0 0 0.5" group="1"/>',
          injectAfter: '<body name="hand"',
        },
      ],
      sceneObjects: [
        {
          name: 'red_cube',
          type: 'box',
          size: [0.025, 0.025, 0.025],
          position: [0.0, 0.4, 0.025],
          rgba: [0.9, 0.2, 0.15, 1],
          mass: 0.05,
          freejoint: true,
          friction: '1.5 0.3 0.1',
          solref: '0.01 1',
          solimp: '0.95 0.99 0.001 0.5 2',
          condim: 4,
        },
        {
          name: 'green_cube',
          type: 'box',
          size: [0.025, 0.025, 0.025],
          position: [-0.1, 0.35, 0.025],
          rgba: [0.15, 0.8, 0.3, 1],
          mass: 0.05,
          freejoint: true,
          friction: '1.5 0.3 0.1',
          solref: '0.01 1',
          solimp: '0.95 0.99 0.001 0.5 2',
          condim: 4,
        },
        {
          name: 'blue_cube',
          type: 'box',
          size: [0.025, 0.025, 0.025],
          position: [0.1, 0.45, 0.025],
          rgba: [0.15, 0.4, 0.9, 1],
          mass: 0.05,
          freejoint: true,
          friction: '1.5 0.3 0.1',
          solref: '0.01 1',
          solimp: '0.95 0.99 0.001 0.5 2',
          condim: 4,
        },
      ],
    },
    camera: { position: [2, -1.5, 2.5], fov: 45 },
    orbitTarget: [0, 0, 0.4],
    hasIk: true,
    ikConfig: { siteName: 'tcp', numJoints: 7 },
  },

  so101: {
    label: 'SO101',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'SO101.xml',
      homeJoints: [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745],
      xmlPatches: [
        {
          target: 'SO101.xml',
          inject:
            '<site name="tcp" pos="0 -0.04 -0.01" size="0.005" rgba="0 1 0 0.5" group="1"/>',
          injectAfter: '<body name="Fixed_Jaw"',
        },
      ],
      sceneObjects: [
        {
          name: 'floor',
          type: 'box',
          size: [2, 2, 0.005],
          position: [0, 0, -0.005],
          rgba: [0.15, 0.15, 0.2, 1],
        },
        {
          name: 'table',
          type: 'box',
          size: [0.4, 0.4, 0.4],
          position: [0.35, -0.3, 0.4],
          rgba: [0.35, 0.3, 0.28, 1],
        },
        {
          name: 'red_cube',
          type: 'box',
          size: [0.015, 0.015, 0.015],
          position: [0.3, -0.65, 0.815],
          rgba: [0.9, 0.2, 0.15, 1],
          mass: 0.02,
          freejoint: true,
          friction: '2 0.3 0.1',
          solref: '0.01 1',
          solimp: '0.95 0.99 0.001 0.5 2',
          condim: 4,
        },
        {
          name: 'blue_cube',
          type: 'box',
          size: [0.012, 0.012, 0.015],
          position: [0.38, -0.65, 0.815],
          rgba: [0.15, 0.4, 0.9, 1],
          mass: 0.02,
          freejoint: true,
          friction: '2 0.3 0.1',
          condim: 4,
        },
      ],
    },
    camera: { position: [1.2, -1.2, 1.6], fov: 45 },
    orbitTarget: [0.35, -0.3, 0.8],
    hasIk: true,
    ikConfig: { siteName: 'tcp', numJoints: 5 },
    gizmoScale: 0.08,
  },

  xlerobot: {
    label: 'XLeRobot',
    config: {
      src: XLEROBOT_BASE,
      sceneFile: 'xlerobot.xml',
      // 16 actuators: [forward, turn, L_rot, L_pitch, L_elbow, L_wristP, L_wristR, L_jaw,
      //                R_rot, R_pitch, R_elbow, R_wristP, R_wristR, R_jaw, head_pan, head_tilt]
      homeJoints: XLEROBOT_HOME_JOINTS,
      xmlPatches: [
        {
          target: 'xlerobot.xml',
          // Rotate root chassis by 180deg at startup so robot faces the camera.
          replace: ['<body name="chassis" pos="0 0 0.38"', '<body name="chassis" pos="0 0 0.38" euler="0 0 3.14159"'],
        },
      ],
      sceneObjects: [
        {
          name: 'floor',
          type: 'box',
          size: [5, 5, 0.005],
          position: [0, 0, -0.005],
          rgba: [0.15, 0.15, 0.2, 1],
        },
        {
          name: 'red_cube',
          type: 'box',
          size: [0.02, 0.02, 0.02],
          position: [0.25, 0, 0.42],
          rgba: [0.9, 0.2, 0.15, 1],
          mass: 0.03,
          freejoint: true,
          friction: '2 0.3 0.1',
          condim: 4,
        },
        {
          name: 'green_cube',
          type: 'box',
          size: [0.02, 0.02, 0.02],
          position: [0.22, -0.1, 0.42],
          rgba: [0.15, 0.8, 0.3, 1],
          mass: 0.03,
          freejoint: true,
          friction: '2 0.3 0.1',
          condim: 4,
        },
        {
          name: 'blue_cube',
          type: 'box',
          size: [0.02, 0.02, 0.02],
          position: [0.22, 0.1, 0.42],
          rgba: [0.15, 0.4, 0.9, 1],
          mass: 0.03,
          freejoint: true,
          friction: '2 0.3 0.1',
          condim: 4,
        },
        {
          name: 'yellow_sphere',
          type: 'sphere',
          size: [0.025, 0.025, 0.025],
          position: [0.28, 0.08, 0.42],
          rgba: [0.9, 0.8, 0.1, 1],
          mass: 0.02,
          freejoint: true,
          friction: '2 0.3 0.1',
          condim: 4,
        },
      ],
    },
    camera: { position: [1.5, -1.5, 1.2], fov: 45 },
    orbitTarget: [0.15, 0, 0.4],
    hasIk: false,
  },
};
