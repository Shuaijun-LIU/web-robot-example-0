const QUARTER_TURN = Math.PI / 2;
const HALF_TURN = Math.PI;

const cube = (name, position, rgba, size = [0.025, 0.025, 0.025]) => ({
  name,
  type: 'box',
  size,
  position,
  rgba,
  mass: 0.05,
  freejoint: true,
  friction: '1.5 0.3 0.1',
  solref: '0.01 1',
  solimp: '0.95 0.99 0.001 0.5 2',
  condim: 4,
});

const fixedBox = (name, size, position, rgba) => ({
  name,
  type: 'box',
  size,
  position,
  rgba,
});

export function repeatPose(pose, count) {
  return Array.from({ length: count }, () => [...pose]).flat();
}

const FRANKA_HOME = [1.707, -1.754, 0.003, -2.702, 0.003, 0.951, 2.49, 0];

export const FRANKA_LAYOUT = {
  instanceCount: 4,
  yawStepDegrees: 90,
  ringRadius: 0.72,
  primaryTcpSite: 'tcp_0',
  primaryGripperActuator: 'gripper_0',
  homeJoints: repeatPose(FRANKA_HOME, 4),
  xmlPatches: [
    {
      target: 'panda.xml',
      replace: ['name="actuator8"', 'name="gripper"'],
    },
    {
      target: 'panda.xml',
      inject:
        '<site name="tcp" pos="0 0 0.1" size="0.01" rgba="0.75 0.18 0.12 0.7" group="1"/>',
      injectAfter: '<body name="hand"',
    },
    {
      target: 'panda.xml',
      replace: [
        '<body name="link0" childclass="panda">',
        `<replicate count="4" euler="0 0 ${QUARTER_TURN}" sep="_"><frame pos="0 -0.72 0"><body name="link0" childclass="panda">`,
      ],
    },
    {
      target: 'panda.xml',
      replace: [
        '\n  </worldbody>',
        '\n      </frame>\n    </replicate>\n  </worldbody>',
      ],
    },
    {
      target: 'panda.xml',
      replace: [
        '  <keyframe>\n    <key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853 0.04 0.04" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853 255"/>\n  </keyframe>\n\n',
        '',
      ],
    },
  ],
  sceneObjects: [
    cube('red_cube', [-0.07, 0, 0.025], [0.72, 0.18, 0.14, 1]),
    cube('green_cube', [0.07, 0, 0.025], [0.18, 0.56, 0.28, 1]),
    cube('blue_cube', [0, 0.08, 0.025], [0.18, 0.35, 0.66, 1]),
  ],
  camera: { position: [1.75, -1.75, 2.2], fov: 45 },
  orbitTarget: [0, 0, 0.42],
};

const SO101_HOME = [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745];
const SO101_TABLE_TOP = 0.8;
const SO101_TABLE_THICKNESS = 0.07;
const SO101_TABLE_UNDERSIDE = SO101_TABLE_TOP - SO101_TABLE_THICKNESS;

export const SO101_LAYOUT = {
  instanceCount: 4,
  yawStepDegrees: 90,
  ringRadius: 0.34,
  workSurfaceHeight: SO101_TABLE_TOP,
  primaryTcpSite: 'tcp_0',
  homeJoints: repeatPose(SO101_HOME, 4),
  xmlPatches: [
    {
      target: 'SO101.xml',
      inject:
        '<site name="tcp" pos="0 -0.04 -0.01" size="0.005" rgba="0.16 0.55 0.26 0.7" group="1"/>',
      injectAfter: '<body name="Fixed_Jaw"',
    },
    {
      target: 'SO101.xml',
      replace: [
        '<body name="Base" pos="0.35 -0.3 0.8" euler="0 0 0">',
        `<replicate count="4" euler="0 0 ${QUARTER_TURN}" sep="_"><frame pos="0 0.34 0.8"><body name="Base" pos="0 0 0" euler="0 0 0">`,
      ],
    },
    {
      target: 'SO101.xml',
      replace: [
        '\n  </worldbody>',
        '\n      </frame>\n    </replicate>\n  </worldbody>',
      ],
    },
    {
      target: 'SO101.xml',
      replace: [
        '  <keyframe>\n    <key name="home" qpos="0 0 0 0 0 -0.37453" ctrl="0 0 0 0 0 -0.37453"/>\n  </keyframe>\n',
        '',
      ],
    },
  ],
  sceneObjects: [
    fixedBox('floor', [2, 2, 0.005], [0, 0, -0.005], [0.12, 0.13, 0.15, 1]),
    fixedBox(
      'shared_work_surface',
      [0.52, 0.52, SO101_TABLE_THICKNESS / 2],
      [0, 0, SO101_TABLE_UNDERSIDE + SO101_TABLE_THICKNESS / 2],
      [0.34, 0.3, 0.25, 1],
    ),
    fixedBox('table_leg_a', [0.035, 0.035, SO101_TABLE_UNDERSIDE / 2], [-0.44, -0.44, SO101_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
    fixedBox('table_leg_b', [0.035, 0.035, SO101_TABLE_UNDERSIDE / 2], [0.44, -0.44, SO101_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
    fixedBox('table_leg_c', [0.035, 0.035, SO101_TABLE_UNDERSIDE / 2], [-0.44, 0.44, SO101_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
    fixedBox('table_leg_d', [0.035, 0.035, SO101_TABLE_UNDERSIDE / 2], [0.44, 0.44, SO101_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
    cube('red_cube', [-0.055, 0, 0.815], [0.72, 0.18, 0.14, 1], [0.015, 0.015, 0.015]),
    cube('blue_cube', [0.055, 0, 0.815], [0.18, 0.35, 0.66, 1], [0.015, 0.015, 0.015]),
    cube('green_cube', [0, 0.06, 0.815], [0.18, 0.56, 0.28, 1], [0.015, 0.015, 0.015]),
  ],
  camera: { position: [1.25, -1.25, 1.65], fov: 45 },
  orbitTarget: [0, 0, 0.78],
};

const XLEROBOT_HOME = [
  0,
  0,
  1.5708,
  1.5785,
  1.5777,
  0.0008,
  1.57,
  -0.25,
  -1.5708,
  1.5785,
  1.5777,
  0.0008,
  1.57,
  -0.25,
  0,
  0,
];
const XLEROBOT_ARM_BASE_HEIGHT = 0.775;
const XLEROBOT_TABLE_HALF_HEIGHT = 0.025;
const XLEROBOT_TABLE_UNDERSIDE = XLEROBOT_ARM_BASE_HEIGHT - 2 * XLEROBOT_TABLE_HALF_HEIGHT;

const xlerobotTableObjects = [
  fixedBox(
    'table_top',
    [0.46, 0.34, XLEROBOT_TABLE_HALF_HEIGHT],
    [0, 0, XLEROBOT_ARM_BASE_HEIGHT - XLEROBOT_TABLE_HALF_HEIGHT],
    [0.34, 0.3, 0.25, 1],
  ),
  fixedBox('table_leg_a', [0.035, 0.035, XLEROBOT_TABLE_UNDERSIDE / 2], [-0.37, -0.27, XLEROBOT_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
  fixedBox('table_leg_b', [0.035, 0.035, XLEROBOT_TABLE_UNDERSIDE / 2], [0.37, -0.27, XLEROBOT_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
  fixedBox('table_leg_c', [0.035, 0.035, XLEROBOT_TABLE_UNDERSIDE / 2], [-0.37, 0.27, XLEROBOT_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
  fixedBox('table_leg_d', [0.035, 0.035, XLEROBOT_TABLE_UNDERSIDE / 2], [0.37, 0.27, XLEROBOT_TABLE_UNDERSIDE / 2], [0.2, 0.19, 0.18, 1]),
];

export const XLEROBOT_LAYOUT = {
  instanceCount: 2,
  yawStepDegrees: 180,
  spacing: 1.7,
  armBaseHeight: XLEROBOT_ARM_BASE_HEIGHT,
  tableTopHeight: XLEROBOT_ARM_BASE_HEIGHT,
  homeJoints: repeatPose(XLEROBOT_HOME, 2),
  xmlPatches: [
    {
      target: 'xlerobot.xml',
      replace: [
        '<body name="chassis" pos="0 0 0.38">',
        `<replicate count="2" euler="0 0 ${HALF_TURN}" sep="_"><frame pos="0.85 0 0"><body name="chassis" pos="0 0 0.38">`,
      ],
    },
    {
      target: 'xlerobot.xml',
      replace: [
        '\n  </worldbody>',
        '\n      </frame>\n    </replicate>\n  </worldbody>',
      ],
    },
  ],
  tableObjects: xlerobotTableObjects,
  sceneObjects: [
    fixedBox('floor', [3, 3, 0.005], [0, 0, -0.005], [0.12, 0.13, 0.15, 1]),
    ...xlerobotTableObjects,
  ],
  camera: { position: [2.0, -2.25, 1.65], fov: 45 },
  orbitTarget: [0, 0, 0.62],
};
