import {
  createAssembly1SceneObjects,
  SHARED_ASSEMBLY1_ASSET_XML,
  SHARED_ASSEMBLY1_TOOL_XML,
  SHARED_ASSEMBLY1_WORKCELL_XML,
} from './frankaAssemblyLayouts.js';
import { repeatPose } from './sceneLayouts.js';

const PLATFORM_TOP = 0.1;
const TASK_STATIONS = {
  frame: [0, 0, 0.275],
  parts: [-0.56, 0.42, 0.125],
  poweredTool: [0.53, -0.42, 0.135],
  manualTool: [-0.53, -0.42, 0.13],
  hammer: [0.65, 0, 0.229],
  fasteners: [0.56, 0.42, 0.125],
  handover: [0, -0.48, 0.112],
};

export const PIPER_HOME = [0, 1.57, -1.3485, 0, 0, 0, 0.035];
export const UR5E_HOME = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0, 0];

function attachmentFrames(model, body, radius) {
  const poses = [
    { position: [0, -radius, PLATFORM_TOP], yaw: 0 },
    { position: [radius, 0, PLATFORM_TOP], yaw: 90 },
    { position: [0, radius, PLATFORM_TOP], yaw: 180 },
    { position: [-radius, 0, PLATFORM_TOP], yaw: -90 },
  ];

  return poses.map(({ position, yaw }, index) => {
    const euler = yaw === 0 ? '' : ` euler="0 0 ${yaw}"`;
    return `<frame pos="${position.join(' ')}"${euler}><attach model="${model}" body="${body}" prefix="r${index}_"/></frame>`;
  }).join('');
}

function createPatches({ model, body, radius }) {
  return [
    {
      target: 'scene.xml',
      replace: [
        '  </asset>',
        `    ${SHARED_ASSEMBLY1_ASSET_XML}\n  </asset>`,
      ],
    },
    {
      target: 'scene.xml',
      replace: [
        '  <worldbody>',
        `  <worldbody>${attachmentFrames(model, body, radius)}`,
      ],
    },
    {
      target: 'scene.xml',
      replace: [
        '  </worldbody>',
        `${SHARED_ASSEMBLY1_WORKCELL_XML}${SHARED_ASSEMBLY1_TOOL_XML}\n  </worldbody>`,
      ],
    },
  ];
}

function createAssemblyLayout({
  model,
  body,
  radius,
  home,
  tcp,
  gripper,
  camera,
}) {
  return {
    instanceCount: 4,
    yawStepDegrees: 90,
    ringRadius: radius,
    workSurfaceHeight: PLATFORM_TOP,
    primaryTcpSite: `r0_${tcp}`,
    primaryGripperActuator: `r0_${gripper}`,
    homeJoints: repeatPose(home, 4),
    taskStations: { ...TASK_STATIONS },
    xmlPatches: createPatches({ model, body, radius }),
    sceneObjects: createAssembly1SceneObjects(true),
    camera: { position: camera, fov: 45 },
    orbitTarget: [0, 0, 0.32],
  };
}

export const PIPER_ASSEMBLY1_LAYOUT = createAssemblyLayout({
  model: 'piper_model',
  body: 'base_link',
  radius: 0.78,
  home: PIPER_HOME,
  tcp: 'tcp',
  gripper: 'gripper',
  camera: [2.6, -2.6, 2.75],
});

export const UR5E_ASSEMBLY1_LAYOUT = createAssemblyLayout({
  model: 'ur5e_model',
  body: 'base',
  radius: 0.9,
  home: UR5E_HOME,
  tcp: 'gripper_pinch',
  gripper: 'gripper_fingers_actuator',
  camera: [3, -3, 3.15],
});
