import { shiftIndices } from '../controlTargets.js';

const SO101_HOME = [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745];
const XLEROBOT_HOME = [
  0, 0,
  1.5708, 1.5785, 1.5777, 0.0008, 1.57, -0.25,
  -1.5708, 1.5785, 1.5777, 0.0008, 1.57, -0.25,
  0, 0,
];

export function getFrankaGripperBinding(target) {
  if (!target.gripperActuator) {
    throw new Error(`Control target ${target.key} has no Franka gripper actuator`);
  }
  return {
    v: { actuator: target.gripperActuator, toggle: [0, 255] },
  };
}

export function createSO101ControllerConfig(actuatorOffset) {
  return {
    numActuators: 24,
    arms: [{
      indices: shiftIndices([0, 1, 2, 3, 4, 5], actuatorOffset),
      keys: [
        'KeyD', 'KeyA', 'KeyW', 'KeyS', 'KeyQ', 'KeyE',
        'KeyR', 'KeyF', 'KeyZ', 'KeyC', 'KeyV',
      ],
      initialJoints: [...SO101_HOME],
    }],
  };
}

export function createXLeRobotControllerConfig(actuatorOffset) {
  return {
    numActuators: 32,
    base: {
      indices: shiftIndices([0, 1], actuatorOffset),
      keys: ['KeyW', 'KeyS', 'KeyA', 'KeyD'],
      speed: 1,
    },
    arms: [
      {
        indices: shiftIndices([2, 3, 4, 5, 6, 7], actuatorOffset),
        keys: [
          'Digit7', 'KeyY', 'Digit9', 'KeyI', 'Digit8', 'KeyU',
          'Digit0', 'KeyO', 'Minus', 'KeyP', 'KeyV',
        ],
        initialJoints: XLEROBOT_HOME.slice(2, 8),
      },
      {
        indices: shiftIndices([8, 9, 10, 11, 12, 13], actuatorOffset),
        keys: [
          'KeyH', 'KeyN', 'KeyK', 'Comma', 'KeyJ', 'KeyM',
          'KeyL', 'Period', 'Semicolon', 'Slash', 'KeyB',
        ],
        initialJoints: XLEROBOT_HOME.slice(8, 14),
      },
    ],
    head: {
      indices: shiftIndices([14, 15], actuatorOffset),
      keys: ['KeyR', 'KeyT', 'KeyF', 'KeyG'],
    },
  };
}
