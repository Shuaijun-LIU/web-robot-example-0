import { useArmController } from './useArmController';
import type { ArmControllerConfig } from './useArmController';
import type { IkContextValue } from 'mujoco-react';
import { XLEROBOT_HOME_JOINTS } from '../configs';

const config: ArmControllerConfig = {
  numActuators: 16,
  base: {
    indices: [0, 1],
    keys: ['KeyW', 'KeyS', 'KeyA', 'KeyD'],
    speed: 1,
  },
  arms: [
    {
      indices: [2, 3, 4, 5, 6, 7],
      keys: ['Digit7', 'KeyY', 'Digit9', 'KeyI', 'Digit8', 'KeyU', 'Digit0', 'KeyO', 'Minus', 'KeyP', 'KeyV'],
      initialJoints: XLEROBOT_HOME_JOINTS.slice(2, 8),
    },
    {
      indices: [8, 9, 10, 11, 12, 13],
      keys: ['KeyH', 'KeyN', 'KeyK', 'Comma', 'KeyJ', 'KeyM', 'KeyL', 'Period', 'Semicolon', 'Slash', 'KeyB'],
      initialJoints: XLEROBOT_HOME_JOINTS.slice(8, 14),
    },
  ],
  head: {
    indices: [14, 15],
    keys: ['KeyR', 'KeyT', 'KeyF', 'KeyG'],
  },
};

export function XLeRobotController({ ik }: { ik?: IkContextValue | null }) {
  useArmController(config, ik);
  return null;
}
