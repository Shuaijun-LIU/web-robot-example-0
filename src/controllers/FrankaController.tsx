import { useKeyboardTeleop } from 'mujoco-react';

import { FRANKA_LAYOUT } from '../sceneLayouts.js';

/** Franka gripper toggle — V key opens/closes the gripper. */
export function FrankaController() {
  useKeyboardTeleop({
    bindings: {
      v: { actuator: FRANKA_LAYOUT.primaryGripperActuator, toggle: [0, 255] },
    },
  });
  return null;
}
