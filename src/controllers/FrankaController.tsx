import { useKeyboardTeleop } from 'mujoco-react';

import type { ControlTarget } from '../controlTargets.js';
import { getFrankaGripperBinding } from './controllerConfigs.js';

/** Franka gripper toggle — V key opens/closes the gripper. */
export function FrankaController({ target }: { target: ControlTarget }) {
  useKeyboardTeleop({
    bindings: getFrankaGripperBinding(target),
  });
  return null;
}
