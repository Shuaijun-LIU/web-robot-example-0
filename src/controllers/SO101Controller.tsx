import { useArmController } from './useArmController';
import type { ArmControllerConfig } from './useArmController';
import type { IkContextValue } from 'mujoco-react';

const config: ArmControllerConfig = {
  numActuators: 6,
  arms: [{
    indices: [0, 1, 2, 3, 4, 5],
    keys: ['KeyD', 'KeyA', 'KeyW', 'KeyS', 'KeyQ', 'KeyE', 'KeyR', 'KeyF', 'KeyZ', 'KeyC', 'KeyV'],
    initialJoints: [0.0158, 2.052, 2.1307, -0.0845, 1.5857, -0.3745],
  }],
};

export function SO101Controller({ ik }: { ik?: IkContextValue | null }) {
  useArmController(config, ik);
  return null;
}
