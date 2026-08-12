export function shiftIndices(indices, offset) {
  return indices.map((index) => index + offset);
}

function prefixedNames(prefix, names) {
  return names.map((name) => `${prefix}${name}`);
}

export function createFrankaTargets() {
  return Array.from({ length: 4 }, (_, index) => {
    const prefix = `r${index}_`;
    const actuatorOffset = index * 8;
    return {
      key: `r${index}`,
      label: `Arm ${index + 1}`,
      prefix,
      actuatorOffset,
      gripperActuator: `${prefix}gripper`,
      ik: {
        siteName: `${prefix}tcp`,
        jointNames: prefixedNames(prefix, [
          'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7',
        ]),
        actuatorIndices: shiftIndices([0, 1, 2, 3, 4, 5, 6], actuatorOffset),
      },
    };
  });
}

export function createSO101Targets() {
  return Array.from({ length: 4 }, (_, index) => {
    const prefix = `r${index}_`;
    const actuatorOffset = index * 6;
    return {
      key: `r${index}`,
      label: `Arm ${index + 1}`,
      prefix,
      actuatorOffset,
      ik: {
        siteName: `${prefix}tcp`,
        jointNames: prefixedNames(prefix, [
          'Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll',
        ]),
        actuatorIndices: shiftIndices([0, 1, 2, 3, 4], actuatorOffset),
      },
    };
  });
}

export function createXLeRobotTargets() {
  return Array.from({ length: 2 }, (_, index) => ({
    key: `r${index}`,
    label: `Robot ${index + 1}`,
    prefix: `r${index}_`,
    actuatorOffset: index * 16,
  }));
}
