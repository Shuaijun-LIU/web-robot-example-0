import { readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';

import loadMujoco from 'mujoco-js';
import * as THREE from 'three';

import { topDownTcpQuaternion } from '../src/assemblyStep1.js';
import { ASSEMBLY1_STEP2_ARMS } from '../src/assemblyStep2.js';
import {
  ASSEMBLY1_STEP3_TRANSPORT_ARMS,
  ASSEMBLY1_STEP3_WAYPOINTS,
} from '../src/assemblyStep3.js';
import {
  fitJointAngleToRange,
  solveSelectedIk,
} from '../src/controllers/selectedIkSolver.js';

const assetDirectory = resolve(
  process.argv[2] ?? '../google-deepmind__mujoco_menagerie/franka_emika_panda',
);
const attachmentFrames = [
  { position: [0, -0.9, 0.1], yaw: 0 },
  { position: [0.9, 0, 0.1], yaw: Math.PI / 2 },
  { position: [-0.3, 0.85, 0.1], yaw: Math.PI },
  { position: [-0.8, 0, 0.1], yaw: -Math.PI / 2 },
];
const waypointNames = ['lift', 'transferA', 'transferMid', 'hover', 'descentMid', 'aligned'];
const recordedTargetNames = {
  lift: 'liftJointTargets',
  transferA: 'transferAJointTargets',
  transferMid: 'transferMidJointTargets',
  hover: 'hoverJointTargets',
  descentMid: 'descentMidJointTargets',
  aligned: 'alignedJointTargets',
};

function listFiles(directory) {
  return readdirSync(directory).flatMap((name) => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? listFiles(path) : [path];
  });
}

function ensureDirectory(mujoco, filePath) {
  const parts = dirname(filePath).split('/').filter(Boolean);
  let current = '';
  for (const part of parts) {
    current += `/${part}`;
    try {
      mujoco.FS.mkdir(current);
    } catch {
      // Directory already exists in the virtual filesystem.
    }
  }
}

function nameAt(model, address) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

function findNamedIndex(model, count, addresses, name) {
  return Array.from({ length: count }, (_, index) => index).find(
    (index) => nameAt(model, addresses[index]) === name,
  );
}

function worldToRobot(point, frame) {
  const dx = point[0] - frame.position[0];
  const dy = point[1] - frame.position[1];
  const cosine = Math.cos(frame.yaw);
  const sine = Math.sin(frame.yaw);
  return [
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy,
    point[2] - frame.position[2],
  ];
}

function siteQuaternion(data, siteId) {
  const offset = siteId * 9;
  const rotation = new THREE.Matrix4().set(
    data.site_xmat[offset], data.site_xmat[offset + 1], data.site_xmat[offset + 2], 0,
    data.site_xmat[offset + 3], data.site_xmat[offset + 4], data.site_xmat[offset + 5], 0,
    data.site_xmat[offset + 6], data.site_xmat[offset + 7], data.site_xmat[offset + 8], 0,
    0, 0, 0, 1,
  );
  return new THREE.Quaternion().setFromRotationMatrix(rotation).normalize();
}

function evaluatePose({
  mujoco,
  model,
  data,
  siteId,
  qposAddresses,
  solution,
  targetPosition,
  targetQuaternion,
}) {
  for (let index = 0; index < qposAddresses.length; index += 1) {
    data.qpos[qposAddresses[index]] = solution[index];
  }
  mujoco.mj_forward(model, data);
  const position = Math.hypot(
    data.site_xpos[siteId * 3] - targetPosition[0],
    data.site_xpos[siteId * 3 + 1] - targetPosition[1],
    data.site_xpos[siteId * 3 + 2] - targetPosition[2],
  );
  const actualQuaternion = siteQuaternion(data, siteId);
  const dot = Math.min(1, Math.abs(actualQuaternion.dot(targetQuaternion)));
  return {
    position,
    orientationDegrees: THREE.MathUtils.radToDeg(2 * Math.acos(dot)),
  };
}

function rounded(values) {
  return values.map((value) => Number(value.toFixed(6)));
}

function arraysMatch(first, second) {
  return Array.isArray(first)
    && first.length === second.length
    && first.every((value, index) => Math.abs(value - second[index]) <= 1e-6);
}

const mujoco = await loadMujoco({
  printErr: (message) => console.error(`MuJoCo: ${message}`),
});
try {
  mujoco.FS.mkdir('/working');
} catch {
  // A single invocation can safely reuse the directory.
}

for (const sourcePath of listFiles(assetDirectory)) {
  const fileName = relative(assetDirectory, sourcePath);
  let contents = readFileSync(sourcePath);
  if (fileName === 'panda.xml') {
    const text = contents.toString();
    const anchor = '<body name="hand"';
    const start = text.indexOf(anchor);
    const tagEnd = text.indexOf('>', start + anchor.length);
    if (start < 0 || tagEnd < 0) throw new Error('Could not inject the Assembly1 TCP site');
    contents = `${text.slice(0, tagEnd + 1)}<site name="tcp" pos="0 0 0.1" size="0.01"/>${text.slice(tagEnd + 1)}`;
  }
  const destination = `/working/${fileName}`;
  ensureDirectory(mujoco, destination);
  mujoco.FS.writeFile(destination, contents);
}

const model = mujoco.MjModel.loadFromXML('/working/scene.xml');
const data = new mujoco.MjData(model);
const siteId = findNamedIndex(model, model.nsite, model.name_siteadr, 'tcp');
const jointIds = Array.from({ length: 7 }, (_, index) => (
  findNamedIndex(model, model.njnt, model.name_jntadr, `joint${index + 1}`)
));
if (siteId === undefined || jointIds.some((jointId) => jointId === undefined)) {
  throw new Error('Could not resolve the Franka TCP or arm joints');
}
const qposAddresses = jointIds.map((jointId) => model.jnt_qposadr[jointId]);
const limits = jointIds.map((jointId) => [
  model.jnt_range[jointId * 2],
  model.jnt_range[jointId * 2 + 1],
]);
const results = [];

for (const [transportIndex, contract] of ASSEMBLY1_STEP3_TRANSPORT_ARMS.entries()) {
  const arm = ASSEMBLY1_STEP2_ARMS[contract.armIndex];
  const frame = attachmentFrames[contract.armIndex];
  if (!arm || arm.key !== contract.key) {
    throw new Error(`Step 3 transport contract does not match Step 2 at ${contract.key}`);
  }
  let currentQ = [...arm.contactJointTargets];
  const worldQuaternion = new THREE.Quaternion(
    ...topDownTcpQuaternion(contract.closingAxisYawDegrees),
  ).normalize();
  const baseQuaternion = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    frame.yaw,
  );
  const targetQuaternion = baseQuaternion.clone().invert().multiply(worldQuaternion).normalize();
  const startTarget = worldToRobot(ASSEMBLY1_STEP3_WAYPOINTS.start[transportIndex], frame);
  const startError = evaluatePose({
    mujoco,
    model,
    data,
    siteId,
    qposAddresses,
    solution: currentQ,
    targetPosition: startTarget,
    targetQuaternion,
  });
  if (startError.position > 0.01 || startError.orientationDegrees > 5) {
    throw new Error(
      `${arm.key} Step 2 contact is not a valid Step 3 start `
      + `(${startError.position}m/${startError.orientationDegrees}deg)`,
    );
  }

  const targets = {};
  for (const waypointName of waypointNames) {
    const worldTarget = ASSEMBLY1_STEP3_WAYPOINTS[waypointName][transportIndex];
    const localTarget = worldToRobot(worldTarget, frame);
    const solution = solveSelectedIk({
      mujoco,
      model,
      data,
      siteId,
      qposAddresses,
      currentQ,
      targetPosition: new THREE.Vector3(...localTarget),
      targetQuaternion,
      maxIterations: 800,
      damping: 0.005,
    });
    if (!solution) throw new Error(`${arm.key}/${waypointName} did not produce a solution`);
    const bounded = solution.map((value, joint) => fitJointAngleToRange(
      value,
      limits[joint][0],
      limits[joint][1],
    ));
    const error = evaluatePose({
      mujoco,
      model,
      data,
      siteId,
      qposAddresses,
      solution: bounded,
      targetPosition: localTarget,
      targetQuaternion,
    });
    const withinLimits = bounded.every((value, joint) => (
      Number.isFinite(value)
      && value >= limits[joint][0]
      && value <= limits[joint][1]
    ));
    if (error.position > 0.01 || error.orientationDegrees > 5 || !withinLimits) {
      throw new Error(
        `${arm.key}/${waypointName} exceeds tolerance `
        + `(${error.position}m/${error.orientationDegrees}deg, limits=${withinLimits})`,
      );
    }
    currentQ = bounded;
    const jointTargets = rounded(bounded);
    targets[waypointName] = {
      worldTarget,
      jointTargets,
      positionError: Number(error.position.toFixed(6)),
      orientationErrorDegrees: Number(error.orientationDegrees.toFixed(6)),
      withinLimits,
      matchesContract: arraysMatch(
        contract[recordedTargetNames[waypointName]],
        jointTargets,
      ),
    };
  }
  results.push({
    key: arm.key,
    armIndex: contract.armIndex,
    closingAxisYawDegrees: contract.closingAxisYawDegrees,
    startPositionError: Number(startError.position.toFixed(6)),
    startOrientationErrorDegrees: Number(startError.orientationDegrees.toFixed(6)),
    targets,
  });
}

console.log(JSON.stringify(results, null, 2));
data.delete();
model.delete();

if (
  results.some((result) => Object.values(result.targets).some((target) => !target.matchesContract))
  && process.env.ALLOW_UNRECORDED_IK !== '1'
) {
  throw new Error(
    'Generated Step 3 targets do not match src/assemblyStep3.js; '
    + 'copy the printed arrays, then rerun the solver',
  );
}
