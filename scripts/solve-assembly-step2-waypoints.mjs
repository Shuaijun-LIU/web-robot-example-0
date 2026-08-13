import { readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';

import loadMujoco from 'mujoco-js';
import * as THREE from 'three';

import { ASSEMBLY1_STEP1_ARMS } from '../src/assemblyStep1.js';
import { ASSEMBLY1_STEP2_ARMS } from '../src/assemblyStep2.js';
import {
  fitJointAngleToRange,
  solveSelectedIk,
} from '../src/controllers/selectedIkSolver.js';

const defaultAssetDirectory = resolve(
  '..',
  'google-deepmind__mujoco_menagerie',
  'franka_emika_panda',
);
const assetDirectory = resolve(process.argv[2] ?? defaultAssetDirectory);
const attachmentFrames = [
  { position: [0, -0.9, 0.1], yaw: 0 },
  { position: [0.9, 0, 0.1], yaw: Math.PI / 2 },
  { position: [0, 0.9, 0.1], yaw: Math.PI },
  { position: [-0.9, 0, 0.1], yaw: -Math.PI / 2 },
];

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

function poseError({
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

for (const [index, arm] of ASSEMBLY1_STEP2_ARMS.entries()) {
  const step1Final = ASSEMBLY1_STEP1_ARMS[index].finalJointTargets;
  for (let joint = 0; joint < qposAddresses.length; joint += 1) {
    data.qpos[qposAddresses[joint]] = step1Final[joint];
  }
  mujoco.mj_forward(model, data);

  const worldQuaternion = new THREE.Quaternion(...arm.tcpQuaternion).normalize();
  const baseQuaternion = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    attachmentFrames[index].yaw,
  );
  const targetQuaternion = baseQuaternion.clone().invert().multiply(worldQuaternion).normalize();
  const approachTarget = worldToRobot(arm.approachWaypoint, attachmentFrames[index]);
  const contactTarget = worldToRobot(arm.contactWaypoint, attachmentFrames[index]);
  const shared = { mujoco, model, data, siteId, qposAddresses, targetQuaternion };

  const approach = solveSelectedIk({
    ...shared,
    currentQ: [...step1Final],
    targetPosition: new THREE.Vector3(...approachTarget),
    maxIterations: 300,
  });
  if (!approach) throw new Error(`${arm.key} approach waypoint did not produce a solution`);
  const boundedApproach = approach.map((value, joint) => fitJointAngleToRange(
    value,
    limits[joint][0],
    limits[joint][1],
  ));
  const approachError = poseError({
    ...shared,
    solution: boundedApproach,
    targetPosition: approachTarget,
  });

  const contact = solveSelectedIk({
    ...shared,
    currentQ: boundedApproach,
    targetPosition: new THREE.Vector3(...contactTarget),
    maxIterations: 300,
  });
  if (!contact) throw new Error(`${arm.key} contact waypoint did not produce a solution`);
  const boundedContact = contact.map((value, joint) => fitJointAngleToRange(
    value,
    limits[joint][0],
    limits[joint][1],
  ));
  const contactError = poseError({
    ...shared,
    solution: boundedContact,
    targetPosition: contactTarget,
  });
  const withinLimits = [...boundedApproach, ...boundedContact].every((value, joint) => {
    const [minimum, maximum] = limits[joint % 7];
    return Number.isFinite(value) && value >= minimum && value <= maximum;
  });
  if (
    approachError.position > 0.01
    || contactError.position > 0.01
    || approachError.orientationDegrees > 5
    || contactError.orientationDegrees > 5
    || !withinLimits
  ) {
    throw new Error(
      `${arm.key} exceeds Step 2 IK tolerance `
      + `(approach=${approachError.position}m/${approachError.orientationDegrees}deg, `
      + `contact=${contactError.position}m/${contactError.orientationDegrees}deg)`,
    );
  }

  const approachJointTargets = rounded(boundedApproach);
  const contactJointTargets = rounded(boundedContact);
  results.push({
    key: arm.key,
    approachTarget: arm.approachWaypoint,
    contactTarget: arm.contactWaypoint,
    approachJointTargets,
    contactJointTargets,
    approachPositionError: Number(approachError.position.toFixed(6)),
    contactPositionError: Number(contactError.position.toFixed(6)),
    approachOrientationErrorDegrees: Number(approachError.orientationDegrees.toFixed(6)),
    contactOrientationErrorDegrees: Number(contactError.orientationDegrees.toFixed(6)),
    withinLimits,
    matchesContract: arraysMatch(arm.approachJointTargets, approachJointTargets)
      && arraysMatch(arm.contactJointTargets, contactJointTargets),
  });
}

console.log(JSON.stringify(results, null, 2));
data.delete();
model.delete();

if (results.some((result) => !result.matchesContract) && process.env.ALLOW_UNRECORDED_IK !== '1') {
  throw new Error(
    'Generated Step 2 targets do not match src/assemblyStep2.js; '
    + 'copy the printed arrays, then rerun the solver',
  );
}
