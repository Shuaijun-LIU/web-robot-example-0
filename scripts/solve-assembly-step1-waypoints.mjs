import { readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';

import loadMujoco from 'mujoco-js';
import * as THREE from 'three';

import { ASSEMBLY1_STEP1_ARMS } from '../src/assemblyStep1.js';
import { FRANKA_HOME } from '../src/sceneLayouts.js';
import { solveSelectedIk } from '../src/controllers/selectedIkSolver.js';

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

function positionError(mujoco, model, data, siteId, qposAddresses, solution, target) {
  for (let index = 0; index < qposAddresses.length; index += 1) {
    data.qpos[qposAddresses[index]] = solution[index];
  }
  mujoco.mj_forward(model, data);
  return Math.hypot(
    data.site_xpos[siteId * 3] - target[0],
    data.site_xpos[siteId * 3 + 1] - target[1],
    data.site_xpos[siteId * 3 + 2] - target[2],
  );
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
for (let index = 0; index < qposAddresses.length; index += 1) {
  data.qpos[qposAddresses[index]] = FRANKA_HOME[index];
}
mujoco.mj_forward(model, data);

const rotation = new THREE.Matrix4().set(
  data.site_xmat[siteId * 9], data.site_xmat[siteId * 9 + 1], data.site_xmat[siteId * 9 + 2], 0,
  data.site_xmat[siteId * 9 + 3], data.site_xmat[siteId * 9 + 4], data.site_xmat[siteId * 9 + 5], 0,
  data.site_xmat[siteId * 9 + 6], data.site_xmat[siteId * 9 + 7], data.site_xmat[siteId * 9 + 8], 0,
  0, 0, 0, 1,
);
const targetQuaternion = new THREE.Quaternion().setFromRotationMatrix(rotation);
const results = [];

for (const [index, arm] of ASSEMBLY1_STEP1_ARMS.entries()) {
  const highTarget = worldToRobot(arm.highWaypoint, attachmentFrames[index]);
  const finalTarget = worldToRobot(arm.finalWaypoint, attachmentFrames[index]);
  const currentQ = FRANKA_HOME.slice(0, 7);
  const shared = { mujoco, model, data, siteId, qposAddresses, targetQuaternion };
  const high = solveSelectedIk({
    ...shared,
    currentQ,
    targetPosition: new THREE.Vector3(...highTarget),
    maxIterations: 100,
  });
  if (!high) throw new Error(`${arm.key} high waypoint did not produce a solution`);
  const limits = jointIds.map((jointId) => [
    model.jnt_range[jointId * 2],
    model.jnt_range[jointId * 2 + 1],
  ]);
  const boundedHigh = high.map((value, joint) => Math.max(
    limits[joint][0],
    Math.min(limits[joint][1], value),
  ));
  const highError = positionError(
    mujoco, model, data, siteId, qposAddresses, boundedHigh, highTarget,
  );
  const final = solveSelectedIk({
    ...shared,
    currentQ: boundedHigh,
    targetPosition: new THREE.Vector3(...finalTarget),
    maxIterations: 100,
  });
  if (!final) throw new Error(`${arm.key} final waypoint did not produce a solution`);
  const boundedFinal = final.map((value, joint) => Math.max(
    limits[joint][0],
    Math.min(limits[joint][1], value),
  ));
  const finalError = positionError(
    mujoco, model, data, siteId, qposAddresses, boundedFinal, finalTarget,
  );
  if (highError > 0.03 || finalError > 0.03) {
    throw new Error(
      `${arm.key} exceeds the 3 cm TCP tolerance (high=${highError}, final=${finalError})`,
    );
  }
  results.push({
    key: arm.key,
    highTarget,
    finalTarget,
    high: boundedHigh.map((value) => Number(value.toFixed(6))),
    final: boundedFinal.map((value) => Number(value.toFixed(6))),
    highError: Number(highError.toFixed(6)),
    finalError: Number(finalError.toFixed(6)),
    withinLimits: [...boundedHigh, ...boundedFinal].every((value, joint) => {
      const [minimum, maximum] = limits[joint % 7];
      return value >= minimum && value <= maximum;
    }),
  });
}

console.log(JSON.stringify(results, null, 2));
data.delete();
model.delete();
