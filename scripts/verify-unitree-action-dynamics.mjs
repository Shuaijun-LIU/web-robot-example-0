import { readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative } from 'node:path';

import loadMujoco from 'mujoco-js';

import { UNITREE_ACTION_LAB_LAYOUT } from '../src/unitreeActionLab.js';
import {
  G1_ACTUATORS,
  GO2_ACTUATORS,
  UNITREE_ACTION_DURATION,
  applyUnitreeActionTargets,
  sampleUnitreeAction,
} from '../src/unitreeActionSequence.js';

const assetDirectory = 'public/assets/unitree-action-lab';

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
      // The single-process verifier may revisit a mounted parent directory.
    }
  }
}

function getName(model, address) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

function findNamedIndex(model, count, addresses, name) {
  for (let index = 0; index < count; index += 1) {
    if (getName(model, addresses[index]) === name) return index;
  }
  throw new Error(`MuJoCo resource not found: ${name}`);
}

function tiltDegrees(data, bodyId) {
  const offset = bodyId * 4;
  const x = data.xquat[offset + 1];
  const y = data.xquat[offset + 2];
  const upZ = Math.max(-1, Math.min(1, 1 - 2 * (x * x + y * y)));
  return Math.acos(upZ) * 180 / Math.PI;
}

const mujoco = await loadMujoco({
  printErr: (message) => console.error(`MuJoCo: ${message}`),
});
try {
  mujoco.FS.mkdir('/working');
} catch {
  // A fresh module normally has no /working directory.
}
for (const sourcePath of listFiles(assetDirectory)) {
  const fileName = relative(assetDirectory, sourcePath);
  const destination = `/working/${fileName}`;
  ensureDirectory(mujoco, destination);
  mujoco.FS.writeFile(destination, readFileSync(sourcePath));
}

const model = mujoco.MjModel.loadFromXML('/working/scene.xml');
const data = new mujoco.MjData(model);

const actuatorIds = {
  g1: G1_ACTUATORS.map(({ name }) => findNamedIndex(model, model.nu, model.name_actuatoradr, name)),
  go2: GO2_ACTUATORS.map(({ name }) => findNamedIndex(model, model.nu, model.name_actuatoradr, name)),
};
const allIds = [...actuatorIds.g1, ...actuatorIds.go2];
if (new Set(allIds).size !== 47) throw new Error('Expected 47 unique Unitree actuators');

const jointIds = allIds.map((actuatorId) => {
  const transmissionType = model.actuator_trntype[actuatorId];
  const jointId = model.actuator_trnid[actuatorId * 2];
  if ((transmissionType !== 0 && transmissionType !== 1) || jointId < 0) {
    throw new Error(`Actuator ${actuatorId} is not a scalar joint transmission`);
  }
  const jointType = model.jnt_type[jointId];
  if (jointType !== 2 && jointType !== 3) {
    throw new Error(`Actuator ${actuatorId} does not drive a hinge or slide joint`);
  }
  return jointId;
});

UNITREE_ACTION_LAB_LAYOUT.homeJoints.forEach((home, index) => {
  data.ctrl[allIds[index]] = home;
  data.qpos[model.jnt_qposadr[jointIds[index]]] = home;
});
mujoco.mj_forward(model, data);

const initialJointPositions = jointIds.map((jointId) => data.qpos[model.jnt_qposadr[jointId]]);
const g1BodyId = findNamedIndex(model, model.nbody, model.name_bodyadr, 'g1_pelvis');
const go2BodyId = findNamedIndex(model, model.nbody, model.name_bodyadr, 'go2_base');
const floorGeomId = findNamedIndex(model, model.ngeom, model.name_geomadr, 'floor');
const visitedPhases = [];
let finiteState = true;
let g1GroundContactSteps = 0;
let go2GroundContactSteps = 0;
let g1MaxJointDelta = 0;
let go2MaxLegJointDelta = 0;
let go2MaxArmJointDelta = 0;

function recordPhase(phase) {
  if (visitedPhases.at(-1) !== phase) visitedPhases.push(phase);
}

function updateMetrics() {
  finiteState &&= [...data.qpos, ...data.qvel, ...data.ctrl].every(Number.isFinite);
  for (let index = 0; index < jointIds.length; index += 1) {
    const value = data.qpos[model.jnt_qposadr[jointIds[index]]];
    const delta = Math.abs(value - initialJointPositions[index]);
    if (index < 29) g1MaxJointDelta = Math.max(g1MaxJointDelta, delta);
    else if (index < 41) go2MaxLegJointDelta = Math.max(go2MaxLegJointDelta, delta);
    else go2MaxArmJointDelta = Math.max(go2MaxArmJointDelta, delta);
  }

  let g1Contact = false;
  let go2Contact = false;
  for (let index = 0; index < data.ncon; index += 1) {
    const contact = data.contact.get(index);
    if (!contact || (contact.geom1 !== floorGeomId && contact.geom2 !== floorGeomId)) continue;
    const robotGeomId = contact.geom1 === floorGeomId ? contact.geom2 : contact.geom1;
    const bodyName = getName(model, model.name_bodyadr[model.geom_bodyid[robotGeomId]]);
    if (bodyName.startsWith('g1_')) g1Contact = true;
    if (bodyName.startsWith('go2_')) go2Contact = true;
  }
  if (g1Contact) g1GroundContactSteps += 1;
  if (go2Contact) go2GroundContactSteps += 1;
}

const timestep = model.opt.timestep;
const rolloutDuration = UNITREE_ACTION_DURATION + 1.5;
while (data.time < rolloutDuration - timestep * 0.5) {
  const actionTime = Math.min(data.time, UNITREE_ACTION_DURATION);
  const sample = sampleUnitreeAction(actionTime);
  recordPhase(sample.phase);
  applyUnitreeActionTargets(data.ctrl, actuatorIds, sample);
  mujoco.mj_step(model, data);
  updateMetrics();
}
recordPhase(sampleUnitreeAction(UNITREE_ACTION_DURATION).phase);
mujoco.mj_forward(model, data);
updateMetrics();

const result = {
  completed: data.time >= rolloutDuration - timestep,
  finiteState,
  runtimeWrites: 'ctrl-only',
  simulatedSeconds: Number(data.time.toFixed(4)),
  timestep,
  visitedPhases,
  g1: {
    maxJointDelta: Number(g1MaxJointDelta.toFixed(6)),
    finalHeight: Number(data.xpos[g1BodyId * 3 + 2].toFixed(6)),
    finalTiltDegrees: Number(tiltDegrees(data, g1BodyId).toFixed(6)),
    groundContactSteps: g1GroundContactSteps,
  },
  go2: {
    maxLegJointDelta: Number(go2MaxLegJointDelta.toFixed(6)),
    maxArmJointDelta: Number(go2MaxArmJointDelta.toFixed(6)),
    finalHeight: Number(data.xpos[go2BodyId * 3 + 2].toFixed(6)),
    finalTiltDegrees: Number(tiltDegrees(data, go2BodyId).toFixed(6)),
    groundContactSteps: go2GroundContactSteps,
  },
};

const expectedPhases = [
  'settle',
  'rise-greet',
  'scan-wave',
  'lower',
  'recover',
  'final-hold',
  'complete',
];
const failures = [];
if (!result.completed) failures.push('rollout did not complete');
if (!result.finiteState) failures.push('state contains a non-finite value');
if (JSON.stringify(result.visitedPhases) !== JSON.stringify(expectedPhases)) failures.push('phase sequence mismatch');
if (result.g1.maxJointDelta <= 0.25) failures.push('G1 did not articulate');
if (result.go2.maxLegJointDelta <= 0.25) failures.push('Go2 legs did not articulate');
if (result.go2.maxArmJointDelta <= 0.25) failures.push('Airbot arm did not articulate');
if (result.g1.finalHeight < 0.75 || result.g1.finalHeight > 0.85) failures.push('G1 final height is unstable');
if (result.g1.finalTiltDegrees > 5) failures.push('G1 final tilt is unstable');
if (result.go2.finalHeight < 0.22 || result.go2.finalHeight > 0.34) failures.push('Go2 final height is unstable');
if (result.go2.finalTiltDegrees > 10) failures.push('Go2 final tilt is unstable');
if (result.g1.groundContactSteps === 0) failures.push('G1 never contacted the ground');
if (result.go2.groundContactSteps === 0) failures.push('Go2 never contacted the ground');

console.log(JSON.stringify(result));
data.delete();
model.delete();
if (failures.length > 0) throw new Error(failures.join('; '));
