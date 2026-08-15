import { readFileSync, readdirSync, statSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join, relative } from 'node:path';

import loadMujoco from 'mujoco-js';

import { UNITREE_ACTION_LAB_LAYOUT } from '../src/unitreeActionLab.js';
import {
  G1_ACTUATORS,
  GO2_ACTUATORS,
  applyUnitreeActionTargets,
} from '../src/unitreeActionSequence.js';
import { sampleUnitreeLocomotionAction } from '../src/unitreeLocomotionController.js';
import {
  computeRootDisplacement,
  readUnitreeRootState,
  validateLocomotionTargets,
  validateUnitreeDynamicsState,
} from '../src/unitreeDynamicsAdapter.js';

const assetDirectory = 'public/assets/unitree-action-lab';
const writeMetrics = process.argv.includes('--write');
const durationArgument = process.argv.find((argument) => argument.startsWith('--duration='));
const requestedDuration = durationArgument ? Number(durationArgument.split('=')[1]) : 25.25;

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
      // A parent may already exist in the in-memory filesystem.
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
const allActuatorIds = [...actuatorIds.g1, ...actuatorIds.go2];
if (new Set(allActuatorIds).size !== 47) throw new Error('Expected 47 unique Unitree actuators');

const actuatorJointIds = allActuatorIds.map((actuatorId) => {
  const jointId = model.actuator_trnid[actuatorId * 2];
  if (jointId < 0) throw new Error(`Actuator ${actuatorId} has no scalar joint transmission`);
  return jointId;
});
UNITREE_ACTION_LAB_LAYOUT.homeJoints.forEach((home, index) => {
  data.ctrl[allActuatorIds[index]] = home;
  data.qpos[model.jnt_qposadr[actuatorJointIds[index]]] = home;
});
mujoco.mj_forward(model, data);

const rootAddresses = {
  g1: {
    qposAddress: model.jnt_qposadr[
      findNamedIndex(model, model.njnt, model.name_jntadr, 'g1_floating_base_joint')
    ],
    dofAddress: model.jnt_dofadr[
      findNamedIndex(model, model.njnt, model.name_jntadr, 'g1_floating_base_joint')
    ],
  },
  go2: {
    qposAddress: model.jnt_qposadr[
      findNamedIndex(model, model.njnt, model.name_jntadr, 'go2_floating_base_joint')
    ],
    dofAddress: model.jnt_dofadr[
      findNamedIndex(model, model.njnt, model.name_jntadr, 'go2_floating_base_joint')
    ],
  },
};
const readRoots = () => ({
  g1: readUnitreeRootState(data.qpos, data.qvel, rootAddresses.g1),
  go2: readUnitreeRootState(data.qpos, data.qvel, rootAddresses.go2),
});

const initialRoots = readRoots();
const floorGeomId = findNamedIndex(model, model.ngeom, model.name_geomadr, 'floor');
const g1FootSiteIds = [
  findNamedIndex(model, model.nsite, model.name_siteadr, 'g1_left_foot'),
  findNamedIndex(model, model.nsite, model.name_siteadr, 'g1_right_foot'),
];
const visitedPhases = [];
const initialJointPositions = actuatorJointIds.map(
  (jointId) => data.qpos[model.jnt_qposadr[jointId]],
);
let finiteState = true;
let safety = { safe: true, reason: null };
let clampCount = 0;
let g1GroundContactSteps = 0;
let go2GroundContactSteps = 0;
let g1MinHeight = initialRoots.g1.position[2];
let go2MinHeight = initialRoots.go2.position[2];
let g1MaxTiltDegrees = 0;
let go2MaxTiltDegrees = 0;
let g1MaxJointDelta = 0;
let go2MaxLegJointDelta = 0;
const trace = [];
let nextTraceTime = 0;

function recordPhase(phase) {
  if (visitedPhases.at(-1) !== phase) visitedPhases.push(phase);
}

function updateMetrics(roots) {
  finiteState &&= [...data.qpos, ...data.qvel, ...data.ctrl].every(Number.isFinite);
  g1MinHeight = Math.min(g1MinHeight, roots.g1.position[2]);
  go2MinHeight = Math.min(go2MinHeight, roots.go2.position[2]);
  g1MaxTiltDegrees = Math.max(
    g1MaxTiltDegrees,
    Math.hypot(roots.g1.roll, roots.g1.pitch) * 180 / Math.PI,
  );
  go2MaxTiltDegrees = Math.max(
    go2MaxTiltDegrees,
    Math.hypot(roots.go2.roll, roots.go2.pitch) * 180 / Math.PI,
  );
  actuatorJointIds.forEach((jointId, index) => {
    const delta = Math.abs(data.qpos[model.jnt_qposadr[jointId]] - initialJointPositions[index]);
    if (index < 29) g1MaxJointDelta = Math.max(g1MaxJointDelta, delta);
    else if (index < 41) go2MaxLegJointDelta = Math.max(go2MaxLegJointDelta, delta);
  });

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
const rolloutDuration = Number.isFinite(requestedDuration) && requestedDuration > 0
  ? requestedDuration
  : 25.25;
while (data.time < rolloutDuration - timestep * 0.5) {
  const roots = readRoots();
  safety = validateUnitreeDynamicsState(roots);
  if (!safety.safe) break;
  const sample = validateLocomotionTargets(sampleUnitreeLocomotionAction(data.time, {
    g1: roots.g1,
    go2: roots.go2,
  }));
  if (data.time + timestep * 0.5 >= nextTraceTime) {
    trace.push({
      time: Number(data.time.toFixed(3)),
      phase: sample.phase,
      g1: [roots.g1.position[0], roots.g1.position[1], roots.g1.position[2], roots.g1.roll, roots.g1.pitch].map(
        (value) => Number(value.toFixed(5)),
      ),
      go2: [roots.go2.position[0], roots.go2.position[1], roots.go2.position[2], roots.go2.roll, roots.go2.pitch].map(
        (value) => Number(value.toFixed(5)),
      ),
      g1Feet: g1FootSiteIds.map((siteId) => [
        data.site_xpos[siteId * 3],
        data.site_xpos[siteId * 3 + 2],
      ].map((value) => Number(value.toFixed(5)))),
    });
    nextTraceTime += 0.25;
  }
  recordPhase(sample.phase);
  clampCount = Math.max(clampCount, sample.diagnostics.clampCount);
  applyUnitreeActionTargets(data.ctrl, actuatorIds, sample);
  mujoco.mj_step(model, data);
  updateMetrics(readRoots());
}

if (data.time >= 25 - timestep) recordPhase('complete');
mujoco.mj_forward(model, data);
const finalRoots = readRoots();
updateMetrics(finalRoots);
const finalSafety = validateUnitreeDynamicsState(finalRoots);
if (!finalSafety.safe) safety = finalSafety;
const g1Displacement = computeRootDisplacement(initialRoots.g1, finalRoots.g1);
const go2Displacement = computeRootDisplacement(initialRoots.go2, finalRoots.go2);

const result = {
  completed: data.time >= rolloutDuration - timestep,
  finiteState,
  runtimeWrites: 'ctrl-only',
  simulatedSeconds: Number(data.time.toFixed(4)),
  timestep,
  visitedPhases,
  clampCount,
  safety,
  trace,
  g1: {
    forwardDisplacement: Number(g1Displacement.x.toFixed(6)),
    planarDisplacement: Number(g1Displacement.planar.toFixed(6)),
    finalHeight: Number(finalRoots.g1.position[2].toFixed(6)),
    minHeight: Number(g1MinHeight.toFixed(6)),
    finalTiltDegrees: Number((Math.hypot(finalRoots.g1.roll, finalRoots.g1.pitch) * 180 / Math.PI).toFixed(6)),
    maxTiltDegrees: Number(g1MaxTiltDegrees.toFixed(6)),
    maxJointDelta: Number(g1MaxJointDelta.toFixed(6)),
    groundContactSteps: g1GroundContactSteps,
  },
  go2: {
    forwardDisplacement: Number(go2Displacement.x.toFixed(6)),
    planarDisplacement: Number(go2Displacement.planar.toFixed(6)),
    finalHeight: Number(finalRoots.go2.position[2].toFixed(6)),
    minHeight: Number(go2MinHeight.toFixed(6)),
    finalTiltDegrees: Number((Math.hypot(finalRoots.go2.roll, finalRoots.go2.pitch) * 180 / Math.PI).toFixed(6)),
    maxTiltDegrees: Number(go2MaxTiltDegrees.toFixed(6)),
    maxLegJointDelta: Number(go2MaxLegJointDelta.toFixed(6)),
    groundContactSteps: go2GroundContactSteps,
  },
};

if (writeMetrics) {
  const outputPath = 'artifacts/metrics/unitree-locomotion-suite.json';
  mkdirSync(dirname(outputPath), { recursive: true });
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
}
console.log(JSON.stringify(result));
data.delete();
model.delete();
