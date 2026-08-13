import { readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative } from 'node:path';

import loadMujoco from 'mujoco-js';

import {
  FRANKA_LAYOUT,
  SO101_LAYOUT,
  XLEROBOT_LAYOUT,
} from '../src/sceneLayouts.js';
import {
  FRANKA_ASSEMBLY1_LAYOUT,
  FRANKA_ASSEMBLY2_LAYOUT,
} from '../src/frankaAssemblyLayouts.js';
import {
  SO101_GEARBOX_LAYOUT,
  SO101_HOME_LAB_LAYOUT,
  XLEROBOT_KITTING_LAYOUT,
} from '../src/collaborativeSceneLayouts.js';

const definitions = {
  franka: { layout: FRANKA_LAYOUT, sceneFile: 'scene.xml' },
  so101: { layout: SO101_LAYOUT, sceneFile: 'objects_SO101.xml' },
  xlerobot: { layout: XLEROBOT_LAYOUT, sceneFile: 'objects.xml' },
  frankaAssembly1: { layout: FRANKA_ASSEMBLY1_LAYOUT, sceneFile: 'scene.xml' },
  frankaAssembly2: { layout: FRANKA_ASSEMBLY2_LAYOUT, sceneFile: 'scene.xml' },
  so101Gearbox: { layout: SO101_GEARBOX_LAYOUT, sceneFile: 'objects_SO101.xml' },
  so101HomeLab: { layout: SO101_HOME_LAB_LAYOUT, sceneFile: 'objects_SO101.xml' },
  xlerobotKitting: { layout: XLEROBOT_KITTING_LAYOUT, sceneFile: 'objects.xml' },
};
const [sceneKey, assetDirectory] = process.argv.slice(2);
const definition = definitions[sceneKey];

if (!definition || !assetDirectory) {
  throw new Error('Usage: node scripts/validate-mjcf.mjs <franka|so101|xlerobot|frankaAssembly1|frankaAssembly2|so101Gearbox|so101HomeLab|xlerobotKitting> <asset-directory>');
}

const mujoco = await loadMujoco({ printErr: (message) => console.error(`MuJoCo: ${message}`) });
try {
  mujoco.FS.mkdir('/working');
} catch {
  // Reusing a mounted module is safe for this single-process validator.
}

function ensureDirectory(filePath) {
  const parts = dirname(filePath).split('/').filter(Boolean);
  let current = '';
  for (const part of parts) {
    current += `/${part}`;
    try {
      mujoco.FS.mkdir(current);
    } catch {
      // Directory already exists.
    }
  }
}

function listFiles(directory) {
  return readdirSync(directory).flatMap((name) => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? listFiles(path) : [path];
  });
}

function sceneObjectToXml(object) {
  const joint = object.freejoint ? '<freejoint/>' : '';
  const position = object.position.map((value) => value.toFixed(3)).join(' ');
  const size = object.size.map((value) => value.toFixed(3)).join(' ');
  const mass = object.mass ? ` mass="${object.mass}"` : '';
  const friction = object.friction ? ` friction="${object.friction}"` : '';
  const solref = object.solref ? ` solref="${object.solref}"` : '';
  const solimp = object.solimp ? ` solimp="${object.solimp}"` : '';
  const condim = object.condim ? ` condim="${object.condim}"` : '';
  return `<body name="${object.name}" pos="${position}">${joint}<geom type="${object.type}" size="${size}" rgba="${object.rgba.join(' ')}" contype="1" conaffinity="1"${mass}${friction}${solref}${solimp}${condim}/></body>`;
}

for (const sourcePath of listFiles(assetDirectory)) {
  const fileName = relative(assetDirectory, sourcePath);
  let contents = readFileSync(sourcePath);

  if (fileName.endsWith('.xml')) {
    let text = contents.toString();
    for (const patch of definition.layout.xmlPatches) {
      if (fileName === patch.target || fileName.endsWith(`/${patch.target}`)) {
        if (patch.replace) {
          const [from, to] = patch.replace;
          if (!text.includes(from)) {
            throw new Error(`Patch pattern missing in ${fileName}: ${from.slice(0, 80)}`);
          }
          text = text.replace(from, to);
        }
        if (patch.inject && patch.injectAfter) {
          const start = text.indexOf(patch.injectAfter);
          const tagEnd = text.indexOf('>', start + patch.injectAfter.length);
          if (start === -1 || tagEnd === -1) {
            throw new Error(`Patch injection anchor missing in ${fileName}: ${patch.injectAfter}`);
          }
          text = text.slice(0, tagEnd + 1) + patch.inject + text.slice(tagEnd + 1);
        }
      }
    }

    if (process.env.STRIP_CONTACT === '1') {
      text = text.replace(/\s*<contact>[\s\S]*?<\/contact>/, '');
    }
    if (fileName === definition.sceneFile) {
      const objects = definition.layout.sceneObjects.map(sceneObjectToXml).join('');
      text = text.replace('</worldbody>', objects + '</worldbody>');
    }
    contents = text;
  }

  const destination = `/working/${fileName}`;
  ensureDirectory(destination);
  mujoco.FS.writeFile(destination, contents);
}

console.log(`MuJoCo ${mujoco.mj_versionString()}`);
const model = mujoco.MjModel.loadFromXML(`/working/${definition.sceneFile}`);
console.log(
  `${sceneKey}: ${model.nbody} bodies, ${model.ngeom} geoms, ${model.nu} actuators, ${model.nq} qpos`,
);
const getName = (address) => {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
};
const findNamedIndex = (count, addresses, name) =>
  Array.from({ length: count }, (_, index) => index).find(
    (index) => getName(addresses[index]) === name,
  );
if (process.env.LIST_NAMES === '1') {
  console.log(
    'bodies:',
    Array.from({ length: model.nbody }, (_, index) => getName(model.name_bodyadr[index])).join(', '),
  );
  console.log(
    'actuators:',
    Array.from({ length: model.nu }, (_, index) => getName(model.name_actuatoradr[index])).join(', '),
  );
}
if (process.env.POSE_REPORT === '1') {
  const data = new mujoco.MjData(model);
  for (let index = 0; index < Math.min(model.nu, definition.layout.homeJoints.length); index += 1) {
    const home = definition.layout.homeJoints[index];
    data.ctrl[index] = home;
    const transmissionType = model.actuator_trntype[index];
    const jointId = model.actuator_trnid[index * 2];
    if ((transmissionType === 0 || transmissionType === 1) && jointId >= 0) {
      const jointType = model.jnt_type[jointId];
      if (jointType === 2 || jointType === 3) {
        data.qpos[model.jnt_qposadr[jointId]] = home;
      }
    }
  }
  mujoco.mj_forward(model, data);
  const rootName = {
    franka: 'link0',
    frankaAssembly1: 'link0',
    frankaAssembly2: 'link0',
    so101: 'Base',
    so101Gearbox: 'Base',
    so101HomeLab: 'Base',
    xlerobot: 'chassis',
    xlerobotKitting: 'chassis',
  }[sceneKey];
  for (let instance = 0; instance < definition.layout.instanceCount; instance += 1) {
    const prefixedRoot = `r${instance}_${rootName}`;
    const bodyId = Array.from({ length: model.nbody }, (_, index) => index).find(
      (index) => getName(model.name_bodyadr[index]) === prefixedRoot,
    );
    const rootPosition = bodyId === undefined
      ? null
      : Array.from(data.xpos.slice(bodyId * 3, bodyId * 3 + 3));
    const tcpName = `r${instance}_tcp`;
    const siteId = Array.from({ length: model.nsite }, (_, index) => index).find(
      (index) => getName(model.name_siteadr[index]) === tcpName,
    );
    const tcpPosition = siteId === undefined
      ? null
      : Array.from(data.site_xpos.slice(siteId * 3, siteId * 3 + 3));
    console.log(`instance ${instance}: root=${JSON.stringify(rootPosition)} tcp=${JSON.stringify(tcpPosition)}`);
  }
  data.delete();
}
if (process.env.INITIAL_CONTACT_REPORT === '1') {
  const data = new mujoco.MjData(model);
  for (let index = 0; index < Math.min(model.nu, definition.layout.homeJoints.length); index += 1) {
    const home = definition.layout.homeJoints[index];
    data.ctrl[index] = home;
    const transmissionType = model.actuator_trntype[index];
    const jointId = model.actuator_trnid[index * 2];
    if ((transmissionType === 0 || transmissionType === 1) && jointId >= 0) {
      const jointType = model.jnt_type[jointId];
      if (jointType === 2 || jointType === 3) {
        data.qpos[model.jnt_qposadr[jointId]] = home;
      }
    }
  }
  mujoco.mj_forward(model, data);

  const penetratingContacts = [];
  for (let index = 0; index < data.ncon; index += 1) {
    const contact = data.contact.get(index);
    if (!contact || contact.dist >= -1e-4) continue;
    const body1 = model.geom_bodyid[contact.geom1];
    const body2 = model.geom_bodyid[contact.geom2];
    penetratingContacts.push({
      distance: contact.dist,
      pair: `${getName(model.name_bodyadr[body1])}/${getName(model.name_geomadr[contact.geom1])}`
        + ` <-> ${getName(model.name_bodyadr[body2])}/${getName(model.name_geomadr[contact.geom2])}`,
    });
  }
  penetratingContacts.sort((left, right) => left.distance - right.distance);
  console.log(`initial penetrating contacts: ${penetratingContacts.length}/${data.ncon}`);
  for (const { distance, pair } of penetratingContacts.slice(0, 30)) {
    console.log(`  ${distance.toFixed(6)}m ${pair}`);
  }
  if (
    process.env.INITIAL_CONTACT_STRICT === '1'
    && penetratingContacts.some(({ distance }) => distance < -0.005)
  ) {
    throw new Error('Initial scene penetration exceeds 5mm');
  }
  data.delete();
}
if (process.env.GRASP_REPORT === '1') {
  if (!sceneKey.startsWith('frankaAssembly')) {
    throw new Error('GRASP_REPORT is only supported for Franka Assembly scenes');
  }
  const toolName = process.env.GRASP_TOOL ?? 'manual_screwdriver';
  const toolBodyNames = {
    manual_screwdriver: 'manual_screwdriver',
    torque_driver: 'torque_driver',
    hammer: sceneKey === 'frankaAssembly1' ? 'double_face_hammer' : 'claw_hammer',
  };
  const toolGraspPoints = {
    manual_screwdriver: sceneKey === 'frankaAssembly1' ? [-.033, 0, 0] : [-.025, 0, 0],
    torque_driver: sceneKey === 'frankaAssembly1' ? [.03, 0, -.02] : [.03, 0, -.05],
    hammer: [-.075, 0, -.006],
  };
  const graspLocal = process.env.GRASP_POINT
    ? process.env.GRASP_POINT.split(',').map(Number)
    : toolGraspPoints[toolName];
  if (!graspLocal) {
    throw new Error(`Unsupported GRASP_TOOL: ${toolName}`);
  }
  const toolBodyName = toolBodyNames[toolName];

  const data = new mujoco.MjData(model);
  for (let index = 0; index < Math.min(model.nu, definition.layout.homeJoints.length); index += 1) {
    const home = definition.layout.homeJoints[index];
    data.ctrl[index] = home;
    const transmissionType = model.actuator_trntype[index];
    const jointId = model.actuator_trnid[index * 2];
    if ((transmissionType === 0 || transmissionType === 1) && jointId >= 0) {
      const jointType = model.jnt_type[jointId];
      if (jointType === 2 || jointType === 3) {
        data.qpos[model.jnt_qposadr[jointId]] = home;
      }
    }
  }

  const siteId = findNamedIndex(model.nsite, model.name_siteadr, 'r0_tcp');
  const toolBodyId = findNamedIndex(model.nbody, model.name_bodyadr, toolBodyName);
  const namedToolJointId = findNamedIndex(model.njnt, model.name_jntadr, `${toolBodyName}_free`);
  const toolJointId = namedToolJointId ?? (toolBodyId === undefined ? undefined : model.body_jntadr[toolBodyId]);
  const gripperActuatorId = findNamedIndex(model.nu, model.name_actuatoradr, 'r0_gripper');
  if ([siteId, toolBodyId, toolJointId, gripperActuatorId].some((id) => id === undefined)) {
    throw new Error(`Missing r0_tcp, ${toolBodyName} free joint, or r0_gripper`);
  }
  const fingerJointIds = [];
  for (const fingerName of ['r0_finger_joint1', 'r0_finger_joint2']) {
    const fingerJointId = findNamedIndex(model.njnt, model.name_jntadr, fingerName);
    if (fingerJointId === undefined) throw new Error(`Missing ${fingerName}`);
    fingerJointIds.push(fingerJointId);
    data.qpos[model.jnt_qposadr[fingerJointId]] = .04;
  }

  mujoco.mj_forward(model, data);
  const rotation = Array.from(data.site_xmat.slice(siteId * 9, siteId * 9 + 9));
  const trace = rotation[0] + rotation[4] + rotation[8];
  const quaternion = [0, 0, 0, 0];
  if (trace > 0) {
    const scale = Math.sqrt(trace + 1) * 2;
    quaternion[0] = .25 * scale;
    quaternion[1] = (rotation[7] - rotation[5]) / scale;
    quaternion[2] = (rotation[2] - rotation[6]) / scale;
    quaternion[3] = (rotation[3] - rotation[1]) / scale;
  } else if (rotation[0] > rotation[4] && rotation[0] > rotation[8]) {
    const scale = Math.sqrt(1 + rotation[0] - rotation[4] - rotation[8]) * 2;
    quaternion[0] = (rotation[7] - rotation[5]) / scale;
    quaternion[1] = .25 * scale;
    quaternion[2] = (rotation[1] + rotation[3]) / scale;
    quaternion[3] = (rotation[2] + rotation[6]) / scale;
  } else if (rotation[4] > rotation[8]) {
    const scale = Math.sqrt(1 + rotation[4] - rotation[0] - rotation[8]) * 2;
    quaternion[0] = (rotation[2] - rotation[6]) / scale;
    quaternion[1] = (rotation[1] + rotation[3]) / scale;
    quaternion[2] = .25 * scale;
    quaternion[3] = (rotation[5] + rotation[7]) / scale;
  } else {
    const scale = Math.sqrt(1 + rotation[8] - rotation[0] - rotation[4]) * 2;
    quaternion[0] = (rotation[3] - rotation[1]) / scale;
    quaternion[1] = (rotation[2] + rotation[6]) / scale;
    quaternion[2] = (rotation[5] + rotation[7]) / scale;
    quaternion[3] = .25 * scale;
  }
  const rotatedGrasp = [0, 1, 2].map((row) =>
    rotation[row * 3] * graspLocal[0]
      + rotation[row * 3 + 1] * graspLocal[1]
      + rotation[row * 3 + 2] * graspLocal[2],
  );
  const qposAddress = model.jnt_qposadr[toolJointId];
  for (let axis = 0; axis < 3; axis += 1) {
    data.qpos[qposAddress + axis] = data.site_xpos[siteId * 3 + axis] - rotatedGrasp[axis];
  }
  for (let axis = 0; axis < 4; axis += 1) {
    data.qpos[qposAddress + 3 + axis] = quaternion[axis];
  }
  data.ctrl[gripperActuatorId] = 255;
  model.opt.gravity.fill(0);
  mujoco.mj_forward(model, data);
  for (let step = 0; step < 100; step += 1) mujoco.mj_step(model, data);
  data.ctrl[gripperActuatorId] = 0;
  for (let step = 0; step < 750; step += 1) mujoco.mj_step(model, data);

  const contactCount = () => {
    let count = 0;
    for (let index = 0; index < data.ncon; index += 1) {
      const contact = data.contact.get(index);
      if (
        contact
        && (model.geom_bodyid[contact.geom1] === toolBodyId
          || model.geom_bodyid[contact.geom2] === toolBodyId)
      ) count += 1;
    }
    return count;
  };
  const closedContactCount = contactCount();
  const closedForce = data.actuator_force[gripperActuatorId];
  const closedFingerPositions = fingerJointIds.map((jointId) => data.qpos[model.jnt_qposadr[jointId]]);
  const toolHeightBeforeGravity = data.xpos[toolBodyId * 3 + 2];
  model.opt.gravity[2] = -9.81;
  for (let step = 0; step < 1_000; step += 1) mujoco.mj_step(model, data);
  const toolHeightAfterGravity = data.xpos[toolBodyId * 3 + 2];
  const heightLoss = toolHeightBeforeGravity - toolHeightAfterGravity;
  console.log(
    `grasp ${toolName}: closedContacts=${closedContactCount} `
      + `closedForce=${closedForce.toFixed(2)}N fingers=${closedFingerPositions.map((value) => value.toFixed(4)).join(',')} `
      + `heldContacts=${contactCount()} heldForce=${data.actuator_force[gripperActuatorId].toFixed(2)}N `
      + `heightLoss=${heightLoss.toFixed(4)}m`,
  );
  if (contactCount() < 2 || heightLoss > .04) {
    throw new Error(`Physical ${toolName} grasp did not survive the gravity hold`);
  }
  data.delete();
}
model.delete();
