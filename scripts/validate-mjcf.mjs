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

const definitions = {
  franka: { layout: FRANKA_LAYOUT, sceneFile: 'scene.xml' },
  so101: { layout: SO101_LAYOUT, sceneFile: 'objects_SO101.xml' },
  xlerobot: { layout: XLEROBOT_LAYOUT, sceneFile: 'objects.xml' },
  frankaAssembly1: { layout: FRANKA_ASSEMBLY1_LAYOUT, sceneFile: 'scene.xml' },
  frankaAssembly2: { layout: FRANKA_ASSEMBLY2_LAYOUT, sceneFile: 'scene.xml' },
};
const [sceneKey, assetDirectory] = process.argv.slice(2);
const definition = definitions[sceneKey];

if (!definition || !assetDirectory) {
  throw new Error('Usage: node scripts/validate-mjcf.mjs <franka|so101|xlerobot|frankaAssembly1|frankaAssembly2> <asset-directory>');
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
    xlerobot: 'chassis',
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
model.delete();
