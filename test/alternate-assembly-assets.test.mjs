import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const asset = (relativePath) => new URL(`../public/assets/${relativePath}`, import.meta.url);

test('alternate assembly scene roots load only local model packages', async () => {
  const [piperScene, urScene] = await Promise.all([
    readFile(asset('piper-assembly1/scene.xml'), 'utf8'),
    readFile(asset('ur5e-assembly1/scene.xml'), 'utf8'),
  ]);

  assert.match(piperScene, /<model name="piper_model" file="robots\/piper\/piper\.xml"\/>/);
  assert.match(urScene, /<model name="ur5e_model" file="robots\/ur5e\/ur5e\.xml"\/>/);
  assert.doesNotMatch(`${piperScene}\n${urScene}`, /https?:\/\//);
});

test('PiPER assembly package exposes a replicable robot with a TCP site', async () => {
  const model = await readFile(asset('piper-assembly1/robots/piper/piper.xml'), 'utf8');

  assert.match(model, /<body name="base_link"/);
  assert.match(model, /<site name="tcp"/);
  assert.match(model, /<position name="gripper"/);
  assert.doesNotMatch(model, /<keyframe>/);
});

test('UR5e assembly package composes the Robotiq subtree at its wrist', async () => {
  const model = await readFile(asset('ur5e-assembly1/robots/ur5e/ur5e.xml'), 'utf8');

  assert.match(
    model,
    /<model name="robotiq_model" file="\.\.\/robotiq_2f85\/2f85\.xml"\/>/,
  );
  assert.match(
    model,
    /<attach model="robotiq_model" body="base_mount" prefix="gripper_"\/>/,
  );
  assert.doesNotMatch(model, /<keyframe>/);
});

test('alternate assembly packages retain every upstream robot license', async () => {
  const [piperLicense, urLicense, gripperLicense, piperNotice, urNotice] = await Promise.all([
    readFile(asset('piper-assembly1/robots/piper/LICENSE'), 'utf8'),
    readFile(asset('ur5e-assembly1/robots/ur5e/LICENSE'), 'utf8'),
    readFile(asset('ur5e-assembly1/robots/robotiq_2f85/LICENSE'), 'utf8'),
    readFile(asset('piper-assembly1/THIRD_PARTY_NOTICES.md'), 'utf8'),
    readFile(asset('ur5e-assembly1/THIRD_PARTY_NOTICES.md'), 'utf8'),
  ]);

  assert.match(piperLicense, /MIT License/);
  assert.match(urLicense, /Redistribution and use/);
  assert.match(gripperLicense, /Redistribution and use/);
  assert.match(piperNotice, /MuJoCo Menagerie[\s\S]*AgileX PiPER[\s\S]*MIT/);
  assert.match(urNotice, /UR5e[\s\S]*BSD-3-Clause[\s\S]*Robotiq 2F-85[\s\S]*BSD-2-Clause/);
});
