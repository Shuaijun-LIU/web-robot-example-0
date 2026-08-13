import assert from 'node:assert/strict';
import test from 'node:test';
import loadMujoco from 'mujoco-js';
import * as THREE from 'three';

import * as selectedIkSolver from '../src/controllers/selectedIkSolver.js';

const { fitJointAngleToRange, solveSelectedIk } = selectedIkSolver;

const XML = `
<mujoco model="selected_ik_test">
  <compiler angle="radian"/>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="arm0">
      <joint name="joint0" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.02"/>
      <body pos="1 0 0"><site name="tcp0" size="0.01"/></body>
    </body>
    <body name="arm1" pos="0 2 0">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.02"/>
      <body pos="1 0 0"><site name="tcp1" size="0.01"/></body>
    </body>
  </worldbody>
  <actuator>
    <position name="act0" joint="joint0" kp="100"/>
    <position name="act1" joint="joint1" kp="100"/>
  </actuator>
</mujoco>`;

function getName(model, address) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

test('joint-limit fitting preserves an equivalent revolute pose before clamping', () => {
  assert.equal(typeof fitJointAngleToRange, 'function');
  assert.ok(Math.abs(
    fitJointAngleToRange(3.8465730529, -2.8973, 2.8973) - (-2.4366122543),
  ) < 1e-9);
  assert.equal(fitJointAngleToRange(1.2, -2.8973, 2.8973), 1.2);
  assert.equal(fitJointAngleToRange(8, -1, 1), 1);
});

test('selected IK perturbs the selected qpos address and restores the full model state', async () => {
  const mujoco = await loadMujoco();
  try { mujoco.FS.mkdir('/working'); } catch { /* reused module */ }
  mujoco.FS.writeFile('/working/selected-ik.xml', XML);
  const model = mujoco.MjModel.loadFromXML('/working/selected-ik.xml');
  const data = new mujoco.MjData(model);

  try {
    data.qpos[0] = 0.25;
    data.qpos[1] = -0.2;
    mujoco.mj_forward(model, data);
    const siteId = Array.from({ length: model.nsite }, (_, index) => index)
      .find((index) => getName(model, model.name_siteadr[index]) === 'tcp1');
    assert.notEqual(siteId, undefined);

    const result = solveSelectedIk({
      mujoco,
      model,
      data,
      siteId,
      qposAddresses: [1],
      currentQ: [-0.2],
      targetPosition: new THREE.Vector3(0, 3, 0),
      targetQuaternion: new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1),
        Math.PI / 2,
      ),
      damping: 0.001,
      maxIterations: 100,
    });

    assert.ok(result);
    assert.ok(Math.abs(result[0] - Math.PI / 2) < 0.05, `solution was ${result[0]}`);
    assert.ok(Math.abs(data.qpos[0] - 0.25) < 1e-12);
    assert.ok(Math.abs(data.qpos[1] + 0.2) < 1e-12);
  } finally {
    data.delete();
    model.delete();
  }
});
