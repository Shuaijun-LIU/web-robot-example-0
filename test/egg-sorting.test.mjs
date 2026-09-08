import assert from 'node:assert/strict';
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { resolve, relative } from 'node:path';
import test from 'node:test';
import loadMujoco from 'mujoco-js';
import { consumeMujocoContacts } from '../src/mujocoContact.js';
import { EGG_SORTING_LAYOUT } from '../src/eggSortingLayout.js';

// Catches omitted assets, broken attachment names, locked eggs and wrong base height.
test('egg workcell compiles four independently actuated Pandas and sixteen free eggs', async () => {
  const root = resolve('public/assets/franka-egg-sorting');
  assert.ok(existsSync(`${root}/scene.xml`), 'new local egg scene must exist');
  const mj = await loadMujoco();
  mj.FS.mkdir('/eggs');
  function mount(dir) {
    for (const entry of readdirSync(dir, { withFileTypes: true })) {
      const full = resolve(dir, entry.name), dest = '/eggs/' + relative(root, full);
      if (entry.isDirectory()) { mj.FS.mkdir(dest); mount(full); }
      else mj.FS.writeFile(dest, readFileSync(full));
    }
  }
  mount(root);
  const model = mj.MjModel.loadFromXML('/eggs/scene.xml');
  assert.ok(model, 'scene must compile');
  const data = new mj.MjData(model);
  // Match the actual page/Reset initializer: actuator homes for scalar arm
  // joints, free bodies from XML, tendon-driven jaw positions start at zero.
  data.ctrl.set(EGG_SORTING_LAYOUT.homeJoints);
  for(let arm=0;arm<4;arm++) for(let j=0;j<7;j++) {
    const id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT.value,`r${arm}_joint${j+1}`);
    data.qpos[model.jnt_qposadr[id]]=EGG_SORTING_LAYOUT.homeJoints[arm*8+j];
  }
  mj.mj_forward(model, data);
  assert.equal(model.nu, 32);
  assert.equal(model.nq, 148); // 4 × (7 arm + 2 jaw) + 16 × 7 free-body coordinates.
  for (let i = 0; i < 4; i++) {
    const base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, `r${i}_link0`);
    assert.ok(base > 0);
    assert.ok(Math.abs(data.xpos[base * 3 + 2] - .1) < 1e-7);
    assert.ok(mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE.value, `r${i}_tcp`) >= 0);
    assert.ok(mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR.value, `r${i}_gripper`) >= 0);
    assert.ok(mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, `output_tray_${i}`) > 0);
  }
  for (let i = 0; i < 16; i++) {
    const egg = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, `egg_${i}`);
    assert.ok(egg > 0);
    assert.equal(model.body_jntnum[egg], 1);
    assert.ok(Math.abs(data.xpos[egg * 3]) < .23);
    assert.ok(Math.abs(data.xpos[egg * 3 + 1]) < .23);
    assert.ok(model.body_mass[egg] >= .035 && model.body_mass[egg] <= .08);
  }
  assert.ok(Array.from(data.qpos).every(Number.isFinite));
  // A larger end effector must not inherit an old folded pose that hits link 1.
  for (let tick = 0; tick < 250; tick++) mj.mj_step(model, data);
  const robotBodies = new Set(Array.from({length:4},(_,i)=>`r${i}_`));
  const name = id => mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY.value, model.geom_bodyid[id]);
  const selfContacts = consumeMujocoContacts(data.contact, data.ncon).filter(c => {
    const a=name(c.geom1), b=name(c.geom2);
    return [...robotBodies].some(prefix=>a?.startsWith(prefix)&&b?.startsWith(prefix)) && c.distance < -.0003;
  });
  assert.deepEqual(selfContacts, [], 'long gripper must clear the folded robot links');
  const eggIds=Array.from({length:16},(_,i)=>mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY.value,`egg_${i}`));
  const initial=eggIds.map(id=>Array.from(data.xpos.slice(id*3,id*3+3)));
  for(let tick=0;tick<2500;tick++) mj.mj_step(model,data);
  const contacts=consumeMujocoContacts(data.contact,data.ncon);
  eggIds.forEach((id,i)=>{
    const position=Array.from(data.xpos.slice(id*3,id*3+3));
    assert.ok(Math.hypot(...position.map((v,j)=>v-initial[i][j]))<.002,`egg ${i} must remain settled within 2 mm over 5 seconds`);
    assert.ok(contacts.some(c=>model.geom_bodyid[c.geom1]===id||model.geom_bodyid[c.geom2]===id),`egg ${i} needs real support`);
  });
  const manifest=JSON.parse(readFileSync(`${root}/manifest.json`,'utf8'));
  for(let quadrant=0;quadrant<4;quadrant++) assert.equal(new Set(manifest.eggs.slice(quadrant*4,quadrant*4+4).map(e=>e.classIndex)).size,4,'each source insert must actually mix all four classes');
  data.delete(); model.delete();
});
