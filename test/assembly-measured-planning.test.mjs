import assert from 'node:assert/strict';
import test from 'node:test';
import { actualInsertionGeometry } from '../src/assemblyMeasuredPlanning.js';
import { HAMMER_COLLISION_MESHES } from '../src/hammerCollisionGeometry.js';
import { FRANKA_ASSEMBLY1_LAYOUT } from '../src/frankaAssemblyLayouts.js';
import loadMujoco from 'mujoco-js';

test('insertion checks the actual shifted beam hole and shaft tip, not the nominal frame marker',()=>{
  const names=new TextEncoder().encode('cross_member\0fastener_1\0');
  const model={nbody:2,name_bodyadr:[0,13],names};
  const data={xpos:[-.092,.009,.278,-.130,.224,.347],xquat:[1,0,0,0,1,0,0,0]};
  const g=actualInsertionGeometry(model,data);
  assert.ok(Math.abs(g.depth-.004)<1e-9);
  assert.ok(g.radial<1e-9);
  assert.equal(g.tilt,0);
  data.xpos[3]+=.01;
  assert.ok(Math.abs(actualInsertionGeometry(model,data).radial-.01)<1e-9);
});

test('hammer contact meshes preserve the real curved grip and contain no invented 48 mm rib',()=>{
  const receiver=HAMMER_COLLISION_MESHES.filter(p=>p.name.startsWith('handle_')&&p.vertex.some((_,i)=>i%3===0&&Math.abs(p.vertex[i]+.045)<.002));
  assert.ok(receiver.length>=1);
  for(const p of receiver){
    const ys=p.vertex.filter((_,i)=>i%3===1),zs=p.vertex.filter((_,i)=>i%3===2);
    assert.ok(Math.max(...ys)-Math.min(...ys)<.048);
    assert.ok(Math.min(...zs)>-.024&&Math.max(...zs)<.009);
  }
  assert.ok(HAMMER_COLLISION_MESHES.some(p=>p.name.startsWith('head_')));
});

test('attached Panda models retain implicit integration through an explicit parent option',async()=>{
  const patch=FRANKA_ASSEMBLY1_LAYOUT.xmlPatches.find(p=>p.target==='scene.xml'&&p.inject?.includes('<option'));
  assert.ok(patch);
  const m=await loadMujoco();
  m.FS.writeFile('/audit-child.xml','<mujoco><option integrator="implicitfast"/><worldbody><body name="b"><joint/><geom size=".1"/></body></worldbody></mujoco>');
  m.FS.writeFile('/audit-parent.xml',`<mujoco>${patch.inject}<asset><model name="child" file="/audit-child.xml"/></asset><worldbody><attach model="child" body="b" prefix="r_"/></worldbody></mujoco>`);
  const model=m.MjModel.loadFromXML('/audit-parent.xml');
  try {assert.equal(model.opt.integrator,3);assert.ok(Number(model.narena)>=64*1024*1024);} finally {model.delete();}
});
