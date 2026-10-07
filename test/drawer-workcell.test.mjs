import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {drawerEngine} from './helpers/drawer-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';

test('sourced drawers compile as passive fixtures and reject stale start state',async()=>{
  assert.ok(existsSync('public/assets/franka-cooperative/drawer.xml'),'drawer workcell has not been built');
  const {mj,m,d,find,q}=await drawerEngine();
  try{
    assert.equal(m.nu,32,'only four Panda actuator groups; no drawer motors');
    const manifest=JSON.parse(readFileSync('public/assets/franka-cooperative/drawer-manifest.json'));
    assert.ok(manifest.sources.length>=3);
    assert.ok(manifest.sources.every(s=>s.sha256?.length===64));
    for(const name of ['south_slide','north_slide']){
      const j=find.joint(name);assert.ok(j>=0);assert.equal(m.jnt_type[j],mj.mjtJoint.mjJNT_SLIDE.value);
      assert.ok(Math.abs(d.qpos[m.jnt_qposadr[j]])<.006,'drawer stays closed at rest');
    }
    for(const name of ['tea_box','coffee_box'])assert.ok(d.xpos[find.body(name)*3+2]>.12,'objects rest inside drawers, not through the floor');
    const p={version:1,scene:'drawer',initialJoints:q,initialObjects:[],
      initialFixtureJoints:[{joint:'south_slide',position:0,tolerance:.006}],
      phases:[{name:'Confirm closed',duration:.2,paths:Array.from({length:4},()=>[[...q],[...q]]),
        grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],
        gates:[{type:'joint-range',joint:'south_slide',min:-.006,max:.006,maxSpeed:.015}]}]};
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const before=Array.from(d.qpos);r.step(m,d);assert.deepEqual(Array.from(d.qpos),before);
    const g=r.observeGate(m,d,p.phases[0].gates[0]);assert.ok(Math.abs(g.position)<.006);assert.ok(g.speed<.015);
    d.qpos[m.jnt_qposadr[find.joint('south_slide')]]=-.1;mj.mj_forward(m,d);
    assert.throws(()=>r.start(m,d),/fixture moved/,'manual fixture movement must require Reset, not teleport');
  }finally{d.delete();m.delete();}
});
