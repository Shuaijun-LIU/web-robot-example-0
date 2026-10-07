import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync} from 'node:fs';
import {insertionEngine} from './helpers/insertion-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';
import {checkCooperativeGate} from '../src/cooperativeMotion.js';

test('source insertion parts are free and same-center rotation invalidates playback',async()=>{
  assert.ok(existsSync('public/assets/franka-cooperative/insertion.xml'),'insertion scene not exported');
  const {mj,m,d,find,q}=await insertionEngine();
  try{
    assert.equal(m.nu,32);
    assert.ok(Array.from(m.eq_type).every(t=>t===mj.mjtEq.mjEQ_JOINT.value),'only source gripper joint couplings, never object welds');
    for(const name of ['stand','insert']){
      const b=find.body(name),j=m.body_jntadr[b];assert.equal(m.jnt_type[j],mj.mjtJoint.mjJNT_FREE.value);
      assert.ok(d.xpos[b*3+2]>.13);
    }
    const b=find.body('stand'),p={version:1,scene:'insertion',initialJoints:q,
      initialObjects:[{name:'stand',position:Array.from(d.xpos.slice(b*3,b*3+3)),quaternion:Array.from(d.xquat.slice(b*4,b*4+4)),orientationTolerance:.12}],
      phases:[{name:'Hold',duration:.2,paths:Array.from({length:4},()=>[[...q],[...q]]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[]}]};
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);const before=Array.from(d.qpos);r.step(m,d);assert.deepEqual(Array.from(d.qpos),before);
    const qa=m.jnt_qposadr[m.body_jntadr[b]];d.qpos.set([Math.cos(.2),0,0,Math.sin(.2)],qa+3);mj.mj_forward(m,d);
    assert.throws(()=>r.start(m,d),/orientation/);
  }finally{d.delete();m.delete();}
});

test('socket cavity is open while a rod crossing its wall triggers the contact guard',async()=>{
  const {mj,m,d,find,q}=await insertionEngine();
  try{
    const p={version:1,scene:'insertion',initialJoints:q,initialObjects:[],contactLimits:[{a:'stand',b:'insert',maxPenetration:.001}],
      phases:[{name:'Inspect cavity',duration:1,paths:Array.from({length:4},()=>[[...q],[...q]]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[]}]};
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const stand=find.body('stand'),insert=find.body('insert'),qa=m.jnt_qposadr[m.body_jntadr[insert]];
    // Independent analytic fixture: upright rod, its lower tip 50 mm below
    // the 160-mm stand's mouth; the oversized grip remains above the rim.
    const origin=[d.xpos[stand*3],d.xpos[stand*3+1]+.045+.04375,d.xpos[stand*3+2]+.08-.05+.09];
    d.qpos.set([...origin,Math.SQRT1_2,0,0,-Math.SQRT1_2],qa);mj.mj_forward(m,d);r.contacts(m,d);
    assert.equal(r.state.phase,'running');assert.equal(r.metrics.maxLimitedContactPenetration,0,'rod has a genuine free cavity');
    d.qpos[qa]+=.013;mj.mj_forward(m,d);r.contacts(m,d);
    assert.equal(r.state.phase,'error');assert.match(r.state.reason,/object-contact/);
    assert.ok(r.metrics.maxLimitedContactPenetration>.001,'wall crossing is physical contact, not only a visual hole');
  }finally{d.delete();m.delete();}
});

test('actual body observations reject shallow, off-axis and unsupported insertions',async()=>{
  const {mj,m,d,find,q}=await insertionEngine();
  try{
    const p={version:1,scene:'insertion',initialJoints:q,initialObjects:[],
      phases:[{name:'Measure',duration:1,paths:Array.from({length:4},()=>[[...q],[...q]]),
        grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[]}]};
    const g={type:'inserted',object:'insert',socket:'stand',tip:[.04375,0,-.09],mouth:[0,.045,.08],
      axis:[0,0,1],socketAxis:[0,0,1],minDepth:.095,maxDepth:.13,maxLateral:.006,minAlignment:.98,maxSpeed:.025};
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const stand=find.body('stand'),insert=find.body('insert'),qa=m.jnt_qposadr[m.body_jntadr[insert]];
    const origin=[.02,-.06,.22],yaw=.4,c=Math.cos(yaw),s=Math.sin(yaw),quat=[Math.cos(yaw/2),0,0,Math.sin(yaw/2)];
    d.qpos.set([...origin,...quat],m.jnt_qposadr[m.body_jntadr[stand]]);d.qvel.fill(0);
    // Test fixture only: prescribe free-body states to independently probe FK
    // observations. Production motion never writes these body poses.
    for(const [depth,lateral,contact] of [[.05,0,false],[.10,.04,false],[.10,0,false],[.1070001,0,true]]){
      const x=-.04375+lateral,y=.045;
      d.qpos.set([origin[0]+c*x-s*y,origin[1]+s*x+c*y,origin[2]+.08-depth+.09,...quat],qa);
      mj.mj_forward(m,d);r.contacts(m,d);const o=r.observeGate(m,d,g);
      assert.ok(Math.abs(o.depth-depth)<1e-6);assert.ok(Math.abs(o.lateral-lateral)<1e-6);
      assert.ok(o.alignment>.99999);assert.equal(o.supported,contact);
      assert.equal(checkCooperativeGate(g,o)===null,contact,JSON.stringify(o));
    }
  }finally{d.delete();m.delete();}
});
