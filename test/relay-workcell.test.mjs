import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync,readFileSync} from 'node:fs';
import {relayEngine} from './helpers/relay-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';
import {checkCooperativeGate} from '../src/cooperativeMotion.js';
const root='public/assets/franka-cooperative';
test('panel collision uses visible source vertices instead of an inflated envelope',()=>{
  assert.ok(existsSync(`${root}/relay.xml`),'relay scene not exported');
  const vertices=file=>readFileSync(`${root}/assets/${file}`,'utf8').split('\n').filter(l=>l.startsWith('v ')).map(l=>l.trim().split(/\s+/).slice(1).map(Number));
  const visual=vertices('relay_panel-visual-0.obj'),collision=vertices('relay_panel-collision-0.obj');
  assert.ok(visual.length>100&&collision.length>8);
  const keys=new Set(visual.map(v=>v.map(n=>n.toFixed(6)).join(',')));
  assert.ok(collision.every(v=>keys.has(v.map(n=>n.toFixed(6)).join(','))),'collision has no padded/off-mesh vertices');
  for(let j=0;j<3;j++){
    const vv=visual.map(v=>v[j]),cc=collision.map(v=>v[j]);
    assert.ok(Math.abs(Math.max(...vv)-Math.max(...cc))<1e-6);
    assert.ok(Math.abs(Math.min(...vv)-Math.min(...cc))<1e-6);
  }
});
test('free panel is supported and real rotated-body observations reject a tipped payload',async()=>{
  assert.ok(existsSync(`${root}/relay.xml`),'relay scene not exported');
  const {mj,m,d,find,q}=await relayEngine();
  try{
    assert.equal(m.nu,32);assert.equal(Array.from(m.jnt_type).filter(t=>t===mj.mjtJoint.mjJNT_FREE.value).length,1);
    assert.ok(Array.from(m.eq_type).every(t=>t===mj.mjtEq.mjEQ_JOINT.value));
    const b=find.body('panel'),g={type:'upright',object:'panel',axis:[0,0,1],minAlignment:Math.cos(Math.PI/18),maxSpeed:.025};
    const p={version:1,scene:'relay',initialJoints:q,initialObjects:[],phases:[{name:'Inspect',duration:1,paths:Array.from({length:4},()=>[q,q]),
      grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[]}]};
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);r.contacts(m,d);
    assert.equal(checkCooperativeGate(g,r.observeGate(m,d,g)),null);
    assert.equal(r.observeGate(m,d,{type:'support',object:'panel',body:'panel_start'}).supported,true);
    const qa=m.jnt_qposadr[m.body_jntadr[b]];d.qpos.set([Math.cos(.2),Math.sin(.2),0,0],qa+3);mj.mj_forward(m,d);
    assert.equal(checkCooperativeGate(g,r.observeGate(m,d,g)),'object-not-upright');
  }finally{d.delete();m.delete();}
});
