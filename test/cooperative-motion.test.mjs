import test from 'node:test';
import assert from 'node:assert/strict';
import {validateCooperativePlan,sampleCooperativePhase,checkCooperativeGate,loadCooperativePlan} from '../src/cooperativeMotion.js';

const q=[0,-.5,0,-2,0,1.5,0];
const phase=()=>({name:'Approach',duration:2,paths:Array.from({length:4},()=>[[...q],[...q.slice(0,6),.2]]),grippers:Array.from({length:4},()=>[255,0]),allowedContacts:[],carry:[],gates:[]});
const plan=()=>({version:1,scene:'scan',initialJoints:q,initialObjects:[],phases:[phase()]});

test('unsafe or non-finite commands cannot load as an executable plan',()=>{
  assert.equal(validateCooperativePlan(plan(),'scan'),null);
  for(const value of [NaN,Infinity,9]){const p=plan();p.phases[0].paths[2][1][0]=value;assert.ok(validateCooperativePlan(p,'scan'));}
  const wrong=plan();wrong.phases[0].grippers[0][1]=256;assert.ok(validateCooperativePlan(wrong,'scan'));
  assert.ok(validateCooperativePlan(plan(),'pot'));
  const broken=plan();broken.phases[0].paths.pop();assert.ok(validateCooperativePlan(broken,'scan'));
  const gate=plan();gate.phases[0].gates=[{type:'inside',object:'tomato',container:'pot',radius:NaN,minZ:0,maxZ:.1}];assert.ok(validateCooperativePlan(gate,'scan'));
});
test('phase sampling clamps endpoints and produces a continuous minimum-jerk midpoint',()=>{
  const p=phase();
  assert.equal(sampleCooperativePhase(p,-1).joints[0][6],0);
  assert.equal(sampleCooperativePhase(p,.5).joints[0][6],.1);
  assert.equal(sampleCooperativePhase(p,.5).grippers[0],127.5);
  assert.equal(sampleCooperativePhase(p,2).joints[0][6],.2);
});
test('grasp, release and support gates use physical contact rather than proximity',()=>{
  const o={bilateral:false,fingerContacts:0,supported:false,speed:0,aperture:.035};
  assert.ok(checkCooperativeGate({type:'grasp'},o));
  assert.equal(checkCooperativeGate({type:'grasp'},{...o,bilateral:true,fingerContacts:2}),null);
  assert.ok(checkCooperativeGate({type:'support'},o));
  assert.equal(checkCooperativeGate({type:'support'},{...o,supported:true}),null);
  assert.ok(checkCooperativeGate({type:'released'},{...o,fingerContacts:1}));
  assert.equal(checkCooperativeGate({type:'released'},o),null);
});
test('inside and scan gates reject the wrong geometric destination',()=>{
  assert.ok(checkCooperativeGate({type:'inside',radius:.065,minZ:0,maxZ:.11},{relative:[.15,0,.05],supported:true,speed:0}));
  assert.equal(checkCooperativeGate({type:'inside',radius:.065,minZ:0,maxZ:.11},{relative:[.02,0,.04],supported:true,speed:0}),null);
  assert.ok(checkCooperativeGate({type:'scan'},{distance:.05,offAxis:.05,facing:1}));
  assert.equal(checkCooperativeGate({type:'scan'},{distance:.05,offAxis:.005,facing:.98}),null);
});
test('static hosting HTML and HTTP errors are reported before JSON parsing',async()=>{
  await assert.rejects(loadCooperativePlan(async()=>new Response('<html>Missing</html>',{status:404}),'scan','/motion.json'),/HTTP 404/);
  await assert.rejects(loadCooperativePlan(async()=>new Response('<html>Missing</html>',{headers:{'content-type':'text/html'}}),'scan','/motion.json'),/JSON/);
  const loaded=await loadCooperativePlan(async()=>new Response(JSON.stringify(plan()),{headers:{'content-type':'application/json'}}),'scan','/motion.json');
  assert.equal(loaded.scene,'scan');
});
