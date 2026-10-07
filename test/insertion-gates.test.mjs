import test from 'node:test';
import assert from 'node:assert/strict';
import {validateCooperativePlan,checkCooperativeGate} from '../src/cooperativeMotion.js';
import {cooperativeSceneForKey} from '../src/cooperativeWorkcells.js';
const q=[0,-.5,0,-2,0,1.5,0];
const gate=()=>({type:'inserted',object:'insert',socket:'stand',tip:[.04375,0,-.09],mouth:[0,.045,.08],
  axis:[0,0,1],socketAxis:[0,0,1],minDepth:.095,maxDepth:.13,maxLateral:.006,minAlignment:.98,maxSpeed:.025});
const plan=()=>({version:1,scene:'insertion',initialJoints:q,initialObjects:[{name:'stand',position:[0,0,.2],quaternion:[1,0,0,0],orientationTolerance:.12}],
  contactLimits:[{a:'stand',b:'insert',maxPenetration:.001}],phases:[{name:'Seat',duration:1,
    paths:Array.from({length:4},()=>[[...q],[...q]]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[gate()]}]});

test('Demo6 selects an isolated insertion program',()=>assert.equal(cooperativeSceneForKey('frankaDemo6'),'insertion'));
test('insertion requires real contact, depth, alignment and settling together',()=>{
  const good={depth:.10,lateral:.002,alignment:.999,supported:true,speed:.001};
  assert.equal(checkCooperativeGate(gate(),good),null);
  for(const patch of [{depth:.01},{depth:.20},{lateral:.02},{alignment:.8},{supported:false},{speed:.1},{depth:NaN},{lateral:NaN},{alignment:NaN},{speed:NaN}])
    assert.ok(checkCooperativeGate(gate(),{...good,...patch}),JSON.stringify(patch));
});
test('insertion programs reject malformed geometry and physical limits',()=>{
  assert.equal(validateCooperativePlan(plan(),'insertion'),null);
  for(const patch of [{axis:[0,0,0]},{socketAxis:[0,0,2]},{mouth:[0,NaN,0]},{tip:[]},{minDepth:-1},{maxDepth:.05},{maxLateral:-1},{minAlignment:2},{maxSpeed:NaN},{socket:''}]){
    const p=plan();Object.assign(p.phases[0].gates[0],patch);assert.ok(validateCooperativePlan(p,'insertion'));
  }
  for(const patch of [{quaternion:[0,0,0,0]},{orientationTolerance:NaN},{orientationTolerance:-.1}]){
    const p=plan();Object.assign(p.initialObjects[0],patch);assert.ok(validateCooperativePlan(p,'insertion'));
  }
  const p=plan();p.contactLimits[0].maxPenetration=NaN;assert.ok(validateCooperativePlan(p,'insertion'));
});
