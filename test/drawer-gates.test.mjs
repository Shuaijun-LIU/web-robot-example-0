import test from 'node:test';
import assert from 'node:assert/strict';
import {validateCooperativePlan,checkCooperativeGate} from '../src/cooperativeMotion.js';
import {COOPERATIVE_WORKCELLS,cooperativeSceneForKey} from '../src/cooperativeWorkcells.js';

const q=[0,-.5,0,-2,0,1.5,0];
const gate=()=>({type:'joint-range',joint:'south_slide',min:-.205,max:-.175,maxSpeed:.015});
const plan=()=>({version:1,scene:'drawer',initialJoints:q,initialObjects:[],
  initialFixtureJoints:[{joint:'south_slide',position:0,tolerance:.006}],
  phases:[{name:'Check opening',duration:1,paths:Array.from({length:4},()=>[[...q],[...q]]),
    grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[gate()]}]});

test('drawer scene maps to its own four-arm program and leaves old pages mapped',()=>{
  assert.equal(cooperativeSceneForKey('frankaDemo5'),'drawer');
  assert.equal(COOPERATIVE_WORKCELLS.drawer?.homeJoints.length,32);
  assert.equal(cooperativeSceneForKey('frankaDemo3'),'scan');
  assert.equal(cooperativeSceneForKey('frankaDemo4'),'pot');
});
test('passive joint gates accept only measured settled positions within bounds',()=>{
  assert.equal(checkCooperativeGate(gate(),{position:-.19,speed:.001}),null);
  for(const o of [{position:0,speed:0},{position:-.19,speed:.1},{position:NaN,speed:0},{position:-.19,speed:NaN},{}])
    assert.ok(checkCooperativeGate(gate(),o));
});
test('drawer plans reject malformed joint gates and reset tolerances',()=>{
  assert.equal(validateCooperativePlan(plan(),'drawer'),null);
  for(const patch of [{min:NaN},{max:-.3},{maxSpeed:-1},{joint:''}]){
    const p=plan();Object.assign(p.phases[0].gates[0],patch);assert.ok(validateCooperativePlan(p,'drawer'));
  }
  for(const patch of [{position:Infinity},{tolerance:-.1},{tolerance:NaN},{joint:''}]){
    const p=plan();Object.assign(p.initialFixtureJoints[0],patch);assert.ok(validateCooperativePlan(p,'drawer'));
  }
});
test('packing checks the assigned tray slot rather than just the tray center',()=>{
  const g={type:'inside',object:'tea_box',container:'order_tray',radius:.14,minZ:0,maxZ:.09,minX:.02,maxX:.12};
  assert.equal(checkCooperativeGate(g,{supported:true,speed:0,relative:[.07,0,.04]}),null);
  assert.ok(checkCooperativeGate(g,{supported:true,speed:0,relative:[-.07,0,.04]}));
  const p=plan();p.phases[0].gates=[{...g,minX:NaN}];assert.ok(validateCooperativePlan(p,'drawer'));
});
