import test from 'node:test';
import assert from 'node:assert/strict';
import {checkCooperativeGate,validateCooperativePlan} from '../src/cooperativeMotion.js';
import {cooperativeSceneForKey} from '../src/cooperativeWorkcells.js';
const q=[1.570796,-.785398,0,-2.356194,0,1.570796,.785398];
const gate={type:'upright',object:'panel',axis:[0,0,1],minAlignment:Math.cos(Math.PI/18),maxSpeed:.025};
const plan=g=>({version:1,scene:'relay',initialJoints:q,initialObjects:[],phases:[{name:'Hold',duration:1,
  paths:Array.from({length:4},()=>[q,q]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[g]}]});
test('relay is independently selectable and a finite level panel passes',()=>{
  assert.equal(cooperativeSceneForKey('frankaDemo7'),'relay');
  assert.equal(validateCooperativePlan(plan(gate),'relay'),null);
  assert.equal(checkCooperativeGate(gate,{alignment:1,speed:.001}),null);
});
test('tilt, motion and nonfinite upright observations cannot pass',()=>{
  for(const o of [{alignment:.9,speed:0},{alignment:1,speed:.1},{alignment:NaN,speed:0},{alignment:1,speed:NaN},{alignment:2,speed:0},{alignment:1,speed:-1}])
    assert.equal(checkCooperativeGate(gate,o),'object-not-upright');
});
test('upright gate rejects malformed axes and thresholds',()=>{
  for(const patch of [{axis:[0,0,2]},{axis:[0,NaN,1]},{minAlignment:0},{minAlignment:1.1},{maxSpeed:-1}])
    assert.equal(validateCooperativePlan(plan({...gate,...patch}),'relay'),'Invalid physical gate');
});
