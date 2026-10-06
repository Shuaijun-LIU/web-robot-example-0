import test from 'node:test';
import assert from 'node:assert/strict';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';
import {validateCooperativePlan} from '../src/cooperativeMotion.js';

const q=[1.570796,-.785398,0,-2.356194,0,1.570796,.785398];
function fixture(){
  const phase={name:'Confirm support',duration:.2,settleTimeout:.3,
    paths:Array.from({length:4},()=>[[...q],[...q]]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[{type:'support',object:'pot',body:'pad'}]};
  const plan={version:1,scene:'pot',initialJoints:q,initialObjects:[],phases:[phase,{...phase,name:'Next motion',settleTimeout:0,gates:[]}]};
  const r=new CooperativeMotionRuntime(plan,{});
  r.refs={qa:Array.from({length:4},()=>[])};r.previous=0;r.state={phase:'running',stage:0,active:true};
  r.command={joints:[q,q,q,q],grippers:[255,255,255,255]};
  // Isolate the phase clock from the already separately tested physics adapter.
  r.contacts=()=>{};r.apply=()=>{};
  const tick=time=>r.step({}, {time,qpos:new Float64Array(0)});
  return {r,plan,tick};
}

test('temporary loss of settled support waits at the endpoint without consuming the next motion',()=>{
  const {r,tick}=fixture();
  r.observeGate=()=>({supported:r.time>=.24,speed:.01});
  tick(.2);assert.equal(r.state.phase,'running');assert.equal(r.state.stage,0);
  assert.deepEqual(r.command.joints,[q,q,q,q]);
  tick(.24);assert.equal(r.state.stage,1);assert.equal(r.phaseTime,0);
  tick(.25);assert.ok(Math.abs(r.phaseTime-.01)<1e-10);
  tick(.44);assert.equal(r.state.phase,'complete');
});

test('support waiting remains bounded and retains the failing measurements',()=>{
  const {r,tick}=fixture();r.observeGate=()=>({supported:false,speed:0});
  tick(.49);assert.equal(r.state.phase,'running');
  tick(.50);assert.equal(r.state.phase,'error');
  assert.equal(r.history.at(-1).passed,false);
  assert.equal(r.history.at(-1).observations[0].supported,false);
});

test('invalid or unbounded settling windows are rejected',()=>{
  const {plan}=fixture();
  for(const value of [-1,Infinity,4,'1']){
    plan.phases[0].settleTimeout=value;
    assert.match(validateCooperativePlan(plan,'pot'),/settling/);
  }
});
