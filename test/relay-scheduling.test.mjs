import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {sampleCooperativePhase} from '../src/cooperativeMotion.js';

test('paired relay motion removes idle waits without rushing contact entry or release',()=>{
  const {phases}=JSON.parse(readFileSync('public/assets/franka-cooperative/relay-motion.json'));
  const moving=p=>sampleCooperativePhase(p,.5).joints.map((q,a)=>
    Math.max(...q.map((v,j)=>Math.abs(v-p.paths[a][0][j])))).flatMap((d,a)=>d>.05?[a]:[]);
  const approach=phases.find(p=>p.name.startsWith('Approach')&&JSON.stringify(moving(p))==='[1,3]');
  assert.ok(approach,'opposing receivers should approach together, with both donors stationary');
  const entry=phases.find(p=>p.name.startsWith('Engage')&&JSON.stringify(moving(p))==='[1,3]');
  assert.ok(entry&&entry.duration>=2.5,'retain the slow physical entry interval');
  const firstRelease=phases.findIndex(p=>p.grippers[0][0]===0&&p.grippers[0][1]===255);
  let previousClose=-1;
  for(const arm of [1,3]){
    const close=phases.findIndex(p=>p.grippers[arm][0]===255&&p.grippers[arm][1]===0);
    assert.ok(close>phases.indexOf(entry)&&close<firstRelease,'both receivers close before any donor opens');
    assert.ok(close>previousClose,'receivers establish their grips in separate, ordered stages');
    assert.deepEqual(phases[close].grippers.flatMap(([a,b],i)=>a!==b?[i]:[]),[arm],'only one receiver closes at a time');
    previousClose=close;
    assert.ok(phases[close].duration>=1.5);
    assert.ok(phases[close].gates.some(g=>g.type==='grasp'&&g.arm===arm&&g.object==='panel'));
  }
  const secondRelease=phases.findIndex(p=>p.grippers[2][0]===0&&p.grippers[2][1]===255);
  assert.ok(secondRelease>firstRelease,'donors still release sequentially');
  for(const index of [firstRelease,secondRelease])assert.ok(phases[index].duration>=1.5,'retain slow donor release');
  for(const p of phases.slice(firstRelease,secondRelease+1)){
    assert.deepEqual(moving(p),[],'release holds all TCP goals before withdrawal');
    for(const arm of [1,3])assert.ok(p.gates.some(g=>g.type==='grasp'&&g.arm===arm));
  }
  assert.ok(phases.slice(secondRelease+1).some(p=>p.name.includes('clear relay')&&JSON.stringify(moving(p))==='[0,2]'));
  assert.ok(phases.reduce((s,p)=>s+p.duration,0)<=50,'complete under 50 seconds without shortening contact intervals');
});
