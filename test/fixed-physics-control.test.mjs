import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { clockDeclaration, patchPhysicsControl } from '../scripts/fixedPhysicsControlPlugin.mjs';

test('physics control ticks are identical across different display-frame groupings', () => {
  function run(groups) {
    const tick = new Function(`${clockDeclaration}; return assemblyControlTick;`)();
    const data = {time:0,qfrc_applied:[12]}, model={nv:1}, times=[];
    const callbacks=[(_m,d)=>{assert.equal(d.qfrc_applied[0],0);times.push(Math.round(d.time*1000));d.qfrc_applied[0]=4;}];
    let step=0;
    for(const group of groups) for(let i=0;i<group;i++) {data.time=step++*.002;tick(model,data,.01,callbacks);}
    return times;
  }
  assert.deepEqual(run([1,7,15,2,25]),run([10,10,10,10,10]));
  assert.deepEqual(run([50]),Array.from({length:10},(_,i)=>i*10));
});

test('dependency adaptation is checked and opt-in; no silent version mismatch', () => {
  const code=readFileSync(new URL('../node_modules/mujoco-react/dist/index.js',import.meta.url),'utf8');
  const patched=patchPhysicsControl(code);
  assert.match(patched,/configRef.current.controlTimestep/);
  assert.equal((patched.match(/assemblyControlTick\(model, data, controlPeriod/g)??[]).length,2);
  assert.throws(()=>patchPhysicsControl('changed upstream'),/loop changed/);
});
