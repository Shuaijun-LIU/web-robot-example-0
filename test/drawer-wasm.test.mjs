import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync,readFileSync,writeFileSync} from 'node:fs';
import {drawerEngine} from './helpers/drawer-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';

test('four arms physically open, retrieve, kit, close and release both drawers',async()=>{
  const path='public/assets/franka-cooperative/drawer-motion.json';assert.ok(existsSync(path),'complete drawer motion not exported');
  const p=JSON.parse(readFileSync(path)),{mj,m,d,find}=await drawerEngine();
  try{
    const extraSettleTicks=Number(process.env.DRAWER_EXTRA_SETTLE_TICKS??0);
    assert.ok(Number.isInteger(extraSettleTicks)&&extraSettleTicks>=0&&extraSettleTicks<=10000);
    for(let i=0;i<extraSettleTicks;i++)mj.mj_step(m,d);
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const before=Array.from(d.qpos);r.step(m,d);assert.deepEqual(Array.from(d.qpos),before);
    const budget=Math.ceil(p.phases.reduce((s,p)=>s+p.duration+(p.settleTimeout??0),0)/.002)+10;
    for(let i=0;i<budget&&r.state.phase==='running';i++){r.step(m,d);mj.mj_step(m,d);}
    assert.equal(r.state.phase,'complete',JSON.stringify({state:r.state,last:r.history.at(-1),metrics:r.metrics}));
    for(const [prefix,arm] of [['south',0],['north',2]]){
      assert.ok(p.phases.some(p=>p.gates.some(g=>g.type==='grasp'&&g.object===`${prefix}_handle`&&g.arm===arm)));
      assert.ok(r.history.some(h=>{
        const phase=p.phases.find(phase=>phase.name===h.phase);
        return phase.gates.some((gate,i)=>gate.type==='joint-range'&&gate.joint===`${prefix}_slide`&&h.observations[i]?.position<-.175);
      }),`${prefix} drawer was actually opened`);
      assert.ok(Math.abs(d.qpos[m.jnt_qposadr[find.joint(`${prefix}_slide`)]])<.006);
    }
    for(const [box,arm,sign] of [['tea_box',1,1],['coffee_box',3,-1]]){
      assert.ok(p.phases.some(p=>p.gates.some(g=>g.type==='grasp'&&g.object===box&&g.arm===arm)));
      assert.ok(sign*d.xpos[find.body(box)*3]>.02,'correct product in its own tray slot');
      assert.equal(r.observeGate(m,d,{type:'released',object:box}).fingerContacts,0);
      assert.equal(r.observeGate(m,d,{type:'support',object:box,body:'order_tray'}).supported,true);
    }
    assert.ok(r.metrics.maxForbiddenPenetration<1e-6,'nominal path clears robot/fixture surfaces');
    assert.equal(r.observeGate(m,d,{type:'home'}).error<.025,true);
    writeFileSync(`artifacts/reports/cooperative-drawer-wasm${extraSettleTicks?`-extra-settle-${extraSettleTicks}`:''}.json`,JSON.stringify({success:true,engine:mj.mj_versionString(),extraSettleTicks,state:r.state,time:r.time,metrics:r.metrics,history:r.history},null,2)+'\n');
  }finally{d.delete();m.delete()}
});
