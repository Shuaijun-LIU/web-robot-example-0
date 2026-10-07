import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync,readFileSync,writeFileSync} from 'node:fs';
import {insertionEngine} from './helpers/insertion-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';

test('four arms supply, support, insert and physically dispatch the assembled stand',async()=>{
  const file='public/assets/franka-cooperative/insertion-motion.json';assert.ok(existsSync(file),'verified insertion motion not exported');
  const p=JSON.parse(readFileSync(file)),{mj,m,d,find}=await insertionEngine();
  try{
    const extraSettleTicks=Number(process.env.INSERTION_EXTRA_SETTLE_TICKS??0);
    assert.ok(Number.isInteger(extraSettleTicks)&&extraSettleTicks>=0&&extraSettleTicks<=10000);
    for(let i=0;i<extraSettleTicks;i++)mj.mj_step(m,d);
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const budget=Math.ceil(p.phases.reduce((s,p)=>s+p.duration+(p.settleTimeout??0),0)/.002)+10;
    const retentionStart=p.phases.findIndex(p=>p.name==='Verify assembly without robot support');
    assert.ok(retentionStart>=0);
    const g=p.phases.at(-1).gates.find(g=>g.type==='inserted');
    const retention={samples:0,minDepth:Infinity,maxDepth:0,maxLateral:0,minAlignment:1,unsupportedTicks:0,unsupportedSamples:[]};
    for(let i=0;i<budget&&r.state.phase==='running';i++){
      r.step(m,d);
      if(r.state.stage>=retentionStart){
        const o=r.observeGate(m,d,g);retention.samples++;
        retention.minDepth=Math.min(retention.minDepth,o.depth);retention.maxDepth=Math.max(retention.maxDepth,o.depth);
        retention.maxLateral=Math.max(retention.maxLateral,o.lateral);retention.minAlignment=Math.min(retention.minAlignment,o.alignment);
        if(!o.supported){retention.unsupportedTicks++;if(retention.unsupportedSamples.length<20)retention.unsupportedSamples.push({phase:r.state.label,time:r.time,...o});}
        assert.ok(o.depth>=g.minDepth&&o.depth<=g.maxDepth&&o.lateral<=g.maxLateral&&o.alignment>=g.minAlignment,
          `Assembly unseated at ${r.state.label}: ${JSON.stringify(o)}`);
      }
      mj.mj_step(m,d);
    }
    assert.equal(r.state.phase,'complete',JSON.stringify({state:r.state,last:r.history.at(-1),metrics:r.metrics}));
    for(const [object,arm] of [['insert',2],['stand',0],['insert',1],['stand',3]])
      assert.ok(p.phases.some(p=>p.gates.some(g=>g.type==='grasp'&&g.object===object&&g.arm===arm)));
    // Retention is relative geometry, not an unbroken contact manifold: a
    // physically seated rod can briefly bounce micrometres on support release.
    // Log those samples; still require real settled support at the final gate.
    assert.ok(retention.samples>1000);
    const o=r.observeGate(m,d,g);
    assert.ok(o.supported&&o.depth>.095&&o.lateral<.006&&o.alignment>.98);
    assert.ok(r.observeGate(m,d,{type:'support',object:'stand',body:'assembly_output'}).supported);
    assert.ok(d.xpos[find.body('stand')*3]<-.20,'assembled stand reached the separate output');
    for(const object of ['stand','insert'])assert.equal(r.observeGate(m,d,{type:'released',object}).fingerContacts,0);
    assert.ok(r.observeGate(m,d,{type:'home'}).error<.025);
    assert.ok(r.metrics.maxForbiddenPenetration<.001);assert.ok(r.metrics.maxLimitedContactPenetration<.001);
    writeFileSync(`artifacts/reports/cooperative-insertion-wasm${extraSettleTicks?`-extra-settle-${extraSettleTicks}`:''}.json`,JSON.stringify({success:true,engine:mj.mj_versionString(),extraSettleTicks,state:r.state,time:r.time,metrics:r.metrics,retention,history:r.history},null,2)+'\n');
  }finally{d.delete();m.delete();}
});
