import test from 'node:test';
import assert from 'node:assert/strict';
import {existsSync,readFileSync,writeFileSync} from 'node:fs';
import {relayEngine} from './helpers/relay-engine.mjs';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';
import {RelayContactAudit,meshBounds,boundsSeparation} from './helpers/relay-contact-audit.mjs';
import {MeshBVH} from 'three-mesh-bvh';
import {OBJLoader} from 'three/examples/jsm/loaders/OBJLoader.js';
import {Vector3} from 'three';

test('two physical support pairs relay a level panel over the barrier and release it at output',async()=>{
  const file='public/assets/franka-cooperative/relay-motion.json';assert.ok(existsSync(file),'verified relay motion not exported');
  const p=JSON.parse(readFileSync(file)),{mj,m,d,find}=await relayEngine();
  try{
    const extraSettleTicks=Number(process.env.RELAY_EXTRA_SETTLE_TICKS??0);
    assert.ok(Number.isInteger(extraSettleTicks)&&extraSettleTicks>=0&&extraSettleTicks<=10000);
    for(let i=0;i<extraSettleTicks;i++)mj.mj_step(m,d);
    const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
    const budget=Math.ceil(p.phases.reduce((s,p)=>s+p.duration+(p.settleTimeout??0),0)/.002)+10;
    const lift=p.phases.findIndex(p=>p.name==='Arms 1 and 3 lift panel together');assert.ok(lift>=0);
    const evidence={airborneSamples:0,minBilateralSupports:4,minSurfaceSupports:4,surfaceFallbackSamples:0,maxSurfaceGap:0,
      minAirborneHeight:Infinity,minObstacleClearance:Infinity,maxTilt:0,maxVisualContactGap:0,visualContactSamples:0,relayChecks:[]};
    const visual=new OBJLoader().parse(readFileSync('public/assets/franka-cooperative/assets/relay_panel-visual-0.obj','utf8')).children[0].geometry;
    const bvh=new MeshBVH(visual),point=new Vector3();
    const panelGeom=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM.value,'panel_contact_0');
    const audit=new RelayContactAudit(m,d,find.body('panel'),r.refs.fingers);
    const obstacles=new Set([0,1,2].map(i=>find.body(`barrier_${i}`)));
    const obstacleGeoms=Array.from({length:m.ngeom},(_,g)=>g).filter(g=>obstacles.has(m.geom_bodyid[g])&&m.geom_contype[g]);
    const obstacleBounds=obstacleGeoms.map(g=>meshBounds(m,d,g));
    const established=new Set();
    function physicalSupport(arm){
      const grasp=r.grasp(d,'panel',arm);
      if(grasp.bilateral){established.add(arm);return true;}
      if(!established.has(arm)||d.ctrl[arm*8+7]>1||!p.phases[r.state.stage].carry.some(c=>c.arm===arm&&c.object==='panel'))return false;
      const touch=r.touching.get(find.body('panel'))??new Set();
      const gap=Math.max(...r.refs.fingers[arm].map(f=>touch.has(f)?0:audit.gap(d,f)));
      if(gap>.00005)return false;
      evidence.surfaceFallbackSamples++;evidence.maxSurfaceGap=Math.max(evidence.maxSurfaceGap,gap);return true;
    }
    for(const receiver of [1,3])assert.ok(p.phases.findIndex(p=>p.name===`Arm ${receiver+1} establish receiver grip`)<p.phases.findIndex(p=>p.name==='Arm 1 release to Arm 2'));
    const upright={type:'upright',object:'panel',axis:[0,0,1],minAlignment:Math.cos(Math.PI/18),maxSpeed:.5};
    for(let i=0;i<budget&&r.state.phase==='running';i++){
      r.step(m,d);
      audit.update(m,d);
      for(const c of audit.currentPoints){
        const distance=bvh.closestPointToPoint(point.fromArray(c.point),{}).distance;
        evidence.visualContactSamples++;evidence.maxVisualContactGap=Math.max(evidence.maxVisualContactGap,distance);
        assert.ok(distance<.0003,`Invisible panel contact during ${r.state.label}: finger ${c.finger}, gap=${distance}, point=${c.point}`);
      }
      const o=r.observeGate(m,d,upright);evidence.maxTilt=Math.max(evidence.maxTilt,Math.acos(Math.min(1,Math.max(-1,o.alignment))));
      assert.ok(o.alignment>=upright.minAlignment,`tilted at ${r.state.label}: ${JSON.stringify(o)}`);
      if(r.state.stage>=lift){
        const panelBounds=meshBounds(m,d,panelGeom);
        for(const [i,b] of obstacleBounds.entries()){const distance=boundsSeparation(panelBounds,b);
          if(distance<evidence.minObstacleClearance){evidence.minObstacleClearance=distance;evidence.closestObstacle={geom:obstacleGeoms[i],phase:r.state.label,time:r.time,position:Array.from(d.xpos.slice(find.body('panel')*3,find.body('panel')*3+3))};}}
        const supported=['panel_start','panel_output'].some(body=>r.observeGate(m,d,{type:'support',object:'panel',body}).supported);
        if(!supported){
          const grips=[0,1,2,3].map(a=>r.grasp(d,'panel',a));const n=grips.filter(g=>g.bilateral).length;
          evidence.airborneSamples++;evidence.minBilateralSupports=Math.min(evidence.minBilateralSupports,n);
          evidence.minAirborneHeight=Math.min(evidence.minAirborneHeight,d.xpos[find.body('panel')*3+2]);
          const count=[0,1,2,3].filter(physicalSupport).length;
          evidence.minSurfaceSupports=Math.min(evidence.minSurfaceSupports,count);
          assert.ok(count>=2,`Only ${count} surface-qualified grips during ${r.state.label} at ${r.time}: ${JSON.stringify(grips)}`);
        }
        for(const label of ['Arm 1 release to Arm 2','Arm 3 release to Arm 4']){
          if(r.state.label===label){
            for(const a of [1,3])assert.ok(physicalSupport(a),`${label}: receiving arm ${a+1} not gripping`);
            if(!evidence.relayChecks.includes(label))evidence.relayChecks.push(label);
          }
        }
      }
      mj.mj_step(m,d);
    }
    assert.equal(r.state.phase,'complete',JSON.stringify({state:r.state,last:r.history.at(-1),metrics:r.metrics}));
    for(const arm of [0,1,2,3])assert.ok(p.phases.some(p=>p.gates.some(g=>g.type==='grasp'&&g.object==='panel'&&g.arm===arm)));
    assert.equal(evidence.relayChecks.length,2);assert.ok(evidence.airborneSamples>1000);
    assert.ok(evidence.visualContactSamples>1000);visual.dispose();
    assert.ok(evidence.minObstacleClearance>.002,`panel must clear the real obstacle: ${JSON.stringify(evidence)}`);
    assert.ok(r.observeGate(m,d,{type:'support',object:'panel',body:'panel_output'}).supported);
    assert.ok(d.xpos[find.body('panel')*3+1]>.30);
    assert.equal(r.observeGate(m,d,{type:'released',object:'panel'}).fingerContacts,0);
    assert.ok(r.observeGate(m,d,{type:'home'}).error<.025);
    assert.ok(r.metrics.maxForbiddenPenetration<.001);assert.equal(r.metrics.maxLimitedContactPenetration,0);
    writeFileSync(`artifacts/reports/cooperative-relay-wasm${extraSettleTicks?`-extra-settle-${extraSettleTicks}`:''}.json`,JSON.stringify({success:true,engine:mj.mj_versionString(),extraSettleTicks,state:r.state,time:r.time,metrics:r.metrics,evidence,history:r.history},null,2)+'\n');
  }finally{d.delete();m.delete();}
});
