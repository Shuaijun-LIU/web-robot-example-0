import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,readdirSync,mkdirSync,writeFileSync} from 'node:fs';
import {resolve,relative} from 'node:path';
import loadMujoco from 'mujoco-js';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';

for(const scene of ['scan','pot'])test(`${scene} complete cooperative cycle uses physical grasps and support`,async()=>{
  const root=resolve('public/assets/franka-cooperative');
  const p=JSON.parse(readFileSync(`${root}/${scene}-motion.json`,'utf8'));
  const mj=await loadMujoco();mj.FS.mkdir('/cell');
  function mount(dir){for(const e of readdirSync(dir,{withFileTypes:true})){
    const path=resolve(dir,e.name),dest='/cell/'+relative(root,path);
    if(e.isDirectory()){mj.FS.mkdir(dest);mount(path);}else mj.FS.writeFile(dest,readFileSync(path));
  }}mount(root);
  const m=mj.MjModel.loadFromXML(`/cell/${scene}.xml`),d=new mj.MjData(m);
  const id=(type,name)=>mj.mj_name2id(m,mj.mjtObj[type].value,name),find={body:n=>id('mjOBJ_BODY',n),joint:n=>id('mjOBJ_JOINT',n)};
  for(let a=0;a<4;a++){
    d.ctrl.set([...p.initialJoints,255],a*8);
    for(let j=0;j<7;j++)d.qpos[m.jnt_qposadr[find.joint(`r${a}_joint${j+1}`)]]=p.initialJoints[j];
  }
  const settleTicks=Number(process.env.COOPERATIVE_SETTLE_TICKS??500);
  assert.ok(Number.isInteger(settleTicks)&&settleTicks>=0&&settleTicks<=10000);
  mj.mj_forward(m,d);for(let i=0;i<settleTicks;i++)mj.mj_step(m,d);
  const r=new CooperativeMotionRuntime(p,find);r.start(m,d);
  try {
    const ticks=Math.ceil(p.phases.reduce((s,p)=>s+p.duration+(p.settleTimeout??0),0)/.002)+20;
    for(let tick=0;tick<ticks&&r.state.phase==='running';tick++){r.step(m,d);mj.mj_step(m,d);}
    assert.equal(r.state.phase,'complete',JSON.stringify({state:r.state,history:r.history.at(-1),metrics:r.metrics}));
    assert.equal(r.history.length,p.phases.length);
    if(scene==='scan')assert.equal(p.phases.flatMap(p=>p.gates).filter(g=>g.type==='scan').length,2,'both products need an actual inspection gate');
    else assert.ok(p.phases.some(p=>p.carry.filter(c=>c.object==='cooking_pot').length===2&&p.gates.some(g=>g.type==='height')),'pot must be physically lifted by both arms');
    for(const object of scene==='scan'?['tea_box','coffee_box']:['carrot','tomato'])assert.ok(p.phases.at(-1).gates.some(g=>g.type==='inside'&&g.object===object));
    assert.ok(p.phases.at(-1).gates.some(g=>g.type==='home'));
    assert.ok(r.metrics.maxForbiddenPenetration<=.001);
    if(scene==='pot')assert.ok(r.metrics.maxForbiddenPenetration<1e-6,'loader return must clear the holding wrist, not brush it');
    mkdirSync('artifacts/reports',{recursive:true});
    writeFileSync(`artifacts/reports/cooperative-${scene}-wasm${settleTicks===500?'':`-settle-${settleTicks}`}.json`,JSON.stringify({success:true,engine:mj.mj_versionString(),settleTicks,state:r.state,time:r.time,metrics:r.metrics,history:r.history},null,2)+'\n');
  }finally{d.delete();m.delete();}
});
