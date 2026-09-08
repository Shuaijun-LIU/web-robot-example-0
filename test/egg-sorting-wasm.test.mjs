import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,readdirSync,existsSync,writeFileSync} from 'node:fs';
import {resolve,relative} from 'node:path';
import loadMujoco from 'mujoco-js';

for(const trial of [
  {name:'two-arm',file:'artifacts/reports/demo2-sorting-parallel.json',counts:[1,1,0,0],arms:2},
  {name:'four-arm',file:process.env.SORTING_PLAN??'public/assets/franka-egg-sorting/sorting-motion.json',counts:[4,4,4,4],arms:4},
]) test(`${trial.name} browser runtime sorts physical eggs and returns home`,async()=>{
  const module=await import('../src/EggSortingRuntime.js').catch(()=>null);
  assert.ok(module,'multi-arm runtime is missing');
  const root=resolve('public/assets/franka-egg-sorting');
  assert.ok(existsSync(trial.file),'checked multi-arm motion is missing');
  const plan=JSON.parse(readFileSync(trial.file));
  const mj=await loadMujoco();mj.FS.mkdir('/sorting');
  function mount(dir){for(const item of readdirSync(dir,{withFileTypes:true})){
    const file=resolve(dir,item.name),dest='/sorting/'+relative(root,file);
    if(item.isDirectory()){mj.FS.mkdir(dest);mount(file);}else mj.FS.writeFile(dest,readFileSync(file));
  }}mount(root);
  const m=mj.MjModel.loadFromXML('/sorting/scene.xml'),d=new mj.MjData(m);
  const lookup=(type,name)=>mj.mj_name2id(m,mj.mjtObj[type].value,name);
  const find={body:n=>lookup('mjOBJ_BODY',n),joint:n=>lookup('mjOBJ_JOINT',n),site:n=>lookup('mjOBJ_SITE',n)};
  try {
    for(let a=0;a<4;a++){
      d.ctrl.set([...plan.initialJoints,255],a*8);
      for(let j=0;j<7;j++)d.qpos[m.jnt_qposadr[find.joint(`r${a}_joint${j+1}`)]]=plan.initialJoints[j];
    }
    mj.mj_forward(m,d);for(let i=0;i<500;i++)mj.mj_step(m,d);
    const runtime=new module.EggSortingRuntime(plan,find);
    const jaw=m.jnt_qposadr[find.joint('r0_finger_joint1')],open=d.qpos[jaw];
    d.qpos[jaw]=.01;runtime.start(m,d);
    assert.equal(runtime.state.reason,'reset-required-grippers');
    d.qpos[jaw]=open;mj.mj_forward(m,d);
    runtime.start(m,d);
    for(let i=0;i<(plan.duration+30)/.002&&runtime.state.phase==='running';i++){
      runtime.step(m,d);mj.mj_step(m,d);
    }
    const warnings=[],handles=d.warning;
    try {for(let i=0;i<handles.size();i++){const w=handles.get(i);try{warnings.push(w.number);}finally{w.delete();}}}finally{handles.delete();}
    const result={state:runtime.state,history:runtime.history,metrics:runtime.metrics,warnings,engine:mj.mj_versionString()};
    writeFileSync(`artifacts/reports/demo2-${trial.name}-wasm.json`,JSON.stringify(result,null,2)+'\n');
    assert.equal(runtime.state.phase,'complete',JSON.stringify(result.state));
    assert.deepEqual(runtime.state.counts,trial.counts);
    assert.ok(runtime.metrics.parallelSeconds>1);
    assert.ok(runtime.metrics.maxSimultaneousArms===trial.arms);
    assert.ok(runtime.metrics.maxForbiddenPenetration<=.0001);
    assert.ok(runtime.metrics.maxGripPenetration<=.001);
    assert.ok(warnings.every(n=>n===0));
    for(let a=0;a<4;a++)for(let j=0;j<7;j++)assert.ok(Math.abs(d.qpos[m.jnt_qposadr[find.joint(`r${a}_joint${j+1}`)]]-plan.initialJoints[j])<.025);
  } finally {d.delete();m.delete();}
});
