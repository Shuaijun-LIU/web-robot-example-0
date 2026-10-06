import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,readdirSync} from 'node:fs';
import {resolve,relative} from 'node:path';
import loadMujoco from 'mujoco-js';
import {CooperativeMotionRuntime} from '../src/CooperativeMotionRuntime.js';

test('real engine playback owns only actuators and stops its clock across reset/pause',async()=>{
  const root=resolve('public/assets/franka-cooperative'),mj=await loadMujoco();mj.FS.mkdir('/cell');
  function mount(dir){for(const e of readdirSync(dir,{withFileTypes:true})){
    const path=resolve(dir,e.name),dest='/cell/'+relative(root,path);
    if(e.isDirectory()){mj.FS.mkdir(dest);mount(path);}else mj.FS.writeFile(dest,readFileSync(path));
  }}mount(root);
  const m=mj.MjModel.loadFromXML('/cell/scan.xml'),d=new mj.MjData(m);
  const id=(type,name)=>mj.mj_name2id(m,mj.mjtObj[type].value,name);
  const find={body:n=>id('mjOBJ_BODY',n),joint:n=>id('mjOBJ_JOINT',n)};
  const q=[1.570796,-.785398,0,-2.356194,0,1.570796,.785398];
  for(let a=0;a<4;a++){
    d.ctrl.set([...q,255],a*8);
    for(let j=0;j<7;j++)d.qpos[m.jnt_qposadr[find.joint(`r${a}_joint${j+1}`)]]=q[j];
  }
  mj.mj_forward(m,d);for(let i=0;i<500;i++)mj.mj_step(m,d);
  const p={version:1,scene:'scan',initialJoints:q,initialObjects:[],phases:[{name:'Safe hold',duration:.2,
    paths:Array.from({length:4},()=>[[...q],[...q]]),grippers:Array.from({length:4},()=>[255,255]),allowedContacts:[],carry:[],gates:[{type:'home'}]}]};
  const r=new CooperativeMotionRuntime(p,find);
  try {
    r.start(m,d);
    const before=Array.from(d.qpos);r.step(m,d);assert.deepEqual(Array.from(d.qpos),before,'control callback cannot mutate physical state');
    r.step(m,d);assert.equal(r.time,0,'paused simulation cannot advance task time');
    for(let i=0;i<120;i++){r.step(m,d);mj.mj_step(m,d);}
    assert.equal(r.state.phase,'complete',JSON.stringify(r.state));assert.equal(r.history.length,1);
    r.fail('test safety stop');assert.equal(r.state.active,true,'failed playback must retain control ownership until Reset');
    mj.mj_resetData(m,d);r.step(m,d);assert.equal(r.state.phase,'ready');assert.equal(r.time,0);
  }finally{d.delete();m.delete();}
});
