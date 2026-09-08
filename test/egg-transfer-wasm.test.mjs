import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync,readdirSync,mkdirSync,writeFileSync } from 'node:fs';
import { resolve,relative } from 'node:path';
import loadMujoco from 'mujoco-js';
import { EGG_SORTING_LAYOUT } from '../src/eggSortingLayout.js';
import { sampleEggPhase,checkEggGate,isForbiddenEggContact } from '../src/eggTransfer.js';
import { consumeMujocoContacts } from '../src/mujocoContact.js';

test('browser-engine replay carries and releases the egg through real contacts',async()=>{
  const root=resolve('public/assets/franka-egg-sorting');
  const plan=JSON.parse(readFileSync(`${root}/first-egg-motion.json`,'utf8'));
  const mj=await loadMujoco();mj.FS.mkdir('/eggs');
  function mount(dir){for(const item of readdirSync(dir,{withFileTypes:true})){
    const full=resolve(dir,item.name),dest='/eggs/'+relative(root,full);
    if(item.isDirectory()){mj.FS.mkdir(dest);mount(full);}else mj.FS.writeFile(dest,readFileSync(full));
  }}mount(root);
  const m=mj.MjModel.loadFromXML('/eggs/scene.xml'),d=new mj.MjData(m);
  const id=(type,name)=>mj.mj_name2id(m,mj.mjtObj[type].value,name);
  const joint=name=>id('mjOBJ_JOINT',name),body=name=>id('mjOBJ_BODY',name);
  const qa=Array.from({length:7},(_,j)=>m.jnt_qposadr[joint(`r0_joint${j+1}`)]);
  const da=Array.from({length:7},(_,j)=>m.jnt_dofadr[joint(`r0_joint${j+1}`)]);
  const egg=body('egg_0'),site=id('mjOBJ_SITE','r0_tcp'),tray=body('output_tray_0');
  const fingers=['r0_left_finger','r0_right_finger'].map(body);
  const eggs=Array.from({length:16},(_,i)=>body(`egg_${i}`));
  const source=Array.from({length:4},(_,i)=>body(`source_insert_${i}`));
  const robotBodies=new Set();const roots=Array.from({length:4},(_,i)=>body(`r${i}_link0`));
  for(let i=1;i<m.nbody;i++){let a=i;while(a>0&&!roots.includes(a))a=m.body_parentid[a];if(roots.includes(a))robotBodies.add(i);}
  const jaw=[1,2].map(i=>m.jnt_qposadr[joint(`r0_finger_joint${i}`)]);
  const eggDof=m.jnt_dofadr[joint('egg_0_free')];
  d.ctrl.set(EGG_SORTING_LAYOUT.homeJoints);
  for(let arm=0;arm<4;arm++)for(let j=0;j<7;j++)d.qpos[m.jnt_qposadr[joint(`r${arm}_joint${j+1}`)]]=EGG_SORTING_LAYOUT.homeJoints[arm*8+j];
  mj.mj_forward(m,d);
  for(let t=0;t<500;t++)mj.mj_step(m,d);
  const initial=eggs.map(i=>Array.from(d.xpos.slice(i*3,i*3+3)));
  let lastGrip=255;const records=[];
  try{
    for(let phaseIndex=0;phaseIndex<plan.phases.length;phaseIndex++){
      const phase=plan.phases[phaseIndex];let observation,loss=0;
      const ticks=Math.round(phase.duration/.002);
      for(let tick=0;tick<ticks;tick++){
        const command=sampleEggPhase(phase,(tick+1)/ticks,lastGrip);
        for(let j=0;j<7;j++)d.ctrl[j]=command.joints[j]+d.qfrc_bias[da[j]]/m.actuator_gainprm[j*10];
        d.ctrl[7]=command.gripper;mj.mj_step(m,d);
        const contacts=consumeMujocoContacts(d.contact,d.ncon);
        const touching=new Set();let forbidden=0,gripPenetration=0;
        const lift=d.xpos[egg*3+2]-initial[0][2];
        const allowedSupports=new Set(phaseIndex<=3&&lift<.02?source:phaseIndex>=6?[tray]:[]);
        for(const c of contacts){if(c.distance>0)continue;
          const a=m.geom_bodyid[c.geom1],b=m.geom_bodyid[c.geom2];
          if(a===egg||b===egg)touching.add(a===egg?b:a);
          if((a===egg&&fingers.includes(b))||(b===egg&&fingers.includes(a)))gripPenetration=Math.max(gripPenetration,-c.distance);
          if(isForbiddenEggContact(a,b,{robotBodies,fingers,egg,allowedSupports}))forbidden=Math.max(forbidden,-c.distance);
        }
        const pos=Array.from(d.xpos.slice(egg*3,egg*3+3));
        observation={bilateral:fingers.every(i=>touching.has(i)),fingerContacts:fingers.filter(i=>touching.has(i)).length,
          aperture:jaw.reduce((s,i)=>s+d.qpos[i],0),tcpDistance:Math.hypot(...pos.map((v,i)=>v-d.site_xpos[site*3+i])),lift,
          traySupported:touching.has(tray),tiltDegrees:Math.acos(Math.max(-1,Math.min(1,d.xmat[egg*9+8])))*180/Math.PI,
          cellError:Math.hypot(pos[0]-plan.cell[0],pos[1]-plan.cell[1]),speed:Math.hypot(...d.qvel.slice(eggDof,eggDof+3)),
          forbiddenPenetration:forbidden,gripPenetration,neighborDisplacement:Math.max(...eggs.slice(1).map((b,i)=>Math.hypot(...Array.from(d.xpos.slice(b*3,b*3+3)).map((v,j)=>v-initial[i+1][j]))))};
        assert.equal(checkEggGate(null,observation),null,`${phase.name}: ${JSON.stringify(observation)}`);
        if(['carried','supported'].includes(phase.gate)){loss=observation.bilateral?0:loss+.002;assert.ok(loss<=.12,`lost egg during ${phase.name}`);}
      }
      records.push({phase:phase.name,...observation});
      assert.equal(checkEggGate(phase.gate,observation),null,`${phase.name}: ${JSON.stringify(observation)}`);
      lastGrip=phase.gripper;
    }
    assert.ok(Array.from(d.qpos).every(Number.isFinite));
    mkdirSync('artifacts/reports',{recursive:true});
    writeFileSync('artifacts/reports/demo2-first-egg-wasm.json',JSON.stringify({success:true,engine:mj.mj_versionString(),records},null,2)+'\n');
    console.log(JSON.stringify({engine:mj.mj_versionString(),records}));
  }finally{d.delete();m.delete();}
});
