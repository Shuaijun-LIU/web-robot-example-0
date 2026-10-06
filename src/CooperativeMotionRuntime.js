import {validateCooperativePlan,sampleCooperativePhase,checkCooperativeGate} from './cooperativeMotion.js';
import {consumeMujocoContacts} from './mujocoContact.js';

const pair=(a,b)=>a<b?`${a}:${b}`:`${b}:${a}`;
const vec=(array,id)=>Array.from(array.slice(id*3,id*3+3));
const rotated=(matrix,id,p)=>[0,1,2].map(row=>p.reduce((s,v,k)=>s+matrix[id*9+row*3+k]*v,0));
const worldPoint=(d,id,p)=>rotated(d.xmat,id,p).map((v,j)=>v+d.xpos[id*3+j]);

/** Actuator-only playback. No free-body state, equality or force overrides. */
export class CooperativeMotionRuntime {
  constructor(plan,find){
    const reason=validateCooperativePlan(plan,plan.scene);if(reason)throw new Error(reason);
    this.plan=plan;this.find=find;this.reset();
  }
  reset(){
    this.state={phase:'ready',label:'Ready',stage:0};this.time=0;this.phaseTime=0;this.previous=null;this.refs=null;
    this.history=[];this.metrics={maxForbiddenPenetration:0,maxGripPenetration:0};this.loss=new Map();this.touching=new Map();
  }
  fail(reason){this.state={...this.state,phase:'error',label:'Motion stopped',reason};}
  start(m,d){
    this.reset();const f=this.find;
    const body=name=>{const id=f.body(name);if(id<0)throw new Error(`Missing body ${name}`);return id;};
    const joint=name=>{const id=f.joint(name);if(id<0)throw new Error(`Missing joint ${name}`);return id;};
    const joints=Array.from({length:4},(_,a)=>Array.from({length:7},(_,j)=>joint(`r${a}_joint${j+1}`)));
    const qa=joints.map(row=>row.map(i=>m.jnt_qposadr[i])),da=joints.map(row=>row.map(i=>m.jnt_dofadr[i]));
    const fingers=Array.from({length:4},(_,a)=>['left','right'].map(s=>body(`r${a}_${s}_finger`)));
    const jaws=Array.from({length:4},(_,a)=>[1,2].map(j=>m.jnt_qposadr[joint(`r${a}_finger_joint${j}`)]));
    const roots=Array.from({length:4},(_,a)=>body(`r${a}_link0`)),robots=new Set();
    for(let i=1;i<m.nbody;i++){let p=i;while(p>0&&!roots.includes(p))p=m.body_parentid[p];if(roots.includes(p))robots.add(i);}
    if(qa.some(row=>row.some((v,j)=>Math.abs(d.qpos[v]-this.plan.initialJoints[j])>.035)))throw new Error('Reset required: arms moved');
    if(this.plan.initialObjects.some(o=>Math.hypot(...vec(d.xpos,body(o.name)).map((v,j)=>v-o.position[j]))>.008))throw new Error('Reset required: objects moved');
    const allowed=this.plan.phases.map(p=>new Set(p.allowedContacts.map(([a,b])=>pair(body(a),body(b)))));
    this.refs={qa,da,fingers,jaws,robots,body,allowed};
    this.previous=d.time;this.command=sampleCooperativePhase(this.plan.phases[0],0);
    this.state={phase:'running',label:this.plan.phases[0].name,stage:0,active:true};
  }
  grasp(d,object,arm){
    const r=this.refs,id=r.body(object),contacts=this.touching.get(id)??new Set();
    return {bilateral:r.fingers[arm].every(b=>contacts.has(b)),fingerContacts:r.fingers[arm].filter(b=>contacts.has(b)).length,
      aperture:r.jaws[arm].reduce((sum,j)=>sum+d.qpos[j],0)};
  }
  observeGate(m,d,g){
    const r=this.refs;
    if(g.type==='home')return {error:Math.max(...r.qa.flatMap(row=>row.map((q,j)=>Math.abs(d.qpos[q]-this.plan.initialJoints[j]))))};
    if(g.type==='grasp')return this.grasp(d,g.object,g.arm);
    const id=r.body(g.object),contacts=this.touching.get(id)??new Set();
    if(g.type==='released')return {fingerContacts:r.fingers.flat().filter(b=>contacts.has(b)).length};
    if(g.type==='height')return {height:d.xpos[id*3+2]};
    const joint=m.body_jntadr[id],dof=m.jnt_dofadr[joint],speed=Math.hypot(...d.qvel.slice(dof,dof+3));
    if(g.type==='support')return {supported:contacts.has(r.body(g.body)),speed};
    if(g.type==='inside'){
      const container=r.body(g.container),offset=vec(d.xipos,id).map((v,j)=>v-d.xpos[container*3+j]);
      const relative=[0,1,2].map(k=>offset.reduce((s,v,j)=>s+v*d.xmat[container*9+j*3+k],0));
      return {relative,speed,supported:contacts.has(container)};
    }
    if(g.type==='scan'){
      const scanner=r.body(g.scanner),origin=worldPoint(d,scanner,g.origin),axis=rotated(d.xmat,scanner,g.axis),target=worldPoint(d,id,g.target),normal=rotated(d.xmat,id,g.normal);
      const delta=target.map((v,j)=>v-origin[j]),distance=delta.reduce((s,v,j)=>s+v*axis[j],0);
      return {distance,offAxis:Math.hypot(...delta.map((v,j)=>v-distance*axis[j])),facing:-normal.reduce((s,v,j)=>s+v*axis[j],0)};
    }
    return {};
  }
  contacts(m,d){
    const r=this.refs,p=this.plan.phases[this.state.stage],allowed=r.allowed[this.state.stage];
    this.touching=new Map();let forbidden=0,grip=0,worst=null;
    for(const c of consumeMujocoContacts(d.contact,d.ncon)){
      if(c.distance>0)continue;
      const a=m.geom_bodyid[c.geom1],b=m.geom_bodyid[c.geom2];
      if(!this.touching.has(a))this.touching.set(a,new Set());if(!this.touching.has(b))this.touching.set(b,new Set());
      this.touching.get(a).add(b);this.touching.get(b).add(a);
      if(!r.robots.has(a)&&!r.robots.has(b))continue;
      if(allowed.has(pair(a,b)))grip=Math.max(grip,-c.distance);
      else if(-c.distance>forbidden){forbidden=-c.distance;worst=[a,b];}
    }
    this.metrics.maxForbiddenPenetration=Math.max(this.metrics.maxForbiddenPenetration,forbidden);
    this.metrics.maxGripPenetration=Math.max(this.metrics.maxGripPenetration,grip);
    if(forbidden>.001){this.fail(`forbidden-contact ${worst?.join('/')} ${forbidden.toFixed(5)} m`);return;}
    if(grip>.0025){this.fail(`grip-penetration ${grip.toFixed(5)} m`);return;}
    for(const carry of p.carry){
      const key=`${carry.object}:${carry.arm}`,held=this.grasp(d,carry.object,carry.arm).bilateral;
      if(held)this.loss.delete(key);
      else {if(!this.loss.has(key))this.loss.set(key,d.time);if(d.time-this.loss.get(key)>.12)this.fail(`lost-grasp ${key}`);}
    }
  }
  apply(m,d){
    if(!this.refs||!this.command)return;
    for(let a=0;a<4;a++){
      for(let j=0;j<7;j++)d.ctrl[a*8+j]=this.command.joints[a][j]+d.qfrc_bias[this.refs.da[a][j]]/m.actuator_gainprm[(a*8+j)*10];
      d.ctrl[a*8+7]=this.command.grippers[a];
    }
  }
  step(m,d){
    try{this.advance(m,d);}catch(error){this.fail(String(error));}
    if(this.state.phase==='error'&&this.refs&&Array.from(d.qpos).every(Number.isFinite)){
      this.command={joints:this.refs.qa.map(row=>row.map(q=>d.qpos[q])),grippers:this.command.grippers};this.apply(m,d);
    }
  }
  advance(m,d){
    if(!this.refs)return;
    if(d.time+1e-9<this.previous){this.reset();return;}
    if(this.state.phase!=='running'){if(this.state.phase==='complete')this.apply(m,d);return;}
    if(!Array.from(d.qpos).every(Number.isFinite)){this.fail('non-finite-state');return;}
    const dt=Math.max(0,d.time-this.previous);this.previous=d.time;this.time+=dt;this.phaseTime+=dt;
    this.contacts(m,d);if(this.state.phase==='error')return;
    let p=this.plan.phases[this.state.stage];
    if(this.phaseTime>=p.duration-1e-9){
      const observations=p.gates.map(g=>this.observeGate(m,d,g));
      for(let i=0;i<p.gates.length;i++){const reason=checkCooperativeGate(p.gates[i],observations[i]);if(reason){this.fail(`${p.name}: ${reason}`);return;}}
      this.history.push({phase:p.name,time:this.time,observations});
      this.command=sampleCooperativePhase(p,1);
      const next=this.state.stage+1;
      if(next===this.plan.phases.length){this.state={phase:'complete',label:'Task complete · all arms returned',stage:this.state.stage,active:true};this.apply(m,d);return;}
      this.phaseTime=Math.max(0,this.phaseTime-p.duration);this.loss.clear();p=this.plan.phases[next];
      this.state={phase:'running',label:p.name,stage:next,active:true};
    }
    this.command=sampleCooperativePhase(p,this.phaseTime/p.duration);this.apply(m,d);
  }
}
