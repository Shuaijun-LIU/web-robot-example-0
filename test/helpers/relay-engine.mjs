import {readFileSync,readdirSync} from 'node:fs';
import {resolve,relative} from 'node:path';
import loadMujoco from 'mujoco-js';
export async function relayEngine(){
  const root=resolve('public/assets/franka-cooperative'),mj=await loadMujoco();mj.FS.mkdir('/cell');
  function mount(dir){for(const e of readdirSync(dir,{withFileTypes:true})){
    const path=resolve(dir,e.name),dest='/cell/'+relative(root,path);
    if(e.isDirectory()){mj.FS.mkdir(dest);mount(path);}else mj.FS.writeFile(dest,readFileSync(path));
  }}mount(root);
  const m=mj.MjModel.loadFromXML('/cell/relay.xml'),d=new mj.MjData(m);
  const id=(type,name)=>mj.mj_name2id(m,mj.mjtObj[type].value,name);
  const find={body:n=>id('mjOBJ_BODY',n),joint:n=>id('mjOBJ_JOINT',n)};
  const q=[1.570796,-.785398,0,-2.356194,0,1.570796,.785398];
  for(let a=0;a<4;a++){d.ctrl.set([...q,255],a*8);for(let j=0;j<7;j++)d.qpos[m.jnt_qposadr[find.joint(`r${a}_joint${j+1}`)]]=q[j];}
  mj.mj_forward(m,d);for(let i=0;i<500;i++)mj.mj_step(m,d);
  return {mj,m,d,find,q};
}
