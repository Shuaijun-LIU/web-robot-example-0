// Read-only kinematics audit of a recorded state, in a separate, non-rendered
// MuJoCo model. Never restores or moves objects in the user's running scene.
import {readFileSync} from 'node:fs';
import {chromium} from 'playwright';
const source=readFileSync(process.argv[2],'utf8');
const report=JSON.parse(source.slice(source.indexOf('{\n  "status"'),source.lastIndexOf('\n}')+2));
const sample=report.pageState.trace.phaseSamples.at(-1);
const browser=await chromium.launch({headless:true});
try {
  const page=await browser.newPage();
  await page.route('**/__ik-audit',route=>route.fulfill({contentType:'text/html',body:'<!doctype html><html><body>Non-rendered IK audit</body></html>'}));
  await page.goto('http://127.0.0.1:3000/__ik-audit');
  console.log(await page.evaluate(async({sample,phase})=>{
    const {default:load}=await import('/node_modules/mujoco-js/dist/mujoco_wasm.js');
    const {loadScene}=await import('/node_modules/mujoco-react/dist/index.js');
    const {robots}=await import('/src/configs.ts');
    const {planMeasuredAssemblyPhase}=await import('/src/assemblyMeasuredPlanning.js');
    const T=await import('/node_modules/three/build/three.module.js');
    const m=await load(),{mjModel:model,mjData:data}=await loadScene(m,robots.frankaAssembly1.config);
    try {
      if(data.qpos.length!==sample.qpos.length)throw new Error('Snapshot model mismatch');
      data.qpos.set(sample.qpos);data.ctrl.set(sample.ctrl);m.mj_forward(model,data);
      const plans=Array.from({length:4},(_,i)=>({armKey:`r${i}`}));
      const named=(kind,name)=>{for(let i=0;i<model[`n${kind}`];i++){let s='';for(let p=model[`name_${kind}adr`][i];model.names[p];p++)s+=String.fromCharCode(model.names[p]);if(s===name)return i;}throw new Error(name);};
      const body=named('body','double_face_hammer'),site=named('site','r3_tcp');
      const tcp=()=>{const a=data.site_xmat.slice(site*9,site*9+9),p=data.site_xpos.slice(site*3,site*3+3);return new T.Matrix4().set(a[0],a[1],a[2],p[0],a[3],a[4],a[5],p[1],a[6],a[7],a[8],p[2],0,0,0,1);};
      const hammer=()=>new T.Matrix4().compose(new T.Vector3(...data.xpos.slice(body*3,body*3+3)),new T.Quaternion(data.xquat[body*4+1],data.xquat[body*4+2],data.xquat[body*4+3],data.xquat[body*4]),new T.Vector3(1,1,1));
      const results=[];
      for(const stage of phase.split(',')) {
        const relative=tcp().invert().multiply(hammer());
        planMeasuredAssemblyPhase(m,model,data,stage,plans);
        const path=plans[3].phasePaths?.[stage];
        if(path) {
          path.at(-1).forEach((q,j)=>{data.qpos[model.jnt_qposadr[named('jnt',`r3_joint${j+1}`)]]=q;});
          m.mj_kinematics(model,data);
          // Kinematic prediction ONLY in this disposable, non-rendered model.
          // It is not used as physical evidence and never reaches the demo.
          if(stage.startsWith('hammer-')) {
            const pose=tcp().multiply(relative),p=new T.Vector3(),q=new T.Quaternion();pose.decompose(p,q,new T.Vector3());
            const address=model.jnt_qposadr[model.body_jntadr[body]];
            data.qpos.set([...p.toArray(),q.w,q.x,q.y,q.z],address);m.mj_kinematics(model,data);
          }
        }
        results.push({stage,yaw:plans[3].hammerYaw,points:path?.length});
      }
      return results;
    } finally {data.delete();model.delete();}
  },{sample,phase:process.argv[3]??'hammer-stage'}));
} finally {await browser.close();}
