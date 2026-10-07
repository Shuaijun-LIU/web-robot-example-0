export const PANDA_LIMITS=[[-2.8973,2.8973],[-1.7628,1.7628],[-2.8973,2.8973],[-3.0718,-.0698],[-2.8973,2.8973],[-.0175,3.7525],[-2.8973,2.8973]];
const jointVector=q=>Array.isArray(q)&&q.length===7&&q.every((v,j)=>Number.isFinite(v)&&v>=PANDA_LIMITS[j][0]&&v<=PANDA_LIMITS[j][1]);
const point=p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite);
function validGate(g){
  if(!g||typeof g!=='object')return false;
  const name=n=>typeof n==='string'&&n.length>0;
  if(g.type==='home')return true;
  if(g.type==='joint-range')return name(g.joint)&&[g.min,g.max,g.maxSpeed].every(Number.isFinite)&&g.min<=g.max&&g.maxSpeed>=0;
  if(!name(g.object))return false;
  if(g.type==='grasp')return Number.isInteger(g.arm)&&g.arm>=0&&g.arm<4;
  if(g.type==='released')return true;
  if(g.type==='height')return Number.isFinite(g.minZ);
  if(g.type==='support')return name(g.body);
  if(g.type==='inside')return name(g.container)&&Number.isFinite(g.radius)&&g.radius>0&&Number.isFinite(g.minZ)&&Number.isFinite(g.maxZ)&&g.minZ<g.maxZ&&((g.minX===undefined&&g.maxX===undefined)||([g.minX,g.maxX].every(Number.isFinite)&&g.minX<g.maxX));
  if(g.type==='scan')return name(g.scanner)&&[g.origin,g.axis,g.target,g.normal].every(point)&&Math.abs(Math.hypot(...g.axis)-1)<.001&&Math.abs(Math.hypot(...g.normal)-1)<.001;
  return false;
}
export function validateCooperativePlan(p,scene) {
  if(!p||p.version!==1||p.scene!==scene||!['scan','pot','drawer'].includes(scene)||!jointVector(p.initialJoints))return 'Invalid program header';
  if(!Array.isArray(p.initialObjects)||p.initialObjects.some(o=>typeof o.name!=='string'||!point(o.position)))return 'Invalid initial objects';
  if(p.initialFixtureJoints!==undefined&&(!Array.isArray(p.initialFixtureJoints)||p.initialFixtureJoints.some(j=>!j||typeof j.joint!=='string'||!j.joint||!Number.isFinite(j.position)||!Number.isFinite(j.tolerance)||j.tolerance<=0)))return 'Invalid initial fixture joints';
  if(!Array.isArray(p.phases)||!p.phases.length)return 'Empty motion';
  let previous=Array.from({length:4},()=>p.initialJoints),grip=[255,255,255,255];
  for(const phase of p.phases){
    if(typeof phase.name!=='string'||!Number.isFinite(phase.duration)||phase.duration<.02||phase.duration>60)return 'Invalid phase duration/name';
    if(phase.settleTimeout!==undefined&&(!Number.isFinite(phase.settleTimeout)||phase.settleTimeout<0||phase.settleTimeout>3))return 'Invalid settling window';
    if(!Array.isArray(phase.paths)||phase.paths.length!==4||phase.paths.some(path=>!Array.isArray(path)||path.length<2||!path.every(jointVector)))return 'Invalid joint path';
    if(!Array.isArray(phase.grippers)||phase.grippers.length!==4||phase.grippers.some(p=>!Array.isArray(p)||p.length!==2||p.some(v=>!Number.isFinite(v)||v<0||v>255)))return 'Invalid gripper command';
    if(phase.paths.some((p,a)=>p[0].some((v,j)=>Math.abs(v-previous[a][j])>1e-5))||phase.grippers.some((p,a)=>Math.abs(p[0]-grip[a])>1e-5))return 'Discontinuous program';
    if(!Array.isArray(phase.allowedContacts)||phase.allowedContacts.some(p=>!Array.isArray(p)||p.length!==2||p.some(n=>typeof n!=='string'||!n)))return 'Invalid contact allowance';
    if(!Array.isArray(phase.carry)||phase.carry.some(c=>typeof c.object!=='string'||!Number.isInteger(c.arm)||c.arm<0||c.arm>3))return 'Invalid carried object';
    if(!Array.isArray(phase.gates)||!phase.gates.every(validGate))return 'Invalid physical gate';
    previous=phase.paths.map(p=>p.at(-1));grip=phase.grippers.map(p=>p[1]);
  }
  return null;
}
export function sampleCooperativePhase(phase,fraction) {
  const t=Math.max(0,Math.min(1,fraction)),s=t*t*t*(10+t*(-15+6*t));
  return {joints:phase.paths.map(path=>{
    const u=s*(path.length-1),i=Math.min(path.length-2,Math.floor(u)),f=u-i;
    return path[i].map((v,j)=>v+(path[i+1][j]-v)*f);
  }),grippers:phase.grippers.map(([a,b])=>a+(b-a)*s)};
}
export function checkCooperativeGate(g,o) {
  if(!o)return 'missing-observation';
  if(g.type==='joint-range')return Number.isFinite(o.position)&&Number.isFinite(o.speed)&&o.position>=g.min&&o.position<=g.max&&Math.abs(o.speed)<=g.maxSpeed?null:'fixture-position-mismatch';
  if(g.type==='grasp')return o.bilateral&&o.aperture>.002&&o.aperture<.081?null:'missing-bilateral-contact';
  if(g.type==='support')return o.supported&&o.speed<.03?null:'object-not-supported';
  if(g.type==='released')return o.fingerContacts===0?null:'fingers-not-clear';
  if(g.type==='height')return o.height>=g.minZ?null:'object-not-lifted';
  if(g.type==='home')return o.error<.025?null:'arms-not-home';
  if(g.type==='inside')return o.supported&&o.speed<.04&&Math.hypot(o.relative[0],o.relative[1])<g.radius&&o.relative[2]>=g.minZ&&o.relative[2]<=g.maxZ&&(g.minX===undefined||(o.relative[0]>=g.minX&&o.relative[0]<=g.maxX))?null:'outside-destination';
  if(g.type==='scan')return o.distance>=.025&&o.distance<=.09&&o.offAxis<.02&&o.facing>.9?null:'scan-pose-mismatch';
  return 'unknown-gate';
}
export async function loadCooperativePlan(fetcher,scene,url,signal) {
  const response=await fetcher(url,{signal});
  if(!response.ok)throw new Error(`Motion HTTP ${response.status}`);
  if(!response.headers.get('content-type')?.includes('json'))throw new Error('Motion response is not JSON');
  const plan=await response.json(),reason=validateCooperativePlan(plan,scene);
  if(reason)throw new Error(reason);
  return plan;
}

// Development inspection uses the provider's real fixed-step loop. This only
// schedules ticks; it neither seeks a trajectory nor writes simulation state.
export function requestCooperativeInspectionSteps(api,ticks){
  if(!api||!Number.isInteger(ticks)||ticks<1||ticks>1000)return false;
  api.step(ticks);return true;
}
