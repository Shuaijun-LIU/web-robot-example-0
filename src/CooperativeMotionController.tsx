import {useEffect,useRef} from 'react';
import {findBodyByName,findJointByName,useBeforePhysicsStep,useMujoco} from 'mujoco-react';
import {loadCooperativePlan,requestCooperativeInspectionSteps} from './cooperativeMotion.js';
import type {CooperativePlan,CooperativeState} from './cooperativeMotion.js';
import type {CooperativeScene} from './cooperativeWorkcells.js';
import {CooperativeMotionRuntime} from './CooperativeMotionRuntime.js';

declare global {interface Window {cooperativeMotion?:{scene:CooperativeScene;state:CooperativeState;time:number;history:unknown[];metrics:Record<string,number>;stepInspection?:(ticks:number)=>boolean}}}

export function CooperativeMotionController({scene,requestId,resetGeneration,onStateChange}:{scene:CooperativeScene;requestId:number;resetGeneration:number;onStateChange:(s:CooperativeState)=>void}) {
  const simulation=useMujoco();
  const plan=useRef<CooperativePlan|null>(null),runtime=useRef<CooperativeMotionRuntime|null>(null),lastRequest=useRef(requestId);
  const lastState=useRef<CooperativeState|null>(null),callback=useRef(onStateChange);callback.current=onStateChange;
  useEffect(()=>{
    const abort=new AbortController();let cancelled=false;
    plan.current=null;runtime.current=null;lastRequest.current=requestId;lastState.current=null;
    delete window.cooperativeMotion;
    callback.current({phase:'loading',label:'Loading verified motion',stage:0});
    loadCooperativePlan(fetch,scene,`${import.meta.env.BASE_URL}assets/franka-cooperative/${scene}-motion.json`,abort.signal)
      .then(p=>{if(!cancelled){plan.current=p;callback.current({phase:'ready',label:'Ready',stage:0});}})
      .catch(error=>{if(!cancelled)callback.current({phase:'error',label:'Automatic motion unavailable',stage:0,reason:String(error)});});
    return ()=>{cancelled=true;abort.abort();runtime.current=null;delete window.cooperativeMotion;};
  },[scene,resetGeneration]);
  useBeforePhysicsStep((m,d)=>{
    if(requestId>lastRequest.current){
      lastRequest.current=requestId;if(!plan.current)return;
      try{
        runtime.current=new CooperativeMotionRuntime(plan.current,{body:n=>findBodyByName(m,n),joint:n=>findJointByName(m,n)});
        runtime.current.start(m,d);
      }catch(error){runtime.current=null;callback.current({phase:'error',label:'Cannot start motion',stage:0,reason:String(error)});}
    }
    const r=runtime.current;if(!r)return;
    r.step(m,d);window.cooperativeMotion={scene,state:r.state,time:r.time,history:r.history,metrics:r.metrics,
      ...(import.meta.env.DEV?{stepInspection:(ticks:number)=>requestCooperativeInspectionSteps(simulation.api,ticks)}:{})};
    if(r.state!==lastState.current){lastState.current=r.state;callback.current(r.state);}
  });
  return null;
}
