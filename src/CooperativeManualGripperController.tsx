import {useEffect} from 'react';
import {findActuatorByName,useMujoco} from 'mujoco-react';
import type {ControlTarget} from './controlTargets.js';
import {toggleCooperativeGripper} from './cooperativeManual.js';

export function CooperativeManualGripperController({target,enabled}:{target:ControlTarget;enabled:boolean}){
  const {mjModelRef,mjDataRef}=useMujoco();
  useEffect(()=>{
    const onKeyDown=(event:KeyboardEvent)=>{
      if(!enabled||event.repeat||event.key.toLowerCase()!=='v')return;
      if(event.target instanceof HTMLElement&&event.target.closest('input,textarea,[contenteditable="true"]'))return;
      const model=mjModelRef?.current,data=mjDataRef?.current;
      if(!model||!data||!target.gripperActuator)return;
      toggleCooperativeGripper(data,findActuatorByName(model,target.gripperActuator),enabled);
    };
    window.addEventListener('keydown',onKeyDown);
    return ()=>window.removeEventListener('keydown',onKeyDown);
  },[enabled,target.gripperActuator,mjModelRef,mjDataRef]);
  return null;
}
