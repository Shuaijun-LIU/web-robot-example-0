import {useEffect} from 'react';
import {useThree} from '@react-three/fiber';
import {useMujoco} from 'mujoco-react';
import {installCooperativeTextures} from './cooperativeTextures.js';

export function CooperativePresentation() {
  const {scene}=useThree(),{status,mjModelRef}=useMujoco();
  useEffect(()=>{
    if(status!=='ready'||!mjModelRef.current)return;
    let installed:ReturnType<typeof installCooperativeTextures>|undefined;
    const frame=requestAnimationFrame(()=>{
      installed=installCooperativeTextures(scene,mjModelRef.current);
      document.documentElement.dataset.cooperativeTextures=String(installed.count);
    });
    return ()=>{cancelAnimationFrame(frame);installed?.dispose();delete document.documentElement.dataset.cooperativeTextures;};
  },[scene,status,mjModelRef]);
  return null;
}
