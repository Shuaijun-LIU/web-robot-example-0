import {useState} from 'react';
import type {MutableRefObject} from 'react';
import {CAMERA_KEYS,CAMERA_LABELS,selectedCameraViews} from './assemblyCameras.js';

export type CameraTiles=MutableRefObject<Map<string,HTMLDivElement>>;
export function AssemblyCameraPanel({selection,onSelection,tiles,status}:{selection:string;onSelection:(value:string)=>void;tiles:CameraTiles;status:string}){
  const [open,setOpen]=useState(true);
  return <section className={`assembly-cameras-panel ${selection==='all'&&open?'assembly-cameras-panel--all':''}`} aria-label="Camera views">
    <header>
      <button type="button" aria-expanded={open} onClick={()=>setOpen(!open)}>{open?'Hide cameras':'Cameras'}</button>
      {open&&<select aria-label="Camera view" value={selection} onChange={e=>onSelection(e.target.value)}>
        {CAMERA_KEYS.map((key:string,i:number)=><option value={key} key={key}>{CAMERA_LABELS[i]}</option>)}
        <option value="all">All five cameras</option>
      </select>}
      {open&&<span className="assembly-cameras-panel__badge">RGB · 10 Hz</span>}
    </header>
    {open&&<div className="assembly-cameras-panel__views">
      {selectedCameraViews(selection).map((key:string)=><figure key={key}>
        <div className="assembly-camera-tile" data-camera={key} ref={node=>{if(node)tiles.current.set(key,node);else tiles.current.delete(key);}} />
        <figcaption>{CAMERA_LABELS[CAMERA_KEYS.indexOf(key)]}</figcaption>
      </figure>)}
    </div>}
    {open&&status!=='ready'&&<p role="status">{status==='loading'?'Loading camera models…':status}</p>}
  </section>;
}
