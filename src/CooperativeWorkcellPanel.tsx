import {COOPERATIVE_WORKCELLS} from './cooperativeWorkcells.js';
import type {CooperativeScene} from './cooperativeWorkcells.js';
import type {CooperativeState} from './cooperativeMotion.js';

export function CooperativeWorkcellPanel({scene,state,ready,paused,onRun}:{scene:CooperativeScene;state:CooperativeState;ready:boolean;paused:boolean;onRun:()=>void}) {
  const cell=COOPERATIVE_WORKCELLS[scene];
  return <aside className={`cooperative-panel${scene==='relay'?' cooperative-panel--relay':''}`} aria-label={cell.title}>
    <h2>{cell.title}</h2>
    <p>{cell.description}</p>
    <ol>{cell.roles.map((role,i)=><li key={role}><strong>Arm {i+1}</strong><span>{role}</span></li>)}</ol>
    <button className="cooperative-panel__play" disabled={!ready||state.phase!=='ready'} onClick={onRun}>Play task</button>
    <p role="status">{state.label}{paused?' · Paused':''}</p>
    {state.reason&&<p className="cooperative-panel__error">{state.reason}. {state.active?'Reset to restore manual control.':'Manual scene inspection remains available.'}</p>}
    <small>{cell.note}</small>
  </aside>;
}
