import {COOPERATIVE_WORKCELLS} from './cooperativeWorkcells.js';
import type {CooperativeScene} from './cooperativeWorkcells.js';

export function CooperativeWorkcellPanel({scene}:{scene:CooperativeScene}) {
  const cell=COOPERATIVE_WORKCELLS[scene];
  return <aside className="cooperative-panel" aria-label={cell.title}>
    <h2>{cell.title}</h2>
    <p>{cell.description}</p>
    <ol>{cell.roles.map((role,i)=><li key={role}><strong>Arm {i+1}</strong><span>{role}</span></li>)}</ol>
    <p role="status">Layout preview · manual controls available</p>
    <small>{cell.note}</small>
  </aside>;
}
