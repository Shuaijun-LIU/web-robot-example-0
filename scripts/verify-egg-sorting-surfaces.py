#!/usr/bin/env python3
"""Read-only mesh-surface audit of the checked four-arm physical replay."""
import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('sorting',Path(__file__).with_name('solve-egg-sorting.py'))
sorting=importlib.util.module_from_spec(spec);spec.loader.exec_module(sorting)
root=Path(__file__).resolve().parents[1]
plan=json.loads((root/'public/assets/franka-egg-sorting/sorting-motion.json').read_text())
original_gate=sorting.Sorting.gate
sorting.Sorting.contacts=sorting.base.Transfer.contacts
records=[]

def audited_gate(sim,task,gate):
    observation=original_gate(sim,task,gate)
    if gate in ('bilateral','carried','supported'):
        sim.egg=sim.eggs[task['eggIndex']];sim.fingers=sim.fingerSets[task['arm']]
        sim.max_surface_gap=0.
        sorting.base.Transfer.audit_visible_contacts(sim)
        records.append({'arm':task['arm'],'egg':task['eggIndex'],'gate':gate,'maxMeshSurfaceGap':sim.max_surface_gap})
        if sim.max_surface_gap>.001:raise RuntimeError(f'Contact is off the finger mesh surface: {records[-1]}')
    return observation

sorting.Sorting.gate=audited_gate
result=sorting.replay_timeline(root,plan['tasks'],[t['start'] for t in plan['tasks']])
assert result['counts']==[4,4,4,4]
report={'success':True,'counts':result['counts'],'maxMeshSurfaceGap':max(r['maxMeshSurfaceGap'] for r in records),
    'scope':'All grasp, carried and supported phase gates; distance from physical contacts to the sourced finger mesh triangles.',
    'records':records}
(root/'artifacts/reports/demo2-four-arm-surfaces.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
