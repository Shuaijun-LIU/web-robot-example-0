#!/usr/bin/env python3
"""Compile, settle and probe both workcells using CPU MuJoCo (no renderer)."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh

ROOT=Path('public/assets/franka-cooperative').resolve()
MANIFEST=json.loads((ROOT/'manifest.json').read_text())
HOME=[1.570796,-.785398,0,-2.356194,0,1.570796,.785398]


def initialize(model):
    data=mujoco.MjData(model)
    for arm in range(4):
        data.ctrl[arm*8:arm*8+8]=[*HOME,255]
        for j in range(7):data.joint(f'r{arm}_joint{j+1}').qpos[0]=HOME[j]
    mujoco.mj_forward(model,data)
    return data


report={'success':False,'engine':mujoco.mj_versionString(),'scenes':{},'alignment':{}}
for asset,meta in MANIFEST['assets'].items():
    parts=[trimesh.load(p,force='mesh',process=False) for p in sorted((ROOT/'assets').glob(f'{asset}-collision-*.obj'))]
    bounds=np.array([np.vstack([p.vertices for p in parts]).min(0),np.vstack([p.vertices for p in parts]).max(0)])
    difference=float(np.max(np.abs(bounds-np.array(meta['bounds']))))
    report['alignment'][asset]={'maxBoundsDifference':difference,'collisionPieces':len(parts)}
    assert difference<.004,(asset,'collision/visual bounds mismatch',difference)

for scene in ['scan','pot']:
    m=mujoco.MjModel.from_xml_path(str(ROOT/f'{scene}.xml'));d=initialize(m)
    assert m.nu==32
    for i in range(4):assert m.site(f'r{i}_tcp').id>=0
    for mesh in range(m.nmesh):
        if m.mesh(mesh).name.endswith('_v0'):
            assert m.mesh_texcoordnum[mesh]>0,(m.mesh(mesh).name,'source UVs were lost in conversion')
    robot={i for i in range(m.nbody) if m.body(i).name.startswith(tuple(f'r{a}_' for a in range(4)))}
    max_pen=0.;last={}
    for tick in range(2500):
        mujoco.mj_step(m,d)
        for c in d.contact:
            if m.geom_bodyid[c.geom1] in robot or m.geom_bodyid[c.geom2] in robot:
                max_pen=max(max_pen,-float(c.dist))
        if tick==2000:last={o['name']:d.body(o['name']).xpos.copy() for o in MANIFEST['scenes'][scene]['objects']}
    assert np.isfinite(d.qpos).all()
    assert max_pen<.001,('robot collision at rest',scene,max_pen)
    drift={name:float(np.linalg.norm(d.body(name).xpos-pos)) for name,pos in last.items()}
    assert max(drift.values())<.002,('unstable idle object',scene,drift)
    report['scenes'][scene]={'actuators':m.nu,'maxRobotPenetration':max_pen,'lastSecondObjectDrift':drift,
        'positions':{name:d.body(name).xpos.tolist() for name in last}}

# Independent physical cavity probe: release a 20 mm diameter sphere over the
# opening, then require it to contact the inner pot floor below the rim.
xml=ET.parse(ROOT/'pot.xml').getroot()
for file in xml.findall('.//*[@file]'):
    path=ROOT/('assets' if file.tag in ['mesh','texture'] else '')/file.get('file')
    file.set('file',str(path))
xml.find('compiler').attrib.pop('meshdir',None)
xml.find('compiler').attrib.pop('texturedir',None)
body=ET.SubElement(xml.find('worldbody'),'body',name='cavity_probe',pos='0 0 .27')
ET.SubElement(body,'freejoint',name='cavity_probe_free')
ET.SubElement(body,'geom',type='sphere',size='.01',mass='.01')
m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));d=initialize(m)
for tick in range(2000):mujoco.mj_step(m,d)
probe=m.body('cavity_probe').id;pot=m.body('cooking_pot').id
contacts=[{m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]} for c in d.contact if c.dist<=0]
height=float(d.xpos[probe,2]-d.xpos[pot,2])
assert {probe,pot} in contacts,'probe does not rest on the pot'
assert .005<height<.055,('pot cavity is blocked or bottom missing',height)
report['cavityProbe']={'heightAbovePotOrigin':height,'supportedByPot':True}
report['success']=True
Path('artifacts/reports').mkdir(parents=True,exist_ok=True)
Path('artifacts/reports/cooperative-assets-native.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
