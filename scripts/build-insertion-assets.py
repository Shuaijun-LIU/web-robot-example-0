#!/usr/bin/env python3
"""Export existing robosuite constructors; CPU-only, no environment/rendering."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import robosuite
from robosuite.models.objects import StandWithMount, HookFrame


def save(root,path):
    ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('public/assets/franka-cooperative'));args=parser.parse_args()
    out=args.output;root=ET.parse(out/'scan.xml').getroot();root.set('model','Four Panda supported insertion')
    world=root.find('worldbody');assets=root.find('asset')
    for b in list(world.findall('body')):world.remove(b)
    # The horizontal insertion wrist needs overlap beyond the nominal center.
    # Adjust only this new workcell, not the shared robot or previous scenes.
    for frame in world.findall('frame'):
        if frame.find('attach') is not None:
            x,y,z=map(float,frame.get('pos').split());frame.set('pos',f'{x*.75/.78} {y*.75/.78} {z}')
    for a in list(assets):
        if a.tag!='model':assets.remove(a)
    for name,rgba,specular in [('insert_metal','.76 .77 .74 1','.4'),('insert_grip','.17 .18 .17 1','.08')]:
        ET.SubElement(assets,'material',name=name,rgba=rgba,specular=specular,shininess='.2')
    stand_args=dict(name='stand',size=(.06,.14,.16),mount_location=(0,.045),mount_width=.030,
      wall_thickness=.004,base_thickness=.012,initialize_on_side=False,add_hole_vis=False,
      density=2700,use_texture=False,solref=(.005,1),solimp=(.99,.999,.0001),friction=(.8,.01,.001))
    frame_args=dict(name='insert',frame_length=.095,frame_height=.18,frame_thickness=.0075,
      hook_height=.012,grip_location=.0525,grip_size=(.0127,.03175),tip_size=None,
      density=2700,use_texture=False,solref=(.005,1),solimp=(.99,.999,.0001),friction=(.8,.01,.001))
    sources=out/'insertion-sources';sources.mkdir(exist_ok=True)
    objects=[]
    for name,obj,pos,euler in [('stand',StandWithMount(**stand_args),[0,-.06,.222],[0,0,0]),
                              ('insert',HookFrame(**frame_args),[0,.32,.1437],[90,0,0])]:
        original=obj.get_obj();save(copy.deepcopy(original),sources/f'{name}-generated.xml')
        body=copy.deepcopy(original);body.set('name',name);body.set('pos',' '.join(map(str,pos)));body.set('euler',' '.join(map(str,euler)))
        for g in body.findall('geom'):
            visual=g.get('contype')=='0';g.set('group','2' if visual else '3')
            if visual:g.set('mass','0');g.set('material','insert_grip' if 'grip' in g.get('name','') else 'insert_metal');g.attrib.pop('rgba',None)
            elif 'grip' in g.get('name',''):g.set('density','1000')
        for site in body.findall('site'):site.set('rgba','0 0 0 0');site.set('group','1')
        world.append(body);objects.append({'name':name,'asset':type(obj).__name__,'position':pos,'euler':euler})
    for name,x,y,sx,sy,h in [('stand_support',0,-.06,.021,.065,.04),('incoming_insert',0,.32,.08,.14,.03),
                           ('insert_staging',.25,.10,.08,.14,.03),('assembly_output',-.30,-.06,.021,.075,.04)]:
        b=ET.SubElement(world,'body',name=name,pos=f'{x} {y} {.1+h/2}')
        ET.SubElement(b,'geom',name=name+'_surface',type='box',size=f'{sx} {sy} {h/2}',rgba='.38 .40 .35 1',friction='.8 .01 .001')
    save(root,out/'insertion.xml')
    source_root=Path(robosuite.__file__).resolve().parent
    records=[]
    for relative in ['models/objects/composite/stand_with_mount.py','models/objects/composite/hook_frame.py','models/objects/generated_objects.py','environments/manipulation/tool_hang.py']:
        p=source_root/relative;shutil.copy2(p,sources/p.name)
        records.append({'source':'robosuite/'+relative,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'license':'MIT'})
    shutil.copy2(source_root.parent/'LICENSE',out/'licenses/robosuite.txt')
    manifest={'version':1,'sourceVersion':robosuite.__version__,'armRingRadius':.75,'sources':records,'parameters':{'stand':stand_args,'insert':frame_args},
      'objects':objects,'mouth':[0,.045,.08],'tip':[.04375,0,-.09],'grip':[.04375,0,.04875],
      'densityOverrides':{'insert_grip':1000},'note':'Source-generated geometry, no remeshing; two independent free objects. Dimensions and densities are simulation assumptions.'}
    (out/'insertion-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    common=json.loads((out/'manifest.json').read_text());common['scenes']['insertion']={'arms':4,'objects':objects,'homeJoints':common['scenes']['drawer']['homeJoints'],'tableHeight':.1}
    (out/'manifest.json').write_text(json.dumps(common,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
