#!/usr/bin/env python3
"""Source-mesh-only payload and obstacle conversion for Demo7; CPU-only."""
import argparse
import copy
import hashlib
import importlib.util
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

spec=importlib.util.spec_from_file_location('asset_base',Path(__file__).with_name('build-cooperative-assets.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
add,save,text=base.add,base.save,base.text


def main():
    p=argparse.ArgumentParser();p.add_argument('--objects',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('public/assets/franka-cooperative'));args=p.parse_args()
    out=args.output;package=base.Package(out);sources=out/'relay-sources';sources.mkdir(exist_ok=True);records=[]
    for key,folder,index,scale in [('relay_panel','104_board',3,.2),('relay_block','086_woodenblock',0,.5)]:
        source=args.objects/folder/'visual'/f'base{index}.glb'
        visual=base.chunks(source,base.UP,scale)
        # This solid panel/block has no handle hole. Use the actual visible
        # surface vertices, not the upstream padded collision envelope.
        collision=[trimesh.util.concatenate([m.copy() for m in visual]).convex_hull]
        package.export(key,visual,collision,source,scale,{'source':f'RoboTwin/{folder}/visual/base{index}.glb',
          'license':'MIT','collisionConversion':'Convex hull of visible mesh; no added margin; source padded collider not used.'})
        # The upstream OBJ exporter appends an extra empty line. Normalize only
        # our generated text files; keep original archived assets byte-exact.
        for mesh_file in (out/'assets').glob(f'{key}-*.obj'):
            mesh_file.write_text(mesh_file.read_text().rstrip()+'\n')
        for kind,path in [('visual',source),('collision-reference',args.objects/folder/'collision'/f'base{index}.glb'),
                          ('metadata',args.objects/folder/f'model_data{index}.json')]:
            name=f'{key}-{kind}{path.suffix}';shutil.copy2(path,sources/name)
            records.append({'file':name,'source':f'RoboTwin/{folder}/{path.relative_to(args.objects/folder)}',
              'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    root,world=package.scene('relay',['relay_panel','relay_block'])
    for f in world.findall('frame'):
        x,y,z=map(float,f.get('pos').split());f.set('pos',text([x*.85/.78,y*.85/.78,z]))
    package.pad(world,'panel_start',0,-.05,.12,.10,height=.12)
    package.pad(world,'panel_output',0,.38,.12,.10,height=.12)
    package.object(world,'relay','panel','relay_panel',[0,-.05,.222],.9)
    # Sourced cube stacks: tall outer posts require payload elevation; the
    # lower center leaves physical palm access for the north donor.
    for i,x in enumerate([-.10,0,.10]):
        b=package.object(world,'relay',f'barrier_{i}','relay_block',[x,.16,.1],.3)
        b.remove(b.find('freejoint'));b.remove(b.find('inertial'))
        if i!=1:
            for level in [1,2]:
                for geom in list(b.findall('geom'))[:2]:
                    extra=copy.deepcopy(geom);extra.set('pos',text([0,0,level*.05]))
                    if extra.get('name'):extra.set('name',extra.get('name')+f'_level{level}')
                    b.append(extra)
    package.manifest['scenes']['relay']['objects']=[o for o in package.manifest['scenes']['relay']['objects'] if o['name']=='panel']
    save(root,out/'relay.xml')
    manifest={'version':1,'sources':records,'assets':package.manifest['assets'],'armRingRadius':.85,
      'panelMass':.9,'panelInitial':[0,-.05,.222],'obstacles':[f'barrier_{i}' for i in range(3)],
      'license':'RoboTwin official asset distribution MIT; see licenses/RoboTwin.txt',
      'licenseSource':'https://huggingface.co/datasets/TianxingChen/RoboTwin2.0/blob/main/README.md',
      'note':'Source visuals/textures retained; two visible support pads; no custom grip fixture. Mass is a simulation assumption.'}
    (out/'relay-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    common=json.loads((out/'manifest.json').read_text());common['scenes']['relay']=package.manifest['scenes']['relay']
    common['assets'].update(package.manifest['assets']);(out/'manifest.json').write_text(json.dumps(common,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
