#!/usr/bin/env python3
"""Reuse RoboCasa Drawer topology/dimensioning and sourced panel/handle assets.

CPU-only conversion. Existing scan/pot XML and object geometry are unchanged.
The drawer slide is passive; only robot controls can move it during playback.
"""
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
from PIL import Image

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('cooperative_assets',HERE/'build-cooperative-assets.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
add,text,save=base.add,base.text,base.save


def fixture_asset(source,key,assets,out,scale):
    """Keep source visual mesh, texture and compound primitive collisions."""
    xml=ET.parse(source).getroot();obj=xml.find(".//body[@name='object']")
    mesh=xml.find('asset/mesh');mesh_file=source.parent/mesh.get('file')
    filename=f'drawer-{key}.obj';shutil.copy2(mesh_file,out/'assets'/filename)
    add(assets,'mesh',name=f'drawer_{key}',file=filename,scale=text(scale))
    texture=xml.find('asset/texture');image=Image.open(source.parent/texture.get('file')).convert('RGB')
    image.thumbnail((1024,1024));image.save(out/'assets'/f'drawer-{key}.png')
    add(assets,'texture',name=f'drawer_{key}_tex',type='2d',file=f'drawer-{key}.png')
    add(assets,'material',name=f'drawer_{key}_mat',texture=f'drawer_{key}_tex',rgba='1 1 1 1',specular='.25',shininess='.25')
    result=[]
    for source_geom in obj.findall('geom'):
        kind=source_geom.get('class')
        if kind not in ['visual','collision']:continue
        g=copy.deepcopy(source_geom);g.attrib.pop('class',None)
        if kind=='visual':
            g.set('mesh',f'drawer_{key}');g.set('material',f'drawer_{key}_mat');g.set('contype','0');g.set('conaffinity','0');g.set('group','2');g.set('mass','0')
        else:
            p=np.fromstring(g.get('pos','0 0 0'),sep=' ')*scale
            size=np.fromstring(g.get('size'),sep=' ')
            if g.get('type')=='box':size*=scale
            else:
                if not np.allclose(scale,scale[0]):raise ValueError('Only uniform scaling for cylinder handle')
                size*=scale[0]
            g.set('pos',text(p));g.set('size',text(size));g.set('group','3')
            g.set('friction','1 .015 .001');g.set('solref','.005 1');g.set('solimp','.99 .999 .0001');g.set('density','600')
        result.append(g)
    return result


def drawer(world,source,name,xy,yaw,panel,handle,size):
    # The following positions/sizes are RoboCasa Drawer._create_cab formulas,
    # applied to its original housing/inner_box geoms; no cavity is convexified.
    w,d,h=size;x,y,z=np.array(size)/2;th=.006
    ix,iy,iz=x-2*th-.001,y-2*th,z-2*th-.001
    sizes={'top':[x,y-th,th],'bottom':[x,y-th,th],'back':[x-2*th,th,z-2*th],
      'left':[th,y-th,z-2*th],'right':[th,y-th,z-2*th],
      'inner_bottom':[ix,iy,th],'inner_back':[ix-2*th,th,iz-2*th],
      'inner_left':[th,iy,iz-2*th],'inner_right':[th,iy,iz-2*th]}
    positions={'top':[0,th,z-th],'bottom':[0,th,-z+th],'back':[0,y-th,0],
      'left':[-x+th,th,0],'right':[x-th,th,0],
      'inner_bottom':[0,0,-iz+th],'inner_back':[0,iy-th,0],
      'inner_left':[-ix+th,0,0],'inner_right':[ix-th,0,0]}
    template=ET.parse(source).getroot().find(".//body[@name='object']")
    housing=add(world,'body',name=f'{name}_housing',pos=text([*xy,.1+z]),euler=f'0 0 {yaw}')
    inner=add(housing,'body',name=f'{name}_drawer')
    j=copy.deepcopy(template.find('.//joint'));j.set('name',f'{name}_slide');j.set('range','-.225 0');j.set('frictionloss','.25');inner.append(j)
    for g in template.iter('geom'):
        old=g.get('name')
        if old not in sizes:continue
        new=copy.deepcopy(g);new.attrib.pop('class',None);new.attrib.pop('material',None)
        new.set('name',f'{name}_{old}');new.set('size',text(sizes[old]));new.set('pos',text(positions[old]))
        new.set('rgba','.62 .63 .59 1' if old.startswith('inner') else '.67 .64 .57 1')
        new.set('friction','.65 .01 .001');new.set('solref','.005 1')
        (inner if old.startswith('inner') else housing).append(new)
    front=add(inner,'body',name=f'{name}_front',pos=text([0,-y+th,0]))
    for g in panel:front.append(copy.deepcopy(g))
    # Handle001 is vertical in its source. Rotate 90 degrees about Y to make
    # a horizontal drawer handle; its mounting-pad back touches the front face.
    hb=add(inner,'body',name=f'{name}_handle',pos=text([0,-y-.02456,0]),euler='0 90 0')
    for g in handle:hb.append(copy.deepcopy(g))
    add(hb,'site',name=f'{name}_grasp',pos='0 -.017739 .000504',size='.002',group='1',rgba='.5 .4 .2 0')
    return {'name':name,'joint':f'{name}_slide','position':[*xy,.1+z],'yaw':yaw,'size':list(size),
      'openPosition':-.205,'floorHeight':float(.1+z-iz+2*th),'handleBody':f'{name}_handle','drawerBody':f'{name}_drawer'}


def main():
    p=argparse.ArgumentParser();p.add_argument('--robocasa',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('public/assets/franka-cooperative'));args=p.parse_args()
    out=args.output;assetroot=args.robocasa/'robocasa/models/assets'
    files={'housing':assetroot/'fixtures/cabinets/drawer.xml',
      'panel':assetroot/'fixtures/cabinets/cabinet_panels/CabinetDoorPanel029/model.xml',
      'handle':assetroot/'fixtures/handles/CabinetHandle001/model.xml',
      'dimensioning':args.robocasa/'robocasa/models/fixtures/cabinets.py'}
    root=ET.parse(out/'scan.xml').getroot();root.set('model','Four Panda drawer kitting')
    world=root.find('worldbody');assets=root.find('asset')
    for body in list(world.findall('body')):world.remove(body)
    size=(.28,.30,.17)
    panel_mesh=files['panel'].parent/'visuals/CabinetDoorPanel029.obj'
    bounds=trimesh.load(panel_mesh,force='mesh',process=False).bounds
    panel_scale=np.array([size[0]-.003,.012,size[2]-.003])/(bounds[1]-bounds[0])
    panel=fixture_asset(files['panel'],'panel',assets,out,panel_scale)
    handle=fixture_asset(files['handle'],'handle',assets,out,np.ones(3))
    fixtures=[drawer(world,files['housing'],'south',[.32,0],0,panel,handle,size),
      drawer(world,files['housing'],'north',[-.32,0],180,panel,handle,size)]
    # Reuse existing free-body object definitions exactly (geometry and mass).
    old_world=ET.parse(out/'scan.xml').getroot().find('worldbody')
    objects=[]
    for name,asset,xy,mass in [('tea_box','tea',[.32,-.045],.09),('coffee_box','coffee',[-.32,.045],.10),('order_tray','tray',[0,0],.12)]:
        body=copy.deepcopy(old_world.find(f"body[@name='{name}']"));bottom=fixtures[0]['floorHeight']+.001 if name!='order_tray' else .115
        yaw=90 if name=='tea_box' else 0
        body.set('pos',text([*xy,bottom]));body.set('euler',f'0 0 {yaw}');world.append(body)
        objects.append({'name':name,'asset':asset,'position':[*xy,bottom],'mass':mass,'yaw':yaw})
    base.Package.pad(world,'order_pad',0,0,.18,.125,height=.014,color='.38 .40 .35 1')
    save(root,out/'drawer.xml')
    sources=[];source_dir=out/'drawer-sources';source_dir.mkdir(exist_ok=True)
    for key,file in files.items():
        shutil.copy2(file,source_dir/f'{key}{file.suffix}')
        sources.append({'role':key,'source':str(file.relative_to(args.robocasa)),
          'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'license':'MIT' if file.suffix=='.py' else 'CC-BY-4.0'})
    for key in ['panel','handle']:
        directory=files[key].parent
        for file in [*directory.glob('visuals/*.obj'),*directory.glob('visuals/*.png')]:
            sources.append({'role':f'{key}-{file.suffix[1:]}','source':str(file.relative_to(args.robocasa)),
              'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'license':'CC-BY-4.0'})
    shutil.copy2(args.robocasa/'LICENSE',out/'licenses/RoboCasa-code.txt')
    shutil.copy2(args.robocasa/'README.md',source_dir/'RoboCasa-source-README.md')
    manifest={'version':1,'sources':sources,'fixtures':fixtures,'objects':objects,
      'transforms':{'panelScale':panel_scale.tolist(),'handleRotationDegrees':[0,90,0]},
      'physics':{'drawerActuators':0,'damping':10,'frictionloss':.25,'range':[-.225,0]},
      'note':'Source Drawer parametric housing resized to tabletop scale; original panel and handle visual/collision geometry retained.'}
    (out/'drawer-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    common=json.loads((out/'manifest.json').read_text());common['scenes']['drawer']={'arms':4,'objects':objects,'homeJoints':base.HOME,'tableHeight':.1}
    (out/'manifest.json').write_text(json.dumps(common,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
