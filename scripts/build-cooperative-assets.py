#!/usr/bin/env python3
"""CPU-only conversion of sourced objects for the two cooperative workcells.

Geometry is transformed from the original meshes. Tables and support pads are
the only procedural visuals. Each exported collision piece remains separate.
"""
import argparse
import copy
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from PIL import Image

HOME = [1.570796, -.785398, 0, -2.356194, 0, 1.570796, .785398]
UP = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1.]])


def text(values):
    return ' '.join(f'{float(v):.9g}' for v in values)


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})


def save(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')


def chunks(path, transform, scale):
    scene = trimesh.load(path, force='scene', process=False)
    result=[]
    for node in scene.graph.nodes_geometry:
        matrix, key = scene.graph[node]
        mesh=scene.geometry[key].copy()
        mesh.apply_transform(transform @ matrix)
        mesh.vertices *= scale
        result.append(mesh)
    return result


class Package:
    def __init__(self, out):
        self.out=out
        (out/'assets').mkdir(parents=True,exist_ok=True)
        (out/'licenses').mkdir(exist_ok=True)
        self.assets={}
        self.manifest={'version':1,'assets':{},'scenes':{}}

    def robot(self, source):
        # Meshes and arm dynamics are sourced without geometric stretching.
        panda=ET.parse(source/'panda.xml').getroot()
        panda.remove(panda.find('keyframe'))
        for mesh in panda.findall('asset/mesh'):
            shutil.copy2(source/'assets'/mesh.get('file'),self.out/'assets'/mesh.get('file'))
        hand=panda.find(".//body[@name='hand']")
        add(hand,'site',name='tcp',pos='0 0 .103',size='.004',group='1',rgba='.45 .4 .26 .3')
        grip=panda.find("actuator/general[@name='actuator8']")
        grip.set('name','gripper')
        # 0..255 remains 0..40 mm jaw stroke; finite 40 N actuator limit.
        grip.set('gainprm',str(.04*1000/255)+' 0 0')
        grip.set('biasprm','0 -1000 -20')
        grip.set('forcerange','-40 40')
        for finger in ['left_finger','right_finger']:
            for geom in hand.find(f"body[@name='{finger}']").findall('geom'):
                if geom.get('class')!='visual':
                    geom.set('friction','1 .015 .001')
                    geom.set('solref','.005 1')
        save(panda,self.out/'panda.xml')
        shutil.copy2(source/'LICENSE',self.out/'licenses/Panda.txt')

    def export(self,key,visual,collision,source,scale,extra=None):
        bounds=np.array([np.vstack([m.vertices for m in visual]).min(0),np.vstack([m.vertices for m in visual]).max(0)])
        offset=np.r_[bounds.mean(0)[:2],bounds[0,2]]
        for mesh in visual+collision:mesh.vertices-=offset
        bounds-=offset
        definition={'visual':[],'collision':[]}
        for i,mesh in enumerate(visual):
            filename=f'{key}-visual-{i}.obj'
            (self.out/'assets'/filename).write_text(trimesh.exchange.obj.export_obj(mesh,include_texture=True,write_texture=False))
            material=getattr(mesh.visual,'material',None)
            image=getattr(material,'baseColorTexture',None)
            if image is None:image=getattr(material,'image',None)
            color=getattr(material,'baseColorFactor',None)
            if color is None:color=getattr(material,'diffuse',None)
            record={'file':filename,'rgba':[1.,1.,1.,1.] if color is None else (np.array(color)/255).tolist()}
            if image is not None:
                image=image.convert('RGB');image.thumbnail((1024,1024))
                texture=f'{key}-texture-{i}.png';image.save(self.out/'assets'/texture)
                record['texture']=texture
            definition['visual'].append(record)
        for i,mesh in enumerate(collision):
            filename=f'{key}-collision-{i}.obj'
            mesh.export(self.out/'assets'/filename,include_texture=False)
            definition['collision'].append(filename)
        self.assets[key]=definition
        self.manifest['assets'][key]={
            'source':str(source), 'sourceSha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'scale':scale,'bounds':bounds.tolist(),'dimensions':(bounds[1]-bounds[0]).tolist(),
            'offsetAfterTransform':offset.tolist(),'visualParts':len(visual),'collisionParts':len(collision),
            **(extra or {}),
        }

    def robotwin(self,key,objects,folder,index,scale):
        source=objects/folder/'visual'/f'base{index}.glb'
        visual=chunks(source,UP,scale)
        collision=chunks(objects/folder/'collision'/f'base{index}.glb',UP,scale)
        self.export(key,visual,collision,source,scale,{'source':f'RoboTwin/{folder}/visual/base{index}.glb','transform':'source scene graph, then Y-up to Z-up; XY centered; bottom Z=0'})

    def robocasa(self,key,objects,folder,factor=1):
        directory=objects/folder;source=directory/'model.xml'
        root=ET.parse(source).getroot()
        meshes={m.get('name'):m for m in root.findall('asset/mesh')}
        materials={m.get('name'):m for m in root.findall('asset/material')}
        textures={m.get('name'):m for m in root.findall('asset/texture')}
        visual=[];collision=[]
        for geom in root.findall('.//body[@name="object"]/geom'):
            if geom.get('mesh') is None:continue
            definition=meshes[geom.get('mesh')]
            quat=np.fromstring(definition.get('refquat','1 0 0 0'),sep=' ')
            # MJCF refquat maps reference coordinates to mesh coordinates;
            # the inverse rotation brings vertices into the body's frame.
            transform=trimesh.transformations.quaternion_matrix(quat).T
            scale=np.fromstring(definition.get('scale','1 1 1'),sep=' ')*factor
            parts=chunks(directory/definition.get('file'),transform,scale)
            is_visual=geom.get('class')=='visual' or geom.get('contype')=='0'
            if is_visual:
                mat=materials.get(geom.get('material'))
                if mat is not None and mat.get('texture') in textures:
                    texture=Image.open(directory/textures[mat.get('texture')].get('file'))
                    for mesh in parts:
                        mesh.visual.material=trimesh.visual.material.SimpleMaterial(image=texture,diffuse=[255,255,255,255])
                visual.extend(parts)
            else:collision.extend(parts)
        self.export(key,visual,collision,source,factor,{'source':f'RoboCasa/{folder}/model.xml','transform':'source mesh scale and inverse refquat; uniform scene scale; XY centered; bottom Z=0','license':'CC-BY-4.0'})

    def scene(self,key,used):
        root=ET.Element('mujoco',model=f'Four Panda cooperative {key}')
        add(root,'compiler',angle='degree',meshdir='assets',texturedir='assets')
        option=add(root,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='50',noslip_iterations='10')
        add(option,'flag',multiccd='enable')
        visual=add(root,'visual')
        add(visual,'headlight',diffuse='.75 .75 .75',ambient='.4 .4 .4',specular='.1 .1 .1')
        assets=add(root,'asset');add(assets,'model',name='panda_model',file='panda.xml')
        for name in used:
            for i,part in enumerate(self.assets[name]['visual']):
                add(assets,'mesh',name=f'{name}_v{i}',file=part['file'])
                mat={'name':f'{name}_mat{i}','rgba':text(part['rgba']),'specular':'.18','shininess':'.15'}
                if 'texture' in part:
                    add(assets,'texture',name=f'{name}_tex{i}',type='2d',file=part['texture'])
                    mat['texture']=f'{name}_tex{i}'
                add(assets,'material',**mat)
            for i,file in enumerate(self.assets[name]['collision']):
                add(assets,'mesh',name=f'{name}_c{i}',file=file)
        world=add(root,'worldbody')
        add(world,'geom',name='floor',type='plane',pos='0 0 -.01',size='0 0 .01',rgba='.77 .75 .66 1')
        add(world,'geom',name='table',type='box',pos='0 0 .045',size='1.02 1.02 .055',rgba='.76 .77 .73 1')
        for i,(x,y,yaw) in enumerate([(0,-.78,0),(.78,0,90),(0,.78,180),(-.78,0,-90)]):
            frame=add(world,'frame',pos=f'{x} {y} .1',euler=f'0 0 {yaw}')
            add(frame,'attach',model='panda_model',body='link0',prefix=f'r{i}_')
        self.manifest['scenes'][key]={'arms':4,'objects':[],'homeJoints':HOME,'tableHeight':.1}
        return root,world

    def object(self,world,scene,name,asset,pos,mass,yaw=0):
        body=add(world,'body',name=name,pos=text(pos),euler=f'0 0 {yaw}')
        add(body,'freejoint',name=name+'_free')
        b=np.array(self.manifest['assets'][asset]['bounds']);dx,dy,dz=b[1]-b[0]
        inertia=mass*np.array([dy*dy+dz*dz,dx*dx+dz*dz,dx*dx+dy*dy])/12
        add(body,'inertial',pos=text(b.mean(0)),mass=mass,diaginertia=text(inertia))
        for i,part in enumerate(self.assets[asset]['visual']):
            add(body,'geom',type='mesh',mesh=f'{asset}_v{i}',material=f'{asset}_mat{i}',contype='0',conaffinity='0',group='2',mass='0')
        for i,file in enumerate(self.assets[asset]['collision']):
            add(body,'geom',name=f'{name}_contact_{i}',type='mesh',mesh=f'{asset}_c{i}',group='3',friction='.85 .015 .001',solref='.005 1',solimp='.99 .999 .0001')
        self.manifest['scenes'][scene]['objects'].append({'name':name,'asset':asset,'mass':mass,'position':pos,'yaw':yaw})
        return body

    @staticmethod
    def pad(world,name,x,y,sx,sy,height=.014,color='.35 .38 .32 1'):
        body=add(world,'body',name=name,pos=f'{x} {y} {.1+height/2}')
        add(body,'geom',name=name+'_surface',type='box',size=f'{sx} {sy} {height/2}',rgba=color,friction='.8 .01 .001')
        return body

    def scan(self):
        root,w=self.scene('scan',['scanner','tea','coffee','tray'])
        self.pad(w,'incoming',-.19,-.34,.22,.09)
        self.pad(w,'scanner_dock',.32,-.16,.09,.115,height=.018)
        self.pad(w,'handoff',0,.05,.09,.08,height=.02,color='.5 .49 .42 1')
        self.pad(w,'packing',-.28,.25,.175,.125)
        self.pad(w,'dispatch',-.4,-.16,.175,.125)
        self.pad(w,'dispatch_lane',-.34,.045,.235,.08)
        self.object(w,'scan','tea_box','tea',[-.28,-.34,.116],.09)
        self.object(w,'scan','coffee_box','coffee',[-.10,-.34,.116],.10)
        self.object(w,'scan','scanner','scanner',[.32,-.16,.120],.13,yaw=-90)
        self.object(w,'scan','order_tray','tray',[-.28,.25,.116],.12)
        save(root,self.out/'scan.xml')

    def pot(self):
        root,w=self.scene('pot',['pot','carrot','tomato','tray'])
        self.pad(w,'pot_pad',0,0,.14,.14,height=.016,color='.35 .38 .32 1')
        self.pad(w,'prep_east',.36,.02,.13,.18,height=.012)
        self.pad(w,'prep_west',-.36,.02,.13,.18,height=.012)
        self.object(w,'pot','cooking_pot','pot',[0,0,.118],.55,yaw=90)
        self.object(w,'pot','east_tray','tray',[.36,.02,.114],.12,yaw=90)
        self.object(w,'pot','west_tray','tray',[-.36,.02,.114],.12,yaw=90)
        carrot=self.object(w,'pot','carrot','carrot',[.37483,.018,.12771],.055)
        # Source carrot is asymmetric. Start on its measured stable side,
        # rather than allowing it to roll to a side while the user inspects it.
        carrot.attrib.pop('euler')
        carrot.set('quat','.7019187 .0504383 -.7063373 -.0765095')
        self.manifest['scenes']['pot']['objects'][-1]['quaternion']=[.7019187,.0504383,-.7063373,-.0765095]
        self.object(w,'pot','tomato','tomato',[-.36,.02,.125],.07)
        save(root,self.out/'pot.xml')

    def finish(self):
        (self.out/'manifest.json').write_text(json.dumps(self.manifest,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--menagerie',type=Path,required=True)
    parser.add_argument('--robotwin',type=Path,required=True)
    parser.add_argument('--robocasa-objects',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path('public/assets/franka-cooperative'))
    args=parser.parse_args()
    package=Package(args.output)
    package.robot(args.menagerie/'franka_emika_panda')
    for key,folder,scale in [('scanner','024_scanner',.08),('tea','112_tea-box',.05),('coffee','113_coffee-box',.05),('tray','008_tray',.16)]:
        package.robotwin(key,args.robotwin/'assets/objects',folder,0,scale)
    for key,folder,factor in [('pot','lightwheel/pot/Pot054',.65),('carrot','aigen_objs/carrot/carrot_0',1),('tomato','aigen_objs/tomato/tomato_0',1)]:
        package.robocasa(key,args.robocasa_objects,folder,factor)
    shutil.copy2(args.robotwin/'LICENSE',args.output/'licenses/RoboTwin.txt')
    shutil.copy2(args.robotwin/'assets/objects/README.md',args.output/'licenses/RoboTwin-asset-card.md')
    package.scan()
    package.pot()
    package.finish()
    print(json.dumps(package.manifest['assets'],indent=2))


if __name__=='__main__':main()
