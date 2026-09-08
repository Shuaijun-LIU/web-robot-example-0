#!/usr/bin/env python3
"""Build the isolated static Demo2 from existing Panda/UMI/RoboDojo assets.

CPU-only conversion. Geometry is imported, not regenerated. The simple table
and outer box are scene fixtures; shaped egg inserts are sourced meshes.
"""
import argparse
import copy
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import coacd
import numpy as np
import trimesh
import mujoco
from pxr import Usd, UsdGeom


def extract(source, prim_paths, scale):
    stage = Usd.Stage.Open(str(source))
    cache = UsdGeom.XformCache()
    chunks = []
    for path in prim_paths:
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
        transform = cache.GetLocalToWorldTransform(mesh.GetPrim())
        points = np.array([transform.Transform(p) for p in mesh.GetPointsAttr().Get()])
        points *= UsdGeom.GetStageMetersPerUnit(stage)
        counts = list(mesh.GetFaceVertexCountsAttr().Get())
        indices = list(mesh.GetFaceVertexIndicesAttr().Get())
        faces, offset = [], 0
        for count in counts:
            faces.extend([[indices[offset], indices[offset + j], indices[offset + j + 1]] for j in range(1, count - 1)])
            offset += count
        chunks.append(trimesh.Trimesh(points * scale, faces, process=True))
    return trimesh.util.concatenate(chunks)


def save_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode')


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def geom(parent, name, kind, pos, size, rgba, **attrs):
    return add(parent, 'geom', name=name, type=kind, pos=pos, size=size, rgba=rgba, **attrs)


def output_cells(mesh, convex=False):
    """Reuse the four sourced pockets at egg-fit diameter with hand clearance.

    Cut at the source divider lines; preserve each pocket's surface, shrink
    around its measured hole center, then separate pockets to 84 mm pitch.
    Each collision input is convex; clipping and affine scaling preserve that.
    """
    pieces=[]
    for sx,sy,cx,cy in [(-1,-1,-.03784,-.03869),(1,-1,.03785,-.03869),
                        (-1,1,-.03784,.03819),(1,1,.03773,.03831)]:
        part=mesh.slice_plane([0,0,0],[sx,0,0])
        if not len(part.faces):continue
        part=part.slice_plane([0,0,0],[0,sy,0])
        if len(part.vertices)<4:continue
        part.vertices=(part.vertices-[cx,cy,0])*[.35,.35,.6]+[sx*.042,sy*.042,0]
        if convex:
            if np.min(part.extents)<1e-8:continue
            part=part.convex_hull
        pieces.append(part)
    return pieces


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--menagerie', type=Path, required=True)
    p.add_argument('--robodojo', type=Path, required=True)
    p.add_argument('--output', type=Path, default=Path('public/assets/franka-egg-sorting'))
    args = p.parse_args()
    out = args.output
    (out / 'assets').mkdir(parents=True, exist_ok=True)
    (out / 'licenses').mkdir(exist_ok=True)
    panda_source = args.menagerie / 'franka_emika_panda'
    umi_source = args.menagerie / 'umi_gripper'
    shutil.copytree(panda_source / 'assets', out / 'assets', dirs_exist_ok=True)
    for file in (umi_source / 'assets').iterdir():
        shutil.copy2(file, out / 'assets' / ('umi_' + file.name))
    for name, path in [('Panda', panda_source / 'LICENSE'), ('UMI', umi_source / 'LICENSE'), ('RoboDojo', args.robodojo / 'LICENSE')]:
        shutil.copy2(path, out / 'licenses' / (name + '.txt'))

    egg_source = args.robodojo / 'Assets/Object/RoboDojo/Rigid/egg/00000/object.usdz'
    tray_source = args.robodojo / 'Assets/Object/RoboDojo/Articulation/egg_holder/00000/object.usdz'
    egg = extract(egg_source, ['/World/visual'], np.ones(3))
    egg.vertices -= egg.bounds.mean(0)
    egg.export(out / 'assets' / 'egg.obj')
    # Reuse the open four-cell bottom, not the lid. Widen cavities for finger
    # access and reduce wall height so the egg equator remains accessible.
    tray = extract(tray_source, ['/root/E_body_5/P_966ad814d7ae68ac', '/root/E_body_5/E_part_01_39/P_6ae187f8ffadf82'], np.array([1.4, 1.4, .65]))
    tray.vertices[:, 2] -= tray.bounds[0, 2]
    tray.export(out / 'assets' / 'egg-insert.obj')
    collision_manifest = out / 'insert-collisions.json'
    settings = {'threshold': .025, 'preprocess_mode': 'auto', 'preprocess_resolution': 50,
                'mcts_iterations': 30, 'mcts_max_depth': 3, 'seed': 0}
    cache_key = hashlib.sha256(tray.vertices.tobytes() + tray.faces.tobytes() + json.dumps(settings, sort_keys=True).encode()).hexdigest()
    cache = json.loads(collision_manifest.read_text()) if collision_manifest.exists() else {}
    if isinstance(cache, dict) and cache.get('key') == cache_key and all((out / 'assets' / f).exists() for f in cache.get('files', [])):
        collision_files = cache['files']
    else:
        coacd.set_log_level('warn')
        print('Decomposing the sourced hollow insert (CPU)...', flush=True)
        parts = coacd.run_coacd(coacd.Mesh(tray.vertices, tray.faces), **settings)
        collision_files = []
        for i, (v, f) in enumerate(parts):
            filename = f'insert-collision-{i}.obj'
            trimesh.Trimesh(v, f, process=False).export(out / 'assets' / filename)
            collision_files.append(filename)
        collision_manifest.write_text(json.dumps({'key': cache_key, 'settings': settings, 'files': collision_files}, indent=2))

    # Source-derived modular output pockets: tighter holes but ample clearance
    # between neighboring eggs for the long, physically sized UMI fingers.
    trimesh.util.concatenate(output_cells(tray)).export(out/'assets'/'output-insert.obj')
    output_collision_files=[]
    for filename in collision_files:
        for part in output_cells(trimesh.load(out/'assets'/filename),convex=True):
            name=f'output-collision-{len(output_collision_files)}.obj'
            part.export(out/'assets'/name)
            output_collision_files.append(name)

    # Adapt the complete sourced UMI body, retaining meshes, holders and GoPro.
    # This is a simulation mount, not a qualified physical flange adapter.
    panda = ET.parse(panda_source / 'panda.xml').getroot()
    panda.find('compiler').set('texturedir', 'assets')
    panda.remove(panda.find('keyframe'))
    umi = ET.parse(umi_source / 'umi_gripper.xml').getroot()
    for asset in umi.find('asset'):
        item = copy.deepcopy(asset)
        old_name = item.get('name', Path(item.get('file', '')).stem)
        item.set('name', 'umi_' + old_name)
        if 'file' in item.attrib:
            item.set('file', 'umi_' + item.get('file'))
        if 'texture' in item.attrib:
            item.set('texture', 'umi_' + item.get('texture'))
        panda.find('asset').append(item)
    hand_parent = panda.find(".//body[@name='link7']")
    hand_parent.remove(hand_parent.find("body[@name='hand']"))
    hand = add(hand_parent, 'body', name='hand', pos='0 0 0.134115', quat='0.9238795 0 0 -0.3826834')
    add(hand, 'inertial', mass='.48', pos='0 0 .03', diaginertia='.0015 .0015 .002')
    add(hand, 'site', name='tcp', pos='0 .003 .17', size='.006', rgba='.4 .3 .18 .5', group='1')
    root = umi.find('worldbody/body')
    for element in root:
        if element.tag == 'joint':
            continue
        item = copy.deepcopy(element)
        for node in item.iter():
            cls = node.attrib.pop('class', None)
            if node.tag == 'geom':
                for attr in ['mesh', 'material']:
                    if attr in node.attrib:
                        node.set(attr, 'umi_' + node.get(attr))
                if 'mesh' in node.attrib:
                    node.set('type', 'mesh')
                if cls in ['visual', 'screw', 'marker']:
                    node.set('contype', '0'); node.set('conaffinity', '0'); node.set('mass', '0')
                    node.set('group', '2')
                if cls == 'collision':
                    node.set('group', '3'); node.set('friction', '.9 .01 .001')
                if cls == 'screw':
                    node.set('type', 'cylinder'); node.set('size', '.0035 .0035'); node.set('material', 'umi_black')
                if cls == 'marker':
                    node.set('type', 'box'); node.set('size', '.009 .009 .00001')
            if node.tag == 'joint':
                left = node.get('name') == 'left_finger_joint'
                node.set('name', 'finger_joint1' if left else 'finger_joint2')
                node.set('type', 'slide'); node.set('axis', '-1 0 0' if left else '1 0 0')
                node.set('range', '0 .05'); node.set('damping', '1')
        if item.tag == 'body':
            left = item.get('name') == 'left_finger_holder'
            item.set('name', 'left_finger' if left else 'right_finger')
            item.set('pos', '.041722 0 0' if left else '-.041722 0 0')
        hand.append(item)
    actuator = panda.find("actuator/general[@name='actuator8']")
    actuator.set('name', 'gripper'); actuator.set('forcerange', '-12 12')
    actuator.set('gainprm', '.019607843137 0 0')
    actuator.set('biasprm', '0 -100 -2')
    save_xml(panda, out / 'panda.xml')

    scene = ET.Element('mujoco', model='Franka Demo2 - mixed egg sorting static workcell')
    add(scene, 'compiler', angle='degree', meshdir='assets', texturedir='assets')
    option=add(scene, 'option', timestep='.002', integrator='implicitfast', cone='elliptic', iterations='50', noslip_iterations='10')
    add(option,'flag',multiccd='enable')
    visual = add(scene, 'visual')
    add(visual, 'headlight', diffuse='.75 .75 .75', ambient='.4 .4 .4', specular='.1 .1 .1')
    assets = add(scene, 'asset')
    add(assets, 'model', name='panda_model', file='panda.xml')
    add(assets, 'mesh', name='insert', file='output-insert.obj')
    add(assets, 'mesh', name='source_insert_mesh', file='egg-insert.obj', scale='1 1 .6')
    classes = [('Ivory', '0.91 0.86 0.73 1', 1., .055), ('Brown', '.53 .29 .13 1', 1.03, .06),
               ('Pale green', '.60 .73 .59 1', .96, .05), ('Cream', '.76 .62 .39 1', .91, .045)]
    for i, (_, color, scale, mass) in enumerate(classes):
        add(assets, 'mesh', name=f'egg_class_{i}', file='egg.obj', scale=f'{scale} {scale} {scale}')
        add(assets, 'material', name=f'shell_{i}', rgba=color, specular='.12', shininess='.08')
    for i, filename in enumerate(collision_files):
        add(assets, 'mesh', name=f'source_insert_c_{i}', file=filename, scale='1 1 .6')
    for i, filename in enumerate(output_collision_files):
        add(assets, 'mesh', name=f'insert_c_{i}', file=filename)
    world = add(scene, 'worldbody')
    geom(world, 'floor', 'plane', '0 0 -.01', '0 0 .01', '.77 .75 .66 1')
    geom(world, 'table', 'box', '0 0 .045', '1.02 1.02 .055', '.76 .77 .73 1')
    for i, (x, y, angle) in enumerate([(0, -.78, 0), (.78, 0, 90), (0, .78, 180), (-.78, 0, -90)]):
        frame = add(world, 'frame', pos=f'{x} {y} .1', euler=f'0 0 {angle}')
        add(frame, 'attach', model='panda_model', body='link0', prefix=f'r{i}_')
    box = add(world, 'body', name='source_box')
    geom(box, 'box_floor', 'box', '0 0 .106', '.188 .188 .006', '.36 .38 .33 1')
    for name, pos, size in [('north', '0 .19 .128', '.198 .006 .028'), ('south', '0 -.19 .128', '.198 .006 .028'),
                            ('east', '.192 0 .128', '.006 .184 .028'), ('west', '-.192 0 .128', '.006 .184 .028')]:
        geom(box, 'box_' + name, 'box', pos, size, '.43 .45 .38 1')

    def insert(name, x, y, z, color):
        body = add(world, 'body', name=name, pos=f'{x} {y} {z}')
        source=name.startswith('source_')
        add(body, 'geom', type='mesh', mesh='source_insert_mesh' if source else 'insert', rgba=color, contype='0', conaffinity='0', mass='0', group='2')
        for i in range(len(collision_files if source else output_collision_files)):
            add(body, 'geom', name=f'{name}_contact_{i}', type='mesh', mesh=f'source_insert_c_{i}' if source else f'insert_c_{i}', group='3',
                friction='.6 .01 .001', solref='.005 1', solimp='.99 .999 .0001')
        return body

    eggs = []
    for quadrant, (bx, by) in enumerate([(-.091, -.091), (.091, -.091), (.091, .091), (-.091, .091)]):
        insert(f'source_insert_{quadrant}', bx, by, .112, '.60 .55 .44 1')
        for cell, (cx, cy) in enumerate([(-.042, -.042), (.042, -.042), (.042, .042), (-.042, .042)]):
            i = quadrant * 4 + cell
            category = (cell + quadrant) % 4
            _, color, scale, mass = classes[category]
            position = [bx + cx, by + cy, .174]
            body = add(world, 'body', name=f'egg_{i}', pos=' '.join(map(str, position)))
            add(body, 'freejoint', name=f'egg_{i}_free')
            add(body, 'geom', name=f'egg_{i}_shell', type='mesh', mesh=f'egg_class_{category}', material=f'shell_{category}',
                mass=mass, friction='.6 .01 .001', solref='.005 1', solimp='.99 .999 .0001', condim='4')
            eggs.append({'name': f'egg_{i}', 'classIndex': category, 'position': position, 'scale': scale, 'massKg': mass})
    trays = []
    for i, (x, y) in enumerate([(-.40, -.40), (.40, -.40), (.40, .40), (-.40, .40)]):
        insert(f'output_tray_{i}', x, y, .106, '.64 .66 .59 1')
        # Small class-colored mat in front of each tray, no floating tool labels.
        geom(world, f'tray_mat_{i}', 'box', f'{x} {y} .103', '.092 .092 .003', classes[i][1])
        trays.append({'name': f'output_tray_{i}', 'owner': f'Arm {i + 1}', 'classIndex': i, 'position': [x, y, .106], 'capacity': 4})
    home = [1.570796, -.785398, 0, -2.356194, 0, 1.570796, .785398]
    qpos = sum((home + [.05, .05] for _ in range(4)), []) + sum((e['position'] + [1, 0, 0, 0] for e in eggs), [])
    ctrl = (home + [255]) * 4
    keys = add(scene, 'keyframe')
    add(keys, 'key', name='home', qpos=' '.join(map(str, qpos)), ctrl=' '.join(map(str, ctrl)))
    save_xml(scene, out / 'scene.xml')
    # Generate the initial conditions from a passive gravity/contact settle.
    # This runs only at asset-build time, never as an object-following controller.
    model = mujoco.MjModel.from_xml_path(str((out / 'scene.xml').resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(100000):
        mujoco.mj_step(model, data)
    initial_egg_positions = []
    for e in eggs:
        body = world.find(f"body[@name='{e['name']}']")
        pose = data.body(e['name'])
        e['position'] = pose.xpos.tolist()
        e['quaternion'] = pose.xquat.tolist()
        body.set('pos', ' '.join(f'{v:.10f}' for v in e['position']))
        body.set('quat', ' '.join(f'{v:.10f}' for v in e['quaternion']))
        initial_egg_positions.append(pose.xpos.copy())
    qpos = sum((home + [.05, .05] for _ in range(4)), []) + sum((e['position'] + e['quaternion'] for e in eggs), [])
    keys.find('key').set('qpos', ' '.join(map(str, qpos)))
    save_xml(scene, out / 'scene.xml')
    # Re-load exported initial conditions with zero velocity, like browser Reset.
    model = mujoco.MjModel.from_xml_path(str((out / 'scene.xml').resolve()))
    data = mujoco.MjData(model)
    for arm in range(4):
        data.ctrl[arm * 8:arm * 8 + 8] = ctrl[arm * 8:arm * 8 + 8]
        for j in range(7):
            data.joint(f'r{arm}_joint{j + 1}').qpos[0] = home[j]
    for _ in range(2500):
        mujoco.mj_step(model, data)
    drift = [float(np.linalg.norm(data.body(e['name']).xpos - p)) for e, p in zip(eggs, initial_egg_positions)]
    metadata = {
        'stage': 'scene geometry; motion acceptance is recorded separately in first-egg-motion.json and artifacts/reports',
        'eggSource': 'RoboDojo/Rigid/egg/00000', 'eggSourceSha256': hashlib.sha256(egg_source.read_bytes()).hexdigest(),
        'insertSource': 'RoboDojo/Articulation/egg_holder/00000 bottom only',
        'insertSourceSha256': hashlib.sha256(tray_source.read_bytes()).hexdigest(),
        'insertScale': [1.4, 1.4, .65], 'insertCollisionPieces': len(collision_files),
        'sourceInsertAdditionalZScale': .6,
        'outputPocketScale': [.35, .35, .6], 'outputPocketPitchMeters': .084,
        'outputCollisionPieces': len(output_collision_files),
        'eggBoundsMeters': egg.bounds.tolist(), 'insertBoundsMeters': tray.bounds.tolist(),
        'classes': [{'name': name, 'rgba': color, 'scale': scale, 'massKg': mass} for name, color, scale, mass in classes],
        'classNote': 'Four appearance classes adapted from one sourced shell, not four independently verified species.',
        'gripper': 'Menagerie UMI meshes and holder assembly; rigid simulation flange mount; not a validated real-hardware Panda adapter or deformable simulation.',
        'gripperForceLimitN': 12, 'homeJoints': ctrl, 'eggs': eggs, 'trays': trays,
        'initialization': '200 seconds of offline passive contact settling; poses used only as scene/Reset initial conditions',
        'resetFollowup5sMaxDriftMeters': max(drift),
    }
    (out / 'manifest.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps({'eggs': len(eggs), 'trayCollisionParts': len(collision_files), 'eggExtents': egg.extents.tolist()}), flush=True)


if __name__ == '__main__':
    main()
