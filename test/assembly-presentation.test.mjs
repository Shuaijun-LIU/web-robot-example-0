import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import * as presentation from '../src/assemblyPresentation.js';

test('presentation clones shared materials without changing physical geometry or unrelated meshes',()=>{
  const source=new THREE.MeshStandardMaterial({color:'#777777'});
  const geometry=new THREE.BoxGeometry(.1,.1,.1);
  const beam=new THREE.Mesh(geometry,source),other=new THREE.Mesh(geometry,source);
  beam.position.set(1,2,3);
  const restore=presentation.styleAssemblyMesh(beam,'cross_member_flange_left_center','cross_member');
  assert.notEqual(beam.material,source);
  assert.equal(other.material,source);
  assert.equal(beam.geometry,geometry);
  assert.deepEqual(beam.position.toArray(),[1,2,3]);
  assert.ok(beam.material.metalness>.5);
  assert.equal(beam.material.transparent,false);
  restore();
  assert.equal(beam.material,source);
});

test('hidden collision meshes remain hidden and rubber differs from metal',()=>{
  const hidden=new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshStandardMaterial({opacity:0,transparent:true}));
  const original=hidden.material;
  presentation.styleAssemblyMesh(hidden,'robotwin_hammer_handle_0_collision','double_face_hammer');
  assert.equal(hidden.material,original);
  const rubber=new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshStandardMaterial());
  const metal=rubber.clone();
  presentation.styleAssemblyMesh(rubber,'robotwin_hammer_dark_visual_geom','double_face_hammer');
  presentation.styleAssemblyMesh(metal,'robotwin_hammer_metal_visual_geom','double_face_hammer');
  assert.ok(rubber.material.roughness>metal.material.roughness);
  assert.ok(rubber.material.metalness<metal.material.metalness);
});

test('workstation does not add textual tool or station labels',()=>{
  const specs=presentation.assemblyStationDecals();
  assert.equal(specs.filter(s=>s.text).length,0);
  for(const spec of specs){
    assert.ok(spec.position.every(Number.isFinite));
    assert.equal(spec.position.length,3);
    assert.ok(spec.size.every(v=>v>0));
    assert.ok(spec.position[2]>=.1&&spec.position[2]<.121);
    assert.equal(spec.text.match(/[^\x00-\x7F]/),null);
  }
});

test('workstation retains source model colors while preserving metal and matte finishes',()=>{
  const create=(geom,body)=>{
    const mesh=new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshStandardMaterial({color:'#444444'}));
    presentation.styleAssemblyMesh(mesh,geom,body);return mesh.material;
  };
  const beam=create('cross_member_north_round_opening_segment_01','cross_member');
  const pin=create('pin_head','fastener_0');
  const support=create('cradle_top','hammer_pickup_cradle');
  const tray=create('tray_floor','fastener_tray');
  const plate=create('mounting_plate_body','mounting_plate');
  for(const material of [beam,pin,support,tray,plate])assert.equal(material.color.getHexString(),'444444');
  assert.equal(beam.metalness,.56);
  assert.equal(beam.roughness,.32);
  assert.equal(pin.metalness,.65);
  assert.equal(pin.roughness,.25);
  assert.ok(support.metalness<.1&&tray.metalness<.1);
  assert.ok(support.roughness>.7&&tray.roughness>.7);
});

test('source palette keeps inner/outer rail contrast and distinct mats instead of repainting them',()=>{
  // Linear RGB values from the model before the September 6 presentation layer.
  for(const [geom,body,rgb] of [
    ['frame_rail_north_outer','assembly_frame',[.55,.57,.58]],
    ['frame_rail_north_inner','assembly_frame',[.68,.69,.69]],
    ['cross_member_slot','cross_member',[.08,.09,.1]],
    ['frame_grip_west','assembly_frame',[.15,.17,.18]],
    ['parts_tray_floor','parts_tray',[.24,.3,.34]],
    ['mat','tool_mat_manual',[.31,.27,.21]],
    ['pad','handover_pad',[.24,.31,.36]],
    ['inset','platform_inset',[.33,.35,.36]],
    ['platform','assembly_platform',[.25,.27,.29]],
  ]){
    const mesh=new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshStandardMaterial({color:new THREE.Color(...rgb)}));
    const restore=presentation.styleAssemblyMesh(mesh,geom,body);
    assert.deepEqual(mesh.material.color.toArray(),rgb,geom);
    restore();
  }
});

test('drill shares hammer yellow housing and retains black grip and steel chuck',()=>{
  const geometry=new THREE.BufferGeometry();
  // Source CAD uses X along battery → motor, Z along the motor/chuck axis.
  geometry.setAttribute('position',new THREE.Float32BufferAttribute([.08,0,-.03,0,.02,-.044,.085,0,.06],3));
  const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial());
  const restore=presentation.cleanToolSurface(mesh,'torque_driver');
  const colors=mesh.geometry.attributes.color;
  const yellow=new THREE.Color('#aa8746');
  assert.ok(Math.abs(colors.getX(0)-yellow.r)<1e-6);
  assert.ok(colors.getX(1)<colors.getX(0)*.2);
  assert.ok(colors.getX(2)>colors.getX(0));
  restore();
});

test('refined connector keeps both bores open and the existing top seating height',()=>{
  assert.equal(typeof presentation.createConnectorSurface,'function');
  const mesh=presentation.createConnectorSurface();mesh.updateMatrixWorld(true);
  assert.deepEqual(mesh.material.color.toArray(),[.24,.27,.29]);
  assert.equal(mesh.material.metalness,.56);
  const ray=new THREE.Raycaster();
  for(const x of [-.038,.04]){
    ray.set(new THREE.Vector3(x,0,.10),new THREE.Vector3(0,0,-1));
    assert.equal(ray.intersectObject(mesh).length,0);
  }
  ray.set(new THREE.Vector3(0,0,.1),new THREE.Vector3(0,0,-1));
  assert.ok(Math.abs(ray.intersectObject(mesh)[0].point.z-.048)<1e-6);
});

test('clean tool coloring is continuous across source material partitions and preserves every vertex',()=>{
  const geometry=new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.Float32BufferAttribute([-.06,.015,0,.085,.015,0,-.12,.015,0],3));
  const a=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial()),b=a.clone();
  const restoreA=presentation.cleanToolSurface(a,'double_face_hammer');
  const restoreB=presentation.cleanToolSurface(b,'double_face_hammer');
  assert.deepEqual(Array.from(a.geometry.attributes.position.array),Array.from(geometry.attributes.position.array));
  assert.deepEqual(Array.from(a.geometry.attributes.color.array),Array.from(b.geometry.attributes.color.array));
  assert.notDeepEqual(Array.from(a.geometry.attributes.color.array.slice(0,3)),Array.from(a.geometry.attributes.color.array.slice(3,6)));
  restoreA();restoreB();assert.equal(a.geometry,geometry);assert.equal(b.geometry,geometry);
});
