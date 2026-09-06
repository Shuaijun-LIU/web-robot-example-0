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

test('station markings are surface decals, not extra support solids or network assets',()=>{
  const specs=presentation.assemblyStationDecals();
  assert.ok(specs.some(s=>s.text==='HAMMER / PICKUP'));
  assert.ok(specs.some(s=>s.text==='HAMMER / RETURN'));
  assert.ok(specs.some(s=>s.text==='LOCATING PINS'));
  for(const spec of specs){
    assert.ok(spec.position.every(Number.isFinite));
    assert.equal(spec.position.length,3);
    assert.ok(spec.size.every(v=>v>0));
    assert.ok(spec.position[2]>=.1&&spec.position[2]<.121);
    assert.equal(spec.text.match(/[^\x00-\x7F]/),null);
  }
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
