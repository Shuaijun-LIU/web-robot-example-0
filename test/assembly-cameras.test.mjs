import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {readFile} from 'node:fs/promises';
import {GLTFLoader} from 'three/examples/jsm/loaders/GLTFLoader.js';

test('five camera rig follows each wrist independently and leaves the global view fixed',async()=>{
  const module=await import('../src/assemblyCameras.js').catch(()=>({}));
  assert.equal(typeof module.createAssemblyCameraRig,'function');
  const rig=module.createAssemblyCameraRig();
  assert.equal(rig.views.length,5);
  const hands=Array.from({length:4},()=>new THREE.Group());
  for(let i=0;i<4;i++)hands[i].add(rig.wrists[i]);
  for(const hand of hands)hand.updateMatrixWorld(true);
  const before=rig.views.map(v=>v.camera.getWorldPosition(new THREE.Vector3()));
  hands[1].position.set(1,2,3);hands[1].rotation.z=Math.PI/2;hands[1].updateMatrixWorld(true);
  const after=rig.views.map(v=>v.camera.getWorldPosition(new THREE.Vector3()));
  assert.deepEqual(after[0].toArray(),before[0].toArray());
  assert.deepEqual(after[1].toArray(),before[1].toArray());
  assert.ok(after[2].distanceTo(before[2])>3);
  assert.deepEqual(after[3].toArray(),before[3].toArray());
  const expected=before[2].clone().applyAxisAngle(new THREE.Vector3(0,0,1),Math.PI/2).add(new THREE.Vector3(1,2,3));
  assert.ok(expected.distanceTo(after[2])<1e-8);
});

test('camera selection renders only the requested real sensor views',async()=>{
  const module=await import('../src/assemblyCameras.js').catch(()=>({}));
  assert.equal(typeof module.selectedCameraViews,'function');
  assert.deepEqual(module.selectedCameraViews('arm3'),['arm3']);
  assert.deepEqual(module.selectedCameraViews('all'),['global','arm1','arm2','arm3','arm4']);
  assert.deepEqual(module.selectedCameraViews('bad'),['global']);
});

test('sensor pass omits UI gizmos but never hides physical occluders and always restores visibility',async()=>{
  const module=await import('../src/assemblyCameras.js');
  assert.equal(typeof module.withCameraOverlaysHidden,'function');
  const scene=new THREE.Scene(),physical=new THREE.Mesh(),overlay=new THREE.Group();
  overlay.userData.sensorOverlay=true;scene.add(physical,overlay);
  assert.throws(()=>module.withCameraOverlaysHidden(scene,()=>{
    assert.equal(physical.visible,true);assert.equal(overlay.visible,false);throw new Error('render test');
  }),/render test/);
  assert.equal(overlay.visible,true);assert.equal(physical.visible,true);
});

test('downloaded D435 remains life-sized with a distinct optical face after web conversion',async()=>{
  const binary=await readFile(new URL('../public/assets/assembly-cameras/d435.glb',import.meta.url));
  const {scene}=await new GLTFLoader().parseAsync(binary.buffer.slice(binary.byteOffset,binary.byteOffset+binary.byteLength),'');
  const size=new THREE.Box3().setFromObject(scene).getSize(new THREE.Vector3());
  assert.ok(Math.abs(size.x-.09)<.001&&Math.abs(size.y-.025)<.001&&Math.abs(size.z-.02505)<.001);
  let dark=0,silver=0,triangles=0;
  scene.traverse(object=>{if(object.isMesh){
    const color=object.geometry.attributes.color;
    assert.ok(color);assert.equal(object.material.vertexColors,true);
    for(let i=0;i<color.count;i++){if(color.getX(i)<.04)dark++;if(color.getX(i)>.5)silver++;}
    triangles+=(object.geometry.index?.count??object.geometry.attributes.position.count)/3;
  }});
  assert.ok(dark>100&&silver>100);assert.ok(triangles<60000);
});
