import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {installCooperativeTextures} from '../src/cooperativeTextures.js';

test('source texture and per-face UV seams reach the rendered mesh and clean up',()=>{
  const m={nmat:1,geom_matid:[0],geom_dataid:[0],mat_texid:[-1,0,-1,-1,-1,-1,-1,-1,-1,-1],
    mesh_faceadr:[0],mesh_facenum:[2],mesh_facetexcoord:[0,1,2,3,2,1],mesh_texcoordadr:[0],
    mesh_texcoord:[0,0,1,0,0,1,.2,.3],tex_width:[1],tex_height:[1],tex_nchannel:[3],tex_adr:[0],tex_data:new Uint8Array([120,40,20])};
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,1,0,0,0,1,0,1,1,0],3));geometry.setIndex([0,1,2,3,2,1]);
  const material=new THREE.MeshStandardMaterial(),mesh=new THREE.Mesh(geometry,material),scene=new THREE.Scene();
  mesh.userData.geomID=0;scene.add(mesh);
  const installed=installCooperativeTextures(scene,m);
  assert.equal(installed.count,1);
  assert.deepEqual(Array.from(mesh.material.map.image.data),[120,40,20,255]);
  assert.equal(mesh.geometry.getAttribute('uv').count,6);
  assert.ok(Math.abs(mesh.geometry.getAttribute('uv').getX(3)-.2)<1e-6);
  assert.deepEqual(Array.from(mesh.geometry.getAttribute('position').array).slice(9,12),[1,1,0]);
  installed.dispose();assert.equal(mesh.geometry,geometry);assert.equal(mesh.material,material);
});
