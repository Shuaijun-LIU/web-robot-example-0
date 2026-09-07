// Prepare the official CAD for web display. Indexed geometry and bounded-error
// simplification retain source shape; no camera housing is constructed here.
import {readFile,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';
const browser=await chromium.launch({headless:true});
try{
  const page=await browser.newPage();
  await page.route('**/__camera-convert',route=>route.fulfill({contentType:'text/html',body:'<html></html>'}));
  await page.goto('http://127.0.0.1:3000/__camera-convert');
  const result=await page.evaluate(async text=>{
    const THREE=await import('/node_modules/three/build/three.module.js');
    const {ColladaLoader}=await import('/node_modules/three/examples/jsm/loaders/ColladaLoader.js');
    const {GLTFExporter}=await import('/node_modules/three/examples/jsm/exporters/GLTFExporter.js');
    const {mergeVertices}=await import('/node_modules/three/examples/jsm/utils/BufferGeometryUtils.js');
    const {MeshoptSimplifier}=await import('/node_modules/meshoptimizer/meshopt_simplifier.module.js');
    await MeshoptSimplifier.ready;
    const source=new ColladaLoader().parse(text,'').scene;
    // ColladaLoader auto-converts Z-up. Keep the source CAD coordinates instead.
    source.quaternion.identity();source.updateMatrixWorld(true);
    const bounds=new THREE.Box3().setFromObject(source);
    const materials=[];
    source.traverse(mesh=>{
      if(!mesh.isMesh)return;
      const geometry=mergeVertices(mesh.geometry,1e-7);
      if(geometry.index.count>3000){
        const [indices,error]=MeshoptSimplifier.simplify(new Uint32Array(geometry.index.array),geometry.attributes.position.array,3,Math.max(3000,Math.floor(geometry.index.count*.12/3)*3),.0005);
        geometry.setIndex(new THREE.BufferAttribute(indices,1));
        materials.push({sourceTriangles:mesh.geometry.attributes.position.count/3,triangles:indices.length/3,relativeError:error});
      }
      const compact=mergeVertices(geometry.toNonIndexed(),1e-7);
      const positions=compact.attributes.position,colors=new Float32Array(positions.count*3);
      const steel=new THREE.Color('#c3c8c7'),glass=new THREE.Color('#171e20'),lens=new THREE.Color('#172b35');
      for(let i=0;i<positions.count;i++){
        const x=positions.getX(i),y=positions.getY(i),z=positions.getZ(i);
        const face=z>-.007&&Math.abs(x)<.0425&&Math.abs(y)<.0098;
        const pupil=[-.03275,.01725,.0325].some(cx=>Math.hypot(x-cx,y)<.0032);
        (face?(pupil?lens:glass):steel).toArray(colors,i*3);
      }
      compact.setAttribute('color',new THREE.BufferAttribute(colors,3));mesh.geometry=compact;
      const convert=m=>{
        return new THREE.MeshStandardMaterial({vertexColors:true,metalness:.4,roughness:.32});
      };
      mesh.material=Array.isArray(mesh.material)?mesh.material.map(convert):convert(mesh.material);
    });
    const binary=await new GLTFExporter().parseAsync(source,{binary:true});
    return {bytes:Array.from(new Uint8Array(binary)),bounds:{min:bounds.min.toArray(),max:bounds.max.toArray()},materials};
  },await readFile('public/assets/assembly-cameras/d435.dae','utf8'));
  await writeFile('public/assets/assembly-cameras/d435.glb',Buffer.from(result.bytes));
  console.log(JSON.stringify({bounds:result.bounds,materials:result.materials,bytes:result.bytes.length},null,2));
}finally{await browser.close();}
