import * as THREE from 'three';

// Scoped adapter: upstream GeomBuilder renders RGB but drops texture maps and
// UV seams. Keep its exact mesh coordinates/body transforms; add only shading.
export function installCooperativeTextures(scene,m) {
  const restores=[],textures=new Map();let count=0;
  scene.traverse(mesh=>{
    const g=mesh.userData.geomID;
    if(!mesh.isMesh||!Number.isInteger(g))return;
    const mat=m.geom_matid[g],id=m.geom_dataid[g];
    if(mat<0||id<0)return;
    const stride=m.mat_texid.length/m.nmat,tex=m.mat_texid[mat*stride+1];
    if(tex<0||m.mesh_texcoordadr[id]<0)return;
    if(!textures.has(tex)){
      const width=m.tex_width[tex],height=m.tex_height[tex],channels=m.tex_nchannel[tex],start=m.tex_adr[tex];
      const pixels=new Uint8Array(width*height*4);
      for(let i=0;i<width*height;i++){
        for(let c=0;c<3;c++)pixels[i*4+c]=m.tex_data[start+i*channels+Math.min(c,channels-1)];
        pixels[i*4+3]=channels===4?m.tex_data[start+i*channels+3]:255;
      }
      const texture=new THREE.DataTexture(pixels,width,height,THREE.RGBAFormat);
      texture.colorSpace=THREE.SRGBColorSpace;texture.magFilter=THREE.LinearFilter;
      texture.minFilter=THREE.LinearMipmapLinearFilter;texture.generateMipmaps=true;texture.needsUpdate=true;
      textures.set(tex,texture);
    }
    const originalGeometry=mesh.geometry,originalMaterial=mesh.material;
    const geometry=originalGeometry.index?originalGeometry.toNonIndexed():originalGeometry.clone();
    const uv=new Float32Array(m.mesh_facenum[id]*6),face=m.mesh_faceadr[id]*3,adr=m.mesh_texcoordadr[id];
    for(let i=0;i<uv.length/2;i++){
      const t=adr+m.mesh_facetexcoord[face+i];
      uv[i*2]=m.mesh_texcoord[t*2];uv[i*2+1]=m.mesh_texcoord[t*2+1];
    }
    geometry.setAttribute('uv',new THREE.BufferAttribute(uv,2));
    const material=originalMaterial.clone();material.map=textures.get(tex);material.roughness=.65;material.metalness=.08;
    mesh.geometry=geometry;mesh.material=material;count++;
    restores.push(()=>{mesh.geometry=originalGeometry;mesh.material=originalMaterial;geometry.dispose();material.dispose();});
  });
  return {count,dispose(){for(const restore of restores)restore();for(const texture of textures.values())texture.dispose();}};
}
