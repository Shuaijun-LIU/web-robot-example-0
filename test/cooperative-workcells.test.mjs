import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import {resolve,dirname} from 'node:path';
import {COOPERATIVE_WORKCELLS} from '../src/cooperativeWorkcells.js';
import {createFrankaTargets} from '../src/controlTargets.js';

const root=resolve('public/assets/franka-cooperative');
for(const key of ['scan','pot']) test(`${key} page resolves a complete four-arm physical asset package`,()=>{
  const scene=COOPERATIVE_WORKCELLS[key],seen=new Set();
  assert.equal(scene.homeJoints.length,32);
  assert.equal(scene.roles.length,4);
  assert.equal(createFrankaTargets().length,4);
  const queue=[scene.sceneFile];
  while(queue.length){
    const file=queue.pop();if(seen.has(file))continue;seen.add(file);
    const path=resolve(root,file);assert.ok(path.startsWith(root+'/'));
    assert.ok(existsSync(path),`missing browser dependency ${file}`);
    if(!file.endsWith('.xml'))continue;
    const xml=readFileSync(path,'utf8');
    const compiler=xml.match(/<compiler\b[^>]*>/)?.[0]??'';
    const meshdir=compiler.match(/meshdir="([^"]+)"/)?.[1]??'';
    const texturedir=compiler.match(/texturedir="([^"]+)"/)?.[1]??'';
    for(const m of xml.matchAll(/<(\w+)\b[^>]*\bfile="([^"]+)"/g)){
      const prefix=m[1]==='mesh'?meshdir:m[1]==='texture'?texturedir:'';
      queue.push(resolve(dirname(path),prefix,m[2]).slice(root.length+1));
    }
  }
  assert.ok(seen.size>50,'real robot/object geometry must be included');
  const manifest=JSON.parse(readFileSync(resolve(root,'manifest.json')));
  assert.equal(manifest.scenes[key].arms,4);
  for(const body of manifest.scenes[key].objects){
    assert.ok(body.mass>0&&body.mass<3,'task objects need realistic nonzero mass');
    assert.equal(body.position.length,3);
    assert.ok(body.position.every(Number.isFinite));
    assert.ok(manifest.assets[body.asset].sourceSha256.length===64);
  }
  for(const a of Object.values(manifest.assets)){
    assert.ok(a.bounds[1].every((v,i)=>v>a.bounds[0][i]));
    assert.ok(a.dimensions.every(v=>v>0&&v<.65),'millimetre/metre conversion must be applied');
  }
});

test('scan and pot never share a scene file or task state key',()=>{
  assert.notEqual(COOPERATIVE_WORKCELLS.scan.sceneFile,COOPERATIVE_WORKCELLS.pot.sceneFile);
  assert.notEqual(COOPERATIVE_WORKCELLS.scan.key,COOPERATIVE_WORKCELLS.pot.key);
});
