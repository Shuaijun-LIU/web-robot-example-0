import assert from 'node:assert/strict';
import {existsSync,readFileSync,readdirSync} from 'node:fs';
import test from 'node:test';
import {FRANKA_ASSEMBLY1_LAYOUT,FRANKA_ASSEMBLY2_LAYOUT} from '../src/frankaAssemblyLayouts.js';

const xml=layout=>layout.xmlPatches.map(p=>p.inject??p.replace?.[1]??'').join('\n');
test('Assembly1 adds a free wrench on the empty pad without adding it to Assembly2',()=>{
  assert.match(xml(FRANKA_ASSEMBLY1_LAYOUT),/<body name="open_end_wrench"[^>]*>\s*<freejoint\/>/);
  assert.doesNotMatch(xml(FRANKA_ASSEMBLY2_LAYOUT),/open_end_wrench/);
});

test('thin wrench collision does not inflate outside the visible tool envelope',()=>{
  const dir=new URL('../public/assets/franka-assembly2/tools/',import.meta.url);
  const vertices=file=>readFileSync(new URL(file,dir),'utf8').split('\n').filter(l=>l.startsWith('v ')).map(l=>l.slice(2).split(/\s+/).map(Number));
  const visual=vertices('robodojo-wrench.obj');
  const bounds=[0,1,2].map(i=>[Math.min(...visual.map(p=>p[i])),Math.max(...visual.map(p=>p[i]))]);
  for(const file of readdirSync(dir).filter(f=>/^robodojo-wrench-collision-\d+\.obj$/.test(f))){
    for(const point of vertices(file))for(let i=0;i<3;i++)assert.ok(point[i]>=bounds[i][0]-.00002&&point[i]<=bounds[i][1]+.00002,`${file} protrudes beyond the source mesh`);
  }
});

test('benchmark wrench preserves its measured size and complete source visual topology',()=>{
  const path=new URL('../public/assets/franka-assembly2/tools/robodojo-wrench.obj',import.meta.url);
  assert.ok(existsSync(path),'converted benchmark mesh must exist');
  const lines=readFileSync(path,'utf8').split('\n');
  const points=lines.filter(l=>l.startsWith('v ')).map(l=>l.slice(2).split(/\s+/).map(Number));
  assert.equal(points.length,1399);
  assert.equal(lines.filter(l=>l.startsWith('f ')).length,876);
  const extent=[0,1,2].map(i=>Math.max(...points.map(p=>p[i]))-Math.min(...points.map(p=>p[i])));
  assert.ok(Math.abs(extent[0]-.0273)<.0002);
  assert.ok(Math.abs(extent[1]-.1699)<.0002);
  assert.ok(Math.abs(extent[2]-.0062)<.0002);
  assert.ok(Math.abs(Math.min(...points.map(p=>p[2]))+.0031)<.0002);
});
