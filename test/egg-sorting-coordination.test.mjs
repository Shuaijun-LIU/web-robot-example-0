import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

test('conflicting whole-path reservations yield while independent arms run together',async()=>{
  const module=await import('../src/eggSorting.js').catch(()=>null);
  assert.ok(module,'multi-arm reservation implementation is missing');
  const {reserveBatch}=module;
  assert.deepEqual(reserveBatch([0,1,2,3],[[0,1],[1,2]]),{running:[0,2,3],waiting:[1]});
  assert.deepEqual(reserveBatch([1],[[0,1],[1,2]]),{running:[1],waiting:[]});
  assert.deepEqual(reserveBatch([0,1,2,3],[]),{running:[0,1,2,3],waiting:[]});
});

test('sorting recipes reject duplicated eggs, wrong owners and overlapping actuator ownership',async()=>{
  const {isValidSortingPlan}=await import('../src/eggSorting.js');
  assert.equal(typeof isValidSortingPlan,'function','sorting asset validator is missing');
  const single=JSON.parse(readFileSync('public/assets/franka-egg-sorting/first-egg-motion.json'));
  const task={arm:0,classIndex:0,eggIndex:0,cellIndex:0,cell:single.cell,start:0,phases:single.phases};
  const duration=single.phases.reduce((s,p)=>s+p.duration,0);
  const plan={schemaVersion:1,program:'sorting',success:true,initialJoints:single.initialJoints,initialEggPositions:single.initialEggPositions,
    duration,tasks:[task],counts:[1,0,0,0]};
  assert.equal(isValidSortingPlan(plan),true);
  assert.equal(isValidSortingPlan({...plan,tasks:[task,task],counts:[2,0,0,0]}),false);
  assert.equal(isValidSortingPlan({...plan,tasks:[{...task,arm:1,classIndex:1}],counts:[0,1,0,0]}),false);
  assert.equal(isValidSortingPlan({...plan,tasks:[task,{...task,eggIndex:7,cellIndex:1,cell:[-.358,-.442,.1463]}],counts:[2,0,0,0]}),false);
  assert.equal(isValidSortingPlan({...plan,counts:[4,4,4,4]}),false);
  assert.equal(isValidSortingPlan({...plan,tasks:[{...task,cell:[0,0,.1463]}]}),false);
  assert.equal(isValidSortingPlan({...plan,initialJoints:Array(7).fill(0)}),false);
});

test('sampling holds waiting arms at home and exposes all due physical gates',async()=>{
  const {sampleSortingTask,sortingEvents}=await import('../src/eggSorting.js');
  assert.equal(typeof sampleSortingTask,'function');
  const phases=[{name:'lower',duration:2,gripper:160,gate:null,path:[[0,0],[1,1]]},
    {name:'close',duration:1,gripper:0,gate:'bilateral',path:[[1,1],[1,1]]}];
  const task={start:4,phases};
  assert.equal(sampleSortingTask(task,3),null);
  assert.deepEqual(sampleSortingTask(task,4-1e-10)?.joints,[0,0],'reservation epsilon must agree with the sampler at start');
  assert.deepEqual(sampleSortingTask(task,5).joints,[.5,.5]);
  assert.equal(sampleSortingTask(task,6.5).gripper,80);
  assert.deepEqual(sortingEvents({tasks:[task]}),[{time:6,taskIndex:0,stage:0},{time:7,taskIndex:0,stage:1}]);
});

test('opposite arms can share opposite quadrants but must yield in the same quadrant',async()=>{
  const {sourcePathsConflict}=await import('../src/eggSorting.js');
  assert.equal(typeof sourcePathsConflict,'function');
  assert.equal(sourcePathsConflict({arm:0,eggIndex:0},{arm:2,eggIndex:8}),false);
  assert.equal(sourcePathsConflict({arm:1,eggIndex:1},{arm:3,eggIndex:3}),true);
  assert.equal(sourcePathsConflict({arm:0,eggIndex:0},{arm:1,eggIndex:4}),true);
  assert.equal(sourcePathsConflict({arm:1,eggIndex:14},{arm:3,eggIndex:6}),true,'opposite far reaches cross over the center');
});

test('plan validation prevents conflicting starts and admits the delayed lease',async()=>{
  const {isValidSortingPlan}=await import('../src/eggSorting.js');
  const p=JSON.parse(readFileSync('artifacts/reports/demo2-sorting-parallel.json'));
  assert.equal(isValidSortingPlan(p),true);
  const bad=structuredClone(p);bad.tasks[1].start=0;bad.duration=33;
  assert.equal(isValidSortingPlan(bad),false);
});

test('asset failures never retain actuator ownership after Reset',async()=>{
  const {sortingLocksControls}=await import('../src/eggSorting.js');
  assert.equal(typeof sortingLocksControls,'function');
  assert.equal(sortingLocksControls({phase:'running'}),true);
  assert.equal(sortingLocksControls({phase:'error',failureSource:'runtime'}),true);
  assert.equal(sortingLocksControls({phase:'error',failureSource:'asset'}),false);
  assert.equal(sortingLocksControls({phase:'ready'}),false);
});

test('load transfers to the tray on landing, never to an invisible grasp',async()=>{
  const {checkSortingGate,isSortingTrayLoadBearing}=await import('../src/eggSorting.js');
  assert.equal(typeof checkSortingGate,'function');
  const o={bilateral:false,fingerContacts:1,traySupported:true,lift:.006,tiltDegrees:2,cellError:.0003,
    speed:.0006,tcpDistance:.025,aperture:.048,forbiddenPenetration:0,gripPenetration:.0003,neighborDisplacement:.001};
  assert.equal(checkSortingGate('supported',o),null);
  assert.equal(checkSortingGate('carried',o),'missing-two-sided-contact');
  assert.equal(isSortingTrayLoadBearing({...o,traySupported:false}),false);
  assert.notEqual(checkSortingGate('supported',{...o,traySupported:false}),null);
  assert.notEqual(checkSortingGate('supported',{...o,cellError:.005}),null);
  assert.notEqual(checkSortingGate('supported',{...o,tiltDegrees:25}),null);
  assert.notEqual(checkSortingGate('supported',{...o,fingerContacts:0}),null);
});

test('unexpected controller exceptions stop the task without tearing down the scene',async()=>{
  const {EggSortingRuntime}=await import('../src/EggSortingRuntime.js');
  const p=JSON.parse(readFileSync('artifacts/reports/demo2-sorting-parallel.json'));
  const r=new EggSortingRuntime(p,{});r.state.phase='running';
  r.advance=()=>{throw new Error('injected-controller-error');};
  assert.doesNotThrow(()=>r.step({},{}));
  assert.equal(r.state.phase,'error');
  assert.match(r.state.reason,/injected-controller-error/);
  r.state.phase='running';
  r.refs={qa:Array.from({length:4},(_,a)=>Array.from({length:7},(_,j)=>a*7+j)),da:Array.from({length:4},(_,a)=>Array.from({length:7},(_,j)=>a*7+j))};
  const d={qpos:new Float64Array(28).fill(.2),qfrc_bias:new Float64Array(28).fill(2),ctrl:new Float64Array(32).fill(17)};
  assert.doesNotThrow(()=>r.step({actuator_gainprm:new Float64Array(320).fill(100)},d));
  for(let a=0;a<4;a++){
    for(let j=0;j<7;j++)assert.equal(d.ctrl[a*8+j],.22);
    assert.equal(d.ctrl[a*8+7],17,'safety stop preserves the commanded physical pinch');
  }
  const startup=new EggSortingRuntime(p,{joint:()=>{throw new Error('injected-startup-error');}});
  assert.doesNotThrow(()=>startup.start({},{}));
  assert.equal(startup.state.phase,'error');
  assert.match(startup.state.reason,/injected-startup-error/);
});
