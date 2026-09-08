import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {checkEggGate,isValidEggPlan,isEggInspectionMatch} from '../src/eggTransfer.js';

test('measured regrasp pose must stay inside the checked correction envelope',()=>{
  const expected={position:[0,0,.15],quaternion:[1,0,0,0]};
  assert.equal(isEggInspectionMatch(expected,[.001,0,.15],[-1,0,0,0]),true);
  assert.equal(isEggInspectionMatch(expected,[.005,0,.15],[1,0,0,0]),false);
  assert.equal(isEggInspectionMatch(expected,[0,0,.15],[Math.cos(.1),Math.sin(.1),0,0]),false);
  assert.equal(isEggInspectionMatch(expected,[NaN,0,.15],[1,0,0,0]),false);
});

test('correction inspection requires a settled tilted egg released onto its tray',()=>{
  const o={bilateral:false,fingerContacts:0,aperture:.10,tcpDistance:.18,lift:.007,
    traySupported:true,tiltDegrees:25,cellError:.004,speed:0,forbiddenPenetration:0,gripPenetration:0,neighborDisplacement:0};
  assert.equal(checkEggGate('tilted',o),null);
  assert.equal(checkEggGate('tilted',{...o,tiltDegrees:5}),'correction-not-needed');
  assert.equal(checkEggGate('tilted',{...o,fingerContacts:1}),'egg-not-released');
  assert.equal(checkEggGate('tilted',{...o,traySupported:false}),'missing-tray-support');
  assert.equal(checkEggGate('tilted',{...o,cellError:.03}),'outside-correction-envelope');
  assert.equal(checkEggGate('tilted',{...o,speed:.03}),'egg-not-settled');
});

test('baseline recipe remains valid while correction requires a separate ordered second grasp',()=>{
  const baseline=JSON.parse(readFileSync('public/assets/franka-egg-sorting/first-egg-motion.json','utf8'));
  assert.equal(isValidEggPlan(baseline),true);
  assert.equal(isValidEggPlan({...baseline,program:'reseat'}),false);
  assert.equal(isValidEggPlan({...baseline,program:'unknown'}),false);
  const gates=[null,null,'bilateral','carried','carried','carried','supported','released',null,'tilted',null,null,'bilateral','carried','carried','supported','released',null,null,'seated'];
  const p={...baseline,schemaVersion:2,program:'reseat',inspection:{phaseIndex:9,position:[-.438,-.438,.145],quaternion:[1,0,0,0],tiltDegrees:25},
    phases:gates.map(gate=>({name:'Test phase',duration:2,gripper:130,gate,path:[baseline.initialJoints,baseline.initialJoints]}))};
  assert.equal(isValidEggPlan(p),true);
  assert.equal(isValidEggPlan({...p,phases:p.phases.filter((_,i)=>i!==12)}),false);
  assert.equal(isValidEggPlan({...p,inspection:undefined}),false);
  assert.equal(isValidEggPlan({...p,program:'unknown'}),false);
});
