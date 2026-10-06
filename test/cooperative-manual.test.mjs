import test from 'node:test';
import assert from 'node:assert/strict';
import {toggleCooperativeGripper} from '../src/cooperativeManual.js';

test('manual toggles use each selected actuator state and honor automation ownership',()=>{
  const data={ctrl:new Float64Array(32).fill(255)};
  assert.equal(toggleCooperativeGripper(data,7,true),true);
  assert.equal(data.ctrl[7],0);
  toggleCooperativeGripper(data,15,true);
  assert.equal(data.ctrl[7],0,'operating Arm 2 cannot reopen Arm 1');
  assert.equal(data.ctrl[15],0);
  assert.equal(toggleCooperativeGripper(data,7,false),false);
  assert.equal(data.ctrl[7],0,'automation owns actuators while running');
  toggleCooperativeGripper(data,7,true);
  assert.equal(data.ctrl[7],255);
  for(const id of [-1,32,NaN])assert.equal(toggleCooperativeGripper(data,id,true),false);
});
