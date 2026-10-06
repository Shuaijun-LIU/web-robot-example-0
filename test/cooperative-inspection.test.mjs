import test from 'node:test';
import assert from 'node:assert/strict';
import {requestCooperativeInspectionSteps} from '../src/cooperativeMotion.js';

test('inspection requests bounded real physics steps, never changes body state',()=>{
  const calls=[],api={step:n=>calls.push(n)};
  assert.equal(requestCooperativeInspectionSteps(null,500),false);
  for(const n of [0,-1,1.5,1001,Infinity,NaN])assert.equal(requestCooperativeInspectionSteps(api,n),false);
  assert.deepEqual(calls,[]);
  assert.equal(requestCooperativeInspectionSteps(api,500),true);
  assert.deepEqual(calls,[500]);
});
