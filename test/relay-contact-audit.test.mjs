import test from 'node:test';
import assert from 'node:assert/strict';
import {RelayContactAudit,boundsSeparation} from './helpers/relay-contact-audit.mjs';

test('contact witnesses reject stale evidence and actual separation instead of inventing a grasp',()=>{
  let deleted=0;const m={geom_bodyid:[0,1]};
  const d={time:1,xpos:new Float64Array(6),xmat:new Float64Array([1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1]),ncon:1,
    get contact(){return {get(){return {geom1:0,geom2:1,dist:-.00002,pos:[0,0,0],frame:[1,0,0],delete(){deleted++;}};},delete(){deleted++;}};}};
  const audit=new RelayContactAudit(m,d,0,[[1]]);
  assert.equal(audit.gap(d,1),Infinity,'unobserved proximity is never a grasp');
  audit.update(m,d);assert.equal(deleted,2,'release contact and vector handles');
  assert.ok(Math.abs(audit.gap(d,1)-.00002)<1e-12);
  d.time+=.002;d.xpos[3]=.001;
  assert.ok(audit.gap(d,1)>.0009,'one millimetre separation must fail the 50-micrometre bound');
  d.xpos[3]=.00002;assert.ok(audit.gap(d,1)<1e-12);
  d.time=1.01;assert.equal(audit.gap(d,1),Infinity,'old contact cannot justify current hold');
  d.time=.5;assert.equal(audit.gap(d,1),Infinity,'reset invalidates future witnesses');
});

test('vertex bounds certify separation conservatively without a mesh-distance query',()=>{
  assert.equal(boundsSeparation([[0,0,0],[1,1,1]],[[1.5,0,0],[2,1,1]]),.5);
  assert.equal(boundsSeparation([[0,0,0],[1,1,1]],[[.9,.9,.9],[2,2,2]]),0);
  assert.equal(boundsSeparation([[0,0,0],[1,1,1]],[[1,0,0],[2,1,1]]),0);
});
