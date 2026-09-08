import test from 'node:test';
import assert from 'node:assert/strict';
import { existsSync } from 'node:fs';

const file = new URL('../src/eggTransfer.js', import.meta.url);

test('a physics Reset cancels playback before stale actuator targets can run',async()=>{
  const {didEggClockReset}=await import(file);
  assert.equal(typeof didEggClockReset,'function');
  assert.equal(didEggClockReset(null,0),false);
  assert.equal(didEggClockReset(0,0),false);
  assert.equal(didEggClockReset(12.4,12.402),false);
  assert.equal(didEggClockReset(.002,0),true);
  assert.equal(didEggClockReset(12.4,0),true);
});

test('motion loading rejects stale or malformed assets before physics playback',async()=>{
  const {isValidEggPlan}=await import(file);
  assert.equal(typeof isValidEggPlan,'function');
  const plan={schemaVersion:1,success:true,arm:0,egg:'egg_0',initialJoints:Array(7).fill(0),initialGripper:255,
    cell:[0,0,.14],initialEggPositions:Array.from({length:16},()=>[0,0,.14]),
    phases:[null,null,'bilateral','carried','carried','carried','supported','released',null,null,'seated'].map(gate=>({name:'Move',duration:2,gripper:160,gate,path:[Array(7).fill(0),Array(7).fill(0)]}))};
  assert.equal(isValidEggPlan(plan),true);
  for(const invalid of [null,{}, {...plan,initialEggPositions:undefined},
    {...plan,initialEggPositions:[[0,0,.14]]},
    {...plan,phases:[plan.phases[0]]},
    {...plan,phases:plan.phases.map((phase,i)=>i===2?{...phase,path:[Array(7).fill(1),Array(7).fill(0)]}:phase)},
    ...[{duration:NaN},{gripper:300},{gate:'pretend-grasp'}].map(change=>({...plan,phases:[{...plan.phases[0],...change},...plan.phases.slice(1)]}))])assert.equal(isValidEggPlan(invalid),false);
});

test('egg transfer gates reject empty-air grasp, dropped egg and unsupported seating', async () => {
  assert.ok(existsSync(file), 'contact-gated egg transfer controller must exist');
  const { checkEggGate } = await import(file);
  const held = { bilateral:true, fingerContacts:2, aperture:.043, tcpDistance:.035, lift:.12, traySupported:false,
    tiltDegrees:5, cellError:.002, speed:0, forbiddenPenetration:0, gripPenetration:.0002, neighborDisplacement:0 };
  assert.equal(checkEggGate('bilateral', held), null);
  assert.equal(checkEggGate('bilateral', {...held, bilateral:false}), 'missing-two-sided-contact');
  assert.equal(checkEggGate('bilateral', {...held, aperture:.002}), 'invalid-grip-aperture');
  assert.equal(checkEggGate('carried', {...held, tcpDistance:.14}), 'egg-not-carried');
  assert.equal(checkEggGate('seated', {...held, bilateral:false}), 'missing-tray-support');
  assert.equal(checkEggGate('seated', {...held, bilateral:false, fingerContacts:0, traySupported:true}), null);
  assert.equal(checkEggGate('seated', {...held, bilateral:false, fingerContacts:0, traySupported:true, tiltDegrees:45}), 'egg-not-upright');
  assert.equal(checkEggGate('released', {...held, bilateral:false, fingerContacts:1, traySupported:true}), 'egg-not-released');
  assert.equal(checkEggGate('bilateral', {...held, forbiddenPenetration:.003}), 'unexpected-collision');
  assert.equal(checkEggGate('bilateral', {...held, gripPenetration:.003}), 'excessive-grip-penetration');
  assert.equal(checkEggGate('bilateral', {...held, tcpDistance:NaN}), 'non-finite-state');
});

test('egg motion follows every checked path knot and interpolates aperture continuously', async () => {
  assert.ok(existsSync(file));
  const { sampleEggPhase } = await import(file);
  const phase = { path:[[0,0],[1,0],[1,1]], gripper:0 };
  assert.deepEqual(sampleEggPhase(phase,0,160), {joints:[0,0],gripper:160});
  assert.deepEqual(sampleEggPhase(phase,.5,160), {joints:[1,0],gripper:80});
  assert.deepEqual(sampleEggPhase(phase,1,160), {joints:[1,1],gripper:0});
  assert.deepEqual(sampleEggPhase(phase,2,160), {joints:[1,1],gripper:0});
});

test('carried egg collisions include fixed obstacles and permit only intended support', async () => {
  const module=await import(file);
  assert.equal(typeof module.isForbiddenEggContact,'function','held-object contacts need an explicit policy');
  const context={robotBodies:new Set([1,2,3]),fingers:[2,3],egg:4,allowedSupports:new Set([5])};
  assert.equal(module.isForbiddenEggContact(4,5,context),false);
  assert.equal(module.isForbiddenEggContact(2,4,context),false);
  assert.equal(module.isForbiddenEggContact(4,6,context),true,'egg versus fixed box is a collision');
  assert.equal(module.isForbiddenEggContact(4,7,context),true,'egg versus neighboring egg is a collision');
  assert.equal(module.isForbiddenEggContact(1,5,context),true,'wrist versus support is not an allowed egg contact');
  assert.equal(module.isForbiddenEggContact(8,5,context),false,'untouched egg may rest on its own insert');
});
