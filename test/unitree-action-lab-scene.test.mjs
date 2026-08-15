import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import { UNITREE_ACTION_LAB_LAYOUT } from '../src/unitreeActionLab.js';

const repoRoot = new URL('..', import.meta.url).pathname;

test('Unitree Action Lab compiles two floating articulated robots', () => {
  const report = execFileSync(process.execPath, [
    'scripts/validate-mjcf.mjs',
    'unitreeActionLab',
    'public/assets/unitree-action-lab',
  ], { cwd: repoRoot, encoding: 'utf8' });

  assert.match(report, /unitreeActionLab: .* 47 actuators, 61 qpos/);
  assert.match(
    report,
    /floating roots: g1_floating_base_joint, go2_floating_base_joint/,
  );
  assert.equal(UNITREE_ACTION_LAB_LAYOUT.homeJoints.length, 47);
  assert.equal(UNITREE_ACTION_LAB_LAYOUT.instanceCount, 2);
  assert.deepEqual(
    UNITREE_ACTION_LAB_LAYOUT.dynamicRoots,
    ['g1_pelvis', 'go2_base'],
  );
  assert.deepEqual(UNITREE_ACTION_LAB_LAYOUT.camera.position, [3.1, -3.8, 2.1]);
});

test('vendored dynamic robot packages retain their source licenses', async () => {
  const g1License = await readFile(
    new URL('../public/assets/unitree-action-lab/robots/g1/LICENSE', import.meta.url),
    'utf8',
  );
  const go2License = await readFile(
    new URL('../public/assets/unitree-action-lab/robots/go2_arm/LICENSE', import.meta.url),
    'utf8',
  );
  assert.match(g1License, /directory 'unitree_g1\/'/);
  assert.match(g1License, /Unitree Robotics/);
  assert.match(go2License, /Apache License\s+Version 2\.0/);
});
