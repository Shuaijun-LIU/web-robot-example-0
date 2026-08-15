import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const repoRoot = new URL('..', import.meta.url).pathname;

test('scene catalog registers local PiPER and UR5e Assembly1 entries', async () => {
  const source = await readFile(new URL('../src/configs.ts', import.meta.url), 'utf8');

  assert.match(source, /const PIPER_ASSEMBLY1_BASE = `\$\{import\.meta\.env\.BASE_URL\}assets\/piper-assembly1\//);
  assert.match(source, /const UR5E_ASSEMBLY1_BASE = `\$\{import\.meta\.env\.BASE_URL\}assets\/ur5e-assembly1\//);
  assert.match(source, /piperAssembly1:\s*\{[\s\S]*?label:\s*'Piper Assembly1'/);
  assert.match(source, /piperAssembly1:\s*\{[\s\S]*?controlFamily:\s*'industrialArm'/);
  assert.match(source, /piperAssembly1:\s*\{[\s\S]*?controlTargets:\s*createPiperTargets\(\)/);
  assert.match(source, /ur5eAssembly1:\s*\{[\s\S]*?label:\s*'UR5e Assembly1'/);
  assert.match(source, /ur5eAssembly1:\s*\{[\s\S]*?controlFamily:\s*'industrialArm'/);
  assert.match(source, /ur5eAssembly1:\s*\{[\s\S]*?controlTargets:\s*createUR5eTargets\(\)/);
});

for (const [sceneKey, assetDirectory, rootName, tcpName] of [
  ['piperAssembly1', 'public/assets/piper-assembly1', 'r3_base_link', 'r3_tcp'],
  ['ur5eAssembly1', 'public/assets/ur5e-assembly1', 'r3_base', 'r3_gripper_pinch'],
]) {
  test(`${sceneKey} strictly compiles four finite robots and the Assembly1 workcell`, () => {
    const report = execFileSync(process.execPath, [
      'scripts/validate-mjcf.mjs',
      sceneKey,
      assetDirectory,
    ], {
      cwd: repoRoot,
      encoding: 'utf8',
      env: {
        ...process.env,
        LIST_NAMES: '1',
        POSE_REPORT: '1',
        INITIAL_CONTACT_REPORT: '1',
        INITIAL_CONTACT_STRICT: '1',
      },
    });

    assert.match(report, new RegExp(`${sceneKey}: .* 28 actuators`));
    assert.match(report, new RegExp(rootName));
    assert.match(report, new RegExp(tcpName));
    assert.match(report, /instance 3: root=\[[^\]]+\] tcp=\[[^\]]+\]/);
    assert.doesNotMatch(report, /null|NaN/);
    assert.match(report, /initial penetrating contacts: 0\//);
  });
}
