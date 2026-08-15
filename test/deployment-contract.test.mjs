import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const workflowPath = new URL('../.github/workflows/pages.yml', import.meta.url);
const readmePath = new URL('../README.md', import.meta.url);
const packagePath = new URL('../package.json', import.meta.url);
const captureScenesPath = new URL('../scripts/capture-scenes.mjs', import.meta.url);
const verifyControlsPath = new URL('../scripts/verify-controls.mjs', import.meta.url);

test('GitHub Actions deploys the Pages build from main', async () => {
  const workflow = await readFile(workflowPath, 'utf8');

  assert.match(workflow, /branches:\s*\[main\]/);
  assert.match(workflow, /npm run build:pages/);
  assert.match(workflow, /actions\/upload-pages-artifact@v4/);
  assert.match(workflow, /actions\/deploy-pages@v5/);
});

test('Pages build preserves design and plan records under docs', async () => {
  const packageJson = JSON.parse(await readFile(packagePath, 'utf8'));

  assert.match(packageJson.scripts['build:pages'], /--emptyOutDir=false/);
});

test('README documents and displays all eleven verified layouts', async () => {
  const readme = await readFile(readmePath, 'utf8');

  assert.match(readme, /artifacts\/screenshots\/franka\.png/);
  assert.match(readme, /artifacts\/screenshots\/so101\.png/);
  assert.match(readme, /artifacts\/screenshots\/xlerobot\.png/);
  assert.match(readme, /artifacts\/screenshots\/franka-assembly1\.png/);
  assert.match(readme, /artifacts\/screenshots\/franka-assembly2\.png/);
  assert.match(readme, /artifacts\/screenshots\/xlerobot-kitting\.png/);
  assert.match(readme, /artifacts\/screenshots\/so101-gearbox\.png/);
  assert.match(readme, /artifacts\/screenshots\/so101-home-lab\.png/);
  assert.match(readme, /artifacts\/screenshots\/unitree-action-lab\.png/);
  assert.match(readme, /artifacts\/screenshots\/piper-assembly1\.png/);
  assert.match(readme, /artifacts\/screenshots\/ur5e-assembly1\.png/);
  assert.match(readme, /artifacts\/videos\/unitree-action-lab\.mp4/);
  assert.match(readme, /Unitree Action Lab/);
  assert.match(readme, /actuator-only/i);
  assert.match(readme, /0\.775 m/);
  assert.match(readme, /Control target/);
  assert.match(readme, /every physical instance/i);
});

test('browser verification selects every requested scene independent of the page default', async () => {
  for (const scriptPath of [captureScenesPath, verifyControlsPath]) {
    const source = await readFile(scriptPath, 'utf8');
    assert.doesNotMatch(source, /if \(scene(?:Index > 0|\.key !== 'franka')\)/);
    assert.match(source, /selectOption\(\{ label: scene\.label \}\)/);
  }
});
