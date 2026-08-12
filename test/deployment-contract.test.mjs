import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const workflowPath = new URL('../.github/workflows/pages.yml', import.meta.url);
const readmePath = new URL('../README.md', import.meta.url);
const packagePath = new URL('../package.json', import.meta.url);

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

test('README documents and displays all three verified layouts', async () => {
  const readme = await readFile(readmePath, 'utf8');

  assert.match(readme, /artifacts\/screenshots\/franka\.png/);
  assert.match(readme, /artifacts\/screenshots\/so101\.png/);
  assert.match(readme, /artifacts\/screenshots\/xlerobot\.png/);
  assert.match(readme, /0\.775 m/);
  assert.match(readme, /Control target/);
  assert.match(readme, /every physical instance/i);
});
