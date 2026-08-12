import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const appPath = new URL('../src/App.tsx', import.meta.url);
const githubLinkPath = new URL('../src/GitHubLink.tsx', import.meta.url);
const stylesPath = new URL('../src/styles.css', import.meta.url);

test('GitHub link points to this project repository', async () => {
  const source = await readFile(githubLinkPath, 'utf8');

  assert.match(source, /https:\/\/github\.com\/Shuaijun-LIU\/web-robot-example-0/);
});

test('performance panels stay visible in one vertical column', async () => {
  const [source, styles] = await Promise.all([
    readFile(appPath, 'utf8'),
    readFile(stylesPath, 'utf8'),
  ]);

  assert.match(source, /className="performance-stats"/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{0\} \/>/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{1\} \/>/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{2\} \/>/);
  assert.match(styles, /\.performance-stats\s*\{[\s\S]*flex-direction:\s*column/);
  assert.match(styles, /\.performance-stats > div\s*\{[\s\S]*position:\s*static !important/);
});
