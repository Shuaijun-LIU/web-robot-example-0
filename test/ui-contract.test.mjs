import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const appPath = new URL('../src/App.tsx', import.meta.url);
const githubLinkPath = new URL('../src/GitHubLink.tsx', import.meta.url);
const stylesPath = new URL('../src/styles.css', import.meta.url);
const keyboardHelpPath = new URL('../src/KeyboardHelp.tsx', import.meta.url);

test('GitHub link points to this project repository', async () => {
  const source = await readFile(githubLinkPath, 'utf8');

  assert.match(source, /https:\/\/github\.com\/Shuaijun-LIU\/web-robot-example-0/);
});

test('scene performance panels stay visible in one labeled vertical column', async () => {
  const [source, styles] = await Promise.all([
    readFile(appPath, 'utf8'),
    readFile(stylesPath, 'utf8'),
  ]);

  assert.match(source, /className="performance-stats"/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{0\} \/>/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{1\} \/>/);
  assert.match(source, /<Stats parent=\{performanceStatsParentRef\} showPanel=\{2\} \/>/);
  assert.match(source, /Scene performance/);
  assert.match(styles, /\.performance-stats\s*\{[\s\S]*flex-direction:\s*column/);
  assert.match(styles, /\.performance-stats > div\s*\{[\s\S]*position:\s*static !important/);
});

test('keyboard help has comfortable background padding and identifies the active target', async () => {
  const [source, styles] = await Promise.all([
    readFile(keyboardHelpPath, 'utf8'),
    readFile(stylesPath, 'utf8'),
  ]);

  assert.match(source, /className="keyboard-help"/);
  assert.match(source, /controlTargetLabel/);
  assert.match(styles, /\.keyboard-help\s*\{[\s\S]*min-width:\s*15rem/);
  assert.match(styles, /\.keyboard-help\s*\{[\s\S]*padding:\s*0\.875rem 1rem/);
});

test('scene lifecycle is exposed for deterministic browser smoke tests', async () => {
  const source = await readFile(appPath, 'utf8');

  assert.match(source, /dataset\.sceneStatus = 'loading'/);
  assert.match(source, /dataset\.sceneStatus = 'ready'/);
  assert.match(source, /dataset\.sceneStatus = 'error'/);
  assert.match(source, /dataset\.sceneBodies/);
  assert.match(source, /dataset\.sceneInstances/);
  assert.match(source, /dataset\.controlTarget/);
  assert.match(source, /dataset\.ikSite/);
});

test('scene starts running with IK gizmo visible for immediate interaction', async () => {
  const source = await readFile(appPath, 'utf8');

  assert.match(source, /paused:\s*false/);
  assert.match(source, /gizmo:\s*\{\s*value:\s*true/);
  assert.match(source, /label:\s*'Control target'/);
});
