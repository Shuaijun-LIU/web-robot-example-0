import { access } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const failures = [];
const allScenes = [
  {
    key: 'franka',
    label: 'Franka Panda',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 8,
    keyCode: 'KeyV',
    ik: true,
  },
  {
    key: 'so101',
    label: 'SO101',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 6,
    keyCode: 'KeyW',
    ik: true,
  },
  {
    key: 'xlerobot',
    label: 'XLeRobot',
    targets: ['Robot 1', 'Robot 2'],
    blockSize: 16,
    keyCode: 'KeyR',
    ik: false,
  },
  {
    key: 'so101Gearbox',
    label: 'SO101 Gearbox',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 6,
    keyCode: 'KeyW',
    ik: true,
  },
  {
    key: 'so101HomeLab',
    label: 'SO101 Home Lab',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 6,
    keyCode: 'KeyW',
    ik: true,
  },
  {
    key: 'xlerobotKitting',
    label: 'XLeRobot Kitting',
    targets: ['Robot 1', 'Robot 2'],
    blockSize: 16,
    keyCode: 'KeyR',
    ik: false,
  },
  {
    key: 'frankaAssembly1',
    label: 'Franka Assembly1',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 8,
    keyCode: 'KeyV',
    ik: true,
  },
  {
    key: 'frankaAssembly2',
    label: 'Franka Assembly2',
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'],
    blockSize: 8,
    keyCode: 'KeyV',
    ik: true,
  },
];
const requestedKeys = new Set(
  (process.env.SCENES ?? allScenes.map(({ key }) => key).join(','))
    .split(',')
    .map((key) => key.trim()),
);
const scenes = allScenes.filter(({ key }) => requestedKeys.has(key));

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('console', (message) => {
  if (message.type() === 'error') failures.push(`console error: ${message.text()}`);
});

async function routeLocalAssets(urlPrefix, directory) {
  if (!directory) return;
  await page.route(`${urlPrefix}**`, async (route) => {
    const relativePath = decodeURIComponent(route.request().url().slice(urlPrefix.length));
    const localPath = resolve(directory, relativePath);
    try {
      await access(localPath);
      await route.fulfill({ path: localPath });
    } catch {
      await route.continue();
    }
  });
}

await routeLocalAssets(
  'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/',
  process.env.FRANKA_ASSET_DIR,
);
await routeLocalAssets(
  'https://raw.githubusercontent.com/Vector-Wangel/MuJoCo-GS-Web/main/assets/robots/xlerobot/',
  process.env.XLEROBOT_ASSET_DIR,
);

function changedIndices(before, after, epsilon = 1e-5) {
  return before.flatMap((value, index) => (
    Math.abs(value - after[index]) > epsilon ? [index] : []
  ));
}

function assertSelectedBlock(indices, blockStart, blockSize, context) {
  if (indices.length === 0) throw new Error(`${context}: no actuator changed`);
  const outside = indices.filter((index) => index < blockStart || index >= blockStart + blockSize);
  if (outside.length > 0) {
    throw new Error(`${context}: changed actuators outside selected block: ${outside.join(', ')}`);
  }
}

try {
  await page.goto(baseUrl, {
    waitUntil: 'domcontentloaded',
    timeout: Math.min(timeout, 120_000),
  });
  const robotSelector = page.locator('select').nth(0);
  const targetSelector = page.locator('select').nth(1);

  for (const scene of scenes) {
    await robotSelector.selectOption({ label: scene.label });
    await page.waitForFunction(
      (key) => document.documentElement.dataset.sceneKey === key
        && document.documentElement.dataset.sceneStatus === 'ready',
      scene.key,
      { timeout },
    );
    await page.waitForFunction(() => Boolean(window.robotDemo), null, { timeout });

    for (let targetIndex = 0; targetIndex < scene.targets.length; targetIndex += 1) {
      const targetLabel = scene.targets[targetIndex];
      await targetSelector.selectOption({ label: targetLabel });
      const targetKey = `r${targetIndex}`;
      await page.waitForFunction(
        ({ key, needsIk }) => document.documentElement.dataset.controlTarget === key
          && (!needsIk || document.documentElement.dataset.ikSite === `${key}_tcp`),
        { key: targetKey, needsIk: scene.ik },
        { timeout },
      );

      await page.evaluate(() => window.robotDemo.reset());
      await page.waitForTimeout(250);
      const beforeKeyboard = await page.evaluate(() => window.robotDemo.getCtrl());
      await page.keyboard.down(scene.keyCode);
      await page.waitForTimeout(300);
      await page.keyboard.up(scene.keyCode);
      await page.waitForTimeout(100);
      const afterKeyboard = await page.evaluate(() => window.robotDemo.getCtrl());
      assertSelectedBlock(
        changedIndices(beforeKeyboard, afterKeyboard),
        targetIndex * scene.blockSize,
        scene.blockSize,
        `${scene.key}/${targetLabel}/keyboard`,
      );

      if (scene.ik) {
        await page.evaluate(() => window.robotDemo.reset());
        await page.waitForTimeout(250);
        const beforeIk = await page.evaluate(() => window.robotDemo.getCtrl());
        await page.evaluate(() => window.robotDemo.moveIkTargetBy(0, 0, 0.015));
        await page.waitForTimeout(900);
        const afterIk = await page.evaluate(() => window.robotDemo.getCtrl());
        assertSelectedBlock(
          changedIndices(beforeIk, afterIk),
          targetIndex * scene.blockSize,
          scene.blockSize,
          `${scene.key}/${targetLabel}/IK`,
        );
      }

      console.log(`${scene.key}/${targetLabel}: keyboard${scene.ik ? ' + IK' : ''} PASS`);
    }
  }

  if (failures.length > 0) throw new Error(failures.join('\n'));
} finally {
  await browser.close();
}
