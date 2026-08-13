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
    targets: ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4', 'G1', 'Go2 + Arm'],
    targetKeys: ['r0', 'r1', 'r2', 'r3', 'g1', 'go2Arm'],
    blockStarts: [6, 12, 18, 24, 0, 3],
    blockSizes: [6, 6, 6, 6, 3, 3],
    mobileBodies: [null, null, null, null, 'home_lab_g1_mobile_root', 'home_lab_go2_mobile_root'],
    mobileYawJoints: [null, null, null, null, 'home_lab_g1_yaw', 'home_lab_go2_yaw'],
    keyHoldMs: [300, 300, 300, 300, 1500, 1500],
    blockSize: 6,
    keyCode: 'KeyW',
    ik: true,
    ikTargetCount: 4,
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
const requestedTargets = new Set(
  (process.env.TARGETS ?? '').split(',').map((label) => label.trim()).filter(Boolean),
);

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
      if (requestedTargets.size > 0 && !requestedTargets.has(targetLabel)) continue;
      await targetSelector.selectOption({ label: targetLabel });
      const targetKey = scene.targetKeys?.[targetIndex] ?? `r${targetIndex}`;
      const needsIk = scene.ik && targetIndex < (scene.ikTargetCount ?? scene.targets.length);
      await page.waitForFunction(
        ({ key, needsIk }) => document.documentElement.dataset.controlTarget === key
          && (!needsIk || document.documentElement.dataset.ikSite === `${key}_tcp`),
        { key: targetKey, needsIk },
        { timeout },
      );
      const mobileBody = scene.mobileBodies?.[targetIndex];
      if (mobileBody) {
        await page.waitForFunction(
          (key) => document.documentElement.dataset.mobileController === key,
          targetKey,
          { timeout },
        );
      }

      await page.evaluate(() => window.robotDemo.reset());
      await page.waitForTimeout(250);
      const beforeBody = mobileBody
        ? await page.evaluate((name) => window.robotDemo.getBodyPositions([name])[name], mobileBody)
        : null;
      const beforeKeyboard = await page.evaluate(() => window.robotDemo.getCtrl());
      await page.keyboard.down(scene.keyCode);
      await page.waitForTimeout(scene.keyHoldMs?.[targetIndex] ?? 300);
      const duringKeyboard = await page.evaluate(() => window.robotDemo.getCtrl());
      await page.keyboard.up(scene.keyCode);
      await page.waitForTimeout(100);
      if (process.env.CONTROL_DEBUG === '1') {
        console.log(`${scene.key}/${targetLabel} ctrl before=${JSON.stringify(beforeKeyboard)} during=${JSON.stringify(duringKeyboard)}`);
      }
      assertSelectedBlock(
        changedIndices(beforeKeyboard, duringKeyboard),
        scene.blockStarts?.[targetIndex] ?? targetIndex * scene.blockSize,
        scene.blockSizes?.[targetIndex] ?? scene.blockSize,
        `${scene.key}/${targetLabel}/keyboard`,
      );
      if (mobileBody && beforeBody) {
        const afterBody = await page.evaluate(
          (name) => window.robotDemo.getBodyPositions([name])[name],
          mobileBody,
        );
        const travel = Math.hypot(afterBody[0] - beforeBody[0], afterBody[1] - beforeBody[1]);
        if (travel < 0.002) {
          throw new Error(`${scene.key}/${targetLabel}: velocity controls did not move ${mobileBody}`);
        }

        const yawJoint = scene.mobileYawJoints[targetIndex];
        await page.evaluate(() => window.robotDemo.reset());
        await page.waitForTimeout(250);
        const beforeYaw = await page.evaluate(
          (name) => window.robotDemo.getJointPositions([name])[name],
          yawJoint,
        );
        await page.keyboard.down('KeyA');
        await page.waitForTimeout(scene.keyHoldMs?.[targetIndex] ?? 300);
        await page.keyboard.up('KeyA');
        await page.waitForTimeout(100);
        const afterYaw = await page.evaluate(
          (name) => window.robotDemo.getJointPositions([name])[name],
          yawJoint,
        );
        if (Math.abs(afterYaw - beforeYaw) < 0.005) {
          throw new Error(`${scene.key}/${targetLabel}: turn control did not rotate ${yawJoint}`);
        }
      }

      if (needsIk) {
        await page.evaluate(() => window.robotDemo.reset());
        await page.waitForTimeout(250);
        const beforeIk = await page.evaluate(() => window.robotDemo.getCtrl());
        await page.evaluate(() => window.robotDemo.moveIkTargetBy(0, 0, 0.015));
        await page.waitForTimeout(900);
        const afterIk = await page.evaluate(() => window.robotDemo.getCtrl());
        assertSelectedBlock(
          changedIndices(beforeIk, afterIk),
          scene.blockStarts?.[targetIndex] ?? targetIndex * scene.blockSize,
          scene.blockSizes?.[targetIndex] ?? scene.blockSize,
          `${scene.key}/${targetLabel}/IK`,
        );
      }

      console.log(`${scene.key}/${targetLabel}: keyboard${needsIk ? ' + IK' : ''} PASS`);
    }
  }

  if (failures.length > 0) throw new Error(failures.join('\n'));
} finally {
  await browser.close();
}
