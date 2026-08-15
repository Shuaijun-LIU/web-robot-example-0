import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const targets = ['Arm 1', 'Arm 2', 'Arm 3', 'Arm 4'];
const allScenes = [
  {
    key: 'piperAssembly1',
    label: 'Piper Assembly1',
    siteSuffix: 'tcp',
  },
  {
    key: 'ur5eAssembly1',
    label: 'UR5e Assembly1',
    siteSuffix: 'gripper_pinch',
  },
];
const requestedKeys = new Set(
  (process.env.SCENES ?? allScenes.map(({ key }) => key).join(','))
    .split(',')
    .map((key) => key.trim()),
);
const scenes = allScenes.filter(({ key }) => requestedKeys.has(key));
const failures = [];

function changedIndices(before, after, epsilon = 1e-5) {
  return before.flatMap((value, index) => (
    Math.abs(value - after[index]) > epsilon ? [index] : []
  ));
}

function assertExactly(indices, expected, context) {
  if (indices.length !== expected.length || indices.some((value, index) => value !== expected[index])) {
    throw new Error(`${context}: changed [${indices.join(', ')}], expected [${expected.join(', ')}]`);
  }
}

function assertSelectedArm(indices, armIndex, context) {
  const start = armIndex * 7;
  const armIndices = indices.filter((index) => index >= start && index < start + 6);
  const outside = indices.filter((index) => index < start || index >= start + 6);
  if (armIndices.length === 0) throw new Error(`${context}: IK changed no selected-arm actuator`);
  if (outside.length > 0) {
    throw new Error(`${context}: IK changed actuators outside selected arm: ${outside.join(', ')}`);
  }
}

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('requestfailed', (request) => {
  failures.push(`request failed: ${request.url()} (${request.failure()?.errorText ?? 'unknown'})`);
});
page.on('console', (message) => {
  const text = message.text();
  if (message.type() === 'error' || /Simulation error|XML Error|Aborted\(/i.test(text)) {
    failures.push(`console ${message.type()}: ${text}`);
  }
});

try {
  await page.goto(baseUrl, {
    waitUntil: 'domcontentloaded',
    timeout: Math.min(timeout, 120_000),
  });
  const sceneSelector = page.locator('select').nth(0);
  const targetSelector = page.locator('select').nth(1);

  for (const scene of scenes) {
    await sceneSelector.selectOption({ label: scene.label });
    await page.waitForFunction(
      (key) => document.documentElement.dataset.sceneKey === key
        && document.documentElement.dataset.sceneStatus !== 'loading',
      scene.key,
      { timeout },
    );
    const state = await page.evaluate(() => ({ ...document.documentElement.dataset }));
    if (state.sceneStatus !== 'ready') {
      throw new Error(`${scene.key}: ${state.sceneError ?? 'scene did not become ready'}`);
    }
    if (Number(state.sceneInstances) !== 4) {
      throw new Error(`${scene.key}: Number(sceneInstances) !== 4 (${state.sceneInstances})`);
    }
    await page.waitForFunction(() => Boolean(window.robotDemo), null, { timeout });

    for (let armIndex = 0; armIndex < targets.length; armIndex += 1) {
      const targetLabel = targets[armIndex];
      const targetKey = `r${armIndex}`;
      const expectedSite = `${targetKey}_${scene.siteSuffix}`;
      await targetSelector.selectOption({ label: targetLabel });
      await page.waitForFunction(
        ({ targetKey: key, expectedSite: site }) => (
          document.documentElement.dataset.controlTarget === key
          && document.documentElement.dataset.ikSite === site
        ),
        { targetKey, expectedSite },
        { timeout },
      );

      await page.evaluate(() => window.robotDemo.reset());
      await page.waitForTimeout(300);
      const beforeGripper = await page.evaluate(() => window.robotDemo.getCtrl());
      await page.keyboard.down('KeyV');
      await page.waitForTimeout(120);
      await page.keyboard.up('KeyV');
      await page.waitForTimeout(120);
      const afterGripper = await page.evaluate(() => window.robotDemo.getCtrl());
      assertExactly(
        changedIndices(beforeGripper, afterGripper),
        [armIndex * 7 + 6],
        `${scene.key}/${targetLabel}/gripper`,
      );

      await page.evaluate(() => window.robotDemo.reset());
      await page.waitForTimeout(300);
      const beforeIk = await page.evaluate(() => window.robotDemo.getCtrl());
      const moved = await page.evaluate(() => window.robotDemo.moveIkTargetBy(0, 0, 0.012));
      if (!moved) throw new Error(`${scene.key}/${targetLabel}: IK controller unavailable`);
      await page.waitForTimeout(1_000);
      const afterIk = await page.evaluate(() => window.robotDemo.getCtrl());
      assertSelectedArm(
        changedIndices(beforeIk, afterIk),
        armIndex,
        `${scene.key}/${targetLabel}/IK`,
      );

      console.log(`${scene.key}/${targetLabel}: gripper + IK PASS (${expectedSite})`);
    }
  }

  if (failures.length > 0) throw new Error(failures.join('\n'));
} finally {
  await browser.close();
}
