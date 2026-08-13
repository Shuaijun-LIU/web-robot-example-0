import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const screenshotPath = resolve('artifacts/screenshots/franka-assembly1-step1-grasp-ready.png');
const taskBodies = [
  'assembly_frame',
  'cross_member',
  'manual_screwdriver',
  'torque_driver',
  'double_face_hammer',
  'mounting_plate',
  'fastener_1',
  'fastener_2',
  'fastener_3',
  'fastener_4',
];
const tcpTargets = {
  r0_tcp: { position: [0, -0.23, 0.33], closingAxis: [0, 1, 0] },
  r1_tcp: { position: [0.559, -0.421, 0.28], closingAxis: [-0.951057, 0.309017, 0] },
  r2_tcp: { position: [-0.49, 0.56, 0.26], closingAxis: [1, 0, 0] },
  r3_tcp: { position: [-0.49, 0.32, 0.26], closingAxis: [1, 0, 0] },
};

function distance(a, b) {
  return Math.hypot(...a.map((value, index) => value - b[index]));
}

function axisErrorDegrees(actual, target, signSymmetric = false) {
  const denominator = Math.hypot(...actual) * Math.hypot(...target);
  if (denominator === 0) return Number.POSITIVE_INFINITY;
  const rawDot = actual.reduce((sum, value, index) => sum + value * target[index], 0)
    / denominator;
  const cosine = Math.max(-1, Math.min(1, signSymmetric ? Math.abs(rawDot) : rawDot));
  return Math.acos(cosine) * 180 / Math.PI;
}

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
const failures = [];
const validationFailures = [];
page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('console', (message) => {
  if (message.type() === 'error') failures.push(`console error: ${message.text()}`);
});

if (process.env.FRANKA_ASSET_DIR) {
  const prefix = 'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/franka_emika_panda/';
  await page.route(`${prefix}**`, async (route) => {
    const relativePath = decodeURIComponent(route.request().url().slice(prefix.length));
    const localPath = resolve(process.env.FRANKA_ASSET_DIR, relativePath);
    try {
      await access(localPath);
      await route.fulfill({ path: localPath });
    } catch {
      await route.continue();
    }
  });
}

try {
  await page.goto(baseUrl, { waitUntil: 'domcontentloaded', timeout: Math.min(timeout, 120_000) });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneKey === 'frankaAssembly1'
      && document.documentElement.dataset.sceneStatus === 'ready'
      && Boolean(window.robotDemo),
    null,
    { timeout },
  );
  await page.waitForTimeout(1_500);

  const button = page.locator('.assembly-step1-panel button');
  try {
    await button.waitFor({ state: 'visible', timeout: 15_000 });
    const buttonText = (await button.textContent())?.trim();
    if (buttonText !== '执行第一步：协作就位') {
      throw new Error(`Unexpected Step 1 button label: ${buttonText}`);
    }
  } catch (error) {
    console.error('Available buttons:', await page.getByRole('button').allTextContents());
    console.error('Assembly1 status:', await page.evaluate(
      () => document.documentElement.dataset.assemblyStep1Status,
    ));
    throw error;
  }
  const before = await page.evaluate((bodyNames) => ({
    ctrl: window.robotDemo.getCtrl(),
    bodies: window.robotDemo.getBodyPositions(bodyNames),
  }), taskBodies);

  await button.click();
  await page.waitForFunction(
    () => ['complete', 'error'].includes(
      document.documentElement.dataset.assemblyStep1Status ?? '',
    ),
    null,
    { timeout: 60_000 },
  );
  const stepStatus = await page.evaluate(
    () => document.documentElement.dataset.assemblyStep1Status,
  );
  if (stepStatus !== 'complete') {
    throw new Error(failures.join('\n') || `Assembly1 Step 1 ended in ${stepStatus}`);
  }
  await page.waitForTimeout(300);

  const after = await page.evaluate(({ bodyNames, siteNames }) => ({
    ctrl: window.robotDemo.getCtrl(),
    bodies: window.robotDemo.getBodyPositions(bodyNames),
    sites: window.robotDemo.getSitePositions(siteNames),
    orientations: window.robotDemo.getSiteOrientations(siteNames),
  }), { bodyNames: taskBodies, siteNames: Object.keys(tcpTargets) });

  for (let arm = 0; arm < 4; arm += 1) {
    const offset = arm * 8;
    const changedArmJoints = Array.from({ length: 7 }, (_, joint) => offset + joint)
      .filter((index) => Math.abs(after.ctrl[index] - before.ctrl[index]) > 0.05);
    if (changedArmJoints.length < 2) {
      throw new Error(`Arm ${arm + 1} did not execute a visible joint trajectory`);
    }
    if (Math.abs(after.ctrl[offset + 7] - 255) > 1e-6) {
      throw new Error(`Arm ${arm + 1} gripper was not held open`);
    }
  }

  const poseErrors = {};
  for (const [siteName, target] of Object.entries(tcpTargets)) {
    const positionError = distance(after.sites[siteName], target.position);
    const matrix = after.orientations[siteName];
    const closingAxis = [matrix[1], matrix[4], matrix[7]];
    const approachAxis = [matrix[2], matrix[5], matrix[8]];
    const closingAxisErrorDegrees = axisErrorDegrees(closingAxis, target.closingAxis, true);
    const approachAxisErrorDegrees = axisErrorDegrees(approachAxis, [0, 0, -1]);
    poseErrors[siteName] = {
      positionError,
      closingAxisErrorDegrees,
      approachAxisErrorDegrees,
    };
    if (positionError > 0.03) {
      validationFailures.push(
        `${siteName} final error ${positionError.toFixed(4)}m exceeds 0.03m`,
      );
    }
    if (closingAxisErrorDegrees > 8 || approachAxisErrorDegrees > 8) {
      validationFailures.push(
        `${siteName} orientation errors ${closingAxisErrorDegrees.toFixed(2)}°/`
        + `${approachAxisErrorDegrees.toFixed(2)}° exceed 8°`,
      );
    }
  }
  const taskBodyDrifts = {};
  for (const bodyName of taskBodies) {
    const drift = distance(after.bodies[bodyName], before.bodies[bodyName]);
    taskBodyDrifts[bodyName] = drift;
    if (drift > 0.003) {
      validationFailures.push(`${bodyName} drifted ${drift.toFixed(4)}m during staging`);
    }
  }
  if (validationFailures.length > 0) {
    console.error(JSON.stringify({ poseErrors, validationFailures }, null, 2));
    throw new Error(validationFailures.join('\n'));
  }
  if (!(await button.isDisabled())) throw new Error('Completed Step 1 button must stay disabled');

  await page.keyboard.press('v');
  await page.waitForFunction(
    () => Math.abs((window.robotDemo?.getCtrl()[7] ?? 255) - 0) < 1e-6,
    null,
    { timeout: 15_000 },
  );
  await page.keyboard.press('v');
  await page.waitForFunction(
    () => Math.abs((window.robotDemo?.getCtrl()[7] ?? 0) - 255) < 1e-6,
    null,
    { timeout: 15_000 },
  );

  await mkdir(resolve('artifacts/screenshots'), { recursive: true });
  await page.screenshot({ path: screenshotPath, fullPage: true });
  console.log(JSON.stringify({
    status: 'PASS',
    tcpPositions: after.sites,
    tcpPoseErrors: poseErrors,
    taskBodyPositions: after.bodies,
    taskBodyDrifts,
    screenshot: screenshotPath,
  }, null, 2));

  if (failures.length > 0) throw new Error(failures.join('\n'));
} finally {
  await browser.close();
}
