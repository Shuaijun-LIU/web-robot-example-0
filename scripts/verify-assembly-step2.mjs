import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const screenshotPath = resolve('artifacts/screenshots/franka-assembly1-step2-physical-clamp.png');
const gripperClampControls = [48, 96, 0, 0];
const taskBodies = ['assembly_frame', 'torque_driver', 'cross_member'];

function distance(first, second) {
  return Math.hypot(...first.map((value, index) => value - second[index]));
}

function quaternionAngleDegrees(first, second) {
  const denominator = Math.hypot(...first) * Math.hypot(...second);
  const dot = Math.abs(first.reduce(
    (sum, value, index) => sum + value * second[index],
    0,
  ) / denominator);
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
}

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 800, height: 500 } });
const browserFailures = [];
page.on('pageerror', (error) => browserFailures.push(`page error: ${error.message}`));
page.on('console', (message) => {
  if (message.type() === 'error') browserFailures.push(`console error: ${message.text()}`);
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

let finalDiagnostics = null;
try {
  await page.goto(baseUrl, {
    waitUntil: 'domcontentloaded',
    timeout: Math.min(timeout, 120_000),
  });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneKey === 'frankaAssembly1'
      && document.documentElement.dataset.sceneStatus === 'ready'
      && Boolean(window.robotDemo),
    null,
    { timeout },
  );
  const buttons = page.locator('.assembly-sequence-panel button');
  await buttons.nth(1).waitFor({ state: 'visible', timeout: 15_000 });
  if (!(await buttons.nth(1).isDisabled())) {
    throw new Error('Step 2 must be disabled before Step 1 completes');
  }

  await buttons.nth(0).click();
  await page.waitForFunction(
    () => ['complete', 'error'].includes(
      document.documentElement.dataset.assemblyStep1Status ?? '',
    ),
    null,
    { timeout: 180_000 },
  );
  const step1Status = await page.evaluate(
    () => document.documentElement.dataset.assemblyStep1Status,
  );
  if (step1Status !== 'complete') throw new Error(`Step 1 ended in ${step1Status}`);
  if (await buttons.nth(1).isDisabled()) {
    throw new Error('Step 2 did not become enabled after Step 1');
  }

  const before = await page.evaluate((bodyNames) => ({
    ctrl: window.robotDemo.getCtrl(),
    positions: window.robotDemo.getBodyPositions(bodyNames),
    orientations: window.robotDemo.getBodyOrientations(bodyNames),
  }), taskBodies);
  for (const control of [7, 15, 23, 31]) {
    if (Math.abs(before.ctrl[control] - 255) > 1e-6) {
      throw new Error(`Gripper control ${control} is not open after Step 1`);
    }
  }

  await buttons.nth(1).click();
  await page.waitForFunction(
    () => ['complete', 'error'].includes(
      document.documentElement.dataset.assemblyStep2Status ?? '',
    ),
    null,
    { timeout: 180_000 },
  );
  finalDiagnostics = await page.evaluate(
    () => window.robotDemo.getAssemblyStep2Diagnostics(),
  );
  if (!finalDiagnostics || finalDiagnostics.phase !== 'complete') {
    throw new Error(
      `Step 2 ended in ${finalDiagnostics?.phase ?? 'missing diagnostics'}: `
      + JSON.stringify(finalDiagnostics?.failure ?? null),
    );
  }

  const after = await page.evaluate((bodyNames) => ({
    ctrl: window.robotDemo.getCtrl(),
    positions: window.robotDemo.getBodyPositions(bodyNames),
    orientations: window.robotDemo.getBodyOrientations(bodyNames),
  }), taskBodies);
  for (let arm = 0; arm < 4; arm += 1) {
    const changedJoints = Array.from({ length: 7 }, (_, joint) => arm * 8 + joint)
      .filter((control) => Math.abs(after.ctrl[control] - before.ctrl[control]) > 0.01);
    if (changedJoints.length < 2) throw new Error(`Arm ${arm + 1} did not visibly descend`);
    if (Math.abs(after.ctrl[arm * 8 + 7] - gripperClampControls[arm]) > 1e-6) {
      throw new Error(
        `Arm ${arm + 1} gripper did not finish at clamp command ${gripperClampControls[arm]}`,
      );
    }
  }

  const crossClosureDelta = Math.abs(
    finalDiagnostics.arms[2].closureStartedAt - finalDiagnostics.arms[3].closureStartedAt,
  );
  if (crossClosureDelta > 0.0021) {
    throw new Error(`Arm 3/4 closure start delta ${crossClosureDelta}s exceeds one timestep`);
  }
  for (const arm of finalDiagnostics.arms) {
    if (!arm.verdict.ok) throw new Error(`${arm.armKey} final grasp is invalid`);
    if (arm.maximumContactSeconds < 0.25) {
      throw new Error(`${arm.armKey} contact window ${arm.maximumContactSeconds}s is too short`);
    }
    if (!(arm.aperture > 0.02)) throw new Error(`${arm.armKey} aperture is ${arm.aperture}m`);
    if (arm.translation > 0.005) throw new Error(`${arm.armKey} drift is ${arm.translation}m`);
    if (arm.rotationDegrees > 5) throw new Error(`${arm.armKey} rotation is ${arm.rotationDegrees}deg`);
    if (arm.verticalDisplacement > 0.003) {
      throw new Error(`${arm.armKey} vertical displacement is ${arm.verticalDisplacement}m`);
    }
  }
  for (const body of taskBodies) {
    const translation = distance(after.positions[body], before.positions[body]);
    const rotation = quaternionAngleDegrees(after.orientations[body], before.orientations[body]);
    if (translation > 0.005 || rotation > 5) {
      throw new Error(`${body} production pose changed ${translation}m/${rotation}deg`);
    }
  }

  await page.waitForTimeout(2_000);
  const held = await page.evaluate(() => window.robotDemo.getAssemblyStep2Diagnostics());
  if (held?.phase !== 'complete' || held.arms.some((arm) => !arm.verdict.ok)) {
    throw new Error(`Clamped hold did not remain stable: ${JSON.stringify(held)}`);
  }
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.waitForTimeout(300);
  await mkdir(resolve('artifacts/screenshots'), { recursive: true });
  await page.screenshot({ path: screenshotPath, fullPage: true });

  await page.evaluate(() => window.robotDemo.reset());
  await page.waitForFunction(
    () => document.documentElement.dataset.assemblyStep1Status === 'idle'
      && document.documentElement.dataset.assemblyStep2Status === 'idle',
    null,
    { timeout: 15_000 },
  );
  if (!(await buttons.nth(1).isDisabled())) throw new Error('Reset did not disable Step 2');

  if (browserFailures.length > 0) throw new Error(browserFailures.join('\n'));
  console.log(JSON.stringify({
    status: 'PASS',
    diagnostics: held,
    screenshot: screenshotPath,
  }, null, 2));
} catch (error) {
  const pageState = await page.evaluate(() => ({
    step1Status: document.documentElement.dataset.assemblyStep1Status,
    step2Status: document.documentElement.dataset.assemblyStep2Status,
    panelText: document.querySelector('.assembly-sequence-panel')?.textContent ?? '',
    diagnostics: window.robotDemo?.getAssemblyStep2Diagnostics?.() ?? null,
  })).catch(() => null);
  console.error(JSON.stringify({
    status: 'FAIL',
    error: error instanceof Error ? error.message : String(error),
    diagnostics: finalDiagnostics,
    browserFailures,
    pageState,
  }, null, 2));
  throw error;
} finally {
  await browser.close();
}
