import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 300_000);
const screenshotPath = resolve('artifacts/screenshots/franka-assembly1-step3-released.png');
const step2GripperClampControls = [130, 130, 135, 130];
const finalGripperControls = [130, 130, 255, 255];

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

const browser = await chromium.launch({
  headless: true,
  ...(process.env.CHROME_EXECUTABLE
    ? {
      executablePath: process.env.CHROME_EXECUTABLE,
      args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'],
    }
    : {}),
});
const page = await browser.newPage({ viewport: { width: 900, height: 600 } });
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
  if (process.env.SCENE_SPEED) {
    const speedInput = page.locator('input[type="text"]').first();
    await speedInput.fill(process.env.SCENE_SPEED);
    await speedInput.press('Enter');
  }

  const buttons = page.locator('.assembly-sequence-panel button');
  await buttons.nth(2).waitFor({ state: 'visible', timeout: 15_000 });
  if (!(await buttons.nth(1).isDisabled()) || !(await buttons.nth(2).isDisabled())) {
    throw new Error('Steps 2/3 must be gated before Step 1 completes');
  }

  await buttons.nth(0).click();
  await page.waitForFunction(
    () => ['complete', 'error'].includes(
      document.documentElement.dataset.assemblyStep1Status ?? '',
    ),
    null,
    { timeout },
  );
  const step1Status = await page.evaluate(
    () => document.documentElement.dataset.assemblyStep1Status,
  );
  if (step1Status !== 'complete') throw new Error(`Step 1 ended in ${step1Status}`);
  if (await buttons.nth(1).isDisabled()) throw new Error('Step 2 did not unlock');

  await buttons.nth(1).click();
  await page.waitForFunction(
    () => ['complete', 'error'].includes(
      document.documentElement.dataset.assemblyStep2Status ?? '',
    ),
    null,
    { timeout },
  );
  const step2Diagnostics = await page.evaluate(
    () => window.robotDemo.getAssemblyStep2Diagnostics(),
  );
  if (!step2Diagnostics || step2Diagnostics.phase !== 'complete') {
    const failureContacts = await page.evaluate(() => window.robotDemo.getContacts().filter(
      ({ body1, body2 }) => /^r[0-3]_(?:left|right)_finger$/.test(body1)
        || /^r[0-3]_(?:left|right)_finger$/.test(body2),
    ));
    throw new Error(
      `Step 2 ended in ${step2Diagnostics?.phase ?? 'missing diagnostics'}: `
      + JSON.stringify({
        failure: step2Diagnostics?.failure ?? null,
        arms: step2Diagnostics?.arms ?? null,
        contacts: failureContacts,
      }),
    );
  }
  if (await buttons.nth(2).isDisabled()) throw new Error('Step 3 did not unlock');

  const before = await page.evaluate(() => ({
    ctrl: window.robotDemo.getCtrl(),
    positions: window.robotDemo.getBodyPositions(['assembly_frame', 'cross_member', 'double_face_hammer']),
    orientations: window.robotDemo.getBodyOrientations(['double_face_hammer']),
  }));
  for (let arm = 0; arm < 4; arm += 1) {
    const control = before.ctrl[arm * 8 + 7];
    if (Math.abs(control - step2GripperClampControls[arm]) > 1e-6) {
      throw new Error(`Arm ${arm + 1} lost its Step 2 clamp before transport`);
    }
  }

  await page.evaluate(() => {
    const bodies = window.robotDemo.getBodyPositions(['cross_member', 'double_face_hammer']);
    const initial = bodies.cross_member;
    const initialHammer = bodies.double_face_hammer;
    const initialSites = window.robotDemo.getSitePositions(['r1_tcp', 'r2_tcp', 'r3_tcp']);
    window.__assemblyStep3Trace = {
      initial,
      initialSites,
      initialHammer,
      maximumHammerZ: initialHammer[2],
      maximumZ: initial[2],
      maximumPlanarTravel: 0,
      minimumTcpSpan: Number.POSITIVE_INFINITY,
      maximumTcpSpan: 0,
      lastSites: initialSites,
      lastCtrl: window.robotDemo.getCtrl(),
      lastQpos: window.robotDemo.getQpos(),
      lastQvel: window.robotDemo.getQvel(),
      lastContacts: window.robotDemo.getContacts(),
      maximumAbsoluteQvel: 0,
      phases: [],
    };
    window.__assemblyStep3TraceTimer = window.setInterval(() => {
      const trace = window.__assemblyStep3Trace;
      if (!trace || !window.robotDemo) return;
      const position = window.robotDemo.getBodyPositions(['cross_member']).cross_member;
      const hammer = window.robotDemo.getBodyPositions(['double_face_hammer']).double_face_hammer;
      trace.maximumZ = Math.max(trace.maximumZ, position[2]);
      trace.maximumHammerZ = Math.max(trace.maximumHammerZ, hammer[2]);
      trace.maximumPlanarTravel = Math.max(
        trace.maximumPlanarTravel,
        Math.hypot(position[0] - trace.initial[0], position[1] - trace.initial[1]),
      );
      const sites = window.robotDemo.getSitePositions(['r1_tcp', 'r2_tcp', 'r3_tcp']);
      const tcpSpan = Math.hypot(
        ...sites.r2_tcp.map((value, index) => value - sites.r3_tcp[index]),
      );
      trace.minimumTcpSpan = Math.min(trace.minimumTcpSpan, tcpSpan);
      trace.maximumTcpSpan = Math.max(trace.maximumTcpSpan, tcpSpan);
      trace.lastSites = sites;
      trace.lastCtrl = window.robotDemo.getCtrl();
      trace.lastQpos = window.robotDemo.getQpos();
      trace.lastQvel = window.robotDemo.getQvel();
      trace.lastContacts = window.robotDemo.getContacts();
      trace.maximumAbsoluteQvel = Math.max(
        trace.maximumAbsoluteQvel,
        ...trace.lastQvel.map(Math.abs),
      );
      const phase = document.documentElement.dataset.assemblyStep3Status ?? '';
      if (phase && trace.phases.at(-1) !== phase) trace.phases.push(phase);
    }, 20);
  });

  await buttons.nth(2).click();
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneStatus === 'error'
      || ['complete', 'error'].includes(
        document.documentElement.dataset.assemblyStep3Status ?? '',
      ),
    null,
    { timeout },
  );
  const sceneStatusAfterStep3 = await page.evaluate(
    () => document.documentElement.dataset.sceneStatus,
  );
  if (sceneStatusAfterStep3 === 'error') {
    throw new Error('Scene crashed while executing Step 3');
  }
  const result = await page.evaluate(() => {
    window.clearInterval(window.__assemblyStep3TraceTimer);
    return {
      diagnostics: window.robotDemo.getAssemblyStep3Diagnostics(),
      trace: window.__assemblyStep3Trace,
      ctrl: window.robotDemo.getCtrl(),
      positions: window.robotDemo.getBodyPositions(['assembly_frame', 'cross_member', 'double_face_hammer']),
      orientations: window.robotDemo.getBodyOrientations(['double_face_hammer']),
    };
  });
  finalDiagnostics = result.diagnostics;
  if (!finalDiagnostics || finalDiagnostics.phase !== 'complete') {
    throw new Error(
      `Step 3 ended in ${finalDiagnostics?.phase ?? 'missing diagnostics'}: `
      + JSON.stringify(finalDiagnostics?.failure ?? null),
    );
  }
  if (!result.trace || result.trace.maximumZ - result.trace.initial[2] < 0.08) {
    throw new Error(`Cross-member lift was not observed: ${JSON.stringify(result.trace)}`);
  }
  // Step 3 now uses the compact 60 mm TCP staging lift; the grasp offset makes
  // the hammer body's measured rise about 48 mm.  Require a real 40 mm lift.
  if (result.trace.maximumHammerZ - result.trace.initialHammer[2] < 0.04) {
    throw new Error(`Hammer lift was not observed: ${JSON.stringify(result.trace)}`);
  }
  if (result.trace.maximumPlanarTravel < 0.35) {
    throw new Error(`Cross-member transfer was too short: ${result.trace.maximumPlanarTravel}m`);
  }
  if (finalDiagnostics.frameTranslation > 0.02) {
    throw new Error(`Frame drifted ${finalDiagnostics.frameTranslation}m during Step 3`);
  }
  if (finalDiagnostics.holePlanarDistances.length !== 4
    || finalDiagnostics.holePlanarDistances.some((value) => value > 0.041)
    || finalDiagnostics.holeVerticalOffsets.some((value) => value > 0.02)) {
    throw new Error(`Hole alignment failed: ${JSON.stringify({
      planar: finalDiagnostics.holePlanarDistances,
      vertical: finalDiagnostics.holeVerticalOffsets,
    })}`);
  }
  if (finalDiagnostics.arms.some((arm) => !arm.verdict.ok)) {
    throw new Error(`A retained grasp is invalid: ${JSON.stringify(finalDiagnostics.arms)}`);
  }
  for (const arm of finalDiagnostics.arms) {
    for (const contactDistance of [
      arm.leftTargetContactDistance,
      arm.rightTargetContactDistance,
    ]) {
      if (typeof contactDistance === 'number' && contactDistance < -0.00215) {
        throw new Error(`${arm.armKey} penetration is ${contactDistance}m`);
      }
    }
  }
  for (let arm = 0; arm < 4; arm += 1) {
    if (Math.abs(result.ctrl[arm * 8 + 7] - finalGripperControls[arm]) > 1e-6) {
      throw new Error(`Arm ${arm + 1} did not reach its final gripper command`);
    }
  }
  const home = [1.707, -1.754, 0.003, -2.702, 0.003, 0.951, 2.49];
  for (const arm of [2, 3]) {
    const target = result.ctrl.slice(arm * 8, arm * 8 + 7);
    if (target.some((value, joint) => Math.abs(value - home[joint]) > 1e-6)) {
      throw new Error(`Arm ${arm + 1} did not return to its initial joint target`);
    }
  }
  const finalBodyOffset = distance(result.positions.cross_member, [0, 0, 0.278]);
  if (finalBodyOffset > 0.03) {
    throw new Error(`Cross-member body is ${finalBodyOffset}m from its installed pose`);
  }
  const finalHammer = result.positions.double_face_hammer;
  // Step 3 now keeps the hammer at a compact pickup-side staging pose so Arm 4
  // can receive it in Step 4.  The old 360 mm floor belonged to the retired
  // high handover pose and rejected a valid, physically retained 48 mm lift.
  if (finalHammer[0] < 0.55 || finalHammer[0] > 0.78
    || finalHammer[1] < -0.50 || finalHammer[1] > -0.30
    || finalHammer[2] < 0.175) {
    throw new Error(`Hammer did not reach the collision-free pickup-side staging pose: ${finalHammer}`);
  }
  const hammerRotation = quaternionAngleDegrees(
    before.orientations.double_face_hammer,
    result.orientations.double_face_hammer,
  );
  if (hammerRotation > 30) {
    throw new Error(`Hammer rotated ${hammerRotation} degrees during transport`);
  }

  await page.waitForTimeout(1_000);
  const held = await page.evaluate(() => window.robotDemo.getAssemblyStep3Diagnostics());
  if (held?.phase !== 'complete'
    || held.holePlanarDistances.some((value) => value > 0.041)
    || held.holeVerticalOffsets.some((value) => value > 0.02)
    || held.frameTranslation > 0.02
    || !held.arms.find((arm) => arm.armKey === 'r1')?.verdict.ok) {
    throw new Error(`Aligned hold did not remain stable: ${JSON.stringify(held)}`);
  }

  await page.setViewportSize({ width: 1440, height: 900 });
  await page.waitForTimeout(300);
  await mkdir(resolve('artifacts/screenshots'), { recursive: true });
  await page.screenshot({ path: screenshotPath, fullPage: true });

  await page.evaluate(() => window.robotDemo.reset());
  await page.waitForFunction(
    () => document.documentElement.dataset.assemblyStep1Status === 'idle'
      && document.documentElement.dataset.assemblyStep2Status === 'idle'
      && document.documentElement.dataset.assemblyStep3Status === 'idle',
    null,
    { timeout: 15_000 },
  );
  if (!(await buttons.nth(1).isDisabled()) || !(await buttons.nth(2).isDisabled())) {
    throw new Error('Reset did not restore sequence gating');
  }

  if (browserFailures.length > 0) throw new Error(browserFailures.join('\n'));
  console.log(JSON.stringify({
    status: 'PASS',
    diagnostics: held,
    trace: result.trace,
    screenshot: screenshotPath,
  }, null, 2));
} catch (error) {
  const pageState = await page.evaluate(() => ({
    step1Status: document.documentElement.dataset.assemblyStep1Status,
    step2Status: document.documentElement.dataset.assemblyStep2Status,
    step3Status: document.documentElement.dataset.assemblyStep3Status,
    panelText: document.querySelector('.assembly-sequence-panel')?.textContent ?? '',
    diagnostics: window.robotDemo?.getAssemblyStep3Diagnostics?.() ?? null,
    trace: window.__assemblyStep3Trace ?? null,
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
