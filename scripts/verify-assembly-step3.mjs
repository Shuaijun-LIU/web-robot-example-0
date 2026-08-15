import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 300_000);
const screenshotPath = resolve('artifacts/screenshots/franka-assembly1-step3-aligned-hold.png');
const gripperClampControls = [48, 96, 24, 24];

function distance(first, second) {
  return Math.hypot(...first.map((value, index) => value - second[index]));
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
    { timeout: 90_000 },
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
    { timeout: 120_000 },
  );
  const step2Diagnostics = await page.evaluate(
    () => window.robotDemo.getAssemblyStep2Diagnostics(),
  );
  if (!step2Diagnostics || step2Diagnostics.phase !== 'complete') {
    throw new Error(
      `Step 2 ended in ${step2Diagnostics?.phase ?? 'missing diagnostics'}: `
      + JSON.stringify(step2Diagnostics?.failure ?? null),
    );
  }
  if (await buttons.nth(2).isDisabled()) throw new Error('Step 3 did not unlock');

  const before = await page.evaluate(() => ({
    ctrl: window.robotDemo.getCtrl(),
    positions: window.robotDemo.getBodyPositions(['assembly_frame', 'cross_member']),
  }));
  for (let arm = 0; arm < 4; arm += 1) {
    const control = before.ctrl[arm * 8 + 7];
    if (Math.abs(control - gripperClampControls[arm]) > 1e-6) {
      throw new Error(`Arm ${arm + 1} lost its Step 2 clamp before transport`);
    }
  }

  await page.evaluate(() => {
    const initial = window.robotDemo.getBodyPositions(['cross_member']).cross_member;
    const initialSites = window.robotDemo.getSitePositions(['r2_tcp', 'r3_tcp']);
    window.__assemblyStep3Trace = {
      initial,
      initialSites,
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
      trace.maximumZ = Math.max(trace.maximumZ, position[2]);
      trace.maximumPlanarTravel = Math.max(
        trace.maximumPlanarTravel,
        Math.hypot(position[0] - trace.initial[0], position[1] - trace.initial[1]),
      );
      const sites = window.robotDemo.getSitePositions(['r2_tcp', 'r3_tcp']);
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
    { timeout: 120_000 },
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
      positions: window.robotDemo.getBodyPositions(['assembly_frame', 'cross_member']),
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
  if (result.trace.maximumPlanarTravel < 0.35) {
    throw new Error(`Cross-member transfer was too short: ${result.trace.maximumPlanarTravel}m`);
  }
  if (finalDiagnostics.frameTranslation > 0.008) {
    throw new Error(`Frame drifted ${finalDiagnostics.frameTranslation}m during Step 3`);
  }
  if (finalDiagnostics.crossMemberRotationDegrees > 5) {
    throw new Error(
      `Cross-member rotated ${finalDiagnostics.crossMemberRotationDegrees}deg during Step 3`,
    );
  }
  if (finalDiagnostics.holeDistances.length !== 4
    || finalDiagnostics.holeDistances.some((value) => value > 0.008)) {
    throw new Error(`Hole alignment failed: ${JSON.stringify(finalDiagnostics.holeDistances)}`);
  }
  if (finalDiagnostics.arms.some((arm) => !arm.verdict.ok)) {
    throw new Error(`A retained grasp is invalid: ${JSON.stringify(finalDiagnostics.arms)}`);
  }
  for (let arm = 0; arm < 4; arm += 1) {
    if (Math.abs(result.ctrl[arm * 8 + 7] - gripperClampControls[arm]) > 1e-6) {
      throw new Error(`Arm ${arm + 1} did not retain its gripper command`);
    }
  }
  const finalBodyOffset = distance(result.positions.cross_member, [0, 0, 0.278]);
  if (finalBodyOffset > 0.015) {
    throw new Error(`Cross-member body is ${finalBodyOffset}m from its installed pose`);
  }

  await page.waitForTimeout(1_000);
  const held = await page.evaluate(() => window.robotDemo.getAssemblyStep3Diagnostics());
  if (held?.phase !== 'complete'
    || held.holeDistances.some((value) => value > 0.008)
    || held.frameTranslation > 0.008) {
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
