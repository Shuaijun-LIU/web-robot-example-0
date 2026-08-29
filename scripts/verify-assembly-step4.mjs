import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 600_000);
const handoverOnly = process.env.HANDOVER_ONLY === '1';
const diagnoseHandover = process.env.DIAGNOSE_HANDOVER === '1';
const screenshotPath = resolve('artifacts/screenshots/franka-assembly1-step4-fastener-staged.png');

const browser = await chromium.launch({
  headless: true,
  ...(process.env.CHROME_EXECUTABLE
    ? {
      executablePath: process.env.CHROME_EXECUTABLE,
      args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'],
    }
    : {}),
});
const page = await browser.newPage({ viewport: { width: 1200, height: 760 } });
const browserFailures = [];
let beforeStep4 = null;
page.on('pageerror', (error) => browserFailures.push(`page error: ${error.message}`));
page.on('console', (message) => {
  if (message.type() === 'error') browserFailures.push(`console error: ${message.text()}`);
});

if (process.env.ENV_HDR_PATH) {
  await page.route('**/st_fagans_interior_1k.hdr', (route) => route.fulfill({
    path: resolve(process.env.ENV_HDR_PATH),
    contentType: 'image/vnd.radiance',
  }));
}

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

async function runStep(buttons, index, datasetName, diagnosticsName) {
  if (await buttons.nth(index).isDisabled()) {
    throw new Error(`Step ${index + 1} is unexpectedly disabled`);
  }
  await buttons.nth(index).click();
  await page.waitForFunction(
    ({ datasetName }) => ['complete', 'error'].includes(
      document.documentElement.dataset[datasetName] ?? '',
    ),
    { datasetName },
    { timeout },
  );
  const result = await page.evaluate(({ datasetName, diagnosticsName }) => ({
    status: document.documentElement.dataset[datasetName],
    diagnostics: diagnosticsName && window.robotDemo?.[diagnosticsName]
      ? window.robotDemo[diagnosticsName]()
      : null,
  }), { datasetName, diagnosticsName });
  if (result.status !== 'complete') {
    throw new Error(
      `Step ${index + 1} ended in ${result.status}: `
      + JSON.stringify(result.diagnostics ?? null),
    );
  }
  return result.diagnostics;
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
  if (process.env.SCENE_SPEED) {
    const speedInput = page.locator('input[type="text"]').first();
    await speedInput.fill(process.env.SCENE_SPEED);
    await speedInput.press('Enter');
  }
  const buttons = page.locator('.assembly-sequence-panel button');
  await buttons.nth(3).waitFor({ state: 'visible', timeout });
  if (!(await buttons.nth(1).isDisabled())
    || !(await buttons.nth(2).isDisabled())
    || !(await buttons.nth(3).isDisabled())) {
    throw new Error('Later steps must be gated before Step 1 completes');
  }

  await runStep(buttons, 0, 'assemblyStep1Status', null);
  await runStep(buttons, 1, 'assemblyStep2Status', 'getAssemblyStep2Diagnostics');
  await runStep(buttons, 2, 'assemblyStep3Status', 'getAssemblyStep3Diagnostics');

  const before = await page.evaluate(() => {
    const fasteners = window.robotDemo.getBodyPositions([
      'fastener_1',
      'fastener_2',
      'fastener_3',
    ]);
    return {
      fastener: fasteners.fastener_1,
      spareFasteners: {
        fastener_2: fasteners.fastener_2,
        fastener_3: fasteners.fastener_3,
      },
      receiver: window.robotDemo.getSitePositions(['frame_receiver_nw']).frame_receiver_nw,
      drillSupportContacts: window.robotDemo.getContacts().filter(({ body1, body2 }) => (
        (body1 === 'torque_driver' && body2 === 'tool_mat_powered')
        || (body2 === 'torque_driver' && body1 === 'tool_mat_powered')
      )),
    };
  });
  if (before.drillSupportContacts.some(({ distance }) => distance < -0.002)) {
    throw new Error(`Drill support penetration is too deep: ${JSON.stringify(before.drillSupportContacts)}`);
  }
  beforeStep4 = before;
  await page.evaluate((spareFasteners) => {
    window.__assemblyStep4Trace = {
      phases: [],
      phaseSamples: [],
      sawLeftContact: false,
      sawRightContact: false,
      sawHammerLeftContact: false,
      sawHammerRightContact: false,
      maximumFastenerZ: Number.NEGATIVE_INFINITY,
      maximumSpareTranslation: 0,
      arm1Arm2Contacts: [],
    };
    window.__assemblyStep4TraceTimer = window.setInterval(() => {
      const diagnostics = window.robotDemo?.getAssemblyStep4Diagnostics?.();
      const trace = window.__assemblyStep4Trace;
      if (!diagnostics || !trace) return;
      trace.sawLeftContact ||= diagnostics.fastenerLeftContact;
      trace.sawRightContact ||= diagnostics.fastenerRightContact;
      trace.sawHammerLeftContact ||= diagnostics.hammerLeftContact;
      trace.sawHammerRightContact ||= diagnostics.hammerRightContact;
      trace.maximumFastenerZ = Math.max(trace.maximumFastenerZ, diagnostics.fastenerPosition[2]);
      const contacts = window.robotDemo.getContacts();
      for (const contact of contacts) {
        const pair = [contact.body1, contact.body2];
        if (
          pair.some((name) => name.startsWith('r0_'))
          && pair.some((name) => name.startsWith('r1_'))
          && trace.arm1Arm2Contacts.length < 12
        ) {
          trace.arm1Arm2Contacts.push(contact);
        }
      }
      const sparePositions = window.robotDemo.getBodyPositions(['fastener_2', 'fastener_3']);
      for (const name of ['fastener_2', 'fastener_3']) {
        const initial = spareFasteners[name];
        const current = sparePositions[name];
        if (!initial || !current) continue;
        trace.maximumSpareTranslation = Math.max(
          trace.maximumSpareTranslation,
          Math.hypot(...current.map((value, axis) => value - initial[axis])),
        );
      }
      if (trace.phases.at(-1) !== diagnostics.phase) {
        const sites = window.robotDemo.getSitePositions(['r1_tcp', 'r2_tcp', 'r3_tcp']);
        trace.phases.push(diagnostics.phase);
        trace.phaseSamples.push({
          phase: diagnostics.phase,
          donorTcp: sites.r1_tcp,
          tcp: sites.r2_tcp,
          hammerTcp: sites.r3_tcp,
          hammer: window.robotDemo.getBodyPositions(['double_face_hammer']).double_face_hammer,
          fastener: [...diagnostics.fastenerPosition],
          aperture: diagnostics.fastenerAperture,
        });
      }
    }, 20);
  }, before.spareFasteners);

  if (handoverOnly) {
    await buttons.nth(3).click();
    if (diagnoseHandover) {
      await page.waitForFunction(
        () => window.robotDemo?.getAssemblyStep4Diagnostics?.()?.phase
          === 'handover-verification',
        null,
        { timeout },
      );
      const sample = await page.evaluate(() => ({
        diagnostics: window.robotDemo.getAssemblyStep4Diagnostics(),
        hammer: window.robotDemo.getBodyPositions(['double_face_hammer']).double_face_hammer,
        hammerOrientation: window.robotDemo.getBodyOrientations(['double_face_hammer']).double_face_hammer,
        sites: window.robotDemo.getSitePositions(['r1_tcp', 'r3_tcp']),
        fingers: window.robotDemo.getBodyPositions([
          'r1_left_finger',
          'r1_right_finger',
          'r3_left_finger',
          'r3_right_finger',
        ]),
        contacts: window.robotDemo.getContacts().filter(({ body1, body2 }) => (
          body1 === 'double_face_hammer' || body2 === 'double_face_hammer'
        )),
      }));
      console.log(JSON.stringify({ status: 'DIAGNOSTIC', sample }, null, 2));
      process.exitCode = 0;
      await browser.close();
      process.exit();
    }
    await page.waitForFunction(() => {
      const phase = window.robotDemo?.getAssemblyStep4Diagnostics?.()?.phase;
      return phase === 'error' || [
        'donor-clear',
        'fastener-tighten',
        'lift',
        'transfer',
        'transfer-settle',
        'insert',
        'fastener-release',
        'clear',
        'placement-verification',
        'hammer-stage',
        'hammer-strike',
        'hammer-recover',
        'complete',
      ].includes(phase);
    }, null, { timeout });
    // Sample while the receiver is holding the hammer and the donor has just
    // cleared. Waiting into the fastener lift would mix two independent checks.
    await page.waitForTimeout(400);
    const handover = await page.evaluate(() => ({
      diagnostics: window.robotDemo.getAssemblyStep4Diagnostics(),
      trace: window.__assemblyStep4Trace,
      ctrl: window.robotDemo.getCtrl(),
      hammer: window.robotDemo.getBodyPositions(['double_face_hammer']).double_face_hammer,
    }));
    await page.evaluate(() => window.clearInterval(window.__assemblyStep4TraceTimer));
    if (handover.diagnostics?.phase === 'error') {
      throw new Error(`Hammer handover failed: ${JSON.stringify(handover.diagnostics.failure)}`);
    }
    if (!handover.trace?.sawHammerLeftContact || !handover.trace?.sawHammerRightContact) {
      throw new Error(`Bilateral hammer handover contact was not observed: ${JSON.stringify(handover.trace)}`);
    }
    if (!handover.diagnostics?.hammerLeftContact || !handover.diagnostics?.hammerRightContact) {
      throw new Error(`Receiver did not retain bilateral hammer contact: ${JSON.stringify(handover.diagnostics)}`);
    }
    if (handover.trace?.arm1Arm2Contacts?.length > 0) {
      throw new Error(`Arm 2 touched Arm 1: ${JSON.stringify(handover.trace.arm1Arm2Contacts)}`);
    }
    for (const distanceValue of [
      handover.diagnostics.hammerLeftContactDistance,
      handover.diagnostics.hammerRightContactDistance,
    ]) {
      if (typeof distanceValue === 'number' && distanceValue < -0.00215) {
        throw new Error(`Hammer handover penetration is ${distanceValue}m`);
      }
    }
    if (handover.ctrl[15] < 250 || Math.abs(handover.ctrl[31] - 127) > 1e-6) {
      throw new Error(`Hammer gripper transfer is incomplete: ${handover.ctrl[15]}/${handover.ctrl[31]}`);
    }
    if (handover.hammer[2] < 0.30) {
      throw new Error(`Hammer fell during handover: ${handover.hammer}`);
    }
    if (browserFailures.length > 0) throw new Error(browserFailures.join('\n'));
    console.log(JSON.stringify({ status: 'PASS', handover }, null, 2));
  } else {
  const diagnostics = await runStep(
    buttons,
    3,
    'assemblyStep4Status',
    'getAssemblyStep4Diagnostics',
  );
  const result = await page.evaluate(() => {
    window.clearInterval(window.__assemblyStep4TraceTimer);
    return {
      diagnostics: window.robotDemo.getAssemblyStep4Diagnostics(),
      trace: window.__assemblyStep4Trace,
      ctrl: window.robotDemo.getCtrl(),
      fastener: window.robotDemo.getBodyPositions(['fastener_1']).fastener_1,
    };
  });
  if (!diagnostics || result.diagnostics?.phase !== 'complete') {
    throw new Error(`Missing complete Step 4 diagnostics: ${JSON.stringify(result.diagnostics)}`);
  }
  if (!result.trace?.sawLeftContact || !result.trace?.sawRightContact) {
    throw new Error(`Bilateral fastener contact was not observed: ${JSON.stringify(result.trace)}`);
  }
  if (!result.trace.sawHammerLeftContact || !result.trace.sawHammerRightContact) {
    throw new Error(`Bilateral hammer handover contact was not observed: ${JSON.stringify(result.trace)}`);
  }
  if (result.trace.arm1Arm2Contacts.length > 0) {
    throw new Error(`Arm 2 touched Arm 1: ${JSON.stringify(result.trace.arm1Arm2Contacts)}`);
  }
  if (result.trace.maximumSpareTranslation > 0.005) {
    throw new Error(`A spare fastener was disturbed by ${result.trace.maximumSpareTranslation}m`);
  }
  if (result.trace.maximumFastenerZ - before.fastener[2] < 0.12) {
    throw new Error(`Fastener lift was too short: ${JSON.stringify(result.trace)}`);
  }
  if (result.diagnostics.fastenerPlanarDistance > 0.018) {
    throw new Error(`Fastener planar placement failed: ${result.diagnostics.fastenerPlanarDistance}`);
  }
  if (result.diagnostics.fastenerHeight < 0.275 || result.diagnostics.fastenerHeight > 0.34) {
    throw new Error(`Fastener height is invalid: ${result.diagnostics.fastenerHeight}`);
  }
  if (result.diagnostics.frameTranslation > 0.0125) {
    throw new Error(`Assembly stability failed: ${JSON.stringify(result.diagnostics)}`);
  }
  const expectedGrippers = [130, 255, 255, 127];
  for (let arm = 0; arm < expectedGrippers.length; arm += 1) {
    if (Math.abs(result.ctrl[arm * 8 + 7] - expectedGrippers[arm]) > 1e-6) {
      throw new Error(`Arm ${arm + 1} final gripper command is invalid`);
    }
  }

  await page.waitForTimeout(750);
  await mkdir(resolve('artifacts/screenshots'), { recursive: true });
  await page.screenshot({ path: screenshotPath, fullPage: true });
  if (browserFailures.length > 0) throw new Error(browserFailures.join('\n'));
  console.log(JSON.stringify({
    status: 'PASS',
    before,
    diagnostics: result.diagnostics,
    trace: result.trace,
    screenshot: screenshotPath,
  }, null, 2));
  }
} catch (error) {
  const pageState = await page.evaluate(() => ({
    step1: document.documentElement.dataset.assemblyStep1Status,
    step2: document.documentElement.dataset.assemblyStep2Status,
    step3: document.documentElement.dataset.assemblyStep3Status,
    step4: document.documentElement.dataset.assemblyStep4Status,
    panelText: document.querySelector('.assembly-sequence-panel')?.textContent ?? '',
    diagnostics: window.robotDemo?.getAssemblyStep4Diagnostics?.() ?? null,
    geometry: window.robotDemo ? {
      tcp: window.robotDemo.getSitePositions(['r2_tcp']),
      fingers: window.robotDemo.getBodyPositions(['r2_left_finger', 'r2_right_finger']),
      fastener: window.robotDemo.getBodyPositions(['fastener_1']),
      hammerTcp: window.robotDemo.getSitePositions(['r3_tcp']),
      hammerFingers: window.robotDemo.getBodyPositions(['r3_left_finger', 'r3_right_finger']),
      donorTcp: window.robotDemo.getSitePositions(['r1_tcp']),
      donorFingers: window.robotDemo.getBodyPositions(['r1_left_finger', 'r1_right_finger']),
      hammer: window.robotDemo.getBodyPositions(['double_face_hammer']),
    } : null,
    trace: window.__assemblyStep4Trace ?? null,
  })).catch(() => null);
  console.error(JSON.stringify({
    status: 'FAIL',
    error: error instanceof Error ? error.message : String(error),
    browserFailures,
    beforeStep4,
    pageState,
  }, null, 2));
  throw error;
} finally {
  await browser.close();
}
