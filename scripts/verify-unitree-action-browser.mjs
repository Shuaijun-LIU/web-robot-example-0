import { mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const screenshotPath = resolve('artifacts/screenshots/unitree-locomotion-suite.png');
const failures = [];
const useVulkan = process.env.PLAYWRIGHT_USE_VULKAN === '1';
const browserArgs = [
  '--disable-background-timer-throttling',
  '--disable-renderer-backgrounding',
  '--disable-backgrounding-occluded-windows',
  ...(useVulkan
    ? ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist', '--enable-gpu']
    : []),
];
const launchOptions = {
  headless: true,
  args: browserArgs,
  ...(process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE
    ? { executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE }
    : {}),
};

function tiltDegrees(quaternion) {
  const [, x, y] = quaternion;
  const upZ = Math.max(-1, Math.min(1, 1 - 2 * (x * x + y * y)));
  return Math.acos(upZ) * 180 / Math.PI;
}

function jointDelta(before, after, name) {
  return Math.abs(after[name] - before[name]);
}

await mkdir(resolve('artifacts/screenshots'), { recursive: true });
const browser = await chromium.launch(launchOptions);
const page = await browser.newPage({ viewport: { width: 960, height: 600 } });

page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('requestfailed', (request) => {
  failures.push(`request failed: ${request.url()} (${request.failure()?.errorText ?? 'unknown'})`);
});
page.on('console', (message) => {
  const text = message.text();
  if (message.type() === 'error' || /Simulation error|XML Error|Aborted\(|WebGL.*error/i.test(text)) {
    failures.push(`console ${message.type()}: ${text}`);
  }
});

try {
  await page.goto(baseUrl, { waitUntil: 'domcontentloaded', timeout: Math.min(timeout, 120_000) });
  await page.locator('select').first().selectOption({ label: 'Unitree Action Lab' });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneKey === 'unitreeActionLab'
      && document.documentElement.dataset.sceneStatus === 'ready',
    null,
    { timeout },
  );
  const sceneState = await page.evaluate(() => ({ ...document.documentElement.dataset }));
  if (Number(sceneState.sceneInstances) !== 2) {
    throw new Error(`Unitree Action Lab has ${sceneState.sceneInstances ?? 0} instances instead of 2`);
  }
  const selectedTargetLabel = await page.locator('select').nth(1)
    .locator('xpath=following-sibling::div[1]').textContent();
  if (selectedTargetLabel !== 'G1 + Go2 action') {
    throw new Error(`Unitree control target selector shows ${selectedTargetLabel ?? 'nothing'}`);
  }
  await page.waitForFunction(
    () => Boolean(
      window.robotDemo?.runUnitreeAction
      && window.robotDemo?.getUnitreeActionState
      && window.robotDemo?.selectUnitreeActionProgram
      && window.robotDemo?.getUnitreeActionDiagnostics,
    ),
    null,
    { timeout },
  );
  await page.waitForTimeout(1_500);

  const selectedProgram = await page.evaluate(
    () => window.robotDemo.selectUnitreeActionProgram('locomotion'),
  );
  if (!selectedProgram) throw new Error('Could not select the Unitree locomotion program');
  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionProgram === 'locomotion',
    null,
    { timeout },
  );

  const jointNames = [
    'g1_left_knee_joint',
    'g1_right_shoulder_roll_joint',
    'go2_FL_thigh_joint',
    'go2_joint1',
  ];
  const home = await page.evaluate((names) => ({
    joints: window.robotDemo.getJointPositions(names),
    roots: window.robotDemo.getBodyPositions(['g1_pelvis', 'go2_base']),
  }), jointNames);

  const started = await page.evaluate(() => window.robotDemo.runUnitreeAction());
  if (!started) throw new Error('runUnitreeAction() rejected a ready idle action');
  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionPhase === 'g1-walk'
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const gaitState = await page.evaluate(() => window.robotDemo.getUnitreeActionState());
  if (gaitState.status === 'error') throw new Error(`Unitree action failed: ${gaitState.error}`);
  await page.waitForFunction(
    () => window.robotDemo.getUnitreeActionState().elapsed >= 7.5
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const g1Walk = await page.evaluate((names) => ({
    joints: window.robotDemo.getJointPositions(names),
    state: window.robotDemo.getUnitreeActionState(),
    diagnostics: window.robotDemo.getUnitreeActionDiagnostics(),
  }), jointNames);
  if (g1Walk.state.status === 'error') throw new Error(`Unitree action failed: ${g1Walk.state.error}`);
  const midTargetLabel = await page.locator('select').nth(1)
    .locator('xpath=following-sibling::div[1]').textContent();
  if (midTargetLabel !== 'G1 + Go2 action') {
    throw new Error(`Unitree control target changed during action to ${midTargetLabel ?? 'nothing'}`);
  }
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.waitForTimeout(500);
  const screenshotTargetLabel = await page.locator('select').nth(1)
    .locator('xpath=following-sibling::div[1]').textContent();
  if (screenshotTargetLabel !== 'G1 + Go2 action') {
    throw new Error(`Unitree control target changed before screenshot to ${screenshotTargetLabel ?? 'nothing'}`);
  }
  await page.screenshot({ path: screenshotPath, fullPage: true });
  await page.setViewportSize({ width: 960, height: 600 });

  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionPhase === 'go2-walk'
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  await page.waitForFunction(
    () => window.robotDemo.getUnitreeActionState().elapsed >= 15.35
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const go2Walk = await page.evaluate((names) => ({
    joints: window.robotDemo.getJointPositions(names),
    state: window.robotDemo.getUnitreeActionState(),
    diagnostics: window.robotDemo.getUnitreeActionDiagnostics(),
  }), jointNames);
  if (go2Walk.state.status === 'error') throw new Error(`Unitree action failed: ${go2Walk.state.error}`);

  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionPhase === 'final-greeting'
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  await page.waitForFunction(
    () => window.robotDemo.getUnitreeActionState().elapsed >= 22.5
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const greeting = await page.evaluate((names) => ({
    joints: window.robotDemo.getJointPositions(names),
    state: window.robotDemo.getUnitreeActionState(),
  }), jointNames);
  if (greeting.state.status === 'error') throw new Error(`Unitree action failed: ${greeting.state.error}`);

  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionStatus === 'complete'
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const completionState = await page.evaluate(() => window.robotDemo.getUnitreeActionState());
  if (completionState.status === 'error') throw new Error(`Unitree action failed: ${completionState.error}`);
  await page.waitForTimeout(750);
  const final = await page.evaluate(() => ({
    state: window.robotDemo.getUnitreeActionState(),
    roots: window.robotDemo.getBodyPositions(['g1_pelvis', 'go2_base']),
    orientations: window.robotDemo.getBodyOrientations(['g1_pelvis', 'go2_base']),
    contacts: window.robotDemo.getContacts(),
    qpos: window.robotDemo.getQpos(),
    diagnostics: window.robotDemo.getUnitreeActionDiagnostics(),
  }));

  const g1LegDelta = jointDelta(home.joints, g1Walk.joints, jointNames[0]);
  const g1GreetingDelta = jointDelta(home.joints, greeting.joints, jointNames[1]);
  const go2LegDelta = jointDelta(home.joints, go2Walk.joints, jointNames[2]);
  const airbotDelta = jointDelta(home.joints, greeting.joints, jointNames[3]);
  if (g1LegDelta <= 0.15) throw new Error(`G1 leg moved only ${g1LegDelta}`);
  if (g1GreetingDelta <= 0.4) throw new Error(`G1 greeting arm moved only ${g1GreetingDelta}`);
  if (go2LegDelta <= 0.05) throw new Error(`Go2 leg moved only ${go2LegDelta}`);
  if (airbotDelta <= 0.2) throw new Error(`Airbot joint moved only ${airbotDelta}`);
  if (!final.qpos.every(Number.isFinite)) throw new Error('Browser MuJoCo state is not finite');
  if (!final.diagnostics) throw new Error('Unitree runtime diagnostics are unavailable');
  if (!final.diagnostics.safe) {
    throw new Error(`Unitree runtime diagnostics are unsafe: ${final.diagnostics.safetyReason}`);
  }
  if (final.diagnostics.clampCount !== 0) {
    throw new Error(`Unitree targets were clamped ${final.diagnostics.clampCount} times`);
  }
  if (final.diagnostics.displacement.g1.x < 0.05) {
    throw new Error(`G1 advanced only ${final.diagnostics.displacement.g1.x} m`);
  }
  if (final.diagnostics.displacement.go2.x < 0.4) {
    throw new Error(`Go2 advanced only ${final.diagnostics.displacement.go2.x} m`);
  }

  const g1Height = final.roots.g1_pelvis[2];
  const go2Height = final.roots.go2_base[2];
  const g1Tilt = tiltDegrees(final.orientations.g1_pelvis);
  const go2Tilt = tiltDegrees(final.orientations.go2_base);
  if (g1Height < 0.75 || g1Height > 0.85 || g1Tilt > 5) {
    throw new Error(`G1 final root is unstable: height=${g1Height}, tilt=${g1Tilt}`);
  }
  if (go2Height < 0.22 || go2Height > 0.34 || go2Tilt > 10) {
    throw new Error(`Go2 final root is unstable: height=${go2Height}, tilt=${go2Tilt}`);
  }
  const g1Contacts = final.contacts.filter(({ body1, body2 }) => body1.startsWith('g1_') || body2.startsWith('g1_')).length;
  const go2Contacts = final.contacts.filter(({ body1, body2 }) => body1.startsWith('go2_') || body2.startsWith('go2_')).length;
  if (g1Contacts === 0 || go2Contacts === 0) {
    throw new Error(`Missing final ground contacts: G1=${g1Contacts}, Go2=${go2Contacts}`);
  }
  if (failures.length > 0) throw new Error(failures.join('\n'));

  console.log(JSON.stringify({
    completed: final.state.status === 'complete',
    elapsed: final.state.elapsed,
    phases: [g1Walk.state.phase, go2Walk.state.phase, greeting.state.phase],
    jointMotion: { g1LegDelta, g1GreetingDelta, go2LegDelta, airbotDelta },
    g1: {
      forwardDisplacement: final.diagnostics.displacement.g1.x,
      finalHeight: g1Height,
      finalTiltDegrees: g1Tilt,
      contacts: g1Contacts,
    },
    go2: {
      forwardDisplacement: final.diagnostics.displacement.go2.x,
      finalHeight: go2Height,
      finalTiltDegrees: go2Tilt,
      contacts: go2Contacts,
    },
    initialRoots: home.roots,
    screenshot: screenshotPath,
  }));
} finally {
  await browser.close();
}
