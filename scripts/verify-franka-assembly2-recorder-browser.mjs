import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const expectedArms = [
  { key: 'r0', label: 'Arm 1' },
  { key: 'r1', label: 'Arm 2' },
  { key: 'r2', label: 'Arm 3' },
  { key: 'r3', label: 'Arm 4' },
];
const failures = [];

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });

page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('requestfailed', (request) => {
  failures.push(`request failed: ${request.url()} (${request.failure()?.errorText ?? 'unknown'})`);
});
page.on('console', (message) => {
  if (message.type() === 'error') failures.push(`console error: ${message.text()}`);
});

try {
  await page.goto(baseUrl, {
    waitUntil: 'domcontentloaded',
    timeout: Math.min(timeout, 120_000),
  });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneStatus === 'ready',
    null,
    { timeout },
  );

  const sceneSelector = page.locator('select').nth(0);
  const targetSelector = page.locator('select').nth(1);
  await sceneSelector.selectOption({ label: 'Franka Assembly2' });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneKey === 'frankaAssembly2'
      && document.documentElement.dataset.sceneStatus === 'ready',
    null,
    { timeout },
  );

  const panel = page.getByRole('region', { name: 'Franka Assembly2 示教采集' });
  await panel.waitFor({ state: 'visible', timeout });
  const availableArms = panel.locator('.franka-recorder-panel__line').filter({
    hasText: '可采集机械臂：',
  });
  await availableArms.waitFor({ state: 'visible', timeout });

  const expectedSummary = `可采集机械臂：${expectedArms.map(({ label }) => label).join(' / ')}`;
  const actualSummary = (await availableArms.innerText()).trim();
  if (actualSummary !== expectedSummary) {
    throw new Error(`recorder arm summary is "${actualSummary}", expected "${expectedSummary}"`);
  }

  for (const { key, label } of expectedArms) {
    await targetSelector.selectOption({ label });
    await page.waitForFunction(
      (expectedKey) => document.documentElement.dataset.controlTarget === expectedKey,
      key,
      { timeout },
    );

    const checkboxLabel = panel.locator(`input[type="checkbox"][value="${key}"]`).locator('..');
    const recorderLabel = (await checkboxLabel.innerText()).trim();
    if (recorderLabel !== label) {
      throw new Error(`${key} is labeled "${recorderLabel}" in the recorder, expected "${label}"`);
    }
  }

  if (failures.length > 0) throw new Error(failures.join('\n'));
  console.log('Franka Assembly2 recorder labels: r0→Arm 1, r1→Arm 2, r2→Arm 3, r3→Arm 4');
} finally {
  await browser.close();
}
