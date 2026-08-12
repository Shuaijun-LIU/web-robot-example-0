import { access, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const sceneTimeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const outputDirectory = resolve('artifacts/screenshots');
const allScenes = [
  { key: 'franka', label: 'Franka Panda', instances: 4 },
  { key: 'so101', label: 'SO101', instances: 4 },
  { key: 'xlerobot', label: 'XLeRobot', instances: 2 },
  {
    key: 'frankaAssembly1',
    label: 'Franka Assembly1',
    instances: 4,
    screenshotName: 'franka-assembly1',
  },
  {
    key: 'frankaAssembly2',
    label: 'Franka Assembly2',
    instances: 4,
    screenshotName: 'franka-assembly2',
  },
];
const requestedKeys = new Set(
  (process.env.SCENES ?? allScenes.map(({ key }) => key).join(','))
    .split(',')
    .map((key) => key.trim()),
);
const scenes = allScenes.filter(({ key }) => requestedKeys.has(key));

await mkdir(outputDirectory, { recursive: true });

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
const failures = [];
const pendingRequests = new Map();

page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('request', (request) => pendingRequests.set(request.url(), Date.now()));
page.on('requestfinished', (request) => pendingRequests.delete(request.url()));
page.on('requestfailed', (request) => {
  failures.push(`request failed: ${request.url()} (${request.failure()?.errorText ?? 'unknown'})`);
  pendingRequests.delete(request.url());
});
page.on('console', (message) => {
  const text = message.text();
  if (message.type() === 'error' || /Simulation error|XML Error|Aborted\(/i.test(text)) {
    failures.push(`console ${message.type()}: ${text}`);
  }
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

try {
  await page.goto(baseUrl, { waitUntil: 'domcontentloaded', timeout: 30_000 });
  const selector = page.locator('select').first();

  for (const scene of scenes) {
    if (scene.key !== 'franka') {
      await selector.selectOption({ label: scene.label });
    }

    await page.waitForFunction(
      ({ key }) =>
        document.documentElement.dataset.sceneKey === key &&
        document.documentElement.dataset.sceneStatus !== 'loading',
      scene,
      { timeout: sceneTimeout },
    );

    const state = await page.evaluate(() => ({ ...document.documentElement.dataset }));
    if (state.sceneStatus !== 'ready') {
      throw new Error(`${scene.key} failed to load: ${state.sceneError ?? 'unknown error'}`);
    }
    if (Number(state.sceneInstances) !== scene.instances) {
      throw new Error(
        `${scene.key} compiled ${state.sceneInstances ?? 0} instances; expected ${scene.instances}`,
      );
    }

    await page.waitForTimeout(2_000);
    await page.screenshot({
      path: resolve(outputDirectory, `${scene.screenshotName ?? scene.key}.png`),
      fullPage: true,
    });
    console.log(
      `${scene.key}: ${state.sceneInstances} physical instances, ${state.sceneBodies} bodies`,
    );
  }

  if (failures.length > 0) {
    throw new Error(failures.join('\n'));
  }
} catch (error) {
  const state = await page.evaluate(() => ({ ...document.documentElement.dataset }));
  const pending = [...pendingRequests.entries()]
    .filter(([url]) => url.startsWith('http'))
    .map(([url, startedAt]) => `${Date.now() - startedAt}ms ${url}`);
  console.error('scene state:', state);
  console.error('pending requests:\n' + (pending.join('\n') || '(none)'));
  throw error;
} finally {
  await browser.close();
}
