import { execFileSync } from 'node:child_process';
import { mkdir, mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';

import { chromium } from 'playwright';

const baseUrl = process.env.SCENE_URL ?? 'http://127.0.0.1:3000';
const timeout = Number(process.env.SCENE_TIMEOUT_MS ?? 240_000);
const ffmpegPath = process.env.FFMPEG_PATH ?? 'ffmpeg';
const ffprobePath = process.env.FFPROBE_PATH ?? 'ffprobe';
const outputDirectory = resolve('artifacts/videos');
const outputPath = join(outputDirectory, 'unitree-locomotion-suite.mp4');
const temporaryDirectory = await mkdtemp(join(tmpdir(), 'unitree-action-video-'));
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
await mkdir(outputDirectory, { recursive: true });

const browser = await chromium.launch(launchOptions);
const context = await browser.newContext({
  viewport: { width: 1440, height: 900 },
  recordVideo: { dir: temporaryDirectory, size: { width: 1440, height: 900 } },
});
const page = await context.newPage();
const video = page.video();
const failures = [];
page.on('pageerror', (error) => failures.push(`page error: ${error.message}`));
page.on('requestfailed', (request) => failures.push(`request failed: ${request.url()}`));
page.on('console', (message) => {
  if (message.type() === 'error') failures.push(`console error: ${message.text()}`);
});

let webmPath;
try {
  await page.goto(baseUrl, { waitUntil: 'domcontentloaded', timeout: Math.min(timeout, 120_000) });
  await page.locator('select').first().selectOption({ label: 'Unitree Action Lab' });
  await page.waitForFunction(
    () => document.documentElement.dataset.sceneKey === 'unitreeActionLab'
      && document.documentElement.dataset.sceneStatus === 'ready'
      && Number(document.documentElement.dataset.sceneInstances) === 2,
    null,
    { timeout },
  );
  await page.waitForFunction(
    () => Boolean(window.robotDemo?.runUnitreeAction && window.robotDemo?.selectUnitreeActionProgram),
    null,
    { timeout },
  );
  await page.waitForTimeout(1_000);
  const selectedProgram = await page.evaluate(
    () => window.robotDemo.selectUnitreeActionProgram('locomotion'),
  );
  if (!selectedProgram) throw new Error('Could not select locomotion for video recording');
  const started = await page.evaluate(() => window.robotDemo.runUnitreeAction());
  if (!started) throw new Error('Unitree action did not start for video recording');
  await page.waitForFunction(
    () => document.documentElement.dataset.unitreeActionStatus === 'complete'
      || document.documentElement.dataset.unitreeActionStatus === 'error',
    null,
    { timeout },
  );
  const completionState = await page.evaluate(() => window.robotDemo.getUnitreeActionState());
  if (completionState.status === 'error') throw new Error(`Unitree action failed: ${completionState.error}`);
  await page.waitForTimeout(1_000);
  if (failures.length > 0) throw new Error(failures.join('\n'));
  await page.close();
  webmPath = await video.path();
  await context.close();
  await browser.close();

  const sourceDuration = Number(execFileSync(ffprobePath, [
    '-v', 'error',
    '-show_entries', 'format=duration',
    '-of', 'default=noprint_wrappers=1:nokey=1',
    webmPath,
  ], { encoding: 'utf8' }).trim());
  if (!Number.isFinite(sourceDuration) || sourceDuration <= 0) {
    throw new Error(`Could not read recorded video duration: ${sourceDuration}`);
  }
  const playbackRatio = Math.min(1, 27 / sourceDuration);

  execFileSync(ffmpegPath, [
    '-y',
    '-i', webmPath,
    '-vf', `setpts=${playbackRatio.toFixed(8)}*PTS,fps=30`,
    '-an',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-movflags', '+faststart',
    outputPath,
  ], { stdio: 'inherit' });
  console.log(JSON.stringify({ outputPath, sourceDuration, playbackRatio }));
} finally {
  if (browser.isConnected()) await browser.close();
  await rm(temporaryDirectory, { recursive: true, force: true });
}
