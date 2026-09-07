import assert from 'node:assert/strict';
import {mkdir} from 'node:fs/promises';
import {chromium} from 'playwright';

const browser = await chromium.launch({headless: true, ...(process.env.CHROME_EXECUTABLE ? {
  executablePath: process.env.CHROME_EXECUTABLE,
  args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'],
} : {})});
const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
const errors = [];
page.on('pageerror', error => errors.push(error.message));
const assertEnglish = async () => assert.doesNotMatch(await page.locator('body').innerText(), /\p{Script=Han}/u);
try {
  await page.goto(process.env.SCENE_URL ?? 'http://127.0.0.1:3000');
  await page.waitForSelector('.franka-demo1-panel select', {timeout: 120000});
  await page.locator('.franka-demo1-panel select').selectOption('frankaAssembly1');
  await page.waitForFunction(() => document.documentElement.dataset.sceneKey === 'frankaAssembly1'
    && document.documentElement.dataset.sceneStatus === 'ready' && window.robotDemo, null, {timeout: 120000});
  await assertEnglish();
  await page.getByRole('button', {name: 'Enter manual posing', exact: true}).click();
  await page.getByRole('button', {name: 'Exit manual posing', exact: true}).waitFor();
  await assertEnglish();
  await page.getByLabel('Pose name', {exact: true}).fill('../invalid');
  await page.getByRole('button', {name: 'Save current pose', exact: true}).click();
  await page.locator('.assembly-pose-panel__message--error').waitFor();
  await assertEnglish();
  await page.getByLabel('Pose name', {exact: true}).fill('english-ui-check');
  const downloadReady = page.waitForEvent('download');
  await page.getByRole('button', {name: 'Save current pose', exact: true}).click();
  assert.equal((await downloadReady).suggestedFilename(), 'franka-assembly1-english-ui-check.json');
  await page.getByText('Downloaded:', {exact: false}).waitFor();
  await assertEnglish();
  await page.getByRole('button', {name: 'Exit manual posing', exact: true}).click();
  await page.getByRole('button', {name: 'Step 1: Move into position', exact: true}).click();
  await page.waitForFunction(() => ['complete', 'error'].includes(document.documentElement.dataset.assemblyStep1Status), null, {timeout: 120000});
  assert.equal(await page.evaluate(() => document.documentElement.dataset.assemblyStep1Status), 'complete');
  await assertEnglish();
  await mkdir('artifacts/screenshots', {recursive: true});
  await page.screenshot({path: 'artifacts/screenshots/assembly1-english-ui.png'});
  assert.deepEqual(errors, []);
  console.log('PASS: Assembly1 English UI, manual mode, validation, JSON download and Step 1 completion');
} finally {
  await browser.close();
}
