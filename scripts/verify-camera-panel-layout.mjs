import assert from 'node:assert/strict';
import {mkdir} from 'node:fs/promises';
import {chromium} from 'playwright';

const browser = await chromium.launch({headless: true, ...(process.env.CHROME_EXECUTABLE ? {
  executablePath: process.env.CHROME_EXECUTABLE,
  args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'],
} : {})});
const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
const near = (a, b, message) => assert.ok(Math.abs(a - b) <= 1, `${message}: ${a} vs ${b}`);
try {
  await page.goto(process.env.SCENE_URL ?? 'http://127.0.0.1:3000');
  await page.waitForFunction(() => document.querySelector('.assembly-cameras-panel')
    && !document.querySelector('.assembly-cameras-panel [role="status"]'), null, {timeout: 120000});
  await page.getByRole('combobox', {name: 'Camera view', exact: true}).selectOption('all');
  for (const [width, height] of [[1600, 1000], [1280, 800], [900, 700], [390, 844]]) {
    await page.setViewportSize({width, height});
    const boxes = await page.locator('.assembly-camera-tile').evaluateAll(tiles => tiles.map(tile => {
      const box = tile.getBoundingClientRect();
      const caption = tile.parentElement.querySelector('figcaption').getBoundingClientRect();
      return {x: box.x, y: box.y, right: box.right, bottom: box.bottom, width: box.width, height: box.height, captionBottom: caption.bottom};
    }));
    assert.equal(boxes.length, 5);
    const [global, arm1, arm2, arm3, arm4] = boxes;
    near(global.y, arm1.y, 'Global and upper wrist views align at the top');
    near(global.bottom, arm3.bottom, 'Global and lower wrist views align at the bottom');
    near(arm1.y, arm2.y, 'Upper wrist views align');
    near(arm3.y, arm4.y, 'Lower wrist views align');
    near(arm1.x, arm3.x, 'Left wrist column aligns');
    near(arm2.x, arm4.x, 'Right wrist column aligns');
    near(global.captionBottom, arm3.captionBottom, 'Bottom captions align');
    for (const box of boxes) {
      near(box.height, box.width * 9 / 16, 'Every feed keeps its native 16:9 aspect ratio');
      assert.ok(box.x >= 0 && box.right <= width && box.y >= 0 && box.bottom <= height, 'Feeds stay within the viewport');
    }
  }
  await page.setViewportSize({width: 1600, height: 1000});
  await mkdir('artifacts/screenshots', {recursive: true});
  await page.screenshot({path: 'artifacts/screenshots/assembly-cameras-aligned.png'});
  await page.getByRole('combobox', {name: 'Camera view', exact: true}).selectOption('arm2');
  assert.equal(await page.locator('.assembly-camera-tile').count(), 1);
  await page.getByRole('button', {name: 'Hide cameras', exact: true}).click();
  assert.equal(await page.locator('.assembly-camera-tile').count(), 0);
  console.log('PASS: five camera views aligned at four viewport sizes, native aspect ratios, single view and collapse');
} finally {
  await browser.close();
}
