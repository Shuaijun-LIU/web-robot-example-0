import assert from 'node:assert/strict';
import {mkdir} from 'node:fs/promises';
import {chromium} from 'playwright';
const browser=await chromium.launch({headless:true,...process.env.CHROME_EXECUTABLE?{executablePath:process.env.CHROME_EXECUTABLE,args:['--use-angle=vulkan','--enable-features=Vulkan','--ignore-gpu-blocklist']}:{}});
const page=await browser.newPage({viewport:{width:1600,height:1000}}),failures=[];
page.on('pageerror',e=>failures.push(String(e)));
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:3000');
  await page.waitForFunction(()=>document.documentElement.dataset.sceneStatus==='ready'&&window.robotDemo,null,{timeout:120000});
  await page.waitForTimeout(1500);
  await mkdir('artifacts/screenshots',{recursive:true});
  await page.screenshot({path:'artifacts/screenshots/assembly-clean-overview.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.25,-1.45,1.8],[0,-.05,.2]));
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/assembly-clean-detail.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([1,-.95,.7],[.6,-.38,.16]));
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/assembly-clean-tools.png'});
  await page.locator('select').first().selectOption('frankaAssembly1');
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaAssembly1'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  await page.waitForTimeout(1000);
  await page.screenshot({path:'artifacts/screenshots/assembly-clean-editor.png'});
  assert.equal(await page.locator('.assembly-sequence-panel').count(),1);
  assert.deepEqual(failures,[]);
  console.log('PASS: Demo1 presentation, scene switching and Assembly1 editor render');
} finally {await browser.close();}
