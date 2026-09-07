import assert from 'node:assert/strict';
import {mkdir} from 'node:fs/promises';
import {createRequire} from 'node:module';
import {dirname,join} from 'node:path';
import {chromium} from 'playwright';

// Reuse Playwright's PNG decoder; no additional production dependency.
const require=createRequire(import.meta.url);
const {PNG}=require(join(dirname(require.resolve('playwright-core/package.json')),'lib/utilsBundle.js'));
const browser=await chromium.launch({headless:true,...(process.env.CHROME_EXECUTABLE?{
  executablePath:process.env.CHROME_EXECUTABLE,
  args:['--use-angle=vulkan','--enable-features=Vulkan','--ignore-gpu-blocklist'],
}:{})});
const page=await browser.newPage({viewport:{width:1600,height:1000}});
const errors=[];
page.on('pageerror',error=>errors.push(error.message));
try{
  await mkdir('artifacts/screenshots',{recursive:true});
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4174/web-robot-example-0/',{waitUntil:'domcontentloaded',timeout:120000});
  for(const [key,label,file] of [
    ['frankaDemo1','Franka Demo1','demo1-warm-palette'],
    ['frankaAssembly1','Franka Assembly1','assembly1-warm-palette'],
  ]){
    await page.locator('.franka-demo1-panel select').selectOption({label});
    await page.waitForFunction(key=>document.documentElement.dataset.sceneKey===key&&document.documentElement.dataset.sceneStatus==='ready',key,{timeout:120000});
    await page.evaluate(()=>new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve))));
    const screenshot=await page.screenshot({path:`artifacts/screenshots/${file}.png`});
    const png=PNG.sync.read(screenshot);
    const offset=(10*png.width+800)*4;
    const rgb=[...png.data.subarray(offset,offset+3)];
    // Unobstructed rendered background: pre-September-6 warm beige #d8d2b5.
    assert.ok(rgb.every((v,i)=>Math.abs(v-[216,210,181][i])<=2),`${label} background must be warm beige, got ${rgb}`);
    assert.equal(await page.evaluate(()=>document.documentElement.dataset.sceneInstances),'4');
    console.log(`PASS: ${label}, four arms loaded, background RGB ${rgb}`);
  }
  assert.deepEqual(errors,[]);
}finally{
  await browser.close();
}
