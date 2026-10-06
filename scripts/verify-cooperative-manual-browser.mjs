import assert from 'node:assert/strict';
import {chromium} from 'playwright';
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:800,height:600}});
await page.routeWebSocket(/.*/,socket=>socket.close());
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4185/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo1$/})}).selectOption({label:'Franka Demo3'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo3'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  const target=page.locator('select').filter({has:page.locator('option').filter({hasText:/^Arm 1$/})});
  await target.selectOption({label:'Arm 1'});
  await page.keyboard.press('v');
  await page.waitForFunction(()=>window.robotDemo.getCtrl()[7]===0);
  await target.selectOption({label:'Arm 2'});
  await page.waitForTimeout(1500);
  await target.selectOption({label:'Arm 1'});
  await page.waitForTimeout(1500);
  assert.equal(await page.evaluate(()=>window.robotDemo.getCtrl()[7]),0,'selecting another arm and back must not reopen a closed gripper');
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForTimeout(1500);
  assert.equal(await page.evaluate(()=>window.robotDemo.getCtrl()[7]),255,'Reset restores open grippers');
  console.log('PASS: target switching preserves gripper commands; Reset restores open jaws.');
}finally{await browser.close();}
