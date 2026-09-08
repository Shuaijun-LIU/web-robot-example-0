import assert from 'node:assert/strict';
import { mkdir, writeFile } from 'node:fs/promises';
import { chromium } from 'playwright';

// Explicit CPU software renderer. This check does not initialize NVIDIA GPUs.
const browser = await chromium.launch({headless:true,
  executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader'],
});
const page = await browser.newPage({viewport:{width:1600,height:1100}});
const errors = [], failedRequests = [];
page.on('pageerror', error => errors.push(error.message));
page.on('response', response => { if(response.status() >= 400) failedRequests.push({url:response.url(),status:response.status()}); });
try {
  await mkdir('artifacts/screenshots',{recursive:true});
  await mkdir('artifacts/reports',{recursive:true});
  await page.goto(process.env.SCENE_URL ?? 'http://127.0.0.1:4174/web-robot-example-0/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.locator('select').filter({has:page.locator('option', {hasText:'Franka Demo2'})}).selectOption({label:'Franka Demo2'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo2'&&document.documentElement.dataset.sceneStatus==='ready'&&window.robotDemo,null,{timeout:120000});
  const targetSelector=page.locator('select').filter({has:page.locator('option').filter({hasText:/^Arm 1$/})});
  const controls=[];
  for(let i=0;i<4;i++){
    await targetSelector.selectOption({label:`Arm ${i+1}`});
    await page.waitForFunction(i=>document.documentElement.dataset.controlTarget===`r${i}`,i);
    const before=await page.evaluate(()=>window.robotDemo.getCtrl());
    await page.keyboard.press('v');
    await page.waitForFunction(i=>window.robotDemo.getCtrl()[i*8+7]===0,i);
    const closed=await page.evaluate(()=>window.robotDemo.getCtrl());
    for(let other=0;other<4;other++) if(other!==i) assert.equal(closed[other*8+7],before[other*8+7]);
    await page.keyboard.press('v');
    await page.waitForFunction(i=>window.robotDemo.getCtrl()[i*8+7]===255,i);
    const ik=await page.evaluate(()=>window.robotDemo.moveIkTargetBy(.002,0,0));
    assert.equal(ik,true,`Arm ${i+1} IK control must be available`);
    controls.push({arm:i+1,isolatedGripperToggle:true,ikTargetAccepted:ik});
  }
  await targetSelector.selectOption({label:'Arm 1'});
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForTimeout(3000);
  const initial = await page.evaluate(()=>({
    poses:window.robotDemo.getBodyPositions(Array.from({length:16},(_,i)=>`egg_${i}`)),
    fingers:window.robotDemo.getJointPositions(['r0_finger_joint1','r0_finger_joint2']),
    instances:document.documentElement.dataset.sceneInstances,
  }));
  await page.screenshot({path:'artifacts/screenshots/demo2-eggs-overview.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([.48,-.64,.72],[0,0,.145]));
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/demo2-eggs-box.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([.04,-.8,.65],[-.4,-.4,.115]));
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/demo2-eggs-tray.png'});
  await page.evaluate(()=>{
    const p=window.robotDemo.getBodyPositions(['r0_hand']).r0_hand;
    window.robotDemo.setInspectionCamera([p[0]+.26,p[1]-.32,p[2]+.24],p);
  });
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/demo2-eggs-gripper.png'});
  const report = await page.evaluate(()=>{
    const canvas=document.querySelector('canvas');
    const gl=canvas.getContext('webgl2');
    const ext=gl?.getExtension('WEBGL_debug_renderer_info');
    return {
      renderer:ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):null,
      poses:window.robotDemo.getBodyPositions(Array.from({length:16},(_,i)=>`egg_${i}`)),
      physics:window.robotDemo.getPhysicsDiagnostics(),
      contacts:window.robotDemo.getContacts(),
      qpos:window.robotDemo.getQpos(),
      selectOptions:Array.from(document.querySelectorAll('select')).map(select=>Array.from(select.options).map(o=>({text:o.text,value:o.value}))),
    };
  });
  await writeFile('artifacts/reports/demo2-eggs-browser.json',JSON.stringify({initial,...report,controls,errors,failedRequests},null,2));
  assert.equal(initial.instances,'4');
  assert.match(report.renderer,/SwiftShader/i);
  assert.ok(report.qpos.every(Number.isFinite));
  assert.deepEqual(errors,[]);
  assert.deepEqual(failedRequests,[]);
  assert.ok(report.physics.warnings.every(w=>w.count===0));
  const robotContacts=report.contacts.filter(c=>(c.body1.startsWith('r')||c.body2.startsWith('r'))&&c.distance<-.0003);
  assert.deepEqual(robotContacts,[],'static robots must not penetrate objects or each other');
  for(let i=0;i<16;i++) assert.ok(report.contacts.some(c=>c.body1===`egg_${i}`||c.body2===`egg_${i}`),`egg ${i} must have physical support`);
  await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo1$/})}).selectOption({label:'Franka Demo1'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo1'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  await page.getByRole('button',{name:'Play',exact:true}).waitFor();
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'PASS',renderer:report.renderer,instances:initial.instances,contacts:report.physics.contacts,eggs:report.poses}));
} finally { await browser.close(); }
