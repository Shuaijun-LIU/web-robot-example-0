import assert from 'node:assert/strict';
import { mkdir,writeFile } from 'node:fs/promises';
import { chromium } from 'playwright';

// Explicit CPU SwiftShader renderer; no NVIDIA device is initialized.
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:900}});
const errors=[];
const phaseErrors=[];
page.on('pageerror',e=>errors.push(String(e)));
page.on('console',m=>{if(m.text().startsWith('[egg-phase]')){console.log(m.text());if(m.text().includes('"phase":"error"'))phaseErrors.push(m.text());}});
await mkdir('artifacts/screenshots',{recursive:true});
await mkdir('artifacts/reports',{recursive:true});
let result;
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4174/web-robot-example-0/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo2$/})}).selectOption({label:'Franka Demo2'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneStatus==='ready'&&window.eggTransfer?.state.phase==='ready',null,{timeout:120000});
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForTimeout(1500);
  await page.getByLabel('speed',{exact:true}).fill('3.0');
  await page.getByLabel('speed',{exact:true}).press('Enter');
  await page.evaluate(()=>{
    window.robotDemo.setInspectionCamera([.45,-.85,.75],[-.1,-.18,.2]);
    let diagnostic=window.eggTransfer;
    window.__eggTransitions=[];
    Object.defineProperty(window,'eggTransfer',{configurable:true,get:()=>diagnostic,set:value=>{
      const previous=diagnostic?.state;diagnostic=value;
      if(value?.state&&(previous?.phase!==value.state.phase||previous?.label!==value.state.label))window.__eggTransitions.push({...value.state});
    }});
    let last='';window.__eggAuditTimer=setInterval(()=>{
      const state=window.eggTransfer?.state;
      if(state&&state.label!==last){last=state.label;console.info('[egg-phase]',JSON.stringify({state,observation:window.eggTransfer.observation}));}
    },200);
  });
  await page.getByRole('button',{name:'Run first egg',exact:true}).click();
  // Keyboard must not replace the automation's selected-jaw command.
  await page.keyboard.press('v');
  if(!process.env.EGG_RESET_ONLY) {
  await page.waitForFunction(()=>['complete','error'].includes(window.eggTransfer?.state.phase),null,{timeout:480000});
  result=await page.evaluate(()=>({
    transfer:window.eggTransfer,physics:window.robotDemo.getPhysicsDiagnostics(),
    positions:window.robotDemo.getBodyPositions(Array.from({length:16},(_,i)=>`egg_${i}`)),
    contacts:window.robotDemo.getContacts(),qpos:window.robotDemo.getQpos(),
  }));
  // Leva styles the native checkbox as hidden; native click dispatches its
  // actual change handler without relying on the decorative square's CSS.
  await page.getByLabel('paused',{exact:true}).evaluate(el=>el.click());
  await page.getByLabel('IK gizmo',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([-.13,-.82,.50],[-.4,-.4,.14]));
  await page.screenshot({path:`artifacts/screenshots/demo2-first-egg-${result.transfer.state.phase}.png`});
  await page.getByLabel('paused',{exact:true}).evaluate(el=>el.click());
  await writeFile('artifacts/reports/demo2-first-egg-browser.json',JSON.stringify({result,errors},null,2));
  assert.equal(result.transfer.state.phase,'complete',JSON.stringify(result.transfer));
  assert.ok(result.transfer.history.find(s=>s.phase==='Close on egg').observation.bilateral);
  assert.ok(result.transfer.history.find(s=>s.phase==='Lift clear of box').observation.lift>.06);
  assert.ok(result.transfer.observation.traySupported);
  assert.equal(result.transfer.observation.bilateral,false);
  assert.ok(result.transfer.observation.tiltDegrees<20);
  assert.ok(result.physics.warnings.every(w=>w.count===0));
  assert.deepEqual(errors,[]);
  if(!process.env.EGG_PLAY_ONLY) {
    await page.getByRole('button',{name:'reset',exact:true}).click();
    await page.waitForFunction(()=>window.eggTransfer?.state.phase==='ready');
    await page.getByRole('button',{name:'Run first egg',exact:true}).click();
  }
  }
  if(!process.env.EGG_PLAY_ONLY) {
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([.45,-.85,.75],[-.1,-.18,.2]));
  await page.waitForFunction(()=>window.eggTransfer?.state.phase==='running'&&window.eggTransfer?.observation?.lift>.08,null,{timeout:180000});
  await page.screenshot({path:'artifacts/screenshots/demo2-first-egg-carry.png'});
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForFunction(()=>window.eggTransfer?.state.phase==='ready');
  await page.waitForTimeout(3000);
  assert.equal(await page.evaluate(()=>window.eggTransfer?.state.phase),'ready');
  assert.deepEqual(phaseErrors,[],'Reset must cancel before the old path observes freshly reset egg poses');
  assert.equal(await page.evaluate(()=>window.robotDemo.getBodyPositions(['egg_0']).egg_0[0]>-.2),true);
  }
  const transitions=await page.evaluate(()=>window.__eggTransitions);
  assert.deepEqual(transitions.filter(s=>s.phase==='error'),[],'lossless transition audit must not contain a transient error');
  await writeFile(`artifacts/reports/${process.env.EGG_RESET_ONLY?'demo2-egg-reset-browser':'demo2-first-egg-browser'}.json`,JSON.stringify({result,errors,phaseErrors,transitions,reset:process.env.EGG_PLAY_ONLY?null:{testedWhileCarrying:true,cancelledWithoutStaleCommand:true}},null,2));
  console.log(process.env.EGG_RESET_ONLY?'PASS: carrying-state Reset with lossless transition audit':'PASS: actuator-only egg transfer, real contacts and supported upright release');
} finally {
  await page.evaluate(()=>clearInterval(window.__eggAuditTimer)).catch(()=>{});
  await browser.close();
}
