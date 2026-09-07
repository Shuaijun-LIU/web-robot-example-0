import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

const browser=await chromium.launch({headless:true,...(process.env.CHROME_EXECUTABLE?{
  executablePath:process.env.CHROME_EXECUTABLE,
  args:['--use-angle=vulkan','--enable-features=Vulkan','--ignore-gpu-blocklist'],
}:{})});
const page=await browser.newPage({viewport:{width:1600,height:1000}});
const errors=[];
page.on('pageerror',e=>errors.push(e.message));
try{
  await mkdir('artifacts/screenshots',{recursive:true});
  await mkdir('artifacts/reports',{recursive:true});
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4174/web-robot-example-0/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneStatus==='ready'&&window.robotDemo,null,{timeout:120000});
  await page.waitForTimeout(2000); // Observe contact settling before taking a baseline.
  const initial=await page.evaluate(()=>({position:window.robotDemo.getBodyPositions(['open_end_wrench']).open_end_wrench,
    contacts:window.robotDemo.getContacts().filter(c=>[c.body1,c.body2].includes('open_end_wrench'))}));
  assert.ok(initial.position&&initial.position.every(Number.isFinite));
  assert.ok(initial.contacts.some(c=>[c.body1,c.body2].includes('handover_pad')),'wrench must physically rest on the pad');
  assert.ok(Math.abs(initial.position[0])<.002&&Math.abs(initial.position[1]+.48)<.002);
  assert.ok(Math.abs(initial.position[2]-.121079)<.00015,'visible and collision bottom must sit at the pad top');
  const hideCameras=page.getByRole('button',{name:'Hide cameras',exact:true});
  if(await hideCameras.count())await hideCameras.click();
  await page.screenshot({path:'artifacts/screenshots/assembly-wrench-overview.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([.22,-.85,.6],[0,-.48,.12]));
  await page.waitForTimeout(300);
  await page.screenshot({path:'artifacts/screenshots/assembly-wrench-closeup.png'});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([2.85,-2.85,3.05],[0,0,.32]));
  if(process.env.PLAY_DEMO==='1'){
    await page.evaluate(()=>{
      window.__wrenchAudit={foreignContacts:[],maxDisplacement:0,baseline:window.robotDemo.getBodyPositions(['open_end_wrench']).open_end_wrench};
      window.__wrenchTimer=setInterval(()=>{
        const a=window.__wrenchAudit,d=window.robotDemo,p=d.getBodyPositions(['open_end_wrench']).open_end_wrench;
        a.maxDisplacement=Math.max(a.maxDisplacement,Math.hypot(...p.map((v,i)=>v-a.baseline[i])));
        for(const c of d.getContacts())if([c.body1,c.body2].includes('open_end_wrench')&&![c.body1,c.body2].includes('handover_pad')&&a.foreignContacts.length<30)a.foreignContacts.push(c);
      },40);
    });
    await page.getByRole('button',{name:'Play',exact:true}).click();
    await page.waitForFunction(()=>{const d=document.documentElement.dataset;return d.assemblyStep4Status==='complete'||[d.assemblyStep1Status,d.assemblyStep2Status,d.assemblyStep3Status,d.assemblyStep4Status].includes('error');},null,{timeout:600000});
  }
  const report=await page.evaluate(()=>{clearInterval(window.__wrenchTimer);return {
    status:document.querySelector('.franka-demo1-panel__status')?.textContent,
    audit:window.__wrenchAudit,physics:window.robotDemo.getPhysicsDiagnostics(),
    position:window.robotDemo.getBodyPositions(['open_end_wrench']).open_end_wrench,
    contacts:window.robotDemo.getContacts().filter(c=>[c.body1,c.body2].includes('open_end_wrench')),
  };});
  await writeFile(`artifacts/reports/assembly-wrench-${process.env.PLAY_DEMO==='1'?'playback':'idle'}.json`,JSON.stringify({initial,...report,errors},null,2));
  if(process.env.PLAY_DEMO==='1'){
    assert.equal(report.status,'Demo complete');
    assert.deepEqual(report.audit.foreignContacts,[]);
    assert.ok(report.audit.maxDisplacement<.001);
  }
  assert.ok(report.physics.warnings.every(w=>w.count===0));
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({result:'PASS',status:report.status,position:report.position,audit:report.audit}));
}finally{await browser.close();}
