import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {chromium} from 'playwright';
import {ASSEMBLY1_STEP3_HOME_JOINT_TARGETS} from '../src/assemblyStep3.js';

const browser=await chromium.launch({headless:true,...process.env.CHROME_EXECUTABLE?{executablePath:process.env.CHROME_EXECUTABLE,args:['--use-angle=vulkan','--enable-features=Vulkan','--ignore-gpu-blocklist']}:{}});
const page=await browser.newPage({viewport:{width:1440,height:900},recordVideo:{dir:resolve('artifacts/videos/assembly-physical-audit'),size:{width:1440,height:900}}});
const failures=[];
page.on('pageerror',e=>failures.push(String(e)));
page.on('console',m=>{if(m.type()==='error')failures.push(m.text());if(m.text().startsWith('[demo-phase]'))console.log(m.text());});
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4173',{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo1'&&document.documentElement.dataset.sceneStatus==='ready'&&window.robotDemo,null,{timeout:120000});
  assert.equal(await page.locator('.franka-demo1-panel button').count(),2);
  assert.equal(await page.locator('.assembly-sequence-panel').count(),0);
  await page.evaluate(()=>{
    window.__demoAudit={violations:[],phases:[],phaseSamples:[],peakGripError:0,maximumPinPenetration:0};
    window.__demoAuditTimer=setInterval(()=>{
      const a=window.__demoAudit,d=window.robotDemo.getAssemblyStep4Diagnostics();
      if(d&&a.phases.at(-1)!==d.phase){a.phases.push(d.phase);a.phaseSamples.push({phase:d.phase,diagnostics:d,qpos:window.robotDemo.getQpos(),tools:window.robotDemo.getBodyPositions(['double_face_hammer']),sites:window.robotDemo.getSitePositions(['r1_tcp','r3_tcp','hammer_donor_grasp','hammer_receiver_grasp'])});console.info('[demo-phase]',d.phase);}
      if(d){a.maximumPinPenetration=Math.max(a.maximumPinPenetration,d.workpiecePenetration);if(['hammer-release','donor-clear','receiver-retreat','fastener-tighten','lift','transfer','insert','hammer-stage','hammer-strike','hammer-recover','hammer-return','hammer-lower'].includes(d.phase))a.peakGripError=Math.max(a.peakGripError,d.hammerReceiverGraspPointDistance);}
      for(const c of window.robotDemo.getContacts()) {
        const crossArm=/^r[0-3]_/.test(c.body1)&&/^r[0-3]_/.test(c.body2)&&c.body1.slice(0,2)!==c.body2.slice(0,2);
        const grasp=[c.body1,c.body2].some(n=>['cross_member','double_face_hammer','fastener_1'].includes(n))&&[c.body1,c.body2].some(n=>/^r[0-3]_(left|right)_finger$/.test(n));
        if((crossArm&&c.distance<-.001)||(grasp&&c.distance<-.0025))if(a.violations.length<30)a.violations.push({steps:[document.documentElement.dataset.assemblyStep1Status,document.documentElement.dataset.assemblyStep2Status,document.documentElement.dataset.assemblyStep3Status,document.documentElement.dataset.assemblyStep4Status],...c});
      }
    },20);
  });
  await page.getByRole('button',{name:'Play',exact:true}).click();
  await page.waitForFunction(()=>{
    const d=document.documentElement.dataset;
    return d.assemblyStep4Status==='complete'||[d.assemblyStep1Status,d.assemblyStep2Status,d.assemblyStep3Status,d.assemblyStep4Status].includes('error');
  },null,{timeout:600000});
  await page.waitForTimeout(1000);
  const result=await page.evaluate(()=>{
    clearInterval(window.__demoAuditTimer);
    return {status:document.querySelector('.franka-demo1-panel__status')?.textContent,diagnostics:window.robotDemo.getAssemblyStep4Diagnostics(),physics:window.robotDemo.getPhysicsDiagnostics(),audit:window.__demoAudit,joints:window.robotDemo.getJointPositions(Array.from({length:4},(_,a)=>Array.from({length:7},(_,j)=>`r${a}_joint${j+1}`)).flat()),contacts:window.robotDemo.getContacts()};
  });
  console.log(JSON.stringify(result,null,2));
  assert.equal(result.status,'Demo complete');
  assert.equal(result.diagnostics.phase,'complete');
  assert.ok(result.diagnostics.strikeObserved);
  assert.ok(result.diagnostics.strikeSupportContacts.every(pair=>pair.every(Boolean)));
  assert.deepEqual(result.audit.violations,[]);
  assert.ok(result.audit.peakGripError<.016);
  assert.ok(result.audit.maximumPinPenetration<.0015);
  assert.ok(result.diagnostics.fastenerPlanarDistance<.004&&result.diagnostics.insertionDepth>.003);
  assert.ok(result.physics.warnings.every(w=>w.count===0));
  for(let a=0;a<4;a++)for(let j=0;j<7;j++)assert.ok(Math.abs(result.joints[`r${a}_joint${j+1}`]-ASSEMBLY1_STEP3_HOME_JOINT_TARGETS[j])<.04);
  for(const support of ['hammer_return_cradle_tail','hammer_return_cradle_head'])assert.ok(result.contacts.some(c=>[c.body1,c.body2].includes(support)&&[c.body1,c.body2].includes('double_face_hammer')&&c.distance<=.0001));
  assert.deepEqual(failures,[]);
  await mkdir(resolve('artifacts/reports'),{recursive:true});
  await writeFile(resolve('artifacts/reports/franka-demo1-physical-audit.json'),JSON.stringify({result:'PASS',...result},null,2));
  await page.screenshot({path:resolve('artifacts/screenshots/franka-demo1-complete.png')});
  await page.close();
  await page.video().saveAs(resolve('artifacts/videos/franka-demo1-complete.webm'));
  console.log('PASS: production Demo1 continuous playback, contacts, tap and home positions');
} finally {await browser.close();}
