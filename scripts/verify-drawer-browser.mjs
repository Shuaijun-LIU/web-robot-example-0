import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

// Software WebGL only; never initializes an NVIDIA renderer.
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
page.on('pageerror',e=>errors.push(String(e)));await page.routeWebSocket(/.*/,s=>s.close());
let result;
try{
  const program=JSON.parse(await readFile('public/assets/franka-cooperative/drawer-motion.json','utf8'));
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4185/',{waitUntil:'domcontentloaded',timeout:120000});
  const selector=page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo1$/})});
  await selector.selectOption({label:'Franka Demo5'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo5'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(b=>b.textContent==='Play task'&&!b.disabled));
  await page.getByLabel('IK gizmo',{exact:true}).evaluate(e=>{if(e.checked)e.click();});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.3,-1.65,1.65],[0,0,.18]));
  await page.screenshot({path:'artifacts/screenshots/drawer-layout-2026-10-07.png'});
  await page.getByRole('button',{name:'Play task',exact:true}).click();
  await page.waitForFunction(()=>window.cooperativeMotion?.time>1);
  await page.getByLabel('paused',{exact:true}).evaluate(e=>{if(!e.checked)e.click();});
  await page.waitForTimeout(200);
  const paused=await page.evaluate(()=>window.cooperativeMotion.time);await page.waitForTimeout(400);
  assert.equal(await page.evaluate(()=>window.cooperativeMotion.time),paused);
  const captured=new Set(),deadline=Date.now()+25*60*1000;let stage=-1;
  while(Date.now()<deadline){
    const cur=await page.evaluate(()=>({time:window.cooperativeMotion.time,phaseTime:window.cooperativeMotion.phaseTime,stage:window.cooperativeMotion.state.stage}));
    const left=program.phases[cur.stage].duration-cur.phaseTime;
    const ticks=left<=0?25:Math.min(1000,Math.max(1,Math.ceil(left/.002)+1));
    assert.equal(await page.evaluate(n=>window.cooperativeMotion.stepInspection(n),ticks),true);
    await page.waitForFunction(t=>window.cooperativeMotion.time>=t||window.cooperativeMotion.state.phase!=='running',cur.time+(ticks-1)*.002-1e-7,{timeout:120000});
    result=await page.evaluate(()=>{const {stepInspection,...r}=window.cooperativeMotion;return r;});
    assert.notEqual(result.state.phase,'error',JSON.stringify({state:result.state,last:result.history.at(-1),metrics:result.metrics}));
    if(stage!==result.state.stage){stage=result.state.stage;console.log(stage,result.state.label,result.time);}
    const completed=result.history.at(-1)?.phase;
    const shot=completed==='Open: drawer operators return home'?'open':completed==='Confirm tea_box in assigned order slot'?'packing':null;
    if(shot&&!captured.has(shot)){await page.screenshot({path:`artifacts/screenshots/drawer-${shot}-2026-10-07.png`});captured.add(shot);}
    if(result.state.phase==='complete')break;
  }
  assert.equal(result.state.phase,'complete');assert.equal(result.history.length,program.phases.length);
  assert.ok(result.metrics.maxForbiddenPenetration<1e-6);
  const diagnostics=await page.evaluate(()=>window.robotDemo.getPhysicsDiagnostics());
  assert.ok(diagnostics.warnings.every(w=>w.count===0));
  await page.screenshot({path:'artifacts/screenshots/drawer-complete-2026-10-07.png'});
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForFunction(()=>!window.cooperativeMotion);
  await page.getByLabel('paused',{exact:true}).evaluate(e=>{if(e.checked)e.click();});
  await selector.selectOption({label:'Franka Demo3'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo3'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  await selector.selectOption({label:'Franka Demo5'});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo5'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  assert.deepEqual(errors,[]);
  await writeFile('artifacts/reports/cooperative-drawer-browser.json',JSON.stringify({success:true,...result,diagnostics,resetAndSwitch:true,errors},null,2)+'\n');
}catch(error){
  await page.screenshot({path:'artifacts/screenshots/drawer-browser-failure.png'}).catch(()=>{});
  await writeFile('artifacts/reports/drawer-browser-failure.json',JSON.stringify({error:String(error),result,errors},null,2)+'\n');
  throw error;
}finally{await browser.close();}
