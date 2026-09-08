import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

// Software rendering keeps this browser check independent of GPU availability.
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:800,height:600}});
const errors=[],transitions=[];
page.on('pageerror',e=>errors.push(String(e)));
page.on('console',m=>{if(m.text().startsWith('[sorting-check]'))console.log(m.text());});
await mkdir('artifacts/screenshots',{recursive:true});
await mkdir('artifacts/reports',{recursive:true});
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4175/web-robot-example-0/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo2$/})}).selectOption({label:'Franka Demo2'});
  const run=page.getByRole('button',{name:'Run four-arm sorting',exact:true});
  await run.waitFor({timeout:120000});
  await page.waitForFunction(()=>!Array.from(document.querySelectorAll('button')).find(b=>b.textContent==='Run four-arm sorting')?.disabled,null,{timeout:120000});
  await page.keyboard.press('v'); // Reset must clear an idle manual closed toggle
  await page.waitForTimeout(1500);
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.getByLabel('speed',{exact:true}).fill('3.0');
  await page.getByLabel('speed',{exact:true}).press('Enter');
  await page.getByLabel('IK gizmo',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.15,-1.65,1.8],[0,0,.12]));
  await page.waitForTimeout(3000);
  await page.screenshot({path:'artifacts/screenshots/demo2-four-arm-ready.png'});
  await run.click();
  await page.waitForFunction(()=>window.eggSorting?.observations.some(o=>o.bilateral&&o.lift>.08)||window.eggSorting?.state.phase==='error',null,{timeout:240000});
  const startup=await page.evaluate(()=>window.eggSorting);
  assert.equal(startup.state.phase,'running',JSON.stringify(startup));
  assert.ok(await page.evaluate(()=>window.eggSorting.observations.some(o=>o.bilateral&&o.lift>.06)));
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForFunction(()=>!Array.from(document.querySelectorAll('button')).find(b=>b.textContent==='Run four-arm sorting')?.disabled);
  await page.waitForTimeout(3000);
  assert.ok(await page.evaluate(()=>!window.eggSorting||window.eggSorting.state.phase==='ready'));
  await run.click();
  await page.keyboard.press('v'); // manual commands must not replace automation
  let previous='',capturedParallel=false;
  const deadline=Date.now()+45*60*1000;
  while(Date.now()<deadline){
    const state=await page.evaluate(()=>({state:window.eggSorting?.state,time:window.eggSorting?.time,metrics:window.eggSorting?.metrics}));
    if(state.state){
      const key=JSON.stringify(state.state);
      if(key!==previous){previous=key;transitions.push(state);console.log('[sorting-check]',JSON.stringify(state));}
      if(!capturedParallel&&state.metrics?.maxSimultaneousArms===4){
        capturedParallel=true;
        await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(!el.checked)el.click();});
        await page.setViewportSize({width:1280,height:900});
        await page.screenshot({path:'artifacts/screenshots/demo2-four-arm-parallel.png'});
        await page.setViewportSize({width:800,height:600});
        await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
      }
      if(['complete','error'].includes(state.state.phase))break;
    }
    await page.waitForTimeout(2000);
  }
  const result=await page.evaluate(()=>({sorting:window.eggSorting,physics:window.robotDemo.getPhysicsDiagnostics(),
    positions:window.robotDemo.getBodyPositions(Array.from({length:16},(_,i)=>`egg_${i}`))}));
  await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(!el.checked)el.click();});
  await page.setViewportSize({width:1280,height:900});
  await page.screenshot({path:`artifacts/screenshots/demo2-four-arm-${result.sorting.state.phase}.png`});
  await writeFile('artifacts/reports/demo2-four-arm-browser.json',JSON.stringify({result,errors,transitions,resetWhileCarrying:true},null,2)+'\n');
  assert.equal(result.sorting.state.phase,'complete',JSON.stringify(result.sorting.state));
  assert.deepEqual(result.sorting.state.counts,[4,4,4,4]);
  assert.equal(result.sorting.metrics.maxSimultaneousArms,4);
  assert.ok(result.sorting.metrics.maxForbiddenPenetration<=.0001);
  assert.ok(result.physics.warnings.every(w=>w.count===0));
  assert.deepEqual(errors,[]);
  console.log('PASS: full four-arm browser sorting, physical contacts and carrying-state Reset');
}catch(error){
  const diagnostic=await page.evaluate(()=>({sorting:window.eggSorting,physics:window.robotDemo?.getPhysicsDiagnostics()})).catch(()=>null);
  await writeFile('artifacts/reports/demo2-four-arm-browser-failure.json',JSON.stringify({error:String(error),diagnostic,errors,transitions},null,2)+'\n');
  throw error;
}finally{await browser.close();}
