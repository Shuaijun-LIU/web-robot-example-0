import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
page.on('pageerror',e=>errors.push(String(e)));
try{
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4185/',{waitUntil:'domcontentloaded',timeout:120000});
  await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo2$/})}).selectOption({label:'Franka Demo2'});
  const run=page.getByRole('button',{name:'Run four-arm sorting',exact:true});
  await page.waitForFunction(()=>document.documentElement.dataset.sceneKey==='frankaDemo2'&&document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
  await run.waitFor({timeout:120000});
  const panel=page.getByRole('complementary',{name:'Egg sorting scene review'});
  assert.equal(await page.getByRole('button',{name:'Collapse sorting details'}).count(),1,'sorting panel needs a compact view');
  console.log('Checking compact panel and progress');
  assert.equal(await page.getByRole('progressbar',{name:'Completed eggs'}).getAttribute('value'),'0');
  const expanded=await panel.boundingBox();
  await page.getByRole('button',{name:'Collapse sorting details'}).click();
  const collapsed=await panel.boundingBox();
  assert.ok(collapsed.height<expanded.height-80,'collapse should expose more of the scene');
  await page.getByRole('button',{name:'Expand sorting details'}).click();
  await page.getByLabel('IK gizmo',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
  await page.getByLabel('speed',{exact:true}).fill('3.0');await page.getByLabel('speed',{exact:true}).press('Enter');
  await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.15,-1.65,1.8],[0,0,.12]));
  await run.click();
  await page.waitForFunction(()=>window.eggSorting?.time>.25||window.eggSorting?.state.phase==='error',null,{timeout:120000});
  assert.equal(await page.evaluate(()=>window.eggSorting.state.phase),'running');
  await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(!el.checked)el.click();});
  await page.getByText('Paused · simulation and task clock are stopped',{exact:true}).waitFor();
  console.log('Checking paused simulation and responsive layout');
  const before=await page.evaluate(()=>({time:window.eggSorting.time,eggs:window.robotDemo.getBodyPositions(['egg_0','egg_4'])}));
  await page.waitForTimeout(1000);
  assert.deepEqual(await page.evaluate(()=>({time:window.eggSorting.time,eggs:window.robotDemo.getBodyPositions(['egg_0','egg_4'])})),before);
  await page.setViewportSize({width:800,height:600});
  const small=await panel.boundingBox();assert.ok(small.x>=0&&small.y>=0&&small.x+small.width<=800&&small.y+small.height<=600);
  // Run software-rendered motion at the smaller viewport; capture at full size.
  await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
  await page.waitForFunction(()=>window.eggSorting?.time>16||window.eggSorting?.state.phase==='error',null,{timeout:240000});
  const active=await page.evaluate(()=>window.eggSorting);
  console.log('Physical startup',JSON.stringify({state:active.state,time:active.time,metrics:active.metrics}));
  assert.equal(active.state.phase,'running',JSON.stringify(active.state));
  assert.ok(active.metrics.maxSimultaneousArms>=2);
  await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(!el.checked)el.click();});
  await page.setViewportSize({width:1280,height:900});
  await mkdir('artifacts/screenshots',{recursive:true});await mkdir('artifacts/reports',{recursive:true});
  await page.screenshot({path:'artifacts/screenshots/demo2-polish-2026-10-07.png'});
  await page.getByRole('button',{name:'reset',exact:true}).click();
  await page.waitForFunction(()=>!Array.from(document.querySelectorAll('button')).find(b=>b.textContent==='Run four-arm sorting')?.disabled,null,{timeout:60000});
  assert.equal(await page.getByRole('progressbar',{name:'Completed eggs'}).getAttribute('value'),'0');
  assert.ok(await page.evaluate(()=>!window.eggSorting||window.eggSorting.state.phase==='ready'));
  assert.deepEqual(errors,[]);
  await writeFile('artifacts/reports/demo2-polish-browser-2026-10-07.json',JSON.stringify({success:true,scope:'UI, pause/resume, 16-second physical startup and Reset; full-16 replay is tested separately in WASM',expanded,collapsed,small,active,errors},null,2)+'\n');
  console.log('PASS: compact panel, progress, pause/resume, real startup and Reset');
}catch(error){
  const diagnostic=await page.evaluate(()=>({sorting:window.eggSorting,contacts:window.robotDemo?.getContacts(),physics:window.robotDemo?.getPhysicsDiagnostics()})).catch(()=>null);
  await mkdir('artifacts/reports',{recursive:true});
  await writeFile('artifacts/reports/demo2-polish-browser-failure.json',JSON.stringify({error:String(error),diagnostic,errors},null,2)+'\n');
  throw error;
}finally{await browser.close();}
