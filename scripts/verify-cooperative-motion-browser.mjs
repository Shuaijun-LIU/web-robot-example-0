import assert from 'node:assert/strict';
import {mkdir,writeFile,readFile} from 'node:fs/promises';
import {chromium} from 'playwright';

// Software WebGL: this check does not initialize an NVIDIA renderer.
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1000,height:750}}),errors=[],records=[];
await page.routeWebSocket(/.*/,socket=>socket.close());
page.on('pageerror',e=>errors.push(String(e)));
await mkdir('artifacts/screenshots',{recursive:true});await mkdir('artifacts/reports',{recursive:true});
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4185/',{waitUntil:'domcontentloaded',timeout:120000});
  for(const scene of (process.env.COOPERATIVE_SCENES??'scan,pot').split(',')){
    const label=scene==='scan'?'Franka Demo3':'Franka Demo4';
    const program=JSON.parse(await readFile(`public/assets/franka-cooperative/${scene}-motion.json`,'utf8'));
    const selector=page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo1$/})});
    await selector.selectOption({label});
    await page.waitForFunction(()=>document.documentElement.dataset.sceneStatus==='ready',null,{timeout:120000});
    await page.getByRole('button',{name:'Play task',exact:true}).waitFor();
    await page.waitForFunction(()=>Array.from(document.querySelectorAll('button')).some(b=>b.textContent==='Play task'&&!b.disabled));
    await page.getByLabel('IK gizmo',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
    await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.3,-1.65,1.65],[0,0,.16]));
    await page.getByRole('button',{name:'Play task',exact:true}).click();
    await page.waitForFunction(()=>window.cooperativeMotion?.time>1);
    await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(!el.checked)el.click();});
    await page.waitForTimeout(200);
    const pausedTime=await page.evaluate(()=>window.cooperativeMotion.time);
    await page.waitForTimeout(500);
    assert.equal(await page.evaluate(()=>window.cooperativeMotion.time),pausedTime,'pause must stop the task clock');
    const end=Date.now()+25*60*1000,captured=new Set();let result=null,stage=-1;
    while(Date.now()<end){
      const current=await page.evaluate(()=>({time:window.cooperativeMotion.time,phaseTime:window.cooperativeMotion.phaseTime,stage:window.cooperativeMotion.state.stage}));
      const remaining=program.phases[current.stage].duration-current.phaseTime;
      const ticks=remaining<=0?25:Math.min(1000,Math.max(1,Math.ceil(remaining/.002)+1));
      assert.equal(await page.evaluate(ticks=>window.cooperativeMotion.stepInspection(ticks),ticks),true);
      await page.waitForFunction(min=>window.cooperativeMotion.time>=min||window.cooperativeMotion.state.phase!=='running',current.time+(ticks-1)*.002-1e-7,{timeout:120000});
      result=await page.evaluate(()=>{const {stepInspection,...result}=window.cooperativeMotion;return result;});
      assert.ok(result,'motion diagnostics should exist after Play');
      if(result.state.phase==='error'){
        await writeFile(`artifacts/reports/cooperative-${scene}-browser-failure.json`,JSON.stringify(result,null,2)+'\n');
        await page.screenshot({path:`artifacts/screenshots/${scene}-failure-2026-10-07.png`});
      }
      assert.notEqual(result.state.phase,'error',JSON.stringify(result));
      if(result.state.stage!==stage){stage=result.state.stage;console.log(scene,stage,result.state.label,result.time);}
      const name=result.history.at(-1)?.phase??'';
      const shot=scene==='scan'
        ? (/Inspect tea_box/.test(name)?'inspect':/Arm 4 push/.test(name)?'dispatch':null)
        : (name==='Lift pot for loading'?'lift':name==='Settle tomato inside pot'?'load':null);
      if(shot&&!captured.has(shot)){
        await page.screenshot({path:`artifacts/screenshots/${scene}-${shot}-2026-10-07.png`});captured.add(shot);
      }
      if(result.state.phase==='complete')break;
    }
    assert.equal(result?.state.phase,'complete',JSON.stringify(result));
    assert.ok(result.metrics.maxForbiddenPenetration<1e-6,'the rendered route must clear other robots and fixtures');
    const diagnostics=await page.evaluate(()=>window.robotDemo.getPhysicsDiagnostics());
    assert.ok(diagnostics.warnings.every(w=>w.count===0),JSON.stringify(diagnostics));
    await page.screenshot({path:`artifacts/screenshots/${scene}-complete-2026-10-07.png`});
    records.push({...result,diagnostics});
    await writeFile(`artifacts/reports/cooperative-${scene}-browser.json`,JSON.stringify({success:true,...result,diagnostics,errors},null,2)+'\n');
    await page.getByRole('button',{name:'reset',exact:true}).click();
    await page.waitForFunction(()=>!window.cooperativeMotion);
    await page.getByLabel('paused',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
  }
  assert.deepEqual(errors,[]);
  await writeFile('artifacts/reports/cooperative-browser-motion.json',JSON.stringify({success:true,records,errors},null,2)+'\n');
}finally{await browser.close();}
