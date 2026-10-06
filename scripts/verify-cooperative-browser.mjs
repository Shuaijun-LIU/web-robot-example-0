import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_EXECUTABLE,
  args:['--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[],records=[];
page.on('pageerror',e=>errors.push(String(e)));
await mkdir('artifacts/screenshots',{recursive:true});await mkdir('artifacts/reports',{recursive:true});
try {
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:4185/',{waitUntil:'domcontentloaded',timeout:120000});
  for(const [key,label,bodies] of [['frankaDemo3','Franka Demo3',['tea_box','coffee_box','scanner','order_tray']],['frankaDemo4','Franka Demo4',['cooking_pot','carrot','tomato']]]) {
    await page.locator('select').filter({has:page.locator('option').filter({hasText:/^Franka Demo1$/})}).selectOption({label});
    await page.waitForFunction(key=>document.documentElement.dataset.sceneKey===key&&document.documentElement.dataset.sceneStatus==='ready',key,{timeout:120000});
    await page.getByLabel('IK gizmo',{exact:true}).evaluate(el=>{if(el.checked)el.click();});
    await page.evaluate(()=>window.robotDemo.setInspectionCamera([1.3,-1.65,1.65],[0,0,.16]));
    await page.waitForTimeout(4000);
    assert.ok(Number(await page.evaluate(()=>document.documentElement.dataset.cooperativeTextures))>=4,'source textures must reach rendered objects');
    const snapshot=await page.evaluate(bodies=>({objects:window.robotDemo.getBodyPositions(bodies),sites:window.robotDemo.getSitePositions(['r0_tcp','r1_tcp','r2_tcp','r3_tcp']),contacts:window.robotDemo.getContacts(),physics:window.robotDemo.getPhysicsDiagnostics()}),bodies);
    assert.ok(Object.values(snapshot.objects).flat().every(Number.isFinite));
    assert.ok(snapshot.physics.warnings.every(w=>w.count===0),JSON.stringify(snapshot.physics));
    await page.screenshot({path:`artifacts/screenshots/${key}-layout-2026-10-07.png`});
    await page.getByRole('button',{name:'reset',exact:true}).click();
    records.push({key,...snapshot});
    console.log('Layout displayed',key,JSON.stringify(snapshot.objects));
  }
  assert.deepEqual(errors,[]);
  await writeFile('artifacts/reports/cooperative-browser-layouts.json',JSON.stringify({success:true,scope:'static scene loading, physical settling, four TCPs, Reset and scene switch',records,errors},null,2)+'\n');
}finally{await browser.close();}
