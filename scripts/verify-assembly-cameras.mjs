import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';
const browser=await chromium.launch({headless:true,...process.env.CHROME_EXECUTABLE?{executablePath:process.env.CHROME_EXECUTABLE,args:['--use-angle=vulkan','--enable-features=Vulkan','--ignore-gpu-blocklist']}:{}});
const page=await browser.newPage({viewport:{width:1600,height:1000}}),errors=[];
page.on('pageerror',error=>errors.push(error.message));
page.on('console',message=>{if(message.type()==='error')errors.push(message.text());});
page.on('response',response=>{if(response.status()>=400&&response.url().includes('/assembly-cameras/'))errors.push(`${response.status()} ${response.url()}`);});
const ready=async()=>{
  await page.waitForFunction(()=>document.documentElement.dataset.sceneStatus==='ready'&&window.robotDemo,null,{timeout:120000});
  await page.waitForFunction(()=>document.querySelector('.assembly-cameras-panel')&&!document.querySelector('.assembly-cameras-panel [role="status"]'),null,{timeout:120000});
};
try{
  await mkdir('artifacts/screenshots',{recursive:true});
  await page.goto(process.env.SCENE_URL??'http://127.0.0.1:3000');await ready();
  assert.equal(await page.locator('.assembly-camera-tile').count(),1);
  await page.getByRole('combobox',{name:'Camera view',exact:true}).selectOption('all');
  await page.waitForTimeout(500);
  assert.equal(await page.locator('.assembly-camera-tile').count(),5);
  await page.screenshot({path:'artifacts/screenshots/assembly-cameras-overview.png'});
  await page.getByRole('button',{name:'Hide cameras',exact:true}).click();
  assert.equal(await page.locator('.assembly-camera-tile').count(),0);
  await page.getByRole('button',{name:'Cameras',exact:true}).click();
  assert.equal(await page.locator('.assembly-camera-tile').count(),5);
  // The editor and demo must share the rig, and remounting must not duplicate it.
  await page.locator('.franka-demo1-panel select').selectOption('frankaAssembly1');await ready();
  await page.waitForTimeout(250);
  const before=await page.screenshot({clip:{x:420,y:260,width:700,height:360}});
  await page.evaluate(()=>window.robotDemo.runAssemblyStep1());
  await page.waitForFunction(()=>['complete','error'].includes(document.documentElement.dataset.assemblyStep1Status),null,{timeout:120000});
  assert.equal(await page.evaluate(()=>document.documentElement.dataset.assemblyStep1Status),'complete');
  await page.waitForTimeout(500);
  await page.screenshot({path:'artifacts/screenshots/assembly-cameras-working.png'});
  assert.equal(before.equals(await page.screenshot({clip:{x:420,y:260,width:700,height:360}})),false,'Main scene must continue to animate with camera panel enabled');
  const crops=[];
  for(const key of ['global','arm1','arm2','arm3','arm4']){
    const rect=await page.locator(`[data-camera="${key}"]`).boundingBox();
    assert.ok(rect?.width>0);
    const image=await page.screenshot({clip:rect});crops.push(image);
    await writeFile(`artifacts/screenshots/assembly-camera-${key}.png`,image);
  }
  for(let i=0;i<crops.length;i++)for(let j=i+1;j<crops.length;j++)assert.equal(crops[i].equals(crops[j]),false,'Camera feeds must not be duplicates');
  assert.deepEqual(errors,[]);
  console.log('PASS: Pages-compatible assets, five distinct working camera views, collapse and scene switching');
}finally{await browser.close();}
