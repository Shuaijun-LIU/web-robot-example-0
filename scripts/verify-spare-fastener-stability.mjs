import assert from 'node:assert/strict';
import {mkdir, writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';

const playDemo = process.env.PLAY_DEMO === '1';
const browser = await chromium.launch({headless: true, ...(process.env.CHROME_EXECUTABLE ? {
  executablePath: process.env.CHROME_EXECUTABLE,
  args: ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'],
} : {})});
const page = await browser.newPage({viewport: {width: 1400, height: 1000}});
const errors = [];
page.on('pageerror', error => errors.push(error.message));
page.on('console', message => {if (message.text().startsWith('[spare-audit]')) console.log(message.text());});
try {
  await page.goto(process.env.SCENE_URL ?? 'http://127.0.0.1:4174/web-robot-example-0/');
  await page.waitForFunction(() => document.documentElement.dataset.sceneStatus === 'ready' && window.robotDemo, null, {timeout: 120000});
  await page.evaluate(() => {
    window.__spareSamples = [];
    window.__sparePhase = '';
    window.__spareForeignContacts = [];
    window.__spareTimer = setInterval(() => {
      const d = window.robotDemo, names = ['fastener_2', 'fastener_3'];
      const states = document.documentElement.dataset;
      const phase = [states.assemblyStep1Status, states.assemblyStep2Status, states.assemblyStep3Status, states.assemblyStep4Status].join('/');
      if (phase !== window.__sparePhase) {window.__sparePhase = phase; console.info('[spare-audit]', phase);}
      window.__spareSamples.push({ms: performance.now(), phase, p: d.getBodyPositions(names), q: d.getBodyOrientations(names)});
      if (window.__spareSamples.length % 5 === 0 && window.__spareForeignContacts.length < 20) {
        window.__spareForeignContacts.push(...d.getContacts().filter(c => names.includes(c.body1) || names.includes(c.body2)).filter(c => ![c.body1, c.body2].includes('fastener_tray')).map(c => ({phase, ...c})));
      }
    }, 40);
  });
  if (playDemo) {
    await page.getByRole('button', {name: 'Play', exact: true}).click();
    await page.waitForFunction(() => {
      const d = document.documentElement.dataset;
      return d.assemblyStep4Status === 'complete' || [d.assemblyStep1Status, d.assemblyStep2Status, d.assemblyStep3Status, d.assemblyStep4Status].includes('error');
    }, null, {timeout: 600000});
    assert.equal(await page.locator('.franka-demo1-panel__status').innerText(), 'Demo complete');
  } else {
    await page.waitForFunction(() => window.__spareSamples.length >= 500, null, {timeout: 120000});
  }
  const report = await page.evaluate(() => {
    clearInterval(window.__spareTimer);
    const samples = window.__spareSamples.slice(100);
    return {mode: document.documentElement.dataset.assemblyStep4Status, sampleCount: samples.length, foreignContacts: window.__spareForeignContacts,
      elapsedSeconds: (samples.at(-1).ms - samples[0].ms) / 1000,
      spares: ['fastener_2', 'fastener_3'].map(name => {
        const p0 = samples[0].p[name], q0 = samples[0].q[name];
        return {name, position: samples.at(-1).p[name],
          maxTranslationMm: Math.max(...samples.map(s => Math.hypot(...s.p[name].map((v, i) => v - p0[i])) * 1000)),
          maxRotationDegrees: Math.max(...samples.map(s => 2 * Math.acos(Math.min(1, Math.abs(s.q[name].reduce((dot, v, i) => dot + v * q0[i], 0)))) * 180 / Math.PI)),
          contacts: window.robotDemo.getContacts().filter(c => [c.body1, c.body2].includes(name)),
        };
      }), physics: window.robotDemo.getPhysicsDiagnostics()};
  });
  console.log(JSON.stringify(report, null, 2));
  await mkdir('artifacts/reports', {recursive: true});
  await writeFile(`artifacts/reports/spare-fasteners-${playDemo ? 'demo' : 'idle'}.json`, JSON.stringify(report, null, 2));
  for (const spare of report.spares) {
    assert.ok(spare.maxRotationDegrees < .5, `${spare.name} must not keep spinning: ${spare.maxRotationDegrees} degrees`);
    assert.ok(spare.maxTranslationMm < .1, `${spare.name} must remain seated: ${spare.maxTranslationMm} mm`);
    assert.ok(spare.contacts.some(c => [c.body1, c.body2].includes('fastener_tray')));
  }
  assert.ok(report.physics.warnings.every(w => w.count === 0));
  assert.deepEqual(errors, []);
  console.log(`PASS: spare fasteners remain stable during ${playDemo ? 'complete Demo1 playback' : 'idle simulation'}`);
} finally {
  await browser.close();
}
