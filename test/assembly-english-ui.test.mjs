import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import test from 'node:test';
import ts from 'typescript';
import {createElement} from 'react';
import {renderToStaticMarkup} from 'react-dom/server';

async function loadPanel(filename) {
  const url = new URL(`../src/${filename}`, import.meta.url);
  const {outputText} = ts.transpileModule(await readFile(url, 'utf8'), {
    compilerOptions: {jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2022},
    fileName: filename,
  });
  const resolved = outputText.replace(/from (["'])([^"']+)\1/g, (_, quote, specifier) =>
    `from ${JSON.stringify(specifier.startsWith('.') ? new URL(specifier, url).href : import.meta.resolve(specifier))}`);
  return import(`data:text/javascript;base64,${Buffer.from(resolved).toString('base64')}`);
}

async function statesFor(step) {
  const filename = `assemblyStep${step}.d.ts`;
  const source = ts.createSourceFile(filename, await readFile(new URL(`../src/${filename}`, import.meta.url), 'utf8'), ts.ScriptTarget.Latest);
  const name = step === 1 ? 'AssemblyStep1Status' : `AssemblyStep${step}Phase`;
  return source.statements.find(node => ts.isTypeAliasDeclaration(node) && node.name.text === name).type.types.map(node => node.literal.text);
}

test('Assembly1 renders English labels for every action phase and failure', async () => {
  const {AssemblySequencePanel} = await loadPanel('AssemblySequencePanel.tsx');
  const defaults = {step1Status: 'idle', step2State: {phase: 'idle'}, step3State: {phase: 'idle'}, step4State: {phase: 'idle'}};
  for (let step = 1; step <= 4; step++) {
    for (const phase of await statesFor(step)) {
      const state = step === 1 ? {step1Status: phase} : {[`step${step}State`]: {phase, failure: phase === 'error' ? {code: 'verification-timeout', detail: '0.02'} : null}};
      const html = renderToStaticMarkup(createElement(AssemblySequencePanel, {...defaults, ...state}));
      assert.doesNotMatch(html, /\p{Script=Han}/u, `Step ${step}: ${phase} must render in English`);
      assert.equal((html.match(/<button/g) ?? []).length, 4);
      if (phase === 'error') assert.match(html, /Reset/);
    }
  }
});

test('Assembly1 manual capture renders English in both control modes', async () => {
  const {Assembly1PoseCapturePanel} = await loadPanel('Assembly1PoseCapturePanel.tsx');
  for (const manualMode of [true, false]) {
    const html = renderToStaticMarkup(createElement(Assembly1PoseCapturePanel, {sceneReady: true, selectedControlTarget: 'r2', manualMode}));
    assert.doesNotMatch(html, /\p{Script=Han}/u);
    assert.match(html, /r2/);
    assert.match(html, /JSON/);
  }
});
