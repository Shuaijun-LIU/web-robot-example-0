import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

const docsIndexUrl = new URL('../docs/index.html', import.meta.url);

test('committed Pages artifact contains the released Step 3 flow and Step 4 controls', async () => {
  const index = await readFile(docsIndexUrl, 'utf8');
  const scriptPath = index.match(/src="[^"]*\/assets\/(index-[^"]+\.js)"/)?.[1];

  assert.ok(scriptPath, 'docs/index.html must reference a built JavaScript asset');

  const bundle = await readFile(new URL(`../docs/assets/${scriptPath}`, import.meta.url), 'utf8');
  assert.ok(bundle.includes('release-settle'), 'Pages bundle must include Step 3 release settling');
  assert.ok(
    bundle.includes('第三步已完成：横梁已落位并释放'),
    'Pages bundle must include the released Step 3 completion state',
  );
  assert.ok(
    bundle.includes('执行第四步：拾取并插入第一颗紧固件'),
    'Pages bundle must include the Step 4 action button',
  );
});
