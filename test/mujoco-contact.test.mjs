import assert from 'node:assert/strict';
import test from 'node:test';

import { consumeMujocoContact, consumeMujocoContacts } from '../src/mujocoContact.js';

test('contact reads release the temporary WASM handle', () => {
  let deletes = 0;
  const pair = consumeMujocoContact(() => ({
    geom1: 12,
    geom2: 34,
    delete: () => { deletes += 1; },
  }), 0);

  assert.deepEqual(pair, { geom1: 12, geom2: 34 });
  assert.equal(deletes, 1);
});

test('contact handles are released even when field access throws', () => {
  let deletes = 0;
  const contact = {
    get geom1() { throw new Error('bad contact'); },
    geom2: 34,
    delete: () => { deletes += 1; },
  };

  assert.throws(() => consumeMujocoContact(() => contact, 0), /bad contact/);
  assert.equal(deletes, 1);
});

test('missing contacts remain missing without a release attempt', () => {
  assert.equal(consumeMujocoContact(() => undefined, 0), null);
});

test('contact array reads release both item handles and the vector wrapper', () => {
  let itemDeletes = 0;
  let vectorDeletes = 0;
  const contacts = [
    { geom1: 1, geom2: 2, delete: () => { itemDeletes += 1; } },
    { geom1: 3, geom2: 4, delete: () => { itemDeletes += 1; } },
  ];
  const vector = {
    get: (index) => contacts[index],
    delete: () => { vectorDeletes += 1; },
  };

  assert.deepEqual(consumeMujocoContacts(vector, contacts.length), [
    { geom1: 1, geom2: 2 },
    { geom1: 3, geom2: 4 },
  ]);
  assert.equal(itemDeletes, 2);
  assert.equal(vectorDeletes, 1);
});
