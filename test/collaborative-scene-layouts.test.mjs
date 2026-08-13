import assert from 'node:assert/strict';
import test from 'node:test';

import {
  SO101_GEARBOX_LAYOUT,
  XLEROBOT_KITTING_LAYOUT,
} from '../src/collaborativeSceneLayouts.js';
import { SO101_LAYOUT, XLEROBOT_LAYOUT } from '../src/sceneLayouts.js';

function patchText(layout) {
  return layout.xmlPatches
    .map((patch) => patch.replace?.join('\n') ?? patch.inject ?? '')
    .join('\n');
}

test('new collaboration layouts do not mutate the original SO101 and XLeRobot scenes', () => {
  assert.notStrictEqual(SO101_GEARBOX_LAYOUT, SO101_LAYOUT);
  assert.notStrictEqual(XLEROBOT_KITTING_LAYOUT, XLEROBOT_LAYOUT);
  assert.equal(SO101_LAYOUT.sceneObjects.some(({ name }) => name.includes('gearbox')), false);
  assert.equal(XLEROBOT_LAYOUT.sceneObjects.some(({ name }) => name.includes('kitting')), false);
  assert.equal(SO101_LAYOUT.ringRadius, 0.34);
  assert.equal(XLEROBOT_LAYOUT.spacing, 1.7);
});

test('SO101 Gearbox keeps four independently controllable arms around exact task stations', () => {
  assert.equal(SO101_GEARBOX_LAYOUT.instanceCount, 4);
  assert.equal(SO101_GEARBOX_LAYOUT.homeJoints.length, 24);
  assert.equal(SO101_GEARBOX_LAYOUT.ringRadius, 0.34);
  assert.equal(SO101_GEARBOX_LAYOUT.workSurfaceHeight, 0.8);
  assert.deepEqual(SO101_GEARBOX_LAYOUT.taskStations, {
    fixture: [0, 0, 0.81],
    housing: [0, 0.21, 0.805],
    shaftsAndSpacers: [0.21, 0, 0.82],
    gears: [0, -0.21, 0.814],
    coverAndPins: [-0.21, 0, 0.809],
  });

  const xml = patchText(SO101_GEARBOX_LAYOUT);
  assert.match(xml, /<frame pos="0 0\.34 0\.8">/);
  assert.match(xml, /<frame pos="0\.34 0 0\.8" euler="0 0 -90">/);
  assert.match(xml, /<frame pos="0 -0\.34 0\.8" euler="0 0 180">/);
  assert.match(xml, /<frame pos="-0\.34 0 0\.8" euler="0 0 90">/);
});

test('SO101 Gearbox contains a future-usable housing, cover, shafts, open gears and press pins', () => {
  const xml = patchText(SO101_GEARBOX_LAYOUT);
  for (const bodyName of [
    'gearbox_lower_housing',
    'gearbox_top_cover',
    'input_shaft',
    'intermediate_shaft',
    'gear_large',
    'gear_medium',
    'gear_small',
    'spacer_input',
    'spacer_intermediate',
    'press_pin_1',
    'press_pin_2',
    'press_pin_3',
    'press_pin_4',
  ]) {
    assert.match(
      xml,
      new RegExp(`<body name="${bodyName}"[^>]*>\\s*<freejoint\\/>`),
      `${bodyName} must remain a contact-driven free body`,
    );
  }

  for (const fixedName of [
    'gearbox_fixture',
    'shaft_rack_input',
    'shaft_rack_intermediate',
    'gear_parts_tray',
    'pin_holder',
    'pin_holder_support_1_left',
    'pin_holder_support_1_right',
    'pin_holder_support_4_left',
    'pin_holder_support_4_right',
  ]) {
    assert.match(xml, new RegExp(`name="${fixedName}"`));
  }

  for (const gear of ['large', 'medium', 'small']) {
    assert.match(xml, new RegExp(`name="gear_${gear}_ring_segment_0"`));
    assert.match(xml, new RegExp(`name="gear_${gear}_tooth_0"`));
    assert.doesNotMatch(xml, new RegExp(`name="gear_${gear}_collision"[^>]*type="cylinder"`));
  }
  for (const siteName of [
    'housing_input_bearing_site',
    'housing_intermediate_bearing_site',
    'cover_input_bearing_site',
    'cover_intermediate_bearing_site',
  ]) {
    assert.match(xml, new RegExp(`site name="${siteName}"`));
  }
});

test('XLeRobot Kitting uses two complete opposing robots and an arm-height narrow table', () => {
  assert.equal(XLEROBOT_KITTING_LAYOUT.instanceCount, 2);
  assert.equal(XLEROBOT_KITTING_LAYOUT.homeJoints.length, 32);
  assert.equal(XLEROBOT_KITTING_LAYOUT.spacing, 2.2);
  assert.equal(XLEROBOT_KITTING_LAYOUT.armBaseHeight, 0.775);
  assert.equal(XLEROBOT_KITTING_LAYOUT.tableTopHeight, 0.775);
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.taskStations, {
    sourceTote: [-0.18, -0.3, 0.781],
    handoffSouth: [0, -0.09, 0.787],
    handoffNorth: [0, 0.09, 0.787],
    scannerDock: [0.22, 0, 0.787],
    orderTray: [0.1, 0.32, 0.781],
  });

  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  assert.match(xml, /<frame pos="-1\.1 0 0" euler="0 0 180">/);
  assert.match(xml, /<frame pos="1\.1 0 0"><attach/);
  assert.match(xml, /name="chassis_rack_collision"/);
  assert.match(xml, /name="payload_deck"/);
  assert.match(xml, /name="payload_deck_left_rail"/);
  assert.match(xml, /name="payload_deck_right_rail"/);

  const table = XLEROBOT_KITTING_LAYOUT.sceneObjects.find(
    ({ name }) => name === 'kitting_table_top',
  );
  assert.deepEqual(table?.size, [0.36, 0.5, 0.025]);
  assert.deepEqual(table?.position, [0, 0, 0.75]);
});

test('XLeRobot Kitting stages physical goods, scanner, handoff cradles and a divided tray', () => {
  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  for (const bodyName of [
    'source_tote',
    'order_tray',
    'handheld_scanner',
    'pill_bottle',
    'tea_box',
    'drink_carton',
    'water_bottle',
  ]) {
    assert.match(
      xml,
      new RegExp(`<body name="${bodyName}"[^>]*>\\s*<freejoint\\/>`),
      `${bodyName} must remain a contact-driven free body`,
    );
  }

  for (const name of [
    'handoff_cradle_south',
    'handoff_cradle_north',
    'scanner_dock',
    'order_tray_divider_a',
    'order_tray_divider_b',
    'order_tray_handle_left',
    'order_tray_handle_right',
    'scanner_lens_site',
    'pill_bottle_barcode_site',
    'tea_box_barcode_site',
    'drink_carton_barcode_site',
    'water_bottle_barcode_site',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
});

test('neither new layout contains an object-carrying shortcut', () => {
  for (const layout of [SO101_GEARBOX_LAYOUT, XLEROBOT_KITTING_LAYOUT]) {
    const xml = patchText(layout);
    assert.doesNotMatch(xml, /<equality|<weld|magnet|auto[_-]?attach|proximity[_-]?attach/i);
  }
});
