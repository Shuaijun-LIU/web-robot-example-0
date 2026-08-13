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
  assert.equal(SO101_GEARBOX_LAYOUT.ringRadius, 0.42);
  assert.equal(SO101_GEARBOX_LAYOUT.workSurfaceHeight, 0.8);
  assert.deepEqual(SO101_GEARBOX_LAYOUT.taskStations, {
    fixture: [0, 0, 0.81],
    housing: [0, 0.21, 0.805],
    shaftsAndSpacers: [0.21, 0, 0.82],
    gears: [0, -0.21, 0.814],
    coverAndPins: [-0.21, 0, 0.809],
  });
  assert.deepEqual(SO101_GEARBOX_LAYOUT.reachEnvelope, {
    baseRadius: 0.42,
    nominalChainReach: 0.455,
    nearestStationDistance: 0.21,
    homeTcpRadius: 0.1366,
  });

  const xml = patchText(SO101_GEARBOX_LAYOUT);
  assert.match(xml, /<frame pos="0 0\.42 0\.8">/);
  assert.match(xml, /<frame pos="0\.42 0 0\.8" euler="0 0 -90">/);
  assert.match(xml, /<frame pos="0 -0\.42 0\.8" euler="0 0 180">/);
  assert.match(xml, /<frame pos="-0\.42 0 0\.8" euler="0 0 90">/);

  const assignedBases = [
    [0, 0.42],
    [0.42, 0],
    [0, -0.42],
    [-0.42, 0],
  ];
  const assignedStations = [
    SO101_GEARBOX_LAYOUT.taskStations.housing,
    SO101_GEARBOX_LAYOUT.taskStations.shaftsAndSpacers,
    SO101_GEARBOX_LAYOUT.taskStations.gears,
    SO101_GEARBOX_LAYOUT.taskStations.coverAndPins,
  ];
  const assignedDistances = assignedBases.map(([baseX, baseY], index) => {
    const [stationX, stationY] = assignedStations[index];
    return Math.hypot(stationX - baseX, stationY - baseY);
  });
  assert.ok(Math.max(...assignedDistances) <= 0.21);
  assert.equal(Math.hypot(...assignedBases[0]), 0.42);

  const fixedNames = SO101_GEARBOX_LAYOUT.sceneObjects.map(({ name }) => name);
  assert.equal(new Set(fixedNames).size, fixedNames.length);
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

  for (const name of [
    'housing_input_bearing_seat_segment_0',
    'housing_intermediate_bearing_seat_segment_0',
    'cover_input_bearing_seat_segment_0',
    'cover_intermediate_bearing_seat_segment_0',
    'input_shaft_key',
    'intermediate_shaft_key',
    'housing_rib_north_1',
    'housing_rib_south_2',
    'cover_pin_socket_1',
    'cover_pin_socket_4',
    'gear_large_keyway_site',
    'gear_medium_keyway_site',
    'gear_small_keyway_site',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
  for (const spacer of ['input', 'intermediate']) {
    assert.match(xml, new RegExp(`name="spacer_${spacer}_ring_segment_0"`));
    assert.doesNotMatch(xml, new RegExp(`name="spacer_${spacer}_body"[^>]*type="cylinder"`));
  }
});

test('XLeRobot Kitting uses two complete opposing robots and an arm-height narrow table', () => {
  assert.equal(XLEROBOT_KITTING_LAYOUT.instanceCount, 2);
  assert.equal(XLEROBOT_KITTING_LAYOUT.homeJoints.length, 32);
  assert.equal(XLEROBOT_KITTING_LAYOUT.spacing, 1);
  assert.equal(XLEROBOT_KITTING_LAYOUT.armBaseHeight, 0.775);
  assert.equal(XLEROBOT_KITTING_LAYOUT.tableTopHeight, 0.775);
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.taskStations, {
    sourceTote: [-0.15, -0.36, 0.781],
    handoffSouth: [0, -0.1, 0.787],
    handoffNorth: [0, 0.1, 0.787],
    scannerDock: [0.16, 0, 0.787],
    orderTray: [0.1, 0.36, 0.781],
  });
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.reachEnvelope, {
    chassisTableClearance: 0.05,
    inwardArmBaseToCenter: 0.41,
    nominalArmReach: 0.413,
  });

  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  assert.match(xml, /<frame pos="-0\.5 0 0" euler="0 0 180">/);
  assert.match(xml, /<frame pos="0\.5 0 0"><attach/);
  assert.match(xml, /name="chassis_rack_collision"/);
  assert.match(xml, /name="payload_deck"/);
  assert.match(xml, /name="payload_deck_left_rail"/);
  assert.match(xml, /name="payload_deck_right_rail"/);
  assert.match(xml, /<body name="payload_deck" pos="0\.255 0 0\.405">/);

  const sceneObjects = Object.fromEntries(
    XLEROBOT_KITTING_LAYOUT.sceneObjects.map((object) => [object.name, object]),
  );
  assert.deepEqual(sceneObjects.kitting_table_spine?.size, [0.24, 0.5, 0.025]);
  assert.deepEqual(sceneObjects.kitting_table_spine?.position, [0, 0, 0.75]);
  assert.deepEqual(sceneObjects.kitting_table_north_wing?.size, [0.36, 0.115, 0.025]);
  assert.deepEqual(sceneObjects.kitting_table_north_wing?.position, [0, 0.385, 0.75]);
  assert.deepEqual(sceneObjects.kitting_table_south_wing?.position, [0, -0.385, 0.75]);

  const tableClearance = 0.5 - 0.21 - sceneObjects.kitting_table_spine.size[0];
  const centerReach = 0.5 - 0.09;
  assert.ok(tableClearance >= 0.05 - Number.EPSILON);
  assert.ok(centerReach <= 0.413);
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
    'source_tote_rib_west_1',
    'source_tote_divider',
    'source_tote_label_plate',
    'pill_bottle_label',
    'tea_box_top_flap',
    'drink_carton_side_label',
    'water_bottle_label_band',
    'scanner_status_button',
    'scanner_speaker_slot_1',
    'scanner_battery_seam',
    'order_tray_corner_bumper_nw',
    'order_tray_corner_bumper_se',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
});

test('XLeRobot Kitting is an open-plan kitchen with clear mobile circulation', () => {
  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  for (const name of [
    'kitchen_back_wall',
    'kitchen_sink_cabinet',
    'kitchen_sink_basin',
    'kitchen_faucet_spout',
    'kitchen_prep_cabinet',
    'kitchen_cutting_board',
    'kitchen_stove',
    'kitchen_burner_front_left',
    'kitchen_oven',
    'kitchen_pantry',
    'dining_table',
    'storage_shelf',
    'storage_bin_1',
    'storage_bin_3',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }

  for (const bodyName of ['produce_crate', 'tomato', 'cucumber', 'bell_pepper']) {
    assert.match(xml, new RegExp(`<body name="${bodyName}"[^>]*>\\s*<freejoint\/>`));
  }
  for (const detailName of [
    'produce_crate_slatted_side',
    'tomato_stem',
    'cucumber_tip',
    'bell_pepper_lobe_4',
  ]) {
    assert.match(xml, new RegExp(`name="${detailName}"`));
  }

  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.navigationClearances, {
    north: 0.65,
    south: 0.65,
    west: 1.11,
  });
  assert.ok(XLEROBOT_KITTING_LAYOUT.navigationClearances.north >= 0.65);
  assert.ok(XLEROBOT_KITTING_LAYOUT.navigationClearances.south >= 0.65);
});

test('XLeRobot Kitting has a connected faucet and recognizable kitchen support props', () => {
  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  for (const name of [
    'kitchen_faucet_lever_pivot',
    'saucepan_bottom',
    'saucepan_wall_segment_0',
    'saucepan_rim_segment_0',
    'saucepan_handle_end_cap',
    'kitchen_trash_bin',
    'trash_bin_opening',
    'trash_bin_foot_pedal',
    'produce_table_scale',
    'produce_scale_display',
    'produce_prep_bowl',
    'prep_bowl_wall_segment_0',
    'prep_bowl_rim_segment_0',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }

  assert.doesNotMatch(xml, /name="saucepan_body"[^>]*type="cylinder"/);
  assert.doesNotMatch(xml, /name="kitchen_faucet_lever"[^>]*fromto="0\.04 0 0\.08/);
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.taskStations, {
    sourceTote: [-0.15, -0.36, 0.781],
    handoffSouth: [0, -0.1, 0.787],
    handoffNorth: [0, 0.1, 0.787],
    scannerDock: [0.16, 0, 0.787],
    orderTray: [0.1, 0.36, 0.781],
  });
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.navigationClearances, {
    north: 0.65,
    south: 0.65,
    west: 1.11,
  });
});

test('XLeRobot Kitting is enclosed by three detailed walls while keeping the south side open', () => {
  const xml = patchText(XLEROBOT_KITTING_LAYOUT);
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.roomBounds, {
    halfWidth: 2.2,
    halfDepth: 2.05,
    wallHeight: 2.4,
    openSide: 'south',
  });
  assert.deepEqual(XLEROBOT_KITTING_LAYOUT.camera, {
    position: [0, -4.8, 2.75],
    fov: 48,
  });

  for (const name of [
    'kitchen_west_wall',
    'kitchen_east_wall',
    'kitchen_west_window',
    'kitchen_west_window_glass',
    'kitchen_west_window_frame_vertical',
    'kitchen_west_window_frame_horizontal',
    'kitchen_east_wall_art_1',
    'kitchen_east_wall_art_2',
    'kitchen_side_cabinet',
    'kitchen_side_cabinet_door_upper',
    'kitchen_side_cabinet_drawer_lower',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
});

test('SO101 Gearbox expands only outside a protected central workcell', () => {
  assert.deepEqual(SO101_GEARBOX_LAYOUT.roomBounds, {
    halfWidth: 4.6,
    halfDepth: 3.6,
    wallHeight: 2.7,
    openSide: 'south',
  });
  assert.equal(SO101_GEARBOX_LAYOUT.protectedWorkcellRadius, 1.4);
  assert.deepEqual(SO101_GEARBOX_LAYOUT.roomZones, {
    lounge: [-3.35, -1.15],
    office: [2.8, 2.65],
    g1: [0.9, 2.25],
    go2Arm: [2.7, -1.5],
  });
  for (const center of Object.values(SO101_GEARBOX_LAYOUT.roomZones)) {
    assert.ok(
      Math.hypot(...center) > SO101_GEARBOX_LAYOUT.protectedWorkcellRadius,
      `zone ${center.join(',')} must remain outside the protected workcell`,
    );
  }

  assert.deepEqual(SO101_GEARBOX_LAYOUT.taskStations, {
    fixture: [0, 0, 0.81],
    housing: [0, 0.21, 0.805],
    shaftsAndSpacers: [0.21, 0, 0.82],
    gears: [0, -0.21, 0.814],
    coverAndPins: [-0.21, 0, 0.809],
  });
});

test('SO101 Gearbox room provides a furnished lounge and dual-screen office', () => {
  const xml = patchText(SO101_GEARBOX_LAYOUT);
  for (const name of [
    'gearbox_room_back_wall',
    'gearbox_room_west_wall',
    'gearbox_room_east_wall',
    'gearbox_lounge_rug',
    'gearbox_sofa',
    'gearbox_sofa_seat',
    'gearbox_sofa_back',
    'gearbox_tv_console',
    'gearbox_tv',
    'gearbox_tv_screen',
    'gearbox_side_table',
    'gearbox_floor_lamp',
    'gearbox_desk',
    'gearbox_monitor_left',
    'gearbox_monitor_right',
    'gearbox_keyboard',
    'gearbox_office_chair',
    'gearbox_room_art_1',
    'gearbox_room_art_2',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }

  const sceneObjects = Object.fromEntries(
    SO101_GEARBOX_LAYOUT.sceneObjects.map((object) => [object.name, object]),
  );
  assert.deepEqual(sceneObjects.gearbox_floor.size, [4.6, 3.6, 0.005]);
  assert.deepEqual(sceneObjects.gearbox_work_surface.size, [0.52, 0.52, 0.035]);
  assert.deepEqual(sceneObjects.gearbox_work_surface.position, [0, 0, 0.765]);
  assert.deepEqual(SO101_GEARBOX_LAYOUT.camera, {
    position: [0, -8.4, 4.2],
    fov: 47,
  });
});

test('SO101 Gearbox room attaches static G1 and Go2 arm models outside the workcell', () => {
  const xml = patchText(SO101_GEARBOX_LAYOUT);
  assert.match(
    xml,
    /<model name="g1_room_model" file="robots\/g1\/g1_static\.xml"\/>/,
  );
  assert.match(
    xml,
    /<model name="go2_arm_room_model" file="robots\/go2_arm\/go2_arm_static\.xml"\/>/,
  );
  assert.match(
    xml,
    /<frame pos="0\.9 2\.25 0" euler="0 0 -112"><attach model="g1_room_model" body="pelvis" prefix="room_g1_"\/><\/frame>/,
  );
  assert.match(
    xml,
    /<frame pos="2\.7 -1\.5 0" euler="0 0 150"><attach model="go2_arm_room_model" body="base" prefix="room_go2_"\/><\/frame>/,
  );
  assert.equal(SO101_GEARBOX_LAYOUT.staticRobots.g1.controlled, false);
  assert.equal(SO101_GEARBOX_LAYOUT.staticRobots.go2Arm.controlled, false);
});

test('neither new layout contains an object-carrying shortcut', () => {
  for (const layout of [SO101_GEARBOX_LAYOUT, XLEROBOT_KITTING_LAYOUT]) {
    const xml = patchText(layout);
    assert.doesNotMatch(xml, /<equality|<weld|magnet|auto[_-]?attach|proximity[_-]?attach/i);
  }
});
