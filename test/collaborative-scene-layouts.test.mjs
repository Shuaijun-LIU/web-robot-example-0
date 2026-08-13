import assert from 'node:assert/strict';
import test from 'node:test';

import * as collaborationLayouts from '../src/collaborativeSceneLayouts.js';
import { SO101_LAYOUT, XLEROBOT_LAYOUT } from '../src/sceneLayouts.js';

const {
  SO101_GEARBOX_LAYOUT,
  SO101_HOME_LAB_LAYOUT,
  XLEROBOT_KITTING_LAYOUT,
} = collaborationLayouts;

function patchText(layout) {
  return layout.xmlPatches
    .map((patch) => patch.replace?.join('\n') ?? patch.inject ?? '')
    .join('\n');
}

function namedXmlTag(xml, name) {
  const match = xml.match(new RegExp(`<[^>]+name="${name}"[^>]*>`));
  assert.ok(match, `missing MJCF element ${name}`);
  return match[0];
}

function vectorAttribute(xml, name, attribute) {
  const tag = namedXmlTag(xml, name);
  const match = tag.match(new RegExp(`${attribute}="([^"]+)"`));
  assert.ok(match, `missing ${attribute} on ${name}`);
  return match[1].trim().split(/\s+/).map(Number);
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

test('SO101 Gearbox restores the compact pre-room scene while Home Lab is independent', () => {
  const compactXml = patchText(SO101_GEARBOX_LAYOUT);
  const compactObjects = Object.fromEntries(
    SO101_GEARBOX_LAYOUT.sceneObjects.map((object) => [object.name, object]),
  );

  assert.deepEqual(compactObjects.gearbox_floor.size, [2, 2, 0.005]);
  assert.deepEqual(SO101_GEARBOX_LAYOUT.camera, {
    position: [1.15, -1.15, 1.28],
    fov: 40,
  });
  assert.deepEqual(SO101_GEARBOX_LAYOUT.orbitTarget, [0, 0, 0.81]);
  assert.doesNotMatch(compactXml, /g1_room_model|go2_arm_room_model|gearbox_room_back_wall/);

  assert.ok(SO101_HOME_LAB_LAYOUT, 'SO101 Home Lab must be exported');
  assert.notStrictEqual(SO101_HOME_LAB_LAYOUT, SO101_GEARBOX_LAYOUT);
  assert.equal(SO101_HOME_LAB_LAYOUT.instanceCount, 4);
  assert.equal(SO101_HOME_LAB_LAYOUT.homeJoints.length, 30);
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.homeJoints.slice(0, 6), [0, 0, 0, 0, 0, 0]);
  assert.notDeepEqual(SO101_HOME_LAB_LAYOUT.taskStations, SO101_GEARBOX_LAYOUT.taskStations);
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

test('XLeRobot Kitting uses a compact white wall refrigerator and dark west-facing bin', () => {
  const xml = patchText(XLEROBOT_KITTING_LAYOUT);

  assert.match(xml, /<body name="storage_shelf" pos="-1\.92 -0\.4 0">/);
  assert.match(xml, /<body name="kitchen_refrigerator" pos="1\.8 -0\.1 0">/);
  for (const name of [
    'refrigerator_upper_door',
    'refrigerator_freezer_drawer',
    'refrigerator_water_dispenser',
    'refrigerator_upper_handle',
    'refrigerator_freezer_handle',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
  assert.match(
    xml,
    /name="refrigerator_case"[^>]*pos="0 0 0\.72"[^>]*size="0\.28 0\.3 0\.72"[^>]*rgba="1 1 1 1"/,
  );
  assert.match(
    xml,
    /name="refrigerator_upper_door"[^>]*size="0\.014 0\.275 0\.34"[^>]*rgba="1 1 1 1"/,
  );
  assert.match(
    xml,
    /name="refrigerator_upper_handle"[^>]*rgba="0\.14 0\.16 0\.16 1"/,
  );
  assert.doesNotMatch(xml, /name="kitchen_side_cabinet"/);

  assert.match(xml, /<body name="kitchen_trash_bin" pos="1\.55 -1\.25 0" euler="0 0 -90">/);
  assert.match(
    xml,
    /name="trash_bin_lid"[^>]*pos="0 0\.07 0\.81"[^>]*euler="-58 0 0"/,
  );
  for (const [name, rgba] of [
    ['trash_bin_base', '0.1 0.12 0.12 1'],
    ['trash_bin_wall_west', '0.17 0.19 0.19 1'],
    ['trash_bin_wall_south', '0.2 0.22 0.21 1'],
    ['trash_bin_lid', '0.13 0.15 0.15 1'],
  ]) {
    assert.match(xml, new RegExp(`name="${name}"[^>]*rgba="${rgba}"`));
  }

  assert.match(xml, /name="produce_scale_platform"[^>]*rgba="0\.86 0\.88 0\.84 1"/);
  assert.match(xml, /name="prep_bowl_bottom"[^>]*rgba="0\.76 0\.82 0\.8 1"/);
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
    'kitchen_refrigerator',
    'refrigerator_upper_door',
    'refrigerator_freezer_drawer',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }
});

test('SO101 Home Lab moves the complete SO101 workcell into the open southwest zone', () => {
  assert.ok(SO101_HOME_LAB_LAYOUT, 'SO101 Home Lab must be exported');
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.roomBounds, {
    halfWidth: 5,
    halfDepth: 4.2,
    wallHeight: 2.7,
    openSide: 'south',
  });
  assert.equal(SO101_HOME_LAB_LAYOUT.protectedWorkcellRadius, 1.15);
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.workcellCenter, [-2.25, -1.85]);
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.roomZones, {
    lounge: [-3.55, 1.55],
    office: [2.65, 3.35],
    g1: [2.45, -0.9],
    go2Arm: [3.55, -2.35],
  });
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.taskStations, {
    fixture: [-2.25, -1.85, 0.81],
    housing: [-2.25, -1.64, 0.805],
    shaftsAndSpacers: [-2.04, -1.85, 0.82],
    gears: [-2.25, -2.06, 0.814],
    coverAndPins: [-2.46, -1.85, 0.809],
  });

  const xml = patchText(SO101_HOME_LAB_LAYOUT);
  assert.match(xml, /<frame pos="-2\.25 -1\.85 0">[\s\S]*?<attach model="so101_model"/);
});

test('SO101 Home Lab provides a furnished lounge and dual-screen office', () => {
  assert.ok(SO101_HOME_LAB_LAYOUT, 'SO101 Home Lab must be exported');
  const xml = patchText(SO101_HOME_LAB_LAYOUT);
  for (const name of [
    'home_lab_room_back_wall',
    'home_lab_room_west_wall',
    'home_lab_room_east_wall',
    'home_lab_lounge_rug',
    'home_lab_sofa',
    'home_lab_sofa_seat',
    'home_lab_sofa_back',
    'home_lab_tv_console',
    'home_lab_tv',
    'home_lab_tv_screen',
    'home_lab_side_table',
    'home_lab_floor_lamp',
    'home_lab_office_desk',
    'home_lab_monitor_left',
    'home_lab_monitor_right',
    'home_lab_keyboard',
    'home_lab_office_chair',
    'home_lab_room_art_1',
    'home_lab_room_art_2',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }

  const sceneObjects = Object.fromEntries(
    SO101_HOME_LAB_LAYOUT.sceneObjects.map((object) => [object.name, object]),
  );
  assert.deepEqual(sceneObjects.gearbox_floor.size, [5, 4.2, 0.005]);
  assert.deepEqual(sceneObjects.gearbox_work_surface.size, [0.52, 0.52, 0.035]);
  assert.deepEqual(sceneObjects.gearbox_work_surface.position, [-2.25, -1.85, 0.765]);
  assert.deepEqual(SO101_HOME_LAB_LAYOUT.camera, {
    position: [0, -9.2, 4.8],
    fov: 48,
  });
});

test('SO101 Home Lab exposes detailed residential, office, and robot-service objects', () => {
  assert.ok(SO101_HOME_LAB_LAYOUT, 'SO101 Home Lab must be exported');
  const xml = patchText(SO101_HOME_LAB_LAYOUT);

  for (const name of [
    'home_lab_west_window',
    'home_lab_west_window_glass',
    'home_lab_north_window',
    'home_lab_east_door',
    'home_lab_east_door_handle',
    'home_lab_ceiling_light_west',
    'home_lab_ceiling_light_east',
    'home_lab_coffee_table',
    'home_lab_coffee_table_lower_shelf',
    'home_lab_sofa_seam_1',
    'home_lab_sofa_back_cushion_1',
    'home_lab_sofa_back_cushion_2',
    'home_lab_sofa_front_apron',
    'home_lab_tv_left_speaker',
    'home_lab_tv_right_speaker',
    'home_lab_tv_remote',
    'home_lab_coffee_mug',
    'home_lab_mouse',
    'home_lab_mouse_pad',
    'home_lab_pc_tower',
    'home_lab_pc_vent_1',
    'home_lab_cable_tray',
    'home_lab_desk_book_1',
    'home_lab_wall_organizer',
    'home_lab_g1_status_pedestal',
    'home_lab_maintenance_cabinet',
    'home_lab_maintenance_drawer_1',
    'home_lab_tool_board',
    'home_lab_tool_wrench',
    'home_lab_service_cart',
    'home_lab_robot_warning_band_1',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"`));
  }

  assert.doesNotMatch(xml, /home_lab_sofa_pillow_/);
  assert.match(xml, /<body name="home_lab_sofa" pos="-4\.30 1\.35 0">/);
  assert.match(xml, /<body name="home_lab_side_table" pos="-4\.30 2\.85 0">/);
  assert.match(xml, /<body name="home_lab_coffee_table" pos="-2\.95 1\.35 0">/);
  assert.match(xml, /<body name="home_lab_tv_console" pos="-1\.82 1\.35 0">/);

  assert.doesNotMatch(xml, /name="gearbox_tv_console"|name="gearbox_desk"/);
  assert.doesNotMatch(
    xml,
    /home_lab_g1_display_pad|home_lab_go2_charging_pad|home_lab_go2_charge_contact/,
  );
  for (let wheel = 1; wheel <= 4; wheel += 1) {
    assert.match(
      xml,
      new RegExp(`name="home_lab_service_cart_wheel_${wheel}"[^>]*pos="[^"]+ 0\\.075"[^>]*size="0\\.075 0\\.03"`),
      `service cart wheel ${wheel} must exist and touch the floor`,
    );
  }
});

test('SO101 Home Lab chair has complete armrests and media equipment sits on the console', () => {
  const xml = patchText(SO101_HOME_LAB_LAYOUT);
  for (const name of [
    'home_lab_office_chair_arm_support_front_left',
    'home_lab_office_chair_arm_support_rear_left',
    'home_lab_office_chair_arm_support_front_right',
    'home_lab_office_chair_arm_support_rear_right',
    'home_lab_office_chair_arm_rail_left',
    'home_lab_office_chair_arm_rail_right',
    'home_lab_office_chair_arm_pad_left',
    'home_lab_office_chair_arm_pad_right',
  ]) {
    namedXmlTag(xml, name);
  }

  const consolePosition = vectorAttribute(xml, 'home_lab_tv_console', 'pos');
  const consoleTopPosition = vectorAttribute(xml, 'home_lab_tv_console_top', 'pos');
  const consoleTopSize = vectorAttribute(xml, 'home_lab_tv_console_top', 'size');
  const consoleTopElevation = consolePosition[2] + consoleTopPosition[2] + consoleTopSize[2];
  const consoleX = [
    consolePosition[0] - consoleTopSize[0],
    consolePosition[0] + consoleTopSize[0],
  ];
  const consoleY = [
    consolePosition[1] - consoleTopSize[1],
    consolePosition[1] + consoleTopSize[1],
  ];

  const tvPosition = vectorAttribute(xml, 'home_lab_tv', 'pos');
  const tvStandPosition = vectorAttribute(xml, 'home_lab_tv_stand', 'pos');
  const tvStandSize = vectorAttribute(xml, 'home_lab_tv_stand', 'size');
  const tvPanelSize = vectorAttribute(xml, 'home_lab_tv_panel', 'size');
  const tvStandBottom = tvPosition[2] + tvStandPosition[2] - tvStandSize[2];
  assert.ok(Math.abs(tvStandBottom - consoleTopElevation) < 1e-9);
  assert.ok(tvPosition[0] - tvStandSize[0] >= consoleX[0]);
  assert.ok(tvPosition[0] + tvStandSize[0] <= consoleX[1]);

  const speakerRanges = ['left', 'right'].map((side) => {
    const position = vectorAttribute(xml, `home_lab_tv_${side}_speaker`, 'pos');
    const size = vectorAttribute(xml, `home_lab_tv_${side}_speaker_case`, 'size');
    assert.ok(Math.abs(position[2] - size[2] - consoleTopElevation) < 1e-9);
    assert.ok(position[0] - size[0] >= consoleX[0]);
    assert.ok(position[0] + size[0] <= consoleX[1]);
    assert.ok(position[1] - size[1] >= consoleY[0]);
    assert.ok(position[1] + size[1] <= consoleY[1]);
    return [position[1] - size[1], position[1] + size[1]];
  });
  const tvPanelY = [tvPosition[1] - tvPanelSize[1], tvPosition[1] + tvPanelSize[1]];
  assert.ok(speakerRanges[0][1] < tvPanelY[0], 'left speaker must clear the TV panel');
  assert.ok(speakerRanges[1][0] > tvPanelY[1], 'right speaker must clear the TV panel');
});

test('SO101 Home Lab attaches G1 and Go2 arm models to actuated planar mobile roots', () => {
  assert.ok(SO101_HOME_LAB_LAYOUT, 'SO101 Home Lab must be exported');
  const xml = patchText(SO101_HOME_LAB_LAYOUT);
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
    /<body name="home_lab_g1_mobile_root" pos="2\.45 -0\.9 0">[\s\S]*?<joint name="home_lab_g1_x" type="slide"[\s\S]*?<joint name="home_lab_g1_yaw" type="hinge"[\s\S]*?<frame euler="0 0 155"><attach model="g1_room_model" body="pelvis" prefix="room_g1_"\/><\/frame>/,
  );
  assert.match(
    xml,
    /<body name="home_lab_go2_mobile_root" pos="3\.55 -2\.35 0">[\s\S]*?<joint name="home_lab_go2_x" type="slide"[\s\S]*?<joint name="home_lab_go2_yaw" type="hinge"[\s\S]*?<frame euler="0 0 150"><attach model="go2_arm_room_model" body="base" prefix="room_go2_"\/><\/frame>/,
  );
  for (const actuator of [
    'home_lab_g1_velocity_x',
    'home_lab_g1_velocity_y',
    'home_lab_g1_velocity_yaw',
    'home_lab_go2_velocity_x',
    'home_lab_go2_velocity_y',
    'home_lab_go2_velocity_yaw',
  ]) {
    assert.match(xml, new RegExp(`<velocity name="${actuator}"`));
  }
  assert.equal(SO101_HOME_LAB_LAYOUT.mobileRobots.g1.controlled, true);
  assert.equal(SO101_HOME_LAB_LAYOUT.mobileRobots.go2Arm.controlled, true);
});

test('no collaboration layout contains an object-carrying shortcut', () => {
  for (const layout of [
    SO101_GEARBOX_LAYOUT,
    SO101_HOME_LAB_LAYOUT,
    XLEROBOT_KITTING_LAYOUT,
  ]) {
    const xml = patchText(layout);
    assert.doesNotMatch(xml, /<equality|<weld|magnet|auto[_-]?attach|proximity[_-]?attach/i);
  }
});
