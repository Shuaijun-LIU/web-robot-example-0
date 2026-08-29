import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import {
  applyAssemblyTargetPose,
  FRANKA_ASSEMBLY1_LAYOUT,
  FRANKA_ASSEMBLY2_LAYOUT,
  FRANKA_ASSEMBLY_INTERFACE,
  SHARED_ASSEMBLY1_TOOL_XML,
} from '../src/frankaAssemblyLayouts.js';

const layoutXml = (layout) => layout.xmlPatches
  .map((patch) => [patch.inject, patch.replace?.[1]].filter(Boolean).join('\n'))
  .join('\n');

test('both assembly strategies preserve the same four-arm workcell envelope', () => {
  for (const layout of [FRANKA_ASSEMBLY1_LAYOUT, FRANKA_ASSEMBLY2_LAYOUT]) {
    assert.equal(layout.instanceCount, 4);
    assert.equal(layout.ringRadius, 0.9);
    assert.equal(layout.homeJoints.length, 32);
    assert.deepEqual(layout.taskStations, {
      frame: [0, 0, 0.275],
      parts: [-0.56, 0.42, 0.125],
      poweredTool: layout === FRANKA_ASSEMBLY1_LAYOUT
        ? [0.65, 0, 0.238]
        : [0.53, -0.42, 0.135],
      manualTool: [-0.53, -0.42, 0.13],
      hammer: layout === FRANKA_ASSEMBLY1_LAYOUT
        ? [0.642, -0.421, 0.144]
        : [0.65, 0, 0.229],
      fasteners: layout === FRANKA_ASSEMBLY1_LAYOUT
        ? [0.18, 0.48, 0.125]
        : [0.56, 0.42, 0.125],
      handover: [0, -0.48, 0.112],
    });
  }
});

test('Assembly1 isolates one target fastener from two spare fasteners in Arm 3 reach', () => {
  const assembly1Xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  const assembly2Xml = layoutXml(FRANKA_ASSEMBLY2_LAYOUT);
  assert.match(assembly1Xml, /<body name="fastener_tray" pos="\.18 \.48 \.11">/);
  assert.match(assembly1Xml, /name="fastener_tray_floor" type="box" pos="0 0 -\.03" size="\.18 \.18 \.04"/);
  assert.doesNotMatch(assembly1Xml, /fastener_1_guide_/);
  assert.match(assembly1Xml, /<body name="fastener_1" pos="\.10 \.38 \.152"><freejoint\/>/);
  assert.match(assembly1Xml, /<body name="fastener_2" pos="\.14 \.55 \.152">/);
  assert.match(assembly1Xml, /<body name="fastener_3" pos="\.24 \.55 \.152">/);
  assert.doesNotMatch(assembly1Xml, /<body name="fastener_4"/);
  for (let fastener = 1; fastener <= 3; fastener += 1) {
    assert.match(
      assembly1Xml,
      new RegExp(`name="fastener_${fastener}_shaft" type="cylinder" size="\\.007 \\.025"[^>]*mass="\\.012"`),
    );
    assert.match(
      assembly1Xml,
      new RegExp(`name="fastener_${fastener}_head" type="cylinder" pos="0 0 \\.032" size="\\.015 \\.007"[^>]*mass="\\.006"`),
    );
  }
  assert.doesNotMatch(assembly1Xml, /fastener_1_retention_cap|fastener_1_guide_/);
  assert.match(assembly2Xml, /<body name="fastener_tray" pos="\.56 \.42 \.11">/);
  assert.match(assembly2Xml, /<body name="fastener_1" pos="\.50 \.36 \.152">/);
  assert.match(assembly2Xml, /name="fastener_1_head"[^>]+size="\.015 \.007"/);
});

test('Assembly1 positions Arms 3/4 to reach both staging and the central interface', () => {
  const assembly1Xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  const assembly2Xml = layoutXml(FRANKA_ASSEMBLY2_LAYOUT);
  assert.match(
    assembly1Xml,
    /<frame pos="-0\.3 0\.85 0\.1" euler="0 0 180"><attach model="panda_model" body="link0" prefix="r2_"\/><\/frame>/,
  );
  assert.match(
    assembly1Xml,
    /<frame pos="-0\.8 0 0\.1" euler="0 0 -90"><attach model="panda_model" body="link0" prefix="r3_"\/><\/frame>/,
  );
  assert.match(
    assembly2Xml,
    /<frame pos="0 0\.9 0\.1" euler="0 0 180"><attach model="panda_model" body="link0" prefix="r2_"\/><\/frame>/,
  );
  assert.match(
    assembly2Xml,
    /<frame pos="-0\.9 0 0\.1" euler="0 0 -90"><attach model="panda_model" body="link0" prefix="r3_"\/><\/frame>/,
  );
});

test('cross-member target pose aligns all four installation holes with frame receivers', () => {
  const expected = [
    [-0.04, 0.215, 0.275],
    [0.04, 0.215, 0.275],
    [-0.04, -0.215, 0.275],
    [0.04, -0.215, 0.275],
  ];
  assert.deepEqual(FRANKA_ASSEMBLY_INTERFACE.frameReceiverPositions, expected);
  assert.deepEqual(FRANKA_ASSEMBLY_INTERFACE.crossMemberTargetPose, [0, 0, 0.278]);
  assert.ok(FRANKA_ASSEMBLY_INTERFACE.crossMemberHoleLocalPositions.every(
    (position) => position[2] === -0.003,
  ));

  const actual = applyAssemblyTargetPose(
    FRANKA_ASSEMBLY_INTERFACE.crossMemberHoleLocalPositions,
    FRANKA_ASSEMBLY_INTERFACE.crossMemberTargetPose,
  );
  actual.forEach((point, pointIndex) => point.forEach((value, axis) => {
    assert.ok(Math.abs(value - expected[pointIndex][axis]) < 1e-9);
  }));
});

test('cross-member target rests on top of the frame instead of intersecting its rails', () => {
  const frameTop = 0.235 + 0.025;
  const crossMemberBottom = FRANKA_ASSEMBLY_INTERFACE.crossMemberTargetPose[2] - 0.018;
  assert.ok(crossMemberBottom >= frameTop - 1e-9);
});

test('Assembly1 south frame rail exposes a nominal 50 mm physical grasp profile', () => {
  assert.match(
    layoutXml(FRANKA_ASSEMBLY1_LAYOUT),
    /name="frame_rail_south_outer"[^>]*size="\.34 \.018 \.025"/,
  );
});

test('cross-member uses underside grasp pockets and integral hollow round/square plates', () => {
  const xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  const pickupGuides = {
    west: ['.043 0 .07', '.005 .05 .01'],
    east: ['.117 0 .07', '.005 .05 .01'],
    north: ['.08 .262 .07', '.08 .005 .01'],
    south: ['.08 -.262 .07', '.08 .005 .01'],
  };
  for (const [name, [position, size]] of Object.entries(pickupGuides)) {
    assert.match(xml, new RegExp(
      `name="cross_member_pickup_guide_${name}"[^>]*pos="${position}"[^>]*size="${size}"[^>]*friction="2 \.2 \.03"`,
    ));
  }
  for (const name of [
    'cross_member_grip_stop_north_outer',
    'cross_member_grip_stop_north_inner',
    'cross_member_grip_stop_south_outer',
    'cross_member_grip_stop_south_inner',
  ]) {
    assert.match(xml, new RegExp(`name="${name}"[^>]*mass="\\.005"`));
  }
  assert.doesNotMatch(xml, /cross_member_grip_cap_(?:north|south)/);
  for (const side of ['north', 'south']) {
    assert.match(xml, new RegExp(
      `name="cross_member_grip_recess_bridge_${side}"[^>]*size="\\.012 \\.025 \\.012"[^>]*friction="10 2 1"[^>]*condim="6"[^>]*solref="\\.002 1"`,
    ));
    assert.match(xml, new RegExp(
      `name="cross_member_grip_lower_guard_${side}"[^>]*size="\\.026 \\.021 \\.004"[^>]*mass="\\.002"[^>]*friction="10 2 1"[^>]*condim="6"`,
    ));
    assert.match(xml, new RegExp(
      `name="cross_member_grip_upper_guard_${side}"[^>]*pos="0 ${side === 'north' ? '\\.1275' : '-\\.1275'} \\.024"[^>]*size="\\.026 \\.021 \\.004"[^>]*mass="\\.002"[^>]*friction="10 2 1"[^>]*condim="6"`,
    ));
    for (const opening of ['left', 'right']) {
      assert.match(xml, new RegExp(
        `name="cross_member_${side}_hole_${opening}"[^>]*rgba="0 0 0 0"`,
      ));
    }
    for (const flange of ['left', 'right']) {
      assert.match(xml, new RegExp(
        `name="cross_member_flange_${flange}_${side}_grip_recess"[^>]*pos="[^\"]+ -?0?\\.1275 0?\\.006"[^>]*size="\\.0045 0?\\.025 0?\\.012"[^>]*friction="10 2 1"[^>]*condim="6"[^>]*solref="\\.002 1"`,
      ));
    }
    for (const edge of ['outer', 'inner', 'left', 'center', 'right']) {
      assert.match(xml, new RegExp(
        `name="cross_member_${side}_plate_${edge}"[^>]*pos="[^\"]+ \\.033"[^>]*size="[^\"]+ \\.015"`,
      ));
    }
    assert.equal(
      (xml.match(new RegExp(`cross_member_${side}_round_opening_segment_`, 'g')) ?? []).length,
      12,
    );
    assert.match(xml, new RegExp(`name="cross_member_${side}_square_opening"`));
  }
});

test('Assembly1 uses the Assembly2 RoboTwin tools while retaining the legacy tool set', () => {
  const xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  for (const tool of ['screwdriver', 'drill', 'hammer']) {
    for (const role of ['primary', 'dark', 'metal']) {
      assert.match(xml, new RegExp(`mesh="robotwin_${tool}_${role}"`));
    }
    assert.match(xml, new RegExp(`name="robotwin_${tool}_collision"`));
  }
  assert.doesNotMatch(xml, /manual_screwdriver_octagonal_handle|hammer_handle_core/);
  assert.match(SHARED_ASSEMBLY1_TOOL_XML, /manual_screwdriver_octagonal_handle/);
  assert.match(SHARED_ASSEMBLY1_TOOL_XML, /name="hammer_handle_core"/);
  assert.match(
    xml,
    /<body name="torque_driver" pos="\.65 0 \.222">/,
  );
  assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
    ({ name, size, position }) => name === 'hammer_pickup_cradle_tail'
      && JSON.stringify(size) === JSON.stringify([.035, .04, .015])
      && JSON.stringify(position) === JSON.stringify([.50, -.421, .108]),
  ));
  assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
    ({ name, size, position }) => name === 'hammer_pickup_cradle_head'
      && JSON.stringify(size) === JSON.stringify([.035, .05, .015])
      && JSON.stringify(position) === JSON.stringify([.745, -.421, .106]),
  ));
  assert.ok(!FRANKA_ASSEMBLY2_LAYOUT.sceneObjects.some(
    ({ name }) => name.startsWith('torque_driver_cradle_'),
  ));
  assert.doesNotMatch(
    layoutXml(FRANKA_ASSEMBLY2_LAYOUT),
    /<body name="torque_driver" pos="\.53 -\.42 \.166" euler="90 0 0">/,
  );
  assert.ok(!FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(({ name }) => name === 'tool_mat_hammer'));
  assert.match(xml, /<body name="double_face_hammer" pos="\.642 -\.421 \.147" gravcomp="\.99">/);
  assert.match(
    xml,
    /name="robotwin_hammer_collision" type="box"[^>]*pos="-\.05 0 -\.008"[^>]*size="\.11 \.020 \.016"[^>]*mass="\.025"[^>]*friction="20 3 1"[^>]*condim="6"[^>]*solref="\.002 1"/,
  );
  assert.match(xml, /name="robotwin_hammer_head_collision"[^>]*mass="\.025"[^>]*friction="20 3 1"/);
  assert.doesNotMatch(xml, /robotwin_hammer_(?:handle_extension|receiver_guard|grip_guard)/);
  assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
    ({ name, size, position }) => name === 'tool_mat_powered'
      && JSON.stringify(size) === JSON.stringify([.2, .13, .006])
      && JSON.stringify(position) === JSON.stringify([.65, 0, .112]),
  ));
  assert.doesNotMatch(xml, /<body name="claw_hammer"/);
});

test('Assembly layouts strengthen physical finger contact without attachment', () => {
  assert.match(
    layoutXml(FRANKA_ASSEMBLY1_LAYOUT),
    /name="gripper" tendon="split" forcerange="-180 180"/,
  );
  assert.match(
    layoutXml(FRANKA_ASSEMBLY2_LAYOUT),
    /name="gripper" tendon="split" forcerange="-100 100"/,
  );
  assert.match(
    layoutXml(FRANKA_ASSEMBLY1_LAYOUT),
    /gainprm="\.23529411765 0 0" biasprm="0 -1500 -40"/,
  );
  assert.match(
    layoutXml(FRANKA_ASSEMBLY2_LAYOUT),
    /gainprm="\.23529411765 0 0" biasprm="0 -1500 -40"/,
  );
  for (const layout of [FRANKA_ASSEMBLY1_LAYOUT, FRANKA_ASSEMBLY2_LAYOUT]) {
    const xml = layoutXml(layout);
    for (let pad = 1; pad <= 5; pad += 1) {
      assert.match(
        xml,
        new RegExp(
          `fingertip_pad_collision_${pad}[\\s\\S]*?friction="10 \\.5 \\.1" condim="6" solref="\\.002 1"`,
        ),
      );
    }
    assert.doesNotMatch(xml, /weld|equality[^>]*tool|attach_tool|magnet/i);
  }
});

test('Assembly automation avoids delayed per-frame gravity-force feedback', async () => {
  const assembly1Xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  const assembly2Xml = layoutXml(FRANKA_ASSEMBLY2_LAYOUT);
  assert.doesNotMatch(assembly1Xml, /forcerange="-220 220"/);
  assert.doesNotMatch(assembly2Xml, /forcerange="-220 220"/);

  const [step1Controller, step2Controller] = await Promise.all([
    readFile(new URL('../src/AssemblyStep1Controller.tsx', import.meta.url), 'utf8'),
    readFile(new URL('../src/AssemblyStep2Controller.tsx', import.meta.url), 'utf8'),
  ]);
  assert.doesNotMatch(step1Controller, /qfrc_applied|qfrc_bias/);
  assert.doesNotMatch(step2Controller, /qfrc_applied|qfrc_bias/);
});

test('Assembly2 uses palette-baked RoboTwin meshes and explicit collision geometry', async () => {
  const xml = layoutXml(FRANKA_ASSEMBLY2_LAYOUT);
  assert.match(
    xml,
    /name="robotwin_drill_grip_collision" type="box"[^>]*size="\.026 \.023 \.045"[^>]*friction="1\.5 \.25 \.03"/,
  );
  for (const tool of ['screwdriver', 'drill', 'hammer']) {
    assert.match(xml, new RegExp(`name="robotwin_${tool}_collision"`));
    const colors = [];
    for (const role of ['primary', 'dark', 'metal']) {
      assert.match(xml, new RegExp(`mesh="robotwin_${tool}_${role}"`));
      const material = new RegExp(
        `name="robotwin_${tool}_${role}_material"[^>]*rgba="([^"]+)"`,
      ).exec(xml);
      assert.ok(material, `${tool}/${role} material is missing`);
      colors.push(material[1]);

      const asset = await readFile(
        new URL(`../public/assets/franka-assembly2/tools/robotwin-${tool}-${role}.obj`, import.meta.url),
        'utf8',
      );
      assert.match(asset, /^f \d+ \d+ \d+$/m);
    }
    assert.equal(new Set(colors).size, 3, `${tool} must expose three visible colors`);
  }
});

test('both scenes expose realistic extrusion slots and paired interface sites', () => {
  for (const layout of [FRANKA_ASSEMBLY1_LAYOUT, FRANKA_ASSEMBLY2_LAYOUT]) {
    const xml = layoutXml(layout);
    assert.match(xml, /name="frame_rail_north_slot"/);
    assert.match(xml, /name="cross_member_north_hole_left"/);
    for (const suffix of ['nw', 'ne', 'sw', 'se']) {
      assert.match(xml, new RegExp(`name="frame_receiver_${suffix}"`));
      assert.match(xml, new RegExp(`name="cross_member_hole_${suffix}"`));
    }
  }
});
