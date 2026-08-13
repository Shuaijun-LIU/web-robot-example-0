import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';

import {
  applyAssemblyTargetPose,
  FRANKA_ASSEMBLY1_LAYOUT,
  FRANKA_ASSEMBLY2_LAYOUT,
  FRANKA_ASSEMBLY_INTERFACE,
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
      poweredTool: [0.53, -0.42, 0.135],
      manualTool: [-0.53, -0.42, 0.13],
      hammer: [0.65, 0, 0.229],
      fasteners: [0.56, 0.42, 0.125],
      handover: [0, -0.48, 0.112],
    });
  }
});

test('cross-member target pose aligns all four installation holes with frame receivers', () => {
  const expected = [
    [-0.04, 0.215, 0.275],
    [0.04, 0.215, 0.275],
    [-0.04, -0.215, 0.275],
    [0.04, -0.215, 0.275],
  ];
  assert.deepEqual(FRANKA_ASSEMBLY_INTERFACE.frameReceiverPositions, expected);

  const actual = applyAssemblyTargetPose(
    FRANKA_ASSEMBLY_INTERFACE.crossMemberHoleLocalPositions,
    FRANKA_ASSEMBLY_INTERFACE.crossMemberTargetPose,
  );
  actual.forEach((point, pointIndex) => point.forEach((value, axis) => {
    assert.ok(Math.abs(value - expected[pointIndex][axis]) < 1e-9);
  }));
});

test('Assembly1 exposes stable faceted hand tools with recognizable detail', () => {
  const xml = layoutXml(FRANKA_ASSEMBLY1_LAYOUT);
  assert.match(xml, /mesh="manual_screwdriver_octagonal_handle"/);
  assert.match(xml, /<joint name="manual_screwdriver_free" type="free" damping="\.08"\/>/);
  assert.match(
    xml,
    /name="manual_screwdriver_handle"[^>]*contype="0"[^>]*conaffinity="0"/,
  );
  assert.match(
    xml,
    /name="manual_screwdriver_handle_collision" type="box"[^>]*size="\.072 \.022 \.022"/,
  );
  assert.match(xml, /name="torque_driver_trigger"/);
  assert.match(xml, /name="torque_driver_vent_/);
  assert.match(
    xml,
    /<body name="torque_driver" pos="\.53 -\.42 \.166" euler="90 0 0">/,
  );
  assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
    ({ name }) => name === 'torque_driver_cradle_south',
  ));
  assert.ok(FRANKA_ASSEMBLY1_LAYOUT.sceneObjects.some(
    ({ name }) => name === 'torque_driver_cradle_north',
  ));
  assert.ok(!FRANKA_ASSEMBLY2_LAYOUT.sceneObjects.some(
    ({ name }) => name.startsWith('torque_driver_cradle_'),
  ));
  assert.doesNotMatch(
    layoutXml(FRANKA_ASSEMBLY2_LAYOUT),
    /<body name="torque_driver" pos="\.53 -\.42 \.166" euler="90 0 0">/,
  );
  assert.match(xml, /<body name="double_face_hammer"[^>]*euler="0 0 180"/);
  assert.match(xml, /name="hammer_eye"/);
  assert.match(xml, /name="hammer_cheek"/);
  assert.match(xml, /name="hammer_striking_face_a" type="cylinder"[^>]*fromto="\.075 -\.053 0 \.075 -\.073 0"/);
  assert.match(xml, /name="hammer_striking_face_a"[^>]*fromto="\.075 -\.053 0 \.075 -\.073 0"/);
  assert.match(xml, /name="hammer_striking_face_b"[^>]*fromto="\.075 \.053 0 \.075 \.073 0"/);
  assert.doesNotMatch(xml, /claw/i);
});

test('both Assembly layouts strengthen physical finger contact without attachment', () => {
  for (const layout of [FRANKA_ASSEMBLY1_LAYOUT, FRANKA_ASSEMBLY2_LAYOUT]) {
    const xml = layoutXml(layout);
    assert.match(xml, /gainprm="\.23529411765 0 0" biasprm="0 -1500 -40"/);
    assert.match(xml, /fingertip_pad_collision_1[\s\S]*friction="3 \.2 \.05" condim="6"/);
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
