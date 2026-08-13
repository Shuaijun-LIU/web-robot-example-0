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
  assert.match(xml, /<body name="claw_hammer"[^>]*euler="0 0 180"/);
  assert.match(xml, /name="hammer_eye"/);
  assert.match(xml, /name="hammer_cheek"/);
  assert.match(xml, /name="hammer_striking_face" type="cylinder"[^>]*fromto="\.075 -\.053 0 \.075 -\.073 0"/);
  assert.match(xml, /mesh="hammer_claw_left_plate"/);
  assert.match(xml, /mesh="hammer_claw_right_plate"/);
  assert.match(xml, /name="hammer_claw_left" type="mesh"[^>]*contype="0"[^>]*conaffinity="0"/);
  assert.match(xml, /name="hammer_claw_right" type="mesh"[^>]*contype="0"[^>]*conaffinity="0"/);
  assert.match(xml, /name="hammer_claw_collision" type="box"[^>]*rgba="0 0 0 0"/);
  assert.doesNotMatch(xml, /name="hammer_claw_[^"]*" type="capsule"/);
});

test('Assembly2 uses palette-baked RoboTwin meshes and explicit collision geometry', async () => {
  const xml = layoutXml(FRANKA_ASSEMBLY2_LAYOUT);
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
