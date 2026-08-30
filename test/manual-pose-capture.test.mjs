import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import test from 'node:test';

import {
  createAssembly1PoseSnapshot,
  sanitizePoseLabel,
} from '../src/manualPoseCapture.js';
import { persistManualPoseSnapshot } from '../scripts/manualPoseCapturePlugin.mjs';

test('manual pose labels are filesystem-safe and reject traversal', () => {
  assert.equal(sanitizePoseLabel('handover-clamp_01'), 'handover-clamp_01');
  assert.throws(() => sanitizePoseLabel('../outside'), /字母、数字/);
  assert.throws(() => sanitizePoseLabel(''), /字母、数字/);
  assert.throws(() => sanitizePoseLabel('a'.repeat(65)), /64/);
});

test('Assembly1 pose snapshot captures controls, four TCPs, objects, joints, and contacts', () => {
  const calls = [];
  const robotDemo = {
    getCtrl: () => [1, 2],
    getQpos: () => [3, 4],
    getQvel: () => [5, 6],
    getSitePositions: (names) => {
      calls.push(['sites', names]);
      return Object.fromEntries(names.map((name, index) => [name, [index, index + 1, index + 2]]));
    },
    getSiteOrientations: (names) => Object.fromEntries(names.map((name) => [name, [1, 0, 0, 0, 1, 0, 0, 0, 1]])),
    getBodyPositions: (names) => {
      calls.push(['bodies', names]);
      return Object.fromEntries(names.map((name) => [name, [0.1, 0.2, 0.3]]));
    },
    getBodyOrientations: (names) => Object.fromEntries(names.map((name) => [name, [1, 0, 0, 0]])),
    getJointPositions: (names) => {
      calls.push(['joints', names]);
      return Object.fromEntries(names.map((name) => [name, 0.02]));
    },
    getContacts: () => [{ geomName1: 'finger', geomName2: 'hammer', distance: -0.001 }],
  };

  const snapshot = createAssembly1PoseSnapshot(robotDemo, {
    label: 'handover-clamp',
    selectedControlTarget: 'r3',
    capturedAt: '2026-08-30T10:00:00.000Z',
  });

  assert.equal(snapshot.schemaVersion, 'franka-assembly1-manual-pose-v1');
  assert.equal(snapshot.scene, 'frankaAssembly1');
  assert.equal(snapshot.label, 'handover-clamp');
  assert.equal(snapshot.selectedControlTarget, 'r3');
  assert.deepEqual(snapshot.controls, { ctrl: [1, 2], qpos: [3, 4], qvel: [5, 6] });
  assert.deepEqual(Object.keys(snapshot.tcpPositions), ['r0_tcp', 'r1_tcp', 'r2_tcp', 'r3_tcp']);
  assert.equal(Object.keys(snapshot.gripperJointPositions).length, 8);
  assert.ok(Object.hasOwn(snapshot.bodyPositions, 'double_face_hammer'));
  assert.ok(Object.hasOwn(snapshot.bodyPositions, 'cross_member'));
  assert.ok(Object.hasOwn(snapshot.bodyPositions, 'fastener_3'));
  assert.equal(snapshot.contacts.length, 1);
  assert.equal(calls[0][1].length, 4);
});

test('server persistence writes only under artifacts/manual-poses using an atomic JSON file', async () => {
  const projectRoot = await mkdtemp(path.join(tmpdir(), 'manual-pose-'));
  try {
    const payload = {
      schemaVersion: 'franka-assembly1-manual-pose-v1',
      scene: 'frankaAssembly1',
      label: 'donor-clear',
      capturedAt: '2026-08-30T10:00:00.000Z',
      controls: { qpos: [], qvel: [], ctrl: [] },
    };
    const result = await persistManualPoseSnapshot(projectRoot, payload);
    assert.equal(result.relativePath, 'artifacts/manual-poses/donor-clear.json');
    const saved = JSON.parse(await readFile(path.join(projectRoot, result.relativePath), 'utf8'));
    assert.deepEqual(saved, payload);
    await assert.rejects(
      persistManualPoseSnapshot(projectRoot, { ...payload, label: '../escape' }),
      /字母、数字/,
    );
  } finally {
    await rm(projectRoot, { recursive: true, force: true });
  }
});
