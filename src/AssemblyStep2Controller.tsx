import { useEffect, useRef } from 'react';
import type { MutableRefObject } from 'react';
import {
  findBodyByName,
  findJointByName,
  findSiteByName,
  getContact,
  useBeforePhysicsStep,
  useMujoco,
} from 'mujoco-react';
import type { MujocoData, MujocoModel } from 'mujoco-react';
import * as THREE from 'three';

import { ASSEMBLY1_STEP1_ARMS, applyAssemblyJointGravityCompensation } from './assemblyStep1.js';
import {
  ASSEMBLY1_STEP2_ARMS,
  ASSEMBLY1_STEP2_LIMITS,
  advanceAssemblyStep2Machine,
  createAssemblyStep2ControlFrame,
  createAssemblyStep2Machine,
  evaluateAssemblyStep2Grasp,
  quaternionAngularDistanceDegrees,
} from './assemblyStep2.js';
import type {
  AssemblyPose,
  AssemblyStep1CompletionSnapshot,
  AssemblyStep2ArmPlan,
  AssemblyStep2Failure,
  AssemblyStep2GraspVerdict,
  AssemblyStep2Machine,
  AssemblyStep2RuntimeDiagnostics,
  AssemblyStep2State,
} from './assemblyStep2.js';

const FORBIDDEN_BODY_NAMES = [
  'frame_supports',
  'parts_tray',
  'mounting_plate',
  'fastener_tray',
  'fastener_1',
  'fastener_2',
  'fastener_3',
  'fastener_4',
  'manual_screwdriver',
  'double_face_hammer',
  'tool_mat',
  'torque_driver_cradle_south',
  'torque_driver_cradle_north',
];

interface RuntimeArmPlan extends AssemblyStep2ArmPlan {
  actuatorIndices: number[];
  siteId: number;
  gripperActuatorIndex: number;
  dofAddresses: number[];
  fingerQposAddresses: [number, number];
  leftFingerBodyId: number;
  rightFingerBodyId: number;
  targetBodyId: number;
  targetBody: string;
}

interface RuntimePlan {
  arms: RuntimeArmPlan[];
  objectBaselines: Record<string, AssemblyPose>;
  forbiddenBodyIds: Set<number>;
}

interface AssemblyStep2ControllerProps {
  requestId: number;
  resetGeneration: number;
  step1Complete: boolean;
  step1SnapshotRef: MutableRefObject<AssemblyStep1CompletionSnapshot | null>;
  ownershipRef: MutableRefObject<'manual' | 'step1' | 'step2'>;
  diagnosticsRef: MutableRefObject<AssemblyStep2RuntimeDiagnostics | null>;
  onStateChange: (state: AssemblyStep2State) => void;
}

function distance(first: readonly number[], second: readonly number[]) {
  return Math.hypot(...first.map((value, index) => value - second[index]));
}

function vector3(values: Float64Array, index: number): [number, number, number] {
  const offset = index * 3;
  return [values[offset], values[offset + 1], values[offset + 2]];
}

function quaternion4(values: Float64Array, index: number): [number, number, number, number] {
  const offset = index * 4;
  return [values[offset], values[offset + 1], values[offset + 2], values[offset + 3]];
}

function siteQuaternion(data: MujocoData, siteId: number): [number, number, number, number] {
  const offset = siteId * 9;
  const rotation = new THREE.Matrix4().set(
    data.site_xmat[offset], data.site_xmat[offset + 1], data.site_xmat[offset + 2], 0,
    data.site_xmat[offset + 3], data.site_xmat[offset + 4], data.site_xmat[offset + 5], 0,
    data.site_xmat[offset + 6], data.site_xmat[offset + 7], data.site_xmat[offset + 8], 0,
    0, 0, 0, 1,
  );
  const result = new THREE.Quaternion().setFromRotationMatrix(rotation).normalize();
  return [result.x, result.y, result.z, result.w];
}

function nameAt(model: MujocoModel, address: number) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

function bodyName(model: MujocoModel, bodyId: number) {
  if (bodyId < 0 || bodyId >= model.nbody) return `body-${bodyId}`;
  return nameAt(model, model.name_bodyadr[bodyId]);
}

function currentPose(data: MujocoData, bodyId: number): AssemblyPose {
  return {
    position: vector3(data.xpos, bodyId),
    quaternion: quaternion4(data.xquat, bodyId),
  };
}

function solutionIsWithinLimits(model: MujocoModel, jointIds: number[], solution: readonly number[]) {
  return solution.length === jointIds.length && solution.every((value, index) => {
    const jointId = jointIds[index];
    if (!model.jnt_limited[jointId]) return Number.isFinite(value);
    const minimum = model.jnt_range[jointId * 2];
    const maximum = model.jnt_range[jointId * 2 + 1];
    return Number.isFinite(value) && value >= minimum && value <= maximum;
  });
}

function planningFailure(code: AssemblyStep2Failure['code'], detail: string): AssemblyStep2Failure {
  return { code, detail };
}

function createRuntimePlan(
  model: MujocoModel,
  data: MujocoData,
  snapshot: AssemblyStep1CompletionSnapshot,
): { plan: RuntimePlan | null; failure: AssemblyStep2Failure | null } {
  const objectBaselines: Record<string, AssemblyPose> = {};
  for (const body of ['assembly_frame', 'torque_driver', 'cross_member']) {
    const bodyId = findBodyByName(model, body);
    const baseline = snapshot.objectPoses[body];
    if (bodyId < 0 || !baseline) {
      return { plan: null, failure: planningFailure('missing-resource', body) };
    }
    const pose = currentPose(data, bodyId);
    if (distance(pose.position, baseline.position) > ASSEMBLY1_STEP2_LIMITS.preStepObjectDrift) {
      return { plan: null, failure: planningFailure('invalid-precondition', `${body} drift`) };
    }
    objectBaselines[body] = pose;
  }

  const arms: RuntimeArmPlan[] = [];
  for (const [index, arm] of ASSEMBLY1_STEP2_ARMS.entries()) {
    const step1Arm = ASSEMBLY1_STEP1_ARMS[index];
    const siteId = findSiteByName(model, arm.siteName);
    const jointIds = arm.jointNames.map((name) => findJointByName(model, name));
    const fingerJointIds = arm.fingerJointNames.map((name) => findJointByName(model, name));
    const leftFingerBodyId = findBodyByName(model, arm.leftFingerBody);
    const rightFingerBodyId = findBodyByName(model, arm.rightFingerBody);
    const targetBodyId = findBodyByName(model, arm.targetBody);
    if (
      siteId < 0
      || jointIds.some((id) => id < 0)
      || fingerJointIds.some((id) => id < 0)
      || leftFingerBodyId < 0
      || rightFingerBodyId < 0
      || targetBodyId < 0
    ) {
      return { plan: null, failure: planningFailure('missing-resource', arm.key) };
    }
    if (
      arm.actuatorIndices.some((actuator) => actuator < 0 || actuator >= model.nu)
      || arm.gripperActuatorIndex < 0
      || arm.gripperActuatorIndex >= model.nu
    ) {
      return { plan: null, failure: planningFailure('missing-resource', `${arm.key} actuator`) };
    }
    if (
      !solutionIsWithinLimits(model, jointIds, arm.approachJointTargets)
      || !solutionIsWithinLimits(model, jointIds, arm.contactJointTargets)
    ) {
      return { plan: null, failure: planningFailure('joint-limit', arm.key) };
    }
    const tcpPosition = vector3(data.site_xpos, siteId);
    const positionError = distance(tcpPosition, step1Arm.finalWaypoint);
    const orientationError = quaternionAngularDistanceDegrees(
      siteQuaternion(data, siteId),
      step1Arm.tcpQuaternion,
    );
    if (
      positionError > ASSEMBLY1_STEP2_LIMITS.tcpPosition
      || orientationError > ASSEMBLY1_STEP2_LIMITS.tcpOrientationDegrees
    ) {
      return {
        plan: null,
        failure: planningFailure(
          'invalid-precondition',
          `${arm.key} TCP ${positionError.toFixed(4)}m/${orientationError.toFixed(2)}deg`,
        ),
      };
    }
    if (Math.abs(data.ctrl[arm.gripperActuatorIndex] - 255) > 1e-6) {
      return { plan: null, failure: planningFailure('invalid-precondition', `${arm.key} gripper`) };
    }
    arms.push({
      armKey: arm.key,
      start: arm.actuatorIndices.map((actuator) => data.ctrl[actuator]),
      approach: [...arm.approachJointTargets],
      contact: [...arm.contactJointTargets],
      actuatorIndices: [...arm.actuatorIndices],
      siteId,
      gripperActuatorIndex: arm.gripperActuatorIndex,
      dofAddresses: jointIds.map((jointId) => model.jnt_dofadr[jointId]),
      fingerQposAddresses: [
        model.jnt_qposadr[fingerJointIds[0]],
        model.jnt_qposadr[fingerJointIds[1]],
      ],
      leftFingerBodyId,
      rightFingerBodyId,
      targetBodyId,
      targetBody: arm.targetBody,
    });
  }

  const forbiddenBodyIds = new Set(
    FORBIDDEN_BODY_NAMES
      .map((name) => findBodyByName(model, name))
      .filter((id) => id >= 0),
  );
  return { plan: { arms, objectBaselines, forbiddenBodyIds }, failure: null };
}

function contactsByFinger(model: MujocoModel, data: MujocoData, arms: RuntimeArmPlan[]) {
  const result = new Map<number, Set<number>>();
  for (const arm of arms) {
    result.set(arm.leftFingerBodyId, new Set());
    result.set(arm.rightFingerBodyId, new Set());
  }
  for (let index = 0; index < data.ncon; index += 1) {
    const contact = getContact(data, index);
    if (!contact) continue;
    const firstBody = model.geom_bodyid[contact.geom1];
    const secondBody = model.geom_bodyid[contact.geom2];
    result.get(firstBody)?.add(secondBody);
    result.get(secondBody)?.add(firstBody);
  }
  return result;
}

function armVerdicts(
  model: MujocoModel,
  data: MujocoData,
  runtime: RuntimePlan,
) {
  const contacts = contactsByFinger(model, data, runtime.arms);
  return runtime.arms.map((arm) => {
    const leftIds = [...(contacts.get(arm.leftFingerBodyId) ?? [])];
    const rightIds = [...(contacts.get(arm.rightFingerBodyId) ?? [])];
    const oppositeIds = [...leftIds, ...rightIds];
    const forbiddenBodies = [...new Set(oppositeIds)]
      .filter((id) => runtime.forbiddenBodyIds.has(id) && id !== arm.targetBodyId)
      .map((id) => bodyName(model, id));
    const current = currentPose(data, arm.targetBodyId);
    const baseline = runtime.objectBaselines[arm.targetBody];
    const measurements = {
      baselinePosition: baseline.position,
      currentPosition: current.position,
      tcpPosition: vector3(data.site_xpos, arm.siteId),
      leftContactBodies: leftIds.map((id) => bodyName(model, id)),
      rightContactBodies: rightIds.map((id) => bodyName(model, id)),
      aperture: data.qpos[arm.fingerQposAddresses[0]] + data.qpos[arm.fingerQposAddresses[1]],
      translation: distance(current.position, baseline.position),
      rotationDegrees: quaternionAngularDistanceDegrees(current.quaternion, baseline.quaternion),
      verticalDisplacement: Math.abs(current.position[2] - baseline.position[2]),
    };
    const verdict = evaluateAssemblyStep2Grasp({
      targetBody: arm.targetBody,
      ...measurements,
      forbiddenBodies,
    });
    const taggedVerdict: AssemblyStep2GraspVerdict = verdict.ok
      ? verdict
      : { ...verdict, armKey: arm.armKey };
    return { arm, measurements, verdict: taggedVerdict };
  });
}

function combinedVerdict(verdicts: AssemblyStep2GraspVerdict[]) {
  return verdicts.find((verdict) => !verdict.ok) ?? { ok: true as const };
}

export function AssemblyStep2Controller({
  requestId,
  resetGeneration,
  step1Complete,
  step1SnapshotRef,
  ownershipRef,
  diagnosticsRef,
  onStateChange,
}: AssemblyStep2ControllerProps) {
  const simulation = useMujoco();
  const runtimeRef = useRef<RuntimePlan | null>(null);
  const machineRef = useRef<AssemblyStep2Machine | null>(null);
  const lastTimeRef = useRef<number | null>(null);
  const completedRequestRef = useRef(0);
  const lastControlFrameRef = useRef<ReturnType<typeof createAssemblyStep2ControlFrame> | null>(null);
  const stateCallbackRef = useRef(onStateChange);
  const reportedPhaseRef = useRef<AssemblyStep2State['phase']>('idle');
  const currentContactSecondsRef = useRef([0, 0, 0, 0]);
  const maximumContactSecondsRef = useRef([0, 0, 0, 0]);
  const closureStartedAtRef = useRef<Array<number | null>>([null, null, null, null]);
  stateCallbackRef.current = onStateChange;

  useEffect(() => {
    runtimeRef.current = null;
    machineRef.current = null;
    lastTimeRef.current = null;
    lastControlFrameRef.current = null;
    diagnosticsRef.current = null;
    currentContactSecondsRef.current = [0, 0, 0, 0];
    maximumContactSecondsRef.current = [0, 0, 0, 0];
    closureStartedAtRef.current = [null, null, null, null];
    completedRequestRef.current = requestId;
    reportedPhaseRef.current = 'idle';
    if (ownershipRef.current === 'step2') ownershipRef.current = 'manual';
  }, [diagnosticsRef, ownershipRef, resetGeneration]);

  useEffect(() => {
    if (requestId <= 0 || requestId <= completedRequestRef.current) return;
    if (simulation.status !== 'ready') return;
    const model = simulation.mjModelRef.current;
    const data = simulation.mjDataRef.current;
    if (!model || !data) return;
    completedRequestRef.current = requestId;
    stateCallbackRef.current({ phase: 'planning', failure: null });
    reportedPhaseRef.current = 'planning';
    const snapshot = step1SnapshotRef.current;
    if (!step1Complete || !snapshot) {
      const failure = planningFailure('invalid-precondition', 'Step 1 is not complete');
      diagnosticsRef.current = {
        phase: 'error',
        phaseElapsed: 0,
        continuousValidSeconds: 0,
        failure,
        simulationTime: data.time,
        arms: [],
      };
      stateCallbackRef.current({ phase: 'error', failure });
      reportedPhaseRef.current = 'error';
      return;
    }
    const { plan, failure } = createRuntimePlan(model, data, snapshot);
    if (!plan || failure) {
      const resolvedFailure = failure
        ?? planningFailure('invalid-precondition', 'unknown planning error');
      diagnosticsRef.current = {
        phase: 'error',
        phaseElapsed: 0,
        continuousValidSeconds: 0,
        failure: resolvedFailure,
        simulationTime: data.time,
        arms: [],
      };
      stateCallbackRef.current({
        phase: 'error',
        failure: resolvedFailure,
      });
      reportedPhaseRef.current = 'error';
      return;
    }
    const machine = createAssemblyStep2Machine();
    runtimeRef.current = plan;
    machineRef.current = machine;
    lastControlFrameRef.current = createAssemblyStep2ControlFrame(machine, plan.arms);
    lastTimeRef.current = data.time;
    ownershipRef.current = 'step2';
    stateCallbackRef.current({ phase: machine.phase, failure: null });
    reportedPhaseRef.current = machine.phase;
  }, [ownershipRef, requestId, simulation, step1Complete, step1SnapshotRef]);

  useBeforePhysicsStep((model, data) => {
    if (ownershipRef.current !== 'step2') return;
    const runtime = runtimeRef.current;
    const machine = machineRef.current;
    if (!runtime || !machine) return;
    const previousTime = lastTimeRef.current ?? data.time;
    const deltaSeconds = Math.max(0, data.time - previousTime);
    lastTimeRef.current = data.time;
    const samples = armVerdicts(model, data, runtime);
    for (let index = 0; index < samples.length; index += 1) {
      currentContactSecondsRef.current[index] = samples[index].verdict.ok
        ? currentContactSecondsRef.current[index] + deltaSeconds
        : 0;
      maximumContactSecondsRef.current[index] = Math.max(
        maximumContactSecondsRef.current[index],
        currentContactSecondsRef.current[index],
      );
    }
    const evidence = {
      frame: samples[0].verdict,
      crossMember: combinedVerdict([samples[2].verdict, samples[3].verdict]),
      tool: samples[1].verdict,
      all: combinedVerdict(samples.map(({ verdict }) => verdict)),
    };
    const nextMachine = advanceAssemblyStep2Machine(machine, deltaSeconds, evidence);
    machineRef.current = nextMachine;
    if (nextMachine.phase !== reportedPhaseRef.current) {
      reportedPhaseRef.current = nextMachine.phase;
      stateCallbackRef.current({
        phase: nextMachine.phase,
        failure: nextMachine.failure,
      });
    }
    if (nextMachine.phase !== 'error') {
      lastControlFrameRef.current = createAssemblyStep2ControlFrame(nextMachine, runtime.arms);
    }
    const controlFrame = lastControlFrameRef.current;
    if (!controlFrame) return;
    applyAssemblyJointGravityCompensation(
      data.qfrc_applied,
      data.qfrc_bias,
      runtime.arms.flatMap((arm) => arm.dofAddresses),
    );
    for (let index = 0; index < runtime.arms.length; index += 1) {
      const arm = runtime.arms[index];
      const controls = controlFrame.arms[index];
      for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
        data.ctrl[arm.actuatorIndices[joint]] = controls.jointTargets[joint];
      }
      data.ctrl[arm.gripperActuatorIndex] = controls.gripperTarget;
      if (
        closureStartedAtRef.current[index] === null
        && controls.gripperTarget < 254.999
      ) {
        closureStartedAtRef.current[index] = data.time;
      }
    }
    diagnosticsRef.current = {
      phase: nextMachine.phase,
      phaseElapsed: nextMachine.phaseElapsed,
      continuousValidSeconds: nextMachine.continuousValidSeconds,
      failure: nextMachine.failure,
      simulationTime: data.time,
      arms: samples.map(({ arm, measurements, verdict }, index) => ({
        armKey: arm.armKey,
        targetBody: arm.targetBody,
        ...measurements,
        currentContactSeconds: currentContactSecondsRef.current[index],
        maximumContactSeconds: maximumContactSecondsRef.current[index],
        gripperControl: data.ctrl[arm.gripperActuatorIndex],
        closureStartedAt: closureStartedAtRef.current[index],
        verdict,
      })),
    };
  });

  return null;
}
