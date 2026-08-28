import { useEffect, useRef } from 'react';
import type { MutableRefObject } from 'react';
import {
  findBodyByName,
  findJointByName,
  findSiteByName,
  useBeforePhysicsStep,
  useMujoco,
} from 'mujoco-react';
import type { MujocoData, MujocoModel } from 'mujoco-react';

import {
  consumeMujocoContacts,
  isPassiveRetainingContactGeom,
} from './mujocoContact.js';
import { ASSEMBLY1_STEP2_ARMS, quaternionAngularDistanceDegrees } from './assemblyStep2.js';
import {
  ASSEMBLY1_STEP4_ARMS,
  ASSEMBLY1_STEP4_DURATIONS,
  ASSEMBLY1_STEP4_GRIPPERS,
  ASSEMBLY1_STEP4_LIMITS,
  advanceAssemblyStep4Machine,
  createAssemblyStep4ControlFrame,
  createAssemblyStep4Machine,
  evaluateAssemblyStep4Placement,
  evaluateAssemblyStep4Stability,
} from './assemblyStep4.js';
import type {
  AssemblyStep4ArmPlan,
  AssemblyStep4Failure,
  AssemblyStep4Machine,
  AssemblyStep4RuntimeDiagnostics,
  AssemblyStep4State,
  AssemblyStep4Verdict,
} from './assemblyStep4.js';

type AssemblyOwnership = 'manual' | 'step1' | 'step2' | 'step3' | 'step4';

interface RuntimeArm {
  armKey: string;
  actuatorIndices: number[];
  gripperActuatorIndex: number;
  qposAddresses: number[];
}

interface RuntimePlan {
  arms: RuntimeArm[];
  armPlans: AssemblyStep4ArmPlan[];
  frameBodyId: number;
  crossMemberBodyId: number;
  fastenerBodyId: number;
  hammerBodyId: number;
  receiverSiteId: number;
  fastenerFingerQposAddresses: [number, number];
  fastenerLeftFingerBodyId: number;
  fastenerRightFingerBodyId: number;
  hammerFingerQposAddresses: [number, number];
  hammerLeftFingerBodyId: number;
  hammerRightFingerBodyId: number;
  frameBaseline: [number, number, number];
  crossMemberBaseline: [number, number, number];
  crossMemberBaselineQuaternion: [number, number, number, number];
}

interface AssemblyStep4ControllerProps {
  requestId: number;
  resetGeneration: number;
  step3Complete: boolean;
  ownershipRef: MutableRefObject<AssemblyOwnership>;
  diagnosticsRef: MutableRefObject<AssemblyStep4RuntimeDiagnostics | null>;
  onStateChange: (state: AssemblyStep4State) => void;
}

function vector3(values: Float64Array, index: number): [number, number, number] {
  const offset = index * 3;
  return [values[offset], values[offset + 1], values[offset + 2]];
}

function quaternion4(values: Float64Array, index: number): [number, number, number, number] {
  const offset = index * 4;
  return [values[offset], values[offset + 1], values[offset + 2], values[offset + 3]];
}

function distance(first: readonly number[], second: readonly number[]) {
  return Math.hypot(...first.map((value, index) => value - second[index]));
}

function nameAt(model: MujocoModel, address: number) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

function planningFailure(code: string, detail: string): AssemblyStep4Failure {
  return { code, detail };
}

function solutionIsWithinLimits(
  model: MujocoModel,
  jointIds: number[],
  solution: readonly number[],
) {
  return solution.length === jointIds.length && solution.every((value, index) => {
    const jointId = jointIds[index];
    if (!model.jnt_limited[jointId]) return Number.isFinite(value);
    return Number.isFinite(value)
      && value >= model.jnt_range[jointId * 2]
      && value <= model.jnt_range[jointId * 2 + 1];
  });
}

function createRuntimePlan(
  model: MujocoModel,
  data: MujocoData,
): { plan: RuntimePlan | null; failure: AssemblyStep4Failure | null } {
  const frameBodyId = findBodyByName(model, 'assembly_frame');
  const crossMemberBodyId = findBodyByName(model, 'cross_member');
  const fastenerBodyId = findBodyByName(model, 'fastener_1');
  const hammerBodyId = findBodyByName(model, 'double_face_hammer');
  const receiverSiteId = findSiteByName(model, 'frame_receiver_nw');
  const fastenerArm = ASSEMBLY1_STEP2_ARMS[2];
  const hammerArm = ASSEMBLY1_STEP2_ARMS[3];
  const fastenerFingerJointIds = fastenerArm.fingerJointNames.map(
    (name) => findJointByName(model, name),
  );
  const fastenerLeftFingerBodyId = findBodyByName(model, fastenerArm.leftFingerBody);
  const fastenerRightFingerBodyId = findBodyByName(model, fastenerArm.rightFingerBody);
  const hammerFingerJointIds = hammerArm.fingerJointNames.map(
    (name) => findJointByName(model, name),
  );
  const hammerLeftFingerBodyId = findBodyByName(model, hammerArm.leftFingerBody);
  const hammerRightFingerBodyId = findBodyByName(model, hammerArm.rightFingerBody);
  if (
    frameBodyId < 0
    || crossMemberBodyId < 0
    || fastenerBodyId < 0
    || hammerBodyId < 0
    || receiverSiteId < 0
    || fastenerFingerJointIds.some((id) => id < 0)
    || fastenerLeftFingerBodyId < 0
    || fastenerRightFingerBodyId < 0
    || hammerFingerJointIds.some((id) => id < 0)
    || hammerLeftFingerBodyId < 0
    || hammerRightFingerBodyId < 0
  ) {
    return { plan: null, failure: planningFailure('missing-resource', 'Step 4 workcell') };
  }

  const expectedGrippers = [
    ASSEMBLY1_STEP4_GRIPPERS.frame,
    ASSEMBLY1_STEP4_GRIPPERS.donorEntry,
    ASSEMBLY1_STEP4_GRIPPERS.open,
    ASSEMBLY1_STEP4_GRIPPERS.open,
  ];
  const arms: RuntimeArm[] = [];
  const armPlans: AssemblyStep4ArmPlan[] = [];
  for (const [index, arm] of ASSEMBLY1_STEP2_ARMS.entries()) {
    const contract = ASSEMBLY1_STEP4_ARMS[index];
    const jointIds = arm.jointNames.map((name) => findJointByName(model, name));
    if (
      !contract
      || contract.key !== arm.key
      || jointIds.some((id) => id < 0)
      || arm.actuatorIndices.some((actuator) => actuator < 0 || actuator >= model.nu)
      || arm.gripperActuatorIndex < 0
      || arm.gripperActuatorIndex >= model.nu
    ) {
      return { plan: null, failure: planningFailure('missing-resource', arm.key) };
    }
    if (Math.abs(data.ctrl[arm.gripperActuatorIndex] - expectedGrippers[index]) > 1e-6) {
      return { plan: null, failure: planningFailure('invalid-precondition', `${arm.key} gripper`) };
    }
    const generatedTargets = Object.values(contract.jointTargets);
    if (generatedTargets.some((target) => !solutionIsWithinLimits(model, jointIds, target))) {
      return { plan: null, failure: planningFailure('joint-limit', arm.key) };
    }
    const qposAddresses = jointIds.map((jointId) => model.jnt_qposadr[jointId]);
    const hold = qposAddresses.map((address) => data.qpos[address]);
    arms.push({
      armKey: arm.key,
      actuatorIndices: [...arm.actuatorIndices],
      gripperActuatorIndex: arm.gripperActuatorIndex,
      qposAddresses,
    });
    armPlans.push({
      armKey: arm.key,
      hold,
      prepare: contract.jointTargets.prepare,
      engage: contract.jointTargets.engage,
      lift: contract.jointTargets.lift,
      transfer: contract.jointTargets.transfer,
      insert: contract.jointTargets.insert,
      clear: contract.jointTargets.clear,
      ready: contract.jointTargets.ready,
      strike: contract.jointTargets.strike,
    });
  }

  return {
    plan: {
      arms,
      armPlans,
      frameBodyId,
      crossMemberBodyId,
      fastenerBodyId,
      hammerBodyId,
      receiverSiteId,
      fastenerFingerQposAddresses: [
        model.jnt_qposadr[fastenerFingerJointIds[0]],
        model.jnt_qposadr[fastenerFingerJointIds[1]],
      ],
      fastenerLeftFingerBodyId,
      fastenerRightFingerBodyId,
      hammerFingerQposAddresses: [
        model.jnt_qposadr[hammerFingerJointIds[0]],
        model.jnt_qposadr[hammerFingerJointIds[1]],
      ],
      hammerLeftFingerBodyId,
      hammerRightFingerBodyId,
      frameBaseline: vector3(data.xpos, frameBodyId),
      crossMemberBaseline: vector3(data.xpos, crossMemberBodyId),
      crossMemberBaselineQuaternion: quaternion4(data.xquat, crossMemberBodyId),
    },
    failure: null,
  };
}

function targetContacts(
  model: MujocoModel,
  data: MujocoData,
  targetBodyId: number,
  leftFingerBodyId: number,
  rightFingerBodyId: number,
) {
  let left = false;
  let right = false;
  let leftDistance: number | null = null;
  let rightDistance: number | null = null;
  for (const contact of consumeMujocoContacts(data.contact, data.ncon)) {
    const firstBody = model.geom_bodyid[contact.geom1];
    const secondBody = model.geom_bodyid[contact.geom2];
    const targetGeomId = firstBody === targetBodyId ? contact.geom1 : contact.geom2;
    const passiveRetainer = isPassiveRetainingContactGeom(
      nameAt(model, model.name_geomadr[targetGeomId]),
    );
    if (
      (firstBody === leftFingerBodyId && secondBody === targetBodyId)
      || (secondBody === leftFingerBodyId && firstBody === targetBodyId)
    ) {
      left = true;
      if (!passiveRetainer) {
        leftDistance = leftDistance === null
          ? contact.distance
          : Math.min(leftDistance, contact.distance);
      }
    }
    if (
      (firstBody === rightFingerBodyId && secondBody === targetBodyId)
      || (secondBody === rightFingerBodyId && firstBody === targetBodyId)
    ) {
      right = true;
      if (!passiveRetainer) {
        rightDistance = rightDistance === null
          ? contact.distance
          : Math.min(rightDistance, contact.distance);
      }
    }
  }
  return { left, right, leftDistance, rightDistance };
}

function sampleRuntime(
  model: MujocoModel,
  data: MujocoData,
  runtime: RuntimePlan,
  lastContactTimes: {
    fastener: { left: number; right: number };
    hammer: { left: number; right: number };
  },
) {
  const fastenerContacts = targetContacts(
    model,
    data,
    runtime.fastenerBodyId,
    runtime.fastenerLeftFingerBodyId,
    runtime.fastenerRightFingerBodyId,
  );
  const hammerContacts = targetContacts(
    model,
    data,
    runtime.hammerBodyId,
    runtime.hammerLeftFingerBodyId,
    runtime.hammerRightFingerBodyId,
  );
  if (fastenerContacts.left) lastContactTimes.fastener.left = data.time;
  if (fastenerContacts.right) lastContactTimes.fastener.right = data.time;
  if (hammerContacts.left) lastContactTimes.hammer.left = data.time;
  if (hammerContacts.right) lastContactTimes.hammer.right = data.time;
  const frameTranslation = distance(
    vector3(data.xpos, runtime.frameBodyId),
    runtime.frameBaseline,
  );
  const crossMemberTranslation = distance(
    vector3(data.xpos, runtime.crossMemberBodyId),
    runtime.crossMemberBaseline,
  );
  const crossMemberRotationDegrees = quaternionAngularDistanceDegrees(
    quaternion4(data.xquat, runtime.crossMemberBodyId),
    runtime.crossMemberBaselineQuaternion,
  );
  const all = evaluateAssemblyStep4Stability({
    frameTranslation,
    crossMemberTranslation,
    crossMemberRotationDegrees,
  });
  const fastenerAperture = data.qpos[runtime.fastenerFingerQposAddresses[0]]
    + data.qpos[runtime.fastenerFingerQposAddresses[1]];
  let fastenerGrasp: AssemblyStep4Verdict = { ok: true };
  const leftRecent = data.time - lastContactTimes.fastener.left <= ASSEMBLY1_STEP4_DURATIONS.contactGrace;
  const rightRecent = data.time - lastContactTimes.fastener.right <= ASSEMBLY1_STEP4_DURATIONS.contactGrace;
  if (!leftRecent && !rightRecent) {
    fastenerGrasp = { ok: false, code: 'missing-finger-contact', armKey: 'r2' };
  } else if (!(fastenerAperture > ASSEMBLY1_STEP4_LIMITS.minimumFastenerAperture)) {
    fastenerGrasp = { ok: false, code: 'empty-closure', armKey: 'r2' };
  }
  const fastenerPosition = vector3(data.xpos, runtime.fastenerBodyId);
  const receiverPosition = vector3(data.site_xpos, runtime.receiverSiteId);
  const fastenerPlanarDistance = Math.hypot(
    fastenerPosition[0] - receiverPosition[0],
    fastenerPosition[1] - receiverPosition[1],
  );
  const fastenerHeight = fastenerPosition[2];
  const placement = evaluateAssemblyStep4Placement({
    fastenerPlanarDistance,
    fastenerHeight,
  });
  const hammerAperture = data.qpos[runtime.hammerFingerQposAddresses[0]]
    + data.qpos[runtime.hammerFingerQposAddresses[1]];
  const hammerLeftRecent = data.time - lastContactTimes.hammer.left
    <= ASSEMBLY1_STEP4_DURATIONS.contactGrace;
  const hammerRightRecent = data.time - lastContactTimes.hammer.right
    <= ASSEMBLY1_STEP4_DURATIONS.contactGrace;
  let hammerGrasp: AssemblyStep4Verdict = { ok: true };
  if (!hammerLeftRecent) {
    hammerGrasp = { ok: false, code: 'missing-hammer-left-contact', armKey: 'r3' };
  } else if (!hammerRightRecent) {
    hammerGrasp = { ok: false, code: 'missing-hammer-right-contact', armKey: 'r3' };
  } else if (
    [hammerContacts.leftDistance, hammerContacts.rightDistance]
      .some((value) => (
        typeof value === 'number'
        && value < -ASSEMBLY1_STEP4_LIMITS.maximumContactPenetration
      ))
  ) {
    const deepest = Math.min(
      ...[hammerContacts.leftDistance, hammerContacts.rightDistance]
        .filter((value): value is number => typeof value === 'number'),
    );
    hammerGrasp = { ok: false, code: 'deep-penetration', armKey: 'r3', detail: String(deepest) };
  } else if (!(hammerAperture > ASSEMBLY1_STEP4_LIMITS.minimumToolAperture)) {
    hammerGrasp = { ok: false, code: 'empty-closure', armKey: 'r3' };
  }
  return {
    all,
    fastenerGrasp,
    hammerGrasp,
    placement,
    frameTranslation,
    crossMemberTranslation,
    crossMemberRotationDegrees,
    fastenerPosition,
    fastenerPlanarDistance,
    fastenerHeight,
    fastenerAperture,
    fastenerLeftContact: fastenerContacts.left,
    fastenerRightContact: fastenerContacts.right,
    hammerAperture,
    hammerLeftContact: hammerContacts.left,
    hammerRightContact: hammerContacts.right,
    hammerLeftContactDistance: hammerContacts.leftDistance,
    hammerRightContactDistance: hammerContacts.rightDistance,
  };
}

function holdCurrentJointControls(data: MujocoData, arms: RuntimeArm[]) {
  for (const arm of arms) {
    for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
      data.ctrl[arm.actuatorIndices[joint]] = data.qpos[arm.qposAddresses[joint]];
    }
  }
}

export function AssemblyStep4Controller({
  requestId,
  resetGeneration,
  step3Complete,
  ownershipRef,
  diagnosticsRef,
  onStateChange,
}: AssemblyStep4ControllerProps) {
  const simulation = useMujoco();
  const runtimeRef = useRef<RuntimePlan | null>(null);
  const machineRef = useRef<AssemblyStep4Machine | null>(null);
  const lastTimeRef = useRef<number | null>(null);
  const lastContactTimesRef = useRef({
    fastener: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
    hammer: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
  });
  const completedRequestRef = useRef(0);
  const stateCallbackRef = useRef(onStateChange);
  const reportedPhaseRef = useRef<AssemblyStep4State['phase']>('idle');
  stateCallbackRef.current = onStateChange;

  useEffect(() => {
    runtimeRef.current = null;
    machineRef.current = null;
    lastTimeRef.current = null;
    lastContactTimesRef.current = {
      fastener: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
      hammer: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
    };
    diagnosticsRef.current = null;
    completedRequestRef.current = requestId;
    reportedPhaseRef.current = 'idle';
    if (ownershipRef.current === 'step4') ownershipRef.current = 'manual';
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
    if (!step3Complete || ownershipRef.current !== 'step3') {
      const failure = planningFailure('invalid-precondition', 'Step 3 is not complete');
      stateCallbackRef.current({ phase: 'error', failure });
      reportedPhaseRef.current = 'error';
      return;
    }
    const { plan, failure } = createRuntimePlan(model, data);
    if (!plan || failure) {
      const resolvedFailure = failure ?? planningFailure('invalid-precondition', 'unknown planning error');
      stateCallbackRef.current({ phase: 'error', failure: resolvedFailure });
      reportedPhaseRef.current = 'error';
      return;
    }
    const machine = createAssemblyStep4Machine();
    runtimeRef.current = plan;
    machineRef.current = machine;
    lastTimeRef.current = data.time;
    lastContactTimesRef.current = {
      fastener: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
      hammer: { left: Number.NEGATIVE_INFINITY, right: Number.NEGATIVE_INFINITY },
    };
    ownershipRef.current = 'step4';
    stateCallbackRef.current({ phase: machine.phase, failure: null });
    reportedPhaseRef.current = machine.phase;
  }, [ownershipRef, requestId, simulation, step3Complete]);

  useBeforePhysicsStep((model, data) => {
    if (ownershipRef.current !== 'step4') return;
    const runtime = runtimeRef.current;
    const machine = machineRef.current;
    if (!runtime || !machine) return;
    const previousTime = lastTimeRef.current ?? data.time;
    const deltaSeconds = Math.max(0, data.time - previousTime);
    lastTimeRef.current = data.time;
    const sample = sampleRuntime(model, data, runtime, lastContactTimesRef.current);
    const nextMachine = advanceAssemblyStep4Machine(machine, deltaSeconds, {
      all: ['donor-tighten', 'prepare', 'engage'].includes(machine.phase)
        ? { ok: true }
        : sample.all,
      fastenerGrasp: sample.fastenerGrasp,
      hammerGrasp: sample.hammerGrasp,
      placement: sample.placement,
    });
    if (machine.phase === 'prepare' && nextMachine.phase === 'engage') {
      runtime.frameBaseline = vector3(data.xpos, runtime.frameBodyId);
      runtime.crossMemberBaseline = vector3(data.xpos, runtime.crossMemberBodyId);
      runtime.crossMemberBaselineQuaternion = quaternion4(data.xquat, runtime.crossMemberBodyId);
    }
    if (machine.phase === 'engage-settle' && nextMachine.phase === 'dual-clamp') {
      runtime.frameBaseline = vector3(data.xpos, runtime.frameBodyId);
      runtime.crossMemberBaseline = vector3(data.xpos, runtime.crossMemberBodyId);
      runtime.crossMemberBaselineQuaternion = quaternion4(data.xquat, runtime.crossMemberBodyId);
    }
    machineRef.current = nextMachine;
    if (nextMachine.phase !== reportedPhaseRef.current) {
      reportedPhaseRef.current = nextMachine.phase;
      stateCallbackRef.current({ phase: nextMachine.phase, failure: nextMachine.failure });
    }
    if (nextMachine.phase === 'error') {
      holdCurrentJointControls(data, runtime.arms);
      ownershipRef.current = 'manual';
    } else {
      const controlFrame = createAssemblyStep4ControlFrame(nextMachine, runtime.armPlans);
      for (let index = 0; index < runtime.arms.length; index += 1) {
        const arm = runtime.arms[index];
        const controls = controlFrame.arms[index];
        for (let joint = 0; joint < arm.actuatorIndices.length; joint += 1) {
          data.ctrl[arm.actuatorIndices[joint]] = controls.jointTargets[joint];
        }
        data.ctrl[arm.gripperActuatorIndex] = controls.gripperTarget;
      }
    }
    diagnosticsRef.current = {
      phase: nextMachine.phase,
      phaseElapsed: nextMachine.phaseElapsed,
      continuousValidSeconds: nextMachine.continuousValidSeconds,
      failure: nextMachine.failure,
      simulationTime: data.time,
      frameTranslation: sample.frameTranslation,
      crossMemberTranslation: sample.crossMemberTranslation,
      crossMemberRotationDegrees: sample.crossMemberRotationDegrees,
      fastenerPosition: sample.fastenerPosition,
      fastenerPlanarDistance: sample.fastenerPlanarDistance,
      fastenerHeight: sample.fastenerHeight,
      fastenerAperture: sample.fastenerAperture,
      fastenerLeftContact: sample.fastenerLeftContact,
      fastenerRightContact: sample.fastenerRightContact,
      hammerAperture: sample.hammerAperture,
      hammerLeftContact: sample.hammerLeftContact,
      hammerRightContact: sample.hammerRightContact,
      hammerLeftContactDistance: sample.hammerLeftContactDistance,
      hammerRightContactDistance: sample.hammerRightContactDistance,
    };
    if (nextMachine.phase === 'error') {
      runtimeRef.current = null;
      machineRef.current = null;
    }
  });

  return null;
}
