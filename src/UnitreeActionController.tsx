import { useEffect, useRef } from 'react';
import {
  findActuatorByName,
  useBeforePhysicsStep,
  useMujoco,
} from 'mujoco-react';
import type { MujocoModel } from 'mujoco-react';

import {
  G1_ACTUATORS,
  GO2_ACTUATORS,
  UNITREE_ACTION_DURATION,
  applyUnitreeActionTargets,
  isControlRangeCompatible,
  sampleUnitreeAction,
} from './unitreeActionSequence.js';
import {
  completeAction,
  failAction,
} from './unitreeActionState.js';
import type { UnitreeActionState } from './unitreeActionState.js';
import type {
  UnitreeActionSample,
  UnitreeActuatorIds,
} from './unitreeActionSequence.js';

interface UnitreeActionControllerProps {
  requestId: number;
  resetGeneration: number;
  state: UnitreeActionState;
  onStateChange: (state: UnitreeActionState) => void;
}

function resolveActuators(model: MujocoModel): UnitreeActuatorIds {
  const resolveGroup = (actuators: typeof G1_ACTUATORS) => actuators.map(({ name, min, max }) => {
    const id = findActuatorByName(model, name);
    if (id < 0) throw new Error(`找不到执行器 ${name}`);
    const rangeMin = model.actuator_ctrlrange[id * 2];
    const rangeMax = model.actuator_ctrlrange[id * 2 + 1];
    if (!isControlRangeCompatible({ name, min, max }, rangeMin, rangeMax)) {
      throw new Error(`执行器 ${name} 的控制范围与动作定义不一致`);
    }
    return id;
  });
  const ids = {
    g1: resolveGroup(G1_ACTUATORS),
    go2: resolveGroup(GO2_ACTUATORS),
  };
  if (new Set([...ids.g1, ...ids.go2]).size !== 47) {
    throw new Error('动作执行器映射存在重复项');
  }
  return ids;
}

export function UnitreeActionController({
  requestId,
  resetGeneration,
  state,
  onStateChange,
}: UnitreeActionControllerProps) {
  const simulation = useMujoco();
  const actuatorIdsRef = useRef<UnitreeActuatorIds | null>(null);
  const elapsedRef = useRef(0);
  const lastPhysicsTimeRef = useRef(0);
  const lastSampleRef = useRef<UnitreeActionSample>(sampleUnitreeAction(0));
  const processedRequestRef = useRef(0);
  const lastPublishedElapsedRef = useRef(0);
  const stateRef = useRef(state);
  const onStateChangeRef = useRef(onStateChange);
  stateRef.current = state;
  onStateChangeRef.current = onStateChange;

  useEffect(() => {
    actuatorIdsRef.current = null;
    elapsedRef.current = 0;
    lastPhysicsTimeRef.current = 0;
    lastSampleRef.current = sampleUnitreeAction(0);
    processedRequestRef.current = 0;
    lastPublishedElapsedRef.current = 0;
  }, [resetGeneration]);

  useEffect(() => {
    if (requestId === 0 || requestId === processedRequestRef.current || simulation.status !== 'ready') return;
    const model = simulation.mjModelRef.current;
    const data = simulation.mjDataRef.current;
    if (!model || !data) return;
    processedRequestRef.current = requestId;
    try {
      actuatorIdsRef.current = resolveActuators(model);
      elapsedRef.current = 0;
      lastPhysicsTimeRef.current = data.time;
      lastPublishedElapsedRef.current = 0;
      lastSampleRef.current = sampleUnitreeAction(0);
    } catch (error) {
      actuatorIdsRef.current = null;
      onStateChangeRef.current(failAction(stateRef.current, error));
    }
  }, [requestId, simulation]);

  useBeforePhysicsStep((_model, data) => {
    const ids = actuatorIdsRef.current;
    const currentState = stateRef.current;
    if (!ids || (currentState.status !== 'running' && currentState.status !== 'paused')) return;

    try {
      if (currentState.status === 'paused') {
        lastPhysicsTimeRef.current = data.time;
        applyUnitreeActionTargets(data.ctrl, ids, lastSampleRef.current);
        return;
      }

      const physicsDelta = Math.max(0, data.time - lastPhysicsTimeRef.current);
      lastPhysicsTimeRef.current = data.time;
      elapsedRef.current = Math.min(
        UNITREE_ACTION_DURATION,
        elapsedRef.current + physicsDelta,
      );
      const sample = sampleUnitreeAction(elapsedRef.current);
      lastSampleRef.current = sample;
      applyUnitreeActionTargets(data.ctrl, ids, sample);

      if (elapsedRef.current >= UNITREE_ACTION_DURATION) {
        actuatorIdsRef.current = null;
        onStateChangeRef.current(completeAction(currentState));
        return;
      }

      const phaseChanged = sample.phase !== currentState.phase;
      const publishElapsed = elapsedRef.current - lastPublishedElapsedRef.current >= 0.1;
      if (phaseChanged || publishElapsed) {
        lastPublishedElapsedRef.current = elapsedRef.current;
        onStateChangeRef.current({
          status: 'running',
          phase: sample.phase,
          elapsed: elapsedRef.current,
          error: null,
        });
      }
    } catch (error) {
      actuatorIdsRef.current = null;
      onStateChangeRef.current(failAction(currentState, error));
    }
  });

  return null;
}
