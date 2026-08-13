import { useCallback, useEffect, useMemo, useRef, useState, type RefObject } from 'react';
import { OrbitControls, Html, Stats, Environment } from '@react-three/drei';
import { Leva, useControls, button } from 'leva';
import {
  MujocoProvider,
  MujocoCanvas,
  IkGizmo,
  DragInteraction,
  ContactMarkers,
  Debug,
  useSelectionHighlight,
  useMujoco,
  useGravityCompensation,
  findBodyByName,
  findJointByName,
  findSiteByName,
  getContact,
} from 'mujoco-react';
import type { MujocoData, MujocoModel, MujocoSimAPI } from 'mujoco-react';
import { robots } from './configs';
import type { ControlTarget } from './controlTargets.js';
import { FrankaController } from './controllers/FrankaController';
import { SO101Controller } from './controllers/SO101Controller';
import { XLeRobotController } from './controllers/XLeRobotController';
import { PlanarMobileController } from './controllers/PlanarMobileController';
import { useSelectedIkController } from './controllers/useSelectedIkController';
import { useClickSelect } from './useClickSelect';
import { KeyboardHelp } from './KeyboardHelp';
import { GitHubLink } from './GitHubLink';
import { AssemblyStep1Controller } from './AssemblyStep1Controller';
import { AssemblyStep2Controller } from './AssemblyStep2Controller';
import { AssemblySequencePanel } from './AssemblySequencePanel';
import type { AssemblyStep1Status } from './assemblyStep1.js';
import { ASSEMBLY1_STEP2_ARMS } from './assemblyStep2.js';
import type {
  AssemblyStep1CompletionSnapshot,
  AssemblyStep2RuntimeDiagnostics,
  AssemblyStep2State,
} from './assemblyStep2.js';

function LoadingOverlay() {
  const sim = useMujoco();
  if (sim.isReady) return null;
  return (
    <Html center>
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        gap: 12,
        color: sim.isError ? '#f87171' : '#94a3b8',
        fontFamily: 'system-ui, sans-serif',
      }}>
        {sim.isError ? (
          <span style={{ fontSize: 14 }}>{sim.error}</span>
        ) : (
          <>
            <div style={{
              width: 32,
              height: 32,
              border: '3px solid #334155',
              borderTop: '3px solid #38bdf8',
              borderRadius: '50%',
              animation: 'spin 0.8s linear infinite',
            }} />
            <span style={{ fontSize: 14 }}>Loading model...</span>
            <style>{`@keyframes spin { to { transform: rotate(360deg) } }`}</style>
          </>
        )}
      </div>
    </Html>
  );
}

/** Composable gravity compensation — uses the hook. */
function GravityCompensation({ enabled }: { enabled: boolean }) {
  useGravityCompensation(enabled);
  return null;
}

/** Composable click-to-select — uses hooks. */
function ClickSelectOverlay() {
  const selectedBodyId = useClickSelect();
  useSelectionHighlight(selectedBodyId);
  return null;
}

function modelName(model: MujocoModel, address: number) {
  let name = '';
  for (let index = address; model.names[index] !== 0; index += 1) {
    name += String.fromCharCode(model.names[index]);
  }
  return name;
}

function captureAssemblyStep1Snapshot(
  model: MujocoModel,
  data: MujocoData,
): AssemblyStep1CompletionSnapshot {
  const objectPoses = Object.fromEntries(
    ['assembly_frame', 'torque_driver', 'cross_member'].map((name) => {
      const bodyId = findBodyByName(model, name);
      if (bodyId < 0) throw new Error(`Could not capture Assembly1 body: ${name}`);
      const positionOffset = bodyId * 3;
      const quaternionOffset = bodyId * 4;
      return [name, {
        position: [
          data.xpos[positionOffset],
          data.xpos[positionOffset + 1],
          data.xpos[positionOffset + 2],
        ],
        quaternion: [
          data.xquat[quaternionOffset],
          data.xquat[quaternionOffset + 1],
          data.xquat[quaternionOffset + 2],
          data.xquat[quaternionOffset + 3],
        ],
      }];
    }),
  ) as AssemblyStep1CompletionSnapshot['objectPoses'];
  const tcpPositions = Object.fromEntries(ASSEMBLY1_STEP2_ARMS.map((arm) => {
    const siteId = findSiteByName(model, arm.siteName);
    if (siteId < 0) throw new Error(`Could not capture Assembly1 site: ${arm.siteName}`);
    const offset = siteId * 3;
    return [arm.siteName, [
      data.site_xpos[offset],
      data.site_xpos[offset + 1],
      data.site_xpos[offset + 2],
    ]];
  })) as AssemblyStep1CompletionSnapshot['tcpPositions'];
  const tcpOrientations = Object.fromEntries(ASSEMBLY1_STEP2_ARMS.map((arm) => {
    const siteId = findSiteByName(model, arm.siteName);
    const offset = siteId * 9;
    return [arm.siteName, Array.from(data.site_xmat.slice(offset, offset + 9))];
  }));
  return {
    tcpPositions,
    tcpOrientations,
    objectPoses,
    gripperControls: ASSEMBLY1_STEP2_ARMS.map(
      (arm) => data.ctrl[arm.gripperActuatorIndex],
    ) as [number, number, number, number],
  };
}

/** Selection-aware IK and keyboard controllers for the active physical instance. */
function SceneChildren({
  robotKey,
  controlFamily,
  target,
  resetGeneration,
  showGizmo,
  gizmoScale,
  assemblyStep1RequestId,
  assemblyStep1Status,
  assemblyStep2RequestId,
  assemblyStep2State,
  assemblyOwnershipRef,
  step1SnapshotRef,
  step2DiagnosticsRef,
  onAssemblyStep1StatusChange,
  onAssemblyStep2StateChange,
  onRunAssemblyStep1,
  onRunAssemblyStep2,
  onResetAssemblySequence,
}: {
  robotKey: string;
  controlFamily: 'franka' | 'so101' | 'xlerobot';
  target: ControlTarget;
  resetGeneration: number;
  showGizmo: boolean;
  gizmoScale?: number;
  assemblyStep1RequestId: number;
  assemblyStep1Status: AssemblyStep1Status;
  assemblyStep2RequestId: number;
  assemblyStep2State: AssemblyStep2State;
  assemblyOwnershipRef: React.MutableRefObject<'manual' | 'step1' | 'step2'>;
  step1SnapshotRef: React.MutableRefObject<AssemblyStep1CompletionSnapshot | null>;
  step2DiagnosticsRef: React.MutableRefObject<AssemblyStep2RuntimeDiagnostics | null>;
  onAssemblyStep1StatusChange: (status: AssemblyStep1Status) => void;
  onAssemblyStep2StateChange: (state: AssemblyStep2State) => void;
  onRunAssemblyStep1: () => boolean;
  onRunAssemblyStep2: () => boolean;
  onResetAssemblySequence: () => void;
}) {
  const simulation = useMujoco();
  const assemblyAutomationActive = assemblyStep1Status === 'planning'
    || assemblyStep1Status === 'running'
    || assemblyStep2State.phase !== 'idle';
  const { controller: ik, resolvedSiteName } = useSelectedIkController(
    target,
    resetGeneration,
    assemblyAutomationActive,
  );

  useEffect(() => {
    document.documentElement.dataset.controlTarget = target.key;
    if (resolvedSiteName) {
      document.documentElement.dataset.ikSite = resolvedSiteName;
    } else {
      delete document.documentElement.dataset.ikSite;
    }
  }, [target.key, resolvedSiteName]);

  useEffect(() => {
    if (simulation.status !== 'ready') return;
    const model = simulation.mjModelRef.current;
    const data = simulation.mjDataRef.current;
    if (!model || !data) return;
    const collectPositions = (
      names: string[],
      findId: (name: string) => number,
      positions: Float64Array,
    ) => Object.fromEntries(names.map((name) => {
      const id = findId(name);
      if (id < 0) throw new Error(`Could not resolve diagnostic position: ${name}`);
      const offset = id * 3;
      return [name, [positions[offset], positions[offset + 1], positions[offset + 2]]];
    })) as Record<string, [number, number, number]>;
    const diagnostics = {
      getCtrl: () => Array.from(simulation.api.getCtrl()),
      getQpos: () => Array.from(simulation.api.getQpos()),
      getBodyPositions: (names: string[]) => collectPositions(
        names,
        (name) => findBodyByName(model, name),
        data.xpos,
      ),
      getSitePositions: (names: string[]) => collectPositions(
        names,
        (name) => findSiteByName(model, name),
        data.site_xpos,
      ),
      getSiteOrientations: (names: string[]) => Object.fromEntries(names.map((name) => {
        const id = findSiteByName(model, name);
        if (id < 0) throw new Error(`Could not resolve diagnostic orientation: ${name}`);
        const offset = id * 9;
        return [name, Array.from(data.site_xmat.slice(offset, offset + 9))];
      })),
      getBodyOrientations: (names: string[]) => Object.fromEntries(names.map((name) => {
        const id = findBodyByName(model, name);
        if (id < 0) throw new Error(`Could not resolve diagnostic body: ${name}`);
        const offset = id * 4;
        return [name, [
          data.xquat[offset],
          data.xquat[offset + 1],
          data.xquat[offset + 2],
          data.xquat[offset + 3],
        ] as [number, number, number, number]];
      })) as Record<string, [number, number, number, number]>,
      getJointPositions: (names: string[]) => Object.fromEntries(names.map((name) => {
        const id = findJointByName(model, name);
        if (id < 0) throw new Error(`Could not resolve diagnostic joint: ${name}`);
        return [name, data.qpos[model.jnt_qposadr[id]]];
      })),
      getContacts: () => Array.from({ length: data.ncon }, (_, index) => {
        const contact = getContact(data, index);
        if (!contact) return null;
        const body1 = model.geom_bodyid[contact.geom1];
        const body2 = model.geom_bodyid[contact.geom2];
        return {
          geom1: contact.geom1,
          geom2: contact.geom2,
          body1: modelName(model, model.name_bodyadr[body1]),
          body2: modelName(model, model.name_bodyadr[body2]),
        };
      }).filter((contact) => contact !== null),
      reset: onResetAssemblySequence,
      moveIkTargetBy: (x: number, y: number, z: number) => {
        if (!ik) return false;
        ik.syncTargetToSite();
        const nextTarget = ik.ikTargetRef.current.position.clone();
        nextTarget.x += x;
        nextTarget.y += y;
        nextTarget.z += z;
        ik.moveTarget(nextTarget);
        return true;
      },
      runAssemblyStep1: onRunAssemblyStep1,
      runAssemblyStep2: onRunAssemblyStep2,
      getAssemblyStep2Diagnostics: () => step2DiagnosticsRef.current,
    };
    window.robotDemo = diagnostics;
    return () => {
      if (window.robotDemo === diagnostics) delete window.robotDemo;
    };
  }, [
    simulation,
    ik,
    target.key,
    onResetAssemblySequence,
    onRunAssemblyStep1,
    onRunAssemblyStep2,
    step2DiagnosticsRef,
  ]);

  return (
    <>
      {ik && showGizmo && !assemblyAutomationActive && (
        <IkGizmo
          key={`gizmo-${target.key}`}
          controller={ik}
          siteName={target.ik?.siteName}
          scale={gizmoScale}
        />
      )}

      {controlFamily === 'franka' && (
        <FrankaController
          key={`franka-${target.key}-${assemblyStep1Status === 'complete' ? 'open' : 'closed'}`}
          target={target}
          enabled={!assemblyAutomationActive}
          initiallyOpen={assemblyStep1Status === 'complete'}
        />
      )}
      {target.controlMode === 'planar-mobile' && (
        <PlanarMobileController key={`mobile-${target.key}`} target={target} />
      )}
      {controlFamily === 'so101' && target.controlMode !== 'planar-mobile' && (
        <SO101Controller key={`so101-${target.key}`} target={target} ik={ik} />
      )}
      {controlFamily === 'xlerobot' && (
        <XLeRobotController key={`xlerobot-${target.key}`} target={target} ik={ik} />
      )}
      {robotKey === 'frankaAssembly1' && (
        <AssemblyStep1Controller
          requestId={assemblyStep1RequestId}
          resetGeneration={resetGeneration}
          ownershipRef={assemblyOwnershipRef}
          onStatusChange={onAssemblyStep1StatusChange}
          onMotionComplete={(model, data) => {
            step1SnapshotRef.current = captureAssemblyStep1Snapshot(model, data);
            ik?.syncTargetToSite();
            ik?.setIkEnabled(false);
          }}
        />
      )}
      {robotKey === 'frankaAssembly1' && (
        <AssemblyStep2Controller
          requestId={assemblyStep2RequestId}
          resetGeneration={resetGeneration}
          step1Complete={assemblyStep1Status === 'complete'}
          step1SnapshotRef={step1SnapshotRef}
          ownershipRef={assemblyOwnershipRef}
          diagnosticsRef={step2DiagnosticsRef}
          onStateChange={onAssemblyStep2StateChange}
        />
      )}
    </>
  );
}

const robotOptions = Object.fromEntries(
  Object.entries(robots).map(([key, r]) => [r.label, key])
);

const replicatedRootPatterns: Record<string, RegExp> = {
  franka: /^r\d+_link0$/,
  frankaAssembly1: /^r\d+_link0$/,
  frankaAssembly2: /^r\d+_link0$/,
  so101: /^r\d+_Base$/,
  so101Gearbox: /^r\d+_Base$/,
  so101HomeLab: /^r\d+_Base$/,
  xlerobot: /^r\d+_chassis$/,
  xlerobotKitting: /^r\d+_chassis$/,
};

export function App() {
  const apiRef = useRef<MujocoSimAPI>(null);
  const [resetGeneration, setResetGeneration] = useState(0);
  const [assemblyStep1RequestId, setAssemblyStep1RequestId] = useState(0);
  const [assemblyStep1Status, setAssemblyStep1Status] = useState<AssemblyStep1Status>('idle');
  const [assemblyStep2RequestId, setAssemblyStep2RequestId] = useState(0);
  const [assemblyStep2State, setAssemblyStep2State] = useState<AssemblyStep2State>({
    phase: 'idle',
    failure: null,
  });
  const assemblyOwnershipRef = useRef<'manual' | 'step1' | 'step2'>('manual');
  const step1SnapshotRef = useRef<AssemblyStep1CompletionSnapshot | null>(null);
  const step2DiagnosticsRef = useRef<AssemblyStep2RuntimeDiagnostics | null>(null);
  const performanceStatsRef = useRef<HTMLDivElement>(null!);
  // Drei's Stats type omits the null state that every DOM ref has before mount.
  const performanceStatsParentRef = performanceStatsRef as unknown as RefObject<HTMLElement>;

  const { robot: robotKey } = useControls({
    robot: { value: 'frankaAssembly1', options: robotOptions, label: 'Robot' },
  });

  const entry = robots[robotKey];
  const controlTargetOptions = useMemo(
    () => Object.fromEntries(entry.controlTargets.map((target) => [target.label, target.key])),
    [entry],
  );
  const { controlTarget: controlTargetKey } = useControls({
    controlTarget: {
      value: entry.controlTargets[0].key,
      options: controlTargetOptions,
      label: 'Control target',
    },
  }, [robotKey]);
  const controlTarget = entry.controlTargets.find(({ key }) => key === controlTargetKey)
    ?? entry.controlTargets[0];
  const assemblyAutomationActive = assemblyStep1Status === 'planning'
    || assemblyStep1Status === 'running'
    || assemblyStep2State.phase !== 'idle';

  const handleRunAssemblyStep1 = useCallback(() => {
    if (robotKey !== 'frankaAssembly1' || assemblyStep1Status !== 'idle') return false;
    setAssemblyStep1Status('planning');
    setAssemblyStep1RequestId((requestId) => requestId + 1);
    return true;
  }, [assemblyStep1Status, robotKey]);

  const handleRunAssemblyStep2 = useCallback(() => {
    if (
      robotKey !== 'frankaAssembly1'
      || assemblyStep1Status !== 'complete'
      || assemblyStep2State.phase !== 'idle'
      || !step1SnapshotRef.current
    ) return false;
    setAssemblyStep2State({ phase: 'planning', failure: null });
    setAssemblyStep2RequestId((requestId) => requestId + 1);
    return true;
  }, [assemblyStep1Status, assemblyStep2State.phase, robotKey]);

  const handleResetAssemblySequence = useCallback(() => {
    apiRef.current?.reset();
    assemblyOwnershipRef.current = 'manual';
    step1SnapshotRef.current = null;
    step2DiagnosticsRef.current = null;
    setAssemblyStep1Status('idle');
    setAssemblyStep2State({ phase: 'idle', failure: null });
    setResetGeneration((generation) => generation + 1);
  }, []);

  useEffect(() => {
    document.documentElement.dataset.sceneStatus = 'loading';
    document.documentElement.dataset.sceneKey = robotKey;
    delete document.documentElement.dataset.sceneBodies;
    delete document.documentElement.dataset.sceneInstances;
    delete document.documentElement.dataset.sceneError;
    delete document.documentElement.dataset.controlTarget;
    delete document.documentElement.dataset.ikSite;
    assemblyOwnershipRef.current = 'manual';
    step1SnapshotRef.current = null;
    step2DiagnosticsRef.current = null;
    setAssemblyStep1Status('idle');
    setAssemblyStep2State({ phase: 'idle', failure: null });
  }, [robotKey]);

  useEffect(() => {
    if (robotKey === 'frankaAssembly1') {
      document.documentElement.dataset.assemblyStep1Status = assemblyStep1Status;
    } else {
      delete document.documentElement.dataset.assemblyStep1Status;
    }
  }, [assemblyStep1Status, robotKey]);

  useEffect(() => {
    if (robotKey === 'frankaAssembly1') {
      document.documentElement.dataset.assemblyStep2Status = assemblyStep2State.phase;
    } else {
      delete document.documentElement.dataset.assemblyStep2Status;
    }
  }, [assemblyStep2State.phase, robotKey]);

  const handleSceneReady = useCallback((api: MujocoSimAPI) => {
    const bodies = api.getBodies();
    const instanceCount = bodies.filter(({ name }) => replicatedRootPatterns[robotKey].test(name)).length;
    document.documentElement.dataset.sceneStatus = 'ready';
    document.documentElement.dataset.sceneBodies = String(bodies.length);
    document.documentElement.dataset.sceneInstances = String(instanceCount);
    console.info(`[scene] ${robotKey} ready with ${instanceCount} instances and ${bodies.length} bodies`);
  }, [robotKey]);

  const handleSceneError = useCallback((error: Error) => {
    document.documentElement.dataset.sceneStatus = 'error';
    document.documentElement.dataset.sceneError = error.message;
    console.error(`[scene] ${robotKey} failed: ${error.message}`);
  }, [robotKey]);

  const sim = useControls('Simulation', {
    paused: false,
    speed: { value: 1.0, min: 0.1, max: 3.0, step: 0.1 },
    gravityCompensation: { value: false, label: 'gravity compensation' },
    gizmo: { value: true, label: 'IK gizmo' },
    reset: button(handleResetAssemblySequence),
  });

  const debug = useControls('Debug', {
    contacts: false,
    sites: false,
    joints: false,
  });

  const canvasKey = useMemo(() => robotKey, [robotKey]);

  return (
    <MujocoProvider>
      <Leva
        theme={{ sizes: { rootWidth: '320px', controlWidth: '185px' } }}
      />
      <MujocoCanvas
        key={canvasKey}
        ref={apiRef}
        config={entry.config}
        onReady={handleSceneReady}
        onError={handleSceneError}
        camera={{
          position: entry.camera.position,
          up: [0, 0, 1] satisfies [number, number, number],
          fov: entry.camera.fov,
          near: 0.01,
          far: 100,
        }}
        paused={sim.paused}
        speed={sim.speed}
        shadows
        style={{ width: '100%', height: '100%' }}
      >
        <OrbitControls
          enableDamping
          dampingFactor={0.1}
          target={entry.orbitTarget}
          makeDefault
        />

        {/* Core scene */}
        <LoadingOverlay />
        <GravityCompensation enabled={sim.gravityCompensation} />

        {/* IK + per-robot controllers */}
        <SceneChildren
          robotKey={robotKey}
          controlFamily={entry.controlFamily}
          target={controlTarget}
          resetGeneration={resetGeneration}
          showGizmo={sim.gizmo}
          gizmoScale={entry.gizmoScale}
          assemblyStep1RequestId={assemblyStep1RequestId}
          assemblyStep1Status={assemblyStep1Status}
          assemblyStep2RequestId={assemblyStep2RequestId}
          assemblyStep2State={assemblyStep2State}
          assemblyOwnershipRef={assemblyOwnershipRef}
          step1SnapshotRef={step1SnapshotRef}
          step2DiagnosticsRef={step2DiagnosticsRef}
          onAssemblyStep1StatusChange={setAssemblyStep1Status}
          onAssemblyStep2StateChange={setAssemblyStep2State}
          onRunAssemblyStep1={handleRunAssemblyStep1}
          onRunAssemblyStep2={handleRunAssemblyStep2}
          onResetAssemblySequence={handleResetAssemblySequence}
        />

        {/* Opt-in interaction */}
        {!assemblyAutomationActive && <DragInteraction />}
        <ClickSelectOverlay />

        {/* Debug overlays */}
        <ContactMarkers visible={debug.contacts} />
        <Debug showSites={debug.sites} showJoints={debug.joints} />

        {/* Scene decoration — lights, environment, grid */}
        <Environment preset="lobby" background backgroundBlurriness={1} backgroundIntensity={0.6} environmentIntensity={0.5} />
        <ambientLight intensity={0.4} />
        <directionalLight position={[2, -2, 5]} intensity={1.5} castShadow />
        <directionalLight position={[-1, 1, 3]} intensity={0.3} />
        <gridHelper
          args={[entry.gridSize ?? 4, entry.gridDivisions ?? 40, '#64748b', '#94a3b8']}
          rotation={[Math.PI / 2, 0, 0]}
          position={[0, 0, 0.001]}
        />
        <Stats parent={performanceStatsParentRef} showPanel={0} />
        <Stats parent={performanceStatsParentRef} showPanel={1} />
        <Stats parent={performanceStatsParentRef} showPanel={2} />
      </MujocoCanvas>

      {/* HTML overlay — outside R3F canvas */}
      <div
        ref={performanceStatsRef}
        className="performance-stats"
        aria-label="Performance statistics: frames per second, frame time, and memory"
      >
        <span className="performance-stats__label">Scene performance</span>
      </div>
      {robotKey === 'frankaAssembly1' && (
        <AssemblySequencePanel
          step1Status={assemblyStep1Status}
          step2State={assemblyStep2State}
          canRunStep2={
            assemblyStep1Status === 'complete'
            && assemblyStep2State.phase === 'idle'
            && Boolean(step1SnapshotRef.current)
          }
          onRunStep1={handleRunAssemblyStep1}
          onRunStep2={handleRunAssemblyStep2}
        />
      )}
      <KeyboardHelp
        robotKey={entry.controlFamily}
        controlTargetLabel={controlTarget.label}
        controlMode={controlTarget.controlMode}
      />
      <GitHubLink />
    </MujocoProvider>
  );
}
