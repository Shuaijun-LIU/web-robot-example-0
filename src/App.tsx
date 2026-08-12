import { useCallback, useEffect, useMemo, useRef, useState, type RefObject } from 'react';
import { OrbitControls, Html, Stats, Environment } from '@react-three/drei';
import { useControls, button } from 'leva';
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
} from 'mujoco-react';
import type { MujocoSimAPI } from 'mujoco-react';
import { robots } from './configs';
import type { ControlTarget } from './controlTargets.js';
import { FrankaController } from './controllers/FrankaController';
import { SO101Controller } from './controllers/SO101Controller';
import { XLeRobotController } from './controllers/XLeRobotController';
import { useSelectedIkController } from './controllers/useSelectedIkController';
import { useClickSelect } from './useClickSelect';
import { KeyboardHelp } from './KeyboardHelp';
import { GitHubLink } from './GitHubLink';

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

/** Selection-aware IK and keyboard controllers for the active physical instance. */
function SceneChildren({
  robotKey,
  target,
  resetGeneration,
  showGizmo,
  gizmoScale,
}: {
  robotKey: string;
  target: ControlTarget;
  resetGeneration: number;
  showGizmo: boolean;
  gizmoScale?: number;
}) {
  const simulation = useMujoco();
  const { controller: ik, resolvedSiteName } = useSelectedIkController(target, resetGeneration);

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
    const diagnostics = {
      getCtrl: () => Array.from(simulation.api.getCtrl()),
      getQpos: () => Array.from(simulation.api.getQpos()),
      reset: () => {
        simulation.api.reset();
        ik?.setIkEnabled(false);
        ik?.syncTargetToSite();
      },
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
    };
    window.robotDemo = diagnostics;
    return () => {
      if (window.robotDemo === diagnostics) delete window.robotDemo;
    };
  }, [simulation, ik, target.key]);

  return (
    <>
      {ik && showGizmo && (
        <IkGizmo
          key={`gizmo-${target.key}`}
          controller={ik}
          siteName={target.ik?.siteName}
          scale={gizmoScale}
        />
      )}

      {robotKey === 'franka' && (
        <FrankaController key={`franka-${target.key}`} target={target} />
      )}
      {robotKey === 'so101' && (
        <SO101Controller key={`so101-${target.key}`} target={target} ik={ik} />
      )}
      {robotKey === 'xlerobot' && (
        <XLeRobotController key={`xlerobot-${target.key}`} target={target} ik={ik} />
      )}
    </>
  );
}

const robotOptions = Object.fromEntries(
  Object.entries(robots).map(([key, r]) => [r.label, key])
);

const replicatedRootPatterns: Record<string, RegExp> = {
  franka: /^r\d+_link0$/,
  so101: /^r\d+_Base$/,
  xlerobot: /^r\d+_chassis$/,
};

export function App() {
  const apiRef = useRef<MujocoSimAPI>(null);
  const [resetGeneration, setResetGeneration] = useState(0);
  const performanceStatsRef = useRef<HTMLDivElement>(null!);
  // Drei's Stats type omits the null state that every DOM ref has before mount.
  const performanceStatsParentRef = performanceStatsRef as unknown as RefObject<HTMLElement>;

  const { robot: robotKey } = useControls({
    robot: { value: 'franka', options: robotOptions, label: 'Robot' },
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

  useEffect(() => {
    document.documentElement.dataset.sceneStatus = 'loading';
    document.documentElement.dataset.sceneKey = robotKey;
    delete document.documentElement.dataset.sceneBodies;
    delete document.documentElement.dataset.sceneInstances;
    delete document.documentElement.dataset.sceneError;
    delete document.documentElement.dataset.controlTarget;
    delete document.documentElement.dataset.ikSite;
  }, [robotKey]);

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
    reset: button(() => {
      apiRef.current?.reset();
      setResetGeneration((generation) => generation + 1);
    }),
  });

  const debug = useControls('Debug', {
    contacts: false,
    sites: false,
    joints: false,
  });

  const canvasKey = useMemo(() => robotKey, [robotKey]);

  return (
    <MujocoProvider>
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
          target={controlTarget}
          resetGeneration={resetGeneration}
          showGizmo={sim.gizmo}
          gizmoScale={entry.gizmoScale}
        />

        {/* Opt-in interaction */}
        <DragInteraction />
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
          args={[4, 40, '#64748b', '#94a3b8']}
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
      <KeyboardHelp robotKey={robotKey} controlTargetLabel={controlTarget.label} />
      <GitHubLink />
    </MujocoProvider>
  );
}
