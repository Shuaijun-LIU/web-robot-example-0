import type { ThreeElements } from '@react-three/fiber';
import type { AssemblyStep2RuntimeDiagnostics } from './assemblyStep2.js';
import type { UnitreeActionState } from './unitreeActionState.js';
import type { UnitreeRuntimeDiagnostics } from './unitreeDynamicsAdapter.js';

declare global {
  interface Window {
    robotDemo?: {
      getCtrl(): number[];
      getQpos(): number[];
      getBodyPositions(names: string[]): Record<string, [number, number, number]>;
      getSitePositions(names: string[]): Record<string, [number, number, number]>;
      getSiteOrientations(names: string[]): Record<string, number[]>;
      getBodyOrientations(names: string[]): Record<string, [number, number, number, number]>;
      getJointPositions(names: string[]): Record<string, number>;
      getContacts(): Array<{ geom1: number; geom2: number; body1: string; body2: string }>;
      reset(): void;
      moveIkTargetBy(x: number, y: number, z: number): boolean;
      runAssemblyStep1(): boolean;
      runAssemblyStep2(): boolean;
      getAssemblyStep2Diagnostics(): AssemblyStep2RuntimeDiagnostics | null;
      runUnitreeAction(): boolean;
      pauseUnitreeAction(): boolean;
      resumeUnitreeAction(): boolean;
      getUnitreeActionState(): UnitreeActionState;
      getUnitreeActionDiagnostics(): UnitreeRuntimeDiagnostics | null;
    };
  }
}

declare module 'react/jsx-runtime' {
  namespace JSX {
    interface IntrinsicElements extends ThreeElements {}
  }
}
