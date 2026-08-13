import type { ThreeElements } from '@react-three/fiber';

declare global {
  interface Window {
    robotDemo?: {
      getCtrl(): number[];
      getQpos(): number[];
      getBodyPositions(names: string[]): Record<string, [number, number, number]>;
      getSitePositions(names: string[]): Record<string, [number, number, number]>;
      reset(): void;
      moveIkTargetBy(x: number, y: number, z: number): boolean;
      runAssemblyStep1(): boolean;
    };
  }
}

declare module 'react/jsx-runtime' {
  namespace JSX {
    interface IntrinsicElements extends ThreeElements {}
  }
}
