import type { ThreeElements } from '@react-three/fiber';

declare global {
  interface Window {
    robotDemo?: {
      getCtrl(): number[];
      getQpos(): number[];
      reset(): void;
      moveIkTargetBy(x: number, y: number, z: number): boolean;
    };
  }
}

declare module 'react/jsx-runtime' {
  namespace JSX {
    interface IntrinsicElements extends ThreeElements {}
  }
}
