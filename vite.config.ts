import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';
import { manualPoseCapturePlugin } from './scripts/manualPoseCapturePlugin.mjs';
import { fixedPhysicsControlPlugin } from './scripts/fixedPhysicsControlPlugin.mjs';

export default defineConfig({
  plugins: [react(), manualPoseCapturePlugin(), fixedPhysicsControlPlugin()],
  optimizeDeps: { exclude: ['mujoco-react'] },
  resolve: {
    dedupe: ['react', 'react-dom', 'three', '@react-three/fiber', '@react-three/drei'],
    alias: {
      react: path.resolve('./node_modules/react'),
      'react-dom': path.resolve('./node_modules/react-dom'),
      three: path.resolve('./node_modules/three'),
      '@react-three/fiber': path.resolve('./node_modules/@react-three/fiber'),
      '@react-three/drei': path.resolve('./node_modules/@react-three/drei'),
    },
  },
});
