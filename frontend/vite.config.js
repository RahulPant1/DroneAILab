// Plain JS (not TS) so we don't need @types/node. Ports come from .env via run.sh.
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

const backendPort = process.env.BACKEND_PORT || '8001';
const frontendPort = Number(process.env.FRONTEND_PORT || '5174');

export default defineConfig({
  plugins: [react()],
  server: {
    host: '127.0.0.1', // never expose to the network
    port: frontendPort,
    strictPort: true, // run.sh picks a free port; fail loudly rather than drift
    proxy: {
      '/api': `http://127.0.0.1:${backendPort}`,
      '/ws': { target: `ws://127.0.0.1:${backendPort}`, ws: true },
    },
  },
  preview: { host: '127.0.0.1', port: frontendPort },
  build: { chunkSizeWarningLimit: 1500 }, // three.js alone is ~1 MB; fine for a local app
});
