import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Las llamadas usan rutas relativas /api: no hay URL horneada en el build.
// El proxy es solo para desarrollo (backend FastAPI de la Estacion en :8090).
const API = process.env.ATP_API_PROXY || 'http://127.0.0.1:8090'

export default defineConfig({
  plugins: [react()],
  base: './',
  server: { proxy: { '/api': { target: API, changeOrigin: true } } },
  preview: { proxy: { '/api': { target: API, changeOrigin: true } } },
  build: { chunkSizeWarningLimit: 700 },
})
