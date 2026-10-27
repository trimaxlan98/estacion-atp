// Mide el visor 3D del ROVER (#/rover) con Chromium headless. Por defecto WebGL por SOFTWARE (SwiftShader, el peor caso).
// Es el equivalente de perf_visor.mjs para el rover: aquel lee data-fps/data-jsms/data-glms que publica el visor de red,
// y el visor del rover no los publica, asi que aqui se mide por fuera:
//   fps_raf  = cuadros por segundo del navegador (requestAnimationFrame) mientras el rover se reproduce;
//   fps_p1   = peor 1 % de los cuadros (1000 / percentil 99 del tiempo entre cuadros);
//   fps_hud  = el contador que muestra el propio visor (cuadros que dibujo de verdad el motor, no solo los del navegador).
//
// Uso:  BASE=http://127.0.0.1:8090 node tools/perf_rover.mjs        (recorrido de muestra: ?mock=1#/rover, sin backend)
//       BASE=http://127.0.0.1:8090 RUN=<run_id> node tools/perf_rover.mjs   (ademas, una corrida REAL del rover: #/runs/<id>/rover;
//                                                                          sin ?mock=1, el backend tiene que estar arriba)
//
// Variables opcionales (sin ellas el comportamiento es por software):
//   GPU=1      Chromium con aceleracion real. Lee el renderer WebGL y aborta (codigo 2) si es SwiftShader o llvmpipe.
//   ANGLE=...  backend de ANGLE con GPU=1 (por defecto d3d11 en Windows y gl-egl en Linux; tambien vulkan o gl).
//   UNCAP=1    quita el limite de 60 Hz (--disable-gpu-vsync --disable-frame-rate-limit) para medir la capacidad real.
//   SYNC=1     tras cada cuadro espera a que la GPU termine (gl.finish): con UNCAP=1 mide la capacidad real de la GPU.
//   SIZE=1920x1080   tamano del viewport (por defecto 1366x768).
//   REPEAT=3         repite cada caso y reporta la mediana.
//   CHROME=ruta      ejecutable de Chromium/Chrome/Edge en lugar del que instalo Playwright.
import { chromium } from 'playwright-core'
import { existsSync, readdirSync } from 'node:fs'
import { homedir } from 'node:os'
import { join } from 'node:path'

const BASE = (process.env.BASE || 'http://127.0.0.1:5173').replace(/\/$/, '')
const GPU = process.env.GPU === '1'
const UNCAP = process.env.UNCAP === '1'
const REPEAT = Math.max(1, Number(process.env.REPEAT || 1))
const SYNC = process.env.SYNC === '1'
const RUN = process.env.RUN || ''
const ANGLE = process.env.ANGLE || (process.platform === 'win32' ? 'd3d11' : 'gl-egl')

function buscarChromium() {
  if (process.env.CHROME) return process.env.CHROME
  const root = process.platform === 'win32' ? join(process.env.LOCALAPPDATA || homedir(), 'ms-playwright') : join(homedir(), '.cache', 'ms-playwright')
  if (!existsSync(root)) return undefined
  const d = readdirSync(root).filter((x) => x.startsWith('chromium-')).sort().reverse()[0]
  if (!d) return undefined
  for (const rel of ['chrome-linux64/chrome', 'chrome-win64/chrome.exe', 'chrome-win/chrome.exe']) {
    if (existsSync(join(root, d, rel))) return join(root, d, rel)
  }
  return undefined
}

// Recorrido de muestra (datos sinteticos de lib/rover3d.js): planeta, terreno y camara. 'seguir' es la camara por omision;
// 'mapa' deja ver todo el terreno (mas relleno de pantalla).
const CASOS = [
  { nombre: 'muestra marte / rocoso, camara seguir', planeta: 'marte', terreno: 'rocoso', vista: 'seguir' },
  { nombre: 'muestra marte / rocoso, camara mapa', planeta: 'marte', terreno: 'rocoso', vista: 'mapa' },
  { nombre: 'muestra luna / crater, camara seguir', planeta: 'luna', terreno: 'crater', vista: 'seguir' },
  { nombre: 'muestra titan / dunas, camara seguir', planeta: 'titan', terreno: 'dunas', vista: 'seguir' },
]
if (RUN) CASOS.push({ nombre: `corrida real ${RUN}, camara seguir`, run: RUN, vista: 'seguir' }, { nombre: `corrida real ${RUN}, camara mapa`, run: RUN, vista: 'mapa' })

const argsSoft = ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']
const argsGpu = ['--no-sandbox', '--use-gl=angle', `--use-angle=${ANGLE}`, '--ignore-gpu-blocklist', '--enable-gpu-rasterization', '--enable-zero-copy']
if (UNCAP) argsGpu.push('--disable-gpu-vsync', '--disable-frame-rate-limit')
const b = await chromium.launch({ executablePath: buscarChromium(), headless: true, args: GPU ? argsGpu : argsSoft })
const size = (process.env.SIZE || '1366x768').split('x').map(Number)
const p = await b.newPage({ viewport: { width: size[0], height: size[1] } })
const errores = []
p.on('pageerror', (e) => { errores.push(e.message); console.log('PAGEERR', e.message) })

// Renderer WebGL que usa de verdad esta pagina (en un canvas aparte, sin tocar el del visor)
await p.goto(`${BASE}/?mock=1#/`)
await p.waitForSelector('[data-testid=cupos]', { timeout: 30000 })
const renderer = await p.evaluate(() => {
  const gl = document.createElement('canvas').getContext('webgl2') || document.createElement('canvas').getContext('webgl')
  if (!gl) return 'SIN WEBGL'
  const e = gl.getExtension('WEBGL_debug_renderer_info')
  return e ? gl.getParameter(e.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER)
})
console.log(JSON.stringify({ modo: GPU ? `gpu (ANGLE ${ANGLE}${UNCAP ? ', sin limite de 60 Hz' : ''}${SYNC ? ', gl.finish' : ''})` : 'software (SwiftShader)', renderer, viewport: size.join('x') }))
if (GPU && /swiftshader|llvmpipe|SIN WEBGL/i.test(renderer)) {
  console.log('ERROR: GPU=1 pero Chromium cayo a software; la medicion no sirve:', renderer)
  await b.close()
  process.exit(2)
}

const mediana = (xs) => { const s = [...xs].sort((a, c) => a - c); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2 }

async function abrir(c) {
  if (c.run) {
    await p.goto(`${BASE}/#/runs/${encodeURIComponent(c.run)}/rover`)
    await p.reload() // un goto al mismo origen con otro hash no remonta la app: se recarga limpio, sin ?mock=1
  } else {
    await p.goto(`${BASE}/?mock=1#/rover`)
    await p.reload()
    await p.waitForSelector('select[aria-label=Planeta]', { timeout: 30000 })
    await p.selectOption('select[aria-label=Planeta]', c.planeta)
    await p.selectOption('select[aria-label=Terreno]', c.terreno)
  }
  await p.waitForSelector('[data-testid=rover3d-canvas] canvas', { timeout: 60000 })
  await p.getByRole('button', { name: c.vista === 'mapa' ? 'Mapa' : 'Seguir', exact: true }).click()
  // muestreador de tiempos entre cuadros (rAF); con SYNC=1 espera a que la GPU TERMINE cada cuadro
  await p.evaluate((sync) => {
    window.__stop = true
    window.__cuadros = []
    window.__stop = false
    const tic = (t) => {
      if (window.__stop) return
      window.__cuadros.push(t)
      if (sync) {
        setTimeout(() => {
          const cv = document.querySelector('[data-testid=rover3d-canvas] canvas')
          const gl = cv && (cv.getContext('webgl2') || cv.getContext('webgl'))
          if (gl) gl.finish()
        }, 0)
      }
      requestAnimationFrame(tic)
    }
    requestAnimationFrame(tic)
  }, SYNC)
}

async function medirCaso(c) {
  await abrir(c)
  await p.getByRole('button', { name: /Reproducir/ }).click()
  const hud = []
  for (let i = 0; i < 6; i++) {
    await p.waitForTimeout(1200)
    if (i === 1) await p.evaluate(() => { window.__cuadros = [] }) // descarta el calentamiento: solo cuentan las ultimas 4 muestras
    hud.push(await p.locator('.visor3d__fps').innerText().then((t) => Number((t.match(/(\d+) fps/) || [])[1] || 0)).catch(() => 0))
  }
  const marcas = await p.evaluate(() => { window.__stop = true; return window.__cuadros.slice() })
  const reproduciendo = await p.getByRole('button', { name: /Pausa/ }).count()
  await p.getByRole('button', { name: /Pausa/ }).click().catch(() => {})
  const dt = []
  for (let i = 1; i < marcas.length; i++) dt.push(marcas[i] - marcas[i - 1])
  dt.sort((a, c2) => a - c2)
  const media = dt.reduce((s, x) => s + x, 0) / (dt.length || 1)
  const p99 = dt[Math.min(dt.length - 1, Math.floor(dt.length * 0.99))] || 0
  const ult = hud.slice(2)
  return { fps_raf: media ? 1000 / media : 0, fps_p1: p99 ? 1000 / p99 : 0, fps_hud: ult.reduce((s, x) => s + x, 0) / (ult.length || 1), cuadros: dt.length, reproduciendo: reproduciendo > 0 }
}

for (const c of CASOS) {
  const reps = []
  for (let i = 0; i < REPEAT; i++) reps.push(await medirCaso(c))
  const med = (k) => mediana(reps.map((r) => r[k]))
  console.log(JSON.stringify({
    caso: c.nombre, repeticiones: REPEAT,
    fps_raf: med('fps_raf').toFixed(2), fps_p1: med('fps_p1').toFixed(2), fps_hud: med('fps_hud').toFixed(2),
    cuadros: Math.round(med('cuadros')), seguia_reproduciendo: reps.every((r) => r.reproduciendo),
  }))
}
if (errores.length) console.log('ERRORES DE PAGINA:', errores.length, errores[0])
await b.close()
