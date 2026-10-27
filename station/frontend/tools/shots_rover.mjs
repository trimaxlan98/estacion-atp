// Capturas del visor 3D del rover (modo mock) con Chromium headless + SwiftShader (sin GPU).
// Uso: BASE=http://127.0.0.1:4173 node tools/shots_rover.mjs [salida]   (sirve la app antes: npm run preview)
// Sale en docs/ (o en la carpeta indicada): rover-<planeta>.png. No toca nada del backend.
import { chromium } from 'playwright-core'
import { existsSync, readdirSync, mkdirSync } from 'node:fs'
import { homedir } from 'node:os'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const BASE = process.env.BASE || 'http://127.0.0.1:4173'
const OUT = process.argv[2] || join(dirname(fileURLToPath(import.meta.url)), '..', 'docs')
mkdirSync(OUT, { recursive: true })

function findChromium() {
  if (process.env.PW_CHROMIUM) return process.env.PW_CHROMIUM
  const root = join(homedir(), '.cache', 'ms-playwright')
  if (!existsSync(root)) return undefined
  for (const d of readdirSync(root).filter((d) => d.startsWith('chromium')).sort().reverse()) {
    for (const rel of ['chrome-linux64/chrome', 'chrome-linux/chrome', 'chrome-linux/headless_shell', 'chrome-headless-shell-linux64/chrome-headless-shell']) {
      const p = join(root, d, rel)
      if (existsSync(p)) return p
    }
  }
}

const browser = await chromium.launch({
  executablePath: findChromium(),
  args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
})
const page = await browser.newPage({ viewport: { width: 1440, height: 1240 } })
const errores = []
page.on('pageerror', (e) => errores.push('pageerror: ' + e.message))
page.on('console', (m) => m.type() === 'error' && errores.push('console: ' + m.text()))

const casos = [['marte', 'rocoso'], ['luna', 'crater'], ['titan', 'dunas'], ['ceres', 'pendiente'], ['tierra', 'llano']]
for (const [planeta, terreno] of casos) {
  await page.goto(`${BASE}/?mock=1#/rover`)
  await page.waitForSelector('[data-testid=rover3d-canvas] canvas')
  await page.selectOption('select[aria-label=Planeta]', planeta)
  await page.selectOption('select[aria-label=Terreno]', terreno)
  await page.waitForTimeout(800)
  // avanza al 55 % de la corrida para ver el recorrido ya trazado
  await page.evaluate(() => {
    const r = document.querySelector('input[aria-label="Instante de la corrida"]')
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set
    set.call(r, String(Number(r.min) + 0.55 * (Number(r.max) - Number(r.min))))
    r.dispatchEvent(new Event('input', { bubbles: true }))
  })
  await page.waitForTimeout(900)
  await page.screenshot({ path: join(OUT, `rover-${planeta}.png`), fullPage: true })
  console.log('captura', planeta, terreno)
}
await browser.close()
if (errores.length) { console.log(errores.join('\n')); process.exit(1) }
