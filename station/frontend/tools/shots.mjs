// Capturas de verificacion con Chromium headless (playwright-core).
// Uso: BASE=http://127.0.0.1:4173 node tools/shots.mjs   (sirve la app antes: npm run preview)
// Chromium: PW_CHROMIUM=/ruta/al/binario  o el de ~/.cache/ms-playwright.
import { chromium } from 'playwright-core'
import { existsSync, readdirSync, mkdirSync } from 'node:fs'
import { homedir } from 'node:os'
import { join, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const BASE = process.env.BASE || 'http://127.0.0.1:4173'
const OUT = join(dirname(fileURLToPath(import.meta.url)), '..', 'docs')
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

const browser = await chromium.launch({ executablePath: findChromium(), args: ['--no-sandbox'] })
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } })
const errors = []
page.on('pageerror', (e) => errors.push('pageerror: ' + e.message))
page.on('console', (m) => m.type() === 'error' && errors.push('console: ' + m.text()))
const shot = (name, full = true) => page.screenshot({ path: join(OUT, name + '.png'), fullPage: full })

// 1. Experimentos
await page.goto(BASE + '/#/experimentos')
await page.waitForSelector('.card')
await shot('01-experimentos')

// 2. Lanzar el pase oblicuo con reloj x50 y ver la corrida en vivo
const card = page.locator('.card', { hasText: 'Pase oblicuo completo en banda S' })
await card.locator('input[type=number]').nth(2).fill('50')
await card.getByRole('button', { name: 'Lanzar' }).click()
await page.waitForURL(/#\/runs\//)
await page.waitForTimeout(4500)
await shot('02-corrida-en-vivo')
await page.waitForSelector('.verdict', { timeout: 40000 })
await page.waitForTimeout(800)
await shot('03-corrida-terminada')

// 3. Pase casi cenital sembrado (keyhole) desde la biblioteca
await page.goto(BASE + '/#/biblioteca')
await page.waitForSelector('.table tbody tr')
await shot('04-biblioteca')
await page.locator('.table tbody tr', { hasText: 'cenital' }).first().getByRole('link', { name: 'Abrir' }).click()
await page.waitForSelector('.verdict')
await page.waitForTimeout(800)
await shot('05-keyhole')

// 4. Comparar dos corridas
await page.goto(BASE + '/#/biblioteca')
await page.waitForSelector('.table tbody tr')
const loops = page.locator('.table tbody tr:not(:has(.tag--campaign)) input[type=checkbox]')
await loops.nth(0).check()
await loops.nth(1).check()
await page.getByRole('button', { name: /Comparar/ }).click()
await page.waitForSelector('.vs')
await page.waitForTimeout(800)
await shot('06-comparar')

// 5. Campanas
await page.goto(BASE + '/#/campanas')
await page.waitForSelector('.verdict')
await page.waitForTimeout(500)
await shot('07-campanas')

// 6. Movil
await page.setViewportSize({ width: 390, height: 844 })
await page.goto(BASE + '/#/experimentos')
await page.waitForSelector('.card')
await shot('08-movil-experimentos')

await browser.close()
console.log(errors.length ? 'ERRORES:\n' + errors.join('\n') : 'sin errores de consola')
