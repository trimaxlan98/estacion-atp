// Datos del visor 3D del rover (equipo R3, CONTRATO §18): normaliza el JSON de /rover/estado, genera un terreno
// para dibujar (igual de determinista que el de Gazebo, src/atp_rover/gazebo/generar.py; NO es terreno.py de R1) y
// un recorrido de muestra para el modo mock. Todo puro (sin DOM ni three): se prueba con node --test.
//
// Marco: ENU como el contrato (x este, y norte, z arriba). El motor de dibujo lo pasa a three (x, z, -y).

export const TERRENOS = ['llano', 'rocoso', 'dunas', 'crater', 'pendiente']

// gravedad y colores por planeta (los mismos que los mundos de Gazebo, generar.py); `cielo`/`suelo` en hex sRGB
export const PLANETAS = {
  luna: { nombre: 'Luna', g: 1.62, cielo: '#03050A', suelo: '#7C7D82', niebla: '#0b0d12', sol: 1.25, ambiente: 0.35 },
  marte: { nombre: 'Marte', g: 3.71, cielo: '#B8794F', suelo: '#A04A28', niebla: '#B8794F', sol: 0.95, ambiente: 0.55 },
  titan: { nombre: 'Titán', g: 1.35, cielo: '#7A5A22', suelo: '#6B5A42', niebla: '#7A5A22', sol: 0.55, ambiente: 0.7 },
  ceres: { nombre: 'Ceres', g: 0.28, cielo: '#0A1030', suelo: '#7A746A', niebla: '#0a1030', sol: 1.0, ambiente: 0.35 },
  tierra: { nombre: 'Tierra', g: 9.81, cielo: '#6FA8DC', suelo: '#5E7F3C', niebla: '#6FA8DC', sol: 1.0, ambiente: 0.6 },
}

export const LADO_M = 60
export const PLANO_R = 3.5

/** PRNG determinista (mulberry32). */
export function prng(semilla) {
  let a = (semilla >>> 0) || 1
  return () => {
    a = (a + 0x6d2b79f5) >>> 0
    let t = a
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

/** Ruido de valor suave (interpolacion coseno) de `celdas` celdas de lado, en n x n; centrado en 0. */
function ruido(rnd, n, celdas) {
  const m = celdas + 2
  const g = new Float64Array(m * m)
  for (let i = 0; i < g.length; i++) g[i] = rnd()
  const out = new Float64Array(n * n)
  const idx = new Int32Array(n)
  const f = new Float64Array(n)
  for (let k = 0; k < n; k++) {
    const t = (k / n) * celdas
    idx[k] = Math.floor(t)
    f[k] = (1 - Math.cos((t - idx[k]) * Math.PI)) / 2
  }
  for (let r = 0; r < n; r++) {
    const i = idx[r], fr = f[r]
    for (let c = 0; c < n; c++) {
      const j = idx[c], fc = f[c]
      out[r * n + c] =
        g[i * m + j] * (1 - fc) * (1 - fr) + g[(i + 1) * m + j] * (1 - fc) * fr +
        g[i * m + j + 1] * fc * (1 - fr) + g[(i + 1) * m + j + 1] * fc * fr - 0.5
    }
  }
  return out
}

/**
 * Terreno {tipo, semilla, n, lado, z: Float32Array(n*n)} (fila = y creciente, columna = x creciente), centrado en
 * el origen, con z = 0 exacto en (0,0) y llano en PLANO_R m alrededor (el rover nace ahi).
 */
export function generarTerreno(tipo = 'rocoso', semilla = 1, n = 129, lado = LADO_M) {
  if (!TERRENOS.includes(tipo)) throw new Error(`terreno desconocido: ${tipo}`)
  const rnd = prng(semilla)
  const h = new Float64Array(n * n)
  const xs = (k) => -lado / 2 + (k / (n - 1)) * lado
  const suma = (r, a) => { for (let i = 0; i < h.length; i++) h[i] += a * r[i] }
  if (tipo === 'llano') suma(ruido(rnd, n, 24), 0.02)
  else if (tipo === 'rocoso') {
    for (const [a, c] of [[1.1, 4], [0.55, 9], [0.28, 20], [0.14, 44]]) suma(ruido(rnd, n, c), a)
    for (let q = 0; q < 70; q++) {
      const cx = -lado / 2 + 2 + rnd() * (lado - 4), cy = -lado / 2 + 2 + rnd() * (lado - 4)
      const r = 0.35 + rnd() * 0.55, a = 0.15 + rnd() * 0.4
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
        const d2 = (xs(j) - cx) ** 2 + (xs(i) - cy) ** 2
        if (d2 < 36) h[i * n + j] += a * Math.exp(-d2 / (2 * (r / 2) ** 2))
      }
    }
  } else if (tipo === 'dunas') {
    const fase = ruido(rnd, n, 5), p = ruido(rnd, n, 30), amp = ruido(rnd, n, 3)
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      const k = i * n + j
      h[k] = (0.55 * Math.sin((xs(j) * 0.9 + xs(i) * 0.35) * 0.55 + 13.2 * fase[k]) + 0.12 * p[k]) * (0.75 + 0.5 * (amp[k] + 0.5))
    }
  } else if (tipo === 'crater') {
    const rug = ruido(rnd, n, 14)
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      const d = Math.hypot(xs(j) - 13, xs(i) - 4) / 9
      const cuenco = d < 1 ? -2.2 * (1 - d * d) : 0
      h[i * n + j] = cuenco + 0.7 * Math.exp(-(((d - 1.05) / 0.22) ** 2)) + 0.18 * rug[i * n + j] +
        0.5 * Math.exp(-((Math.hypot(xs(j) + 16, xs(i) - 14) / 2.5) ** 2))
    }
  } else {
    const t = Math.tan((14 * Math.PI) / 180), p = ruido(rnd, n, 30)
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) h[i * n + j] = Math.max(xs(j) - 4, 0) * t + 0.06 * p[i * n + j]
  }
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    let w = Math.min(Math.max((Math.hypot(xs(j), xs(i)) - PLANO_R) / 3, 0), 1)
    w = w * w * (3 - 2 * w)
    h[i * n + j] *= w
  }
  const h0 = h[Math.floor(n / 2) * n + Math.floor(n / 2)]
  const z = new Float32Array(n * n)
  for (let k = 0; k < z.length; k++) z[k] = h[k] - h0
  return { tipo, semilla, n, lado, z }
}

/**
 * Terreno de una ESCENA (`args.terreno` y `args.terreno_params` de la corrida). El visor dibuja su propio relieve (el de arriba, que no es
 * el de terreno.py), asi que solo se reproduce el del simulador donde tiene formula cerrada: `pendiente` (plano que pasa por el origen,
 * `angulo_deg` y `direccion_deg`, sin el arranque llano del visor) y `escalones` (`altura`, `x_inicio`, `separacion`, `n`). Alli el suelo es
 * el mismo que el de la telemetria (`exacto: true`); sin esto, una pendiente lateral dejaba al rover enterrado. `rocas` se siembra aqui
 * (aproximado) y el resto usa el relieve del visor; en ambos casos `exacto: false`.
 */
export function terrenoDeEscena(tipo, semilla = 1, params = {}, lado = LADO_M) {
  const p = params && typeof params === 'object' ? params : {}
  const real = (v, d) => (v === undefined || v === null || v === '' || !Number.isFinite(Number(v)) ? d : Number(v))
  if (tipo === 'pendiente' || tipo === 'escalones') {
    const n = tipo === 'escalones' ? 241 : 129
    const xs = (k) => -lado / 2 + (k / (n - 1)) * lado
    const z = new Float32Array(n * n)
    if (tipo === 'pendiente') {
      const t = Math.tan((real(p.angulo_deg, 15) * Math.PI) / 180), d = (real(p.direccion_deg, 0) * Math.PI) / 180
      const c = Math.cos(d), s = Math.sin(d)
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) z[i * n + j] = t * (xs(j) * c + xs(i) * s)
    } else {
      const h = real(p.altura, 0.15), x0 = real(p.x_inicio, 3), sep = real(p.separacion, 6), nesc = Math.trunc(real(p.n, 1))
      for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
        let a = 0
        for (let e = 0; e < nesc; e++) if (xs(j) >= x0 + e * sep - 1e-9) a += h
        z[i * n + j] = a
      }
    }
    return { tipo, semilla, n, lado, z, exacto: true }
  }
  if (tipo === 'rocas') {
    const base = generarTerreno('llano', semilla, 241, lado)
    const n = base.n, z = Float32Array.from(base.z), cel = lado / (n - 1)
    const rnd = prng((semilla >>> 0) + 7919)
    const radio = real(p.radio_rocas, 30), libre = real(p.libre_m, 3), dmin = real(p.d_min, 0.15), dmax = real(p.d_max, 0.8)
    const nroc = Math.round(2400 * real(p.abundancia, 0.1))                    // ~240 con la abundancia por defecto (aproximado)
    for (let q = 0; q < nroc; q++) {
      const cx = (rnd() * 2 - 1) * radio, cy = (rnd() * 2 - 1) * radio
      const D = dmin + (dmax - dmin) * rnd() ** 3                               // muchas chicas y pocas grandes
      const alto = D * (0.3 + 0.3 * rnd()), R = D / 2
      if (Math.hypot(cx, cy) < libre) continue
      const j0 = Math.max(Math.floor((cx - R + lado / 2) / cel), 0), j1 = Math.min(Math.ceil((cx + R + lado / 2) / cel), n - 1)
      const i0 = Math.max(Math.floor((cy - R + lado / 2) / cel), 0), i1 = Math.min(Math.ceil((cy + R + lado / 2) / cel), n - 1)
      for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) {
        const d2 = (-lado / 2 + j * cel - cx) ** 2 + (-lado / 2 + i * cel - cy) ** 2
        if (d2 < R * R) z[i * n + j] = Math.max(z[i * n + j], alto * Math.sqrt(1 - d2 / (R * R)))
      }
    }
    return { ...base, tipo, z, exacto: false }
  }
  return { ...generarTerreno(TERRENOS.includes(tipo) ? tipo : 'llano', semilla, 129, lado), exacto: false }
}

/** Terreno recibido de un backend {tipo, lado_m|lado, n, z:[...]} -> el mismo formato; null si no sirve. */
export function normalizarTerreno(raw) {
  if (!raw || !Array.isArray(raw.z)) return null
  const n = raw.n ?? Math.round(Math.sqrt(raw.z.length))
  if (n < 2 || n * n !== raw.z.length) return null
  return { tipo: raw.tipo ?? '?', semilla: raw.semilla ?? 0, n, lado: raw.lado_m ?? raw.lado ?? LADO_M, z: Float32Array.from(raw.z) }
}

/** Altura bilineal del terreno en (x, y) m; fuera del mapa, el borde. */
export function alturaEn(ter, x, y) {
  const { n, lado, z } = ter
  const u = Math.min(Math.max(((x + lado / 2) / lado) * (n - 1), 0), n - 1 - 1e-9)
  const v = Math.min(Math.max(((y + lado / 2) / lado) * (n - 1), 0), n - 1 - 1e-9)
  const j = Math.floor(u), i = Math.floor(v), fu = u - j, fv = v - i
  return z[i * n + j] * (1 - fu) * (1 - fv) + z[i * n + j + 1] * fu * (1 - fv) + z[(i + 1) * n + j] * (1 - fu) * fv + z[(i + 1) * n + j + 1] * fu * fv
}

/** Pendiente del terreno (grados) y su gradiente [dz/dx, dz/dy] en (x, y). */
export function pendienteEn(ter, x, y) {
  const d = 0.25
  const gx = (alturaEn(ter, x + d, y) - alturaEn(ter, x - d, y)) / (2 * d)
  const gy = (alturaEn(ter, x, y + d) - alturaEn(ter, x, y - d)) / (2 * d)
  return { deg: (Math.atan(Math.hypot(gx, gy)) * 180) / Math.PI, gx, gy }
}

const num = (v) => (typeof v === 'number' && Number.isFinite(v) ? v : undefined)

/**
 * Normaliza filas de /rover/estado (objetos, o texto JSONL) a muestras ordenadas por t y sin repetidos.
 * Un campo sin dato se omite (contrato): aqui queda undefined. Se descartan las filas sin t, x o y.
 */
export function parsearEstados(entrada) {
  let filas = entrada
  if (typeof entrada === 'string') {
    filas = []
    for (const linea of entrada.split('\n')) {
      const s = linea.trim()
      if (!s) continue
      try { filas.push(JSON.parse(s)) } catch { /* una linea mala no tira la corrida */ }
    }
  }
  const out = []
  for (const f of filas || []) {
    const t = num(f?.t), x = num(f?.x), y = num(f?.y)
    if (t === undefined || x === undefined || y === undefined) continue
    // `deslizamiento` es un vector (una cifra por rueda, CONTRATO §18; una rueda sin dato queda como hueco para no mover las
    // demas de sitio); un escalar vale como una sola rueda. El medio y el maximo salen solo de las cifras validas.
    const desliz = Array.isArray(f.deslizamiento) ? f.deslizamiento.map(num) : f.deslizamiento === undefined ? undefined : [num(f.deslizamiento)]
    const validos = (desliz || []).filter((v) => v !== undefined)
    out.push({
      t, x, y, z: num(f.z), rumbo: num(f.rumbo_deg) ?? 0, cabeceo: num(f.cabeceo_deg), alabeo: num(f.alabeo_deg),
      v: num(f.v), w: num(f.w), pendiente: num(f.pendiente_deg), potencia: num(f.potencia_w), energia: num(f.energia_wh),
      atascado: !!f.atascado, desliz,
      deslizMedio: validos.length ? validos.reduce((s, v) => s + v, 0) / validos.length : undefined,
      deslizMax: validos.length ? Math.max(...validos) : undefined,
    })
  }
  out.sort((a, b) => a.t - b.t)
  return out.filter((m, i) => i === 0 || m.t > out[i - 1].t)
}

const angCorto = (a, b, f) => ((a + ((((b - a) % 360) + 540) % 360 - 180) * f) % 360 + 360) % 360
const mezcla = (a, b, f) => (a === undefined || b === undefined ? (f < 0.5 ? a : b) : a + (b - a) * f)

/**
 * Prepara la corrida para el visor: muestras, terreno, trayectoria (con z) y una funcion `en(t)` que interpola.
 * Si una muestra no trae z/cabeceo/alabeo se completan con el terreno (z del suelo + 0.62 m, inclinacion del suelo
 * proyectada sobre el rumbo), asi un estado minimo (solo x, y, rumbo) tambien se ve sobre el relieve.
 */
export function prepararRover({ estados, terreno, planeta = 'marte' }) {
  const m = parsearEstados(estados)
  const ter = terreno || generarTerreno('llano', 1)
  const Z_CHASIS = 0.62
  const completa = (s) => {
    const base = alturaEn(ter, s.x, s.y)
    const { gx, gy } = pendienteEn(ter, s.x, s.y)
    const r = (s.rumbo * Math.PI) / 180
    const alFrente = gx * Math.cos(r) + gy * Math.sin(r)      // dz/ds en el rumbo
    const aLado = -gx * Math.sin(r) + gy * Math.cos(r)        // dz/ds a la izquierda
    return {
      ...s,
      z: s.z ?? base + Z_CHASIS,
      cabeceo: s.cabeceo ?? (Math.atan(alFrente) * 180) / Math.PI,
      alabeo: s.alabeo ?? (Math.atan(aLado) * 180) / Math.PI,   // izquierda arriba = alabeo a la derecha
      suelo: base,
    }
  }
  const pts = m.map(completa)
  let dist = 0, pendMax = 0, vMax = 0
  const acum = new Float64Array(pts.length)
  for (let i = 0; i < pts.length; i++) {
    if (i) dist += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y)
    acum[i] = dist
    pendMax = Math.max(pendMax, pts[i].pendiente ?? pendienteEn(ter, pts[i].x, pts[i].y).deg)
    vMax = Math.max(vMax, Math.abs(pts[i].v ?? 0))
  }
  const t0 = pts.length ? pts[0].t : 0
  const t1 = pts.length ? pts[pts.length - 1].t : 0
  const indice = (t) => {
    let lo = 0, hi = pts.length - 1
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (pts[mid].t <= t) lo = mid; else hi = mid - 1 }
    return lo
  }
  const en = (t) => {
    if (!pts.length) return null
    const i = indice(t)
    const a = pts[i], b = pts[Math.min(i + 1, pts.length - 1)]
    const f = b.t > a.t ? Math.min(Math.max((t - a.t) / (b.t - a.t), 0), 1) : 0
    const d = acum[i] + (acum[Math.min(i + 1, pts.length - 1)] - acum[i]) * f
    return {
      t, i, dist: d, x: mezcla(a.x, b.x, f), y: mezcla(a.y, b.y, f), z: mezcla(a.z, b.z, f),
      rumbo: angCorto(a.rumbo, b.rumbo, f), cabeceo: mezcla(a.cabeceo, b.cabeceo, f), alabeo: mezcla(a.alabeo, b.alabeo, f),
      v: mezcla(a.v, b.v, f), w: mezcla(a.w, b.w, f), pendiente: a.pendiente ?? pendienteEn(ter, a.x, a.y).deg,
      energia: mezcla(a.energia, b.energia, f), potencia: a.potencia, atascado: a.atascado,
      deslizMedio: mezcla(a.deslizMedio, b.deslizMedio, f), deslizMax: a.deslizMax,
    }
  }
  // atascos: el instante en que `atascado` pasa a verdadero (marcas de la linea de tiempo)
  const atascos = []
  for (let i = 0; i < pts.length; i++) {
    if (pts[i].atascado && (i === 0 || !pts[i - 1].atascado)) atascos.push({ t: pts[i].t, x: pts[i].x, y: pts[i].y })
  }
  // que columnas trae la corrida (el panel solo muestra las que existen); se calcula una vez, no en cada actualizacion del panel
  const tiene = { energia: pts.some((p) => p.energia !== undefined), potencia: pts.some((p) => p.potencia !== undefined), desliz: pts.some((p) => p.deslizMedio !== undefined) }
  return { planeta: PLANETAS[planeta] ? planeta : 'marte', terreno: ter, pts, n: pts.length, t0, t1, dist, pendMax, vMax, atascos, tiene, en, indice }
}

/**
 * Recorrido de muestra (modo mock): un rover cinematico que sigue una ruta de comandos (v, w) sobre el terreno,
 * a 10 Hz como /rover/estado. Determinista. No es la dinamica de R1: solo sirve para ver el visor sin backend.
 */
export function mockRover({ planeta = 'marte', terreno = 'rocoso', semilla = 1, dur = 60, hz = 10 } = {}) {
  const ter = generarTerreno(terreno, semilla)
  const ruta = [[0, 0.0, 0.0], [2, 0.6, 0.0], [12, 0.5, 0.3], [22, 0.6, 0.0], [30, 0.5, -0.35], [40, 0.6, 0.0], [47, 0.0, 0.5], [51, 0.6, 0.0]]
  const g = PLANETAS[planeta]?.g ?? 3.71
  const estados = []
  let x = 0, y = 0, rumbo = 0, e = 0, v = 0, w = 0
  const dt = 1 / hz
  for (let k = 0; k <= dur * hz; k++) {
    const t = k * dt
    let [, vc, wc] = ruta[0]
    for (const r of ruta) if (t >= r[0]) [, vc, wc] = r
    const { deg, gx, gy } = pendienteEn(ter, x, y)
    const rr = (rumbo * Math.PI) / 180
    const subida = gx * Math.cos(rr) + gy * Math.sin(rr)
    const lento = Math.max(0.35, 1 - Math.max(subida, 0) * 1.4)        // la cuesta arriba frena
    v += (vc * lento - v) * Math.min(1, dt * 2)
    w += (wc - w) * Math.min(1, dt * 3)
    rumbo += (w * dt * 180) / Math.PI
    x += v * Math.cos((rumbo * Math.PI) / 180) * dt
    y += v * Math.sin((rumbo * Math.PI) / 180) * dt
    const pot = 40 + 120 * Math.abs(v) * (1 + Math.max(subida, 0) * 3) * (g / 3.71) + 6 * Math.abs(w)
    e += (pot * dt) / 3600
    const base = alturaEn(ter, x, y)
    estados.push({
      t: +t.toFixed(3), x: +x.toFixed(4), y: +y.toFixed(4), z: +(base + 0.62).toFixed(4), rumbo_deg: +rumbo.toFixed(3),
      v: +v.toFixed(4), w: +w.toFixed(4), pendiente_deg: +deg.toFixed(3), potencia_w: +pot.toFixed(2), energia_wh: +e.toFixed(5), atascado: false,
    })
  }
  return { estados, terreno: ter, planeta }
}
