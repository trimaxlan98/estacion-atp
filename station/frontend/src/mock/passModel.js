// Pase LEO sintetico para el modo mock. Geometria esferica real (551 km,
// circular) y una montura con retardo: el error crece donde el acimut gira
// mas rapido que el motor (keyhole, cerca del cenit).
// Produce muestras EXACTAMENTE con el formato de telemetry.jsonl (CONTRATO §2).

const RE = 6371.0 // km
const MU = 398600.4418 // km3/s2
const C = 299792.458 // km/s
const D2R = Math.PI / 180

export const BAND_HZ = { UHF: 437e6, S: 2.2e9, X: 8.2e9, Ka: 26e9 }
const BAND_BEAM = { UHF: 12, S: 5, X: 1.6, Ka: 0.45 } // ancho de haz -3 dB (deg), antena de ejemplo

// PRNG determinista (mulberry32) para que una corrida sea reproducible
export function rng(seed) {
  let a = seed >>> 0
  return () => {
    a |= 0
    a = (a + 0x6d2b79f5) | 0
    let t = Math.imul(a ^ (a >>> 15), 1 | a)
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}
export function gauss(rand) {
  const u = Math.max(rand(), 1e-12)
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand())
}
export function hashSeed(str) {
  let h = 2166136261
  for (let i = 0; i < str.length; i++) h = Math.imul(h ^ str.charCodeAt(i), 16777619)
  return h >>> 0
}

// Estado geometrico del satelite en el instante dado (phi = angulo a lo largo de la orbita)
function geometry(phi, lam0, R) {
  const u = [Math.sin(lam0), 0, Math.cos(lam0)]
  const v = [0, 1, 0]
  const p = [
    R * (Math.cos(phi) * u[0] + Math.sin(phi) * v[0]),
    R * (Math.cos(phi) * u[1] + Math.sin(phi) * v[1]),
    R * (Math.cos(phi) * u[2] + Math.sin(phi) * v[2]),
  ]
  const rel = [p[0], p[1], p[2] - RE]
  const rng_ = Math.hypot(rel[0], rel[1], rel[2])
  const el = Math.asin(rel[2] / rng_) / D2R
  let az = Math.atan2(rel[0], rel[1]) / D2R
  if (az < 0) az += 360
  return { az, el, range: rng_ }
}

/**
 * Genera el pase completo como arreglo de muestras a 10 Hz de tiempo simulado.
 * args: el_max (deg), mask (deg), band, alt_km, seed
 */
export function generatePass(args = {}) {
  const elMax = Math.min(Math.max(Number(args.el_max ?? 30), 5), 89.9)
  const mask = Number(args.mask ?? 5)
  const band = args.band in BAND_HZ ? args.band : 'S'
  const alt = Number(args.alt_km ?? 551)
  const seed = Number(args.seed ?? 7)
  const rand = rng(seed * 2654435761)
  const R = RE + alt
  const omega = Math.sqrt(MU / (R * R * R)) // rad/s
  const lam0 = Math.acos(Math.min(1, (RE / R) * Math.cos(elMax * D2R))) - elMax * D2R

  // ventana temporal: donde el > mask
  const tc = 0
  const dt = 0.1
  let t0 = -1, t1 = 1
  while (geometry(omega * t0, lam0, R).el > mask && t0 > -2000) t0 -= 1
  while (geometry(omega * t1, lam0, R).el > mask && t1 < 2000) t1 += 1
  const tStart = t0 + 1
  const tEnd = t1 - 1
  const dur = tEnd - tStart

  const f0 = BAND_HZ[band]
  const theta3 = BAND_BEAM[band]
  const rMin = geometry(0, lam0, R).range
  const rows = []
  let prev = null
  let prevAzRef = null
  const azrate = []
  // primer paso: referencia y derivadas
  const n = Math.floor(dur / dt) + 1
  const ref = new Array(n)
  for (let i = 0; i < n; i++) {
    const t = i * dt
    ref[i] = geometry(omega * (tStart + t), lam0, R)
  }
  const unwrap = (a, b) => {
    let d = b - a
    while (d > 180) d -= 360
    while (d < -180) d += 360
    return d
  }
  for (let i = 0; i < n; i++) {
    const j0 = Math.max(0, i - 1), j1 = Math.min(n - 1, i + 1)
    azrate.push(unwrap(ref[j0].az, ref[j1].az) / ((j1 - j0) * dt))
  }

  // montura: error de seguimiento ~ exceso de velocidad de acimut sobre el limite del motor
  const AZ_MAX_RATE = 5 // deg/s, saturacion del motor
  let eAz = 0, eEl = 0
  let rrPrev = null, dopPrev = null
  for (let i = 0; i < n; i++) {
    const t = i * dt
    const r = ref[i]
    const w = azrate[i]
    const excess = Math.max(0, Math.abs(w) - AZ_MAX_RATE)
    const base = 0.017
    // el error de acimut sigue al exceso con una constante de tiempo (se acumula y se recupera)
    const target = excess * 0.1 + Math.abs(w) * 0.0009
    eAz += (target - eAz) * 0.25
    const nAz = gauss(rand) * base
    const nEl = gauss(rand) * base * 0.55
    eEl += (Math.abs(nEl) * 0.8 + Math.abs(ref[Math.min(n - 1, i + 1)].el - ref[Math.max(0, i - 1)].el) * 0.01 - eEl) * 0.3
    const e_az = Math.abs(eAz + nAz)
    const e_el = Math.abs(eEl + nEl)
    const e_pt = Math.hypot(e_az, e_el)
    const sgn = w >= 0 ? 1 : -1
    const az = (r.az - sgn * e_az + 360) % 360
    const el = r.el - e_el * 0.6

    // Doppler: f = -(dR/dt)/c * f0
    const jj0 = Math.max(0, i - 1), jj1 = Math.min(n - 1, i + 1)
    const rr = (ref[jj1].range - ref[jj0].range) / ((jj1 - jj0) * dt)
    const dop = (-rr / C) * f0
    let dopRate = 0
    if (dopPrev != null) dopRate = (dop - dopPrev) / dt
    dopPrev = dop
    rrPrev = rr

    const lpoint = 12 * Math.pow(e_pt / theta3, 2) // dB, perdida por desapuntamiento
    const margin = 17.68 - 20 * Math.log10(r.range / rMin) * 0.62 - lpoint
    const ebn0 = margin + 10.5
    const cn0 = ebn0 + 10 * Math.log10(9600)
    const noise = (x, s) => x + gauss(rand) * s
    const row = {
      t: round(t, 2),
      ref_az: round(r.az, 3),
      ref_el: round(r.el, 3),
      az: round(az, 3),
      el: round(el, 3),
      e_az: round(e_az, 4),
      e_el: round(e_el, 4),
      e_pt: round(e_pt, 4),
      range_km: round(r.range, 2),
      range_rate_km_s: round(rr, 3),
      doppler_hz: round(dop, 1) + 0,
      doppler_rate_hz_s: round(i === 0 ? undefined : dopRate, 1),
      l_point_db: round(lpoint, 4),
      cn0_dbhz: round(noise(cn0, 0.03), 2),
      ebn0_db: round(noise(ebn0, 0.03), 2),
      margin_db: round(noise(margin, 0.03), 2),
      link_closed: margin > 0,
      u_az: round(noise(w * 0.01, 0.02), 3),
      u_el: round(noise(0, 0.02), 3),
    }
    // un campo sin dato se omite
    if (row.doppler_rate_hz_s === undefined) delete row.doppler_rate_hz_s
    rows.push(row)
  }
  void prev; void prevAzRef; void rrPrev; void tc
  return rows
}

function round(x, d) {
  if (x === undefined) return undefined
  const k = Math.pow(10, d)
  return Math.round(x * k) / k
}

function percentile(sortedAsc, p) {
  if (!sortedAsc.length) return 0
  const idx = (sortedAsc.length - 1) * p
  const lo = Math.floor(idx), hi = Math.ceil(idx)
  return sortedAsc[lo] + (sortedAsc[hi] - sortedAsc[lo]) * (idx - lo)
}

/** metrics.json a partir de las muestras (CONTRATO §3). */
export function computeMetrics(rows, args = {}) {
  const objective = 0.1
  const n = rows.length
  const e = rows.map((r) => r.e_pt)
  const rms = (a) => Math.sqrt(a.reduce((s, x) => s + x * x, 0) / Math.max(1, a.length))
  const sorted = [...e].sort((a, b) => a - b)
  const eaz = rows.map((r) => r.e_az), eel = rows.map((r) => r.e_el)
  const margins = rows.map((r) => r.margin_db)
  const dop = rows.map((r) => r.doppler_hz)
  const dr = rows.map((r) => r.doppler_rate_hz_s ?? 0)
  const within = e.filter((x) => x <= objective).length
  const p95 = percentile(sorted, 0.95)
  const closedPct = (100 * rows.filter((r) => r.link_closed).length) / Math.max(1, n)
  const r2 = (x, d = 3) => Math.round(x * 10 ** d) / 10 ** d
  const pointingRms = rms(e)
  const marginMin = Math.min(...margins)
  return {
    n,
    duration_s: r2(rows.length ? rows[rows.length - 1].t : 0, 1),
    pointing: {
      rms: r2(pointingRms, 4),
      p95: r2(p95, 4),
      max: r2(Math.max(...e), 3),
      within_objective_pct: r2((100 * within) / Math.max(1, n), 1),
      objective_deg: objective,
    },
    axes: {
      az: { rms: r2(rms(eaz), 4), max: r2(Math.max(...eaz), 3) },
      el: { rms: r2(rms(eel), 4), max: r2(Math.max(...eel), 3) },
    },
    link: {
      band: args.band in BAND_HZ ? args.band : 'S',
      margin_min_db: r2(marginMin, 2),
      margin_max_db: r2(Math.max(...margins), 2),
      closed_pct: r2(closedPct, 1),
      doppler_max_hz: Math.round(Math.max(...dop)),
      doppler_min_hz: Math.round(Math.min(...dop)),
      doppler_rate_max_hz_s: r2(Math.max(...dr.map(Math.abs)), 1),
      l_point_max_db: r2(Math.max(...rows.map((r) => r.l_point_db)), 3),
    },
    verdict: pointingRms <= objective && p95 <= objective && marginMin > 0 && closedPct >= 99 ? 'CUMPLE' : 'NO CUMPLE',
  }
}

/** campaign.json (CONTRATO §4): rms log-normal con cola por el keyhole. */
export function generateCampaign(args = {}) {
  const n = Math.min(Math.max(Math.round(Number(args.n ?? 300)), 10), 5000)
  const mode = args.mode || 'lazo'
  const rand = rng(hashSeed(JSON.stringify(args)) ^ 0x9e3779b9)
  const objective = 0.1
  const rms = [], elMax = []
  for (let i = 0; i < n; i++) {
    // elevaciones maximas: mas pases bajos que cenitales (distribucion tipica de pases)
    const u = rand()
    const el = 8 + 81.5 * Math.pow(u, 1.55)
    elMax.push(el)
    let base = Math.exp(Math.log(0.029) + 0.28 * gauss(rand)) // log-normal
    // keyhole: el error explota cuando el pase se acerca al cenit
    const k = Math.max(0, (el - 80) / 9.9)
    base += 0.02 * Math.pow(k, 2.2) * Math.exp(0.6 * gauss(rand)) * 22
    if (mode === 'lazo_abierto') base *= 2.4
    rms.push(base)
  }
  const sorted = [...rms].sort((a, b) => a - b)
  const p = (q) => percentile(sorted, q)
  const r4 = (x) => Math.round(x * 1e4) / 1e4
  const p95 = p(0.95)
  // incertidumbre binomial del percentil: sqrt(p(1-p)/n) en puntos porcentuales
  const unc = (Math.sqrt(0.95 * 0.05 / n) * 100)
  return {
    mode,
    n,
    p50: r4(p(0.5)),
    p95: r4(p95),
    p99: r4(p(0.99)),
    peor: r4(sorted[sorted.length - 1]),
    pasa: p95 <= objective,
    objective_deg: objective,
    p95_uncertainty_pts: Math.round(unc * 10) / 10,
    wall_time_s: Math.round((n * 0.034 + 0.4) * 10) / 10,
    rms_deg: rms.map(r4),
    el_max_deg: elMax.map((x) => Math.round(x * 10) / 10),
  }
}
