// Miniaturas procedurales (SVG como data-URI, 640x360): relleno de las escenas/medios sin miniatura y del mock.
// Marca Co.De: fondo #080F15, cian #00D9FF, ambar #F59E0B solo para el aviso, plata para el trazo fino.

const CIAN = '#00D9FF'
const AMBAR = '#F59E0B'
const PLATA = '#F2F3F4'
const TENUE = '#8E99A6'

function rng(seed) {
  let s = (seed >>> 0) || 1
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0
    return s / 4294967296
  }
}
function hash(str) {
  let h = 2166136261
  for (let i = 0; i < str.length; i++) { h ^= str.charCodeAt(i); h = Math.imul(h, 16777619) }
  return h >>> 0
}

const wrap = (inner) =>
  `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 360" width="640" height="360">` +
  `<defs><radialGradient id="g" cx="50%" cy="62%" r="70%"><stop offset="0" stop-color="#0d2230"/><stop offset="1" stop-color="#080F15"/></radialGradient></defs>` +
  `<rect width="640" height="360" fill="url(#g)"/>${inner}</svg>`

const uri = (svg) => 'data:image/svg+xml;utf8,' + encodeURIComponent(svg)

function stars(r, n) {
  let o = ''
  for (let i = 0; i < n; i++) o += `<circle cx="${(r() * 640).toFixed(0)}" cy="${(r() * 360).toFixed(0)}" r="${(0.4 + r() * 0.9).toFixed(1)}" fill="${PLATA}" opacity="${(0.15 + r() * 0.5).toFixed(2)}"/>`
  return o
}

function dome(r, withSats = true, n = 9) {
  // cupula en perspectiva simple: elipses del horizonte y de 30/60 grados, satelites y un haz
  let o = stars(r, 50)
  const cx = 320
  const cy = 250
  for (const k of [1, 0.66, 0.33]) o += `<ellipse cx="${cx}" cy="${cy}" rx="${230 * k}" ry="${62 * k}" fill="none" stroke="${PLATA}" stroke-opacity="${k === 1 ? 0.5 : 0.22}" stroke-width="${k === 1 ? 1.6 : 1}"/>`
  o += `<path d="M ${cx - 230} ${cy} A 230 190 0 0 1 ${cx + 230} ${cy}" fill="none" stroke="${PLATA}" stroke-opacity="0.2"/>`
  o += `<path d="M ${cx} ${cy - 190} L ${cx} ${cy + 62}" stroke="${PLATA}" stroke-opacity="0.12"/>`
  const pts = []
  for (let i = 0; i < n; i++) {
    const a = r() * Math.PI
    const rr = 0.25 + r() * 0.7
    pts.push([cx + Math.cos(a) * 225 * rr, cy - Math.sin(a) * 175 * rr * (0.4 + 0.6 * r()) - 12])
  }
  if (withSats) {
    pts.forEach(([x, y], i) => {
      const served = i < 2
      if (served) o += `<line x1="${cx + (i ? 30 : -30)}" y1="${cy - 4}" x2="${x}" y2="${y}" stroke="${CIAN}" stroke-width="2" opacity="0.8"/>`
      o += `<path d="M ${x - 20} ${y + 8} Q ${x - 10} ${y + 3} ${x} ${y}" stroke="${CIAN}" stroke-opacity="0.35" fill="none"/>`
      o += `<circle cx="${x}" cy="${y}" r="${served ? 5.5 : 3.4}" fill="${served ? PLATA : TENUE}"/>`
      if (served) o += `<circle cx="${x}" cy="${y}" r="9" fill="none" stroke="${CIAN}" stroke-width="1.6"/>`
    })
    const [hx, hy] = pts[pts.length - 1]
    o += `<circle cx="${hx}" cy="${hy}" r="10" fill="none" stroke="${AMBAR}" stroke-width="2" stroke-dasharray="3 3"/>`
  }
  o += `<circle cx="${cx - 30}" cy="${cy - 2}" r="6" fill="${CIAN}"/><circle cx="${cx + 30}" cy="${cy - 2}" r="6" fill="${CIAN}"/>`
  return o
}

function polar(r) {
  let o = stars(r, 30)
  const cx = 320
  const cy = 185
  for (const k of [1, 0.66, 0.33]) o += `<circle cx="${cx}" cy="${cy}" r="${150 * k}" fill="none" stroke="${PLATA}" stroke-opacity="${k === 1 ? 0.45 : 0.18}"/>`
  o += `<line x1="${cx - 150}" y1="${cy}" x2="${cx + 150}" y2="${cy}" stroke="${PLATA}" stroke-opacity="0.14"/><line x1="${cx}" y1="${cy - 150}" x2="${cx}" y2="${cy + 150}" stroke="${PLATA}" stroke-opacity="0.14"/>`
  const a0 = r() * 1.2 + 0.2
  let d = ''
  for (let i = 0; i <= 40; i++) {
    const u = i / 40
    const th = a0 + u * 2.4
    const rr = 150 * (1 - Math.sin(u * Math.PI) * 0.92)
    d += `${i ? 'L' : 'M'} ${(cx + Math.cos(th) * rr).toFixed(1)} ${(cy - Math.sin(th) * rr).toFixed(1)} `
  }
  o += `<path d="${d}" fill="none" stroke="${CIAN}" stroke-width="3" stroke-linecap="round"/>`
  o += `<circle cx="${cx}" cy="${cy}" r="10" fill="none" stroke="${AMBAR}" stroke-width="2"/>`
  return o
}

function gantt(r) {
  let o = ''
  for (let row = 0; row < 4; row++) {
    const y = 60 + row * 66
    o += `<line x1="40" y1="${y + 40}" x2="600" y2="${y + 40}" stroke="${PLATA}" stroke-opacity="0.12"/>`
    let x = 40 + r() * 40
    while (x < 580) {
      const w = 14 + r() * 56
      o += `<rect x="${x.toFixed(0)}" y="${y}" width="${w.toFixed(0)}" height="30" rx="4" fill="${r() > 0.82 ? AMBAR : CIAN}" opacity="${(0.55 + r() * 0.4).toFixed(2)}"/>`
      x += w + 10 + r() * 60
    }
  }
  return o
}

function wave(r) {
  let o = stars(r, 25)
  const lines = [[CIAN, 0.9], [PLATA, 0.5], [AMBAR, 0.6]]
  lines.forEach(([c, op], i) => {
    let d = ''
    const f = 1.2 + r() * 2.5
    const ph = r() * 6
    for (let x = 0; x <= 600; x += 8) d += `${x ? 'L' : 'M'} ${40 + x} ${(120 + i * 55 + Math.sin(x / 600 * f * 6.28 + ph) * (28 - i * 5) * Math.exp(-x / 520)).toFixed(1)} `
    o += `<path d="${d}" fill="none" stroke="${c}" stroke-width="2.4" opacity="${op}" stroke-linecap="round"/>`
  })
  return o
}

function chart(r) {
  let o = `<line x1="60" y1="300" x2="600" y2="300" stroke="${PLATA}" stroke-opacity="0.4"/><line x1="60" y1="40" x2="60" y2="300" stroke="${PLATA}" stroke-opacity="0.4"/>`
  for (let i = 1; i < 5; i++) o += `<line x1="60" y1="${300 - i * 52}" x2="600" y2="${300 - i * 52}" stroke="${PLATA}" stroke-opacity="0.08"/>`
  let d = ''
  for (let x = 0; x <= 540; x += 10) d += `${x ? 'L' : 'M'} ${60 + x} ${(220 - Math.sin(x / 90) * 50 - Math.exp(-x / 140) * 70 + (r() - 0.5) * 8).toFixed(1)} `
  o += `<path d="${d}" fill="none" stroke="${CIAN}" stroke-width="2.6"/>`
  o += `<line x1="60" y1="260" x2="600" y2="260" stroke="${AMBAR}" stroke-dasharray="6 5" stroke-width="1.6"/>`
  return o
}

/** Miniatura SVG (data-URI) segun el area/estilo y una semilla textual estable. */
export function thumb(kind, seed = '') {
  const r = rng(hash(kind + ':' + seed))
  let inner
  if (kind === 'red' || kind === 'ntn' || kind === 'visor') inner = dome(r, true, 9)
  else if (kind === 'seguidor' || kind === 'control' || kind === 'lazo') inner = polar(r)
  else if (kind === 'plan' || kind === 'mision') inner = gantt(r)
  else if (kind === 'adcs') inner = wave(r)
  else if (kind === 'grafica') inner = chart(r)
  else if (kind === 'rf') inner = chart(r) + stars(r, 20)
  else inner = dome(r, false, 0)
  return uri(wrap(inner))
}

/** Area de una escena -> estilo de miniatura. */
export const thumbDeArea = (area) => (['red', 'ntn'].includes(area) ? 'red' : ['seguidor', 'control', 'hardware', 'pnt'].includes(area) ? 'seguidor' : area === 'adcs' ? 'adcs' : area === 'mision' ? 'mision' : area === 'orbita' ? 'plan' : area === 'rf' ? 'rf' : 'grafica')
