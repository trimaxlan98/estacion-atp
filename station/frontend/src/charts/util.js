import { useEffect, useRef, useState } from 'react'

/** Ancho del contenedor (ResizeObserver). */
export function useWidth(initial = 600) {
  const ref = useRef(null)
  const [w, setW] = useState(initial)
  useEffect(() => {
    const el = ref.current
    if (!el) return
    const ro = new ResizeObserver((e) => {
      const nw = Math.floor(e[0].contentRect.width)
      if (nw > 0) setW(nw)
    })
    ro.observe(el)
    setW(Math.max(1, Math.floor(el.clientWidth)) || initial)
    return () => ro.disconnect()
  }, [initial])
  return [ref, w]
}

/** Marcas "bonitas" (1-2-5) entre lo y hi. */
export function niceTicks(lo, hi, count = 5) {
  if (!(hi > lo)) return [lo]
  const span = hi - lo
  const raw = span / Math.max(1, count)
  const mag = Math.pow(10, Math.floor(Math.log10(raw)))
  const norm = raw / mag
  const step = (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag
  const out = []
  const start = Math.ceil(lo / step - 1e-9) * step
  for (let v = start; v <= hi + step * 1e-6; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v)
  return out
}

export function fmtTick(v, step) {
  const d = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)))
  return v.toFixed(d)
}

/**
 * Submuestreo min/max por cubeta: conserva los picos (un pico de keyhole de
 * 1 muestra NO se pierde) y limita los puntos dibujados a ~maxPts.
 * x debe ser creciente. Devuelve {x, y} nuevos.
 */
export function decimate(x, y, maxPts = 1200) {
  const n = x.length
  if (n <= maxPts) return { x, y }
  const buckets = Math.floor(maxPts / 2)
  const size = n / buckets
  const ox = [x[0]], oy = [y[0]]
  for (let b = 0; b < buckets; b++) {
    const a = Math.floor(b * size)
    const e = Math.min(n, Math.floor((b + 1) * size))
    if (e <= a) continue
    let imin = a, imax = a
    for (let i = a + 1; i < e; i++) {
      if (y[i] < y[imin]) imin = i
      if (y[i] > y[imax]) imax = i
    }
    const first = imin < imax ? imin : imax
    const second = imin < imax ? imax : imin
    ox.push(x[first]); oy.push(y[first])
    if (second !== first) { ox.push(x[second]); oy.push(y[second]) }
  }
  ox.push(x[n - 1]); oy.push(y[n - 1])
  return { x: ox, y: oy }
}

/** Extrae dos columnas saltando muestras donde falta el campo (el contrato omite campos sin dato). */
export function pick(samples, kx, ky, fy) {
  const x = [], y = []
  for (let i = 0; i < samples.length; i++) {
    const s = samples[i]
    const vx = s[kx], vy = s[ky]
    if (vx == null || vy == null) continue
    x.push(vx)
    y.push(fy ? fy(vy) : vy)
  }
  return { x, y }
}

/** Indice del mayor x[i] <= v (busqueda binaria). */
export function bisect(x, v) {
  let lo = 0, hi = x.length - 1
  if (hi < 0) return -1
  if (v <= x[0]) return 0
  if (v >= x[hi]) return hi
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1
    if (x[m] <= v) lo = m
    else hi = m
  }
  return v - x[lo] < x[hi] - v ? lo : hi
}

export const pathOf = (xs, ys, sx, sy) => {
  let d = ''
  for (let i = 0; i < xs.length; i++) d += (i ? 'L' : 'M') + sx(xs[i]).toFixed(1) + ' ' + sy(ys[i]).toFixed(1)
  return d
}

/** Envolvente: maximo por ventana de k muestras (conserva los picos; x = punto medio). */
export function envelope(x, y, k) {
  if (k <= 1 || x.length <= k) return { x, y }
  const ox = [], oy = []
  for (let a = 0; a < x.length; a += k) {
    const e = Math.min(x.length, a + k)
    let m = -Infinity
    for (let i = a; i < e; i++) if (y[i] > m) m = y[i]
    ox.push(x[(a + e - 1) >> 1])
    oy.push(m)
  }
  return { x: ox, y: oy }
}
