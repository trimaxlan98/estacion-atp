import { useMemo, useState } from 'react'
import { C, satColor } from '../colors.js'
import { useWidth } from './util.js'

const M = { l: 132, r: 14, t: 26, b: 6 }
const ROW = 46
const BAR_Y = 7
const BAR_H = 24
const DISC_Y = 34
const DISC_H = 5

const pad = (n) => String(n).padStart(2, '0')
const hhmm = (ms) => {
  const d = new Date(ms)
  return `${pad(d.getUTCHours())}:${pad(d.getUTCMinutes())}`
}
const hhmmss = (ms) => `${hhmm(ms)}:${pad(new Date(ms).getUTCSeconds())}`
const dayLabel = (ms) => {
  const d = new Date(ms)
  return `${pad(d.getUTCMonth() + 1)}-${pad(d.getUTCDate())}`
}

/**
 * Gantt de contactos: una fila por estacion, una barra por contacto coloreada por satelite.
 *
 * - `ventanas`: las de plan.json (aos, los ISO; sat, estacion, vol_gb, el_max_deg, keyhole...).
 * - `ids`: Set con los contactos del plan que se dibuja; el resto de ventanas son "descartadas"
 *   (franja gris debajo de la fila, si showDiscarded).
 * - `sats`: nombres de satelite en orden (define el color).
 * - Zoom: la ventana visible (todo / 12 h / 6 h / 3 h) y un deslizador para moverla.
 *
 * Una barra de 10 min en 24 h mide ~7 px: con el zoom a 3-6 h caben el GB y la elevacion maxima
 * dentro de la barra; el tooltip da siempre el detalle completo.
 */
export default function Gantt({ ventanas, ids, estaciones, nombres, sats, t0, t1, showDiscarded = true, activeSat = null, ariaLabel }) {
  const [wrapRef, width] = useWidth(900)
  const [spanH, setSpanH] = useState(0) // 0 = todo
  const [startFrac, setStartFrac] = useState(0)
  const [tip, setTip] = useState(null)

  const rows = useMemo(
    () =>
      ventanas.map((v) => ({
        v,
        a: Date.parse(v.aos),
        b: Date.parse(v.los),
        plan: ids.has(v.id),
        si: Math.max(0, sats.indexOf(v.sat)),
      })),
    [ventanas, ids, sats],
  )
  const totalMs = Math.max(1, t1 - t0)
  const spanMs = spanH ? Math.min(totalMs, spanH * 3600e3) : totalMs
  const x0 = t0 + (totalMs - spanMs) * startFrac
  const x1 = x0 + spanMs

  const pw = Math.max(80, width - M.l - M.r)
  const height = M.t + estaciones.length * ROW + M.b
  const sx = (ms) => M.l + ((ms - x0) / spanMs) * pw

  const ticks = useMemo(() => {
    const h = spanMs / 3600e3
    const step = (h <= 4 ? 0.5 : h <= 8 ? 1 : h <= 14 ? 2 : h <= 30 ? 3 : 6) * 3600e3
    const out = []
    for (let t = Math.ceil(x0 / step) * step; t <= x1 + 1; t += step) out.push(t)
    return out
  }, [x0, x1, spanMs])

  const visible = rows.filter((r) => r.b >= x0 && r.a <= x1)
  const dim = (r) => activeSat != null && r.v.sat !== activeSat

  function onMove(e, r) {
    const box = wrapRef.current.getBoundingClientRect()
    setTip({ r, x: e.clientX - box.left, y: e.clientY - box.top })
  }

  return (
    <div className="gantt" ref={wrapRef}>
      <div className="gantt__bar">
        <div className="seg" role="radiogroup" aria-label="Ventana de tiempo">
          {[[0, 'todo'], [12, '12 h'], [6, '6 h'], [3, '3 h']].map(([h, lab]) => (
            <button type="button" key={h} role="radio" aria-checked={spanH === h} className={spanH === h ? 'is-on' : ''} onClick={() => setSpanH(h)}>
              {lab}
            </button>
          ))}
        </div>
        {spanH > 0 && spanMs < totalMs && (
          <label className="gantt__pan">
            <span className="muted small">desde {hhmm(x0)} UTC</span>
            <input type="range" min="0" max="1" step="0.005" value={startFrac} onChange={(e) => setStartFrac(Number(e.target.value))} aria-label="Desplazar la ventana de tiempo" />
          </label>
        )}
      </div>
      <svg width={width} height={height} role="img" aria-label={ariaLabel} onMouseLeave={() => setTip(null)}>
        <title>{ariaLabel}</title>
        <defs>
          <clipPath id="gantt-clip">
            <rect x={M.l} y={M.t - 4} width={pw} height={estaciones.length * ROW + 4} />
          </clipPath>
        </defs>
        {ticks.map((t) => (
          <g key={t}>
            <line x1={sx(t)} x2={sx(t)} y1={M.t - 4} y2={height - M.b} stroke={C.furniture} strokeWidth="1" opacity="0.4" />
            <text x={sx(t)} y={M.t - 10} textAnchor="middle" className="chart-tick">{new Date(t).getUTCHours() === 0 && new Date(t).getUTCMinutes() === 0 ? dayLabel(t) + ' ' : ''}{hhmm(t)}</text>
          </g>
        ))}
        {estaciones.map((est, i) => {
          const y = M.t + i * ROW
          return (
            <g key={est} className="gantt-row" data-station={est}>
              <rect x={M.l} y={y} width={pw} height={ROW - 4} fill={i % 2 ? 'transparent' : '#ffffff'} opacity="0.025" />
              <line x1={M.l} x2={M.l + pw} y1={y + ROW - 2} y2={y + ROW - 2} stroke={C.furniture} opacity="0.35" />
              <text x={M.l - 10} y={y + 22} textAnchor="end" className="gantt-label">{nombres[est] ?? est}</text>
              <text x={M.l - 10} y={y + 36} textAnchor="end" className="chart-tick">{est}</text>
            </g>
          )
        })}
        <g clipPath="url(#gantt-clip)">
          {showDiscarded &&
            visible.filter((r) => !r.plan).map((r) => {
              const i = estaciones.indexOf(r.v.estacion)
              if (i < 0) return null
              return (
                <rect key={'d' + r.v.id} className="gantt-disc" x={sx(r.a)} y={M.t + i * ROW + DISC_Y} width={Math.max(1.5, sx(r.b) - sx(r.a))} height={DISC_H} rx="1.5"
                  fill={C.gray} opacity={dim(r) ? 0.12 : 0.45} onMouseMove={(e) => onMove(e, r)} />
              )
            })}
          {visible.filter((r) => r.plan).map((r) => {
            const i = estaciones.indexOf(r.v.estacion)
            if (i < 0) return null
            const bx = sx(r.a)
            const bw = Math.max(2, sx(r.b) - sx(r.a))
            const y = M.t + i * ROW + BAR_Y
            const lost = (r.v.keyhole || r.v.t_perdido_s > 0)
            const label = bw >= 86 ? `${r.v.vol_gb.toFixed(1)} GB · ${r.v.el_max_deg.toFixed(0)}°` : bw >= 40 ? `${r.v.vol_gb.toFixed(1)} GB` : bw >= 24 ? `${r.v.el_max_deg.toFixed(0)}°` : ''
            return (
              <g key={r.v.id} opacity={dim(r) ? 0.18 : 1} onMouseMove={(e) => onMove(e, r)}>
                <rect className="gantt-bar" data-sat={r.v.sat} data-station={r.v.estacion} data-gb={r.v.vol_gb} x={bx} y={y} width={bw} height={BAR_H} rx="3"
                  fill={satColor(r.si)} stroke={lost ? C.red : '#070b12'} strokeWidth={lost ? 2 : 0.8} />
                {label && <text x={bx + bw / 2} y={y + 16} textAnchor="middle" className="gantt-bartext">{label}</text>}
              </g>
            )
          })}
        </g>
        <line x1={M.l} x2={M.l} y1={M.t - 4} y2={height - M.b} stroke={C.furniture} />
      </svg>
      {tip && (
        <div className="gantt-tip" style={{ left: Math.min(tip.x + 14, Math.max(0, width - 250)), top: tip.y + 18 }} role="tooltip">
          <b style={{ color: satColor(tip.r.si) }}>{tip.r.v.sat}</b> en {nombres[tip.r.v.estacion] ?? tip.r.v.estacion}
          <div className="muted">{hhmmss(tip.r.a)} → {hhmmss(tip.r.b)} UTC · {((tip.r.b - tip.r.a) / 60000).toFixed(1)} min</div>
          <div><b>{tip.r.v.vol_gb.toFixed(2)} GB</b> · elevación máx {tip.r.v.el_max_deg.toFixed(1)}° · {tip.r.v.modcod_max}</div>
          {tip.r.v.keyhole && <div style={{ color: C.red }}>pasa por el keyhole{tip.r.v.t_perdido_s > 0 ? ` (${tip.r.v.t_perdido_s.toFixed(0)} s sin seguimiento)` : ''}</div>}
          <div className="muted">{tip.r.plan ? 'en el plan' : 'descartado por el plan'}</div>
        </div>
      )}
    </div>
  )
}
