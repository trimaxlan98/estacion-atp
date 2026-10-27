import { useMemo, useState } from 'react'
import { C } from '../colors.js'
import { niceTicks, useWidth } from './util.js'

const M = { l: 50, r: 14, t: 14, b: 40 }

function logTicks(lo, hi) {
  // 1-2-5 por decada entre lo y hi
  const out = []
  for (let e = Math.floor(Math.log10(lo)); e <= Math.ceil(Math.log10(hi)); e++) {
    for (const m of [1, 2, 5]) {
      const v = m * Math.pow(10, e)
      if (v >= lo && v <= hi) out.push({ v, major: m === 1 })
    }
  }
  return out
}
const fmtLog = (v) => (v >= 1 ? String(v) : v.toFixed(Math.min(4, Math.ceil(-Math.log10(v)))))

/** Histograma del RMS por corrida (eje x logaritmico: el RMS tiene cola larga). */
export function RmsHistogram({ values, objective = 0.1, p95, bins = 36, height = 280 }) {
  const [ref, width] = useWidth(560)
  const pw = Math.max(60, width - M.l - M.r)
  const ph = height - M.t - M.b
  const h = useMemo(() => {
    const pos = values.filter((v) => v > 0)
    if (!pos.length) return null
    const lo = Math.min(Math.min(...pos), objective) / 1.25
    const hi = Math.max(Math.max(...pos), objective, p95 ?? 0) * 1.25
    const l0 = Math.log10(lo), l1 = Math.log10(hi)
    const counts = new Array(bins).fill(0)
    for (const v of pos) {
      const b = Math.min(bins - 1, Math.floor(((Math.log10(v) - l0) / (l1 - l0)) * bins))
      counts[b]++
    }
    return { lo, hi, l0, l1, counts, max: Math.max(...counts) }
  }, [values, objective, p95, bins])
  const [tip, setTip] = useState(null)

  if (!h) return <div className="chart" ref={ref}><div className="chart-empty-box">Sin datos</div></div>
  const sx = (v) => M.l + ((Math.log10(v) - h.l0) / (h.l1 - h.l0)) * pw
  const ymax = h.max * 1.12
  const sy = (c) => M.t + ph - (c / ymax) * ph
  const yt = niceTicks(0, ymax, 4)
  const bw = pw / bins
  const edge = (b) => Math.pow(10, h.l0 + (b / bins) * (h.l1 - h.l0))

  return (
    <div className="chart" ref={ref}>
      <svg width={width} height={height} role="img" aria-label="Histograma del RMS de apuntamiento por corrida">
        {yt.map((v) => (
          <g key={v}>
            <line x1={M.l} x2={M.l + pw} y1={sy(v)} y2={sy(v)} stroke={C.furniture} opacity="0.5" />
            <text x={M.l - 8} y={sy(v) + 4} textAnchor="end" className="chart-tick">{v}</text>
          </g>
        ))}
        {logTicks(h.lo, h.hi).map(({ v, major }) => (
          <g key={v}>
            <line x1={sx(v)} x2={sx(v)} y1={M.t} y2={M.t + ph} stroke={C.furniture} opacity={major ? 0.45 : 0.18} />
            <text x={sx(v)} y={M.t + ph + 15} textAnchor="middle" className="chart-tick">{fmtLog(v)}</text>
          </g>
        ))}
        {h.counts.map((c, b) => {
          const x0 = M.l + b * bw
          const bad = edge(b) >= objective
          return (
            <rect
              key={b}
              x={x0 + 0.5}
              width={Math.max(1, bw - 1)}
              y={sy(c)}
              height={Math.max(0, M.t + ph - sy(c))}
              fill={bad ? C.red : C.cyan}
              opacity={tip === b ? 1 : 0.72}
              onMouseEnter={() => setTip(b)}
              onMouseLeave={() => setTip(null)}
            />
          )
        })}
        <line x1={M.l} x2={M.l + pw} y1={M.t + ph} y2={M.t + ph} stroke={C.furniture} />
        <line x1={M.l} x2={M.l} y1={M.t} y2={M.t + ph} stroke={C.furniture} />
        {/* umbral del objetivo: cian */}
        <line x1={sx(objective)} x2={sx(objective)} y1={M.t} y2={M.t + ph} stroke={C.cyan} strokeWidth="2" strokeDasharray="6 4" />
        <text x={sx(objective) - 5} y={M.t + 11} textAnchor="end" className="chart-hlabel" fill={C.cyan}>objetivo {objective}°</text>
        {/* p95: ambar */}
        {p95 != null && (
          <>
            <line x1={sx(p95)} x2={sx(p95)} y1={M.t} y2={M.t + ph} stroke={C.amber} strokeWidth="2" />
            <text x={sx(p95) + 5} y={M.t + 11} className="chart-hlabel" fill={C.amber}>p95 {p95.toFixed(3)}°</text>
          </>
        )}
        <text x={M.l + pw / 2} y={height - 6} textAnchor="middle" className="chart-axis">RMS de apuntamiento por corrida (deg, escala log)</text>
        <text x={12} y={M.t + ph / 2} transform={`rotate(-90 12 ${M.t + ph / 2})`} textAnchor="middle" className="chart-axis">corridas</text>
        {tip != null && (
          <foreignObject x={Math.min(width - 190, M.l + tip * bw)} y={M.t + 22} width="180" height="40" style={{ pointerEvents: 'none' }}>
            <div className="chart-tip">
              <div className="chart-tip__t">{fmtLog(+edge(tip).toPrecision(2))} – {fmtLog(+edge(tip + 1).toPrecision(2))}°</div>
              <div className="chart-tip__row"><span>corridas</span><b>{h.counts[tip]}</b></div>
            </div>
          </foreignObject>
        )}
      </svg>
    </div>
  )
}

/** Dispersion RMS vs elevacion maxima: ahi se ve el keyhole. */
export function RmsScatter({ rms, elMax, objective = 0.1, height = 280 }) {
  const [ref, width] = useWidth(560)
  const pw = Math.max(60, width - M.l - M.r)
  const ph = height - M.t - M.b
  const [hov, setHov] = useState(null)
  const n = Math.min(rms.length, elMax.length)

  const dom = useMemo(() => {
    const pos = rms.filter((v) => v > 0)
    if (!pos.length) return null
    const lo = Math.min(Math.min(...pos), objective) / 1.4
    const hi = Math.max(Math.max(...pos), objective) * 1.4
    return { lo, hi, l0: Math.log10(lo), l1: Math.log10(hi) }
  }, [rms, objective])
  if (!dom || !n) return <div className="chart" ref={ref}><div className="chart-empty-box">Sin datos</div></div>

  const sx = (v) => M.l + (v / 90) * pw
  const sy = (v) => M.t + ph - ((Math.log10(Math.max(v, dom.lo)) - dom.l0) / (dom.l1 - dom.l0)) * ph
  const xt = [0, 15, 30, 45, 60, 75, 90]

  function onMove(e) {
    const r = e.currentTarget.getBoundingClientRect()
    const mx = e.clientX - r.left, my = e.clientY - r.top
    let best = -1, bd = 14 * 14
    for (let i = 0; i < n; i++) {
      const dx = sx(elMax[i]) - mx, dy = sy(rms[i]) - my
      const d = dx * dx + dy * dy
      if (d < bd) { bd = d; best = i }
    }
    setHov(best >= 0 ? best : null)
  }

  return (
    <div className="chart" ref={ref}>
      <svg width={width} height={height} role="img" aria-label="Dispersion del RMS contra la elevacion maxima del pase" onMouseMove={onMove} onMouseLeave={() => setHov(null)}>
        {logTicks(dom.lo, dom.hi).map(({ v, major }) => (
          <g key={v}>
            <line x1={M.l} x2={M.l + pw} y1={sy(v)} y2={sy(v)} stroke={C.furniture} opacity={major ? 0.45 : 0.18} />
            <text x={M.l - 8} y={sy(v) + 4} textAnchor="end" className="chart-tick">{fmtLog(v)}</text>
          </g>
        ))}
        {xt.map((v) => (
          <g key={v}>
            <line x1={sx(v)} x2={sx(v)} y1={M.t} y2={M.t + ph} stroke={C.furniture} opacity="0.25" />
            <text x={sx(v)} y={M.t + ph + 15} textAnchor="middle" className="chart-tick">{v}°</text>
          </g>
        ))}
        <line x1={M.l} x2={M.l + pw} y1={M.t + ph} y2={M.t + ph} stroke={C.furniture} />
        <line x1={M.l} x2={M.l} y1={M.t} y2={M.t + ph} stroke={C.furniture} />
        {/* region dentro de presupuesto: cian tenue, siempre dibujada */}
        <rect x={M.l} width={pw} y={sy(objective)} height={Math.max(0, M.t + ph - sy(objective))} fill={C.cyan} opacity="0.08" />
        <line x1={M.l} x2={M.l + pw} y1={sy(objective)} y2={sy(objective)} stroke={C.cyan} strokeWidth="2" strokeDasharray="6 4" />
        <text x={M.l + 6} y={sy(objective) - 6} className="chart-hlabel" fill={C.cyan}>objetivo {objective}°</text>
        {Array.from({ length: n }, (_, i) => (
          <circle key={i} cx={sx(elMax[i])} cy={sy(rms[i])} r={hov === i ? 5 : 2.8} fill={rms[i] > objective ? C.red : C.cyan} opacity={hov === i ? 1 : 0.66} />
        ))}
        <text x={M.l + pw / 2} y={height - 6} textAnchor="middle" className="chart-axis">elevación máxima del pase (deg)</text>
        <text x={12} y={M.t + ph / 2} transform={`rotate(-90 12 ${M.t + ph / 2})`} textAnchor="middle" className="chart-axis">RMS (deg, log)</text>
        {hov != null && (
          <foreignObject x={Math.min(width - 190, Math.max(M.l, sx(elMax[hov]) + 10))} y={Math.max(M.t, sy(rms[hov]) - 20)} width="180" height="52" style={{ pointerEvents: 'none' }}>
            <div className="chart-tip">
              <div className="chart-tip__row"><span>pase #{hov + 1}</span></div>
              <div className="chart-tip__row"><span>el máx</span><b>{elMax[hov].toFixed(1)}°</b></div>
              <div className="chart-tip__row"><span>RMS</span><b>{rms[hov].toFixed(4)}°</b></div>
            </div>
          </foreignObject>
        )}
      </svg>
    </div>
  )
}
