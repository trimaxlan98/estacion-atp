import { useMemo } from 'react'
import { C, MODE_COLOR, MODE_NAMES } from '../colors.js'
import { niceTicks, useWidth, fmtTick } from './util.js'

// mismos margenes izquierdo/derecho que Chart.jsx: la tira queda alineada con las graficas
const ML = 50
const MR = 12

/** Tramos [{x0, x1}] de las muestras consecutivas donde `pred` es cierta. */
export function runs(samples, pred) {
  const out = []
  let cur = null
  for (const s of samples) {
    if (pred(s)) {
      if (!cur) { cur = { x0: s.t, x1: s.t }; out.push(cur) } else cur.x1 = s.t
    } else cur = null
  }
  return out
}

/** Tramos de modo de la telemetria (cuando aun no hay metrics.json): [{mode, name, t0, t1}]. */
export function modesFrom(samples) {
  const out = []
  for (const s of samples) {
    if (s.mode == null) continue
    const last = out[out.length - 1]
    if (last && last.mode === s.mode) last.t1 = s.t
    else out.push({ mode: s.mode, name: MODE_NAMES[s.mode], t0: s.t, t1: s.t })
  }
  return out
}

/**
 * Tira de modos de mision (DETUMBLE / NADIR / TRACK_GS), eclipse y pase sobre el eje del tiempo.
 * `modes`: [{name, t0, t1}] (metrics.modes o modesFrom). El tramo i se dibuja hasta el inicio del
 * siguiente, para que no queden huecos entre muestras.
 */
export default function ModeStrip({ modes, samples, xDomain }) {
  const [ref, width] = useWidth(900)
  const x0 = xDomain?.[0] ?? modes[0]?.t0 ?? 0
  const x1 = xDomain?.[1] ?? modes[modes.length - 1]?.t1 ?? 1
  const pw = Math.max(40, width - ML - MR)
  const sx = (v) => ML + ((v - x0) / Math.max(1e-9, x1 - x0)) * pw
  const ecl = useMemo(() => runs(samples, (s) => s.eclipse), [samples])
  const pase = useMemo(() => runs(samples, (s) => s.in_pass), [samples])
  const ticks = niceTicks(x0, x1, Math.max(3, Math.floor(pw / 90)))
  const step = ticks.length > 1 ? ticks[1] - ticks[0] : 1
  const H = 72
  return (
    <div className="chart modestrip" ref={ref}>
      <svg width={width} height={H} role="img" aria-label="Modos de misión, eclipse y pase sobre el tiempo">
        <title>Modos de misión</title>
        {modes.map((m, i) => {
          const end = i + 1 < modes.length ? modes[i + 1].t0 : Math.max(m.t1, x1)
          const w = Math.max(1, sx(end) - sx(m.t0))
          return (
            <g key={i} className="mode-seg" data-mode={m.name}>
              <rect x={sx(m.t0)} y={4} width={w} height={26} fill={MODE_COLOR[m.name]} opacity="0.32" stroke={MODE_COLOR[m.name]} strokeWidth="1" rx="3" />
              {w > 62 && <text x={sx(m.t0) + w / 2} y={21} textAnchor="middle" className="mode-label" fill={MODE_COLOR[m.name]}>{m.name}</text>}
              <title>{m.name}: t = {m.t0.toFixed(0)}–{m.t1.toFixed(0)} s</title>
            </g>
          )
        })}
        {ecl.map((e, i) => (
          <rect key={'e' + i} x={sx(e.x0)} y={34} width={Math.max(1, sx(e.x1) - sx(e.x0))} height={8} fill="#0b121c" stroke={C.gray} strokeWidth="1" rx="2"><title>eclipse</title></rect>
        ))}
        {pase.map((e, i) => (
          <rect key={'p' + i} x={sx(e.x0)} y={46} width={Math.max(1, sx(e.x1) - sx(e.x0))} height={5} fill={C.cyan} rx="2"><title>pase visible (sobre la máscara de la estación)</title></rect>
        ))}
        <text x={ML - 8} y={41} textAnchor="end" className="chart-tick">eclipse</text>
        <text x={ML - 8} y={54} textAnchor="end" className="chart-tick">pase</text>
        <line x1={ML} x2={ML + pw} y1={H - 12} y2={H - 12} stroke={C.furniture} />
        {ticks.map((v) => (
          <g key={v}>
            <line x1={sx(v)} x2={sx(v)} y1={H - 12} y2={H - 8} stroke={C.furniture} />
            <text x={sx(v)} y={H - 2} textAnchor="middle" className="chart-tick">{fmtTick(v, step)}</text>
          </g>
        ))}
      </svg>
    </div>
  )
}
