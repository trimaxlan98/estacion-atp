import { useMemo } from 'react'
import { C } from '../colors.js'

const D2R = Math.PI / 180

/**
 * Cielo polar: acimut alrededor (N arriba, sentido horario), elevacion radial
 * (90 deg en el centro). Referencia en ambar, antena en cian; los tramos donde
 * el error supera el objetivo se pintan en rojo.
 */
export default function PolarSky({ samples, mask = 5, objective = 0.1 }) {
  const S = 360, cx = S / 2, cy = S / 2, R = 150
  const pt = (az, el) => {
    const r = ((90 - el) / 90) * R
    return [cx + r * Math.sin(az * D2R), cy - r * Math.cos(az * D2R)]
  }

  const geo = useMemo(() => {
    const n = samples.length
    const stride = Math.max(1, Math.ceil(n / 1500))
    let ref = '', ant = ''
    for (let i = 0; i < n; i += stride) {
      const s = samples[i]
      if (s.ref_az != null && s.ref_el != null) {
        const [x, y] = pt(s.ref_az, s.ref_el)
        ref += (ref ? 'L' : 'M') + x.toFixed(1) + ' ' + y.toFixed(1)
      }
      if (s.az != null && s.el != null) {
        const [x, y] = pt(s.az, s.el)
        ant += (ant ? 'L' : 'M') + x.toFixed(1) + ' ' + y.toFixed(1)
      }
    }
    // tramos fuera de objetivo a resolucion completa (un pico de 1 muestra no se pierde)
    const bad = []
    let cur = null
    for (let i = 0; i < n; i++) {
      const s = samples[i]
      if (s.az != null && s.el != null && s.e_pt != null && s.e_pt > objective) {
        const [x, y] = pt(s.az, s.el)
        if (!cur) { cur = []; bad.push(cur) }
        cur.push(x.toFixed(1) + ' ' + y.toFixed(1))
      } else cur = null
    }
    const last = samples[n - 1]
    return { ref, ant, bad, last }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [samples, objective])

  const { last } = geo
  const rings = [0, 30, 60]
  const maskR = ((90 - mask) / 90) * R

  return (
    <svg viewBox={`0 0 ${S} ${S}`} className="polar" role="img" aria-label="Cielo polar: traza de referencia y posicion real de la antena">
      <defs>
        <radialGradient id="skyg" cx="50%" cy="50%" r="50%">
          <stop offset="0%" stopColor={C.violet} stopOpacity="0.20" />
          <stop offset="100%" stopColor={C.violet} stopOpacity="0.05" />
        </radialGradient>
        <pattern id="maskhatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
          <line x1="0" y1="0" x2="0" y2="6" stroke={C.red} strokeWidth="2" opacity="0.35" />
        </pattern>
        <clipPath id="skyclip"><circle cx={cx} cy={cy} r={R} /></clipPath>
      </defs>
      {/* cielo (violeta: cielo/predicho) */}
      <circle cx={cx} cy={cy} r={R} fill="url(#skyg)" stroke={C.furniture} strokeWidth="1.5" />
      {/* mascara: anillo entre el horizonte y la elevacion de mascara */}
      <path
        d={`M${cx - R} ${cy}a${R} ${R} 0 1 0 ${2 * R} 0a${R} ${R} 0 1 0 ${-2 * R} 0ZM${cx - maskR} ${cy}a${maskR} ${maskR} 0 1 1 ${2 * maskR} 0a${maskR} ${maskR} 0 1 1 ${-2 * maskR} 0Z`}
        fill="url(#maskhatch)"
        fillRule="evenodd"
      />
      <circle cx={cx} cy={cy} r={maskR} fill="none" stroke={C.red} strokeWidth="1" strokeDasharray="4 3" opacity="0.7" />
      {rings.slice(1).map((el) => (
        <g key={el}>
          <circle cx={cx} cy={cy} r={((90 - el) / 90) * R} fill="none" stroke={C.furniture} strokeWidth="1" opacity="0.8" />
          <text x={cx + 3} y={cy - ((90 - el) / 90) * R + 11} className="chart-tick">{el}°</text>
        </g>
      ))}
      {[0, 45, 90, 135, 180, 225, 270, 315].map((az) => {
        const [x1, y1] = pt(az, 90)
        const [x2, y2] = pt(az, 0)
        return <line key={az} x1={x1} y1={y1} x2={x2} y2={y2} stroke={C.furniture} strokeWidth="1" opacity={az % 90 === 0 ? 0.8 : 0.35} />
      })}
      {[['N', 0], ['E', 90], ['S', 180], ['O', 270]].map(([t, az]) => {
        const [x, y] = pt(az, -9)
        return <text key={t} x={x} y={y + 4} textAnchor="middle" className="polar-card">{t}</text>
      })}
      <text x={cx + maskR * 0.71 + 2} y={cy + maskR * 0.71 + 13} className="polar-mask" fill={C.red}>máscara {mask}°</text>

      <g clipPath="url(#skyclip)">
        {/* referencia (ambar) */}
        {geo.ref && <path d={geo.ref} fill="none" stroke={C.amber} strokeWidth="2" strokeLinejoin="round" opacity="0.9" />}
        {/* antena (cian) */}
        {geo.ant && <path d={geo.ant} fill="none" stroke={C.cyan} strokeWidth="1.3" strokeLinejoin="round" />}
        {geo.bad.map((g, i) => (
          <polyline key={i} points={g.length === 1 ? `${g[0]} ${g[0]}` : g.join(' ')} fill="none" stroke={C.red} strokeWidth="2.6" strokeLinecap="round" />
        ))}
      </g>

      {last && last.ref_az != null && (() => {
        const [rx, ry] = pt(last.ref_az, last.ref_el)
        const [ax, ay] = last.az != null ? pt(last.az, last.el) : [rx, ry]
        return (
          <g>
            <circle cx={rx} cy={ry} r="4.5" fill={C.amber} />
            <circle cx={ax} cy={ay} r="8.5" fill="none" stroke={C.cyan} strokeWidth="2" />
            <line x1={ax - 12} x2={ax + 12} y1={ay} y2={ay} stroke={C.cyan} strokeWidth="1" opacity="0.8" />
            <line x1={ax} x2={ax} y1={ay - 12} y2={ay + 12} stroke={C.cyan} strokeWidth="1" opacity="0.8" />
          </g>
        )
      })()}
      {!last && <text x={cx} y={cy} textAnchor="middle" className="chart-empty">Esperando muestras…</text>}
    </svg>
  )
}
