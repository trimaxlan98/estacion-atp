import { useId, useMemo, useState } from 'react'
import { C } from '../colors.js'
import { bisect, decimate, fmtTick, niceTicks, pathOf, useWidth } from './util.js'

const M = { l: 50, r: 12, t: 10, b: 36 }

/**
 * Grafica de lineas en SVG con submuestreo (min/max por cubeta) y hover.
 *
 * series: [{name, x[], y[], color, dash?, width?, digits?, unit?,
 *           limit?: {y, bad:'above'|'below', color}}]
 *   - `limit` pinta el trazo en `color` (rojo por defecto) en el lado "malo"
 *     de la linea y en el color de la serie en el lado bueno.
 * bands:  [{y0, y1, color, opacity?}]  (banda SIEMPRE dibujada, haya o no datos)
 * hlines: [{y, color, dash?, label?}]
 * xBands: [{x0, x1, color, opacity?, label?}]  (bandas verticales, p. ej. los modos de mision)
 */
export default function Chart({
  series,
  bands = [],
  xBands = [],
  hlines = [],
  yInclude = [],
  yMin,
  yMax,
  xDomain,
  height = 190,
  xLabel = 't (s)',
  yLabel,
  yDigits,
  title,
  desc,
  maxPts = 1400,
  emptyText = 'Esperando datos…',
}) {
  const [wrapRef, width] = useWidth(640)
  const uid = useId().replace(/:/g, '')
  const [hover, setHover] = useState(null)

  const pw = Math.max(40, width - M.l - M.r)
  const ph = Math.max(40, height - M.t - M.b)

  const dom = useMemo(() => {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity
    for (const s of series) {
      const n = s.x.length
      if (!n) continue
      if (s.x[0] < x0) x0 = s.x[0]
      if (s.x[n - 1] > x1) x1 = s.x[n - 1]
      for (let i = 0; i < n; i++) {
        const v = s.y[i]
        if (v < y0) y0 = v
        if (v > y1) y1 = v
      }
    }
    if (xDomain) [x0, x1] = xDomain
    if (!isFinite(x0)) { x0 = 0; x1 = 1 }
    if (x1 <= x0) x1 = x0 + 1
    for (const v of yInclude) { if (v < y0) y0 = v; if (v > y1) y1 = v }
    for (const b of bands) { y0 = Math.min(y0, b.y0); y1 = Math.max(y1, b.y1) }
    for (const h of hlines) { y0 = Math.min(y0, h.y); y1 = Math.max(y1, h.y) }
    if (!isFinite(y0)) { y0 = 0; y1 = 1 }
    if (y1 <= y0) { y1 = y0 + 1 }
    const pad = (y1 - y0) * 0.08
    let lo = y0 - pad, hi = y1 + pad
    if (yMin != null) lo = yMin
    if (yMax != null) hi = yMax
    return { x0, x1, y0: lo, y1: hi }
    // bands/hlines/yInclude llegan como literales nuevos en cada render: se comparan por contenido
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [series, JSON.stringify([bands, hlines, yInclude, xDomain]), yMin, yMax])

  const sx = (v) => M.l + ((v - dom.x0) / (dom.x1 - dom.x0)) * pw
  const sy = (v) => M.t + ph - ((v - dom.y0) / (dom.y1 - dom.y0)) * ph

  const paths = useMemo(
    () =>
      series.map((s) => {
        const d = decimate(s.x, s.y, maxPts)
        return pathOf(d.x, d.y, sx, sy)
      }),
    // sx/sy dependen solo de dom y de las dimensiones
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [series, dom.x0, dom.x1, dom.y0, dom.y1, pw, ph, maxPts],
  )

  const xt = niceTicks(dom.x0, dom.x1, Math.max(3, Math.floor(pw / 90)))
  const yt = niceTicks(dom.y0, dom.y1, Math.max(3, Math.floor(ph / 38)))
  const ystep = yt.length > 1 ? yt[1] - yt[0] : 1
  const xstep = xt.length > 1 ? xt[1] - xt[0] : 1
  const hasData = series.some((s) => s.x.length > 0)

  function onMove(e) {
    const rect = e.currentTarget.getBoundingClientRect()
    const px = e.clientX - rect.left
    if (px < M.l || px > M.l + pw) return setHover(null)
    setHover(px)
  }

  let tip = null
  if (hover != null && hasData) {
    const xv = dom.x0 + ((hover - M.l) / pw) * (dom.x1 - dom.x0)
    const rows = []
    for (const s of series) {
      if (!s.x.length) continue
      const i = bisect(s.x, xv)
      rows.push({ s, i, x: s.x[i], y: s.y[i] })
    }
    if (rows.length) {
      const left = hover > width * 0.62
      tip = (
        <>
          <line x1={sx(rows[0].x)} x2={sx(rows[0].x)} y1={M.t} y2={M.t + ph} stroke={C.gray} strokeWidth="1" strokeDasharray="2 3" opacity="0.7" />
          {rows.map((r, k) => (
            <circle key={k} cx={sx(r.x)} cy={sy(r.y)} r="3.5" fill={r.s.color} stroke="#070b12" strokeWidth="1.5" />
          ))}
          <foreignObject x={left ? hover - 168 : hover + 10} y={M.t + 2} width="160" height={22 + rows.length * 17} style={{ pointerEvents: 'none' }}>
            <div className="chart-tip">
              <div className="chart-tip__t">t = {rows[0].x.toFixed(1)} s</div>
              {rows.map((r, k) => (
                <div key={k} className="chart-tip__row">
                  <i style={{ background: r.s.color }} />
                  <span>{r.s.name}</span>
                  <b>
                    {r.y.toFixed(r.s.digits ?? yDigits ?? 3)}
                    {r.s.unit ? ` ${r.s.unit}` : ''}
                  </b>
                </div>
              ))}
            </div>
          </foreignObject>
        </>
      )
    }
  }

  return (
    <div className="chart" ref={wrapRef}>
      <svg
        width={width}
        height={height}
        role="img"
        aria-label={title}
        onMouseMove={onMove}
        onMouseLeave={() => setHover(null)}
      >
        {title && <title>{title}</title>}
        {desc && <desc>{desc}</desc>}
        <defs>
          {series.map((s, k) =>
            s.limit ? (
              <g key={k}>
                <clipPath id={`${uid}-ok${k}`}>
                  <rect
                    x={M.l}
                    width={pw}
                    y={s.limit.bad === 'above' ? Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y))) : M.t}
                    height={
                      s.limit.bad === 'above'
                        ? Math.max(0, M.t + ph - Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y))))
                        : Math.max(0, Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y))) - M.t)
                    }
                  />
                </clipPath>
                <clipPath id={`${uid}-bad${k}`}>
                  <rect
                    x={M.l}
                    width={pw}
                    y={s.limit.bad === 'above' ? M.t : Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y)))}
                    height={
                      s.limit.bad === 'above'
                        ? Math.max(0, Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y))) - M.t)
                        : Math.max(0, M.t + ph - Math.min(M.t + ph, Math.max(M.t, sy(s.limit.y))))
                    }
                  />
                </clipPath>
              </g>
            ) : null,
          )}
          <clipPath id={`${uid}-plot`}>
            <rect x={M.l} y={M.t} width={pw} height={ph} />
          </clipPath>
        </defs>

        {/* rejilla y ejes: mobiliario */}
        {yt.map((v) => (
          <g key={'y' + v}>
            <line x1={M.l} x2={M.l + pw} y1={sy(v)} y2={sy(v)} stroke={C.furniture} strokeWidth="1" opacity="0.5" />
            <text x={M.l - 8} y={sy(v) + 4} textAnchor="end" className="chart-tick">{fmtTick(v, ystep)}</text>
          </g>
        ))}
        {xt.map((v) => (
          <g key={'x' + v}>
            <line x1={sx(v)} x2={sx(v)} y1={M.t} y2={M.t + ph} stroke={C.furniture} strokeWidth="1" opacity="0.25" />
            <text x={sx(v)} y={M.t + ph + 15} textAnchor="middle" className="chart-tick">{fmtTick(v, xstep)}</text>
          </g>
        ))}
        <line x1={M.l} x2={M.l + pw} y1={M.t + ph} y2={M.t + ph} stroke={C.furniture} />
        <line x1={M.l} x2={M.l} y1={M.t} y2={M.t + ph} stroke={C.furniture} />

        <g clipPath={`url(#${uid}-plot)`}>
          {xBands.map((b, k) => (
            <rect key={'xb' + k} x={sx(b.x0)} width={Math.max(0, sx(b.x1) - sx(b.x0))} y={M.t} height={ph} fill={b.color} opacity={b.opacity ?? 0.1}>
              {b.label && <title>{b.label}</title>}
            </rect>
          ))}
          {bands.map((b, k) => (
            <rect
              key={k}
              x={M.l}
              width={pw}
              y={sy(b.y1)}
              height={Math.max(0, sy(b.y0) - sy(b.y1))}
              fill={b.color}
              opacity={b.opacity ?? 0.12}
            />
          ))}
          {series.map((s, k) =>
            s.limit ? (
              <g key={k}>
                <path d={paths[k]} fill="none" stroke={s.color} strokeWidth={s.width ?? 1.6} strokeLinejoin="round" clipPath={`url(#${uid}-ok${k})`} strokeDasharray={s.dash} />
                <path d={paths[k]} fill="none" stroke={s.limit.color ?? C.red} strokeWidth={(s.width ?? 1.6) + 0.4} strokeLinejoin="round" clipPath={`url(#${uid}-bad${k})`} strokeDasharray={s.dash} />
              </g>
            ) : (
              <path key={k} d={paths[k]} fill="none" stroke={s.color} strokeWidth={s.width ?? 1.6} strokeLinejoin="round" strokeDasharray={s.dash} />
            ),
          )}
        </g>
        {hlines.map((h, k) => (
          <g key={k}>
            <line x1={M.l} x2={M.l + pw} y1={sy(h.y)} y2={sy(h.y)} stroke={h.color} strokeWidth="1.3" strokeDasharray={h.dash ?? '5 4'} />
            {h.label && (
              <text x={M.l + pw - 4} y={sy(h.y) - 5} textAnchor="end" className="chart-hlabel" fill={h.color}>{h.label}</text>
            )}
          </g>
        ))}

        {yLabel && (
          <text x={12} y={M.t + ph / 2} transform={`rotate(-90 12 ${M.t + ph / 2})`} textAnchor="middle" className="chart-axis">{yLabel}</text>
        )}
        <text x={M.l + pw} y={height - 4} textAnchor="end" className="chart-axis">{xLabel}</text>
        {!hasData && (
          <text x={M.l + pw / 2} y={M.t + ph / 2} textAnchor="middle" className="chart-empty">{emptyText}</text>
        )}
        {tip}
      </svg>
    </div>
  )
}
