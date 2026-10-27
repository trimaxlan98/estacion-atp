import { C } from '../colors.js'
import { nf } from '../format.js'

/**
 * Barras horizontales HTML/CSS (sin dependencias). `rows`: [{label, a, b?, color?}].
 * Con `b` son pares (p. ej. con/sin una estacion): `a` en `colorA` y `b` en `colorB`.
 * Todas comparten la misma escala (el maximo de todos los valores).
 */
export default function HBars({ rows, unit = 'GB', digits = 1, colorA = C.cyan, colorB = C.gray, labelA, labelB, ariaLabel }) {
  const max = Math.max(1e-9, ...rows.flatMap((r) => [r.a ?? 0, r.b ?? 0]))
  return (
    <div className="hbars" role="img" aria-label={ariaLabel}>
      {rows.map((r) => (
        <div className="hbars__row" key={r.label}>
          <span className="hbars__label" title={r.label}>{r.label}</span>
          <div className="hbars__tracks">
            <div className="hbars__track" title={labelA ? `${labelA}: ${nf(r.a, digits)} ${unit}` : undefined}>
              <i style={{ width: `${(100 * (r.a ?? 0)) / max}%`, background: r.color ?? colorA }} />
              <b>{nf(r.a, digits)}<small> {unit}</small></b>
            </div>
            {r.b != null && (
              <div className="hbars__track" title={labelB ? `${labelB}: ${nf(r.b, digits)} ${unit}` : undefined}>
                <i style={{ width: `${(100 * r.b) / max}%`, background: colorB }} />
                <b>{nf(r.b, digits)}<small> {unit}</small></b>
              </div>
            )}
          </div>
        </div>
      ))}
    </div>
  )
}
