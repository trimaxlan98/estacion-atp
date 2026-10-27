import { useMemo, useState } from 'react'
import Gantt from '../charts/Gantt.jsx'
import HBars from '../charts/HBars.jsx'
import { C, satColor } from '../colors.js'
import { nf } from '../format.js'
import { Panel, Tile } from './ui.jsx'

export const PLAN_LABEL = {
  optimo: 'Óptimo',
  codicioso_peso: 'Codicioso (por peso)',
  codicioso_densidad: 'Codicioso (por densidad)',
  codicioso_aos: 'Codicioso (por llegada)',
}
const shortSat = (n) => n.replace(/\s*\(.*\)\s*$/, '')
const pad = (n) => String(n).padStart(2, '0')
const utc = (iso) => {
  const d = new Date(iso)
  return `${pad(d.getUTCMonth() + 1)}-${pad(d.getUTCDate())} ${pad(d.getUTCHours())}:${pad(d.getUTCMinutes())}`
}

/** Totales de un conjunto de ventanas (los de un plan `sin`, que plan.json solo guarda como ids). */
function resumir(vs) {
  return {
    contactos: vs.length,
    minutos: vs.reduce((a, v) => a + v.duracion_s, 0) / 60,
    gb: vs.reduce((a, v) => a + v.vol_gb, 0),
  }
}

/**
 * Plan de contactos de la red (plan.json, CONTRATO §1 y §3.2): resumen, Gantt por estacion,
 * codicioso contra optimo, aporte de una estacion (con/sin) y tabla de contactos.
 */
export default function PlanPanel({ plan }) {
  const [sel, setSel] = useState('optimo')
  const [showDisc, setShowDisc] = useState(true)
  const [activeSat, setActiveSat] = useState(null)

  const sats = useMemo(() => plan.satelites.map((s) => s.nombre), [plan])
  const estKeys = useMemo(() => plan.estaciones_lista || Object.keys(plan.estaciones), [plan])
  const nombres = useMemo(() => Object.fromEntries(Object.entries(plan.estaciones).map(([k, e]) => [k, e.nombre])), [plan])
  const byId = useMemo(() => Object.fromEntries(plan.ventanas.map((v) => [v.id, v])), [plan])
  const t0 = Date.parse(plan.inicio_utc)
  const t1 = t0 + plan.horas * 3600e3

  // cada opcion: clave, etiqueta y los ids de contacto de ese plan
  const options = useMemo(() => {
    const o = Object.entries(plan.planes).map(([k, p]) => ({ key: k, label: PLAN_LABEL[k] || k, ids: p.ids, res: p.resumen, probado: p.optimo_probado, algoritmo: p.algoritmo }))
    for (const [est, a] of Object.entries(plan.aporte_estacion || {})) {
      if (!a.ids_sin) continue
      const r = resumir(a.ids_sin.map((i) => byId[i]))
      o.push({ key: 'sin:' + est, label: `Óptimo sin ${nombres[est] ?? est}`, ids: a.ids_sin, res: { ...r, gb_shannon: null }, probado: a.optimo_probado_sin, algoritmo: 'optimo sin ' + est })
    }
    return o
  }, [plan, byId, nombres])
  const cur = options.find((o) => o.key === sel) || options[0]
  const ids = useMemo(() => new Set(cur.ids), [cur])
  const contactos = useMemo(() => cur.ids.map((i) => byId[i]).sort((a, b) => a.aos.localeCompare(b.aos)), [cur, byId])

  const opt = plan.planes.optimo || plan.planes.codicioso_peso
  const optGb = opt.resumen.gb
  const gapOf = (gb) => (optGb > 0 ? (100 * (optGb - gb)) / optGb : 0)
  const bySat = useMemo(() => {
    const m = {}
    for (const v of contactos) m[v.sat] = (m[v.sat] || 0) + v.vol_gb
    return m
  }, [contactos])
  const byEst = useMemo(() => {
    const m = {}
    for (const v of contactos) m[v.estacion] = (m[v.estacion] || 0) + v.vol_gb
    return m
  }, [contactos])

  const shannon = cur.res.gb_shannon
  const brecha = plan.brecha_codicioso_pct
  const aportes = Object.entries(plan.aporte_estacion || {})

  return (
    <div className="stack" data-testid="plan-panel">
      <section className="panel">
        <header className="panel__head">
          <div>
            <h3 className="panel__title">Resumen del plan</h3>
            <p className="panel__sub">
              {plan.horas} h desde {utc(plan.inicio_utc)} UTC · {estKeys.length} estaciones · {sats.length} satélites · banda {plan.banda} · {plan.n_ventanas} ventanas candidatas
            </p>
          </div>
        </header>
        <div className="tiles" data-testid="plan-tiles">
          <Tile label="contactos" value={opt.resumen.contactos} tone="cyan" hint={`Plan ${PLAN_LABEL.optimo.toLowerCase()}: contactos elegidos de ${plan.n_ventanas} ventanas`} />
          <Tile label="minutos" value={nf(opt.resumen.minutos, 1)} unit=" min" tone="cyan" />
          <Tile label="datos bajados" value={nf(optGb, 1)} unit=" GB" tone="cyan" hint="Integral de la tasa ACM DVB-S2 sobre cada contacto" />
          <Tile label="cota Shannon" value={nf((100 * optGb) / opt.resumen.gb_shannon, 0)} unit=" %" tone="gray" hint="GB del plan / GB que permitiría la capacidad de Shannon con la misma tasa de símbolos" />
          <Tile label="brecha codicioso" value={brecha != null ? nf(brecha, 2) : '—'} unit=" %" tone="amber" hint="Lo que pierde el codicioso por peso frente al óptimo, en GB" />
          <Tile label="óptimo" value={opt.optimo_probado ? 'PROBADO' : 'COTA'} tone={opt.optimo_probado ? 'green' : 'amber'} hint={`${opt.algoritmo}. Si no está probado, cota_sup acota lo que podría haber`} />
        </div>
      </section>

      <Panel
        title="Plan de contactos por estación"
        sub={`${cur.label}: ${cur.res.contactos} contactos · ${nf(cur.res.gb, 1)} GB · ${nf(cur.res.minutos, 0)} min · en gris, los pases que el plan descarta`}
        right={
          <div className="gantt__ctl">
            <select value={cur.key} onChange={(e) => setSel(e.target.value)} aria-label="Plan que se dibuja">
              {options.map((o) => <option key={o.key} value={o.key}>{o.label} ({nf(o.res.gb, 1)} GB)</option>)}
            </select>
            <label className="check"><input type="checkbox" checked={showDisc} onChange={(e) => setShowDisc(e.target.checked)} /> descartados</label>
          </div>
        }
      >
        <div className="satlegend" role="group" aria-label="Satélites (clic para aislar uno)">
          {sats.map((s, i) => (
            <button type="button" key={s} className={`satchip ${activeSat === s ? 'is-on' : ''}`} aria-pressed={activeSat === s} onClick={() => setActiveSat(activeSat === s ? null : s)}>
              <i style={{ background: satColor(i) }} />
              {shortSat(s)}
              <small>{nf(bySat[s] || 0, 1)} GB</small>
            </button>
          ))}
          <span className="satchip satchip--key"><i style={{ background: 'transparent', border: `2px solid ${C.red}` }} />pasa por el keyhole</span>
        </div>
        <Gantt ventanas={plan.ventanas} ids={ids} estaciones={estKeys} nombres={nombres} sats={sats} t0={t0} t1={t1} showDiscarded={showDisc} activeSat={activeSat}
          ariaLabel={`Plan de contactos ${cur.label}: ${cur.res.contactos} contactos en ${estKeys.length} estaciones`} />
      </Panel>

      <div className="grid2">
        <Panel title="Codicioso contra óptimo" sub="GB bajados por plan; la brecha se mide contra el óptimo">
          <HBars
            ariaLabel="GB por plan"
            rows={[
              { label: 'Todas las ventanas', a: plan.disponible.gb, color: C.violet },
              ...Object.entries(plan.planes).map(([k, p]) => ({ label: PLAN_LABEL[k] || k, a: p.resumen.gb, color: k === 'optimo' ? C.cyan : C.gray })),
            ]}
          />
          <div className="tablewrap tablewrap--flush">
            <table className="table table--compact" data-testid="plan-algos">
              <thead><tr><th>Plan</th><th className="num">Contactos</th><th className="num">Minutos</th><th className="num">GB</th><th className="num">Brecha</th></tr></thead>
              <tbody>
                <tr><td className="muted">Todas las ventanas</td><td className="num">{plan.disponible.contactos}</td><td className="num">{nf(plan.disponible.minutos, 0)}</td><td className="num">{nf(plan.disponible.gb, 1)}</td><td className="num muted">—</td></tr>
                {Object.entries(plan.planes).map(([k, p]) => (
                  <tr key={k}>
                    <td>{PLAN_LABEL[k] || k}{k === 'optimo' && <span className="muted small"> · {p.optimo_probado ? 'probado' : 'cota'}</span>}</td>
                    <td className="num">{p.resumen.contactos}</td><td className="num">{nf(p.resumen.minutos, 0)}</td><td className="num">{nf(p.resumen.gb, 1)}</td>
                    <td className={`num ${k === 'optimo' ? 'tone-cyan' : 'tone-amber'}`}>{k === 'optimo' ? '—' : `${nf(gapOf(p.resumen.gb), 2)} %`}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Panel>
        <Panel title={`GB por satélite y por estación`} sub={cur.label}>
          <HBars ariaLabel="GB por satélite" rows={sats.map((s, i) => ({ label: shortSat(s), a: bySat[s] || 0, color: satColor(i) }))} />
          <hr className="sep" />
          <HBars ariaLabel="GB por estación" rows={estKeys.map((e) => ({ label: nombres[e] ?? e, a: byEst[e] || 0, color: C.cyan }))} />
        </Panel>
      </div>

      <Panel
        title="Con y sin una estación"
        sub="El mismo problema repetido quitando una estación: lo que esa antena aporta a la red"
      >
        {aportes.length === 0 && (
          <p className="muted">Este plan no incluye la comparación. Lanza el experimento con «Repetir el plan sin la estación» para medir el aporte de una antena.</p>
        )}
        {aportes.map(([est, a]) => {
          const sinSat = a.por_satelite_sin || {}
          return (
            <div key={est} className="aporte" data-testid={`aporte-${est}`}>
              <div className="tiles">
                <Tile label={`con ${nombres[est] ?? est}`} value={nf(a.gb_con, 1)} unit=" GB" tone="cyan" hint={`${a.contactos_con} contactos · ${nf(a.minutos_con, 0)} min`} />
                <Tile label={`sin ${nombres[est] ?? est}`} value={nf(a.gb_sin, 1)} unit=" GB" tone="gray" hint={`${a.contactos_sin} contactos · ${nf(a.minutos_sin, 0)} min`} />
                <Tile label="aporte" value={`${a.delta_gb >= 0 ? '+' : ''}${nf(a.delta_gb, 1)}`} unit=" GB" tone={a.delta_gb > 0 ? 'green' : 'gray'} />
                <Tile label="aporte relativo" value={`${a.delta_pct >= 0 ? '+' : ''}${nf(a.delta_pct, 1)}`} unit=" %" tone={a.delta_pct > 0 ? 'green' : 'gray'} />
                <Tile label="contactos" value={`${a.contactos_con} / ${a.contactos_sin}`} tone="cyan" hint="con / sin la estación" />
                <Tile label="brecha sin ella" value={a.brecha_codicioso_sin_pct != null ? nf(a.brecha_codicioso_sin_pct, 2) : '—'} unit=" %" tone="amber" hint={`Con la estación: ${nf(brecha, 2)} %. Con ventanas densas y solapadas escoger mal bloquea dos buenas`} />
              </div>
              <p className="muted small">
                GB por satélite, con (cian) y sin (gris) la estación.{' '}
                {a.ids_sin && <button type="button" className="btn btn--sm" onClick={() => setSel('sin:' + est)}>Ver el plan sin {nombres[est] ?? est} en el Gantt</button>}
              </p>
              <HBars
                ariaLabel={`GB por satélite con y sin ${est}`}
                labelA="con" labelB="sin" colorA={C.cyan} colorB={C.gray}
                rows={sats.map((s) => ({ label: shortSat(s), a: opt.resumen.por_satelite?.[s]?.gb ?? 0, b: sinSat[s]?.gb ?? 0 }))}
              />
            </div>
          )
        })}
      </Panel>

      <details className="panel log">
        <summary>
          <span className="panel__title">Contactos del plan</span>
          <span className="muted">{contactos.length} contactos · {cur.label}</span>
        </summary>
        <div className="tablewrap tablewrap--scroll">
          <table className="table table--compact" data-testid="plan-contactos">
            <thead><tr><th>Estación</th><th>Satélite</th><th>AOS (UTC)</th><th className="num">Min</th><th className="num">GB</th><th className="num">El máx (°)</th><th>MODCOD máx</th><th>Keyhole</th></tr></thead>
            <tbody>
              {contactos.map((v) => (
                <tr key={v.id}>
                  <td>{nombres[v.estacion] ?? v.estacion}</td>
                  <td><i className="dot" style={{ background: satColor(Math.max(0, sats.indexOf(v.sat))) }} />{shortSat(v.sat)}</td>
                  <td className="nowrap">{utc(v.aos)}</td>
                  <td className="num">{nf(v.duracion_s / 60, 1)}</td>
                  <td className="num">{nf(v.vol_gb, 2)}</td>
                  <td className="num">{nf(v.el_max_deg, 1)}</td>
                  <td>{v.modcod_max}</td>
                  <td className={v.keyhole ? 'tone-red' : 'muted'}>{v.keyhole ? 'sí' : '—'}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </details>
    </div>
  )
}
