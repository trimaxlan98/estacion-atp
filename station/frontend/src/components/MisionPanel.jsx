import { Fragment, useEffect, useMemo, useState } from 'react'
import { getContacto, getContactoTelemetry } from '../api.js'
import HBars from '../charts/HBars.jsx'
import { C, satColor } from '../colors.js'
import { nf } from '../format.js'
import { TimeCharts } from './RunCharts.jsx'
import { Panel, Tile } from './ui.jsx'

const shortSat = (n) => n.replace(/\s*\(.*\)\s*$/, '')
const pad = (n) => String(n).padStart(2, '0')
const utc = (iso) => {
  const d = new Date(iso)
  return `${pad(d.getUTCMonth() + 1)}-${pad(d.getUTCDate())} ${pad(d.getUTCHours())}:${pad(d.getUTCMinutes())}`
}

/** Curvas de UN contacto de la mision: es una corrida en lockstep con el formato del lazo. */
function Contacto({ runId, c }) {
  const [tel, setTel] = useState(null)
  const [met, setMet] = useState(null)
  useEffect(() => {
    let dead = false
    const name = c.carpeta.split('/').pop()
    Promise.all([getContactoTelemetry(runId, name, 4), getContacto(runId, name)])
      .then(([t, m]) => { if (!dead) { setTel(Array.isArray(t) ? t : []); setMet(m) } })
      .catch(() => !dead && setTel([]))
    return () => { dead = true }
  }, [runId, c.carpeta])
  if (!tel) return <p className="muted">Cargando la corrida del contacto…</p>
  if (!tel.length) return <p className="muted">Sin telemetría de este contacto.</p>
  return (
    <div className="stack" data-testid="mision-contacto">
      <p className="muted small">
        Corrida <code>{c.carpeta}</code>: {met?.params?.profile === 'od' ? 'efemérides = órbita estimada' : 'efemérides = TLE'}
        {' · '}{c.seguimiento}{c.sdr ? ' + SDR' : ''} · apuntamiento rms {nf(c.apunt_rms_deg, 4)}° · máx {nf(c.apunt_max_deg, 4)}°
      </p>
      <TimeCharts samples={tel} objective={0.1} />
    </div>
  )
}

/**
 * Informe de la misión (informe.json, CONTRATO 4.2): totales planificado contra ejecutado,
 * GB por satélite, determinación de órbita, mejora por la OD, tabla de contactos ejecutados
 * (cada fila abre la corrida de ese contacto) e incidencias.
 */
export default function MisionPanel({ runId, informe }) {
  const [sat, setSat] = useState('')
  const [abierto, setAbierto] = useState(null)
  const t = informe.totales
  const j = informe.mejora_od
  const sats = informe.mision.satelites
  const filas = useMemo(() => informe.contactos.filter((c) => !sat || c.sat === sat), [informe, sat])
  const pct = t.gb_sin_perdida_apunt ? (100 * t.perdida_apunt_gb) / t.gb_sin_perdida_apunt : 0

  return (
    <div className="stack" data-testid="mision-panel">
      <section className="panel">
        <header className="panel__head">
          <div>
            <h3 className="panel__title">Resumen de la misión</h3>
            <p className="panel__sub">
              {informe.mision.horas} h desde {utc(informe.mision.inicio_utc)} UTC · {informe.mision.estaciones.length} estaciones · {sats.length} satélites · plan {informe.mision.plan_algoritmo}
            </p>
          </div>
        </header>
        <div className="tiles" data-testid="mision-tiles">
          <Tile label="contactos" value={`${t.contactos_ejecutados} / ${t.contactos_planificados}`} tone={t.contactos_fallidos ? 'amber' : 'green'} hint="ejecutados / planificados" />
          <Tile label="minutos" value={nf(t.minutos, 0)} unit={` de ${nf(t.minutos_plan, 0)}`} tone="cyan" />
          <Tile label="GB ejecutados" value={nf(t.gb, 1)} unit={` de ${nf(t.gb_plan, 1)}`} tone="cyan" hint="Con el C/N0 y el apuntamiento medidos en cada corrida" />
          <Tile label="pérdida por apunt." value={nf(t.perdida_apunt_gb, 2)} unit=" GB" tone="amber" hint={`${nf(pct, 2)} % de lo que habría bajado con apuntamiento perfecto`} />
          <Tile label="con órbita estimada" value={j.contactos_con_od} tone="violet" hint="Contactos volados con la órbita de la OD en vez del TLE" />
          <Tile label="mejora por la OD" value={j.delta_gb != null ? `${j.delta_gb >= 0 ? '+' : ''}${nf(j.delta_gb, 2)}` : '—'} unit=" GB" tone={j.delta_gb > 0 ? 'green' : 'gray'} hint="GB con OD menos GB del mismo contacto con TLE (gemelos)" />
          <Tile label="incidencias" value={informe.incidencias.length} tone={informe.incidencias.length ? 'amber' : 'green'} />
          <Tile label="totales" value={informe.verificacion.cuadra_gb && informe.verificacion.cuadra_minutos ? 'CUADRAN' : 'NO CUADRAN'} tone={informe.verificacion.cuadra_gb && informe.verificacion.cuadra_minutos ? 'green' : 'red'} hint={`Releídos desde la telemetría de ${informe.verificacion.corridas_releidas} corridas`} />
        </div>
      </section>

      <div className="grid2">
        <Panel title="GB por satélite" sub="planificado (gris) contra ejecutado (cian)">
          <HBars ariaLabel="GB por satélite" labelA="ejecutado" labelB="planificado" colorA={C.cyan} colorB={C.gray}
            rows={sats.map((s) => ({ label: shortSat(s), a: informe.por_satelite[s]?.gb ?? 0, b: informe.por_satelite[s]?.gb_plan ?? 0 }))} />
        </Panel>
        <Panel title="Mejora por la órbita estimada" sub={`${j.contactos_con_gemelo} contactos con gemelo (el mismo contacto con TLE)`}>
          {j.contactos_con_gemelo === 0 ? <p className="muted">Sin gemelos: no hubo contactos con órbita estimada y seguimiento por program track.</p> : (
            <table className="table table--compact">
              <thead><tr><th /><th className="num">con OD</th><th className="num">con TLE</th></tr></thead>
              <tbody>
                <tr><td>GB</td><td className="num">{nf(j.gb_con_od, 2)}</td><td className="num">{nf(j.gb_con_tle, 2)}</td></tr>
                <tr><td>apuntamiento rms (°, mediana)</td><td className="num">{nf(j.apunt_rms_od_mediana_deg, 4)}</td><td className="num">{nf(j.apunt_rms_tle_mediana_deg, 4)}</td></tr>
                <tr><td>apuntamiento máx (°, mediana)</td><td className="num">{nf(j.apunt_max_od_mediana_deg, 4)}</td><td className="num">{nf(j.apunt_max_tle_mediana_deg, 4)}</td></tr>
                <tr><td>error de efemérides (°, mediana)</td><td className="num">{nf(j.err_efemeride_max_od_mediana_deg, 4)}</td><td className="num">{nf(j.err_efemeride_max_tle_mediana_deg, 4)}</td></tr>
              </tbody>
            </table>
          )}
        </Panel>
      </div>

      {informe.od.length > 0 && (
        <Panel title="Determinación de órbita" sub="Con lo que registraron los contactos de calibración (autotrack enganchado)">
          <div className="tablewrap">
            <table className="table table--compact" data-testid="mision-od">
              <thead><tr><th>Satélite</th><th>Contactos</th><th className="num">Medidas</th><th>Canales</th><th className="num">σ pos (km)</th><th className="num">χ²/dof</th><th className="num">Error de posición (km)</th><th className="num">TLE (km)</th><th>Veredicto</th></tr></thead>
              <tbody>
                {informe.od.map((o) => (
                  <tr key={o.sat + o.contactos.join('-')}>
                    <td>{shortSat(o.sat)}</td><td>{o.contactos.join(', ')}</td><td className="num">{o.n_medidas}</td><td>{o.canales.join('+')}</td>
                    <td className="num">{nf(o.sigma_pos_km, 3)}</td><td className="num">{nf(o.chi2_dof, 2)}</td>
                    <td className="num">{nf(o.error_pos_epoca_km, 3)}</td><td className="num">{nf(o.error_pos_tle_epoca_km, 3)}</td>
                    <td className={o.aceptada ? 'tone-green' : 'tone-red'}>{o.aceptada ? 'aceptada' : `rechazada: ${o.motivo}`}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Panel>
      )}

      <Panel
        title="Contactos ejecutados"
        sub={`${filas.length} contactos · cada fila abre la corrida de ese contacto (error de apuntamiento, Doppler y margen)`}
        right={
          <select value={sat} onChange={(e) => setSat(e.target.value)} aria-label="Satélite">
            <option value="">todos los satélites</option>
            {sats.map((s) => <option key={s} value={s}>{shortSat(s)}</option>)}
          </select>
        }
      >
        <div className="tablewrap tablewrap--scroll">
          <table className="table table--compact" data-testid="mision-contactos">
            <thead><tr><th>#</th><th>Satélite</th><th>Est.</th><th>AOS (UTC)</th><th className="num">El</th><th>Banda</th><th>Rol</th><th>Seguim.</th><th>Efem.</th><th className="num">GB plan</th><th className="num">GB</th><th className="num">rms (°)</th><th className="num">máx (°)</th><th>Estado</th></tr></thead>
            <tbody>
              {filas.map((c) => (
                <Fragment key={c.id}>
                  <tr className={c.carpeta ? 'row--link' : ''} onClick={() => c.carpeta && c.gb != null && setAbierto(abierto === c.id ? null : c.id)}>
                    <td>{c.id}</td>
                    <td><i className="dot" style={{ background: satColor(Math.max(0, sats.indexOf(c.sat))) }} />{shortSat(c.sat)}</td>
                    <td>{c.estacion}</td><td className="nowrap">{utc(c.aos)}</td><td className="num">{nf(c.el_max_deg, 1)}</td>
                    <td>{c.banda}</td><td className={c.rol === 'cal' ? 'tone-violet' : 'muted'}>{c.rol === 'cal' ? 'calibración' : 'normal'}</td>
                    <td>{c.seguimiento}{c.sdr ? ' + SDR' : ''}</td><td className={c.efemeride === 'od' ? 'tone-violet' : ''}>{c.efemeride === 'od' ? 'órbita estimada' : c.efemeride ?? '—'}</td>
                    <td className="num">{nf(c.gb_plan, 2)}</td><td className="num">{nf(c.gb, 2)}</td>
                    <td className="num">{nf(c.apunt_rms_deg, 3)}</td><td className="num">{nf(c.apunt_max_deg, 3)}</td>
                    <td className={c.estado === 'ok' ? 'tone-green' : 'tone-red'}>{c.estado}</td>
                  </tr>
                  {abierto === c.id && (
                    <tr><td colSpan={14}><Contacto runId={runId} c={c} /></td></tr>
                  )}
                </Fragment>
              ))}
            </tbody>
          </table>
        </div>
      </Panel>

      <details className="panel log">
        <summary><span className="panel__title">Incidencias</span><span className="muted">{informe.incidencias.length}</span></summary>
        <div className="tablewrap tablewrap--scroll">
          <table className="table table--compact" data-testid="mision-incidencias">
            <thead><tr><th>Contacto</th><th>Satélite</th><th>Tipo</th><th>Detalle</th></tr></thead>
            <tbody>
              {informe.incidencias.length === 0 && <tr><td colSpan={4} className="muted">Ninguna.</td></tr>}
              {informe.incidencias.map((i, k) => <tr key={k}><td>{i.contacto}</td><td>{shortSat(i.sat)}</td><td>{i.tipo}</td><td>{i.detalle}</td></tr>)}
            </tbody>
          </table>
        </div>
      </details>
    </div>
  )
}
