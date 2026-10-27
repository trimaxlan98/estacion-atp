import { useEffect, useState } from 'react'
import { getExperiments, getRun, getTelemetry, listRuns } from '../api.js'
import AdcsPanel from '../components/AdcsPanel.jsx'
import MisionPanel from '../components/MisionPanel.jsx'
import PlanPanel from '../components/PlanPanel.jsx'
import { StatusBadge } from '../components/ui.jsx'
import { fmtDate, isActive, nf } from '../format.js'
import { go, href } from '../router.js'

// Una vista por kind de corrida: lista de las corridas guardadas (chips) y la ultima terminada abierta.
const CONF = {
  plan: {
    path: '/plan',
    eyebrow: 'Red',
    title: 'Plan de contactos',
    intro: 'Qué contactos hacer, con qué antena y cuántos GB bajan: Gantt por estación, codicioso contra óptimo y lo que aporta cada estación.',
    cta: 'Calcular un plan',
    empty: 'Lanza el experimento de plan de contactos desde el catálogo (nivel S6).',
    chip: (r) => `${r.args?.horas ?? '?'} h · sin ${r.args?.sin || 'ninguna'}${r.metrics?.gb != null ? ` · ${nf(r.metrics.gb, 0)} GB` : ''}`,
  },
  mision: {
    path: '/mision',
    eyebrow: 'Misión integrada',
    title: 'Misión',
    intro: 'Un día de operaciones de la red: plan, una corrida en lockstep por contacto con el pase real, autotrack, receptor SDR, órbita estimada y los GB con lo medido.',
    cta: 'Lanzar una misión',
    empty: 'Lanza el experimento de la misión desde el catálogo (nivel M1).',
    chip: (r) => `${r.args?.horas ?? '?'} h · OD ${r.args?.n_od ?? '?'}${r.metrics?.gb != null ? ` · ${nf(r.metrics.gb, 0)} GB` : ''}`,
  },
  adcs: {
    path: '/adcs',
    eyebrow: 'Segmento espacio',
    title: 'ADCS',
    intro: 'Actitud de un CubeSat 3U en un pase real: detumbling, apuntamiento a nadir y a la estación, y la pérdida de enlace que cuesta.',
    cta: 'Lanzar un escenario',
    empty: 'Lanza el experimento ADCS desde el catálogo (nivel E1).',
    chip: (r) => `${r.args?.w0_deg_s ?? '?'} °/s · grado ${r.args?.model_degree ?? '?'}${r.metrics?.verdict ? ` · ${r.metrics.verdict}` : ''}`,
  },
}

export default function Resultados({ kind, runId }) {
  const conf = CONF[kind]
  const [list, setList] = useState(null)
  const [error, setError] = useState(null)
  const [detail, setDetail] = useState(null)
  const [tel, setTel] = useState([])

  useEffect(() => {
    let dead = false
    Promise.all([listRuns({ limit: 300 }), getExperiments()])
      .then(([rs, ex]) => {
        if (dead) return
        const ids = new Set(ex.filter((e) => e.kind === kind).map((e) => e.id))
        setList(rs.filter((r) => ids.has(r.experiment_id)))
      })
      .catch((e) => !dead && setError(e))
    return () => { dead = true }
  }, [kind])

  // por defecto: la corrida terminada mas reciente
  const current = runId || list?.find((r) => r.status === 'done')?.run_id || list?.[0]?.run_id

  useEffect(() => {
    if (!current) return undefined
    let dead = false
    setDetail(null)
    setTel([])
    getRun(current)
      .then(async (d) => {
        if (dead) return
        setDetail(d)
        if (kind === 'adcs') {
          const n = d.metrics?.n
          const t = await getTelemetry(current, n > 6000 ? Math.ceil(n / 6000) : 1).catch(() => [])
          if (!dead) setTel(Array.isArray(t) ? t : [])
        }
      })
      .catch(() => !dead && setDetail({ error: true }))
    return () => { dead = true }
  }, [current, kind])

  const hasResult = kind === 'plan' ? !!detail?.plan : kind === 'mision' ? !!detail?.mision : !!detail?.metrics

  return (
    <div className="page">
      <header className="pagehead pagehead--row">
        <div>
          <p className="eyebrow">{conf.eyebrow}</p>
          <h1>{conf.title}</h1>
          <p className="muted">{conf.intro}</p>
        </div>
        <a className="btn btn--primary" href={href('/')}>{conf.cta}</a>
      </header>
      {error && <div className="alert">No se pudieron cargar las corridas: {error.message}</div>}
      {!list && !error && <p className="muted">Cargando…</p>}
      {list && list.length === 0 && (
        <div className="empty"><h2>Todavía no hay corridas</h2><p className="muted">{conf.empty}</p></div>
      )}
      {list && list.length > 0 && (
        <>
          <div className="chips" role="tablist" aria-label="Corridas guardadas">
            {list.map((r) => (
              <button key={r.run_id} role="tab" aria-selected={r.run_id === current} className={`chip ${r.run_id === current ? 'is-on' : ''}`} onClick={() => go(`${conf.path}/${r.run_id}`)}>
                <span>{conf.chip(r)}</span>
                <small>{fmtDate(r.created_at)}</small>
              </button>
            ))}
          </div>
          {detail?.error && <div className="alert">No se pudo cargar la corrida.</div>}
          {detail && !detail.error && (
            <>
              <p className="muted small">
                <code>{detail.run_id}</code> <StatusBadge status={detail.status} />{' '}
                <a href={href(`/runs/${detail.run_id}`)}>abrir la corrida</a>
              </p>
              {hasResult ? (
                kind === 'plan' ? <PlanPanel key={detail.run_id} plan={detail.plan} /> : kind === 'mision' ? <MisionPanel key={detail.run_id} runId={detail.run_id} informe={detail.mision} /> : <AdcsPanel key={detail.run_id} metrics={detail.metrics} samples={tel} />
              ) : (
                <div className="panel"><p className="muted">{isActive(detail.status, detail.finished_at) ? 'Esta corrida sigue en curso.' : 'Esta corrida no produjo resultados.'}</p></div>
              )}
            </>
          )}
          {!detail && <p className="muted">Cargando corrida…</p>}
        </>
      )}
    </div>
  )
}
