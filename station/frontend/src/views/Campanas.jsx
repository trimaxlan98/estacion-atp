import { useEffect, useState } from 'react'
import { getExperiments, getRun, listRuns } from '../api.js'
import CampaignPanel from '../components/CampaignPanel.jsx'
import { StatusBadge } from '../components/ui.jsx'
import { fmtDate, isActive } from '../format.js'
import { go, href } from '../router.js'

export default function Campanas({ runId }) {
  const [list, setList] = useState(null)
  const [error, setError] = useState(null)
  const [detail, setDetail] = useState(null)

  useEffect(() => {
    let dead = false
    Promise.all([listRuns({ limit: 300 }), getExperiments()])
      .then(([rs, ex]) => {
        if (dead) return
        const camp = new Set(ex.filter((e) => e.kind === 'campaign').map((e) => e.id))
        setList(rs.filter((r) => camp.has(r.experiment_id)))
      })
      .catch((e) => !dead && setError(e))
    return () => { dead = true }
  }, [])

  // por defecto: la campana terminada mas reciente
  const current = runId || list?.find((r) => r.status === 'done')?.run_id || list?.[0]?.run_id

  useEffect(() => {
    if (!current) return
    let dead = false
    setDetail(null)
    getRun(current).then((d) => !dead && setDetail(d)).catch(() => !dead && setDetail({ error: true }))
    return () => { dead = true }
  }, [current])

  return (
    <div className="page">
      <header className="pagehead pagehead--row">
        <div>
          <p className="eyebrow">Monte Carlo</p>
          <h1>Campañas</h1>
          <p className="muted">Distribución del RMS de apuntamiento sobre cientos de pases. El p95 decide si la estación cumple.</p>
        </div>
        <a className="btn btn--primary" href={href('/')}>Lanzar una campaña</a>
      </header>
      {error && <div className="alert">No se pudieron cargar las campañas: {error.message}</div>}
      {!list && !error && <p className="muted">Cargando…</p>}
      {list && list.length === 0 && (
        <div className="empty"><h2>Todavía no hay campañas</h2><p className="muted">Lanza un experimento de tipo campaña desde el catálogo (nivel N4).</p></div>
      )}
      {list && list.length > 0 && (
        <>
          <div className="chips" role="tablist" aria-label="Campañas guardadas">
            {list.map((r) => (
              <button key={r.run_id} role="tab" aria-selected={r.run_id === current} className={`chip ${r.run_id === current ? 'is-on' : ''}`} onClick={() => go(`/campanas/${r.run_id}`)}>
                <span>{r.args?.mode ?? r.title} · n={r.args?.n ?? '?'}</span>
                <small>{fmtDate(r.created_at)}</small>
              </button>
            ))}
          </div>
          {detail?.error && <div className="alert">No se pudo cargar la campaña.</div>}
          {detail && !detail.error && (
            <>
              <p className="muted small">
                <code>{detail.run_id}</code> <StatusBadge status={detail.status} />{' '}
                <a href={href(`/runs/${detail.run_id}`)}>abrir la corrida</a>
              </p>
              {detail.campaign ? (
                <CampaignPanel campaign={detail.campaign} />
              ) : (
                <div className="panel"><p className="muted">{isActive(detail.status, detail.finished_at) ? 'Esta campaña sigue corriendo.' : 'Esta campaña no produjo resultados.'}</p></div>
              )}
            </>
          )}
          {!detail && <p className="muted">Cargando campaña…</p>}
        </>
      )}
    </div>
  )
}
