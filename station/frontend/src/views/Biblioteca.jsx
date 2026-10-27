import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { ApiError, deleteRun, downloadCsv, getExperiments, getRun, listRuns } from '../api.js'
import { ConfirmButton, StatusBadge } from '../components/ui.jsx'
import { fmtDate, isActive, kindOfRun, nf } from '../format.js'
import { go, href } from '../router.js'

const KIND_TAG = { campaign: 'campaña', plan: 'plan', adcs: 'adcs', mision: 'misión' }

function VerdictCell({ run, kind }) {
  if (kind === 'plan' || kind === 'mision') {
    // un plan no cumple ni falla: se muestra lo que baja
    return run.metrics?.gb != null ? <span className="vchip vchip--info" title="Datos bajados (plan óptimo / contactos ejecutados)">{nf(run.metrics.gb, 0)} GB</span> : <span className="muted">—</span>
  }
  const v = kind === 'campaign' ? (run.campaign ? (run.campaign.pasa ? 'PASA' : 'NO PASA') : null) : run.metrics?.verdict
  if (!v) return <span className="muted">—</span>
  const ok = v === 'CUMPLE' || v === 'PASA'
  return <span className={`vchip ${ok ? 'vchip--ok' : 'vchip--bad'}`}>{v}</span>
}

export default function Biblioteca() {
  const [runs, setRuns] = useState(null)
  const [expMap, setExpMap] = useState({})
  const [error, setError] = useState(null)
  const [msg, setMsg] = useState(null)
  const [sel, setSel] = useState([])
  const [fExp, setFExp] = useState('')
  const [fStatus, setFStatus] = useState('')
  const campCache = useRef({}) // run_id -> campaign (para mostrar p95 en filas de campana)

  const load = useCallback(async () => {
    try {
      const [rs, ex] = await Promise.all([listRuns({ limit: 300 }), getExperiments()])
      const map = Object.fromEntries(ex.map((e) => [e.id, e]))
      setExpMap(map)
      // las filas de campana no traen campaign en la lista: se completan desde el detalle
      await Promise.all(
        rs
          .filter((r) => r.status === 'done' && map[r.experiment_id]?.kind === 'campaign' && !campCache.current[r.run_id])
          .slice(0, 40)
          .map((r) => getRun(r.run_id).then((d) => { campCache.current[r.run_id] = d.campaign }).catch(() => {})),
      )
      setRuns(rs.map((r) => (campCache.current[r.run_id] ? { ...r, campaign: campCache.current[r.run_id] } : r)))
      setError(null)
    } catch (e) {
      setError(e)
    }
  }, [])

  useEffect(() => { load() }, [load])
  useEffect(() => {
    if (!runs?.some((r) => isActive(r.status, r.finished_at))) return
    const t = setInterval(load, 3000)
    return () => clearInterval(t)
  }, [runs, load])

  const shown = useMemo(
    () => (runs || []).filter((r) => (!fExp || r.experiment_id === fExp) && (!fStatus || r.status === fStatus)),
    [runs, fExp, fStatus],
  )

  const toggle = (id) => setSel((s) => (s.includes(id) ? s.filter((x) => x !== id) : s.length >= 2 ? [s[1], id] : [...s, id]))

  async function remove(r) {
    try {
      await deleteRun(r.run_id)
      setRuns((rs) => rs.filter((x) => x.run_id !== r.run_id))
      setSel((s) => s.filter((x) => x !== r.run_id))
      setMsg(null)
    } catch (e) {
      setMsg(e instanceof ApiError && e.status === 409 ? 'No se puede borrar una corrida activa: cancélala primero.' : `No se pudo borrar: ${e.message}`)
    }
  }

  const selRuns = sel.map((id) => runs?.find((r) => r.run_id === id)).filter(Boolean)
  const canCompare = selRuns.length === 2 && selRuns.every((r) => kindOfRun(r, expMap) === 'loop')

  return (
    <div className="page">
      <header className="pagehead pagehead--row">
        <div>
          <p className="eyebrow">Historial</p>
          <h1>Biblioteca</h1>
          <p className="muted">Cada corrida guardada con sus parámetros y métricas. Marca dos para superponer sus curvas.</p>
        </div>
        <div className="toolbar">
          <select value={fExp} onChange={(e) => setFExp(e.target.value)} aria-label="Filtrar por experimento">
            <option value="">Todos los experimentos</option>
            {Object.values(expMap).map((e) => <option key={e.id} value={e.id}>{e.level} · {e.title}</option>)}
          </select>
          <select value={fStatus} onChange={(e) => setFStatus(e.target.value)} aria-label="Filtrar por estado">
            <option value="">Todos los estados</option>
            {['done', 'running', 'queued', 'esperando_carga', 'interrumpida', 'error', 'cancelled', 'timeout'].map((s) => <option key={s} value={s}>{s}</option>)}
          </select>
          <button className="btn btn--primary" disabled={!canCompare} onClick={() => go(`/compare/${sel[0]}/${sel[1]}`)} title={canCompare ? '' : 'Marca dos corridas de lazo'}>
            Comparar ({selRuns.length}/2)
          </button>
        </div>
      </header>
      {error && <div className="alert">No se pudo cargar la biblioteca: {error.message}</div>}
      {msg && <div className="alert">{msg}</div>}
      {!runs && !error && <p className="muted">Cargando…</p>}
      {runs && shown.length === 0 && (
        <div className="empty">
          <h2>Aún no hay corridas{fExp || fStatus ? ' con ese filtro' : ''}</h2>
          <a className="btn btn--primary" href={href('/')}>Lanzar un experimento</a>
        </div>
      )}
      {shown.length > 0 && (
        <div className="tablewrap">
          <table className="table">
            <thead>
              <tr>
                <th aria-label="Comparar" />
                <th>Estado</th>
                <th>Experimento</th>
                <th>Fecha</th>
                <th className="num">p95 (°)</th>
                <th className="num">Margen mín (dB)</th>
                <th>Veredicto</th>
                <th aria-label="Acciones" />
              </tr>
            </thead>
            <tbody>
              {shown.map((r) => {
                const kind = kindOfRun(r, expMap)
                const camp = kind === 'campaign'
                const p95 = camp ? r.campaign?.p95 : kind === 'adcs' ? r.metrics?.track?.p95 : r.metrics?.pointing?.p95
                const mm = r.metrics?.link?.margin_min_db
                const obj = camp ? r.campaign?.objective_deg ?? 0.1 : kind === 'adcs' ? r.metrics?.track?.threshold_deg ?? 2.4 : r.metrics?.pointing?.objective_deg ?? 0.1
                return (
                  <tr key={r.run_id} className={sel.includes(r.run_id) ? 'is-sel' : ''}>
                    <td>
                      <input
                        type="checkbox"
                        checked={sel.includes(r.run_id)}
                        disabled={kind !== 'loop'}
                        onChange={() => toggle(r.run_id)}
                        aria-label={`Seleccionar ${r.run_id} para comparar`}
                        title={kind !== 'loop' ? 'Solo se comparan corridas de lazo' : ''}
                      />
                    </td>
                    <td><StatusBadge status={r.status} /></td>
                    <td>
                      <a className="rowlink" href={href(`/runs/${r.run_id}`)}>{r.title || r.experiment_id}</a>
                      <div className="muted small">
                        <code>{r.run_id}</code>
                        {KIND_TAG[kind] && <span className={`tag tag--${kind}`}>{KIND_TAG[kind]}</span>}
                      </div>
                    </td>
                    <td className="nowrap">{fmtDate(r.created_at)}</td>
                    <td className={`num ${p95 != null ? (p95 > obj ? 'tone-red' : 'tone-amber') : ''}`}>{p95 != null ? nf(p95, 4) : '—'}</td>
                    <td className={`num ${mm != null ? (mm >= 0 ? 'tone-green' : 'tone-red') : ''}`}>{mm != null ? nf(mm, 2) : '—'}</td>
                    <td><VerdictCell run={r} kind={kind} /></td>
                    <td>
                      <div className="actions">
                      <a className="btn btn--sm" href={href(`/runs/${r.run_id}`)}>Abrir</a>
                      {(kind === 'loop' || kind === 'adcs') && r.status !== 'queued' && r.status !== 'esperando_carga' && <button className="btn btn--sm" onClick={() => downloadCsv(r.run_id)}>CSV</button>}
                      <ConfirmButton className="btn--sm" disabled={isActive(r.status, r.finished_at)} confirmText="¿Borrar? Confirmar" onConfirm={() => remove(r)}>Borrar</ConfirmButton>
                      </div>
                    </td>
                  </tr>
                )
              })}
            </tbody>
          </table>
        </div>
      )}
    </div>
  )
}
