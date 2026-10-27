import { useEffect, useMemo, useState } from 'react'
import { cancelRender, crearRender, getMedios, getRenderCatalogo, listRenders } from '../api.js'
import { Field, check } from '../views/Experimentos.jsx'
import { fmtDur, hace, jobActivo } from '../format.js'
import { usePoll } from '../hooks.js'
import { href } from '../router.js'
import { JobBadge, ProgressBar } from './common.jsx'
import { MediaCard, MediaDetail } from './MediaViewer.jsx'
import { Panel } from './ui.jsx'

/** ¿Cumple la corrida lo que pide el render (`requiere`)? Devuelve null si si, o el motivo. */
export function motivoNoCumple(req, escena, status) {
  if (status !== 'done') return 'la corrida debe estar terminada'
  if (!req || req.cualquiera) return null
  if (req.contrato && !escena?.artefactos?.some((a) => a.contrato === req.contrato)) return `necesita una corrida con contrato «${req.contrato}»`
  // requiere.kind: un kind o una lista (manim-corrida acepta loop y campaign)
  const kinds = req.kind ? [].concat(req.kind) : null
  if (kinds && !kinds.includes(escena?.kind)) return `solo para corridas de kind ${kinds.map((k) => `«${k}»`).join(' o ')}`
  if (req.archivo && !escena?.artefactos?.some((a) => a.ruta === req.archivo)) return `necesita el archivo ${req.archivo}`
  return null
}

function RenderForm({ r, runId, motivo, onCreated }) {
  const schema = r.params_schema || {}
  const [vals, setVals] = useState(() => Object.fromEntries(Object.keys(schema).map((k) => [k, r.args?.[k] ?? (schema[k].type === 'enum' ? schema[k].options?.[0] : '')])))
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState(null)
  const errors = useMemo(() => Object.fromEntries(Object.entries(schema).map(([k, spec]) => [k, check(spec, vals[k])]).filter(([, m]) => m)), [schema, vals])
  const invalid = Object.keys(errors).length > 0
  async function go(ev) {
    ev.preventDefault()
    if (invalid || motivo) return
    setBusy(true)
    setErr(null)
    const args = {}
    for (const [k, spec] of Object.entries(schema)) args[k] = spec.type === 'number' || spec.type === 'integer' ? Number(vals[k]) : vals[k]
    try {
      const j = await crearRender(runId, r.id, args)
      onCreated?.(j)
    } catch (e) {
      setErr(e.message)
    } finally {
      setBusy(false)
    }
  }
  return (
    <form className="renderform" onSubmit={go} noValidate>
      <p className="muted small">{r.description ?? r.descripcion}</p>
      <p className="muted small">Recursos: <b>{r.recursos}</b>{r.pantalla ? ' · pantalla virtual del pool' : ''} · unos {fmtDur(r.duracion_estimada_s)} de pared</p>
      {Object.keys(schema).length > 0 && (
        <div className="fields">
          {Object.entries(schema).map(([k, spec]) => (
            <Field key={k} name={`rd-${r.id}-${k}`} spec={spec} value={vals[k]} error={errors[k]} onChange={(v) => setVals((s) => ({ ...s, [k]: v }))} />
          ))}
        </div>
      )}
      {motivo && <div className="note note--amber">No disponible: {motivo}.</div>}
      {err && <div className="alert">{err}</div>}
      <div className="renderform__go">
        <button className="btn btn--primary" type="submit" disabled={busy || invalid || !!motivo} data-testid="render-lanzar">{busy ? 'Encolando…' : 'Renderizar'}</button>
      </div>
    </form>
  )
}

/**
 * Panel «Renderizar» de la corrida: elige un render del catalogo, ajusta sus args, ve la cola de esta corrida
 * y los medios que ya salieron de ella. Los trabajos comparten el gobernador de recursos (esperan a que baje la carga).
 */
export default function RenderPanel({ run, escena }) {
  const runId = run.run_id
  const [cat, setCat] = useState(null)
  const [sel, setSel] = useState(null)
  const [openM, setOpenM] = useState(null)
  const { data: jobs, reload } = usePoll(() => listRenders({ run_id: runId, limit: 20 }), 2500, [runId])
  const [medios, setMedios] = useState([])
  const nHechos = (jobs || []).filter((j) => j.estado === 'hecho').length

  useEffect(() => { getRenderCatalogo().then(setCat).catch(() => setCat([])) }, [])
  // por defecto, el primer render que la corrida SI cumple (si no, el primero)
  useEffect(() => {
    if (!cat?.length || sel) return undefined
    const elegir = () => setSel((cat.find((r) => !motivoNoCumple(r.requiere, escena, run.status)) ?? cat[0]).id)
    if (escena) { elegir(); return undefined }
    const t = setTimeout(elegir, 1500) // sin la escena no se sabe que cumple: espera un momento y toma el primero
    return () => clearTimeout(t)
  }, [cat, escena, run.status, sel])
  useEffect(() => {
    getMedios({ escena: run.experiment_id, limit: 100 }).then((m) => setMedios(m.filter((x) => x.run_id === runId))).catch(() => {})
  }, [runId, run.experiment_id, nHechos])

  const actual = cat?.find((r) => r.id === sel)
  const titulo = Object.fromEntries((cat || []).map((r) => [r.id, r.title]))
  const ejecutorDe = Object.fromEntries((cat || []).map((r) => [r.id, r.ejecutor]))

  return (
    <Panel title="Renderizar" sub="Convierte la corrida en medios: video en Gazebo, animación Manim o gráficas" className="renderpanel" right={<a className="small" href={href('/render')}>Cola completa →</a>}>
      {!cat && <p className="muted">Cargando el catálogo de renders…</p>}
      {cat && cat.length === 0 && <p className="muted">No hay renders definidos (render/*.yaml).</p>}
      {cat && cat.length > 0 && (
        <div className="renderpanel__grid">
          <div>
            <div className="seg seg--wrap" role="radiogroup" aria-label="Render">
              {cat.map((r) => {
                const mot = motivoNoCumple(r.requiere, escena, run.status)
                return (
                  <button key={r.id} type="button" role="radio" aria-checked={sel === r.id} className={`${sel === r.id ? 'is-on' : ''}${mot ? ' is-off' : ''}`} onClick={() => setSel(r.id)} title={mot ? `No disponible: ${mot}` : r.title}>
                    {r.title.split(' (')[0]}
                  </button>
                )
              })}
            </div>
            {actual && <RenderForm key={actual.id + runId} r={actual} runId={runId} motivo={motivoNoCumple(actual.requiere, escena, run.status)} onCreated={reload} />}
          </div>
          <div>
            <h4 className="subh">Trabajos de esta corrida</h4>
            {jobs && jobs.length === 0 && <p className="muted small">Todavía no se ha renderizado esta corrida.</p>}
            <ul className="jobs" data-testid="jobs-corrida">
              {(jobs || []).map((j) => (
                <li key={j.job_id}>
                  <span className="jobs__t"><b>{(j.title ?? titulo[j.render_id] ?? j.render_id).split(' (')[0]}</b><span className="muted small" title={j.job_id}>{hace(j.created_at)}</span></span>
                  <JobBadge estado={j.estado} />
                  {j.estado === 'en_cola' && ejecutorDe[j.render_id] === 'host' && Date.now() - new Date(j.created_at).getTime() > 30000 && <span className="small tone-amber">esperando al trabajador del host</span>}
                  {j.error && <span className="small muted" title={j.error}>{j.error}</span>}
                  {jobActivo(j.estado) && <ProgressBar value={j.estado === 'corriendo' ? j.progreso : null} tone={j.estado === 'esperando_carga' ? 'amber' : 'cyan'} />}
                  {jobActivo(j.estado) && <button type="button" className="btn btn--sm btn--danger" onClick={() => cancelRender(j.job_id).then(reload).catch(() => {})}>Cancelar</button>}
                </li>
              ))}
            </ul>
          </div>
        </div>
      )}
      {medios.length > 0 && (
        <>
          <h4 className="subh">Medios de esta corrida</h4>
          <div className="mgrid mgrid--small" data-testid="medios-corrida">{medios.map((m) => <MediaCard key={m.job_id + m.ruta} m={m} onOpen={setOpenM} />)}</div>
        </>
      )}
      {openM && <MediaDetail m={openM} onClose={() => setOpenM(null)} />}
    </Panel>
  )
}
