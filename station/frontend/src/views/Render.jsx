import { useEffect, useMemo, useRef, useState } from 'react'
import { cancelRender, getRenderCatalogo, listRenders, mediaUrl } from '../api.js'
import { FilterChips, JobBadge, PageHead, ProgressBar } from '../components/common.jsx'
import { MediaCard, MediaDetail } from '../components/MediaViewer.jsx'
import { ConfirmButton, Tile } from '../components/ui.jsx'
import { fmtDate, fmtDur, hace, jobActivo } from '../format.js'
import { usePoll } from '../hooks.js'
import { href } from '../router.js'
import { useJobLive } from '../useJobLive.js'

const GRUPOS = [
  { value: 'activos', label: 'Activos' },
  { value: 'hechos', label: 'Hechos' },
  { value: 'problemas', label: 'Con problema' },
]
const enGrupo = (g, e) => (g === 'activos' ? jobActivo(e) : g === 'hechos' ? e === 'hecho' : g === 'problemas' ? ['error', 'interrumpida', 'cancelado'].includes(e) : true)

export function JobDetail({ jobId, onCancelled, ejecutor }) {
  const { job, log, progreso, estado, error } = useJobLive(jobId)
  const logRef = useRef(null)
  const [sel, setSel] = useState(null)
  useEffect(() => { if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight }, [log])
  if (error) return <div className="alert">No se pudo abrir el trabajo: {error.message}</div>
  if (!job) return <p className="muted">Cargando trabajo…</p>
  const activo = jobActivo(estado)
  const esperaTrabajador = ejecutor === 'host' && job.created_at && Date.now() - new Date(job.created_at).getTime() > 30000
  const medios = (job.media?.archivos || []).filter((f) => f.tipo !== 'texto' && f.ruta !== 'miniatura.jpg').map((f) => ({
    ...f, job_id: job.job_id, run_id: job.run_id, render: job.render_id ?? job.render, escena: null, url: mediaUrl(job.job_id, f.ruta),
    creado_utc: job.media?.creado_utc, procedencia: job.media?.procedencia, notas: job.media?.notas,
    miniatura: job.media?.archivos?.some((x) => x.ruta === 'miniatura.jpg') ? mediaUrl(job.job_id, 'miniatura.jpg') : null,
  }))
  return (
    <div className="jobdetail" data-testid="job-detalle" data-estado={estado}>
      <header className="jobdetail__head">
        <div>
          <p className="eyebrow">{job.render_id ?? job.render} · <code>{job.job_id}</code></p>
          <h2><a href={href(`/runs/${job.run_id}`)}>Corrida {job.run_id}</a></h2>
        </div>
        <div className="runhead__actions">
          <JobBadge estado={estado} />
          {activo && <ConfirmButton onConfirm={() => cancelRender(job.job_id).then(() => onCancelled?.()).catch(() => {})}>Cancelar</ConfirmButton>}
        </div>
      </header>
      <ProgressBar value={estado === 'hecho' ? 1 : progreso} tone={estado === 'error' || estado === 'interrumpida' ? 'red' : estado === 'esperando_carga' ? 'amber' : 'cyan'} label="Progreso del render" />
      <p className="muted small num">
        {progreso != null ? `${Math.round((estado === 'hecho' ? 1 : progreso) * 100)} %` : estado === 'corriendo' ? 'sin progreso informado' : ''}
        {job.pantalla ? ` · pantalla ${job.pantalla}` : ''}
        {job.duracion_pared_s != null ? ` · ${fmtDur(job.duracion_pared_s)} de pared` : ''}
        {job.reintentos ? ` · ${job.reintentos} reintento${job.reintentos === 1 ? '' : 's'}` : ''}
        {job.clase ? ` · clase ${job.clase}` : ''}
      </p>
      {job.error && <div className={estado === 'interrumpida' ? 'note note--amber' : 'alert'} data-testid="job-error">{estado === 'interrumpida' ? 'Interrumpida: ' : ''}{job.error}</div>}
      {estado === 'interrumpida' && !job.error && <div className="note note--amber">Interrumpida por una señal externa (p. ej. el vigilante del host).</div>}
      {estado === 'en_cola' && esperaTrabajador && <div className="note note--amber" data-testid="nota-trabajador">Esperando al trabajador del host: este render lo ejecuta <code>scripts/render/trabajador_host.py</code> (necesita Docker y pantalla, que el contenedor no tiene) y todavía no lo ha reclamado. Si lleva mucho, comprueba que el trabajador esté corriendo en el host.</div>}
      {estado === 'esperando_carga' && <div className="note note--amber">El gobernador espera a que baje la carga del host (se reevalúa cada 10 s). No hace falta hacer nada.</div>}
      <h3 className="subh">Registro</h3>
      <pre className="log__body log__body--tall" ref={logRef} data-testid="job-log">{log.length ? log.join('\n') : 'Sin salida todavía.'}</pre>
      {estado === 'hecho' && medios.length > 0 && (
        <>
          <h3 className="subh">Medios generados</h3>
          <div className="mgrid mgrid--small">{medios.map((m) => <MediaCard key={m.ruta} m={m} onOpen={setSel} />)}</div>
        </>
      )}
      {sel && <MediaDetail m={sel} onClose={() => setSel(null)} />}
    </div>
  )
}

export default function Render() {
  const [grupo, setGrupo] = useState('')
  const [sel, setSel] = useState(null)
  const { data: jobs, error, reload } = usePoll(() => listRenders({ limit: 100 }), 2500)
  const [cat, setCat] = useState([])
  useEffect(() => { getRenderCatalogo().then(setCat).catch(() => {}) }, [])
  const titulo = useMemo(() => Object.fromEntries(cat.map((r) => [r.id, r.title])), [cat])
  const ejecutorDe = useMemo(() => Object.fromEntries(cat.map((r) => [r.id, r.ejecutor])), [cat])

  const lista = (jobs || []).filter((j) => !grupo || enGrupo(grupo, j.estado))
  // por defecto abre el primer trabajo activo (o el mas nuevo)
  useEffect(() => {
    if (!sel && jobs?.length) setSel((jobs.find((j) => jobActivo(j.estado)) || jobs[0]).job_id)
  }, [jobs, sel])
  const cuenta = (g) => (jobs || []).filter((j) => enGrupo(g, j.estado)).length

  return (
    <div className="page">
      <PageHead eyebrow="Cola de render" title="Render">
        Los trabajos de Gazebo, Manim y gráficas comparten el gobernador de recursos con las corridas: esperan a que baje la carga y, si el vigilante los detiene, se reintentan.
      </PageHead>
      <div className="tiles">
        <Tile label="Activos" value={cuenta('activos')} tone={cuenta('activos') ? 'cyan' : 'gray'} />
        <Tile label="Esperando carga" value={(jobs || []).filter((j) => j.estado === 'esperando_carga').length} tone="amber" />
        <Tile label="Hechos" value={cuenta('hechos')} tone="green" />
        <Tile label="Con problema" value={cuenta('problemas')} tone={cuenta('problemas') ? 'red' : 'gray'} />
      </div>
      <div className="filters"><FilterChips label="Mostrar" options={GRUPOS.map((g) => ({ ...g, n: cuenta(g.value) }))} value={grupo} onChange={setGrupo} all="Todos" /></div>
      {error && <div className="alert">No se pudo leer la cola: {error.message}</div>}
      {jobs && jobs.length === 0 && <div className="empty"><p>No hay trabajos de render. Abre una corrida terminada y usa «Renderizar».</p><a className="btn" href={href('/biblioteca')}>Ir a las corridas</a></div>}
      {jobs && jobs.length > 0 && (
        <div className="queue">
          <ul className="queue__list" data-testid="cola-render">
            {lista.map((j) => (
              <li key={j.job_id}>
                <button type="button" className={`qitem${sel === j.job_id ? ' is-sel' : ''}`} onClick={() => setSel(j.job_id)} data-estado={j.estado} aria-current={sel === j.job_id}>
                  <span className="qitem__top"><b>{j.title ?? titulo[j.render_id] ?? j.render_id}</b><JobBadge estado={j.estado} /></span>
                  <span className="muted small"><code>{j.run_id.slice(0, 22)}</code> · <span title={fmtDate(j.created_at)}>{hace(j.created_at)}</span></span>
                  {j.estado === 'en_cola' && ejecutorDe[j.render_id] === 'host' && Date.now() - new Date(j.created_at).getTime() > 30000 && <span className="small tone-amber">esperando al trabajador del host</span>}
                  {jobActivo(j.estado) && <ProgressBar value={j.estado === 'corriendo' ? j.progreso : j.estado === 'esperando_carga' ? 0 : null} tone={j.estado === 'esperando_carga' ? 'amber' : 'cyan'} />}
                </button>
              </li>
            ))}
            {lista.length === 0 && <li className="muted small qempty">Ningún trabajo en este grupo.</li>}
          </ul>
          <section className="panel queue__detail">{sel ? <JobDetail key={sel} jobId={sel} onCancelled={reload} ejecutor={ejecutorDe[(jobs || []).find((j) => j.job_id === sel)?.render_id]} /> : <p className="muted">Elige un trabajo.</p>}</section>
        </div>
      )}
    </div>
  )
}
