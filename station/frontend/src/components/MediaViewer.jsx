import { useEffect, useRef, useState } from 'react'
import { getRender, mediaUrl } from '../api.js'
import { fmtBytes, fmtDate, fmtDur } from '../format.js'
import { href } from '../router.js'
import { JsonBlock, Thumb } from './common.jsx'

/** Miniatura de un medio: la del trabajo (miniatura.jpg) o, si es imagen, ella misma. */
export function medioThumb(m) {
  if (m.miniatura) return /^(\/|\.\/|data:|https?:)/.test(m.miniatura) ? m.miniatura : mediaUrl(m.job_id, m.miniatura) // la API da una URL; acepta tambien una ruta
  if (m.tipo === 'imagen') return medioUrl(m)
  return null
}
/** URL del archivo: la que trae el medio (API real) o /api/medios/{job}/{ruta}. */
export function medioUrl(m) {
  return m.url || mediaUrl(m.job_id, m.ruta)
}

export function MediaCard({ m, onOpen, area = 'red' }) {
  return (
    <button type="button" className="mcard" onClick={() => onOpen(m)} data-testid="medio-card" aria-label={`Abrir ${m.ruta} de ${m.render}`}>
      <span className="mcard__img">
        <Thumb src={medioThumb(m)} area={area} seed={m.job_id + m.ruta} alt="" />
        {m.tipo === 'video' && <span className="mcard__play" aria-hidden="true">▶</span>}
        {m.duracion_s != null && <span className="mcard__dur num">{fmtDur(m.duracion_s)}</span>}
      </span>
      <span className="mcard__txt">
        <b>{m.ruta ?? m.titulo}</b>
        <small className="muted">{m.render} · {m.escena ?? m.run_id}</small>
      </span>
    </button>
  )
}

function Procedencia({ m }) {
  // GET /api/medios ya trae `procedencia` y `notas`; si no (otra vista), se pide el trabajo
  const [job, setJob] = useState(m.procedencia ? { job_id: m.job_id, run_id: m.run_id, render_id: m.render, media: { procedencia: m.procedencia, notas: m.notas, render: m.render, duracion_pared_s: m.duracion_pared_s } } : undefined)
  useEffect(() => {
    if (job) return undefined
    let dead = false
    getRender(m.job_id).then((j) => !dead && setJob(j)).catch(() => !dead && setJob(null))
    return () => { dead = true }
  }, [m.job_id, job])
  const p = job?.media?.procedencia
  return (
    <section className="prov">
      <h4>Procedencia</h4>
      {job === undefined && <p className="muted small">Cargando…</p>}
      {job === null && <p className="muted small">El backend no entregó el media.json de este trabajo.</p>}
      {job && (
        <>
          <dl className="kv">
            <dt>Trabajo</dt><dd><code>{job.job_id}</code></dd>
            <dt>Render</dt><dd>{job.media?.render ?? job.render_id}</dd>
            <dt>Corrida</dt><dd><a href={href(`/runs/${job.run_id}`)}><code>{job.run_id}</code></a></dd>
            {p?.commit && <><dt>Commit</dt><dd><code>{String(p.commit).slice(0, 12)}</code>{p.sucio ? <span className="vchip vchip--bad vchip--inline">árbol sucio</span> : <span className="vchip vchip--ok vchip--inline">árbol limpio</span>}</dd></>}
            {job.media?.duracion_pared_s != null && <><dt>Tiempo de pared</dt><dd>{fmtDur(job.media.duracion_pared_s)}</dd></>}
            {job.media?.notas && <><dt>Notas</dt><dd>{job.media.notas}</dd></>}
          </dl>
          {p && (
            <details>
              <summary>Comando y argumentos exactos</summary>
              <JsonBlock value={{ comando: p.comando, args: p.args, corrida: p.corrida }} />
            </details>
          )}
        </>
      )}
    </section>
  )
}

/** Ventana con el reproductor (video por Range) o la imagen, sus datos, descarga y procedencia. */
export function MediaDetail({ m, onClose, area = 'red' }) {
  const ref = useRef(null)
  useEffect(() => {
    const f = (e) => e.key === 'Escape' && onClose()
    window.addEventListener('keydown', f)
    ref.current?.focus()
    return () => window.removeEventListener('keydown', f)
  }, [onClose])
  const url = medioUrl(m)
  return (
    <div className="modal" role="dialog" aria-modal="true" aria-label={`Medio ${m.ruta}`} onMouseDown={(e) => e.target === e.currentTarget && onClose()}>
      <div className="modal__card" ref={ref} tabIndex={-1} data-testid="medio-detalle">
        <header className="modal__head">
          <div>
            <p className="eyebrow">{m.render} · {m.tipo}</p>
            <h2>{m.ruta}</h2>
          </div>
          <button type="button" className="btn btn--ghost btn--sm" onClick={onClose} aria-label="Cerrar">Cerrar ✕</button>
        </header>
        <div className="modal__body">
          <div className="modal__player">
            {m.tipo === 'video' ? (
              <video controls preload="metadata" playsInline poster={medioThumb(m) || undefined} src={url} data-testid="medio-video" />
            ) : m.tipo === 'imagen' ? (
              <img src={url} alt={m.ruta} onError={(e) => { e.currentTarget.style.display = 'none' }} />
            ) : (
              <div className="empty"><Thumb area={area} seed={m.ruta} alt="" /><p className="muted">Archivo de texto: descárgalo para verlo.</p></div>
            )}
          </div>
          <aside className="modal__side">
            <dl className="kv">
              {m.ancho && <><dt>Resolución</dt><dd className="num">{m.ancho}×{m.alto}</dd></>}
              {m.duracion_s != null && <><dt>Duración</dt><dd className="num">{fmtDur(m.duracion_s)}</dd></>}
              {m.tam_bytes != null && <><dt>Tamaño</dt><dd className="num">{fmtBytes(m.tam_bytes)}</dd></>}
              <dt>Creado</dt><dd>{fmtDate(m.creado_utc)}</dd>
              {m.escena && <><dt>Escena</dt><dd><a href={href(`/escenas/${m.escena}`)}>{m.escena}</a></dd></>}
              {m.sha256 && <><dt>sha256</dt><dd><code className="hash" title={m.sha256}>{m.sha256.slice(0, 16)}…</code></dd></>}
            </dl>
            <div className="toolbar">
              <a className="btn btn--primary btn--sm" href={url} download={m.ruta}>Descargar</a>
              <a className="btn btn--sm" href={href(`/runs/${m.run_id}`)}>Abrir la corrida</a>
            </div>
            <Procedencia m={m} />
          </aside>
        </div>
      </div>
    </div>
  )
}
