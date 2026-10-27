import { useEffect, useState } from 'react'
import { getMedios, getSistema, listRenders, listRuns } from '../api.js'
import { JobBadge, Thumb } from '../components/common.jsx'
import { medioThumb } from '../components/MediaViewer.jsx'
import { Panel, StatusBadge, Tile } from '../components/ui.jsx'
import { usePoll } from '../hooks.js'
import { fmtDate, hace, jobActivo, nf } from '../format.js'
import { href } from '../router.js'
import logo from '../assets/logo-cian.png'

const CLASES = [
  ['ligero', 'Ligero', 'plan, métricas, gráficas'],
  ['pesado', 'Pesado', 'lockstep ROS, misión, estudios'],
  ['gazebo', 'Gazebo', 'render por software'],
  ['manim', 'Manim', 'video explicativo'],
]

function CargaBar({ valor, umbral }) {
  const max = Math.max(umbral * 1.6, valor * 1.1, 1)
  return (
    <div className="loadbar" role="img" aria-label={`carga ${nf(valor, 1)} de un umbral de ${umbral}`}>
      <i className={valor >= umbral ? 'is-over' : ''} style={{ width: `${Math.min(100, (valor / max) * 100)}%` }} />
      <b style={{ left: `${(umbral / max) * 100}%` }} title={`umbral ${umbral}`} />
    </div>
  )
}

function Sistema({ s }) {
  if (!s) return <Panel title="Sistema"><p className="muted">Consultando el estado del host…</p></Panel>
  const l1 = s.carga?.l1
  const mem = s.memoria || {}
  const gob = s.gobernador || {}
  const nAct = (s.activas?.corridas ?? 0) + (s.activas?.renders ?? 0)
  const espera = gob.esperando || []
  const vig = s.vigilante
  return (
    <Panel
      title="Sistema"
      sub="Carga del host, colas del gobernador de recursos y vigilante"
      right={<span className="chip-status chip-status--ok" title={`El gobernador reevalúa la carga cada ${gob.recheck_s ?? 10} s`}><i className="pulse" /> en vivo</span>}
    >
      <div className="tiles">
        <Tile label="Carga 1 min" value={nf(l1, 2)} tone={l1 >= (s.cupos?.pesado?.umbral_carga ?? 6) ? 'amber' : 'cyan'} hint={`5 min ${nf(s.carga?.l5, 2)} · 15 min ${nf(s.carga?.l15, 2)} · ${s.carga?.hilos ?? '?'} hilos`} />
        <Tile label="Carga 5 / 15 min" value={`${nf(s.carga?.l5, 1)} / ${nf(s.carga?.l15, 1)}`} tone="gray" />
        <Tile label="Memoria usada" value={mem.usada_pct == null ? '—' : nf(mem.usada_pct, 0)} unit=" %" tone={mem.usada_pct > 85 ? 'amber' : 'cyan'} hint={mem.total_mb ? `${mem.total_mb - mem.disponible_mb} de ${mem.total_mb} MB` : ''} />
        <Tile label="Tareas activas" value={nAct} tone={nAct ? 'cyan' : 'gray'} hint={`${s.activas?.corridas ?? 0} corridas · ${s.activas?.renders ?? 0} renders`} />
      </div>
      <div className="tablewrap tablewrap--flush">
        <table className="table table--compact" data-testid="cupos">
          <thead><tr><th>Clase</th><th className="num">Cupo</th><th className="num">En uso</th><th className="num">En cola</th><th className="num">Espera carga</th><th>Carga contra el umbral de entrada</th></tr></thead>
          <tbody>
            {CLASES.map(([k, nombre, ej]) => {
              const c = s.cupos?.[k]
              const q = s.colas?.[k] || {}
              if (!c) return null
              return (
                <tr key={k}>
                  <td><b>{nombre}</b> <span className="muted small">{ej}</span></td>
                  <td className="num">{c.cupo}</td>
                  <td className={`num ${q.corriendo ? 'tone-cyan' : ''}`}>{q.corriendo ?? 0}</td>
                  <td className={`num ${q.en_cola ? 'tone-amber' : ''}`}>{q.en_cola ?? 0}</td>
                  <td className={`num ${q.esperando_carga ? 'tone-amber' : ''}`}>{q.esperando_carga ?? 0}</td>
                  <td style={{ minWidth: 200 }}><CargaBar valor={l1 ?? 0} umbral={c.umbral_carga} /><span className="muted small num">{c.admite_ahora ? 'admite ahora' : 'no admite ahora'} · entra con carga &lt; {c.umbral_carga}</span></td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </div>
      {espera.length > 0 && (
        <div className="vigil" data-testid="en-espera">
          <h4>En espera</h4>
          <ul>
            {espera.map((e, i) => (
              <li key={i}><span className="tone-amber">{e.motivo === 'carga' ? 'esperando carga' : 'en cola'}</span> {e.tarea} <span className="muted small">· clase {e.clase} · {Math.round(e.espera_s)} s</span></li>
            ))}
          </ul>
        </div>
      )}
      <div className="vigil">
        <h4>Vigilante del host</h4>
        {vig == null ? (
          <p className="muted small">El diario del vigilante no se puede leer desde aquí (el contenedor no monta el diario del host). Si detiene un trabajo lo verás como «interrumpida».</p>
        ) : vig.lineas?.length ? (
          <>
            <ul>
              {vig.lineas.slice(-4).map((a, i) => (
                <li key={i}>{/SOBRECARGA/.test(a.texto) ? <span className="tone-amber">{a.texto}</span> : a.texto} <span className="muted small">· {hace(a.t)}</span></li>
              ))}
            </ul>
          </>
        ) : <p className="muted small">Sin acciones recientes.</p>}
        <p className="muted small">La app convive con el vigilante: si detiene una tarea, pasa a «interrumpida» y se reintenta hasta {gob.reintentos ?? 2} veces con espera creciente.</p>
      </div>
    </Panel>
  )
}

export default function Inicio() {
  const { data: sis } = usePoll(getSistema, 4000)
  const [runs, setRuns] = useState(null)
  const [medios, setMedios] = useState(null)
  useEffect(() => {
    listRuns({ limit: 6 }).then(setRuns).catch(() => setRuns([]))
    getMedios({ limit: 4 }).then(setMedios).catch(() => setMedios([]))
  }, [])
  const { data: jobs } = usePoll(() => listRenders({ limit: 30 }), 4000)
  const activos = (jobs || []).filter((j) => jobActivo(j.estado))

  return (
    <div className="page">
      <section className="hero">
        <img src={logo} alt="Co.De Aerospace" className="hero__logo" />
        <div className="hero__txt">
          <p className="eyebrow">Co.De Aerospace · Estación ATP</p>
          <h1>Estudio de simulación aeroespacial</h1>
          <p className="muted">Ejecuta una escena sobre el motor ROS 2, míralo en vivo, renderízalo a video y guarda la evidencia con su procedencia.</p>
          <div className="hero__cta">
            <a className="btn btn--primary" href={href('/escenas')}>Explorar escenas</a>
            <a className="btn" href={href('/medios')}>Ver medios</a>
          </div>
        </div>
      </section>

      <div className="grid2 grid2--home">
        <Sistema s={sis} />
        <div className="stack">
          <Panel title="Últimas corridas" right={<a href={href('/biblioteca')} className="small">Biblioteca →</a>}>
            {!runs && <p className="muted">Cargando…</p>}
            {runs && runs.length === 0 && <p className="muted">Todavía no hay corridas.</p>}
            <ul className="list">
              {(runs || []).map((r) => (
                <li key={r.run_id}>
                  <a href={href(`/runs/${r.run_id}`)}><b>{r.title || r.experiment_id}</b></a>
                  <span className="muted small" title={fmtDate(r.created_at)}>{hace(r.created_at)}</span>
                  <StatusBadge status={r.status} />
                </li>
              ))}
            </ul>
          </Panel>
          {activos.length > 0 && (
            <Panel title="Renders en curso" right={<a href={href('/render')} className="small">Cola →</a>}>
              <ul className="list">
                {activos.map((a) => (
                  <li key={a.job_id}><a href={href('/render')}><b>{(a.title ?? a.render_id).split(' (')[0]}</b></a><span className="muted small num">{a.progreso != null ? `${Math.round(a.progreso * 100)} %` : ''}</span><JobBadge estado={a.estado} /></li>
                ))}
              </ul>
            </Panel>
          )}
        </div>
      </div>

      <Panel title="Últimos medios" right={<a href={href('/medios')} className="small">Galería →</a>}>
        {!medios && <p className="muted">Cargando…</p>}
        {medios && medios.length === 0 && <p className="muted">Aún no hay medios: renderiza una corrida desde su página.</p>}
        <div className="mini-media">
          {(medios || []).map((m) => (
            <a key={m.job_id + m.ruta} className="mini-media__item" href={href('/medios')}>
              <Thumb src={medioThumb(m)} area="red" seed={m.job_id + m.ruta} alt="" />
              <span>{m.render}</span>
              <small className="muted">{m.escena}</small>
            </a>
          ))}
        </div>
      </Panel>
    </div>
  )
}
