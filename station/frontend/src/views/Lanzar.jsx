import { useEffect, useMemo, useState } from 'react'
import { createRun, getEscenas, listRuns } from '../api.js'
import { Thumb } from '../components/common.jsx'
import { StatusBadge } from '../components/ui.jsx'
import { AREA_LABEL, fmtDate, hace, secciones } from '../format.js'
import { go, href } from '../router.js'
import { Field, check } from './Experimentos.jsx'

const CONTRATO_LABEL = {
  red: 'Corrida de red: habilita el visor 3D web y el render en Gazebo',
  red_eventos: 'Eventos de handover',
  topologia: 'Topología enlace a enlace (Doppler, latencia, quién sirve a quién)',
  telemetria: 'Telemetría del lazo (10 Hz)',
  campana: 'Campaña Monte Carlo',
  plan: 'Plan de contactos',
}
const RECURSO_AYUDA = {
  ligero: 'ligero: hasta 2 a la vez, entra con carga del host < 10',
  pesado: 'pesado: una a la vez (lockstep ROS), entra con carga < 6',
  gazebo: 'gazebo: render por software, entra con carga < 4',
  manim: 'manim: una a la vez, entra con carga < 5',
}

export default function Lanzar({ id }) {
  const [escena, setEscena] = useState(undefined)
  const [error, setError] = useState(null)
  const [prev, setPrev] = useState(null)

  useEffect(() => {
    let dead = false
    getEscenas().then((l) => !dead && setEscena(l.find((e) => e.id === id) || null)).catch((e) => !dead && setError(e))
    listRuns({ experiment_id: id, limit: 8 }).then((r) => !dead && setPrev(r)).catch(() => !dead && setPrev([]))
    return () => { dead = true }
  }, [id])

  if (error) return <div className="page"><div className="alert">No se pudo cargar la escena: {error.message}</div></div>
  if (escena === undefined) return <div className="page"><p className="muted">Cargando escena…</p></div>
  if (escena === null) {
    return (
      <div className="page"><div className="empty"><h2>No existe la escena «{id}»</h2><a className="btn" href={href('/escenas')}>Volver al catálogo</a></div></div>
    )
  }
  return <Formulario key={escena.id} e={escena} prev={prev} />
}

function Formulario({ e, prev }) {
  const schema = e.params_schema || {}
  const [vals, setVals] = useState(() => {
    const v = {}
    for (const k of Object.keys(schema)) v[k] = e.args?.[k] ?? (schema[k].type === 'enum' ? schema[k].options?.[0] : '')
    return v
  })
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState(null)
  const { partes, resumen } = useMemo(() => secciones(e.description), [e.description])

  const errors = useMemo(() => {
    const o = {}
    for (const [k, spec] of Object.entries(schema)) {
      const m = check(spec, vals[k])
      if (m) o[k] = m
    }
    return o
  }, [schema, vals])
  const invalid = Object.keys(errors).length > 0
  const cambiado = Object.keys(schema).some((k) => String(vals[k]) !== String(e.args?.[k]))

  async function lanzar(ev) {
    ev.preventDefault()
    if (invalid) return
    setBusy(true)
    setErr(null)
    const args = {}
    for (const [k, spec] of Object.entries(schema)) args[k] = spec.type === 'number' || spec.type === 'integer' ? Number(vals[k]) : vals[k]
    try {
      const r = await createRun(e.id, args)
      go(`/runs/${r.run_id}`)
    } catch (x) {
      setErr(x.message)
      setBusy(false)
    }
  }

  return (
    <div className="page">
      <nav className="crumbs" aria-label="Ruta"><a href={href('/escenas')}>Escenas</a><span>/</span><code>{e.id}</code></nav>
      <div className="launch">
        <section className="launch__main">
          <header className="launch__head">
            <p className="eyebrow">{e.level} · {AREA_LABEL[e.area] ?? e.area} · {e.kind}</p>
            <h1>{e.title}</h1>
            <div className="scene__meta">
              <span className="tag tag--area">{e.recursos}</span>
            </div>
          </header>

          {partes.length > 0 ? (
            <dl className="whys">
              {partes.map((p) => (
                <div key={p.titulo} className={`why${p.titulo === 'Validez' ? ' why--warn' : ''}`}>
                  <dt>{p.titulo}</dt>
                  <dd>{p.texto}</dd>
                </div>
              ))}
            </dl>
          ) : (
            <p className="launch__desc">{resumen}</p>
          )}

          <form className="panel launch__form" onSubmit={lanzar} noValidate>
            <header className="panel__head">
              <div>
                <h2 className="panel__title">Parámetros</h2>
                <p className="panel__sub">{Object.keys(schema).length ? 'Valores por defecto de la escena; cámbialos y lanza.' : 'Esta escena no tiene parámetros: se lanza tal cual.'}</p>
              </div>
              {cambiado && (
                <button type="button" className="btn btn--sm btn--ghost" onClick={() => setVals(Object.fromEntries(Object.keys(schema).map((k) => [k, e.args?.[k] ?? ''])))}>Restablecer</button>
              )}
            </header>
            <div className="fields fields--wide">
              {Object.entries(schema).map(([k, spec]) => (
                <Field key={k} name={`${e.id}-${k}`} spec={spec} value={vals[k]} error={errors[k]} onChange={(v) => setVals((s) => ({ ...s, [k]: v }))} />
              ))}
            </div>
            {err && <div className="alert">{err}</div>}
            <footer className="launch__go">
              <span className="muted small">{RECURSO_AYUDA[e.recursos] ?? ''}</span>
              <button className="btn btn--primary" type="submit" disabled={busy || invalid}>{busy ? 'Lanzando…' : 'Lanzar corrida'}</button>
            </footer>
          </form>
        </section>

        <aside className="launch__side">
          <div className="panel">
            <Thumb src={e.miniatura} area={e.area} seed={e.id} alt={`Miniatura de ${e.title}`} className="thumb--wide" />
            <h3 className="panel__title">Qué produce</h3>
            {e.artefactos?.length ? (
              <ul className="arts">
                {e.artefactos.map((a) => (
                  <li key={a.ruta}><code>{a.ruta}</code><span className="muted small">{CONTRATO_LABEL[a.contrato] ?? a.contrato}</span></li>
                ))}
              </ul>
            ) : <p className="muted small">Métricas de la corrida y su registro.</p>}
            {e.artefactos?.some((a) => a.contrato === 'red') && <p className="small">Al terminar, la corrida se abre con el <b className="tone-cyan">visor 3D</b> y se puede renderizar a video en Gazebo.</p>}
          </div>

          <div className="panel">
            <h3 className="panel__title">Corridas previas</h3>
            {prev == null && <p className="muted small">Cargando…</p>}
            {prev && prev.length === 0 && <p className="muted small">Todavía no hay corridas de esta escena.</p>}
            {prev && prev.length > 0 && (
              <ul className="prevs">
                {prev.map((r) => (
                  <li key={r.run_id}>
                    <a href={href(`/runs/${r.run_id}`)}><code>{r.run_id.slice(0, 20)}</code></a>
                    <StatusBadge status={r.status} />
                    <span className="muted small" title={fmtDate(r.created_at)}>{hace(r.created_at)}</span>
                  </li>
                ))}
              </ul>
            )}
          </div>
        </aside>
      </div>
    </div>
  )
}

