import { useEffect, useMemo, useState } from 'react'
import { getEscenas } from '../api.js'
import { FilterChips, PageHead, Thumb } from '../components/common.jsx'
import { AREA_LABEL, secciones } from '../format.js'
import { href } from '../router.js'

const KIND_LABEL = { loop: 'lazo', campaign: 'campaña', plan: 'plan', adcs: 'adcs', mision: 'misión', proceso: 'proceso' }
const RECURSO_LABEL = { ligero: 'ligero', pesado: 'pesado', gazebo: 'gazebo', manim: 'manim' }

export function EscenaCard({ e }) {
  const { resumen } = secciones(e.description)
  return (
    <article className="scene" data-testid="escena-card" data-id={e.id}>
      <a className="scene__img" href={href(`/escenas/${e.id}`)} tabIndex={-1} aria-hidden="true">
        <Thumb src={e.miniatura} area={e.area} seed={e.id} alt="" />
        <span className={`tag tag--${e.kind} scene__kind`}>{KIND_LABEL[e.kind] ?? e.kind}</span>
      </a>
      <div className="scene__body">
        <div className="scene__meta">
          <span className="level__chip">{e.level}</span>
          <span className="tag tag--area">{AREA_LABEL[e.area] ?? e.area}</span>
        </div>
        <h3><a href={href(`/escenas/${e.id}`)}>{e.title}</a></h3>
        <p className="scene__desc">{resumen}</p>
      </div>
      <footer className="scene__foot">
        <span className="muted small"><code>{e.id}</code> · {RECURSO_LABEL[e.recursos] ?? e.recursos ?? '—'}{e.corridas != null ? ` · ${e.corridas} corrida${e.corridas === 1 ? '' : 's'}` : ''}</span>
        <a className="btn btn--primary btn--sm" href={href(`/escenas/${e.id}`)}>Lanzar</a>
      </footer>
    </article>
  )
}

export default function Escenas() {
  const [escenas, setEscenas] = useState(null)
  const [error, setError] = useState(null)
  const [area, setArea] = useState('')
  const [q, setQ] = useState('')

  useEffect(() => {
    getEscenas().then(setEscenas).catch(setError)
  }, [])

  const { areas } = useMemo(() => {
    const a = {}
    for (const e of escenas || []) {
      a[e.area] = (a[e.area] || 0) + 1
    }
    return {
      areas: Object.entries(a).sort().map(([value, n]) => ({ value, n, label: AREA_LABEL[value] ?? value })),
    }
  }, [escenas])

  const shown = useMemo(() => {
    const s = q.trim().toLowerCase()
    return (escenas || []).filter((e) => (!area || e.area === area) && (!s || `${e.title} ${e.id} ${e.description}`.toLowerCase().includes(s)))
  }, [escenas, area, q])

  return (
    <div className="page">
      <PageHead eyebrow="Catálogo" title="Escenas">
        Cada escena es una pregunta que el motor ROS sabe responder. Elige una, ajusta sus parámetros y lánzala; el resultado se ve en vivo, se puede renderizar a video y queda guardado con su procedencia.
      </PageHead>

      <div className="filters">
        <FilterChips label="Área" options={areas} value={area} onChange={setArea} />
        <label className="search">
          <span className="sr-only">Buscar escenas</span>
          <input type="search" placeholder="Buscar escena…" value={q} onChange={(e) => setQ(e.target.value)} />
        </label>
      </div>

      {error && <div className="alert">No se pudo cargar el catálogo de escenas: {error.message}</div>}
      {!escenas && !error && <p className="muted">Cargando escenas…</p>}
      {escenas && shown.length === 0 && <div className="empty"><p>Ninguna escena coincide con los filtros.</p><button type="button" className="btn" onClick={() => { setArea(''); setQ('') }}>Quitar filtros</button></div>}
      <div className="scenes" data-testid="escenas-grid">
        {shown.map((e) => <EscenaCard key={e.id} e={e} />)}
      </div>
      {escenas && <p className="muted small">{shown.length} de {escenas.length} escenas</p>}
    </div>
  )
}
