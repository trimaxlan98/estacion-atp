import { useMemo, useState } from 'react'
import { getMedios } from '../api.js'
import { FilterChips, PageHead } from '../components/common.jsx'
import { MediaCard, MediaDetail } from '../components/MediaViewer.jsx'
import { usePoll } from '../hooks.js'

const TIPOS = [{ value: 'video', label: 'Video' }, { value: 'imagen', label: 'Imagen' }]

export default function Medios() {
  const [tipo, setTipo] = useState('')
  const [escena, setEscena] = useState('')
  const [sel, setSel] = useState(null)
  const { data, error } = usePoll(() => getMedios({ limit: 200 }), 8000)

  const escenas = useMemo(() => {
    const c = {}
    for (const m of data || []) if (m.escena) c[m.escena] = (c[m.escena] || 0) + 1
    return Object.entries(c).sort().map(([value, n]) => ({ value, n, label: value }))
  }, [data])
  const shown = (data || []).filter((m) => (!tipo || m.tipo === tipo) && (!escena || m.escena === escena))

  return (
    <div className="page">
      <PageHead eyebrow="Evidencia" title="Medios">
        Videos e imágenes que salieron de las corridas, cada uno con el commit, el comando y los argumentos exactos que lo generaron.
      </PageHead>
      <div className="filters">
        <FilterChips label="Tipo" options={TIPOS} value={tipo} onChange={setTipo} all="Todos" />
        <FilterChips label="Escena" options={escenas} value={escena} onChange={setEscena} />
      </div>
      {error && <div className="alert">No se pudo cargar la galería: {error.message}</div>}
      {!data && !error && <p className="muted">Cargando medios…</p>}
      {data && shown.length === 0 && (
        <div className="empty"><p>{data.length ? 'Ningún medio coincide con los filtros.' : 'Todavía no hay medios. Abre una corrida terminada y usa el panel «Renderizar».'}</p></div>
      )}
      <div className="mgrid" data-testid="medios-grid">
        {shown.map((m) => <MediaCard key={m.job_id + m.ruta} m={m} onOpen={setSel} area={m.escena?.startsWith('red') ? 'red' : 'seguidor'} />)}
      </div>
      {data && <p className="muted small">{shown.length} de {data.length} medios</p>}
      {sel && <MediaDetail m={sel} onClose={() => setSel(null)} area={sel.escena?.startsWith('red') ? 'red' : 'seguidor'} />}
    </div>
  )
}
