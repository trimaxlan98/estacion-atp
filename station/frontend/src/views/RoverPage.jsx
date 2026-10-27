import { Suspense, lazy, useEffect, useMemo, useState } from 'react'
import { getRun, getTelemetry } from '../api.js'
import { PageHead } from '../components/common.jsx'
import { PLANETAS, TERRENOS, mockRover, normalizarTerreno, parsearEstados, prepararRover, terrenoDeEscena } from '../lib/rover3d.js'
import { href } from '../router.js'

const Rover3D = lazy(() => import('../components/Rover3D.jsx'))

/**
 * #/rover — visor 3D del rover. Sin corrida: recorrido de muestra (modo mock, datos sintéticos) o un
 * telemetry.jsonl que se carga a mano (formato /rover/estado, CONTRATO §18).
 * #/runs/:id/rover — una corrida `kind: rover`: su telemetry.jsonl y los args planeta/terreno/semilla.
 */
export default function RoverPage({ runId }) {
  const [planeta, setPlaneta] = useState('marte')
  const [terreno, setTerreno] = useState('rocoso')
  const [semilla, setSemilla] = useState(1)
  const [archivo, setArchivo] = useState(null) // {nombre, estados}
  const [run, setRun] = useState(null) // {estados, args}
  const [err, setErr] = useState(null)

  useEffect(() => {
    if (!runId) return undefined
    let dead = false
    setRun(null); setErr(null)
    Promise.all([getRun(runId), getTelemetry(runId, 1)])
      .then(([r, tel]) => { if (!dead) setRun({ r, estados: tel }) })
      .catch((e) => { if (!dead) setErr(e) })
    return () => { dead = true }
  }, [runId])

  const datos = useMemo(() => {
    try {
      if (runId) {
        if (!run) return null
        const a = run.r?.args || {}
        const pl = PLANETAS[a.planeta] ? a.planeta : 'marte'
        const ter = normalizarTerreno(a.terreno_datos) || terrenoDeEscena(a.terreno, a.semilla ?? 1, a.terreno_params)
        const d = prepararRover({ estados: run.estados, terreno: ter, planeta: pl })
        return d.n ? d : { vacio: true }
      }
      const base = mockRover({ planeta, terreno, semilla: Number(semilla) || 1 })
      const estados = archivo ? archivo.estados : base.estados
      const d = prepararRover({ estados, terreno: base.terreno, planeta })
      return d.n ? d : { vacio: true }
    } catch (e) {
      return { error: e }
    }
  }, [runId, run, planeta, terreno, semilla, archivo])

  const cargar = async (ev) => {
    const file = ev.target.files?.[0]
    if (!file) return
    const estados = parsearEstados(await file.text())
    setArchivo(estados.length ? { nombre: file.name, estados } : null)
  }

  return (
    <div className="page">
      <PageHead eyebrow="Rover planetario" title="Visor 3D del rover">
        Terreno, rover de seis ruedas rocker-bogie y trayectoria a partir del JSON de <code>/rover/estado</code> (CONTRATO §18).
        Es reproducción cinemática: dibuja lo que midió la corrida, no calcula física. Las articulaciones del balancín se aproximan desde el terreno.
        El relieve es el del visor, salvo en <code>pendiente</code> y <code>escalones</code>, donde sigue la fórmula de la escena (el rover queda sobre el suelo
        simulado); en los demás tipos es aproximado y la trayectoria y las alturas del rover son las medidas.
      </PageHead>
      {runId ? (
        <p className="muted small"><a href={href(`/runs/${encodeURIComponent(runId)}`)}>← Corrida</a> · <code>{runId}</code></p>
      ) : (
        <div className="rover3d__ctl">
          <label>Planeta{' '}
            <select value={planeta} onChange={(e) => setPlaneta(e.target.value)} aria-label="Planeta">
              {Object.entries(PLANETAS).map(([k, p]) => <option key={k} value={k}>{p.nombre} (g {p.g})</option>)}
            </select>
          </label>
          <label>Terreno{' '}
            <select value={terreno} onChange={(e) => setTerreno(e.target.value)} aria-label="Terreno">
              {TERRENOS.map((k) => <option key={k} value={k}>{k}</option>)}
            </select>
          </label>
          <label>Semilla{' '}
            <input type="number" min="0" value={semilla} onChange={(e) => setSemilla(e.target.value)} aria-label="Semilla" style={{ width: 70 }} />
          </label>
          <label className="btn btn--sm btn--ghost">Cargar telemetry.jsonl
            <input type="file" accept=".jsonl,.json,.txt" onChange={cargar} hidden aria-label="Cargar telemetría" />
          </label>
          <span className="muted small">{archivo ? `${archivo.nombre} · ${archivo.estados.length} muestras` : 'recorrido de muestra (datos sintéticos)'}</span>
        </div>
      )}
      {err && <div className="alert">No se pudo cargar la corrida: {err.message}</div>}
      {datos?.error && <div className="alert">No se pudo preparar el visor: {String(datos.error.message || datos.error)}</div>}
      {datos?.vacio && <div className="empty"><p>La corrida no trae muestras de estado (x, y, t) para dibujar.</p></div>}
      {!datos && !err && <p className="muted">Cargando la corrida…</p>}
      {datos?.n > 0 && (
        <Suspense fallback={<p className="muted">Cargando el visor 3D…</p>}>
          <Rover3D datos={datos} etiqueta={runId ? (run?.r?.experiment_id ?? '') : archivo ? 'archivo' : 'mock'} />
        </Suspense>
      )}
    </div>
  )
}
