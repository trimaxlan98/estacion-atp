import { Suspense, lazy, useEffect, useState } from 'react'
import { cancelRun, downloadCsv, getEscenas } from '../api.js'
import AdcsPanel from '../components/AdcsPanel.jsx'
import CampaignPanel from '../components/CampaignPanel.jsx'
import MetricsCard from '../components/MetricsCard.jsx'
import MisionPanel from '../components/MisionPanel.jsx'
import PlanPanel from '../components/PlanPanel.jsx'
import ProcesoPanel from '../components/ProcesoPanel.jsx'
import RenderPanel from '../components/RenderPanel.jsx'
import { DEFAULT_OBJECTIVE, LiveFigures, SkyPanel, TimeCharts } from '../components/RunCharts.jsx'
import { LogPanel, Panel, StatusBadge } from '../components/ui.jsx'
import { argsTxt, fmtDate, isActive } from '../format.js'
import { href } from '../router.js'
import { useRunLive } from '../useRunLive.js'

// three.js va en su propio chunk: solo se descarga al abrir un lazo de seguimiento
const Mount3D = lazy(() => import('../components/Mount3D.jsx'))

const EYEBROW = { loop: 'Corrida', campaign: 'Campaña', plan: 'Plan de contactos', adcs: 'ADCS', mision: 'Misión', proceso: 'Escena de proceso' }

export default function RunView({ runId }) {
  const { run, status, samples, log, error, live, kind } = useRunLive(runId)
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState(null)
  const [escena, setEscena] = useState(null) // entrada del catalogo de escenas (artefactos, contrato)
  useEffect(() => {
    if (!run?.experiment_id) return undefined
    let dead = false
    getEscenas().then((l) => !dead && setEscena(l.find((e) => e.id === run.experiment_id) || null)).catch(() => {})
    return () => { dead = true }
  }, [run?.experiment_id])

  if (error) {
    return (
      <div className="page">
        <div className="empty">
          <h2>No se pudo abrir la corrida</h2>
          <p className="muted">{error.status === 404 ? `No existe la corrida ${runId}.` : error.message}</p>
          <a className="btn" href={href('/biblioteca')}>Ir a la biblioteca</a>
        </div>
      </div>
    )
  }
  if (!run || !kind) return <div className="page"><p className="muted">Cargando corrida…</p></div>

  const metrics = run.metrics
  const objective = metrics?.pointing?.objective_deg ?? DEFAULT_OBJECTIVE
  const mask = Number(run.args?.mask ?? 5)
  const last = samples.length ? samples[samples.length - 1] : null
  const active = isActive(status, run.finished_at)
  const esRover = (run.artefactos ?? escena?.artefactos ?? []).some((a) => a.contrato === 'rover_telemetria')

  async function onCancel() {
    setBusy(true)
    try { await cancelRun(runId) } catch (e) { setErr(e.message) } finally { setBusy(false) }
  }

  const waiting = (txt) => (
    <div className="panel">
      <p className="muted">{active ? txt : 'Sin resultados.'}</p>
      {active && <div className="progress"><i /></div>}
    </div>
  )

  return (
    <div className="page">
      <header className="runhead">
        <div>
          <p className="eyebrow">{EYEBROW[kind] ?? 'Corrida'} · <code>{run.run_id}</code></p>
          <h1>{run.title || run.experiment_id}</h1>
          <p className="muted small">
            {run.experiment_id} · lanzada {fmtDate(run.created_at)}
            {run.finished_at ? ` · terminó ${fmtDate(run.finished_at)}` : ''}
            {' · '}
            {argsTxt(run.args, ['sats', 'estaciones'])}
          </p>
        </div>
        <div className="runhead__actions">
          <StatusBadge status={status} />
          {live && <span className="live-dot" title="Recibiendo la SSE en vivo">EN VIVO</span>}
          {active && <button className="btn btn--danger" disabled={busy} onClick={onCancel}>Cancelar</button>}
          {(kind === 'loop' || kind === 'adcs') && samples.length > 0 && <button className="btn" onClick={() => downloadCsv(runId)}>Exportar CSV</button>}
          <a className="btn btn--ghost" href={href('/biblioteca')}>Biblioteca</a>
        </div>
      </header>
      {err && <div className="alert">{err}</div>}
      {status === 'esperando_carga' && (
        <div className="note note--amber" data-testid="nota-espera">
          Esperando carga: hay cupo en la clase «{run.clase ?? 'pesado'}», pero la carga del host supera el umbral de entrada. El gobernador la reevalúa cada 10 s y la corrida arranca sola.
        </div>
      )}
      {status === 'interrumpida' && (
        <div className="note note--amber" data-testid="nota-interrumpida">
          {run.finished_at
            ? `Interrumpida: una señal externa (p. ej. el vigilante del host, por sobrecarga) la cortó y ya no quedan reintentos${run.reintentos ? ` (${run.reintentos} hechos)` : ''}. Vuelve a lanzarla cuando baje la carga.`
            : `Interrumpida por una señal externa (p. ej. el vigilante del host). El gobernador la reintenta con espera creciente${run.reintentos ? `; reintento ${run.reintentos} en curso` : ''}.`}
        </div>
      )}
      {(status === 'error' || status === 'timeout' || status === 'cancelled') && (
        <div className="alert">La corrida terminó con estado «{status}». Revisa el registro.</div>
      )}

      {kind === 'campaign' && (
        <>
          {run.campaign ? <CampaignPanel campaign={run.campaign} /> : waiting('La campaña está corriendo; los resultados aparecen al terminar.')}
          <LogPanel lines={log} open={active} />
        </>
      )}

      {kind === 'plan' && (
        <>
          {run.plan ? <PlanPanel plan={run.plan} /> : waiting('El planificador está calculando ventanas y planes (SGP4 + óptimo); el plan aparece al terminar.')}
          <LogPanel lines={log} open={active} />
        </>
      )}

      {kind === 'mision' && (
        <>
          {run.mision ? <MisionPanel runId={runId} informe={run.mision} /> : waiting('La misión está en curso: plan, una corrida por contacto y la OD; el informe aparece al terminar (el registro muestra cada contacto).')}
          <LogPanel lines={log} open={active} />
        </>
      )}

      {kind === 'adcs' && (
        <>
          {samples.length > 0 || metrics ? <AdcsPanel metrics={metrics} samples={samples} live={live} /> : waiting('El escenario ADCS está arrancando.')}
          <LogPanel lines={log} />
        </>
      )}

      {kind === 'proceso' && (metrics ? <ProcesoPanel metrics={metrics} /> : waiting('La escena está corriendo; las métricas aparecen al terminar (el registro muestra el avance).'))}

      {kind === 'proceso' && esRover && status === 'done' && (
        <div data-testid="rover-visor-enlace">
          <Panel title="Visor 3D del rover" sub="El recorrido de esta corrida sobre su terreno, en el navegador" right={<a className="btn btn--sm btn--primary" href={href(`/runs/${encodeURIComponent(runId)}/rover`)}>Ver en 3D</a>}>
            <p className="muted small">Rover de seis ruedas, trayectoria, y en el panel el deslizamiento por rueda, la potencia y los atascos (marcas rojas en la línea de tiempo).</p>
          </Panel>
        </div>
      )}

      {kind === 'loop' && (
        <>
          {metrics && <MetricsCard metrics={metrics} />}
          <div className="live">
            <div className="live__side">
              <SkyPanel samples={samples} mask={mask} objective={objective} />
              <Suspense fallback={<Panel title="Montura 3D"><p className="muted">Cargando la escena 3D…</p></Panel>}>
                <Mount3D samples={samples} live={live} />
              </Suspense>
              <LiveFigures last={last} status={status} objective={objective} />
            </div>
            <div className="live__main">
              <TimeCharts samples={samples} objective={objective} />
            </div>
          </div>
          <LogPanel lines={log} />
        </>
      )}

      {status === 'done' && <RenderPanel run={run} escena={escena} />}

      {kind === 'proceso' && <LogPanel lines={log} open={active} />}
    </div>
  )
}
