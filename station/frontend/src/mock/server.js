// Backend simulado en memoria. Habla el contrato (CONTRATO §6) sin red:
// api.js le entrega (metodo, ruta, cuerpo) y openStream() emula la SSE.
import { MOCK_EXPERIMENTS } from './catalog.js'
import { generatePass, computeMetrics, generateCampaign } from './passModel.js'
import { createAppMock } from './app.js'
// corridas REALES guardadas (tools/mock_fixtures.py): plan.json de s6-plan-24h y metrics.json +
// telemetry.jsonl (1 de cada 8 filas) de e1-adcs-funcube, en el formato exacto del contrato
import planFx from './fixtures/plan.json'
import adcsFx from './fixtures/adcs.json'

// `interrumpida` es final solo con finished_at (sin el espera su reintento); el mock la siembra siempre final
const TERMINAL = new Set(['done', 'error', 'cancelled', 'timeout', 'interrumpida'])

class HttpError extends Error {
  constructor(status, detail) {
    super(detail)
    this.status = status
  }
}

function pad(n, w = 2) {
  return String(n).padStart(w, '0')
}
function mkId(date, i) {
  const d = date
  const hex = (0xa1b2c3 + i * 0x2f0d17).toString(16).slice(-6).padStart(6, '0')
  return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}-${hex}`
}
function randId() {
  const d = new Date()
  const hex = Math.floor(Math.random() * 0xffffff).toString(16).padStart(6, '0')
  return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}-${hex}`
}

/** metrics de una corrida de plan (CONTRATO 3.2): el espejo de station/backend/station/plans.py. */
export function planMetrics(plan) {
  const nombre = plan.planes.optimo ? 'optimo' : 'codicioso_peso'
  const ref = plan.planes[nombre]
  const cod = plan.planes.codicioso_peso
  const out = { kind: 'plan', plan: nombre, algoritmo: ref.algoritmo, optimo_probado: !!ref.optimo_probado, ...ref.resumen }
  if (cod) out.codicioso = { contactos: cod.resumen.contactos, minutos: cod.resumen.minutos, gb: cod.resumen.gb }
  if (plan.brecha_codicioso_pct != null) out.brecha_codicioso_pct = plan.brecha_codicioso_pct
  out.ventanas = plan.n_ventanas
  out.disponible = { contactos: plan.disponible.contactos, minutos: plan.disponible.minutos, gb: plan.disponible.gb }
  out.horas = plan.horas
  out.inicio_utc = plan.inicio_utc
  out.banda = plan.banda
  out.estaciones = [...(plan.estaciones_lista || Object.keys(plan.estaciones))]
  out.satelites = plan.satelites.map((x) => x.nombre)
  out.aporte_estacion = Object.fromEntries(
    Object.entries(plan.aporte_estacion || {}).map(([e, a]) => [e, Object.fromEntries(['gb_con', 'gb_sin', 'delta_gb', 'delta_pct', 'contactos_con', 'contactos_sin'].map((k) => [k, a[k]]))]),
  )
  return out
}

function subsample(rows, every) {
  if (!every || every <= 1) return rows
  const out = []
  for (let i = 0; i < rows.length; i += every) out.push(rows[i])
  return out
}

export function createMock() {
  const runs = new Map() // run_id -> record
  const listeners = new Map() // run_id -> Set<handlers>
  const timers = new Map()

  const exp = (id) => MOCK_EXPERIMENTS.find((e) => e.id === id)
  const appApi = createAppMock({ runs }) // API nueva de la app (docs/APP.md §5)

  function emit(runId, ev, data) {
    const set = listeners.get(runId)
    if (!set) return
    for (const h of [...set]) h[ev]?.(data)
  }

  function publicRun(r, detail = false) {
    const o = {
      run_id: r.run_id,
      experiment_id: r.experiment_id,
      title: r.title,
      status: r.status,
      created_at: r.created_at,
      finished_at: r.finished_at,
      args: r.args,
      metrics: r.metrics ?? null,
    }
    if (detail) {
      o.kind = exp(r.experiment_id)?.kind ?? 'loop'
      o.artefactos = (exp(r.experiment_id)?.artefactos ?? []).map((a) => ({ ...a, existe: r.status === 'done' }))
      o.clase = exp(r.experiment_id)?.recursos ?? 'pesado'
      o.reintentos = r.reintentos ?? 0
      o.campaign = r.campaign ?? null
      o.plan = r.plan ?? null
      o.ros_domain_id = r.ros_domain_id
    }
    return o
  }

  function rowsOf(r) {
    if (!r._rows) r._rows = exp(r.experiment_id).kind === 'adcs' ? adcsFx.rows : generatePass(r.args)
    return r._rows
  }
  const kindOf = (r) => exp(r.experiment_id).kind
  const noSeries = (r) => ['campaign', 'plan', 'proceso'].includes(kindOf(r))

  // ---------- siembra: corridas ya terminadas para que la biblioteca no este vacia
  function seed() {
    const now = Date.now()
    const defs = [
      ['n4a-pase-oblicuo-s', { el_max: 30, mask: 5, time_scale: 10, band: 'S' }, 26 * 3600e3],
      ['n4b-pase-cenital-x', { el_max: 85, mask: 5, time_scale: 10, band: 'X' }, 20 * 3600e3],
      ['n2-pase-leo-2ejes', { el_max: 60, mask: 5, time_scale: 10 }, 7 * 3600e3],
      ['n4a-pase-oblicuo-s', { el_max: 62, mask: 5, time_scale: 10, band: 'S' }, 3 * 3600e3],
      ['n4c-campana-montecarlo', { mode: 'lazo', n: 300 }, 2 * 3600e3],
      ['n4c-campana-montecarlo', { mode: 'lazo_abierto', n: 300 }, 1.5 * 3600e3],
      ['s6-plan-24h', {}, 5 * 3600e3],
      ['e1-adcs-funcube', { w0_deg_s: 3, lead_s: 3000, start_utc: '2026-10-02T21:20:00' }, 4 * 3600e3],
    ]
    defs.forEach(([eid, args, ago], i) => {
      const e = exp(eid)
      const created = new Date(now - ago)
      const r = {
        run_id: mkId(created, i),
        experiment_id: eid,
        title: e.title,
        status: 'done',
        created_at: created.toISOString(),
        finished_at: new Date(created.getTime() + 60e3).toISOString(),
        args: { ...e.args, ...args },
        ros_domain_id: 40 + i,
        log: ['[mock] corrida sembrada'],
      }
      if (e.kind === 'campaign') {
        r.campaign = generateCampaign(r.args)
        r.metrics = null
      } else if (e.kind === 'plan') {
        r.plan = planFx
        r.metrics = planMetrics(planFx)
      } else if (e.kind === 'adcs') {
        r.metrics = adcsFx.metrics
      } else if (e.kind === 'proceso') {
        r.metrics = appApi.metricas(r)
      } else {
        r.metrics = computeMetrics(rowsOf(r), r.args)
      }
      runs.set(r.run_id, r)
    })
  }
  seed()
  // una corrida interrumpida por el vigilante (final: tiene finished_at) y otra esperando carga (arranca sola a los 20 s)
  {
    const now = Date.now()
    const e = exp('n2-pase-leo-2ejes')
    const base = (ago, status, extra) => {
      const created = new Date(now - ago)
      const r = { run_id: mkId(created, 90 + runs.size), experiment_id: e.id, title: e.title, status, created_at: created.toISOString(), finished_at: null, args: { ...e.args }, ros_domain_id: 120 + runs.size, log: [], ...extra }
      runs.set(r.run_id, r)
      return r
    }
    const i = base(55 * 60e3, 'interrumpida', { reintentos: 2, finished_at: new Date(now - 50 * 60e3).toISOString() })
    i.log = ['ros2 launch atp_loop.launch.py el_max:=60', '[station] el proceso murió por la señal 15 (vigilante del host)', '[station] reintento 1/2 en 30 s', '[station] reintento 2/2 en 60 s', '[station] sin reintentos restantes: interrumpida']
    const w = base(40e3, 'esperando_carga')
    w.log = ['[gobernador] clase pesado: carga 1 min 6.4 > umbral 6.0: espero, reevalúo cada 10 s']
    setTimeout(() => { if (w.status === 'esperando_carga') startRun(w) }, 20000)
  }
  appApi.seed()

  // ---------- ciclo de vida de una corrida lanzada
  function startRun(r) {
    const e = exp(r.experiment_id)
    r.status = 'queued'
    r.log = [`[station] corrida ${r.run_id} en cola`]
    emit(r.run_id, 'status', { status: 'queued' })
    const t1 = setTimeout(() => {
      r.status = 'running'
      pushLog(r, `ros2 launch ${e.launch} ${Object.entries(r.args).map(([k, v]) => `${k}:=${v}`).join(' ')}`)
      pushLog(r, `[reference_generator] pase generado, ROS_DOMAIN_ID=${r.ros_domain_id}`)
      emit(r.run_id, 'status', { status: 'running' })
      if (e.kind === 'campaign') runCampaign(r)
      else if (e.kind === 'plan') runPlan(r)
      else if (e.kind === 'adcs') runAdcs(r)
      else if (e.kind === 'proceso') runProceso(r)
      else runLoop(r)
    }, 700)
    timers.set(r.run_id, t1)
  }

  function pushLog(r, line) {
    r.log.push(line)
    emit(r.run_id, 'log', { line })
  }

  function finish(r, status) {
    clearInterval(timers.get(r.run_id))
    clearTimeout(timers.get(r.run_id))
    timers.delete(r.run_id)
    r.status = status
    r.finished_at = new Date().toISOString()
    emit(r.run_id, 'status', { status })
  }

  function runLoop(r) {
    const rows = rowsOf(r)
    r._cursor = 0
    const scale = Math.max(1, Number(r.args.time_scale ?? 10))
    // 10 Hz de tiempo simulado x time_scale muestras por segundo real; tick cada 50 ms
    const perTick = Math.max(1, Math.round(scale * 10 * 0.05))
    const id = setInterval(() => {
      for (let k = 0; k < perTick && r._cursor < rows.length; k++) {
        emit(r.run_id, 'sample', rows[r._cursor++])
        if (r._cursor % 500 === 0) pushLog(r, `[recorder] ${r._cursor} muestras (t=${rows[r._cursor - 1].t.toFixed(0)} s)`)
      }
      if (r._cursor >= rows.length) {
        r.metrics = computeMetrics(rows, r.args)
        pushLog(r, `RESULTADO ${r.metrics.verdict} rms=${r.metrics.pointing.rms} p95=${r.metrics.pointing.p95} margen_min=${r.metrics.link.margin_min_db}`)
        finish(r, 'done')
        emit(r.run_id, 'done', { metrics: r.metrics })
      }
    }, 50)
    timers.set(r.run_id, id)
  }

  function runPlan(r) {
    const lines = ['[planificar] TLE de data/tle, 4 estaciones x 8 satélites', '[planificar] ventanas candidatas: 173', '[planificar] óptimo (HiGHS): 349.58 GB, probado', '[planificar] plan sin la estación quitada: 224.98 GB']
    let i = 0
    const id = setInterval(() => {
      if (i < lines.length) return pushLog(r, lines[i++])
      r.plan = planFx
      r.metrics = planMetrics(planFx)
      pushLog(r, `-> runs/${r.run_id}/plan.json, resumen.txt`)
      finish(r, 'done')
      emit(r.run_id, 'done', { metrics: r.metrics })
    }, 450)
    timers.set(r.run_id, id)
  }

  function runProceso(r) {
    const lines = [`$ ${exp(r.experiment_id).launch} --salida runs --id ${r.run_id}`, '[proceso] escena iniciada', '[proceso] metricas.json escrito']
    let i = 0
    const id = setInterval(() => {
      if (i < lines.length) return pushLog(r, lines[i++])
      r.metrics = appApi.metricas(r)
      finish(r, 'done')
      emit(r.run_id, 'done', { metrics: r.metrics })
    }, 700)
    timers.set(r.run_id, id)
  }

  function runAdcs(r) {
    const rows = rowsOf(r)
    r._cursor = 0
    const id = setInterval(() => {
      for (let k = 0; k < 14 && r._cursor < rows.length; k++) emit(r.run_id, 'sample', rows[r._cursor++])
      if (r._cursor % 98 === 0) pushLog(r, `[adcs_recorder] t = ${rows[r._cursor - 1].t.toFixed(0)} s`)
      if (r._cursor >= rows.length) {
        r.metrics = adcsFx.metrics
        pushLog(r, 'RESUMEN veredicto: ' + r.metrics.verdict)
        finish(r, 'done')
        emit(r.run_id, 'done', { metrics: r.metrics })
      }
    }, 60)
    timers.set(r.run_id, id)
  }

  function runCampaign(r) {
    const n = Number(r.args.n ?? 300)
    const camp = generateCampaign(r.args)
    let i = 0
    const step = Math.max(1, Math.round(n / 40))
    const id = setInterval(() => {
      i = Math.min(n, i + step)
      pushLog(r, `[campaign_server] feedback ${i}/${n} pases`)
      if (i >= n) {
        r.campaign = camp
        pushLog(r, `RESULTADO campana p50=${camp.p50} p95=${camp.p95} p99=${camp.p99} peor=${camp.peor} ${camp.pasa ? 'PASA' : 'NO PASA'}`)
        finish(r, 'done')
        emit(r.run_id, 'done', { metrics: null })
      }
    }, 150)
    timers.set(r.run_id, id)
  }

  // ---------- enrutador REST
  async function handle(method, path, body) {
    const u = new URL(path, 'http://mock.local')
    const p = u.pathname
    const q = u.searchParams
    let m

    if (method === 'GET' && p === '/api/health') return { ok: true, ros_distro: 'jazzy (mock)', active_runs: [...runs.values()].filter((r) => !TERMINAL.has(r.status)).length, active_renders: appApi.activeRenders() }
    if (method === 'GET' && p === '/api/experiments') return MOCK_EXPERIMENTS
    if (method === 'GET' && (m = p.match(/^\/api\/experiments\/([^/]+)$/))) {
      const e = exp(m[1])
      if (!e) throw new HttpError(404, 'experimento no encontrado')
      return e
    }
    if (method === 'POST' && p === '/api/runs') {
      const e = exp(body?.experiment_id)
      if (!e) throw new HttpError(404, 'experimento no encontrado')
      const run = {
        run_id: randId(),
        experiment_id: e.id,
        title: e.title,
        status: 'queued',
        created_at: new Date().toISOString(),
        finished_at: null,
        args: { ...e.args, ...(body.args || {}) },
        ros_domain_id: 60 + runs.size,
        metrics: null,
        log: [],
      }
      runs.set(run.run_id, run)
      startRun(run)
      return { run_id: run.run_id, status: 'queued', _status: 201 }
    }
    if (method === 'GET' && p === '/api/runs') {
      const eid = q.get('experiment_id')
      const limit = Number(q.get('limit') || 100)
      return [...runs.values()]
        .filter((r) => !eid || r.experiment_id === eid)
        .sort((a, b) => b.created_at.localeCompare(a.created_at))
        .slice(0, limit)
        .map((r) => publicRun(r))
    }
    const nuevo = appApi.handle(method, p, q, body)
    if (nuevo !== undefined) return nuevo
    if ((m = p.match(/^\/api\/runs\/([^/]+)(?:\/(.+))?$/))) {
      const r = runs.get(m[1])
      if (!r) throw new HttpError(404, 'corrida no encontrada')
      const sub = m[2]
      if (!sub && method === 'GET') return publicRun(r, true)
      if (!sub && method === 'DELETE') {
        if (!TERMINAL.has(r.status)) throw new HttpError(409, 'la corrida esta activa')
        runs.delete(r.run_id)
        return { deleted: r.run_id }
      }
      if (sub === 'cancel' && method === 'POST') {
        if (!TERMINAL.has(r.status)) {
          pushLog(r, '[station] cancelada por el usuario')
          finish(r, 'cancelled')
        }
        return publicRun(r)
      }
      if (sub === 'telemetry' && method === 'GET') {
        const every = Math.max(1, Number(q.get('every') || 1))
        if (noSeries(r)) return []
        const all = rowsOf(r)
        const upto = r.status === 'done' || r.status === 'cancelled' ? (r._cursor ?? all.length) : (r._cursor ?? 0)
        return subsample(all.slice(0, r.metrics ? all.length : upto), every)
      }
      if (sub === 'log' && method === 'GET') return { _text: r.log.join('\n') + '\n' }
      if (sub === 'export.csv' && method === 'GET') {
        const rows = noSeries(r) ? [] : rowsOf(r).slice(0, r._cursor ?? undefined)
        const cols = Object.keys(rows[0] || {})
        return { _text: [cols.join(','), ...rows.map((x) => cols.map((c) => [].concat(x[c] ?? '').join(' ')).join(','))].join('\n') + '\n' }
      }
    }
    throw new HttpError(404, `ruta no soportada por el mock: ${method} ${p}`)
  }

  function openStream(runId, handlers) {
    const r = runs.get(runId)
    if (!r) {
      setTimeout(() => handlers.onError?.(new Error('404')), 0)
      return () => {}
    }
    const h = {
      status: (d) => handlers.onStatus?.(d),
      sample: (d) => handlers.onSample?.(d),
      log: (d) => handlers.onLog?.(d),
      done: (d) => handlers.onDone?.(d),
    }
    if (!listeners.has(runId)) listeners.set(runId, new Set())
    listeners.get(runId).add(h)
    setTimeout(() => {
      handlers.onStatus?.({ status: r.status })
      if (r.status === 'done') handlers.onDone?.({ metrics: r.metrics })
    }, 0)
    return () => listeners.get(runId)?.delete(h)
  }

  return { handle, openStream, openRenderStream: appApi.openRenderStream, mediaUrl: appApi.mediaUrl, HttpError }
}
