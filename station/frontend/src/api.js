// Capa de acceso a la API de la Estacion (CONTRATO §6). Rutas relativas /api.
// Si el backend no responde (o VITE_MOCK=1 / ?mock=1) se usa el mock en memoria.

let mock = null // instancia de createMock() cuando esta activo
let backendMode = 'pending' // 'real' | 'mock'

export class ApiError extends Error {
  constructor(status, message) {
    super(message)
    this.status = status
  }
}

const forceMock =
  import.meta.env.VITE_MOCK === '1' ||
  (typeof location !== 'undefined' && new URLSearchParams(location.search).get('mock') === '1')

async function enableMock() {
  if (!mock) {
    const { createMock } = await import('./mock/server.js')
    mock = createMock()
  }
  backendMode = 'mock'
  // solo en modo mock: ganchos para las herramientas de capturas/rendimiento (tools/) que lanzan corridas con args libres
  if (typeof window !== 'undefined') window.__atpMock = { createRun: (e, a) => createRun(e, a), getRun: (i) => getRun(i) }
}

export function getMode() {
  return backendMode
}

/** Decide real o mock: GET /api/health con 2.5 s de plazo; JSON valido y ok:true => real. */
export async function initBackend() {
  if (forceMock) {
    await enableMock()
    return 'mock'
  }
  try {
    const ctl = new AbortController()
    const t = setTimeout(() => ctl.abort(), 2500)
    const res = await fetch('/api/health', { signal: ctl.signal })
    clearTimeout(t)
    const j = await res.json()
    if (res.ok && j && j.ok !== false) {
      backendMode = 'real'
      return 'real'
    }
  } catch {
    /* sin backend */
  }
  await enableMock()
  return 'mock'
}

export async function health() {
  return request('GET', '/api/health')
}

async function request(method, path, body) {
  if (backendMode === 'mock') {
    try {
      const out = await mock.handle(method, path, body)
      if (out && out._status) {
        const { _status, ...rest } = out
        return rest
      }
      return out
    } catch (e) {
      if (e.status) throw new ApiError(e.status, e.message)
      throw e
    }
  }
  const res = await fetch(path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  })
  if (!res.ok) {
    let msg = res.statusText
    try {
      const j = await res.json()
      msg = typeof j.detail === 'string' ? j.detail : j.detail ? JSON.stringify(j.detail) : msg
    } catch {
      /* cuerpo no JSON */
    }
    throw new ApiError(res.status, msg || `HTTP ${res.status}`)
  }
  const ct = res.headers.get('content-type') || ''
  if (res.status === 204) return null
  return ct.includes('json') ? res.json() : res.text()
}

let expCache = null
export async function getExperiments(force = false) {
  if (!expCache || force) expCache = request('GET', '/api/experiments').catch((e) => { expCache = null; throw e })
  return expCache
}

export const listRuns = (params = {}) => {
  const q = new URLSearchParams()
  if (params.experiment_id) q.set('experiment_id', params.experiment_id)
  q.set('limit', params.limit ?? 200)
  return request('GET', `/api/runs?${q}`)
}
export const getRun = (id) => request('GET', `/api/runs/${id}`)
export const createRun = (experiment_id, args) => request('POST', '/api/runs', { experiment_id, args })
export const deleteRun = (id) => request('DELETE', `/api/runs/${id}`)
export const cancelRun = (id) => request('POST', `/api/runs/${id}/cancel`)
export const getContacto = (id, c) => request('GET', `/api/runs/${id}/contactos/${c}`)
export const getContactoTelemetry = (id, c, every = 1) => request('GET', `/api/runs/${id}/contactos/${c}/telemetry?every=${every}`)
export const getTelemetry = (id, every = 1) => request('GET', `/api/runs/${id}/telemetry?every=${every}`)
export async function getLog(id) {
  const r = await request('GET', `/api/runs/${id}/log`)
  if (r && typeof r === 'object' && '_text' in r) return r._text
  return typeof r === 'string' ? r : ''
}

/** Descarga el CSV. En modo real es el enlace directo del backend. */
export async function downloadCsv(id) {
  if (backendMode !== 'mock') {
    const a = document.createElement('a')
    a.href = `/api/runs/${id}/export.csv`
    a.download = `${id}.csv`
    document.body.appendChild(a)
    a.click()
    a.remove()
    return
  }
  const r = await request('GET', `/api/runs/${id}/export.csv`)
  const blob = new Blob([r._text], { type: 'text/csv' })
  const a = document.createElement('a')
  a.href = URL.createObjectURL(blob)
  a.download = `${id}.csv`
  document.body.appendChild(a)
  a.click()
  a.remove()
  setTimeout(() => URL.revokeObjectURL(a.href), 2000)
}

/**
 * Abre el flujo en vivo. handlers: {onStatus, onSample, onLog, onDone, onError}.
 * Devuelve una funcion close().
 */
export function openStream(runId, handlers) {
  if (backendMode === 'mock') return mock.openStream(runId, handlers)
  const es = new EventSource(`/api/runs/${runId}/stream`)
  let errs = 0
  const parse = (fn) => (ev) => {
    errs = 0
    try {
      fn?.(JSON.parse(ev.data))
    } catch {
      /* evento mal formado: se ignora */
    }
  }
  es.addEventListener('status', parse(handlers.onStatus))
  es.addEventListener('sample', parse(handlers.onSample))
  es.addEventListener('log', parse(handlers.onLog))
  es.addEventListener('done', (ev) => {
    parse(handlers.onDone)(ev)
    es.close() // el servidor cierra tras `done`; sin esto EventSource reconectaria
  })
  es.onerror = () => {
    // EventSource reconecta solo; si el servidor cierra y no hay mas, cortamos tras 4 fallos seguidos
    errs++
    if (es.readyState === EventSource.CLOSED || errs >= 4) {
      es.close()
      handlers.onError?.(new Error('stream cerrado'))
    }
  }
  return () => es.close()
}

// ====================================================================================================
// API nueva de la app (docs/APP.md §5): escenas, renders, medios, sistema.
// ====================================================================================================

const qs = (o) => {
  const q = new URLSearchParams()
  for (const [k, v] of Object.entries(o || {})) if (v != null && v !== '') q.set(k, v)
  const s = q.toString()
  return s ? `?${s}` : ''
}

export const getEscenas = () => request('GET', '/api/escenas')
export const getRenderCatalogo = () => request('GET', '/api/renders/catalogo')
export const crearRender = (runId, render_id, args) => request('POST', `/api/runs/${runId}/renders`, { render_id, args })
export const listRenders = (params) => request('GET', `/api/renders${qs(params)}`)
export const getRender = (jobId) => request('GET', `/api/renders/${jobId}`)
export const cancelRender = (jobId) => request('POST', `/api/renders/${jobId}/cancel`)
export const getMedios = (params) => request('GET', `/api/medios${qs(params)}`)
export const getSistema = () => request('GET', '/api/sistema')

/** URL de un archivo de medios. En el backend real la sirve con Range (el <video> puede saltar). */
export function mediaUrl(jobId, ruta) {
  if (backendMode === 'mock') return mock.mediaUrl(jobId, ruta)
  return `/api/medios/${encodeURIComponent(jobId)}/${String(ruta).split('/').map(encodeURIComponent).join('/')}`
}

/**
 * SSE de un trabajo de render. handlers: {onEstado, onLog, onProgreso, onHecho, onError}.
 * Eventos del backend: `estado` {estado}, `log` {line}, `progreso` (numero 0-1 o {progreso}), `hecho`.
 */
export function openRenderStream(jobId, handlers) {
  if (backendMode === 'mock') return mock.openRenderStream(jobId, handlers)
  const es = new EventSource(`/api/renders/${jobId}/stream`)
  let errs = 0
  const parse = (fn) => (ev) => {
    errs = 0
    let d = ev.data
    try { d = JSON.parse(ev.data) } catch { /* texto plano */ }
    fn?.(d)
  }
  es.addEventListener('estado', parse((d) => handlers.onEstado?.(typeof d === 'string' ? { estado: d } : d)))
  es.addEventListener('log', parse((d) => handlers.onLog?.(typeof d === 'string' ? { line: d } : d)))
  es.addEventListener('progreso', parse((d) => handlers.onProgreso?.(typeof d === 'number' ? { progreso: d } : d)))
  es.addEventListener('hecho', (ev) => {
    parse(handlers.onHecho)(ev)
    es.close()
  })
  es.onerror = () => {
    errs++
    if (es.readyState === EventSource.CLOSED || errs >= 4) {
      es.close()
      handlers.onError?.(new Error('stream cerrado'))
    }
  }
  return () => es.close()
}
