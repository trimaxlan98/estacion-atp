import { useEffect, useRef, useState } from 'react'
import { getExperiments, getLog, getRun, getTelemetry, openStream } from './api.js'
import { isActive, kindOfRun } from './format.js'

const MAX_LOG = 3000
const FLUSH_MS = 150

/**
 * Estado de una corrida: detalle + telemetria historica + flujo SSE en vivo.
 *
 * - Terminada: pide la telemetria (submuestreada si metrics.n es grande).
 * - Activa: pide lo ya grabado y se engancha a la SSE; las muestras repetidas
 *   (por reconexion o por el solape historial/stream) se descartan por `t`.
 * - El volcado a React se limita a ~7 Hz para no congelar con 100 muestras/s.
 */
export function useRunLive(runId) {
  const [run, setRun] = useState(null)
  const [kind, setKind] = useState(null)
  const [status, setStatus] = useState(null)
  const [samples, setSamples] = useState([])
  const [log, setLog] = useState([])
  const [error, setError] = useState(null)
  const [live, setLive] = useState(false)
  const buf = useRef([])
  const logBuf = useRef([])

  useEffect(() => {
    let dead = false
    let closeStream = () => {}
    let timer = null
    let poll = null
    buf.current = []
    logBuf.current = []
    setRun(null); setStatus(null); setSamples([]); setLog([]); setError(null); setLive(false); setKind(null)

    const flush = () => {
      timer = null
      if (dead) return
      setSamples(buf.current.slice())
      setLog(logBuf.current.slice())
    }
    const schedule = () => {
      if (!timer) timer = setTimeout(flush, FLUSH_MS)
    }
    const refresh = async () => {
      const r = await getRun(runId)
      if (dead) return r
      setRun(r)
      setStatus(r.status)
      return r
    }

    ;(async () => {
      try {
        let r = await refresh()
        if (dead) return
        const exps = await getExperiments().catch(() => [])
        const map = Object.fromEntries((exps || []).map((e) => [e.id, e]))
        const k = kindOfRun(r, map)
        setKind(k)

        try {
          const text = await getLog(runId)
          logBuf.current = text ? text.split('\n').filter(Boolean).slice(-MAX_LOG) : []
        } catch { /* el log puede no existir aun */ }

        if (k === 'loop' || k === 'adcs') {
          const n = r.metrics?.n
          const every = !isActive(r.status, r.finished_at) && n > 6000 ? Math.ceil(n / 6000) : 1
          try {
            const tel = await getTelemetry(runId, every)
            if (Array.isArray(tel)) buf.current = tel
          } catch { /* aun sin telemetria */ }
        }
        flush()
        if (dead) return

        if (isActive(r.status, r.finished_at)) {
          setLive(true)
          let lastT = buf.current.length ? buf.current[buf.current.length - 1].t : -Infinity
          closeStream = openStream(runId, {
            onStatus: ({ status: s }) => {
              setStatus(s)
              if (s !== 'queued' && s !== 'running' && s !== 'esperando_carga') {
                refresh().catch(() => {}).finally(() => setLive(false))
              }
            },
            onSample: (s) => {
              if (s.t == null || s.t <= lastT) return
              lastT = s.t
              buf.current.push(s)
              schedule()
            },
            onLog: ({ line }) => {
              logBuf.current.push(line)
              if (logBuf.current.length > MAX_LOG) logBuf.current.splice(0, logBuf.current.length - MAX_LOG)
              schedule()
            },
            onDone: async () => {
              flush()
              setLive(false)
              try { await refresh() } catch { /* ya tenemos lo demas */ }
              try { logBuf.current = (await getLog(runId)).split('\n').filter(Boolean).slice(-MAX_LOG); flush() } catch { /* ok */ }
            },
            onError: () => {
              // sin SSE: sondeo hasta que termine
              setLive(false)
              poll = setInterval(async () => {
                try {
                  const rr = await refresh()
                  if (!isActive(rr.status, rr.finished_at)) { clearInterval(poll); flush() }
                } catch { /* reintenta */ }
              }, 2500)
            },
          })
        }
      } catch (e) {
        if (!dead) setError(e)
      }
    })()

    return () => {
      dead = true
      closeStream()
      clearTimeout(timer)
      clearInterval(poll)
    }
  }, [runId])

  return { run, status, samples, log, error, live, kind, isCampaign: kind === 'campaign' }
}
