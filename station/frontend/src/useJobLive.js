import { useEffect, useRef, useState } from 'react'
import { getRender, openRenderStream } from './api.js'
import { jobActivo } from './format.js'

const MAX_LOG = 1500

/**
 * Detalle de un trabajo de render con su flujo SSE (`estado`, `log`, `progreso`, `hecho`).
 * Un trabajo terminado solo pide el detalle; uno activo se engancha a la SSE y vuelve a pedir el detalle al terminar.
 */
export function useJobLive(jobId) {
  const [job, setJob] = useState(null)
  const [log, setLog] = useState([])
  const [progreso, setProgreso] = useState(null)
  const [estado, setEstado] = useState(null)
  const [error, setError] = useState(null)
  const buf = useRef([])

  useEffect(() => {
    let dead = false
    let close = () => {}
    let timer = null
    buf.current = []
    setJob(null); setLog([]); setProgreso(null); setEstado(null); setError(null)
    if (!jobId) return undefined
    const flush = () => { timer = null; if (!dead) setLog(buf.current.slice()) }
    const schedule = () => { if (!timer) timer = setTimeout(flush, 120) }

    ;(async () => {
      try {
        const j = await getRender(jobId)
        if (dead) return
        setJob(j)
        setEstado(j.estado)
        setProgreso(j.progreso ?? null)
        buf.current = Array.isArray(j.log) ? j.log.slice(-MAX_LOG) : typeof j.log === 'string' ? j.log.split('\n').filter(Boolean).slice(-MAX_LOG) : []
        flush()
        if (!jobActivo(j.estado)) return
        close = openRenderStream(jobId, {
          onEstado: ({ estado: e }) => {
            setEstado(e)
            if (e && !jobActivo(e)) getRender(jobId).then((x) => !dead && setJob(x)).catch(() => {})
          },
          onLog: ({ line }) => {
            if (line == null) return
            if (buf.current.length && buf.current[buf.current.length - 1] === line && /^PROGRESO/.test(line)) return
            buf.current.push(line)
            if (buf.current.length > MAX_LOG) buf.current.splice(0, buf.current.length - MAX_LOG)
            schedule()
          },
          onProgreso: (d) => {
            const v = typeof d === 'number' ? d : d?.progreso ?? d?.valor ?? d?.p
            if (typeof v === 'number') setProgreso(v)
          },
          // `hecho` cierra la SSE con el estado FINAL (hecho | error | cancelado): no siempre es un exito
          onHecho: (d) => {
            const fin = d?.estado ?? 'hecho'
            setEstado(fin)
            if (fin === 'hecho') setProgreso(1)
            getRender(jobId).then((x) => !dead && setJob(x)).catch(() => {})
          },
          onError: () => {},
        })
      } catch (e) {
        if (!dead) setError(e)
      }
    })()
    return () => { dead = true; close(); clearTimeout(timer) }
  }, [jobId])

  return { job, log, progreso, estado: estado ?? job?.estado, error }
}
