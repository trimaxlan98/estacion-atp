import { useCallback, useEffect, useRef, useState } from 'react'

/** Llama `fn()` ahora y cada `ms`; devuelve {data, error, reload}. No consulta con la pestana oculta. */
export function usePoll(fn, ms = 5000, deps = []) {
  const [data, setData] = useState(null)
  const [error, setError] = useState(null)
  const ref = useRef(fn)
  ref.current = fn
  const load = useCallback(async () => {
    try {
      const d = await ref.current()
      setData(d)
      setError(null)
    } catch (e) {
      setError(e)
    }
  }, [])
  useEffect(() => {
    let dead = false
    const run = () => { if (!dead && !document.hidden) load() }
    load()
    const t = ms ? setInterval(run, ms) : null
    return () => { dead = true; if (t) clearInterval(t) }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ms, load, ...deps])
  return { data, error, reload: load }
}
