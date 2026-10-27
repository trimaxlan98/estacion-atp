import { useEffect, useState } from 'react'

// Enrutado por hash (#/runs/ID): el backend sirve dist/ en "/" sin necesidad
// de reglas de reescritura para rutas profundas.
function parse() {
  const h = (location.hash || '#/').replace(/^#/, '')
  const [path, query = ''] = h.split('?')
  return { path: path || '/', query: new URLSearchParams(query) }
}

export function useRoute() {
  const [r, setR] = useState(parse)
  useEffect(() => {
    const f = () => setR(parse())
    window.addEventListener('hashchange', f)
    return () => window.removeEventListener('hashchange', f)
  }, [])
  return r
}

export const go = (path) => {
  location.hash = '#' + path
}
export const href = (path) => '#' + path
