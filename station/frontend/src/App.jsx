import { useEffect, useRef, useState } from 'react'
import { health, initBackend } from './api.js'
import logo from './assets/logo-cian.png'
import { href, useRoute } from './router.js'
import Biblioteca from './views/Biblioteca.jsx'
import Campanas from './views/Campanas.jsx'
import Comparar from './views/Comparar.jsx'
import Escenas from './views/Escenas.jsx'
import Experimentos from './views/Experimentos.jsx'
import Inicio from './views/Inicio.jsx'
import Lanzar from './views/Lanzar.jsx'
import Medios from './views/Medios.jsx'
import Render from './views/Render.jsx'
import Resultados from './views/Resultados.jsx'
import RunView from './views/RunView.jsx'
import RoverPage from './views/RoverPage.jsx'

/** Seccion de la navegacion a la que pertenece una ruta. */
export function seccionDe(path) {
  if (path === '/' || path === '') return 'inicio'
  if (path.startsWith('/escenas')) return 'escenas'
  if (/^\/runs\/[^/]+\/rover$/.test(path)) return 'rover'
  if (path.startsWith('/biblioteca') || path.startsWith('/compare') || path.startsWith('/runs')) return 'corridas'
  if (path.startsWith('/render')) return 'render'
  if (path.startsWith('/medios')) return 'medios'
  if (path.startsWith('/experimentos')) return 'experimentos'
  if (path.startsWith('/campanas')) return 'campanas'
  if (path.startsWith('/plan')) return 'plan'
  if (path.startsWith('/mision')) return 'mision'
  if (path.startsWith('/adcs')) return 'adcs'
  if (path.startsWith('/rover')) return 'rover'
  return 'inicio'
}

const PRINCIPAL = [
  ['inicio', '/', 'Inicio'],
  ['escenas', '/escenas', 'Escenas'],
  ['corridas', '/biblioteca', 'Biblioteca'],
  ['render', '/render', 'Render'],
  ['medios', '/medios', 'Medios'],
]
const ANALISIS = [
  ['experimentos', '/experimentos', 'Experimentos'],
  ['campanas', '/campanas', 'Campañas'],
  ['plan', '/plan', 'Plan de contactos'],
  ['mision', '/mision', 'Misión'],
  ['adcs', '/adcs', 'ADCS'],
  ['rover', '/rover', 'Rover'],
]

function Header({ mode, info, path }) {
  const sec = seccionDe(path)
  const enRun = path.startsWith('/runs')
  return (
    <>
      <div className="top__row">
        <a className="brand" href={href('/')} aria-label="Estación ATP · Estudio, inicio">
          <img src={logo} alt="" />
          <span className="brand__txt"><b>ESTACIÓN ATP</b><small>Estudio · Co.De Aerospace</small></span>
        </a>
        <nav aria-label="Principal">
          {PRINCIPAL.map(([k, p, t]) => <a key={k} href={href(p)} className={sec === k ? 'is-on' : ''} aria-current={sec === k ? 'page' : undefined}>{t}</a>)}
        </nav>
        <div className="top__right">
          {mode === 'mock' ? (
            <span className="chip-status chip-status--mock" title="No hay backend en /api: se usan datos sintéticos que siguen el contrato">MODO MOCK</span>
          ) : info ? (
            <span className="chip-status chip-status--ok" title={`ROS ${info.ros_distro ?? ''}`}>
              <i className="pulse" /> backend en línea{info.active_runs != null ? ` · ${Array.isArray(info.active_runs) ? info.active_runs.length : info.active_runs} activas` : ''}{info.active_renders ? ` · ${info.active_renders} renders` : ''}
            </span>
          ) : (
            <span className="chip-status chip-status--bad">backend sin respuesta</span>
          )}
        </div>
      </div>
      <nav className="subnav" aria-label="Análisis">
        <span className="subnav__label">Análisis</span>
        {ANALISIS.map(([k, p, t]) => <a key={k} href={href(p)} className={sec === k ? 'is-on' : ''} aria-current={sec === k ? 'page' : undefined}>{t}</a>)}
        {enRun && <a className="is-cur" href={href(path)}>Corrida</a>}
      </nav>
    </>
  )
}

export default function App() {
  const [mode, setMode] = useState(null)
  const [info, setInfo] = useState(null)
  const { path } = useRoute()
  const topRef = useRef(null)

  useEffect(() => {
    initBackend().then(setMode)
  }, [])
  useEffect(() => {
    if (mode !== 'real' && mode !== 'mock') return
    let dead = false
    const poll = () => health().then((h) => !dead && setInfo(h)).catch(() => !dead && setInfo(null))
    poll()
    const t = setInterval(poll, 8000)
    return () => { dead = true; clearInterval(t) }
  }, [mode])
  // la altura de la cabecera (sticky) la usa el visor a pantalla completa
  useEffect(() => {
    const el = topRef.current
    if (!el) return undefined
    const set = () => document.documentElement.style.setProperty('--top-h', `${el.offsetHeight}px`)
    set()
    const ro = new ResizeObserver(set)
    ro.observe(el)
    return () => ro.disconnect()
  }, [mode])

  if (!mode) return <div className="boot">Conectando con la Estación…</div>

  let view
  let m
  if ((m = path.match(/^\/runs\/([^/]+)\/rover$/))) view = <RoverPage key={m[1]} runId={decodeURIComponent(m[1])} />
  else if (path === '/rover') view = <RoverPage key="rover" />
  else if ((m = path.match(/^\/runs\/([^/]+)$/))) view = <RunView key={m[1]} runId={decodeURIComponent(m[1])} />
  else if ((m = path.match(/^\/compare\/([^/]+)\/([^/]+)$/))) view = <Comparar a={m[1]} b={m[2]} />
  else if (path.startsWith('/biblioteca')) view = <Biblioteca />
  else if ((m = path.match(/^\/campanas(?:\/([^/]+))?$/))) view = <Campanas runId={m[1]} />
  else if ((m = path.match(/^\/plan(?:\/([^/]+))?$/))) view = <Resultados key="plan" kind="plan" runId={m[1]} />
  else if ((m = path.match(/^\/mision(?:\/([^/]+))?$/))) view = <Resultados key="mision" kind="mision" runId={m[1]} />
  else if ((m = path.match(/^\/adcs(?:\/([^/]+))?$/))) view = <Resultados key="adcs" kind="adcs" runId={m[1]} />
  else if (path.startsWith('/experimentos')) view = <Experimentos />
  else if ((m = path.match(/^\/escenas\/([^/]+)$/))) view = <Lanzar key={m[1]} id={decodeURIComponent(m[1])} />
  else if (path.startsWith('/escenas')) view = <Escenas />
  else if (path.startsWith('/render')) view = <Render />
  else if (path.startsWith('/medios')) view = <Medios />
  else view = <Inicio />

  return (
    <>
      <header className="top" ref={topRef}><Header mode={mode} info={info} path={path} /></header>
      <main>{view}</main>
      <footer className="foot">Estación ATP · Estudio · Co.De Aerospace · {mode === 'mock' ? 'datos sintéticos' : 'datos del backend'}</footer>
    </>
  )
}
