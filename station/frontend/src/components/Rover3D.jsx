import { useCallback, useEffect, useRef, useState } from 'react'
import logo from '../assets/logo-cian.png'
import { fmtReloj } from '../format.js'
import { PLANETAS } from '../lib/rover3d.js'
import { VELOCIDADES, createRoverEngine } from './rover3dEngine.js'

const f = (v, d = 1) => (v === undefined || v === null || !Number.isFinite(v) ? '—' : v.toFixed(d))

/** Visor 3D del rover: terreno, rover rocker-bogie y trayectoria. `datos` sale de prepararRover(). */
export default function Rover3D({ datos, full = false, etiqueta = '' }) {
  const host = useRef(null)
  const eng = useRef(null)
  const wasPlaying = useRef(false)
  const [hud, setHud] = useState(null)
  const [speed, setSpeed] = useState(2)
  const [noGL, setNoGL] = useState(null)
  const pl = PLANETAS[datos.planeta] || PLANETAS.marte

  useEffect(() => {
    let e
    try {
      e = createRoverEngine({ host: host.current, datos, onHud: setHud })
    } catch (err) {
      setNoGL(err)
      return undefined
    }
    eng.current = e
    e.setSpeed(2)
    setSpeed(2)
    return () => { e.dispose(); eng.current = null }
  }, [datos])

  const playing = !!hud?.playing
  const toggle = useCallback(() => {
    const e = eng.current
    if (!e) return
    if (e.state.playing) e.pause()
    else e.play()
  }, [])
  const onKey = (ev) => {
    const e = eng.current
    if (!e || ev.target.tagName === 'SELECT') return
    if (ev.key === ' ' || ev.key === 'k') { ev.preventDefault(); toggle() }
    else if (ev.key === 'ArrowRight') { ev.preventDefault(); e.seek(e.state.t + (ev.shiftKey ? 30 : 5)) }
    else if (ev.key === 'ArrowLeft') { ev.preventDefault(); e.seek(e.state.t - (ev.shiftKey ? 30 : 5)) }
  }

  const t = hud?.t ?? datos.t0
  const m = hud?.m
  const modo = hud?.modo ?? 'seguir'
  return (
    <div className={`visor3d rover3d${full ? ' visor3d--full' : ''}`} data-testid="rover3d" tabIndex={0} onKeyDown={onKey}>
      <div className="visor3d__stage">
        <div className="visor3d__host" ref={host} data-testid="rover3d-canvas" />
        {noGL ? (
          <div className="visor3d__nogl empty"><p className="muted">WebGL no está disponible en este navegador: el visor 3D no se puede dibujar.</p></div>
        ) : (
          <>
            <div className="visor3d__brand">
              <img src={logo} alt="Co.De Aerospace" />
              <div>
                <b>ROVER · {pl.nombre.toUpperCase()}</b>
                <span>g = {pl.g} m/s² · terreno {datos.terreno.tipo}{datos.terreno.exacto === true ? ' (de la escena)' : datos.terreno.exacto === false ? ' (aproximado)' : ''}{etiqueta ? ` · ${etiqueta}` : ''}</span>
              </div>
            </div>
            <div className="visor3d__views" role="group" aria-label="Cámara">
              <button type="button" className={`btn btn--sm btn--ghost${modo === 'seguir' ? ' is-on' : ''}`} onClick={() => { eng.current?.setModo('seguir'); eng.current?.vista('seguir') }}>Seguir</button>
              <button type="button" className="btn btn--sm btn--ghost" onClick={() => eng.current?.vista('cerca')}>Cerca</button>
              <button type="button" className="btn btn--sm btn--ghost" onClick={() => eng.current?.vista('cenit')}>Cenit</button>
              <button type="button" className={`btn btn--sm btn--ghost${modo === 'libre' ? ' is-on' : ''}`} onClick={() => eng.current?.vista('mapa')}>Mapa</button>
            </div>
            <div className="visor3d__status rover3d__status" data-testid="rover3d-status">
              <div><small>POSICIÓN x / y</small><b className="num">{f(m?.x)} / {f(m?.y)}</b><em>m · z {f(m?.z, 2)}</em></div>
              <div><small>RUMBO</small><b className="num">{f(m?.rumbo, 0)}°</b><em>cab {f(m?.cabeceo)}° · alab {f(m?.alabeo)}°</em></div>
              <div><small>VELOCIDAD</small><b className="num tone-cyan">{f(m?.v, 2)}</b><em>m/s</em></div>
              <div><small>RECORRIDO</small><b className="num">{f(hud?.dist)}</b><em>de {f(datos.dist)} m</em></div>
              <div><small>PENDIENTE</small><b className="num tone-amber">{f(m?.pendiente)}°</b><em>máx {f(datos.pendMax)}°</em></div>
              {datos.tiene?.energia && <div><small>ENERGÍA</small><b className="num">{f(m?.energia, 2)}</b><em>Wh</em></div>}
              {datos.tiene?.potencia && <div><small>POTENCIA</small><b className="num">{f(m?.potencia, 0)}</b><em>W</em></div>}
              {/* 0.5 = umbral de «seguro» de los estudios del rover (deslizamiento medio <= 0.5) */}
              {datos.tiene?.desliz && <div><small>DESLIZAMIENTO</small><b className={`num${m?.deslizMedio > 0.5 ? ' tone-amber' : ''}`}>{f(m?.deslizMedio, 2)}</b><em title="Deslizamiento de la rueda que más desliza">máx {f(m?.deslizMax, 2)}</em></div>}
              {m?.atascado && <div><small>ESTADO</small><b className="num tone-red" role="status">ATASCADO</b><em>sin avance</em></div>}
            </div>
          </>
        )}
      </div>
      <div className="visor3d__bar">
        <button type="button" className="btn btn--primary btn--sm visor3d__play" onClick={toggle} aria-label={playing ? 'Pausa' : 'Reproducir'} disabled={!!noGL}>
          {playing ? '❚❚ Pausa' : '▶ Reproducir'}
        </button>
        <div className="visor3d__track">
          <input
            type="range"
            aria-label="Instante de la corrida"
            min={datos.t0}
            max={datos.t1}
            step={0.05}
            value={t}
            disabled={!!noGL}
            onPointerDown={() => { wasPlaying.current = !!eng.current?.state.playing; eng.current?.pause() }}
            onPointerUp={() => { if (wasPlaying.current) eng.current?.play() }}
            onChange={(e) => eng.current?.seek(Number(e.target.value))}
          />
          {datos.atascos?.length > 0 && (
            <div className="visor3d__marks" aria-hidden="true" data-testid="rover3d-marcas">
              {datos.atascos.map((a, i) => (
                <i key={i} className="mk mk--lost" style={{ left: `${(100 * (a.t - datos.t0)) / Math.max(datos.t1 - datos.t0, 1e-9)}%` }}
                  title={`${fmtReloj(a.t - datos.t0)} · atasco en (${f(a.x)}, ${f(a.y)}) m`} />
              ))}
            </div>
          )}
        </div>
        <span className="visor3d__time num" data-testid="rover3d-time">{fmtReloj(t - datos.t0)} / {fmtReloj(datos.t1 - datos.t0)}</span>
        <label className="visor3d__speed">
          <span className="muted small">Velocidad</span>
          <select value={speed} onChange={(e) => { setSpeed(Number(e.target.value)); eng.current?.setSpeed(Number(e.target.value)) }} aria-label="Velocidad de reproducción">
            {VELOCIDADES.map((v) => <option key={v} value={v}>×{v}</option>)}
          </select>
        </label>
        <span className="visor3d__fps muted small num">{hud?.fps ? `${hud.fps} fps` : ''}</span>
      </div>
    </div>
  )
}
