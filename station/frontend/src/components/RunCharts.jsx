import { useMemo } from 'react'
import Chart from '../charts/Chart.jsx'
import PolarSky from '../charts/PolarSky.jsx'
import { pick } from '../charts/util.js'
import { C } from '../colors.js'
import { kHz, nf } from '../format.js'
import { Legend, Panel, Tile } from './ui.jsx'

export const DEFAULT_OBJECTIVE = 0.1

/** Cielo polar de una corrida. */
export function SkyPanel({ samples, mask, objective }) {
  return (
    <Panel
      title="Cielo"
      sub="Acimut alrededor, elevación radial (90° al centro)"
      right={<Legend items={[{ color: C.amber, label: 'referencia' }, { color: C.cyan, label: 'antena' }, { color: C.red, label: 'fuera de objetivo' }]} />}
    >
      <PolarSky samples={samples} mask={mask} objective={objective} />
    </Panel>
  )
}

/** Cifras en vivo (ultima muestra). */
export function LiveFigures({ last, status, objective }) {
  const has = !!last
  const tone = (ok) => (ok ? 'green' : 'red')
  return (
    <Panel title="Cifras en vivo" sub={has ? `t = ${nf(last.t, 1)} s de tiempo simulado` : 'Sin muestras todavía'}>
      <div className="tiles tiles--compact">
        <Tile label="elevación" value={has ? nf(last.el, 2) : '—'} unit="°" tone="cyan" hint="Posición real de la antena" />
        <Tile label="acimut" value={has ? nf(last.az, 2) : '—'} unit="°" tone="cyan" hint="Posición real de la antena" />
        <Tile label="rango" value={has ? nf(last.range_km, 0) : '—'} unit=" km" tone="amber" hint="Distancia al satélite" />
        <Tile label="error de apunt." value={has ? nf(last.e_pt, 3) : '—'} unit="°" tone={has && last.e_pt > objective ? 'red' : 'cyan'} />
        <Tile label="Doppler" value={has ? kHz(last.doppler_hz) : '—'} unit=" kHz" tone="cyan" />
        <Tile label="Eb/N0" value={has ? nf(last.ebn0_db, 1) : '—'} unit=" dB" tone="cyan" />
        <Tile label="margen" value={has ? nf(last.margin_db, 1) : '—'} unit=" dB" tone={has ? tone(last.margin_db >= 0) : 'gray'} />
        <Tile label="enlace" value={has ? (last.link_closed ? 'CERRADO' : 'ABIERTO') : '—'} tone={has ? tone(last.link_closed) : 'gray'} />
      </div>
      {status && <p className="muted small">Estado de la corrida: <b>{status}</b></p>}
    </Panel>
  )
}

/** Las tres graficas temporales: error, Doppler, margen. */
export function TimeCharts({ samples, objective = DEFAULT_OBJECTIVE }) {
  const err = useMemo(() => pick(samples, 't', 'e_pt'), [samples])
  const dop = useMemo(() => pick(samples, 't', 'doppler_hz', (v) => v / 1000), [samples])
  const rate = useMemo(() => pick(samples, 't', 'doppler_rate_hz_s'), [samples])
  const mar = useMemo(() => pick(samples, 't', 'margin_db'), [samples])

  const errSeries = useMemo(
    () => [{ name: 'error de apuntamiento', ...err, color: C.cyan, unit: '°', digits: 4, limit: { y: objective, bad: 'above', color: C.red } }],
    [err, objective],
  )
  const dopSeries = useMemo(() => [{ name: 'Doppler', ...dop, color: C.cyan, unit: 'kHz', digits: 2 }], [dop])
  const rateSeries = useMemo(() => [{ name: 'tasa', ...rate, color: C.cyan, unit: 'Hz/s', digits: 1 }], [rate])
  const marSeries = useMemo(
    () => [{ name: 'margen', ...mar, color: C.green, unit: 'dB', digits: 2, limit: { y: 0, bad: 'below', color: C.red } }],
    [mar],
  )
  const errBands = useMemo(() => [{ y0: 0, y1: objective, color: C.cyan, opacity: 0.13 }], [objective])
  const errLines = useMemo(() => [{ y: objective, color: C.cyan, label: `objetivo ${objective}°` }], [objective])
  const zero = useMemo(() => [{ y: 0, color: C.furniture, dash: '2 0' }], [])
  const zeroDb = useMemo(() => [{ y: 0, color: C.red, label: '0 dB: enlace cerrado por encima' }], [])
  const zeroInc = useMemo(() => [0], [])
  const inc = useMemo(() => [objective * 1.5], [objective])

  return (
    <>
      <Panel
        title="Error de apuntamiento"
        sub="Banda del objetivo siempre visible; lo que se sale, en rojo"
        right={<Legend items={[{ color: C.cyan, label: 'error' }, { color: C.red, label: 'fuera de objetivo' }]} />}
      >
        <Chart title="Error de apuntamiento contra el tiempo" series={errSeries} bands={errBands} hlines={errLines} yMin={0} yInclude={inc} yLabel="error (deg)" yDigits={3} height={210} />
      </Panel>
      <Panel title="Doppler (curva S)" sub="Desplazamiento en kHz y su tasa de cambio" right={<Legend items={[{ color: C.cyan, label: 'Doppler / tasa' }]} />}>
        <Chart title="Doppler contra el tiempo" series={dopSeries} hlines={zero} yLabel="Doppler (kHz)" height={180} />
        <Chart title="Tasa de Doppler contra el tiempo" series={rateSeries} hlines={zero} yLabel="Hz/s" height={110} xLabel="" />
      </Panel>
      <Panel
        title="Margen de enlace"
        sub="Por debajo de 0 dB el enlace se abre"
        right={<Legend items={[{ color: C.green, label: 'enlace cerrado' }, { color: C.red, label: 'fuera de presupuesto' }]} />}
      >
        <Chart title="Margen de enlace contra el tiempo" series={marSeries} hlines={zeroDb} yInclude={zeroInc} yLabel="margen (dB)" yDigits={2} height={190} />
      </Panel>
    </>
  )
}
