import { useEffect, useMemo, useState } from 'react'
import { getRun, getTelemetry } from '../api.js'
import Chart from '../charts/Chart.jsx'
import { envelope, pick } from '../charts/util.js'
import { Legend, Panel } from '../components/ui.jsx'
import { DEFAULT_OBJECTIVE } from '../components/RunCharts.jsx'
import { C } from '../colors.js'
import { argsTxt, fmtDate, nf } from '../format.js'
import { href } from '../router.js'

async function loadOne(id) {
  const run = await getRun(id)
  const n = run.metrics?.n
  const every = n > 6000 ? Math.ceil(n / 6000) : 1
  const tel = await getTelemetry(id, every)
  return { run, tel: Array.isArray(tel) ? tel : [] }
}

const label = (r) => `${r.title || r.experiment_id} · ${fmtDate(r.created_at)}`

const ROWS = [
  ['RMS apuntamiento (°)', (m) => m.pointing?.rms, 4, -1],
  ['p95 (°)', (m) => m.pointing?.p95, 4, -1],
  ['Máximo (°)', (m) => m.pointing?.max, 3, -1],
  ['Dentro del objetivo (%)', (m) => m.pointing?.within_objective_pct, 1, 1],
  ['Margen mínimo (dB)', (m) => m.link?.margin_min_db, 2, 1],
  ['Enlace cerrado (%)', (m) => m.link?.closed_pct, 1, 1],
  ['Doppler máx (kHz)', (m) => (m.link?.doppler_max_hz != null ? m.link.doppler_max_hz / 1000 : null), 2, 0],
]

export default function Comparar({ a, b }) {
  const [data, setData] = useState(null)
  const [error, setError] = useState(null)
  useEffect(() => {
    setData(null); setError(null)
    Promise.all([loadOne(a), loadOne(b)]).then(setData).catch(setError)
  }, [a, b])

  const objective = data?.[0].run.metrics?.pointing?.objective_deg ?? DEFAULT_OBJECTIVE
  const A = C.cyan, B = C.pale

  // ventana de ~3 s de tiempo simulado: el ruido muestra a muestra tapa la comparacion
  const win = (tel) => {
    const dt = tel.length > 1 ? (tel[tel.length - 1].t - tel[0].t) / (tel.length - 1) : 0.1
    return Math.max(1, Math.round(3 / Math.max(dt, 1e-3)))
  }
  const env = (tel) => {
    const { x, y } = pick(tel, 't', 'e_pt')
    return envelope(x, y, win(tel))
  }
  const errSeries = useMemo(() => {
    if (!data) return []
    return [
      { name: 'A', ...env(data[0].tel), color: A, unit: '°', digits: 4, limit: { y: objective, bad: 'above', color: C.red } },
      { name: 'B', ...env(data[1].tel), color: B, dash: '6 3', unit: '°', digits: 4, limit: { y: objective, bad: 'above', color: C.red } },
    ]
  }, [data, objective])
  const marSeries = useMemo(() => {
    if (!data) return []
    return [
      { name: 'A', ...pick(data[0].tel, 't', 'margin_db'), color: C.green, unit: 'dB', digits: 2 },
      { name: 'B', ...pick(data[1].tel, 't', 'margin_db'), color: B, dash: '6 3', unit: 'dB', digits: 2 },
    ]
  }, [data])
  const bands = useMemo(() => [{ y0: 0, y1: objective, color: C.cyan, opacity: 0.13 }], [objective])
  const eLines = useMemo(() => [{ y: objective, color: C.cyan, label: `objetivo ${objective}°` }], [objective])
  const mLines = useMemo(() => [{ y: 0, color: C.red, label: '0 dB' }], [])
  const inc = useMemo(() => [objective * 1.5], [objective])
  const zero = useMemo(() => [0], [])

  if (error) return <div className="page"><div className="alert">No se pudieron cargar las corridas: {error.message}</div><a className="btn" href={href('/biblioteca')}>Volver</a></div>
  if (!data) return <div className="page"><p className="muted">Cargando corridas…</p></div>
  const [ra, rb] = [data[0].run, data[1].run]

  return (
    <div className="page">
      <header className="pagehead pagehead--row">
        <div>
          <p className="eyebrow">Comparación</p>
          <h1>Dos corridas superpuestas</h1>
        </div>
        <a className="btn btn--ghost" href={href('/biblioteca')}>Biblioteca</a>
      </header>

      <div className="vs">
        <a className="vs__item vs__item--a" href={href(`/runs/${ra.run_id}`)}><b>A</b> {label(ra)}<br /><code>{ra.run_id}</code><span className="muted small">{argsTxt(ra.args)}</span></a>
        <a className="vs__item vs__item--b" href={href(`/runs/${rb.run_id}`)}><b>B</b> {label(rb)}<br /><code>{rb.run_id}</code><span className="muted small">{argsTxt(rb.args)}</span></a>
      </div>

      <Panel
        title="Error de apuntamiento"
        sub="Máximo del error por ventana de ~3 s (no se pierde ningún pico). Banda de 0.1° siempre dibujada; lo que se sale, en rojo"
        right={<Legend items={[{ color: A, label: 'A' }, { color: B, dash: true, label: 'B' }, { color: C.red, label: 'fuera de objetivo' }]} />}
      >
        <Chart title="Error de apuntamiento: A contra B" series={errSeries} bands={bands} hlines={eLines} yMin={0} yInclude={inc} yLabel="error (deg)" height={240} />
      </Panel>
      <Panel
        title="Margen de enlace"
        right={<Legend items={[{ color: C.green, label: 'A' }, { color: B, dash: true, label: 'B' }]} />}
      >
        <Chart title="Margen de enlace: A contra B" series={marSeries} hlines={mLines} yInclude={zero} yLabel="margen (dB)" height={220} />
      </Panel>

      {ra.metrics && rb.metrics && (
        <Panel title="Métricas lado a lado">
          <div className="tablewrap">
            <table className="table table--compact">
              <thead><tr><th>Métrica</th><th className="num">A</th><th className="num">B</th><th className="num">B − A</th></tr></thead>
              <tbody>
                {ROWS.map(([name, f, d, better]) => {
                  const va = f(ra.metrics), vb = f(rb.metrics)
                  const diff = va != null && vb != null ? vb - va : null
                  const tone = diff == null || diff === 0 || better === 0 ? '' : diff * better > 0 ? 'tone-green' : 'tone-red'
                  return (
                    <tr key={name}>
                      <td>{name}</td>
                      <td className="num tone-cyan">{nf(va, d)}</td>
                      <td className="num tone-cyan">{nf(vb, d)}</td>
                      <td className={`num ${tone}`}>{diff == null ? '—' : (diff > 0 ? '+' : '') + diff.toFixed(d)}</td>
                    </tr>
                  )
                })}
                <tr>
                  <td>Veredicto</td>
                  {[ra, rb].map((r, i) => (
                    <td key={i} className="num"><span className={`vchip ${r.metrics.verdict === 'CUMPLE' ? 'vchip--ok' : 'vchip--bad'}`}>{r.metrics.verdict}</span></td>
                  ))}
                  <td />
                </tr>
              </tbody>
            </table>
          </div>
        </Panel>
      )}
    </div>
  )
}
