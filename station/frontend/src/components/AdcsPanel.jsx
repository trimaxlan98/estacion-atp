import { useMemo } from 'react'
import Chart from '../charts/Chart.jsx'
import ModeStrip, { modesFrom } from '../charts/ModeStrip.jsx'
import { pick } from '../charts/util.js'
import { C, MODE_COLOR, MODE_NAMES } from '../colors.js'
import { nf } from '../format.js'
import { Legend, Panel, Tile, VerdictBanner } from './ui.jsx'

const MODE_LEGEND = MODE_NAMES.map((m) => ({ color: MODE_COLOR[m], label: m }))
const Q_NAMES = ['q_w', 'q_x', 'q_y', 'q_z']
const Q_DASH = [undefined, '7 3', '2 3', '9 3 2 3']
const WHEEL_DASH = [undefined, '7 3', '2 3']
const sec = (v) => (v == null ? '—' : `${nf(v, 0)} s`)

/**
 * ADCS de un CubeSat (CONTRATO 2.1 y 3.1): veredicto, cifras, modos de mision como bandas de color,
 * velocidad angular, cuaterniones, error de apuntamiento contra la banda de umbral y momento de las
 * ruedas contra la saturacion. Sirve tambien en vivo (sin `metrics`: solo la telemetria).
 */
export default function AdcsPanel({ metrics, samples, live }) {
  const m = metrics
  const modes = useMemo(() => (m?.modes?.length ? m.modes : modesFrom(samples)), [m, samples])
  const thr = m?.track?.threshold_deg
  const cap = m?.wheels?.capacity_nms != null ? m.wheels.capacity_nms * 1e3 : 1.77
  const wThr = m?.detumble?.w_threshold_deg_s
  const tEnd = samples.length ? samples[samples.length - 1].t : 0
  const xDomain = useMemo(() => [0, Math.max(1, m?.duration_s ?? tEnd)], [m, tEnd])

  const w = useMemo(() => pick(samples, 't', 'w_deg_s'), [samples])
  const qs = useMemo(() => [0, 1, 2, 3].map((k) => pick(samples, 't', 'q', (v) => v[k])), [samples])
  const err = useMemo(() => pick(samples, 't', 'err_deg'), [samples])
  const know = useMemo(() => pick(samples, 't', 'know_deg'), [samples])
  const sigma = useMemo(() => pick(samples, 't', 'sigma_deg'), [samples])
  const hs = useMemo(() => [0, 1, 2].map((k) => pick(samples, 't', 'h_nms', (v) => Math.abs(v[k]) * 1e3)), [samples])
  const track = useMemo(() => {
    const rows = samples.filter((s) => s.mode === 2 && s.in_pass && s.err_deg != null)
    return pick(rows, 't', 'err_deg')
  }, [samples])

  const xBands = useMemo(
    () => modes.map((md, i) => ({
      x0: md.t0, x1: i + 1 < modes.length ? modes[i + 1].t0 : Math.max(md.t1, xDomain[1]),
      color: MODE_COLOR[md.name], opacity: 0.1, label: md.name,
    })),
    [modes, xDomain],
  )
  const trackSeg = modes.find((md) => md.name === 'TRACK_GS')
  const trackDomain = useMemo(() => {
    if (!track.x.length) return undefined
    return [track.x[0], track.x[track.x.length - 1]]
  }, [track])

  const last = samples.length ? samples[samples.length - 1] : null
  const errMax = thr ? thr * 5 : undefined
  const thrBand = useMemo(() => (thr ? [{ y0: 0, y1: thr, color: C.cyan, opacity: 0.13 }] : []), [thr])
  const thrLine = useMemo(() => (thr ? [{ y: thr, color: C.cyan, label: `umbral TRACK_GS ${nf(thr, 2)}°` }] : []), [thr])

  const ok = m?.verdict === 'CUMPLE'
  return (
    <div className="stack" data-testid="adcs-panel">
      {m ? (
        <section className="panel">
          <header className="panel__head">
            <div>
              <h3 className="panel__title">Resultado del escenario</h3>
              <p className="panel__sub">{m.n} pasos de {m.paso_s} s · {nf(m.duration_s, 0)} s simulados · antena del satélite {m.link?.antenna_m} m en banda {m.link?.band}</p>
            </div>
          </header>
          <VerdictBanner
            verdict={m.verdict}
            text={`p95 TRACK_GS ${nf(m.track?.p95, 3)}° ${ok ? '≤' : '>'} umbral ${nf(thr, 2)}° (pérdida de 1 dB, haz ${nf(m.track?.beamwidth_deg, 1)}°)`}
          />
          <div className="tiles">
            <Tile label="p95 TRACK_GS" value={nf(m.track?.p95, 3)} unit="°" tone={ok ? 'amber' : 'red'} hint="Percentil 95 del error de apuntamiento de la antena durante el pase" />
            <Tile label="RMS TRACK_GS" value={nf(m.track?.rms, 3)} unit="°" tone="cyan" />
            <Tile label="máx TRACK_GS" value={nf(m.track?.max, 3)} unit="°" tone={m.track?.max > thr ? 'red' : 'cyan'} />
            <Tile label="p95 NADIR" value={nf(m.nadir?.p95, 3)} unit="°" tone="amber" hint={`Régimen: tras ${nf(m.nadir?.settle_s, 0)} s de entrar al modo`} />
            <Tile label="detumbling" value={sec(m.detumble?.t_detumble_s)} tone="cyan" hint={`|ω| bajo ${nf(wThr, 3)} °/s (verdad); partió de ${nf(m.detumble?.w0_deg_s, 1)} °/s`} />
            <Tile label="entra a NADIR" value={sec(m.detumble?.t_nadir_s)} tone="violet" hint="El TRIAD necesita el Sol: si el detumbling termina en eclipse, espera" />
            <Tile label="conocimiento" value={nf(m.knowledge?.rms_deg, 3)} unit="°" tone="violet" hint="Error de la actitud estimada contra la verdad, en el eje de la antena" />
            <Tile label="1-σ filtro" value={nf(m.knowledge?.sigma_rms_deg, 3)} unit="°" tone="gray" hint="Lo que el filtro cree que se equivoca; si es mucho menor que el error real, está sobreconfiado" />
            <Tile label="ruedas (% cap.)" value={nf(100 * (m.wheels?.frac ?? NaN), 0)} unit=" %" tone={m.wheels?.frac > 0.8 ? 'red' : 'cyan'} hint={`${nf(1e3 * m.wheels?.h_max_nms, 3)} mNms de ${nf(cap, 2)} mNms`} />
            <Tile label="pérdida apunt." value={nf(m.link?.l_sat_max_db, 3)} unit=" dB" tone="cyan" />
            <Tile label="margen mín" value={nf(m.link?.margin_min_db, 2)} unit=" dB" tone={m.link?.margin_min_db >= 0 ? 'green' : 'red'} />
            <Tile label="enlace cerrado" value={nf(m.link?.closed_pct, 1)} unit=" %" tone={m.link?.closed_pct >= 99.9 ? 'green' : 'red'} />
          </div>
          {m.eclipse_in_pass && <p className="muted small">El pase ocurre dentro del eclipse: sin sensor solar, la actitud sale solo del campo magnético y el giroscopio.</p>}
        </section>
      ) : (
        <Panel title="Cifras en vivo" sub={last ? `t = ${nf(last.t, 0)} s de tiempo simulado` : 'Sin muestras todavía'}>
          <div className="tiles">
            <Tile label="modo" value={last ? MODE_NAMES[last.mode] : '—'} tone={last ? (last.mode === 2 ? 'cyan' : last.mode === 1 ? 'violet' : 'amber') : 'gray'} />
            <Tile label="|ω|" value={last ? nf(last.w_deg_s, 3) : '—'} unit=" °/s" tone="cyan" />
            <Tile label="error de apunt." value={last?.err_deg != null ? nf(last.err_deg, 3) : '—'} unit="°" tone="cyan" />
            <Tile label="eclipse" value={last ? (last.eclipse ? 'SÍ' : 'NO') : '—'} tone="gray" />
          </div>
          {live && <div className="progress"><i /></div>}
        </Panel>
      )}

      <Panel
        title="Modos de misión"
        sub="DETUMBLE (B-dot con magnetopares) → NADIR (PD con ruedas) → TRACK_GS (antena a la estación)"
        right={<Legend items={MODE_LEGEND} />}
      >
        <ModeStrip modes={modes} samples={samples} xDomain={xDomain} />
      </Panel>

      <Panel title="Velocidad angular" sub="|ω| verdadera; el detumbling termina bajo el umbral que fijan las ruedas" right={<Legend items={[{ color: C.cyan, label: '|ω|' }, { color: C.cyan, dash: true, label: 'umbral de detumbling' }]} />}>
        <Chart title="Velocidad angular contra el tiempo" series={[{ name: '|ω|', ...w, color: C.cyan, unit: '°/s', digits: 3 }]} xBands={xBands} xDomain={xDomain}
          hlines={wThr ? [{ y: wThr, color: C.cyan, label: `umbral ${nf(wThr, 2)} °/s` }] : []} yMin={0} yLabel="|ω| (deg/s)" height={170} />
      </Panel>

      <Panel
        title="Error de apuntamiento y de conocimiento"
        sub={`Error de la antena contra su objetivo (nadir o la estación) y de la actitud estimada; banda del umbral siempre visible${errMax ? `, eje recortado a ${nf(errMax, 0)}°` : ''}`}
        right={<Legend items={[{ color: C.cyan, label: 'apuntamiento' }, { color: C.violet, label: 'conocimiento (est − verdad)' }, { color: C.gray, dash: true, label: '1-σ del filtro' }]} />}
      >
        <Chart title="Error de apuntamiento y de conocimiento contra el tiempo"
          series={[
            { name: 'apuntamiento', ...err, color: C.cyan, unit: '°', digits: 3 },
            { name: 'conocimiento', ...know, color: C.violet, unit: '°', digits: 3 },
            { name: '1-σ filtro', ...sigma, color: C.gray, unit: '°', digits: 3, dash: '4 3', width: 1.2 },
          ]}
          xBands={xBands} xDomain={xDomain} bands={thrBand} hlines={thrLine} yMin={0} yMax={errMax} yLabel="error (deg)" height={230} />
      </Panel>

      <Panel
        title="TRACK_GS en el pase"
        sub={trackSeg ? 'Solo con la estación sobre la máscara: lo que supera el umbral, en rojo' : 'El escenario todavía no llega a TRACK_GS'}
        right={<Legend items={[{ color: C.cyan, label: 'error de la antena' }, { color: C.red, label: 'sobre el umbral' }]} />}
      >
        <Chart title="Error de apuntamiento en TRACK_GS contra el tiempo"
          series={[{ name: 'apuntamiento', ...track, color: C.cyan, unit: '°', digits: 3, limit: thr ? { y: thr, bad: 'above', color: C.red } : undefined }]}
          xDomain={trackDomain} bands={thrBand} hlines={thrLine} yMin={0} yInclude={thr ? [thr * 1.15] : []} yLabel="error (deg)" height={190}
          emptyText="Sin muestras de TRACK_GS todavía" />
      </Panel>

      <div className="grid2">
        <Panel title="Cuaterniones" sub="Actitud verdadera cuerpo → inercial (TEME), escalar primero" right={<Legend items={Q_NAMES.map((n, i) => ({ color: C.cyan, dash: i > 0, label: n }))} />}>
          <Chart title="Cuaternión de actitud contra el tiempo"
            series={qs.map((q, k) => ({ name: Q_NAMES[k], ...q, color: C.cyan, dash: Q_DASH[k], unit: '', digits: 3, width: 1.4 }))}
            xBands={xBands} xDomain={xDomain} yMin={-1.05} yMax={1.05} yDigits={2} yLabel="componente" height={210} />
        </Panel>
        <Panel
          title="Momento de las ruedas"
          sub={`|h| de cada rueda contra su saturación (${nf(cap, 2)} mNms)`}
          right={<Legend items={[{ color: C.cyan, label: 'rueda x' }, { color: C.cyan, dash: true, label: 'y' }, { color: C.red, label: 'saturación' }]} />}
        >
          <Chart title="Momento de las ruedas contra el tiempo"
            series={hs.map((h, k) => ({ name: 'rueda ' + 'xyz'[k], ...h, color: C.cyan, dash: WHEEL_DASH[k], unit: 'mNms', digits: 3, limit: { y: cap, bad: 'above', color: C.red } }))}
            xBands={xBands} xDomain={xDomain} bands={[{ y0: 0.8 * cap, y1: cap, color: C.red, opacity: 0.1 }]}
            hlines={[{ y: cap, color: C.red, label: `saturación ${nf(cap, 2)} mNms` }]} yMin={0} yInclude={[cap * 1.05]} yLabel="|h| (mNms)" height={210} />
        </Panel>
      </div>
    </div>
  )
}
