import { fmtDur, kHz, nf } from '../format.js'
import { Tile, VerdictBanner } from './ui.jsx'

/** Metricas finales (metrics.json, CONTRATO §3) y veredicto. */
export default function MetricsCard({ metrics }) {
  if (!metrics) return null
  const p = metrics.pointing || {}
  const l = metrics.link || {}
  const ax = metrics.axes || {}
  const obj = p.objective_deg ?? 0.1
  return (
    <section className="panel">
      <header className="panel__head">
        <div>
          <h3 className="panel__title">Resultado de la corrida</h3>
          <p className="panel__sub">
            {metrics.n} muestras · {fmtDur(metrics.duration_s)} · banda {l.band ?? '—'}
          </p>
        </div>
      </header>
      <VerdictBanner
        verdict={metrics.verdict}
        text={`RMS ${nf(p.rms, 3)}° · p95 ${nf(p.p95, 3)}° · objetivo ${obj}° · margen mín ${nf(l.margin_min_db, 1)} dB`}
      />
      <div className="tiles">
        <Tile label="RMS apuntamiento" value={nf(p.rms, 4)} unit="°" tone={p.rms > obj ? 'red' : 'cyan'} />
        <Tile label="p95" value={nf(p.p95, 4)} unit="°" tone="amber" hint="Percentil 95 del error de apuntamiento" />
        <Tile label="máximo" value={nf(p.max, 3)} unit="°" tone={p.max > obj ? 'red' : 'cyan'} />
        <Tile label="dentro del objetivo" value={nf(p.within_objective_pct, 1)} unit="%" tone={p.within_objective_pct >= 95 ? 'green' : 'red'} />
        <Tile label="RMS acimut" value={nf(ax.az?.rms, 4)} unit="°" tone="cyan" />
        <Tile label="RMS elevación" value={nf(ax.el?.rms, 4)} unit="°" tone="cyan" />
        <Tile label="margen mínimo" value={nf(l.margin_min_db, 2)} unit=" dB" tone={l.margin_min_db >= 0 ? 'green' : 'red'} />
        <Tile label="margen máximo" value={nf(l.margin_max_db, 2)} unit=" dB" tone="cyan" />
        <Tile label="enlace cerrado" value={nf(l.closed_pct, 1)} unit="%" tone={l.closed_pct >= 99.9 ? 'green' : 'red'} />
        <Tile label="Doppler máx" value={kHz(l.doppler_max_hz)} unit=" kHz" tone="cyan" />
        <Tile label="Doppler mín" value={kHz(l.doppler_min_hz)} unit=" kHz" tone="cyan" />
        <Tile label="tasa Doppler máx" value={nf(l.doppler_rate_max_hz_s, 1)} unit=" Hz/s" tone="cyan" />
        <Tile label="pérdida por desapunt." value={nf(l.l_point_max_db, 3)} unit=" dB" tone="cyan" hint="Pérdida máxima por desapuntamiento" />
      </div>
    </section>
  )
}
