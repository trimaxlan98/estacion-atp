import { Tile, VerdictBanner } from './ui.jsx'

const HUMAN = (k) => k.replace(/_pct$/, ' (%)').replace(/_s$/, ' (s)').replace(/_deg$/, ' (°)').replace(/_/g, ' ')

/** Cifras de una corrida `kind: proceso` (metricas.json libre): cada numero o texto corto es una cifra. */
export default function ProcesoPanel({ metrics, title = 'Resultado de la corrida' }) {
  if (!metrics) return null
  const veredicto = metrics.verdict ?? metrics.veredicto
  const entries = Object.entries(metrics).filter(([k, v]) => !['verdict', 'veredicto', 'kind'].includes(k) && (typeof v === 'number' || (typeof v === 'string' && v.length <= 24) || typeof v === 'boolean'))
  return (
    <section className="panel">
      <header className="panel__head">
        <div>
          <h3 className="panel__title">{title}</h3>
          <p className="panel__sub">metricas.json de la escena{metrics.kind ? ` · ${metrics.kind}` : ''}</p>
        </div>
      </header>
      {veredicto && <VerdictBanner verdict={veredicto} />}
      <div className="tiles">
        {entries.map(([k, v]) => (
          <Tile key={k} label={HUMAN(k)} value={typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(Math.abs(v) < 1 ? 3 : 1)) : String(v)} tone="cyan" />
        ))}
      </div>
    </section>
  )
}
