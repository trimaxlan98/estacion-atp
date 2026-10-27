import { RmsHistogram, RmsScatter } from '../charts/CampaignCharts.jsx'
import { nf } from '../format.js'
import { Legend, Panel, Tile, VerdictBanner } from './ui.jsx'
import { C } from '../colors.js'

/** Resultado de una campana Monte Carlo (campaign.json, CONTRATO §4). */
export default function CampaignPanel({ campaign }) {
  if (!campaign) return null
  const obj = campaign.objective_deg ?? 0.1
  const rms = campaign.rms_deg || []
  const inside = rms.length ? (100 * rms.filter((v) => v <= obj).length) / rms.length : null
  const overs = rms.filter((v) => v > obj).length
  return (
    <div className="stack">
      <VerdictBanner
        verdict={campaign.pasa ? 'PASA' : 'NO PASA'}
        text={`p95 ${nf(campaign.p95, 4)}° ${campaign.pasa ? '≤' : '>'} objetivo ${obj}°`}
      />
      <div className="tiles">
        <Tile label="p50" value={nf(campaign.p50, 4)} unit="°" tone="cyan" hint="Mediana del RMS entre pases" />
        <Tile label="p95" value={nf(campaign.p95, 4)} unit="°" tone="amber" hint="Percentil 95 del RMS: el criterio de aceptación" />
        <Tile label="p99" value={nf(campaign.p99, 3)} unit="°" tone={campaign.p99 > obj ? 'red' : 'cyan'} />
        <Tile label="peor caso" value={nf(campaign.peor, 3)} unit="°" tone={campaign.peor > obj ? 'red' : 'cyan'} />
        <Tile
          label="incertidumbre del p95"
          value={`±${nf(campaign.p95_uncertainty_pts, 1)}`}
          unit=" pts"
          tone="gray"
          hint="Incertidumbre del percentil estimado, en puntos de percentil (p95_uncertainty_pts)"
        />
        <Tile label="pases" value={campaign.n} tone="gray" hint={`modo ${campaign.mode}`} />
        <Tile label="dentro de objetivo" value={nf(inside, 1)} unit="%" tone={inside >= 95 ? 'green' : 'red'} hint="Pases con RMS ≤ objetivo (calculado de rms_deg)" />
        <Tile label="tiempo de cómputo" value={nf(campaign.wall_time_s, 1)} unit="s" tone="gray" />
      </div>
      <div className="grid2">
        <Panel
          title="Histograma del RMS por pase"
          sub={`modo ${campaign.mode} · ${overs} de ${rms.length} pases fuera del objetivo`}
          right={<Legend items={[{ color: C.cyan, dash: true, label: 'objetivo' }, { color: C.amber, label: 'p95' }]} />}
        >
          <RmsHistogram values={rms} objective={obj} p95={campaign.p95} />
        </Panel>
        <Panel
          title="RMS contra elevación máxima"
          sub="Los pases casi cenitales cruzan el keyhole: ahí se dispara el error"
        >
          <RmsScatter rms={rms} elMax={campaign.el_max_deg || []} objective={obj} />
        </Panel>
      </div>
    </div>
  )
}
