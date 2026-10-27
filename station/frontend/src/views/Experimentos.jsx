import { useEffect, useMemo, useState } from 'react'
import { createRun, getExperiments } from '../api.js'
import { go } from '../router.js'

const LEVEL_INFO = {
  N1: 'Lazo de un eje sobre rampa',
  N2: 'Pase LEO de 2 ejes con keyhole',
  N3: 'Gemelo URDF + TF2',
  N4: 'Doppler, presupuesto de enlace y Monte Carlo',
  N5: 'Autotrack por conical scan',
  S4: 'Pase real con SGP4',
  S6: 'Red: plan de contactos de varias estaciones',
  E1: 'Segmento espacio: ADCS de un CubeSat',
  M1: 'Misión integrada: un día de operaciones de la red',
  R1: 'Red NTN: constelación, handover y topología',
  R2: 'Red NTN: Doppler y latencia a través del handover',
  K1: 'Seguidor: estrategia de keyhole sobre el cenit',
}

const KIND_LABEL = { loop: 'lazo', campaign: 'campaña', plan: 'plan', adcs: 'adcs', mision: 'misión', proceso: 'proceso' }

/** Valida un valor contra su especificacion; devuelve mensaje o null. */
export function check(spec, v) {
  if (spec.type === 'enum') return spec.options?.includes(v) ? null : 'Elige una opción'
  if (spec.type === 'number' || spec.type === 'integer') {
    if (v === '' || v == null || isNaN(Number(v))) return 'Valor numérico requerido'
    const n = Number(v)
    if (spec.min != null && n < spec.min) return `Mínimo ${spec.min}`
    if (spec.max != null && n > spec.max) return `Máximo ${spec.max}`
  }
  return null
}

export function Field({ name, spec, value, onChange, error }) {
  const id = `f-${name}`
  const label = spec.label || name
  if (spec.type === 'enum') {
    return (
      <div className="field">
        <label>{label}</label>
        <div className="seg" role="radiogroup" aria-label={label}>
          {(spec.options || []).map((o) => (
            <button
              type="button"
              key={String(o)}
              role="radio"
              aria-checked={value === o}
              className={value === o ? 'is-on' : ''}
              onClick={() => onChange(o)}
            >
              {spec.labels?.[o] ?? String(o)}
            </button>
          ))}
        </div>
      </div>
    )
  }
  if (spec.type === 'boolean') {
    return (
      <div className="field field--check">
        <label htmlFor={id}>
          <input id={id} type="checkbox" checked={!!value} onChange={(e) => onChange(e.target.checked)} /> {label}
        </label>
      </div>
    )
  }
  if (spec.type === 'number' || spec.type === 'integer') {
    const step = spec.type === 'integer' ? 1 : 'any'
    return (
      <div className="field">
        <label htmlFor={id}>
          {label}
          {(spec.min != null || spec.max != null) && (
            <small>{spec.min ?? '−∞'} – {spec.max ?? '∞'}</small>
          )}
        </label>
        <input
          id={id}
          type="number"
          inputMode="decimal"
          value={value}
          min={spec.min}
          max={spec.max}
          step={step}
          onChange={(e) => onChange(e.target.value)}
          aria-invalid={!!error}
          className={error ? 'is-bad' : ''}
        />
        {error && <span className="field__err">{error}</span>}
      </div>
    )
  }
  return (
    <div className="field">
      <label htmlFor={id}>{label}</label>
      <input id={id} type="text" value={value ?? ''} onChange={(e) => onChange(e.target.value)} />
    </div>
  )
}

function ExperimentCard({ exp }) {
  const schema = exp.params_schema || {}
  const [vals, setVals] = useState(() => {
    const v = {}
    for (const k of Object.keys(schema)) v[k] = exp.args?.[k] ?? (schema[k].type === 'enum' ? schema[k].options?.[0] : '')
    return v
  })
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState(null)

  const errors = useMemo(() => {
    const e = {}
    for (const [k, spec] of Object.entries(schema)) {
      const m = check(spec, vals[k])
      if (m) e[k] = m
    }
    return e
  }, [schema, vals])
  const invalid = Object.keys(errors).length > 0

  async function launch(ev) {
    ev.preventDefault()
    if (invalid) return
    setBusy(true)
    setErr(null)
    const args = {}
    for (const [k, spec] of Object.entries(schema)) args[k] = spec.type === 'number' || spec.type === 'integer' ? Number(vals[k]) : vals[k]
    try {
      const r = await createRun(exp.id, args)
      go(`/runs/${r.run_id}`)
    } catch (e) {
      setErr(e.message)
      setBusy(false)
    }
  }

  return (
    <form className="card" onSubmit={launch} noValidate>
      <header>
        <div className="card__tags">
          <span className={`tag tag--${exp.kind}`}>{KIND_LABEL[exp.kind] ?? exp.kind}</span>
          <code className="muted small">{exp.id}</code>
        </div>
        <h3>{exp.title}</h3>
        <p className="card__desc">{exp.description}</p>
      </header>
      <div className="fields">
        {Object.entries(schema).map(([k, spec]) => (
          <Field key={k} name={`${exp.id}-${k}`} spec={spec} value={vals[k]} error={errors[k]} onChange={(v) => setVals((s) => ({ ...s, [k]: v }))} />
        ))}
        {Object.keys(schema).length === 0 && <p className="muted small">Sin parámetros: se lanza con los valores por defecto.</p>}
      </div>
      {err && <div className="alert">{err}</div>}
      <footer>
        <button className="btn btn--primary" type="submit" disabled={busy || invalid}>
          {busy ? 'Lanzando…' : 'Lanzar'}
        </button>
      </footer>
    </form>
  )
}

export default function Experimentos() {
  const [exps, setExps] = useState(null)
  const [error, setError] = useState(null)
  useEffect(() => {
    getExperiments().then(setExps).catch(setError)
  }, [])

  const groups = useMemo(() => {
    const g = {}
    for (const e of exps || []) (g[e.level || '—'] ||= []).push(e)
    return Object.entries(g).sort(([a], [b]) => a.localeCompare(b))
  }, [exps])

  return (
    <div className="page">
      <header className="pagehead">
        <p className="eyebrow">Catálogo</p>
        <h1>Experimentos</h1>
        <p className="muted">Elige un experimento, ajusta sus parámetros y lánzalo: la corrida se abre en vivo.</p>
      </header>
      {error && <div className="alert">No se pudo cargar el catálogo: {error.message}</div>}
      {!exps && !error && <p className="muted">Cargando catálogo…</p>}
      {exps && exps.length === 0 && <div className="empty"><p>El catálogo está vacío.</p></div>}
      {groups.map(([level, list]) => (
        <section key={level} className="level">
          <h2 className="level__title">
            <span className="level__chip">{level}</span>
            <span>{LEVEL_INFO[level] || ''}</span>
          </h2>
          <div className="cards">
            {list.map((e) => <ExperimentCard key={e.id} exp={e} />)}
          </div>
        </section>
      ))}
    </div>
  )
}
