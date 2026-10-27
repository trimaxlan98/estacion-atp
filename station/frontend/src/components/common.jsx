import { useState } from 'react'
import { JOB_LABEL, jobActivo } from '../format.js'
import { thumb, thumbDeArea } from '../lib/thumbs.js'

/** Estado de un trabajo de render (en_cola | esperando_carga | corriendo | hecho | error | cancelado | interrumpida). */
export function JobBadge({ estado }) {
  if (!estado) return null
  return (
    <span className={`badge badge--job-${estado}`}>
      {jobActivo(estado) && <i className="pulse" />}
      {JOB_LABEL[estado] || estado}
    </span>
  )
}

/** Imagen con respaldo: si `src` falta o falla, una miniatura procedural de la marca (por area). */
export function Thumb({ src, area = 'red', seed = '', alt = '', className = '' }) {
  const [bad, setBad] = useState(false)
  const url = !src || bad ? thumb(thumbDeArea(area), seed || area) : src
  return <img className={`thumb ${className}`} src={url} alt={alt} loading="lazy" onError={() => setBad(true)} />
}

/** Barra de progreso 0-1 (null = indeterminada). */
export function ProgressBar({ value, tone = 'cyan', label }) {
  const pct = value == null ? null : Math.max(0, Math.min(1, value)) * 100
  return (
    <div className={`pbar pbar--${tone}${pct == null ? ' is-indeterminate' : ''}`} role="progressbar" aria-valuemin={0} aria-valuemax={100} aria-valuenow={pct == null ? undefined : Math.round(pct)} aria-label={label}>
      <i style={pct == null ? undefined : { width: `${pct}%` }} />
    </div>
  )
}

/** Fila de chips de filtro: `options` = [{value, label, n?}], `value` = '' (todos) o el elegido. */
export function FilterChips({ label, options, value, onChange, all = 'Todas' }) {
  return (
    <div className="filter" role="group" aria-label={label}>
      <span className="filter__label">{label}</span>
      <button type="button" className={`fchip${value === '' ? ' is-on' : ''}`} aria-pressed={value === ''} onClick={() => onChange('')}>{all}</button>
      {options.map((o) => (
        <button key={o.value} type="button" className={`fchip${value === o.value ? ' is-on' : ''}`} aria-pressed={value === o.value} onClick={() => onChange(value === o.value ? '' : o.value)}>
          {o.label}{o.n != null && <small>{o.n}</small>}
        </button>
      ))}
    </div>
  )
}

/** Cabecera de pagina con la marca (ojo de pagina, titulo en Rajdhani, texto y acciones). */
export function PageHead({ eyebrow, title, children, actions }) {
  return (
    <header className="pagehead pagehead--row">
      <div>
        {eyebrow && <p className="eyebrow">{eyebrow}</p>}
        <h1>{title}</h1>
        {children && <p className="muted">{children}</p>}
      </div>
      {actions && <div className="runhead__actions">{actions}</div>}
    </header>
  )
}

/** Muestra un objeto JSON plegado (procedencia, args). */
export function JsonBlock({ value }) {
  return <pre className="json">{JSON.stringify(value, null, 2)}</pre>
}
