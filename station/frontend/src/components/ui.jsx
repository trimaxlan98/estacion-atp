import { useEffect, useRef, useState } from 'react'
import { STATUS_LABEL } from '../format.js'

export function StatusBadge({ status }) {
  if (!status) return null
  return (
    <span className={`badge badge--${status}`}>
      {(status === 'running' || status === 'queued' || status === 'esperando_carga') && <i className="pulse" />}
      {STATUS_LABEL[status] || status}
    </span>
  )
}

/** Cifra con etiqueta. tone: cyan | amber | green | red | gray | violet */
export function Tile({ label, value, unit, tone = 'cyan', hint, big }) {
  return (
    <div className={`tile tone-${tone}${big ? ' tile--big' : ''}`} title={hint}>
      <div className="tile__label">{label}</div>
      <div className="tile__value">
        {value}
        {unit && <small>{unit}</small>}
      </div>
    </div>
  )
}

export function VerdictBanner({ verdict, text }) {
  if (!verdict) return null
  const ok = verdict === 'CUMPLE' || verdict === true
  return (
    <div className={`verdict ${ok ? 'verdict--ok' : 'verdict--bad'}`} role="status">
      <span className="verdict__mark">{ok ? '✓' : '✕'}</span>
      <span className="verdict__text">{typeof verdict === 'string' ? verdict : ok ? 'CUMPLE' : 'NO CUMPLE'}</span>
      {text && <span className="verdict__sub">{text}</span>}
    </div>
  )
}

/** Boton que exige un segundo clic (se rearma a los 4 s). */
export function ConfirmButton({ children, confirmText = '¿Seguro? Confirmar', onConfirm, disabled, className = '' }) {
  const [armed, setArmed] = useState(false)
  const t = useRef(null)
  useEffect(() => () => clearTimeout(t.current), [])
  return (
    <button
      type="button"
      disabled={disabled}
      className={`btn btn--danger ${armed ? 'is-armed' : ''} ${className}`}
      onClick={() => {
        if (!armed) {
          setArmed(true)
          t.current = setTimeout(() => setArmed(false), 4000)
        } else {
          clearTimeout(t.current)
          setArmed(false)
          onConfirm()
        }
      }}
    >
      {armed ? confirmText : children}
    </button>
  )
}

export function LogPanel({ lines, open: defaultOpen = false }) {
  const ref = useRef(null)
  const [open, setOpen] = useState(defaultOpen)
  useEffect(() => {
    if (open && ref.current) ref.current.scrollTop = ref.current.scrollHeight
  }, [lines, open])
  return (
    <details className="panel log" open={open} onToggle={(e) => setOpen(e.currentTarget.open)}>
      <summary>
        <span className="panel__title">Registro</span>
        <span className="muted">{lines.length} líneas</span>
      </summary>
      <pre ref={ref} className="log__body">{lines.length ? lines.join('\n') : 'Sin salida todavía.'}</pre>
    </details>
  )
}

export function Panel({ title, sub, right, children, className = '' }) {
  return (
    <section className={`panel ${className}`}>
      <header className="panel__head">
        <div>
          <h3 className="panel__title">{title}</h3>
          {sub && <p className="panel__sub">{sub}</p>}
        </div>
        {right}
      </header>
      {children}
    </section>
  )
}

export function Legend({ items }) {
  return (
    <div className="legend">
      {items.map((it, i) => (
        <span key={i}>
          <i style={{ background: it.dash ? 'transparent' : it.color, borderTop: it.dash ? `2px dashed ${it.color}` : 'none', height: it.dash ? 0 : 3 }} />
          {it.label}
        </span>
      ))}
    </div>
  )
}
