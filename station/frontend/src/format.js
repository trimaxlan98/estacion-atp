export const nf = (x, d = 2) => (x == null || !isFinite(x) ? '—' : Number(x).toFixed(d))

/** Un argumento de corrida como texto: listas y diccionarios en JSON (con `${v}` salian «[object Object]» y `ruta=10,0`). */
export const argTxt = (v) => (v !== null && typeof v === 'object' ? JSON.stringify(v) : String(v))

/** `clave=valor · clave=valor` de los argumentos de una corrida. */
export const argsTxt = (args, omitir = []) => Object.entries(args || {}).filter(([k]) => !omitir.includes(k)).map(([k, v]) => `${k}=${argTxt(v)}`).join(' · ')

export function fmtDate(iso) {
  if (!iso) return '—'
  const d = new Date(iso)
  if (isNaN(d)) return String(iso)
  return d.toLocaleString('es', { day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit' })
}

export function fmtDur(s) {
  if (s == null) return '—'
  const m = Math.floor(s / 60)
  const r = Math.round(s - m * 60)
  return m ? `${m} min ${String(r).padStart(2, '0')} s` : `${r} s`
}

export const kHz = (hz) => (hz == null ? '—' : (hz / 1000).toFixed(2))

export const STATUS_LABEL = {
  queued: 'En cola',
  running: 'En curso',
  done: 'Terminada',
  error: 'Error',
  cancelled: 'Cancelada',
  timeout: 'Tiempo agotado',
  esperando_carga: 'Esperando carga',
  interrumpida: 'Interrumpida',
}
/**
 * ¿La corrida sigue viva? `esperando_carga` (el gobernador espera a que baje la carga del host) cuenta como viva;
 * `interrumpida` (la mato una senal externa, p. ej. el vigilante) solo mientras no tenga `finished_at`: sin el,
 * espera su reintento; con el, es final (CONTRATO §6).
 */
export const isActive = (s, finishedAt) => s === 'queued' || s === 'running' || s === 'esperando_carga' || (s === 'interrumpida' && !finishedAt)

/** ¿Es una corrida de campana? Por la presencia de `campaign` o por el catalogo. */
export function isCampaignRun(run, expMap) {
  if (!run) return false
  if (run.campaign) return true
  const e = expMap?.[run.experiment_id]
  return e ? e.kind === 'campaign' : false
}

/** kind de una corrida (CONTRATO 6): el detalle lo trae; la lista, por el catalogo. */
export function kindOfRun(run, expMap) {
  if (!run) return 'loop'
  if (run.kind) return run.kind
  if (run.campaign) return 'campaign'
  if (run.plan) return 'plan'
  if (run.mision) return 'mision'
  return expMap?.[run.experiment_id]?.kind ?? 'loop'
}

export function fmtBytes(n) {
  if (n == null || !isFinite(n)) return '—'
  const u = ['B', 'kB', 'MB', 'GB']
  let i = 0
  let x = Number(n)
  while (x >= 1000 && i < u.length - 1) { x /= 1000; i++ }
  return `${x.toFixed(i && x < 100 ? 1 : 0)} ${u[i]}`
}

/** Hace cuanto, corto: «hace 5 min». */
export function hace(iso) {
  if (!iso) return '—'
  const s = (Date.now() - new Date(iso).getTime()) / 1000
  if (!isFinite(s)) return String(iso)
  if (s < 60) return 'hace instantes'
  if (s < 3600) return `hace ${Math.round(s / 60)} min`
  if (s < 86400) return `hace ${Math.round(s / 3600)} h`
  return `hace ${Math.round(s / 86400)} d`
}

// estados de un trabajo de render (docs/APP.md §4)
export const JOB_LABEL = {
  en_cola: 'En cola',
  esperando_carga: 'Esperando carga',
  corriendo: 'Corriendo',
  hecho: 'Hecho',
  error: 'Error',
  cancelado: 'Cancelado',
  interrumpida: 'Interrumpida',
}
export const jobActivo = (e) => e === 'en_cola' || e === 'esperando_carga' || e === 'corriendo'

export const AREA_LABEL = {
  red: 'Red', seguidor: 'Seguidor', control: 'Control', orbita: 'Órbita', rf: 'RF', adcs: 'ADCS', optico: 'Óptico',
  pnt: 'PNT', ntn: 'NTN', mision: 'Misión', hardware: 'Hardware',
}

/** Separa «Qué se mide / Qué se espera / Por qué importa / Validez» de la descripcion de una escena. */
export function secciones(desc) {
  const txt = String(desc || '').replace(/\s+/g, ' ').trim()
  const re = /(Qué se mide|Qué se espera|Por qué importa|Validez)\s*:/g
  const marks = []
  let m
  while ((m = re.exec(txt))) marks.push({ k: m[1], i: m.index, j: m.index + m[0].length })
  if (!marks.length) return { resumen: txt, partes: [] }
  const cap = (t) => t.charAt(0).toUpperCase() + t.slice(1)
  const partes = marks.map((x, n) => ({ titulo: x.k, texto: cap(txt.slice(x.j, marks[n + 1]?.i ?? txt.length).trim()) }))
  return { resumen: partes[0]?.texto ?? txt, partes }
}

/** Segundos -> mm:ss (o h:mm:ss). */
export function fmtReloj(s) {
  const t = Math.max(0, Math.round(s))
  const h = Math.floor(t / 3600)
  const m = Math.floor((t % 3600) / 60)
  const r = t % 60
  const p2 = (n) => String(n).padStart(2, '0')
  return h ? `${h}:${p2(m)}:${p2(r)}` : `${p2(m)}:${p2(r)}`
}
