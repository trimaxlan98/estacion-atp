// Mock de la API NUEVA de la app (docs/APP.md §5): escenas, renders, medios, sistema.
// createAppMock(ctx) -> { handle, openRenderStream, mediaUrl }. `ctx` da acceso a las corridas del mock
// base (server.js): { runs: Map, exp(id), experiments, startRunHook? }.
import { MOCK_EXPERIMENTS } from './catalog.js'
import { thumb, thumbDeArea } from '../lib/thumbs.js'

class HttpError extends Error {
  constructor(status, detail) {
    super(detail)
    this.status = status
  }
}

const pad = (n, w = 2) => String(n).padStart(w, '0')
const hex = (n) => Array.from({ length: n }, () => Math.floor(Math.random() * 16).toString(16)).join('')
function jobId(d = new Date()) {
  return `r${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}-${hex(6)}`
}
const iso = (ms) => new Date(ms).toISOString()

// ---------------------------------------------------------------- catalogo de renders (render/*.yaml)
export const RENDERS = [
  {
    id: 'gz-demo-lazo',
    title: 'Montura en Gazebo (demo del lazo de seguimiento)',
    description: 'La montura az/el de atp_description siguiendo la telemetría de un lazo, grabada en 720p.',
    requiere: { kind: 'loop' },
    recursos: 'gazebo',
    ejecutor: 'host',
    pantalla: true,
    duracion_estimada_s: 240,
    args: { acelerar: 10 },
    params_schema: { acelerar: { type: 'number', min: 1, max: 60, label: 'Compresión de tiempo' } },
  },
  {
    id: 'manim-resumen',
    title: 'Video explicativo con Manim (resumen de la corrida)',
    description: 'Animación 1080p con la identidad Co.De: las cifras de la corrida contadas como una historia corta para divulgación.',
    requiere: { cualquiera: true },
    recursos: 'manim',
    ejecutor: 'host',
    pantalla: false,
    duracion_estimada_s: 180,
    args: { calidad: 'h', idioma: 'es' },
    params_schema: {
      calidad: { type: 'enum', options: ['l', 'm', 'h'], labels: { l: '480p', m: '720p', h: '1080p' }, label: 'Calidad' },
      idioma: { type: 'enum', options: ['es', 'en'], label: 'Idioma' },
    },
  },
  {
    id: 'graficas-run',
    title: 'Gráficas de la corrida (matplotlib, PNG y SVG)',
    description: 'Las curvas de la corrida con el estilo de la marca: error contra tiempo, margen de enlace y mapa del cielo.',
    requiere: { cualquiera: true },
    recursos: 'ligero',
    pantalla: false,
    duracion_estimada_s: 20,
    args: { formato: 'png', dpi: 160 },
    params_schema: {
      formato: { type: 'enum', options: ['png', 'svg'], label: 'Formato' },
      dpi: { type: 'integer', min: 80, max: 300, label: 'Resolución (dpi)' },
    },
  },
]

// ---------------------------------------------------------------- sistema
const VIGILANTE = (now) => {
  const lineas = [
    { t: iso(now - 3.2 * 3600e3), texto: 'SOBRECARGA carga1=14.8 > 12.0: detengo ros2 launch atp_demo (memoria disponible 790 MB)' },
    { t: iso(now - 38 * 60e3), texto: 'SOBRECARGA carga1=14.2 > 12.0: detengo contenedor gz_demo' },
    { t: iso(now - 30 * 60e3), texto: 'carga normal (3.1): sin acciones' },
  ]
  return { lineas, ultima_sobrecarga: lineas[1] }
}

export function createAppMock(ctx) {
  const { runs } = ctx
  const jobs = new Map() // job_id -> job
  const jobLog = new Map()
  const jobLis = new Map()
  const timers = new Map()
  const t00 = Date.now()

  const exps = () => MOCK_EXPERIMENTS
  const escenaDe = (id) => exps().find((e) => e.id === id)
  const render = (id) => RENDERS.find((r) => r.id === id)

  // ---------- trabajos y medios
  function cumple(r, run) {
    const e = escenaDe(run.experiment_id)
    const q = r.requiere || {}
    if (q.cualquiera) return true
    if (q.kind) return e?.kind === q.kind
    if (q.contrato) return !!e?.artefactos?.some((a) => a.contrato === q.contrato)
    if (q.archivo) return !!e?.artefactos?.some((a) => a.ruta === q.archivo)
    return true
  }

  function mediaDe(job) {
    const r = render(job.render_id)
    const run = runs.get(job.run_id)
    const e = run && escenaDe(run.experiment_id)
    const proc = { commit: 'd771fd4ab3c19e', sucio: false, comando: ['bash', `scripts/render/${job.render_id.replace(/-/g, '_')}.sh`, `runs/${job.run_id}`, `runs/${job.run_id}/media/${job.job_id}`], args: job.args, corrida: { run_id: job.run_id, experiment_id: run?.experiment_id, args: run?.args || {} } }
    const base = { version: 1, job_id: job.job_id, render: job.render_id, run_id: job.run_id, creado_utc: job.finished_at || job.created_at, duracion_pared_s: job.duracion_pared_s ?? 0, procedencia: proc }
    if (job.render_id === 'gz-demo-lazo') return { ...base, archivos: [{ ruta: 'demo.mp4', tipo: 'video', ancho: 1280, alto: 720, duracion_s: 41.5, sha256: hex(64), principal: true, tam_bytes: 6_200_000 }, { ruta: 'miniatura.jpg', tipo: 'imagen', ancho: 640, alto: 360 }], notas: '' }
    if (job.render_id === 'manim-resumen') return { ...base, archivos: [{ ruta: 'resumen.mp4', tipo: 'video', ancho: 1920, alto: 1080, duracion_s: 32.0, sha256: hex(64), principal: true, tam_bytes: 9_100_000 }, { ruta: 'miniatura.jpg', tipo: 'imagen', ancho: 640, alto: 360 }], notas: `escena Manim para ${e?.title ?? job.run_id}` }
    return {
      ...base,
      archivos: [
        { ruta: `error_vs_tiempo.${job.args?.formato || 'png'}`, tipo: 'imagen', ancho: 1600, alto: 900, sha256: hex(64), principal: true, tam_bytes: 210_000 },
        { ruta: `margen_enlace.${job.args?.formato || 'png'}`, tipo: 'imagen', ancho: 1600, alto: 900, sha256: hex(64), tam_bytes: 190_000 },
        { ruta: 'datos.csv', tipo: 'texto' },
      ],
      notas: '',
    }
  }

  /** Un trabajo como lo entrega la API real (CONTRATO §6.1): sin `media` salvo en el detalle. */
  function publicJob(j, detail = false) {
    const { _t, media, duracion_pared_s, ...o } = j
    void _t
    const r = render(j.render_id)
    const out = { ...o, title: r?.title ?? j.render_id, exit_code: j.estado === 'hecho' ? 0 : j.estado === 'error' ? 1 : null, error: j.error ?? null, miniatura: media ? urlMedio(j.job_id, miniaturaDe(media)) : null }
    if (detail) return { ...out, media: media ?? null, duracion_estimada_s: r?.duracion_estimada_s ?? null }
    return out
  }
  const miniaturaDe = (media) => (media.archivos.find((x) => x.ruta === 'miniatura.jpg') ?? media.archivos.find((x) => x.tipo === 'imagen'))?.ruta
  /** URL de un archivo de medios: en el mock el propio data-URI o el mp4 de muestra de public/mock/. */
  function urlMedio(job_id, ruta) {
    if (!ruta) return null
    return mediaUrl(job_id, ruta)
  }

  function emitJob(id, ev, data) {
    for (const h of [...(jobLis.get(id) || [])]) h[ev]?.(data)
  }
  function logJob(j, line) {
    const l = jobLog.get(j.job_id) || []
    l.push(line)
    jobLog.set(j.job_id, l)
    emitJob(j.job_id, 'log', { line })
  }
  function setEstado(j, estado) {
    j.estado = estado
    emitJob(j.job_id, 'estado', { estado })
  }

  function simulate(j) {
    const r = render(j.render_id)
    const dur = j.render_id === 'graficas-run' ? 3500 : 9000
    const espera = r.recursos === 'gazebo' ? 3600 : 0
    logJob(j, `[gobernador] clase ${r.recursos}: en cola`)
    const t1 = setTimeout(() => {
      if (espera) {
        setEstado(j, 'esperando_carga')
        logJob(j, '[gobernador] carga 1 min 5.3 > umbral 4.0 (render por software): espero, reevalúo cada 10 s')
      }
      const t2 = setTimeout(() => {
        j.started_at = iso(Date.now())
        setEstado(j, 'corriendo')
        if (r.pantalla) { j.pantalla = ':80'; logJob(j, '[gobernador] pantalla virtual :80 (Xephyr) asignada') }
        logJob(j, `$ ${'bash scripts/render/' + r.id.replace(/-/g, '_') + '.sh'} runs/${j.run_id} runs/${j.run_id}/media/${j.job_id}`)
        const t0 = Date.now()
        const tick = setInterval(() => {
          const p = Math.min(1, (Date.now() - t0) / dur)
          j.progreso = Math.round(p * 1000) / 1000
          emitJob(j.job_id, 'progreso', { progreso: j.progreso })
          if (Math.random() < 0.35) logJob(j, `PROGRESO ${j.progreso.toFixed(2)}`)
          if (p >= 1) {
            clearInterval(tick)
            j.finished_at = iso(Date.now())
            j.duracion_pared_s = Math.round((Date.now() - t0) / 100) / 10
            j.media = mediaDe(j)
            logJob(j, `-> media.json (${j.media.archivos.length} archivos)`)
            setEstado(j, 'hecho')
            emitJob(j.job_id, 'hecho', { media: j.media })
          }
        }, 400)
        timers.set(j.job_id, tick)
      }, espera || 500)
      timers.set(j.job_id + ':b', t2)
    }, 500)
    timers.set(j.job_id + ':a', t1)
  }

  function crearJob(runId, renderId, args, over = {}) {
    const r = render(renderId)
    const j = {
      job_id: jobId(), run_id: runId, render_id: renderId, estado: 'en_cola', progreso: null,
      created_at: iso(Date.now()), started_at: null, finished_at: null, duracion_pared_s: null,
      args: { ...r.args, ...(args || {}) }, recursos: r.recursos, pantalla: null, reintentos: 0, ...over,
    }
    jobs.set(j.job_id, j)
    jobLog.set(j.job_id, [])
    return j
  }

  // ---------- siembra: renders de ejemplo sobre las corridas sembradas
  function seed() {
    const lazos = [...runs.values()].filter((x) => escenaDe(x.experiment_id)?.kind === 'loop')
    const lazo = lazos[0]
    const now = Date.now()
    const mk = (run, rid, hace, dur, extra = {}) => {
      if (!run) return null
      const j = crearJob(run.run_id, rid, {}, {
        created_at: iso(now - hace), started_at: iso(now - hace + 20e3), finished_at: iso(now - hace + dur * 1e3),
        duracion_pared_s: dur, progreso: 1, estado: 'hecho', ...extra,
      })
      j.media = mediaDe(j)
      jobLog.set(j.job_id, [`[gobernador] clase ${render(rid).recursos}`, `$ bash scripts/render/${rid.replace(/-/g, '_')}.sh`, 'PROGRESO 0.50', 'PROGRESO 1.00', `-> media.json (${j.media.archivos.length} archivos)`])
      return j
    }
    mk(lazo, 'graficas-run', 5.4 * 3600e3, 14.8)
    mk(lazo, 'manim-resumen', 2.1 * 3600e3, 171.2)
    mk(lazo, 'gz-demo-lazo', 9 * 3600e3, 226.9)
    mk(lazo, 'graficas-run', 8.9 * 3600e3, 11.0)
    if (lazo) {
      const e = crearJob(lazo.run_id, 'gz-demo-lazo', { acelerar: 60 }, { created_at: iso(now - 70 * 60e3), started_at: iso(now - 69 * 60e3), finished_at: iso(now - 66 * 60e3), duracion_pared_s: 190, estado: 'interrumpida', reintentos: 2, progreso: 0.42, error: 'el vigilante detuvo el contenedor gz_demo (señal 15); sin reintentos restantes' })
      jobLog.set(e.job_id, ['[gobernador] clase gazebo', '$ bash scripts/render/gz_demo_lazo.sh', 'PROGRESO 0.42', '[gobernador] el proceso murió por la señal 15 (vigilante del host)', '[gobernador] reintento 1/2 en 30 s', '[gobernador] reintento 2/2 en 60 s', '[gobernador] sin reintentos restantes: interrumpida'])
      const f = crearJob(lazo.run_id, 'manim-resumen', { calidad: 'h' }, { created_at: iso(now - 4.2 * 3600e3), started_at: iso(now - 4.2 * 3600e3 + 9e3), finished_at: iso(now - 4.2 * 3600e3 + 31e3), duracion_pared_s: 22, estado: 'error', error: 'media.json ausente (el script terminó con código 1)' })
      jobLog.set(f.job_id, ['[gobernador] clase manim', '$ bash scripts/render/manim_resumen.sh', 'Traceback (most recent call last):', "  File \"contenido/render.sh\", line 12", "FileNotFoundError: marca/derivados/logo-cian.png", '[gobernador] código de salida 1; no hay media.json -> error'])
      // un render en curso para que la cola de la vista Render tenga vida al abrirla
      const v = crearJob(lazo.run_id, 'gz-demo-lazo', { acelerar: 20 })
      v.estado = 'corriendo'
      v.started_at = iso(now - 40e3)
      v.pantalla = ':81'
      v.progreso = 0.12
      jobLog.set(v.job_id, ['[gobernador] clase gazebo', '[gobernador] pantalla virtual :81 (Xephyr) asignada', '$ bash scripts/render/gz_demo_lazo.sh', 'PROGRESO 0.12'])
      const t0 = now - 40e3
      const dur = 150000
      timers.set(v.job_id, setInterval(() => {
        const p = Math.min(0.99, 0.12 + (Date.now() - t0) / dur)
        v.progreso = Math.round(p * 1000) / 1000
        emitJob(v.job_id, 'progreso', { progreso: v.progreso })
        if (Math.random() < 0.25) logJob(v, `PROGRESO ${v.progreso.toFixed(2)}`)
      }, 1200))
      const w = crearJob(lazo.run_id, 'gz-demo-lazo', {})
      // en_cola de un render que ejecuta el trabajador del HOST (APP §4.1): lleva rato sin que nadie lo reclame
      w.created_at = iso(now - 95e3)
      jobLog.set(w.job_id, ['[gobernador] clase gazebo: en cola (ejecutor host: espera a que el trabajador lo reclame)'])
      const x = crearJob(lazo.run_id, 'gz-demo-lazo', { acelerar: 40 })
      x.estado = 'esperando_carga'
      x.created_at = iso(now - 25e3)
      jobLog.set(x.job_id, ['[gobernador] clase gazebo: en cola', '[gobernador] carga 1 min 5.3 > umbral 4.0 (render por software): espero, reevalúo cada 10 s'])
    }
  }

  // ---------- metricas.json de una corrida `proceso`
  function metricas(run) {
    return { kind: 'proceso', escena: run.experiment_id, duracion_s: Number(run.args?.dur_s ?? 600), verdict: 'CUMPLE' }
  }

  // ---------- derivados
  function miniaturaDeEscena(e) {
    return thumb(thumbDeArea(e.area), e.id)
  }
  /** Fila de /api/medios (CONTRATO §6.1): el archivo principal de un trabajo `hecho`. */
  function medioFila(j) {
    const run = runs.get(j.run_id)
    const f = j.media.archivos.find((x) => x.principal) ?? j.media.archivos[0]
    return {
      job_id: j.job_id, run_id: j.run_id, escena: run?.experiment_id ?? null, render: j.render_id, titulo: render(j.render_id)?.title ?? j.render_id,
      tipo: f.tipo, ruta: f.ruta, url: urlMedio(j.job_id, f.ruta), miniatura: urlMedio(j.job_id, miniaturaDe(j.media)),
      ancho: f.ancho ?? null, alto: f.alto ?? null, duracion_s: f.duracion_s ?? null, sha256: f.sha256 ?? null,
      creado_utc: j.media.creado_utc, procedencia: j.media.procedencia, notas: j.media.notas ?? '',
      archivos: j.media.archivos.map((x) => ({ ...x, url: urlMedio(j.job_id, x.ruta) })),
    }
  }
  const mediosHechos = () => [...jobs.values()].filter((j) => j.estado === 'hecho' && j.media)

  /** `GET /api/sistema`: la misma forma que station/backend/station/sistema.py. */
  function sistema() {
    const now = Date.now()
    const w = (now - t00) / 1000
    const l1 = Math.round(Math.max(0.4, 3.1 + Math.sin(w / 14) * 1.1 + Math.sin(w / 5) * 0.3) * 100) / 100
    const claseDeRun = (r) => escenaDe(r.experiment_id)?.recursos ?? 'pesado'
    const vivasRun = [...runs.values()].filter((r) => ['queued', 'running', 'esperando_carga'].includes(r.status))
    const vivasJob = [...jobs.values()].filter((j) => ['en_cola', 'esperando_carga', 'corriendo'].includes(j.estado))
    const cupos = { ligero: 2, pesado: 1, gazebo: 1, manim: 1 }
    const umbral = { ligero: 10, pesado: 6, gazebo: 4, manim: 5 }
    const clases = {}
    for (const c of Object.keys(cupos)) {
      const corriendo = vivasJob.filter((j) => j.estado === 'corriendo' && j.recursos === c).length + vivasRun.filter((r) => r.status === 'running' && claseDeRun(r) === c).length
      const espCarga = vivasJob.filter((j) => j.estado === 'esperando_carga' && j.recursos === c).length + vivasRun.filter((r) => r.status === 'esperando_carga' && claseDeRun(r) === c).length
      const enCola = vivasJob.filter((j) => j.estado === 'en_cola' && j.recursos === c).length + vivasRun.filter((r) => r.status === 'queued' && claseDeRun(r) === c).length
      clases[c] = { cupo: cupos[c], corriendo, umbral_carga: umbral[c], en_cola: enCola, esperando_carga: espCarga, admite_ahora: corriendo < cupos[c] && l1 < umbral[c] }
    }
    const libres = [':80', ':81', ':82', ':83'].filter((x) => !vivasJob.some((j) => j.pantalla === x && j.estado === 'corriendo'))
    const gobernador = {
      tope_global: 2, corriendo_total: Object.values(clases).reduce((a, c) => a + c.corriendo, 0), clases,
      corriendo: vivasJob.filter((j) => j.estado === 'corriendo').map((j) => ({ clase: j.recursos, tarea: `render ${j.render_id}`, pantalla: j.pantalla, desde: j.started_at })),
      esperando: [
        ...vivasJob.filter((j) => j.estado !== 'corriendo').map((j) => ({ clase: j.recursos, tarea: `render ${j.render_id}`, motivo: j.estado === 'esperando_carga' ? 'carga' : 'cola', espera_s: Math.round((now - new Date(j.created_at)) / 1000), desde: j.created_at })),
        ...vivasRun.filter((r) => r.status !== 'running').map((r) => ({ clase: claseDeRun(r), tarea: `corrida ${r.experiment_id}`, motivo: r.status === 'esperando_carga' ? 'carga' : 'cola', espera_s: Math.round((now - new Date(r.created_at)) / 1000), desde: r.created_at })),
      ],
      pantallas: { pool: [':80', ':81', ':82', ':83'], libres }, dominios_ros_en_uso: [], reintentos: 2, recheck_s: 10,
    }
    const total = 15870
    const disp = Math.round(7470 - Math.sin(w / 20) * 600)
    return {
      carga: { l1, l5: Math.round((l1 * 0.85 + 0.2) * 100) / 100, l15: Math.round((l1 * 0.7 + 0.5) * 100) / 100, hilos: 8, umbral_vigilante_estimado: 12, leida_por_gobernador: l1 },
      memoria: { total_mb: total, disponible_mb: disp, usada_pct: Math.round(((total - disp) / total) * 1000) / 10, swap_total_mb: 2048, swap_usada_mb: 120 },
      gobernador,
      colas: Object.fromEntries(Object.entries(clases).map(([c, v]) => [c, { en_cola: v.en_cola, esperando_carga: v.esperando_carga, corriendo: v.corriendo }])),
      cupos: Object.fromEntries(Object.entries(clases).map(([c, v]) => [c, { cupo: v.cupo, umbral_carga: v.umbral_carga, admite_ahora: v.admite_ahora }])),
      pantallas: gobernador.pantallas,
      activas: { corridas: vivasRun.length, renders: vivasJob.length },
      vigilante: VIGILANTE(now), // desde el contenedor real puede ser null (el diario del host no se monta)
    }
  }

  // ---------- enrutador
  function handle(method, p, q, body) {
    let m
    if (method === 'GET' && p === '/api/escenas') {
      const porEscena = {}
      for (const j of mediosHechos()) {
        const eid = runs.get(j.run_id)?.experiment_id
        if (eid && !porEscena[eid] && j.media.archivos.some((f) => f.ruta === 'miniatura.jpg')) porEscena[eid] = j
      }
      return exps().map((e) => ({
        ...e,
        miniatura: porEscena[e.id] ? thumb(thumbDeArea(e.area), 'media:' + e.id) : null,
        corridas: [...runs.values()].filter((r) => r.experiment_id === e.id).length,
      }))
    }
    if (method === 'GET' && p === '/api/renders/catalogo') return RENDERS
    if (method === 'GET' && p === '/api/renders') {
      const rid = q.get('run_id')
      const est = q.get('estado')
      const limit = Number(q.get('limit') || 100)
      return [...jobs.values()]
        .filter((j) => (!rid || j.run_id === rid) && (!est || j.estado === est))
        .sort((a, b) => b.created_at.localeCompare(a.created_at))
        .slice(0, limit)
        .map((j) => publicJob(j))
    }
    if ((m = p.match(/^\/api\/renders\/([^/]+)$/)) && method === 'GET') {
      const j = jobs.get(m[1])
      if (!j) throw new HttpError(404, 'trabajo no encontrado')
      return { ...publicJob(j, true), log: (jobLog.get(j.job_id) || []).slice(-200) }
    }
    if ((m = p.match(/^\/api\/renders\/([^/]+)\/cancel$/)) && method === 'POST') {
      const j = jobs.get(m[1])
      if (!j) throw new HttpError(404, 'trabajo no encontrado')
      if (['en_cola', 'esperando_carga', 'corriendo'].includes(j.estado)) {
        for (const k of [j.job_id, j.job_id + ':a', j.job_id + ':b']) { clearInterval(timers.get(k)); clearTimeout(timers.get(k)) }
        j.finished_at = iso(Date.now())
        logJob(j, '[gobernador] cancelado por el usuario')
        setEstado(j, 'cancelado')
      }
      return publicJob(j)
    }
    if ((m = p.match(/^\/api\/runs\/([^/]+)\/renders$/)) && method === 'POST') {
      const run = runs.get(m[1])
      if (!run) throw new HttpError(404, 'corrida no encontrada')
      const r = render(body?.render_id)
      if (!r) throw new HttpError(404, 'render no encontrado')
      if (run.status !== 'done') throw new HttpError(409, `la corrida está en estado «${run.status}»: solo se renderizan corridas terminadas`)
      if (!cumple(r, run)) throw new HttpError(409, `la corrida no cumple lo que pide el render (${JSON.stringify(r.requiere)})`)
      const j = crearJob(run.run_id, r.id, body?.args)
      simulate(j)
      return { job_id: j.job_id, estado: j.estado, _status: 201 }
    }
    if (method === 'GET' && p === '/api/medios') {
      const tipo = q.get('tipo')
      const escena = q.get('escena')
      const limit = Number(q.get('limit') || 200)
      return mediosHechos()
        .sort((a, b) => (b.media.creado_utc || '').localeCompare(a.media.creado_utc || ''))
        .map(medioFila)
        .filter((f) => (!tipo || f.tipo === tipo) && (!escena || f.escena === escena))
        .slice(0, limit)
    }
    if (method === 'GET' && p === '/api/sistema') return sistema()
    if (method === 'GET' && (m = p.match(/^\/api\/runs\/([^/]+)\/export\.mcap$/))) throw new HttpError(501, 'MCAP no disponible en el mock')
    return undefined
  }

  function openRenderStream(id, handlers) {
    const j = jobs.get(id)
    if (!j) { setTimeout(() => handlers.onError?.(new Error('404')), 0); return () => {} }
    const h = { estado: (d) => handlers.onEstado?.(d), log: (d) => handlers.onLog?.(d), progreso: (d) => handlers.onProgreso?.(d), hecho: (d) => handlers.onHecho?.(d) }
    if (!jobLis.has(id)) jobLis.set(id, new Set())
    jobLis.get(id).add(h)
    setTimeout(() => {
      handlers.onEstado?.({ estado: j.estado })
      if (j.progreso != null) handlers.onProgreso?.({ progreso: j.progreso })
      if (j.estado === 'hecho') handlers.onHecho?.({ media: j.media })
    }, 0)
    return () => jobLis.get(id)?.delete(h)
  }

  /** URL de un archivo de medios en el mock (el real es /api/medios/{job}/{ruta}). */
  function mediaUrl(job_id, ruta) {
    const j = jobs.get(job_id)
    if (/\.(mp4|webm)$/i.test(ruta)) return './mock/visor-demo.mp4'
    if (/\.(jpe?g|png|svg)$/i.test(ruta)) {
      const run = j && runs.get(j.run_id)
      const e = run && escenaDe(run.experiment_id)
      const tipo = /miniatura/.test(ruta) ? thumbDeArea(e?.area) : j?.render_id === 'graficas-run' ? 'grafica' : thumbDeArea(e?.area)
      return thumb(tipo, job_id + ruta)
    }
    return '#'
  }

  const activeRenders = () => [...jobs.values()].filter((j) => ['en_cola', 'esperando_carga', 'corriendo'].includes(j.estado)).length
  return { handle, activeRenders, openRenderStream, mediaUrl, seed, metricas, miniaturaDeEscena, HttpError }
}
