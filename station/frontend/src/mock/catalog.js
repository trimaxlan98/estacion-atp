// Catalogo de experimentos del mock (misma forma que CONTRATO §5 parseado a JSON).
const BASE = [
  {
    id: 'n1-lazo-rampa',
    title: 'Lazo de 1 eje sobre rampa',
    level: 'N1',
    kind: 'loop',
    description:
      'Un eje de acimut con controlador PD siguiendo una rampa de velocidad constante. Sirve para ver el error de seguimiento en régimen y la saturación del actuador.',
    launch: 'atp_n1_loop.launch.py',
    args: { rate_dps: 2.0, kp: 8.0, time_scale: 10 },
    params_schema: {
      rate_dps: { type: 'number', min: 0.1, max: 12, label: 'Velocidad de la rampa (deg/s)' },
      kp: { type: 'number', min: 0.5, max: 40, label: 'Ganancia proporcional Kp' },
      time_scale: { type: 'number', min: 1, max: 50, label: 'Aceleración del reloj (x)' },
    },
  },
  {
    id: 'n2-pase-leo-2ejes',
    title: 'Pase LEO de 2 ejes con keyhole',
    level: 'N2',
    kind: 'loop',
    description:
      'Pase LEO completo con montura acimut/elevación. Cerca del cenit el acimut gira mas rápido que el motor: es el keyhole. Prueba con 60 y con 88 grados de elevación máxima.',
    launch: 'atp_loop.launch.py',
    args: { el_max: 60, mask: 5, time_scale: 10 },
    params_schema: {
      el_max: { type: 'number', min: 6, max: 89.9, label: 'Elevación máxima (deg)' },
      mask: { type: 'number', min: 0, max: 30, label: 'Máscara de elevación (deg)' },
      time_scale: { type: 'number', min: 1, max: 50, label: 'Aceleración del reloj (x)' },
    },
  },
  {
    id: 'n3-gemelo-urdf',
    title: 'Gemelo URDF con TF2',
    level: 'N3',
    kind: 'loop',
    description:
      'El mismo pase sobre el gemelo digital de la antena (URDF + TF2). Verifica que la cadena de transformadas reproduce la actitud del lazo.',
    launch: 'atp_twin.launch.py',
    args: { el_max: 45, mask: 5, time_scale: 10 },
    params_schema: {
      el_max: { type: 'number', min: 6, max: 89.9, label: 'Elevación máxima (deg)' },
      mask: { type: 'number', min: 0, max: 30, label: 'Máscara de elevación (deg)' },
      time_scale: { type: 'number', min: 1, max: 50, label: 'Aceleración del reloj (x)' },
    },
  },
  {
    id: 'n4a-pase-oblicuo-s',
    title: 'Pase oblicuo completo en banda S',
    level: 'N4',
    kind: 'loop',
    description:
      'Pase de unos 9 min a 30 grados de elevación máxima. Cierra el presupuesto de enlace en banda S: Doppler en S de +47 a -47 kHz, margen mínimo en el horizonte.',
    launch: 'atp_loop.launch.py',
    args: { el_max: 30, mask: 5, time_scale: 10, band: 'S' },
    params_schema: {
      el_max: { type: 'number', min: 6, max: 89.9, label: 'Elevación máxima (deg)' },
      mask: { type: 'number', min: 0, max: 30, label: 'Máscara de elevación (deg)' },
      time_scale: { type: 'number', min: 1, max: 50, label: 'Aceleración del reloj (x)' },
      band: { type: 'enum', options: ['UHF', 'S', 'X', 'Ka'], label: 'Banda' },
    },
  },
  {
    id: 'n4b-pase-cenital-x',
    title: 'Pase casi cenital en banda X',
    level: 'N4',
    kind: 'loop',
    description:
      'Pase a 85 grados de elevación máxima en banda X (haz estrecho): el keyhole castiga el apuntamiento y el desapuntamiento se come el margen.',
    launch: 'atp_loop.launch.py',
    args: { el_max: 85, mask: 5, time_scale: 10, band: 'X' },
    params_schema: {
      el_max: { type: 'number', min: 6, max: 89.9, label: 'Elevación máxima (deg)' },
      mask: { type: 'number', min: 0, max: 30, label: 'Máscara de elevación (deg)' },
      time_scale: { type: 'number', min: 1, max: 50, label: 'Aceleración del reloj (x)' },
      band: { type: 'enum', options: ['UHF', 'S', 'X', 'Ka'], label: 'Banda' },
    },
  },
  {
    id: 'n4c-campana-montecarlo',
    title: 'Campaña Monte Carlo del lazo',
    level: 'N4',
    kind: 'campaign',
    description:
      'Acción RunCampaign: n pases con elevaciónes máximas y perturbaciones aleatorias. Entrega p50/p95/p99, el peor caso y la incertidumbre del p95.',
    launch: 'atp_campaign.launch.py',
    args: { mode: 'lazo', n: 300 },
    params_schema: {
      mode: { type: 'enum', options: ['lazo', 'lazo_abierto'], label: 'Modo' },
      n: { type: 'number', min: 20, max: 2000, label: 'Número de pases' },
    },
  },
  {
    id: 's6-plan-24h',
    title: 'Plan de contactos de la red (4 estaciones, ACM DVB-S2)',
    level: 'S6',
    kind: 'plan',
    description:
      'Qué contactos hacer, con qué antena y cuántos GB bajan en 24 h con cuatro estaciones y ocho satélites (ISS, NOAA, METEOR). Compara el plan codicioso con el óptimo y mide lo que aporta una estación repitiendo el plan sin ella. En el modo mock se sirve un plan real guardado (los GB son del escenario, no de los satélites).',
    launch: 'planificar.py',
    args: {
      sats: 'ISS,NOAA 15,NOAA 18,NOAA 19,NOAA 20,NOAA 21,METEOR-M2 3,METEOR-M2 4',
      estaciones: 'cdmx,tromso,ecuador,madrid',
      inicio: '2026-10-02T12:00:00',
      horas: 24,
      banda: 'X',
      eirp: 12.0,
      margen_acm: 1.0,
      keyhole: 'marcar',
      metodo: 'auto',
      sin: 'tromso',
    },
    params_schema: {
      horas: { type: 'number', min: 1, max: 48, label: 'Horizonte (h)' },
      banda: { type: 'enum', options: ['UHF', 'S', 'X', 'Ka'], label: 'Banda de bajada' },
      eirp: { type: 'number', min: 0, max: 60, label: 'EIRP del satélite (dBW)' },
      margen_acm: { type: 'number', min: 0, max: 6, label: 'Margen de ACM sobre ETSI (dB)' },
      keyhole: { type: 'enum', options: ['marcar', 'recortar', 'descartar'], label: 'Pases por el keyhole' },
      metodo: { type: 'enum', options: ['auto', 'bb', 'milp'], label: 'Óptimo (auto = propio y, si no prueba, HiGHS)' },
      sin: { type: 'enum', options: ['tromso', 'cdmx', 'ecuador', 'madrid', ''], labels: { '': 'ninguna' }, label: 'Repetir el plan sin la estación' },
    },
  },
  {
    id: 'e1-adcs-funcube',
    title: 'ADCS de un CubeSat 3U (DETUMBLE → NADIR → TRACK_GS) en un pase real',
    level: 'E1',
    kind: 'adcs',
    description:
      'Control de actitud de un CubeSat 3U: frena un tumbling con magnetopares, se orienta a nadir con ruedas y apunta su antena a la estación durante un pase real. En el modo mock se reproduce una corrida real guardada de la variante corta (pase en luz, tumbling de 3 °/s).',
    launch: 'adcs.launch.py',
    args: { sat: 'FUNCUBE-1', station: 'cdmx', start_utc: '2026-10-02T20:30:00', pass_index: 0, el_max_min: 20.0, lead_s: 3600.0, seed: 7, w0_deg_s: 8.0, step_s: 1.0, time_scale: 0, model_degree: 4, band: 'X', d_sat: 0.3 },
    params_schema: {
      start_utc: {
        type: 'enum',
        options: ['2026-10-02T20:30:00', '2026-10-02T08:30:00', '2026-10-02T21:20:00'],
        labels: { '2026-10-02T20:30:00': 'pase de 79° en luz', '2026-10-02T08:30:00': 'pase de 36° en eclipse', '2026-10-02T21:20:00': 'pase en luz (variante corta)' },
        label: 'Escenario (fecha de inicio)',
      },
      lead_s: { type: 'number', min: 600, max: 7200, label: 'Anticipo mínimo al pase (s)' },
      w0_deg_s: { type: 'number', min: 0.5, max: 20, label: 'Tumbling inicial (deg/s)' },
      seed: { type: 'integer', min: 0, max: 9999, label: 'Semilla (actitud inicial y ruido)' },
      model_degree: { type: 'integer', min: 1, max: 13, label: 'Grado del campo magnético a bordo (1 = dipolo)' },
      band: { type: 'enum', options: ['UHF', 'S', 'X', 'Ka'], label: 'Banda de la antena del satélite' },
      d_sat: { type: 'number', min: 0.05, max: 2, label: 'Apertura de la antena del satélite (m)' },
    },
  },
]

/** Metadatos del catalogo de escenas (docs/APP.md §2): area, recursos y artefactos que la app sabe mostrar. */
const META = {
  'n1-lazo-rampa': { area: 'control', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
  'n2-pase-leo-2ejes': { area: 'seguidor', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
  'n3-gemelo-urdf': { area: 'control', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
  'n4a-pase-oblicuo-s': { area: 'rf', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
  'n4b-pase-cenital-x': { area: 'seguidor', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
  'n4c-campana-montecarlo': { area: 'rf', recursos: 'ligero', artefactos: [{ ruta: 'campaign.json', contrato: 'campana' }] },
  's6-plan-24h': { area: 'red', recursos: 'ligero', artefactos: [{ ruta: 'plan.json', contrato: 'plan' }] },
  'e1-adcs-funcube': { area: 'adcs', recursos: 'pesado', artefactos: [{ ruta: 'telemetry.jsonl', contrato: 'telemetria' }] },
}

export const MOCK_EXPERIMENTS = BASE.map((e) => ({ area: 'control', recursos: 'pesado', artefactos: [], ...META[e.id], ...e }))
