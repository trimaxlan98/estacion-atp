# Contrato entre piezas de la Estación

> Nota: la numeración de las secciones es estable (se cita desde otros documentos); las que faltan (7.2, 12, 14–16, 20) corresponden a módulos que no forman parte de esta versión educativa.

Fuente única de verdad para los equipos que trabajan en paralelo. Cambiar algo
aquí exige actualizar a los dos lados.

## 1. Directorio de una corrida — `runs/<run_id>/`

`run_id` = `YYYYmmdd-HHMMSS-<6 hex>`. Lo crea el backend; el grabador escribe dentro.

| Archivo | Quién lo escribe | Contenido |
|---|---|---|
| `params.json` | backend | `{experiment_id, launch, args:{...}, ros_domain_id, created_at}` |
| `launch.log` | backend (stdout+stderr del `ros2 launch`) | texto |
| `telemetry.jsonl` | nodo `recorder` | una línea JSON por muestra (ver §2), con `flush` por línea |
| `metrics.json` | nodo `recorder` al recibir `/atp/pass_done` | ver §3 |
| `campaign.json` | nodo `campaign_client` | resultado de la acción `RunCampaign` (ver §4) |
| `plan.json`, `resumen.txt` | `scripts/planificar.py` (`kind: plan`) | plan de contactos de la red (ver §4.1) y sus tablas en texto |
| `informe.json`, `informe.txt`, `metrics.json`, `plan.json`, `mision.json`, `contactos/cNNN[_tle]/` | `scripts/mision.py` (`kind: mision`) | el día de operaciones: informe (§4.2), sus totales (§3.3), el plan, la configuración y **una corrida por contacto** (cada carpeta es una corrida de lazo: `telemetry.jsonl` §2, `metrics.json` §3, `launch.log`, `params.json`) |

Según el `kind` del experimento (§5) cambia quién produce el resultado y su formato: `loop` y
`adcs` graban `telemetry.jsonl` + `metrics.json` (§2/§3 para el lazo, §2.1/§3.1 para el ADCS),
`campaign` escribe `campaign.json` (§4), `plan` escribe `plan.json` (§4.1; el backend deriva de él
un `metrics.json`, §3.2) y `mision` escribe `informe.json` (§4.2) y su `metrics.json` (§3.3). La corrida está
`done` cuando existe su archivo de resultado.

## 2. `telemetry.jsonl` — una muestra (10 Hz de tiempo simulado)

```json
{"t": 12.35, "ref_az": 140.2, "ref_el": 61.0, "az": 140.18, "el": 60.99,
 "e_az": 0.021, "e_el": 0.004, "e_pt": 0.011,
 "range_km": 702.4, "range_rate_km_s": -5.91,
 "doppler_hz": 43360.1, "doppler_rate_hz_s": 120.4,
 "l_point_db": 0.0001, "cn0_dbhz": 87.1, "ebn0_db": 27.1, "margin_db": 16.6,
 "link_closed": true, "u_az": 0.12, "u_el": -0.03}
```

`t` = segundos de tiempo simulado desde AOS. Ángulos en grados. Un campo que
aún no tiene dato se omite (no se rellena con 0).

### 2.1 `telemetry.jsonl` de una corrida ADCS (`kind: adcs`)

Una fila por paso del lazo ADCS (`paso_s` = 1 s de tiempo simulado, no 10 Hz), la escribe el nodo
`adcs_recorder` (`src/sat_adcs`) con `flush` por línea. `t` = segundos de tiempo simulado desde el
inicio del escenario (empieza en 0, no en el AOS). Ángulos en grados; un campo sin dato se omite.

```json
{"t": 3264.0, "mode": 2, "w_deg_s": 0.071, "q": [-0.3685, 0.4890, -0.7803, 0.1270],
 "q_est": [-0.3681, 0.4897, -0.7801, 0.1272], "h_nms": [-0.0004, 0.0001, 0.0002],
 "eclipse": false, "el_deg": 12.4, "range_km": 1930.2, "in_pass": true,
 "u_w_nm": [0.0, 1.1e-5, 0.0], "m_am2": [0.0, 0.0, 0.0],
 "err_deg": 0.48, "know_deg": 0.31, "sigma_deg": 0.12}
```

| Campo | Qué es |
|---|---|
| `mode` | 0 `DETUMBLE` (B-dot), 1 `NADIR`, 2 `TRACK_GS` (como `AdcsCmd.MODE_*`) |
| `w_deg_s` | \|ω\| verdadera |
| `q` | cuaternión verdadero cuerpo → inercial (TEME), `[w, x, y, z]`, unitario |
| `q_est` | el estimado por el MEKF (omitido hasta que el filtro arranca) |
| `h_nms` | momento de cada rueda (con signo), N·m·s; la saturación de cada una es `metrics.wheels.capacity_nms` |
| `eclipse`, `el_deg`, `range_km`, `in_pass` | el satélite en sombra; elevación y rango a la estación; `in_pass` = sobre su máscara |
| `u_w_nm`, `m_am2` | par de motor de cada rueda y dipolo de cada magnetopar (el mando) |
| `err_deg` | error de apuntamiento de la antena (+Z del cuerpo) contra su objetivo: nadir en `NADIR`, la estación en `TRACK_GS`. **Omitido en `DETUMBLE`** (no hay objetivo) |
| `know_deg`, `sigma_deg` | error de conocimiento (estimado − verdad, en el eje de la antena) y el 1-σ que reporta el filtro (eje peor); omitidos sin estimador |

Con la verdad de la planta (que el controlador no ve) solo mide el monitor y este grabador; las filas
nunca llevan más de lo que el monitor imprime en su RESUMEN. `GET /api/runs/{id}/export.csv` aplana los
arreglos como `q_0..q_3`, `q_est_0..`, `h_nms_0..2`, `u_w_nm_0..2`, `m_am2_0..2`.

## 3. `metrics.json`

```json
{"n": 10900, "duration_s": 545.0,
 "pointing": {"rms": 0.021, "p95": 0.026, "max": 0.41, "within_objective_pct": 99.2, "objective_deg": 0.1},
 "axes": {"az": {"rms": 0.021, "max": 0.42}, "el": {"rms": 0.009, "max": 0.20}},
 "link": {"band": "S", "margin_min_db": 10.75, "margin_max_db": 17.68, "closed_pct": 100.0,
          "doppler_max_hz": 46990, "doppler_min_hz": -46990, "doppler_rate_max_hz_s": 388.8,
          "l_point_max_db": 0.197},
 "verdict": "CUMPLE"}
```

### 3.1 `metrics.json` de una corrida ADCS

Lo escribe `adcs_recorder` al cerrar la cuenta tras `/atp/adcs_done` (y entonces el lanzamiento se
apaga). Las cifras son las de `sat_adcs.misiones.Medidor`, las mismas del RESUMEN del monitor. NaN → `null`.

```json
{"kind": "adcs", "n": 4033, "duration_s": 4032.0, "paso_s": 1.0,
 "modes": [{"mode": 0, "name": "DETUMBLE", "t0": 0.0, "t1": 1246.0},
           {"mode": 1, "name": "NADIR", "t0": 1247.0, "t1": 3263.0},
           {"mode": 2, "name": "TRACK_GS", "t0": 3264.0, "t1": 4001.0},
           {"mode": 1, "name": "NADIR", "t0": 4002.0, "t1": 4032.0}],
 "detumble": {"w0_deg_s": 3.0, "w_threshold_deg_s": 0.5936, "t_detumble_s": 406.0, "t_nadir_s": 1247.0},
 "nadir": {"n": 1809, "rms": 0.926, "p95": 1.654, "max": 1.702, "settle_s": 207.2},
 "track": {"n": 588, "rms": 0.475, "p95": 0.717, "max": 0.749, "threshold_deg": 2.404,
           "beamwidth_deg": 8.33, "mask_deg": 5.0},
 "knowledge": {"rms_deg": 0.813, "sigma_rms_deg": 0.125},
 "wheels": {"h_max_nms": 0.000816, "capacity_nms": 0.00177, "frac": 0.461},
 "link": {"band": "X", "antenna_m": 0.3, "l_sat_rms_db": 0.048, "l_sat_max_db": 0.097,
          "margin_min_db": 14.04, "closed_pct": 100.0},
 "eclipse_in_pass": false, "verdict": "CUMPLE"}
```

`modes` cubre todo el escenario (`t1` = última muestra del tramo); `t_detumble_s`/`t_nadir_s` pueden ser
`null`. El umbral `track.threshold_deg` sale del enlace (pérdida de 1 dB con el haz de la antena del
satélite), no del resultado; `verdict` es `CUMPLE` si `track.p95 <= threshold_deg`. El bloque `link` usa
las mismas claves `margin_min_db`/`closed_pct` que el del lazo (§3), así la biblioteca las lee igual.

### 3.2 `metrics` de una corrida de plan (`kind: plan`)

No las escribe un nodo: el backend las deriva de `plan.json` al terminar (`station/backend/station/plans.py`),
las guarda en la base y en `metrics.json`. Son el bloque `planes.optimo.resumen` (o el del codicioso por
peso si no hubo óptimo) más la brecha y el contexto. No hay `verdict`: un plan no cumple ni falla.

```json
{"kind": "plan", "plan": "optimo", "algoritmo": "optimo-milp", "optimo_probado": true,
 "contactos": 115, "minutos": 1178.42, "gb": 349.583, "gb_shannon": 474.768,
 "por_satelite": {"ISS (ZARYA)": {"contactos": 12, "minutos": 79.8, "gb": 26.31, "gb_shannon": 38.0}},
 "por_estacion": {"cdmx": {"contactos": 26, "minutos": 255.8, "gb": 75.41, "gb_shannon": 100.1}},
 "codicioso": {"contactos": 105, "minutos": 1073.01, "gb": 320.379}, "brecha_codicioso_pct": 8.3541,
 "ventanas": 173, "disponible": {"contactos": 173, "minutos": 1758.17, "gb": 516.307},
 "horas": 24.0, "inicio_utc": "2026-10-02T12:00:00.000Z", "banda": "X",
 "estaciones": ["cdmx", "tromso", "ecuador", "madrid"], "satelites": ["ISS (ZARYA)", "..."],
 "aporte_estacion": {"tromso": {"gb_con": 349.583, "gb_sin": 224.984, "delta_gb": 124.599,
                                "delta_pct": 55.38, "contactos_con": 115, "contactos_sin": 81}}}
```

`brecha_codicioso_pct` = `(gb_óptimo − gb_codicioso_por_peso) / gb_óptimo · 100`. `aporte_estacion` solo existe si se
pidió `--sin`/`sin`.

### 3.3 `metrics` de una corrida de misión (`kind: mision`)

Las escribe `scripts/mision.py` junto con `informe.json` (los totales de §4.2 sin las listas largas):

```json
{"kind": "mision", "inicio_utc": "2026-10-02T12:00:00Z", "horas": 24.0, "estaciones": ["cdmx"], "satelites": ["ISS (ZARYA)"],
 "contactos_planificados": 115, "contactos_ejecutados": 115, "contactos_fallidos": 0,
 "minutos_plan": 1178.4, "minutos": 1180.1, "gb_plan": 349.6, "gb": 348.9, "gb_sin_perdida_apunt": 349.0,
 "perdida_apunt_gb": 0.1, "gb_otras_diferencias": -0.6, "plan_algoritmo": "optimo-milp",
 "contactos_cal": 24, "contactos_od": 60, "contactos_gemelo": 40,
 "gb_con_od": 50.2, "gb_con_tle": 50.1, "delta_gb": 0.1, "delta_pct": 0.2, "incidencias": 3, "cuadra": true}
```

## 4. `campaign.json`

```json
{"mode": "lazo", "n": 300, "p50": 0.0295, "p95": 0.0423, "p99": 1.73, "peor": 2.49,
 "pasa": true, "objective_deg": 0.1, "p95_uncertainty_pts": 1.3, "wall_time_s": 10.2,
 "rms_deg": [...], "el_max_deg": [...]}
```

### 4.1 `plan.json` (`kind: plan`)

Lo escribe `scripts/planificar.py` (ver `docs/PLANIFICADOR.md`); con fecha de inicio fija es idéntico byte a
byte. Lo que la Estación lee:

```json
{"banda": "X", "horas": 24.0, "inicio_utc": "2026-10-02T12:00:00.000Z", "n_ventanas": 173,
 "estaciones": {"cdmx": {"nombre": "Ciudad de Mexico", "lat_deg": 19.43, "...": 0}}, "estaciones_lista": ["cdmx", "tromso"],
 "satelites": [{"nombre": "ISS (ZARYA)", "norad": 25544, "edad_tle_dias": 0.68}],
 "disponible": {"contactos": 173, "minutos": 1758.17, "gb": 516.3, "gb_shannon": 698.7, "por_estacion": {}, "por_satelite": {}},
 "ventanas": [{"id": 0, "sat": "NOAA 20 (JPSS-1)", "estacion": "madrid", "aos": "2026-10-02T12:00:00.000Z",
               "los": "2026-10-02T12:07:14.804Z", "duracion_s": 434.8, "el_max_deg": 20.79, "vol_gb": 2.0864,
               "keyhole": false, "t_perdido_s": 0.0, "modcod_max": "32APSK 5/6", "...": 0}],
 "planes": {"optimo": {"algoritmo": "optimo-milp", "ids": [1, 4, 7], "optimo_probado": true, "valor": 349.58,
                       "cota_sup": 349.58, "resumen": {"contactos": 115, "minutos": 1178.4, "gb": 349.58,
                                                        "gb_shannon": 474.8, "por_satelite": {}, "por_estacion": {}}},
            "codicioso_peso": {}, "codicioso_densidad": {}, "codicioso_aos": {}},
 "brecha_codicioso_pct": 8.3541,
 "aporte_estacion": {"tromso": {"gb_con": 349.58, "gb_sin": 224.98, "delta_gb": 124.6, "delta_pct": 55.38,
                                "contactos_con": 115, "contactos_sin": 81, "minutos_con": 1178.4, "minutos_sin": 758.0,
                                "por_satelite_sin": {}, "brecha_codicioso_sin_pct": 0.13, "ids_sin": [0, 3, 6]}}}
```

`planes.*.ids` y `ids_sin` son `id`s de `ventanas`; las ventanas que no están en `ids` son los pases que ese plan
descarta. **Cambio de esta ronda**: `aporte_estacion.<est>.ids_sin` (el plan óptimo sin esa estación, con los ids
de `ventanas`, no los renumerados de `subred`) es nuevo y aditivo: el resto de `plan.json` no cambió.

### 4.2 `informe.json` (`kind: mision`)

Lo escribe `scripts/mision.py` (`docs/MISION.md`). Con inicio fijo y la misma configuración es idéntico byte a byte
(ningún tiempo de pared entra). Lo que la Estación lee:

```json
{"mision": {"inicio_utc": "2026-10-02T12:00:00Z", "horas": 24, "estaciones": ["cdmx"], "satelites": ["ISS (ZARYA)"],
            "config": {"n_od": 3, "n_min_od": 3, "...": 0}, "cargas": {"ISS (ZARYA)": {"banda": "S", "eirp_dbw": 10.0, "rs_baud": 4e6}},
            "plan_algoritmo": "optimo-milp", "plan_optimo_probado": true},
 "plan": {"contactos": 115, "minutos": 1178.4, "gb": 349.6},
 "totales": {"contactos_planificados": 115, "contactos_ejecutados": 115, "minutos": 1180.1, "gb": 348.9,
             "gb_sin_perdida_apunt": 349.0, "perdida_apunt_gb": 0.1, "...": 0},
 "por_satelite": {"ISS (ZARYA)": {"contactos": 12, "minutos": 79.8, "gb": 26.3, "gb_plan": 26.3, "perdida_apunt_gb": 0.0}},
 "por_estacion": {},
 "contactos": [{"id": 4, "contacto": "c004", "sat": "ISS (ZARYA)", "estacion": "madrid", "aos": "2026-10-02T12:59:58Z",
                "banda": "S", "el_max_deg": 15.1, "rol": "cal", "seguimiento": "autotrack", "sdr": true, "efemeride": "tle",
                "estado": "ok", "gb_plan": 0.907, "gb": 0.907, "gb_sin_apunt": 0.907, "perdida_apunt_gb": 0.0,
                "apunt_rms_deg": 0.04, "apunt_max_deg": 0.13, "err_efemeride_deg": {"aos": 0.03, "max": 0.05, "rms": 0.04, "p95": 0.05},
                "carpeta": "contactos/c004", "incidencias": []}],
 "od": [{"sat": "ISS (ZARYA)", "contactos": [4, 8, 9], "n_medidas": 278, "canales": ["az", "el", "rr"], "sigma_pos_km": 0.004,
         "chi2_dof": 0.9, "aceptada": true, "error_pos_epoca_km": 0.03, "error_pos_tle_epoca_km": 0.9}],
 "mejora_od": {"contactos_con_od": 60, "contactos_con_gemelo": 40, "gb_con_od": 0, "gb_con_tle": 0, "delta_gb": 0, "pares": []},
 "incidencias": [{"contacto": "c009", "sat": "ISS (ZARYA)", "tipo": "enlace", "detalle": "..."}],
 "verificacion": {"corridas_releidas": 115, "gb_releido": 348.9, "cuadra_gb": true, "cuadra_minutos": true}}
```

`efemeride` ∈ `tle | od` (con `od_contactos`: los de calibración que la produjeron); `rol` ∈ `cal | normal`;
`seguimiento` ∈ `autotrack | program`. `err_efemeride_deg` y `error_pos_*` se miden contra la VERDAD del mundo y
solo se usan en el informe. Cada `contactos[].carpeta` es una corrida de lazo (§1) dentro de la de la misión; su
gemelo con TLE (`mejora_od.pares`) vive en `contactos/cNNN_tle/`. Un contacto sin gemelo no tiene `pares`.

## 5. Catálogo de experimentos — `experiments/*.yaml`

```yaml
id: n4a-pase-oblicuo-s
title: Pase oblicuo completo en banda S
level: N4
kind: loop            # loop | campaign | plan | adcs | mision | proceso
description: >
  Pase de 7 min a 30 grados de elevacion maxima...
launch: atp_loop.launch.py
args:                 # valores por defecto; el usuario puede sobrescribirlos
  el_max: 30
  mask: 5
  time_scale: 10
  band: S
params_schema:        # lo que el formulario expone (tipo, min, max, opciones)
  el_max: {type: number, min: 6, max: 89.9, label: "Elevacion maxima (deg)"}
  band: {type: enum, options: [UHF, S, X, Ka], label: "Banda"}
```

Para `kind: campaign`, `args` son los campos del goal de `RunCampaign` (`mode`, `n`, …).

| `kind` | `launch` | Se ejecuta | Resultado |
|---|---|---|---|
| `loop` | `atp_loop.launch.py`, `atp_autotrack.launch.py` | `ros2 launch atp_ros <launch> k:=v … record_dir:=<carpeta>` | `telemetry.jsonl` + `metrics.json` (§2, §3) |
| `campaign` | `campaign_run.launch.py` | ídem | `campaign.json` (§4) |
| `adcs` | `adcs.launch.py` | `ros2 launch sat_adcs adcs.launch.py k:=v … record_dir:=<carpeta>` | `telemetry.jsonl` + `metrics.json` (§2.1, §3.1) |
| `plan` | `planificar.py` | `python3 scripts/planificar.py --k=v … --id=<run_id> --salida=<runs>` (`a_b` → `--a-b`; un valor `""` omite el argumento) | `plan.json` + `resumen.txt` (§4.1), `metrics.json` derivado (§3.2) |
| `mision` | `mision.py` | `python3 scripts/mision.py --k=v … --id=<run_id> --salida=<runs>` (como `plan`; cada contacto lanza `ros2 launch atp_mision mision_pase.launch.py` con un `ROS_DOMAIN_ID` del pool 200-229) | `informe.json` + `metrics.json` (§4.2, §3.3) y una corrida de lazo por contacto en `contactos/` |

| `proceso` | (no lleva; el backend guarda `proceso`) | el `comando` del YAML (argv, sin shell) con sus marcadores resueltos y los `args` al final (`estilo_args`) | el archivo `hecho_cuando` (con salida 0) y, si hay, `metricas` (docs/APP.md §2) |

Los YAML (de cualquier `kind`) pueden traer, además, `area` (`red|seguidor|control|orbita|rf|adcs|optico|mision|hardware|plataforma`),
`recursos` (`ligero|pesado|gazebo|manim`, la clase del gobernador: por defecto `plan` = ligero y el resto pesado),
y `artefactos` (`[{ruta, contrato}]`); son obligatorios `area` y `hecho_cuando` en `proceso`.
Detalle en `docs/APP.md` §2-§3 y `station/README.md`.

Cada entrada de `params_schema` puede traer `labels: {valor: texto}` (opcional) para mostrar un valor de un `enum`
con otro texto (p. ej. `""` como «ninguna»); el backend lo ignora. Las claves de `args` que no están en
`params_schema` son fijas (la fecha de inicio de un plan, la red y los satélites, `time_scale: 0` del ADCS).

## 6. API REST de la Estación (`/api`, JSON)

| Método | Ruta | Respuesta |
|---|---|---|
| GET | `/api/health` | `{ok, ros_distro, active_runs}` |
| GET | `/api/experiments` | lista de experimentos (YAML parseado) |
| GET | `/api/experiments/{id}` | uno |
| POST | `/api/runs` `{experiment_id, args?}` | `201 {run_id, status:"queued"}` |
| GET | `/api/runs?experiment_id=&limit=` | lista `{run_id, experiment_id, title, status, created_at, finished_at, args, metrics}` |
| GET | `/api/runs/{id}` | detalle + `kind` (`loop\|campaign\|plan\|adcs\|mision`) + `metrics` + `campaign` + `plan` + `mision` (el `informe.json`; cada uno `null` si no existe) |
| DELETE | `/api/runs/{id}` | borra (409 si activa) |
| POST | `/api/runs/{id}/cancel` | cancela |
| GET | `/api/runs/{id}/telemetry?every=1` | arreglo de muestras §2 (submuestreo opcional) |
| GET | `/api/runs/{id}/log` | texto |
| GET | `/api/runs/{id}/contactos/{cNNN[_tle]}` , `/telemetry?every=` , `/log` | solo `kind: mision`: la corrida de un contacto (`params` + `metrics`; telemetría §2; log); 404 si el id no es `cNNN` o `cNNN_tle` |
| GET (SSE) | `/api/runs/{id}/stream` | eventos `status` (`{status}`), `sample` (muestra §2), `log` (`{line}`), `done` (`{metrics}`) |
| GET | `/api/runs/{id}/export.csv` | telemetría en CSV (§2; para `adcs`, §2.1 aplanado; para `plan` solo la cabecera) |

`status` ∈ `queued | running | done | error | cancelled | timeout`, y desde el gobernador de recursos (aditivo) también
`esperando_carga` (hay cupo pero la carga del host supera el umbral de su clase) e `interrumpida` (la mató una señal externa;
sin `finished_at` espera su reintento, con `finished_at` es final). El detalle `GET /api/runs/{id}` agrega `clase`,
`reintentos` y `artefactos` (`[{ruta, contrato, existe}]`), y `kind` puede ser también `proceso`.

### 6.1 API de la app (aditiva; detalle y ejemplos en `station/README.md`, diseño en `docs/APP.md` §3-§5)

| Método | Ruta | Respuesta |
|---|---|---|
| GET | `/api/escenas` | `/experiments` + `area`, `recursos`, `artefactos`, `miniatura` (URL de `miniatura.jpg` del último medio de la escena, o `null`) |
| GET/HEAD | `/api/runs/{id}/artefactos/{ruta}` | archivo declarado en `artefactos` de la escena y existente (con `Range`); 404 si no está declarado (C4a, §20) |
| GET | `/api/renders/catalogo` | `render/*.yaml` parseados |
| POST | `/api/runs/{id}/renders` `{render_id, args?}` | `201 {job_id, estado:"en_cola"}`; 404 render o corrida; 422 args; 409 si la corrida no está `done` o no cumple `requiere` |
| GET | `/api/renders?run_id=&estado=&limit=` | trabajos `{job_id, run_id, render_id, title, estado, args, clase, pantalla, reintentos, exit_code, error, progreso, miniatura, created_at, started_at, finished_at}` |
| GET | `/api/renders/{job}` | lo anterior + `media` (el `media.json`), `ejecutor` (`backend\|host`) y `trabajador` |
| POST | `/api/renders/reclamar` `{trabajador, clases?}` | solo `ejecutor: host` (docs/APP.md §4.1). 200 con el trabajo que el gobernador admite ahora, ya `corriendo`: `{job_id, render, run_id, clase, comando, cwd, salida, run_dir, pantalla, entorno, timeout_s, latido_cada_s, raiz}` (rutas relativas a la raíz del repo; `entorno` con `DISPLAY`, `ROS_DOMAIN_ID`, `LP_NUM_THREADS`, `RENDER_JOB_ID`, `RENDER_ID`); 204 si no hay nada admisible; 422 si `trabajador`/`clases` son inválidos |
| POST | `/api/renders/{job}/latido` `{progreso?, log?[]}` | `{cancelar, estado}`; escribe `log` y `PROGRESO` en `render.log`. Sin latido en 60 s (`STATION_LATIDO_TIMEOUT`) el trabajo pasa a `interrumpida` y se reintenta. 409 si el trabajo no es del host; 422 si `progreso` ∉ [0,1] o el log es demasiado grande |
| POST | `/api/renders/{job}/fin` `{exit_code, senal?}` | `{job_id, estado, error, reintentos}`: valida `media.json` y decide `hecho` / `error` / `interrumpida` (señal externa, con reintento); un fin tardío (trabajo cancelado, vencido o ya interrumpido) no cambia el estado |
| POST | `/api/renders/{job}/cancel` | `{job_id, estado}`; 409 si ya terminó |
| DELETE | `/api/renders/{job}` | borra un trabajo final (409 si activo) |
| GET (SSE) | `/api/renders/{job}/stream` | eventos `estado` (`{estado, error, reintentos}`), `log` (`{line}`), `progreso` (`{progreso}` 0-1, de las líneas `PROGRESO <0-1>`), `hecho` (`{estado, error, media}`) |
| GET | `/api/medios?tipo=&escena=&limit=` | `[{job_id, run_id, escena, render, titulo, tipo, ruta, url, miniatura, ancho, alto, duracion_s, sha256, creado_utc, procedencia, notas, archivos}]` de los trabajos `hecho` |
| GET, HEAD | `/api/medios/{job}/{ruta}` | el archivo, con `Range` (206 `Content-Range`, 416 si no cabe; un `Range` mal formado se ignora); confinado a `runs/<run>/media/<job>/` |
| GET | `/api/runs/{id}/export.mcap` | MCAP (`/telemetria`, `/red`, `/eventos`; mensajes `json`); 404 si no hay series |
| GET | `/api/sistema` | `{carga{l1,l5,l15,hilos}, memoria, gobernador, colas, cupos, pantallas, activas, vigilante}`; `vigilante` es `null` si no se puede leer `journalctl -t atp-vigilante` |

El contrato de `media.json` (versión 1) está en `docs/APP.md` §4; el backend lo valida y un trabajo cuyo script no lo deja
(o lo deja inválido) termina en `error`. Estados de un render: `en_cola | esperando_carga | corriendo | hecho | error |
cancelado | interrumpida`.

## 7. Tópicos ROS que consume el grabador

`/atp/reference` (JointState rad), `/joint_states` (JointState rad, effort = mando),
`/atp/error_deg` (Float64MultiArray [e_az, e_el, e_pt]), `/atp/sat_state` (SatState),
`/atp/link` (LinkStatus), `/atp/pass_done` (Bool, transient_local).
Parámetro del grabador: `output_dir`.

Opcional (equipo H, `truth:=true`): `/atp/mount_truth` (JointState, sello del estado) con el
ángulo y la velocidad REALES de la carga; `error_monitor` con `use_truth:=true` mide contra él.
Solo para medir: ningún controlador debe usarlo (ver `docs/CONTROL.md`).

### 7.1 Tópicos del ADCS que consume `adcs_recorder`

`/atp/adcs_state` (`sat_interfaces/AdcsState`: sensores y verdad, sellado), `/atp/adcs_cmd` (`AdcsCmd`: modo, mando y
estimador, con el MISMO sello que el estado al que responde) y `/atp/adcs_done` (`Float64`, transient_local: último
instante del escenario). Parámetros: `output_dir` y todos los de `sat_adcs.misiones.Config` (los mismos que el monitor).
Es un nodo pasivo (§8, regla 3) y con él la planta y el controlador esperan **3** suscriptores en vez de 2
(`wait_subscribers`, regla 4); lo pone `adcs.launch.py` cuando `record_dir` no está vacío.

### 7.3 Tópicos del microcontrolador (equipo S; `firmware/esp32_eje/README.md` §2 y `docs/HARDWARE.md`)

Aditivo, sin cambiar los anteriores. Un ESP32 (o su nodo de Linux, `eje_host`) gobierna **uno o dos ejes** y publica por eje `/atp/mcu/<eje>/joint_states` (`JointState`, `name=[<eje>]`), `/atp/mcu/<eje>/diag`
(`Float32MultiArray`) y escucha `/atp/mcu/<eje>/cmd/enable` (`Bool`), `/atp/mcu/<eje>/cmd/origen` (`Float64`) y `/atp/reference`; `<eje>` = `az` | `el`, **igual con 1 o con 2 ejes en la placa** (`ATP_EJES`). `atp_mcu/mcu_merge` los une en `/joint_states`.
Opcional, para medir la latencia del enlace: `/atp/mcu/eco` (`std_msgs/Int64`, sello en ns de cada `/atp/reference` recibida por el MCU). En el banco HIL (`atp_mcu/mcu_lockstep`) el MCU **sustituye** a `mount_sim` + `pid_controller`:
publica `/joint_states(t_k)` y `/atp/step_ack(t_k)` con las reglas de §8 (el estado lleva el sello del paso; el ack sale cuando el MCU confirmó la referencia de ese paso y la pared alcanzó el siguiente), a ritmo de **tiempo real** (`sim_clock` con `time_scale:=0`).

## 8. Tiempo simulado en lockstep (obligatorio para nodos nuevos)

Toda simulación corre con `use_sim_time:=true` y el reloj `sim_clock` en
**lockstep**: el tiempo avanza en pasos fijos de `step_s` (0.05 s) y **solo**
cuando la montura confirma que recibió el mando calculado para ese instante.
Así una corrida da las mismas cifras sin importar la carga de la máquina, y
`time_scale:=0` (lo más rápido posible) da el mismo resultado que `time_scale:=1`.

Cadena de un paso k:

```
/clock t_k ─▶ mount_sim integra [t_{k-1}, t_k] con el mando vigente ─▶ /joint_states (sello t_k)
           ─▶ reference_generator ─▶ /atp/reference y /atp/sat_state (sello t_k)
           ─▶ pid_controller: estado y referencia con el MISMO sello ─▶ /atp/torque (JointState, effort, sello t_k)
           ─▶ mount_sim ─▶ /atp/step_ack (rosgraph_msgs/Clock = t_k) ─▶ sim_clock publica t_{k+1}
```

Reglas:

1. **Sellar con el instante del cálculo**: `header.stamp = now` usando el MISMO
   `now` con que se calculó el valor (no llamar `now()` dos veces).
2. **Emparejar por sello, nunca por "el último que llegó"**: quien combine dos
   tópicos (p. ej. medida y referencia) debe esperar a tener ambos con el
   mismo sello. Emparejar instantes distintos mete un error v·dt (0.45° a
   9°/s y 50 ms).
3. **Los nodos pasivos no frenan el reloj**: monitor, enlace y grabador usan
   colas profundas (≥ 200) y cierran su cuenta con un temporizador de tiempo
   simulado tras `/atp/pass_done`, para recibir los últimos instantes.
4. **Saludo de arranque**: el reloj no pasa del primer paso hasta la primera
   confirmación (compuerta) y el generador no publica la primera referencia
   hasta que todos los nodos de `wait_nodes` anunciaron en `/atp/listo`
   (`std_msgs/String`, transient_local) que **cada una de sus suscripciones
   tiene ya su publicador emparejado**. `atp_ros.nodo.correr` lo hace solo; un
   nodo con `main` propio debe llamar a `anunciar_listo(node)`. Que un nodo
   exista no basta: con la máquina cargada DDS tarda segundos en emparejar, y
   a máxima velocidad eso son decenas de segundos simulados perdidos.
5. **Sin lockstep** (hardware real, `atp_hw`): nadie confirma pasos y el reloj
   corre libre a `time_scale` (1 en HIL); el monitor detecta que los sellos no
   coinciden y empareja con lo último disponible, avisándolo en el log.

Tópicos del lazo: `/atp/torque` es `sensor_msgs/JointState` (`effort` = par en
N·m, sellado); `/atp/step_ack` es `rosgraph_msgs/Clock`.

## 9. Determinación de órbita (opcional, equipo N; detalle en `docs/OD.md`)

Nodo `od_estimator` (lockstep, §8): empareja por sello `/joint_states` (ángulos de
la montura) con `/atp/sat_state` (rango y tasa de rango, con ruido sembrado) dentro
del pase y, al recibir `/atp/pass_done`, ajusta el estado orbital. Publica
`/atp/od_result` (`std_msgs/String`, JSON, `transient_local`, profundidad 1) y escribe
`<od_out>/od.json`: `{version, satelite, norad, epoca_utc, marco:"TEME", x:[r km, v km/s(, p)],
P (covarianza), sigma_pos_km, modelo:{grav, grado, arrastre, cd_a_m, rho_factor}, chi2_dof, ...}`.
`reference_generator` con `profile:=od od_file:=<od.json>` predice el pase con esa órbita.
Todo apagado por defecto (`od:=false` en `atp_loop.launch.py`): sin él, la corrida es la de siempre.

## 10. ATP óptico entre satélites (equipo O; detalle en `src/sat_isl/README.md`)

Escenario ISL en lockstep (§8) con la misma disciplina que el ADCS (§7.1): `isl_sim` (planta), `isl_controller` y `isl_monitor`
(pasivo), más el `sim_clock` de `atp_ros` con `step_s` = 0.05 s (los lazos finos de 10 kHz corren dentro de la planta).

| Tópico | Tipo | Quién |
|---|---|---|
| `/isl/state` | `isl_interfaces/IslState` (sensores de los terminales + verdad, sellado) | `isl_sim` |
| `/isl/cmd` | `isl_interfaces/IslCmd` (modo, direcciones, barrido, tasa; MISMO sello que el estado) | `isl_controller` |
| `/isl/done` | `std_msgs/Float64` (`transient_local`: último instante) | `isl_sim` |
| `/atp/step_ack` | `rosgraph_msgs/Clock` | `isl_sim` (reglas 1-4 de §8) |

`isl.launch.py` espera 2 suscriptores del estado y del mando (controlador/planta y monitor, regla 4). El monitor imprime el `RESUMEN`
(tiempo de adquisición, error fino rms/p95 en µrad, % de tiempo con enlace cerrado, tasa) y apaga el lanzamiento; la misma corrida
da cifras idénticas con `time_scale:=0` y `2` (verificado), y es idéntica a `python3 -m sat_isl.escenario` (sin ROS). No graba `telemetry.jsonl`
(sin kind de experimento en `experiments/` todavía).

## 11. Satélite verdadero distinto de la predicción (opcional, misión integrada; `docs/MISION.md`)

`reference_generator` con `profile:=tle|od` y `truth_file:=<.npz>` (`od.guardar_efemeride`: `t`, `r`, `v`, `a` en el eje
de tiempo de `epoca_utc`) sigue prediciendo el pase con el TLE o la órbita estimada (la referencia, lo que el operador
sabe), pero **el satélite que ve la estación es el verdadero**: `/atp/sat_state` lleva SU acimut, elevación, rango y
tasa de rango, y `/atp/truth` (`JointState`, rad, mismo sello) su dirección. Reglas: la rejilla de tiempo es la del pase
predicho (`t` desde su AOS); el acimut desenroscado de la verdad se alinea con el de la predicción (múltiplo de 360°);
`error_monitor` se remapea a `/atp/truth` y mide contra la verdad, y `link_budget`, `od_estimator` y `sdr_receiver`
emparejan por sello el rango y el error de ese mismo instante (§8). `rf_beacon` con `external_truth:=true` toma la
verdad de `/atp/truth` (no la sintetiza ni la publica). Sin `truth_file` todo es como siempre: la predicción hace de verdad.

## 13. Estrategia de keyhole: la referencia es la ORDEN, la verdad es otra cosa

`reference_generator` con `keyhole_strategy:=anticipada|sobre_cenit` (defecto `ingenua`: nada cambia) publica en
`/atp/reference` la **orden a las articulaciones** (`atp_ros.keyhole.planificar`; con `sobre_cenit` el' puede pasar de 90° y
llegar a `180° − el`), y la **dirección verdadera del satélite** por `/atp/sat_state` y `/atp/truth` (`JointState`, rad, mismo
sello, acimut continuo). Reglas: (1) `error_monitor` se configura con `ref_topic:=/atp/truth`, `esferico:=true` (separación
angular exacta en la esfera) y, con `sobre_cenit`, `montura_sobre_cenit:=true` (una articulación con el' > 90° apunta a
(az' + 180°, 180° − el')); `atp_loop.launch.py` lo hace solo con `keyhole:=…`. (2) El error de apuntamiento SIEMPRE se mide
contra la verdad, nunca contra la orden. (3) La estrategia `sobre_cenit` exige `el_montura_max_deg ≥ 180° − el` al LOS;
si la montura no lo admite el generador **no arranca** (no hay sustituto silencioso). (4) Perfiles finos: `pass_n`
(muestras del perfil sintético, defecto 1200) y `tle_dt_s` (paso del perfil SGP4, defecto 0.5 s; en keyhole 0.05–0.1 s,
si no el acimut cenital queda submuestreado). (5) `mount_sim` acepta `meas_delay_s` (retardo de la lectura del encoder,
defecto 0) para ensayar el gemelo contra una montura "real" perturbada.

## 17. ATP óptico tierra–espacio: la montura de la Estación + un FSM (equipo Y; detalle en `docs/OPTICO.md`)

`atp_optico` no cambia ningún nodo existente: el nodo **pasivo** `optico_atp` (regla 3 del §8: no frena el reloj) se cuelga del lazo de
la montura de siempre (`sim_clock`, `reference_generator` con `profile:=tle`, `mount_sim` con `publish_truth:=true`, `pid_controller`
y `error_monitor` con `use_truth:=true`; `atp_optico/launch/optico.launch.py` los arma) y consume lo que ya publican:

| Tópico | Tipo | Quién | Uso |
|---|---|---|---|
| `/atp/pointing_error` | `atp_interfaces/PointingError` | `error_monitor` | error de la montura contra la **verdad** (solo AOS–LOS), sellado: la perturbación lenta del FSM |
| `/atp/sat_state` | `atp_interfaces/SatState` | `reference_generator` | az/el continuos, rango, `t_since_aos_s` (elevación → turbulencia, enlace, barrido de la línea de vista) |
| `/atp/pass_done` | `std_msgs/Bool` (latched) | `reference_generator` | cierra la cuenta 1 s simulado después |

Reglas: (1) empareja por **sello** (`pe[s]` con `ss[s]`), nunca por el último que llegó; los pares entran al motor en orden de sello; las
llamadas solo emparejan y encolan y un **hilo** alimenta el motor, porque el motor (10 kHz de FSM por paso de 0.05 s,
~10 ms de pared) es más lento que el lockstep (~3 ms por paso a `time_scale:=0`): con colas de 2000 el nodo perdía el principio del pase
(1787 y 1781 pasos de 1831 en dos corridas, bajo carga) y dejaba de ser determinista, así que las suscripciones son de **50 000** y el cierre espera a haber
emparejado el último paso (`t_since_aos_s` ≥ duración − 0.05 s); (2) todo sorteo
(tilt atmosférico, ruido del sensor, sesgo del camino no común, error del modelo de apuntamiento) sale de **un** generador con `semilla` y en
orden fijo: dos corridas dan `optico.json` y `optico_serie.jsonl` idénticos **byte a byte** (`validar.sh optico`); (3) espera de arranque: el
generador debe esperar `wait_subscribers:=2` (`error_monitor` + `optico_atp` suscritos a `/atp/sat_state`) y `wait_nodes` con
`optico_atp`; (4) el motor es **el mismo código** que la corrida sin ROS (`python3 -m atp_optico.escenario`), que sustituye el error de
montura de `/atp/pointing_error` por el de `atp_optico.montura.simular_montura` (la misma montura de `atp_ros` reproducida sin ROS: PID de
20 Hz, planta `PlantaMontura` a 2 ms), y da el mismo `RESUMEN OPTICO`; (5) los lazos finos de 10 kHz corren **dentro** del motor (en un
terminal real son electrónica/FPGA), como en `sat_isl` (§10). Parámetros: cada campo de `atp_optico.escenario.ConfigOptico` es un
argumento del launch y un parámetro del nodo. Salida: `RESUMEN OPTICO` en el log y, con `salida:=<dir>`, `optico.json` (resumen) y
`optico_serie.jsonl` (una fila por paso: error con y sin FSM, `y_max`, `p_rx_dbm`, tasa, outage).

## 18. Rover planetario: dinámica, terreno y planetas (equipos R1/R2/R3; diseño en `docs/ROVER.md`)

Paquete `src/atp_rover` (el equipo R1 lo crea; R2 y R3 solo añaden sus módulos). Núcleo en Python puro sin ROS
(importable en pruebas y en `scripts/`), nodos ROS 2 encima. Todo en tiempo simulado y determinista (§8): misma semilla,
mismos bytes.

**Planetas** (`atp_rover/planetas.py`, un diccionario `PLANETAS[nombre]`): `tierra`, `luna`, `marte`, `titan`, `ceres`.
Campos: `g` (m/s²), `suelo` (parámetros Bekker–Wong: `kc, kphi, n, c, phi_deg, k_deslizamiento`), `densidad_atm`,
`temp_k`, `retardo_tierra_s` (ida y vuelta típica, para §19), `nombre_es`.

**Terreno** (`atp_rover/terreno.py`): campo de alturas `z(x,y)` determinista por `semilla` y `tipo`
(`llano`, `rocoso`, `dunas`, `crater`, `pendiente`), con rejilla de 0.1 m, `altura(x,y)`, `normal(x,y)`,
`suelo(x,y)` (parámetros del suelo; puede variar por zona: regolito suelto, roca, arena).

**Robot** (`atp_rover/rover.py`): 6 ruedas, rocker-bogie, masa y geometría por YAML. Estado: posición ENU `[x,y,z]` m,
cuaternión `[w,x,y,z]`, velocidades, ángulos de las articulaciones del balancín, deslizamiento por rueda.
Modelo: cuerpo rígido 6-DOF + contacto por rueda (resorte-amortiguador vertical, fuerza de tracción por
terramecánica Bekker–Wong simplificada, deslizamiento lateral), integrador RK4 a `dt` fijo de 0.01 s.

**Tópicos ROS** (namespace `/rover`):

| Tópico | Tipo | Quién | Contenido |
|---|---|---|---|
| `/rover/cmd_vel` | `geometry_msgs/Twist` | operador/navegación | `linear.x` m/s, `angular.z` rad/s |
| `/rover/estado` | `std_msgs/String` (JSON, 10 Hz sim) | dinámica | ver abajo |
| `/rover/meta` | `geometry_msgs/PoseStamped` | operador | meta; el navegador (R2) la sigue |
| `/rover/alarma` | `std_msgs/String` (JSON) | seguridad (R2) | `{"t":…,"tipo":"vuelco|atasco|pendiente|energia","detalle":…}` |

`estado` JSON: `{"t","x","y","z","rumbo_deg","cabeceo_deg","alabeo_deg","v","w","pendiente_deg","deslizamiento":[6],
"par_nm":[6],"potencia_w","energia_wh","atascado":bool}`. Un campo sin dato se omite.

**Archivos de corrida** (`kind: rover`, §5): `telemetry.jsonl` (una fila de `estado` por muestra a 10 Hz) y `metrics.json`
`{"distancia_m","tiempo_s","energia_wh","deslizamiento_medio","pendiente_max_deg","atascos","vuelcos","meta_alcanzada",
"planeta","terreno","semilla"}`.

**Escenas** `experiments/rover-*.yaml`: `kind: rover`, args `planeta`, `terreno`, `semilla`, `ruta` (lista de metas) y `duracion_s`.

**Lazo cerrado y navegación (R4, `atp_rover/lazo.py`).** `cli.correr` delega en `lazo.correr`; el argumento `navegacion` elige el
controlador: `basico` (defecto; el `SeguidorBasico` de R1, mismos bytes que antes) o `completa` (`Navegador` de R2: A* sobre costo de
pendiente y rugosidad, pure pursuit, antideslizamiento y seguridad, evaluado a 10 Hz con la orden mantenida entre muestras). El
`Terreno` de R1 es el duck-type de R2 (`altura(x,y)`, `normal(x,y)` unitaria con z arriba). **`energia_wh` es energía consumida**
(integral de la potencia eléctrica: base + tracción/η + pérdidas Joule; nunca baja) y es lo que compara `energia_max_wh` de la
seguridad. Con `completa`, `metrics.json` añade `navegacion`, `fallo` (`null`, `sin_ruta` o `seguridad`: frenado más de 10 s) y `alarmas`
(conteo por tipo); con `basico` no cambia nada.

**R6 (opcionales, sin romper bytes).** `ParametrosRover.suspension: simple|rocker_bogie` (defecto `simple`; argumento de escena `suspension`): con `rocker_bogie` el estado añade `balancin_deg` [izq, der],
`bogie_deg` [izq, der] (positivo = extremo delantero del eslabón hacia abajo; la barra diferencial impone balancín izq = −balancín der), `carga_n` [6] (fuerza normal por rueda, ruedas 0..2 izquierda delante→atrás, 3..5 derecha) y
`margen_vuelco_deg` (ángulo que falta para que el CM salga del polígono de apoyo; 45° en llano), `pendiente_deg` pasa a ser la del plano medio de las normales de contacto, y `metrics.json` añade `suspension` y `margen_vuelco_min_deg`. Con `simple` ninguno aparece
y, con `rodadura_en_motor: false`, los bytes son los de antes. `rodadura_en_motor` (defecto **true** desde la ronda 8; antes false): el motor vence también la resistencia por compactación; `false` reproduce los bytes históricos de R1/R4. Terrenos nuevos `rocas` (Golombek y Rapp 1997) y `escalones`, y `direccion_deg` en `pendiente`.
`seguridad`: la alarma `vuelco` salta también si `margen_vuelco_deg` < 8°. `navegacion`: `completa` (R6: costo graduado, replanificación, PI de velocidad, vaivén), `completa_r4` (la de R4), `completa_r10` (R10: `completa` con la alarma de pendiente medida sobre la huella —`pendiente_huella_m`—, sin límite de aceleración y sin vaivén; `completa` y `completa_r4` no cambian bytes); `fallo` añade `inalcanzable` y `metrics.json` de `completa` y `completa_r10` añade `vaivenes` y `replanes`.

**Lockstep ROS del rover (R9, `atp_rover/lockstep.py` y `nodos_lockstep.py`; cifras en `docs/ROVER.md` «Resultados R9»).** Los nodos `rover_dinamica`
(`modo:=lockstep`), `rover_navegador` (`modo:=lockstep`) y `rover_enlace` (`modo_nodo:=lockstep`) cumplen las reglas 1-4 de §8 con `sim_clock` a
`step_s = 0.1 s` (el intervalo de decisión y de muestreo; la dinámica integra a 0.01 s dentro de cada tick). Launch: `rover_lockstep.launch.py`
(`escena`, `cfg` JSON, `time_scale`, `salida`, `id`). Semántica **fijada** (todo en tiempo de escena `t_k = k·0.1 s`, el `t` del estado, que vale 0 tras `asentar`;
el instante 0 de la escena es el `/clock` de k = 1, el primero que atiende la compuerta de `sim_clock`):

1. La dinámica publica `/rover/estado` e_k con sello t_k y espera. Sin enlace, el navegador responde a **cada** e_k con el mando calculado de e_k.
2. **Ese mando rige el intervalo [t_k, t_{k+1}) completo** (retención de orden cero): 10 pasos de RK4 con el mismo (v, w). Es exactamente el lazo de `lazo.correr`
   (mando recalculado con el último estado muestreado cada 10 pasos), no el «mando en el paso siguiente» del modo pasivo. La dinámica confirma en `/atp/step_ack` el sello del `/clock` con el que publicó e_k y no pasa al tick siguiente hasta que el reloj llegue.
3. Las órdenes (Twist, sin sello) van seguidas de una **ficha** en `<tópico>_listo` (`std_msgs/String` JSON `{"t": t_k, "n": nº de Twist de ese instante, "info": {...}}`): el consumidor espera la ficha y sus `n` mensajes (DDS no ordena entre tópicos) y descarta fichas rancias o duplicadas. Fichas: `/rover/cmd_vel_listo` (hacia la dinámica), `/rover/cmd_vel_op_listo` (del operador al enlace), `/rover/estado_tierra_listo` (del enlace al operador). `info` lleva `nav` (`terminado`, `alcanzada`, `extras` del controlador) y `enlace` (contadores).
4. **Con enlace** (`enlace` = `ideal|retardo|ventanas`): el enlace recibe e_k, entrega a la dinámica las órdenes cuyo instante de entrega es ≤ t_k y a Tierra los estados vencidos (con `t_tierra`), y manda las fichas. El operador (`rover_navegador` en Tierra, que corre el controlador con lo que le llega por `estado_tierra`, o un `guion` `[[t, v, w], …]`) responde a la ficha de t_k con las órdenes de t_k; entran al enlace en el tick k+1 **con su sello de envío t_k**. Una orden se aplica en el primer tick t ≥ t_envío + retardo (cuantización al paso: error < 0.1 s) y, como mínimo, un paso después (en `ideal` el retardo es 0 pero la entrega es en t_{k+1}). Si llegan varias órdenes en la misma ficha rige la última; sin órdenes sigue la vigente (al arrancar, (0, 0)). `t0_s` coloca t = 0 de la escena dentro del calendario del enlace (`fecha`).
5. Arranque: la dinámica no publica e_0 hasta tener `wait_subscribers` suscriptores de `/rover/estado` y lo repite con cada `/clock` hasta recibir la primera ficha; navegador y enlace esperan a que sus publicadores tengan suscriptor y nunca responden dos veces al mismo sello. `sim_clock` lleva `ack_timeout_s: 3600` (con el defecto de 10 s de pared, un tick lento haría avanzar el reloj sin confirmar y rompería el lockstep).
6. Salida: la dinámica escribe `telemetry.jsonl`, `metrics.json` (con `enlace` si lo hay) y `mandos.jsonl` (`{"t","n","v","w"}` por cada tick en que llegó al menos una orden) en `<salida>/<id>/`, imprime `RESUMEN` y apaga el launch. **Misma escena con y sin ROS (`lockstep.correr_cadena`): mismos bytes.** `dinamica: cinematica` es un monociclo ideal solo para probar el protocolo durante horas simuladas, no es la física de R1.

Los modos `pasivo` (por defecto) de los tres nodos conservan el comportamiento de R1-R7 (mando «el último recibido», sellos por el `/clock` vigente al llegar el mensaje) y **no son reproducibles** con un controlador en el lazo.

## 19. Comunicación con el rover por antenas (equipo R7; diseño y cifras en `docs/ROVER.md` «Resultados R7»)

Módulo `atp_rover/comunicacion.py` (Python puro, solo numpy, determinista) y nodo `rover_enlace` (`atp_rover/nodo_enlace.py`). Modela el
enlace rover ↔ orbitador de relé (UHF ~401.6 MHz estilo Electra en Marte; banda S supuesta en la Luna), el enlace directo a Tierra en banda X,
el horizonte local por el terreno de R1 (§18), la geometría del relé (órbita kepleriana de dos cuerpos), el retardo de luz Tierra–cuerpo y una
política de comandos retrasados y telemetría con pérdidas y almacenamiento (DTN: bundles en cola, retransmisión por pase). Solo `marte` y
`luna` (claves de `PLANETAS`); relés en `RELES[planeta]` (`mro`, `odyssey`, `maven`; `polar_100km`, `nrho_kepler`).
Todos los supuestos están declarados en el docstring del módulo; **nada está validado contra telemetría de un rover real**.

**Tópicos ROS** (el nodo se interpone entre el operador y la dinámica; sin él, §18 sigue igual):

| Tópico | Tipo | Quién | Contenido |
|---|---|---|---|
| `/rover/cmd_vel_op` | `geometry_msgs/Twist` | operador (Tierra) | la orden tal como sale de la Tierra |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | `rover_enlace` | la misma orden cuando llega al rover (retardo de luz + espera de contacto; puede no llegar) |
| `/rover/estado` | `std_msgs/String` (JSON, §18) | dinámica | sin cambios |
| `/rover/estado_tierra` | `std_msgs/String` (JSON) | `rover_enlace` | el `estado` de §18 más `"t_tierra"` (instante simulado de recepción); conserva el `t` original; llega en ráfagas al abrirse el contacto |
| `/rover/enlace` | `std_msgs/String` (JSON, ~1 Hz sim) | `rover_enlace` | `{"t","visible","el_deg","tasa_bps","cmd_pendientes","estados_pendientes","cmd_enviados","cmd_perdidos","cmd_no_llegan","est_enviados","est_perdidos","est_no_llegan","est_desbordados"}` |

Parámetros del nodo: `planeta`, `rele`, `lat_deg`, `lon_deg`, `terreno`, `semilla`, `antena` (`omni_lineal|omni_helice|direccional`), `el_min_deg`,
`fecha` (ISO; fija el retardo de luz), `modo` (`ideal` sin enlace | `retardo` solo retardo de luz | `ventanas` retardo + contactos),
`p_perdida_cmd`, `p_perdida_estado`, `horizonte_dias`, `reloj` (`clock` | `pared`). Tiempo simulado como §8; nodo pasivo (no confirma pasos).

**Formatos.** `FiltroEnlace` entrega en orden de instante de entrega (desempate: orden de llegada). Una orden o un estado que no llega antes del
`horizonte_dias` se descarta y se cuenta. Pérdida: sorteo de Bernoulli con `numpy.random.default_rng(semilla)`.

**Escenas** `experiments/rover-enlace-*.yaml`: `kind: proceso`, `area: rf`, `comando: [python3, scripts/enlace_rover.py, …]`
(`estilo_args: cli`), `hecho_cuando: metrics.json`. Args: `planeta`, `rele`, `lat_deg`, `lon_deg`, `terreno`, `semilla`, `antena`, `el_min_deg`,
`dias`, `fecha`, `dt_s`, `gen_mbit_dia`. Salida: `enlace.json` (`{"pases":[{"inicio_s","fin_s","dur_s","el_max_deg","rango_min_km","bits","tasa_pico_bps"}],
"volumen_bits_por_dia":[…]}`) y `metrics.json` (`{"planeta","rele","antena","terreno","dias","pases_por_dia","volumen_mbit_por_dia",
"dur_pase_media_min","el_max_media_deg","retardo_luz_una_via_min","dtn":{…}}`). Misma escena, mismos bytes.

**Estudio.** `python3 scripts/estudio_enlace_rover.py [--rapido]` escribe `docs/rover_resultados/enlace.json` y `enlace.md` (con `--rapido`, en `runs/rover_enlace_rapido/`).

**Operación con retardo (equipo R8; `atp_rover/operacion.py`, Python puro; diseño y cifras en `docs/ROVER.md` «Resultados R8»).** Lazo cerrado operador en Tierra ↔ enlace de §19
↔ rover con autonomía a bordo (`cli.SeguidorBasico`), sin ROS. Cuatro modos (`modo`): `teleop` (a, mover y esperar), `metas` (b, metas por pase), `plan_sol`
(c, plan de sol con alarmas locales) y `gemelo` (d, plan de sol con predicción y conocimiento del terreno). El rover del lazo es un **sustituto cinemático** calibrado con
micro-corridas de la dinámica de §18 (`docs/rover_resultados/operacion_calibracion.json`, clave `planeta|terreno|semilla` → `{v_ratio, p_mov_w, wh_por_m, ...}`); los peligros
(pendiente ≥ `pend_alarma_deg` del `Terreno` de §18 y zonas blandas ocultas) son una regla declarada. Enlace: el de §19 (`Enlace.entrega_comando/entrega_telemetria`) o uno sintético
(`owl_s` constante, ventanas periódicas `intervalo_ventanas_s` × `dur_ventana_s`).

**Formatos.** Mensajes Tierra → rover: `{"tipo":"wps","desde":n,"wps":[[x,y],…],"t_send":t}` (metas del plan con índice; el rover acepta las de índice mayor que el último aceptado, así que reenviar es
idempotente) o `{"tipo":"recup","t_send":t}`; el rover descarta (y cuenta como `cmd_caducados`) lo que llega después de `ttl_s` (por defecto 2·OWL + 120 s en `teleop` y un sol en los demás).
Telemetría rover → Tierra: `{"t","x","y","n_acc","idle","atascado","vuelco"}` (pasa por `tel(t)` del enlace: puede perderse o llegar en ráfaga). Tiempo simulado; tic de 1 s mientras el rover se mueve, salto de eventos
cuando espera. Misma configuración, mismos bytes (RNG separados para enlace, recuperaciones y vuelcos).

**Escenas** `experiments/rover-operacion-{marte,luna}.yaml`: `kind: proceso`, `area: rf`, `comando: [python3, scripts/operacion_rover.py, …]` (`estilo_args: cli`), `hecho_cuando: metrics.json`.
Args: los de `DEFECTOS_OP` (`planeta`, `terreno`, `modo`, `semilla`, `enlace`, `owl_s`, `intervalo_ventanas_s`, `dur_ventana_s`, `n_soles`, `conduccion_h`, `bache_por_km`, `p_perdida_cmd`, `p_perdida_estado`,
`reaccion_operador_s`, `seguridad_local`, `p_dem`, `adelanto_m`, …). Salida `metrics.json` (claves ordenadas): `{"planeta","terreno","semilla","modo","enlace","owl_s","dist_util_m_sol","dist_util_por_sol",
"recorrido_m_sol","t_ocioso_s_sol","t_atascado_s_sol","t_recup_s_sol","cmd_enviados","cmd_perdidos","cmd_no_llegan","cmd_caducados","atascos","vuelcos","reaccion_local_s","reaccion_tierra_s",
"energia_wh_sol","wh_por_m_util","calibracion","…"}`. `calibracion` dice si las cifras usan la tabla de la física (`fisica`), una explícita o un valor por defecto sin calibrar (`defecto`).

**Estudio.** `python3 scripts/estudio_operacion_rover.py [--rapido] [--recalibrar]` escribe `docs/rover_resultados/operacion.{json,md}` (con `--rapido`, en `runs/rover_operacion_rapido/`).

## 21. Dron multirrotor (rondas D1/D2; diseño y cifras en `docs/DRON.md`)

Paquete `src/atp_dron` (núcleo en Python puro, sin ROS, determinista por semilla: mismas entradas, mismos bytes en una misma máquina, §8).
Escenas `experiments/dron-*.yaml` (`kind: proceso`, comando `scripts/dron.py`); la app solo deja cambiar lo declarado en `params_schema`.

**Argumentos** (`atp_dron.lazo.DEFECTOS`): `cuerpo` (`tierra|marte|titan|luna`), `semilla`, `ruta` (metas `[[x,y,z],...]`, m, ENU),
`duracion_s`, `x0`, `y0`, `yaw_deg`, `v_max`, `radio_meta_m`, `viento` (`[vx,vy,vz]` m/s), `rafaga_sigma`, `rafaga_tau_s`, `hz_telemetria`,
`z_despegue_m`, `parar_al_llegar` y `dron` (mapa que pisa `atp_dron.dron.PARAMS`: `masa_kg`, `rotor_escala`, `efecto_suelo`, `bateria_wh`, ...).

**`dron_telemetria`** = `telemetry.jsonl`, una fila por muestra (10 Hz sim por defecto):
`{"t","x","y","z","vx","vy","vz","roll_deg","pitch_deg","yaw_deg","empuje_n","meta"}` (m, m/s, grados, N; `meta` = índice de la meta en curso).

**`dron_metricas`** = `metrics.json`: `{"cuerpo","semilla","tiempo_s","distancia_m","metas_alcanzadas","metas_total","llego","t_llegada_s",
"error_transversal_rms_m","energia_Wh","inclinacion_max_deg","velocidad_max_ms","choque","empuje_peso_efectivo","vuelo_posible"}`;
con `dron.bateria_wh` se añaden `bateria_agotada` y `soc_final` (0–1) y la corrida termina al agotarse. `vuelo_posible` es empuje/peso > 1.

**Efecto suelo** (`dron.efecto_suelo`, defecto `false`): empuje × 1/(1−(R/4z)²) con z ≥ R/2 (Cheeseman y Bennett 1955); apagado, los bytes son los de D1.
Todo es «en simulación, con estos supuestos»: masa, inercias, CdA y el factor de rotor en Marte no tienen fuente.

**Sensores y estimador (D3).** `sensores: true` (defecto `false`: bytes de D2) y `sensores_params` (pisa `atp_dron.sensores.SENSORES`; `gps_corte_s`
corta el GPS). El control y el guiado leen la estimación del ESKF de 15 estados (`atp_dron/estimador.py`), la verdad solo se usa para las métricas.
El generador de los sensores es propio (semilla + 7919): no cambia la dinámica verdadera. `telemetry.jsonl` añade `ex`, `ey`, `ez` (posición
estimada) y `metrics.json` añade `est_error_pos_rms_m`, `est_error_vel_rms_ms`, `est_error_actitud_rms_deg`, `est_error_pos_final_m`,
`est_sesgo_gyro_error_rads` y `est_sesgo_acel_error_ms2`.

**Lockstep ROS (D5).** `ros2 launch atp_dron dron_lockstep.launch.py` (CONTRATO §8; `sim_clock` con `step_s` = 0.02 s = 5 pasos de la dinámica).
Tópicos: `/dron/estado` (`std_msgs/String`, JSON: `d.estado()` + `k` (sello entero del tick), `obs_p`, `obs_v` = lo que ve a bordo), `/dron/pose`
(`geometry_msgs/PoseStamped`, marco `world` ENU, sello = el del `/clock`), `/dron/setpoint` (`std_msgs/String`, JSON `{"k","t","v":[vx,vy,vz],"yaw",
"guia":{"terminado","dest","seg0","i","meta","n","t_llegada"}}`: UNO por sello `k`, rige [t_k, t_k+1) completo) y `/atp/step_ack`
(`rosgraph_msgs/Clock`). Un setpoint con un `k` distinto del pendiente se descarta; el guiado no responde dos veces al mismo `k`. Salidas:
`telemetry.jsonl`, `metrics.json` y `setpoints.jsonl` (una fila `{k, t, v, yaw}` por tick). La misma escena sin ROS (`lazo.correr`) da los
mismos bytes. Nodos: `dron_dinamica`, `dron_navegador`.

**Opciones de la super corrida (D6), todas apagadas por defecto** (con ellas apagadas las escenas dan los mismos bytes que en D5). En la escena (`args`):
`autoridad_guiado` (bool), `autoridad_acel` (`false` | `'rampa'` | `'recorte'`), `antiwindup`, `mezcla_par`, `mezcla_dT_g`, `seg0_corregido`, `inclinacion_max_deg`.
En `dron:` (parámetros del vehículo): `modo_rotor` (`legacy` | `coherente` | `energia_par`), `f_reynolds`, `cuerda_m`, `re_ref`, `avance`, `k_mu`; `vrs` **no está implementado** y lanza error.
En `sensores_params:` `q_corregida`, `h_mag_exacto`, `est_acel_sesgo_rw`, `est_gyro_sesgo_rw`, `gps_gm_tau_s`, `gps_latencia_s`. Módulos nuevos: `guiado_autoridad`, `rafagas_metrica`, `rotor_fisica`, `cascada_indi`, `consistencia`.

