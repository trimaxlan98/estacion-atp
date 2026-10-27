# Estación ATP — backend

Servicio FastAPI que lanza experimentos sobre el motor ROS 2 (`atp_ros`, `sat_adcs`) y el
planificador de contactos, guarda cada corrida (parámetros, telemetría, métricas) y la sirve por
REST y SSE.
Contrato con el grabador y el frontend: [`docs/CONTRATO.md`](../docs/CONTRATO.md).
La app completa (escenas `kind: proceso`, gobernador de recursos, renders y medios, visor 3D del rover) está en [`docs/APP.md`](../docs/APP.md); este README documenta el backend que la implementa.

> `station/` **no es un paquete ROS** (hay un `COLCON_IGNORE`): colcon no lo
> recorre. El backend es Python puro; ROS solo corre en los subprocesos.

## Arquitectura

```
navegador ──▶ uvicorn (1 worker, 127.0.0.1:8090)
               ├─ experiments/*.yaml      catálogo versionado en git
               ├─ station.db (SQLite WAL) índice de corridas: estado, args, métricas
               ├─ Governor                cupos por clase (ligero|pesado|gazebo|manim), admisión por
               │                          carga del host, pantallas Xephyr :80-:83, dominios ROS 100-199
               ├─ Manager                 las corridas: pasan por el gobernador, reintentan si una señal
               │                          externa las mata (ver "Gobernador de recursos")
               ├─ RenderManager           los trabajos de render/*.yaml -> runs/<id>/media/<job>/media.json
               │    └─ subprocess (grupo propio):
               │         ros2 launch atp_ros <launch> k:=v ... record_dir:=runs/<id>
               │         (kind adcs: ros2 launch sat_adcs adcs.launch.py ...;
               │          kind plan: python3 scripts/planificar.py --k=v ... --id=<id>;
               │          kind mision: python3 scripts/mision.py --k=v ... --id=<id>, que a su vez
               │          lanza un `ros2 launch atp_mision mision_pase.launch.py` por contacto)
               │         con ROS_DOMAIN_ID único (100-199)
               ├─ runs/<id>/             params.json, launch.log, telemetry.jsonl,
               │                         metrics.json | campaign.json | plan.json
               └─ SSE /stream             tail de telemetry.jsonl y launch.log
```

Quién escribe qué (CONTRATO §1): el backend crea `params.json` y `launch.log`;
el nodo `recorder` escribe `telemetry.jsonl` (10 Hz de tiempo simulado) y
`metrics.json`; el nodo `campaign_client` escribe `campaign.json`; en `kind: adcs` el nodo
`adcs_recorder` de `sat_adcs` escribe la telemetría y las métricas de actitud (CONTRATO 2.1 y 3.1);
en `kind: plan` el script `scripts/planificar.py` escribe `plan.json` y `resumen.txt` y el backend
deriva de él las `metrics` (CONTRATO 3.2).

Código (`station/backend/station/`):

| Archivo | Qué hace |
|---|---|
| `config.py` | rutas, límites y regex de ids (variables `STATION_*`) |
| `db.py` | SQLite (WAL) con la tabla `runs` |
| `experiments.py` | carga del catálogo, validación estricta de args, `kind`s (`loop`, `campaign`, `plan`, `adcs`, `mision`, `proceso`) |
| `runner.py` | `Manager`: la corrida (estados, fin por archivos, métricas); el comando según el `kind` |
| `governor.py` | `Governor`: cupos y umbrales de carga por clase, pantallas, dominios; `muerte_por_senal` |
| `executor.py` | ciclo de vida común de corridas y renders: turno -> subprocess (grupo propio) -> timeout / cancelación / reintento tras señal |
| `renders.py` | catálogo `render/*.yaml`, validación de `media.json`, galería y `RenderManager` |
| `sistema.py` | carga, memoria y diario del vigilante (`GET /api/sistema`) |
| `medios.py` | archivos con `Range` (video) confinados a `runs/<id>/media/<job>/` |
| `mcap.py` | escritor MCAP propio (sin la librería) para `export.mcap` |
| `plans.py` | `metrics` de una corrida de plan, derivadas de `plan.json` |
| (misión) | `kind: mision` no necesita módulo propio: el script escribe `metrics.json` e `informe.json`; el backend solo lo lanza (`runner.default_command`), lo da por `done` al existir `informe.json` y sirve las corridas de cada contacto (`/runs/{id}/contactos/...`) |
| `stream.py` | lectura de series, CSV y generador SSE |
| `app.py` | rutas de la API y montaje del frontend |
| `main.py` | `app = create_app()` para uvicorn |

## Cómo levantarla

Desde la raíz del repo:

```bash
./station.sh          # docker compose up station  ->  http://127.0.0.1:8090
./station.sh -d       # en segundo plano (docker compose stop station para detener)
```

El servicio usa la misma imagen `ros2-jazzy-dev` (red `host`, monta `./:/ws`).
Su `station/entrypoint.sh` hace `colcon build` solo si algo en `src/` es más
nuevo que la última compilación, carga `install/setup.bash` y arranca
`uvicorn station.main:app --workers 1`. Si existe `station/frontend/dist`
(lo construye el equipo del frontend) se sirve en `/`. Para otro puerto:
`STATION_PORT=8191 ./station.sh`.

Sin Docker (dentro del contenedor, con ROS cargado):

```bash
cd station/backend && python3 -m uvicorn station.main:app --host 127.0.0.1 --port 8090
```

Ejemplos:

```bash
curl -s localhost:8090/api/experiments | python3 -m json.tool | head
curl -s -X POST localhost:8090/api/runs -H 'content-type: application/json' \
     -d '{"experiment_id":"n4a-pase-oblicuo-s","args":{"band":"X"}}'
curl -N localhost:8090/api/runs/<run_id>/stream     # SSE en vivo
```

Variables de entorno: `STATION_ROOT`, `STATION_RUNS_DIR` (por defecto `runs/`),
`STATION_DB_PATH` (`station/data/station.db`), `STATION_EXPERIMENTS_DIR`,
`STATION_FRONTEND_DIST`, `STATION_MAX_CONCURRENT` (2), `STATION_DEFAULT_TIMEOUT`
(1200 s), `STATION_MAX_TIMEOUT` (3600 s), `STATION_KILL_GRACE` (8 s),
`STATION_DOMAIN_MIN/MAX` (100/199).

Variables nuevas (APP): `STATION_MAX_CONCURRENT` es ahora el **tope global** de tareas a la vez (corridas +
renders, 2); `STATION_CUPO_LIGERO|PESADO|GAZEBO|MANIM` (2/1/1/1) y `STATION_UMBRAL_LIGERO|PESADO|GAZEBO|MANIM`
(carga de 1 min: 10/6/4/5; se admite si la carga es **menor**), `STATION_RECHEQUEO_S` (10: cada cuánto se reevalúa
una tarea que espera), `STATION_REINTENTOS` (2) y `STATION_REINTENTO_ESPERA` (30 s, se duplica en cada
reintento, tope 600 s), `STATION_PANTALLA_MIN/MAX` (80/83), `STATION_LP_THREADS` (3, para los renders),
`STATION_RENDER_DIR` (`render/`), `STATION_RENDER_TIMEOUT` (3600 s), `STATION_VIGILANTE_LINEAS` (15), `STATION_LATIDO_TIMEOUT` (60 s, trabajos del host).
El servicio `station` de `docker-compose.yml` pasa los cupos, umbrales, tope y reintentos con esos valores por
defecto (`STATION_UMBRAL_PESADO=9 docker compose up station` los cambia). **Ojo en una máquina ocupada o en CI:** si la carga
del host está por encima del umbral de la clase, las corridas esperan en `esperando_carga` (es lo que se quiere con el
vigilante, pero un E2E con prisa debe subir `STATION_UMBRAL_*`).

## Endpoints (`/api`, CONTRATO §6)

Los de la ronda 3 (abajo) no cambian. La app agrega los de la sección **API de la app** (después de "Decisiones").

| Método | Ruta | Notas |
|---|---|---|
| GET | `/health` | `{ok, ros_distro, active_runs}` (activas = en cola + corriendo) |
| GET | `/experiments`, `/experiments/{id}` | YAML parseado |
| POST | `/runs` `{experiment_id, args?}` | `201 {run_id, status:"queued"}`; 404 experimento; 422 args inválidos |
| GET | `/runs?experiment_id=&limit=` | más nuevas primero (`limit` 1-500, por defecto 50) |
| GET | `/runs/{id}` | lista + `started_at`, `ros_domain_id`, `exit_code`, `error`, `timeout_s`, `launch`, `metrics`, `campaign` |
| DELETE | `/runs/{id}` | borra carpeta y fila; 409 si está en cola o corriendo |
| POST | `/runs/{id}/cancel` | espera a que el grupo muera; 409 si ya terminó |
| GET | `/runs/{id}/telemetry?every=N` | arreglo de muestras (1 de cada N) |
| GET | `/runs/{id}/log` | texto (últimos 2 MB) |
| GET | `/runs/{id}/export.csv` | columnas del CONTRATO §2, vacío si falta el dato |
| GET (SSE) | `/runs/{id}/stream` | `status`, `sample`, `log`, `done` |

Estados: `queued | running | done | error | cancelled | timeout` y, desde el gobernador, `esperando_carga` (hay
cupo pero la carga del host supera el umbral de su clase) e `interrumpida` (la mató una señal externa; sin
`finished_at` es la espera del reintento, con `finished_at` es final). `GET /runs/{id}` agrega `clase`, `reintentos`
y `artefactos`.

El detalle (`GET /runs/{id}`) trae además `kind` y `plan` (el `plan.json` completo, ~100 kB; la
lista no lo trae). Los `kind` nuevos (ronda 3, equipo L):

| `kind` | Experimento | Qué ejecuta | Resultado |
|---|---|---|---|
| `plan` | `s6-plan-24h` | `python3 scripts/planificar.py --sats=… --estaciones=… --inicio=… --horas=… --sin=… --id=<run_id> --salida=<runs>` | `plan.json`; `metrics` = bloque del óptimo + brecha del codicioso |
| `mision` | `m1-dia-operaciones` | `python3 scripts/mision.py --sats=… --estaciones=… --inicio=… --horas=… --n-od=… --id=<run_id> --salida=<runs>` | `informe.json` (CONTRATO 4.2) + `metrics.json` (3.3); una corrida de lazo por contacto en `runs/<id>/contactos/` |
| `adcs` | `e1-adcs-funcube` | `ros2 launch sat_adcs adcs.launch.py … record_dir:=<carpeta>` | `telemetry.jsonl` (1 fila por paso de 1 s) + `metrics.json` |

Para `plan`, `launch` es el script (`planificar.py`, no un `.launch.py`) y los argumentos del YAML pasan
como `--clave=valor` (`margen_acm` → `--margen-acm`; un valor `""` omite el argumento, p. ej. `sin: ""` =
sin comparar con ninguna estación). La fecha de inicio es un arg fijo: el plan es idéntico byte a byte.
Para `adcs`, el escenario completo son ~7000 pasos (unos 2 min con `time_scale: 0`); la variante corta
(`start_utc: 2026-10-02T21:20:00`, `lead_s: 3000`, `w0_deg_s: 3`, ~4000 pasos, ~75 s) es la que usan las pruebas de CI.

**Misión (`kind: mision`, equipo P, `docs/MISION.md`).** `m1-dia-operaciones` ejecuta `scripts/mision.py`:
planifica la red, lanza **una corrida de lazo por contacto** (hasta `concurrencia` a la vez, cada una con su
`ROS_DOMAIN_ID` del pool propio 200-229, que no choca con el del backend 100-199), determina la órbita con lo que
registraron los pases de calibración y escribe `informe.json` + `metrics.json`. El backend solo lo lanza como el plan
(`--k=v`, `--id`, `--salida`) y lo da por `done` cuando existe `informe.json`; el detalle de la corrida trae `mision`
(el informe) y cada contacto se sirve en `GET /api/runs/{id}/contactos/{cNNN}` (+ `/telemetry`, `/log`; el id debe ser
`cNNN` o su gemelo con TLE `cNNN_tle`). Un día de 24 h dura ~1-2 h con la máquina cargada: el YAML pide `timeout_s: 3600`
(el máximo por defecto, `STATION_MAX_TIMEOUT`); la vista `#/mision` muestra el informe y cada fila abre las curvas de su corrida.

**SSE.** No hay `sse-starlette` en la imagen: es un `StreamingResponse` de
`text/event-stream` que cada 0.25 s lee lo nuevo de `telemetry.jsonl` y
`launch.log` (solo líneas completas). Cada conexión **repite la corrida desde el
principio** (así una reconexión no pierde datos ni necesita `Last-Event-ID`):
primero `status`, luego `sample`/`log` a medida que aparecen y, al terminar, un
`status` final seguido de `done` con `metrics` (nulo en campañas) y el cierre
del flujo. Cada 15 s sin datos manda un comentario `: ping`. Una corrida ya
terminada se emite entera y se cierra.

## Decisiones

- **Un worker, cola en memoria.** El `Manager` guarda cola, procesos y dominios en
  memoria; con varios workers cada uno tendría su propia cola. Al arrancar, lo que
  quedó `queued/running` de una ejecución anterior se marca `error`
  (“interrumpida”) en lugar de quedar colgado.
- **Aislamiento DDS por dominio (obligatorio).** La red es `host`: dos corridas con el
  mismo dominio mezclarían `/clock`, `/atp/reference`, etc. Cada corrida recibe un
  `ROS_DOMAIN_ID` al azar del pool 100-199 que ninguna otra activa usa (queda en la
  base, en `params.json` y en la respuesta). La prueba de integración
  `test_dos_corridas_simultaneas_aisladas_por_dominio` lo comprueba con dos pases
  de duración distinta. Límite: otro proceso ajeno de la máquina en el mismo dominio
  no se detecta.
- **Subprocess sin shell.** `argv` como lista; las claves de args pasan por regex y los
  valores son números o miembros de un enum, así que no hay inyección.
- **Args validados contra `params_schema`.** Solo claves declaradas; tipo (`number`,
  `integer`, `enum`, `bool`; un booleano no cuenta como número, `NaN` se rechaza),
  `min`/`max`. Los args del YAML que no están en el esquema son fijos
  (p. ej. `profile: ramp`, `mode: lazo`). Se devuelven los args ya fusionados con los
  valores por defecto.
- **Ids estrictos.** `run_id` debe cumplir `^\d{8}-\d{6}-[0-9a-f]{6}$` y el id de
  experimento `^[a-z0-9][a-z0-9-]{0,62}$`; cualquier otra cosa es 404 sin tocar el
  disco. Los directorios se construyen solo desde ids validados, y el borrado
  verifica que la carpeta cuelgue directamente de `runs/`.
- **Timeout duro y cancelación.** Cada corrida corre en su propia sesión
  (`start_new_session`). Cancelar o expirar envía `SIGINT` al grupo (el `ros2 launch`
  cierra limpio), `SIGKILL` si no sale en `STATION_KILL_GRACE` s y un barrido
  `SIGKILL` final para huérfanos. Un experimento puede fijar su propio `timeout_s` en
  el YAML (se limita a `STATION_MAX_TIMEOUT`).
- **Cuándo una corrida es `done`.** (`kind: proceso`: salida 0 **y** existe `hecho_cuando`.) Cuando existe su resultado (`metrics.json` en lazos,
  `campaign.json` en campañas), con independencia del código de salida; si no,
  `error` con el código. El estado se calcula por archivos, no solo por `rc`.
- **Cierre del lazo con grabador.** Con `record_dir`, `atp_loop.launch.py` hace que sea
  el grabador (al escribir `metrics.json`) quien apague el lanzamiento, y que
  `error_monitor` no salga antes. Sin `record_dir` el comportamiento de siempre.
- **Métricas.** Se calculan con las muestras completas de `/atp/error_deg` (20 Hz, las
  mismas de `error_monitor`) y `/atp/link` (10 Hz), no con el `telemetry.jsonl`
  submuestreado: `n` es el número de muestras de error. El veredicto es
  `CUMPLE` si p95 < objetivo (el mismo criterio de la campaña), si no `NO CUMPLE`.
- **Telemetría.** Una muestra cada 0.1 s de tiempo simulado, sin repetir el mismo `t`
  (la referencia queda congelada en AOS hasta cerrar el lazo). `u_az`/`u_el` son el
  `effort` de `/joint_states`.
- **Fechas** en ISO 8601 UTC (`2026-10-02T06:20:07Z`).
- **Catálogo** se relee de disco en cada petición: editar un YAML no exige reiniciar;
  un YAML inválido se omite con un aviso en el log.
- **CORS** abierto solo para `localhost:5173` / `127.0.0.1:5173` (servidor de
  desarrollo de Vite).

## Gobernador de recursos (APP §3)

Una sola autoridad (`governor.py`) decide qué corre: las corridas del catálogo y los renders le piden turno.

| Clase | Cupo | Se admite si la carga (1 min del host) es | Por defecto |
|---|---|---|---|
| `ligero` | 2 | < 10 | `plan` |
| `pesado` | 1 | < 6 | `loop`, `adcs`, `campaign`, `mision`, `proceso` |
| `gazebo` | 1 | < 4 | renders de Gazebo |
| `manim` | 1 | < 5 | renders de Manim |

- Una escena puede cambiar su clase con `recursos:` en el YAML. Además hay un **tope global** (`STATION_MAX_CONCURRENT`,
  2): una corrida pesada y un render a la vez, no más, con independencia de los cupos.
- Mientras la tarea espera su turno está `queued` (corridas) / `en_cola` (renders) si lo que falta es cupo, y
  `esperando_carga` si hay cupo pero la carga del host supera el umbral de su clase; se reevalúa cada 10 s
  (`STATION_RECHEQUEO_S`) y en cuanto otra tarea termina. El orden es de llegada.
- **Muerte por señal externa** (OOM, el vigilante que detiene el contenedor, un `kill` ajeno: el proceso sale con
  código < 0, o 129/131/137/143 si lo reporta un envoltorio): la tarea pasa a `interrumpida` y se reintenta hasta
  `STATION_REINTENTOS` (2) veces con espera creciente (30, 60, 120 s…). Antes de reintentar se limpia lo parcial
  (la carpeta de la corrida, o de salida del render, queda como recién creada; el log se conserva con una marca
  `[estacion] …`). Un error propio (código > 0), un timeout o una cancelación **no** se reintentan. Si se agotan,
  queda `interrumpida` final. Si el resultado ya existe (por archivos) la corrida es `done` aunque la señal haya
  llegado al final.
- **Pantallas Xephyr** `:80`–`:83`: los trabajos con `pantalla: true` reciben un número libre (marcador `{pantalla}`,
  y `DISPLAY` en el entorno) y lo devuelven al terminar; el script del render arranca el Xephyr, el gobernador solo
  reparte números. Los dominios ROS 100-199 salen del mismo gobernador.
- **Convive con el vigilante del host** (`scripts/vigilante_host.sh`, que detiene los contenedores ROS con carga > 12):
  no lo toca ni lo sustituye; los umbrales de las clases (4-10) quedan por debajo del suyo. La lectura de carga es
  una función inyectable (`create_app(..., load_reader=)`): las pruebas no dependen de la máquina.

## Escenas `kind: proceso` (APP §2)

Una escena nueva es solo un YAML en `experiments/` (sin código en `runner.py`). Campos (todos validados, ver
`experiments._check_experiment`): `comando` (lista = argv, **sin shell**), `estilo_args` (`cli`: `--clave valor`, los
guiones bajos se respetan; un valor que empiece por `-` va como `--clave=valor`; `ros`: `clave:=valor`; un valor `""`
omite el argumento), `hecho_cuando` (obligatorio: archivo relativo cuya existencia marca el fin correcto),
`metricas` (JSON que se lee como `metrics`), `artefactos` (`[{ruta, contrato}]`), `recursos`, `area` (una de
`red seguidor control orbita rf adcs optico mision hardware plataforma`), `timeout_s`, `args` y `params_schema` como en los demás `kind`. Marcadores del comando: `{run_id}`, `{run_dir}`,
`{runs_dir}`, `{domain}`, `{root}` (cualquier otro se rechaza al cargar; las rutas no pueden salir de la carpeta de la
corrida). La corrida es `done` cuando el proceso sale con **0 y** existe `hecho_cuando`; si no, `error` con el motivo.
El comando se congela al lanzar (cambiar el YAML después no altera una corrida en cola). Un YAML inválido se omite
del catálogo con un aviso en el log (`experimento X ignorado: motivo`).

## Renders y medios (APP §4)

`render/*.yaml` (lo escribe el equipo de render; parser en `renders.py`): `requiere` (una sola clave de `contrato`,
`kind`, `archivo`, `cualquiera`), `comando` (mismos marcadores que las escenas más `{salida}`, `{pantalla}`, `{job_id}`;
`{pantalla}` vale `""` si el render no pide pantalla), `estilo_args`, `recursos`, `pantalla`, `duracion_estimada_s`,
`args`, `params_schema`, `ejecutor` (`backend` por defecto | `host`). Cada trabajo escribe en `runs/<run_id>/media/<job_id>/` (con su `render.log`, que es lo que
sirve el SSE) y **debe dejar `media.json`**: si el script termina bien pero falta, o no cumple el contrato, el trabajo
es `error` con el motivo (versión 1; `job_id`/`render`/`run_id` coinciden; cada archivo existe dentro de la carpeta,
con `tipo` ∈ `video|imagen|audio|datos|documento`, `sha256` verificado si viene; todo video lleva `miniatura.jpg`;
`procedencia` con `commit`, `sucio`, `comando`, `args` y `corrida.run_id`). Un script puede imprimir `PROGRESO <0-1>`
y el SSE lo reenvía como evento `progreso`.

### Renders del host: `ejecutor: host` (APP §4.1)

El contenedor `station` no tiene Docker ni pantalla, así que un render declara `ejecutor: host` (por defecto `backend`: lo
lanza el propio backend como subprocess) y lo ejecuta `scripts/render/trabajador_host.py`, un proceso del host que habla
con la API. **El backend nunca lanza esos trabajos** (quedan `en_cola` hasta que alguien los reclama) y **el gobernador
sigue decidiendo**:

1. `POST /api/renders/reclamar {trabajador, clases?}`: recorre los trabajos del host pendientes (más viejos primero) y le
   pide turno al gobernador **sin esperar** (`Governor.try_acquire`: cupo de la clase, tope global, carga y pantalla
   libre; no se adelanta a las tareas del backend que ya esperan en esa clase). Si admite uno, lo pasa a `corriendo`,
   le asigna pantalla (`:80-:83`) y dominio ROS y responde 200 con `{job_id, render, run_id, clase, comando, cwd, salida,
   run_dir, pantalla, entorno, timeout_s, latido_cada_s}`; si no hay nada admisible, 204 (y los trabajos que esperan por
   la carga quedan `esperando_carga`). `comando`, `salida` y `run_dir` van **relativos a la raíz del repo** (la misma en el
   host y en el contenedor, `./:/ws`; `{root}` = `.`), `entorno` trae `DISPLAY`, `ROS_DOMAIN_ID`, `LP_NUM_THREADS`,
   `RENDER_JOB_ID` y `RENDER_ID`.
2. `POST /api/renders/{job}/latido {progreso?, log?: [líneas]}` (cada ≤ `latido_cada_s`): el backend escribe las líneas
   (y `PROGRESO x`) en `render.log`, de donde salen el SSE y el progreso. Responde `{cancelar, estado}`: `cancelar: true`
   si el usuario canceló, el trabajo venció o el backend ya no lo da por suyo; el trabajador mata el grupo y manda `fin`.
3. `POST /api/renders/{job}/fin {exit_code, senal?}`: libera el turno y decide como con un trabajo del backend: valida
   `media.json` (mismo contrato) y termina `hecho` / `error`; si murió por señal externa (`senal` verdadero, o código
   < 0, 129, 131, 137, 143, sin resultado) pasa a `interrumpida` y se reintenta. Un `fin` tardío (trabajo ya cancelado,
   vencido o interrumpido) no cambia nada.

- **Latido perdido:** sin latido en `STATION_LATIDO_TIMEOUT` (60 s) el trabajo queda `interrumpida`, se libera su turno y
  se reintenta (`STATION_REINTENTOS`, espera creciente); agotados, `interrumpida` final. Si el trabajador zombi vuelve a
  latir recibe `cancelar: true`.
- **Cancelar** (`POST /renders/{job}/cancel`) deja el trabajo `cancelado` al instante, pero el turno y la pantalla se
  retienen hasta el `fin` del trabajador (o hasta ~13 s tras avisarle, o el timeout de latido), para no reasignar una
  pantalla mientras el grupo aún muere. Un trabajo que pasa de su `timeout_s` termina `error` por lo mismo.
- 404 / 409 si el trabajo no existe / no es de `ejecutor: host`; 422 en entradas inválidas (`progreso` fuera de 0-1, log
  de más de 2000 líneas, trabajador o clases inválidos). Los trabajos del host se ven igual en `/renders`, `/renders/{job}`
  (con `ejecutor` y `trabajador`), el SSE, la galería y `/sistema`.
- Pruebas: `test_renders_host.py` (trabajador simulado) y `try_acquire` en `test_gobernador.py`.

## API de la app (APP §5)

Todas bajo `/api`, aditivas a las de arriba (errores `{"detail": …}`; ids por regex; rutas de medios confinadas).

| Método | Ruta | Notas |
|---|---|---|
| GET | `/escenas` | `/experiments` + `area`, `recursos`, `artefactos`, `miniatura` |
| GET | `/renders/catalogo` | `render/*.yaml` parseados |
| POST | `/renders/reclamar`, `/renders/{job}/latido`, `/renders/{job}/fin` | trabajador del host (`ejecutor: host`, ver arriba) |
| POST | `/runs/{id}/renders` `{render_id, args?}` | 201 `{job_id, estado:"en_cola"}`; 404 render; 422 args; 409 si la corrida no está `done` o no cumple `requiere` |
| GET | `/renders?run_id=&estado=&limit=` | trabajos (progreso, miniatura), más nuevos primero |
| GET | `/renders/{job}` | detalle + `media` (el `media.json`) |
| POST / DELETE | `/renders/{job}/cancel`, `/renders/{job}` | cancelar (409 si ya terminó); borrar un trabajo final |
| GET (SSE) | `/renders/{job}/stream` | `estado`, `log`, `progreso`, `hecho` (repite desde el principio, como el de las corridas) |
| GET | `/medios?tipo=&escena=&limit=` | galería: principal de cada trabajo `hecho`, con procedencia, miniatura y `url` |
| GET / HEAD | `/medios/{job}/{ruta}` | el archivo con **Range** (206/416), `?descargar=true` |
| GET | `/runs/{id}/export.mcap` | telemetría/red/eventos como MCAP (Foxglove) |
| GET | `/sistema` | carga, memoria, colas, cupos, pantallas, activas y `vigilante` (`null` si no se lee el diario) |

```bash
curl -s localhost:8090/api/escenas | python3 -m json.tool | head -30
curl -s -X POST localhost:8090/api/runs -H 'content-type: application/json' \
     -d '{"experiment_id":"red-walker72-cdmx","args":{"n_sats":24}}'              # una escena kind: proceso
curl -s -X POST localhost:8090/api/runs/<run_id>/renders -H 'content-type: application/json' \
     -d '{"render_id":"gz-demo","args":{"el_max":85}}'                      # -> {"job_id": ...}
curl -N localhost:8090/api/renders/<job_id>/stream                                 # SSE con PROGRESO
curl -s 'localhost:8090/api/medios?tipo=video' | python3 -m json.tool
curl -s -H 'Range: bytes=0-1023' -o /dev/null -D - localhost:8090/api/medios/<job_id>/demo.mp4   # 206
curl -s localhost:8090/api/sistema | python3 -m json.tool
curl -s -o corrida.mcap localhost:8090/api/runs/<run_id>/export.mcap
```

**MCAP.** La imagen no trae la librería `mcap` (y `rosbag2_py` solo escribe mensajes ROS), así que `mcap.py` escribe el
contenedor a mano (variante sin chunks ni CRC, mensajes `json`, un canal por archivo: `/telemetria`, `/red`,
`/eventos`; `log_time` = creación de la corrida + `t`). Las pruebas lo leen con un lector independiente; **no se ha
abierto todavía en Foxglove**.

## Pruebas

```bash
# dentro del contenedor, con ROS cargado (ver scripts/validar.sh)
source /opt/ros/jazzy/setup.bash && cd /ws && colcon build && source install/setup.bash
cd station/backend
python3 -m pytest -q                       # todo (las de la ronda 3 + las de la app + 5 de integración)
python3 -m pytest -q -m "not integration"  # sin lanzar ROS (~1 min)
python3 -m pytest -q -m integration        # solo las reales (~2.5 min: incluye un plan y un ADCS corto)
```

- `test_experiments.py`: catálogo real y validación de args.
- `test_api.py`: toda la API con un lanzador falso (un `python -c` que escribe lo mismo
  que el grabador): flujo completo, SSE, CSV, ids hostiles, máximo 2 simultáneas con
  dominios distintos, timeout, cancelación que mata el grupo, borrado, reinicio.
- `test_integration.py`: `ros2 launch` de verdad. Se omite si no hay `ros2` en el
  `PATH`. Comprueba el **formato** de `telemetry.jsonl`, `metrics.json` y
  `campaign.json`, no el veredicto. Para los `kind` nuevos: `test_plan_real` (`scripts/planificar.py`
  de verdad con 3 h de la red de 4 estaciones: invariantes de `plan.json` re-verificados desde cero,
  `ids_sin` y dos corridas idénticas byte a byte) y `test_adcs_real_corta` (escenario ADCS corto con el grabador).
- De la app (todas sin ROS; la carga del host se simula con `Carga` en `conftest.py`): `test_gobernador.py`
  (cupos, orden de llegada, umbrales por clase, tope global, pantallas, cancelación y carreras),
  `test_gobernador_api.py` (esperando_carga, reintentos tras `SIGKILL`, cancelar en cada fase, `/sistema`),
  `test_proceso.py` (validación estricta del YAML, comando, fin por `hecho_cuando`, E2E con `tests/fixtures/proceso_falso.py`),
  `test_renders.py` (catálogo, contrato de `media.json`, estados, SSE, galería), `test_medios.py` (Range, 416, rutas
  hostiles y enlaces simbólicos), `test_mcap.py`, `test_sistema.py`.
  Las pruebas de la ronda 3 fijan `STATION_CUPO_PESADO=2` (esperan dos corridas a la vez) y la carga en 0.
  `test_los_yaml_reales_del_repo_validan` revisa `experiments/*.yaml` y `render/*.yaml` reales: si un equipo agrega una
  escena o un render con un error, dice cuál.
- `test_api.py` / `test_experiments.py` (también para `plan` y `adcs`): validación de args (claves fijas,
  rangos, enums, inyección), el comando que se construiría (`default_command`), el flujo con un lanzador
  falso, `plan.json` → `metrics`, error si el script falla o no deja `plan.json`, y el CSV aplanado del ADCS.

## Advertencias

- **Fidelidad con el reloj acelerado.** Con `time_scale` alto y la máquina muy
  cargada (varios equipos compartiendo CPU), los nodos no alcanzan el reloj
  simulado y la física puede divergir (errores de cientos de grados). El grabador
  registra lo que pasó; no lo corrige. Si un resultado se ve absurdo, repetir con
  `time_scale` menor o con la máquina libre. Es una propiedad de `sim_clock`, no del
  backend.
- Los dominios 100-199 son del backend: no usarlos a mano en otras pruebas
  mientras la Estación esté corriendo.
- Sin autenticación: solo escucha en `127.0.0.1`. Si sale a internet (sprint 7) hay
  que añadirla y separar la ejecución, como en ManimStudio.
