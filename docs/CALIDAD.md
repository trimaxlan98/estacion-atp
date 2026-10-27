# Calidad de la Estación ATP

> **Versión educativa.** El arnés E2E de navegador (`station/e2e`, Playwright) y sus capturas no se incluyen aquí: dependían de vistas que no forman parte de esta versión. Las secciones que lo mencionan se conservan como registro histórico; escribir un E2E propio sobre el backend y el frontend actuales es un buen ejercicio.


Qué se prueba, con qué, cómo se corre en local y en CI, y qué supuestos entre
frontend y backend están verificados. Se escribió tras la primera prueba
conjunta *real* (navegador contra el backend, sin mock); los hallazgos de esa
prueba están en la sección 6.

## 1. Qué cubre cada suite

| Suite | Dónde | Qué prueba | Cuánto tarda | Necesita ROS |
|---|---|---|---|---|
| `pytest atp_ros` | `src/atp_ros/test` (30) | física: SGP4 contra vectores oficiales (Vallado, `tcppver`), GMST, geodesia, pases (AOS/LOS/TCA, elevación máxima contra fuerza bruta), catálogo TLE y estaciones, geometría esférica, enlace y Doppler, campañas deterministas, keyhole | ~7 s | no |
| `pytest atp_hw` | `src/atp_hw/test` (39 + 2 HIL omitidas) | protocolo `rotctld`, lógica del rotor (envolvente de acimut, flips, banda muerta, limitador de tasa), cliente contra un rotor falso por sockets reales, puente (preposición, tasa máxima, reconexión con backoff, caída en pleno pase, timeout de referencia, halt) | ~10 s | no |
| `pytest atp_hw` HIL | `test_integration_hil.py` (2) | pase completo en tiempo real contra el rotor falso (~80 s cada uno) | ~3 min | sí; solo con `ATP_HW_INTEGRATION=1` (no corre en CI) |
| `pytest sat_adcs` (grabador) | `src/sat_adcs/test/test_grabador.py` (4) | formato de lo que graba `adcs_recorder` (CONTRATO 2.1/3.1): filas JSON válidas sin NaN, campos omitidos, modos, métricas con `null` | ~3 s | sí (importa `rclpy`, `sat_interfaces`); corre en el job `pruebas`. La suite completa de `sat_adcs` (41 pruebas, ~6 min) no corre en CI |
| `pytest station/backend` unitarias | `station/backend/tests` (102) | catálogo y validación de args (también `plan` y `adcs`), el comando real por `kind`, API completa con un lanzador falso (flujo, SSE, CSV, ids hostiles, máximo 2 simultáneas, timeout, cancelación que mata el grupo, borrado, reinicio), `plan.json` → `metrics`, CSV aplanado del ADCS | ~20 s | no |
| `pytest station/backend -m integration` | `test_integration.py` (5) | `ros2 launch` de verdad: formato de `telemetry.jsonl`, `metrics.json`, `campaign.json`; dos corridas simultáneas aisladas por dominio DDS; **`kind: plan`** (`scripts/planificar.py` con 3 h de la red, invariantes re-verificados desde cero y dos corridas idénticas byte a byte) y **`kind: adcs`** (escenario corto con el grabador) | ~2.5 min | sí |
| `scripts/validar.sh` | escalera N0–N5, S4 y determinismo | los resultados de referencia de la escalera (0.017° rms en N2, p95 en N4, ISS real, autotrack N5...) y que dos corridas iguales dan cifras idénticas | minutos | sí |
| Frontend `npm run build` | `station/frontend` | que el bundle compile (no hay pruebas unitarias del frontend) | ~15 s | no |

Las pruebas con subprocesos reales (integración y E2E) usan dominios DDS 100–199
(el pool del backend); a mano usar `ROS_DOMAIN_ID=73` para no pisarlos.

## 2. Cómo correr todo en local

Todo dentro del contenedor (`./run.sh` o `scripts/ci_ros.sh`, que es lo que usa el CI):

```bash
docker compose build                          # una vez
./run.sh bash scripts/validar_todo.sh         # build + las tres suites pytest + toda la escalera

# por piezas, con el mismo envoltorio que el CI (corre como el usuario `dev`, sin TTY):
scripts/ci_ros.sh 'colcon build 2>&1 | tail -5'
scripts/ci_ros.sh 'python3 -m pytest -q src/atp_ros/test src/atp_hw/test'
scripts/ci_ros.sh 'cd station/backend && python3 -m pytest -q'            # incluye las 3 de integración
scripts/ci_ros.sh 'bash scripts/validar.sh test n2 det_lazo'              # subconjunto rápido
ATP_HW_INTEGRATION=1 scripts/ci_ros.sh 'python3 -m pytest -q -s src/atp_hw/test/test_integration_hil.py'

# E2E navegador <-> backend (construye el frontend si falta, levanta `docker compose up station`
# en STATION_PORT=8291, corre Playwright y detiene el servicio al terminar):
./station/e2e/run.sh
```

El E2E recorre además (ronda 3): la **montura 3D** (WebGL por software con `--use-angle=swiftshader`;
pose contra la telemetría, deslizador, render bajo demanda y una prueba con 12 000 muestras), **plan →
Gantt** (`s6-plan-24h` de 6 h lanzado desde el formulario, con/sin una estación, zoom, biblioteca, CSV) y
**adcs → vista ADCS** (variante corta de `e1-adcs-funcube`: en vivo, modos, umbral, ruedas, CSV, registro,
sin procesos huérfanos). `E2E_RUN_TIMEOUT_MS` (240000 por omisión) fija cuánto se espera a cada corrida:
con la máquina muy cargada hay que subirlo (ver 6.7).

Requisitos del E2E en el anfitrión: Node 24, `docker compose`, y Chromium de
Playwright (`~/.cache/ms-playwright/` o `PW_CHROMIUM=/ruta/chrome`; se instala con
`cd station/frontend && npx playwright-core install chromium`). Deja
`station/e2e/resultado.json` y las capturas en `station/e2e/capturas/`. Todas las
corridas que crea se borran al final. `E2E_KEEP=1` deja el servicio levantado;
`E2E_NO_DOCKER=1` omite la comprobación de procesos huérfanos tras cancelar.

`scripts/validar.sh` acepta niveles sueltos: `test n1 n2 n4a n4b n4c s4 n5a n5b det`
y, desde ahora, `det_lazo` (solo el lazo N2) y `det_n5` (solo el autotrack).

## 3. CI (`.github/workflows/ci.yml`)

Se dispara en cada `push`, en `pull_request` y a mano (`workflow_dispatch`);
corre en `ubuntu-latest`.

| Job | Qué hace |
|---|---|
| `frontend` | Node 24, `npm ci`, `npm run build`; sube `dist/` como artefacto |
| `imagen` | `docker buildx` de `docker/Dockerfile` con **caché de capas** en Actions (`type=gha`, `mode=max`); comprueba ROS y las dependencias Python. Es el único que escribe la caché |
| `pruebas` (needs `imagen`) | `colcon build`; pytest de `atp_ros`, `atp_hw`, del grabador de `sat_adcs` y del backend (sin integración); `validar.sh test n2 det_lazo` comprobando su salida (el script nunca devuelve error: se busca `Traceback`, `failed`, `NO DETERMINISTA` y que haya `RESULTADO`); `validar.sh det_n5` bloqueante (hallazgo 6.4, corregido) |
| `integracion` (needs `imagen`) | `colcon build` y `pytest station/backend -m integration` (ROS real); si algo falla reintenta una vez solo lo fallido con un aviso (hallazgo 6.5) |

Decisiones:

- **No se tocó `docker/Dockerfile`.** El contenedor corre como `dev` (UID 1000
  por defecto) y el workspace se monta en `/ws`; el runner tiene UID 1001, así
  que la imagen se construye con `--build-arg UID=1001 GID=1001` (el job
  `imagen` falla si el runner dejara de ser 1001). Las capas de `apt`, que son
  las caras, no dependen de esos argumentos y salen de la caché.
- `scripts/ci_ros.sh '<comando>'` ejecuta un comando en la imagen con `/ws`
  montado, `--network host` (DDS) y falla al primer error: es lo que usan los
  pasos del CI y sirve igual en local.
- Se libera disco al principio (`dotnet`, `android`, `ghc`): la imagen ocupa ~6 GB
  y el runner trae poco espacio libre.
- Las pruebas HIL de `atp_hw` (80 s de tiempo real cada una) no corren en CI.
- Los jobs que necesitan la imagen la reconstruyen desde la caché (segundos);
  cada uno hace su propio `colcon build` (~1 min) porque `build/` e `install/`
  viven en el volumen montado de cada job.

Estado real de la ejecución: ver la sección 7.

## 4. Pruebas frágiles de `atp_hw` (la «1 de 39» eran dos)

Método: la suite (`src/atp_hw/test`) y cada prueba sospechosa por separado, en
bucle, con 24 procesos de CPU compitiendo dentro del contenedor, en una máquina
ya cargada (load average 25–60).

**1. `test_puente_cae_el_rotor_en_pleno_pase_y_vuelve` (la frecuente).**
Falló en 14 de 60 ejecuciones aisladas y en 1 de 5 de la suite completa
(`assert 'DISCONNECTED' == 'TRACKING'`, `test_client_fake.py:264`). La prueba
corta el socket y espera `stats['connects'] >= 2`; justo después exigía
`core.state == 'TRACKING'`. Pero el puente sube `connects` al abrir el socket y
restaura el estado **después**, tras el `\dump_state` de la reconexión (otro
viaje de ida y vuelta). Con la máquina libre ese intervalo son microsegundos y
casi siempre se ganaba la carrera; con carga, el hilo de la prueba despertaba
entre las dos cosas. No es un fallo del puente: el estado final es el correcto.
**Arreglo:** la prueba espera al estado (`_esperar(lambda: core.state ==
'TRACKING')`), no al contador. Tras el arreglo: 0 fallos en 60.

**2. `test_puente_tasa_maxima_de_ordenes` (la rara).** Falló 1 de 15 en la suite
completa bajo carga 60 (`assert 8 <= 6`): exigía entre 8 y 12 órdenes en 2 s de
pared, y con el hilo del puente atrasado salieron 6. **Arreglo:** se conserva lo
que la prueba quiere demostrar (el limitador nunca pasa de 5 Hz) como cota
superior que no depende de la carga (`n <= 5·transcurrido + 2`, con el tiempo
medido después de leer `n`), y la cota inferior pasa a ser solo de vivacidad
(`n >= 3`, esperando hasta 10 s). Una prueba que sigue detectando un limitador
roto (sin él salen cientos de órdenes) pero ya no depende de que la máquina
vaya holgada. Tras el arreglo: 0 fallos en 50 repeticiones del grupo de pruebas
de tiempo.

No se tocó código del puente. Las otras pruebas de `atp_hw` que usan el reloj de
pared (`timeout_de_referencia_manda_stop`, `halt`, `envia_stop_al_salir`, el
timeout del cliente `0.25 < dt < 1.0`) no fallaron en 50 repeticiones bajo
estrés; son las siguientes candidatas si reaparece algún falso negativo
(en `timeout_de_referencia` el contador `ref_timeouts` sube *antes* de enviar `S`,
la misma clase de carrera que la del punto 1).

## 5. Matriz de supuestos frontend ↔ backend

El README del frontend enumera cinco supuestos explícitamente; el código del
frontend (`api.js`, `useRunLive.js`, las vistas) hace más. Aquí están los 17
que se verifican, cada uno contra el backend real (`station/e2e/e2e.mjs`, los
identificadores `S1..S12` aparecen en el informe `resultado.json`).

| # | Supuesto del frontend | Cómo se verifica | Resultado |
|---|---|---|---|
| S1 | El backend sirve `dist/` en `/` con rutas relativas y `GET /api/health` devuelve JSON con `ok:true`; si no, el frontend cae a mock | `index.html` con `./assets/`; el badge dice «backend en línea», no «MODO MOCK» | OK |
| S2 | `GET /api/experiments` trae `kind` (`loop`/`campaign`), `params_schema` con tipos que el formulario sabe pintar, y **cada parámetro tiene valor por defecto en `args`** | catálogo de 9; ninguna tarjeta con *Lanzar* inhabilitado | **DESAJUSTE** (n2 `vel_max`, n4a `eirp_dbw`), corregido |
| S3 | `POST /api/runs {experiment_id, args}` → 201 `{run_id}`; los errores traen `detail` en texto (404, 422) | alta desde el formulario; 404 experimento, 422 rango, 422 clave inventada | OK |
| S4 | El detalle de una corrida trae `metrics` (lazo) o `campaign` (campaña), `args`, `title`, fechas | detalle de ambos tipos; `metrics` nulo en campañas | OK |
| S5 | La telemetría REST y la SSE usan los nombres de campo del contrato, `t` crece estrictamente, los campos sin dato se **omiten**, y el historial REST es prefijo de lo que repite la SSE (descarta por `t`) | 503 muestras: `t` monótono, REST a mitad de corrida es prefijo de la SSE, `every=N` submuestrea; trazos SVG válidos | **DESAJUSTE** (cielo polar con trazo inválido por la 1.ª muestra sin `az/el`), corregido |
| S6 | La SSE emite `status`, `sample`, `log` y `done`; cada conexión repite la corrida desde el principio; el servidor cierra tras `done` | cliente SSE directo contra una corrida activa y otra terminada; en la UI, «EN VIVO» y el tiempo simulado avanza | OK |
| S7 | Tras `done`, `GET /api/runs/{id}` ya trae `metrics`/`campaign` | detalle inmediatamente después de `done`; las cifras de la UI coinciden con las de la API | OK |
| S8 | Con `metrics.n > 6000` pide `telemetry?every=ceil(n/6000)` y el backend lo atiende | n4a: `n=10898` → `every=2`; la petición real del navegador y la longitud devuelta | OK (ver nota) |
| S9 | `campaign.json` trae `p50 p95 p99 peor pasa objective_deg p95_uncertainty_pts wall_time_s rms_deg el_max_deg mode n` y los arreglos tienen `n` elementos | campaña analítica (n=2000) y de lazo (n=10); la UI muestra «±0.5 pts» = `p95_uncertainty_pts` | OK |
| S10 | `GET /api/runs?limit=300` devuelve el resumen (con `metrics` en lazos, **sin** `campaign`: el frontend la completa con el detalle) y filtra por `experiment_id` | lista y filtro; la fila de la biblioteca y su p95 coinciden con la API | OK |
| S11 | `POST /cancel` deja la corrida `cancelled` (409 si ya terminó), `DELETE` de una activa da 409, `active_runs` de `/health` vuelve a 0 | cancelar desde la UI una corrida a tiempo real; el botón *Borrar* de una activa está inhabilitado; sin procesos que sobrevivan (`pgrep` dentro del contenedor) | OK |
| S12 | `export.csv` es un adjunto con la cabecera del CONTRATO §2 y una fila por muestra; `DELETE` borra fila y carpeta (luego 404) | descarga real desde el navegador; cabecera, nombre `<run_id>.csv`, filas = muestras, RMS recalculado del CSV = `metrics`; carpeta ausente tras borrar | OK |
| S13 | El detalle de una corrida trae `kind` (`loop/campaign/plan/adcs`) y, en `plan`, el `plan.json` completo; el frontend elige la vista por él (la lista no lo trae: usa el catálogo) | `kind` y `plan` del detalle; el Gantt, el resumen y la tabla de algoritmos de la UI contra `metrics` y `plan.json` | OK |
| S14 | En `plan.json`, los ids de `planes.*.ids` y de `aporte_estacion.<est>.ids_sin` son ids de `ventanas` (una barra por contacto, una fila por estación), y los GB de las barras suman `metrics.gb` | barras = contactos, contactos por estación, suma de GB, «sin la estación» = `contactos_sin` y 0 barras en su fila | **DESAJUSTE** (`ids_sin` no existía y `subred` renumera las ventanas), corregido |
| S15 | La telemetría de una corrida ADCS es una fila por paso de 1 s (no 10 Hz), `t` desde 0, `q` de 4 y `h_nms` de 3, campos sin dato omitidos, y `metrics.n` = número de filas; `metrics.modes` cubre el escenario | 4033 filas = `metrics.n`; bandas de la tira = `modes`; p95 de TRACK_GS, % de ruedas y veredicto de la UI contra la API; CSV aplanado | OK |
| S16 | Un `enum` puede traer `labels` y un valor `""` (el formulario lo pinta como «ninguna» y se envía vacío; el backend lo valida contra `options` y omite el argumento) | tarjeta del plan, `422` para `sin: atlantida`, lanzamiento sin comparar | OK |
| S17 | La montura 3D usa `az/el` y `ref_az/ref_el` de la telemetría (la 1.ª muestra no los trae: se usa la última que sí) y no depende del número de muestras | `data-az/el` = última muestra; el deslizador va a la muestra elegida; 0 cuadros en 1.5 s de reposo; con 12 000 muestras 60 fps de rAF en reposo y 57 pasos/s barriendo el deslizador | OK |

Notas:

- **S8.** `metrics.n` cuenta las muestras de error a 20 Hz, pero la telemetría
  se graba a 10 Hz: para n=10898 hay ~5170 muestras y con `every=2` llegan ~2590.
  Funciona, solo submuestrea el doble de lo necesario. No se cambió.
- **S12.** El CSV usa fin de línea CRLF (`csv.DictWriter` por defecto, válido
  según RFC 4180); no afecta al frontend.
- La biblioteca no se refresca sola al volver a ella con el mismo hash (solo
  sondea mientras haya corridas activas): quien lanza algo por API y mira la
  biblioteca debe recargar. Comportamiento del frontend, no un fallo del contrato.
- El README del frontend lista cinco supuestos; los otros siete se
  dedujeron de leer el código (ver `api.js`, `useRunLive.js`, `Biblioteca.jsx`).

## 6. Hallazgos de la integración real

1. **Catálogo: parámetros sin valor por defecto (corregido).** `n2-keyhole-cenital`
   (`vel_max`) y `n4a-pase-oblicuo-s` (`eirp_dbw`) declaraban el parámetro en
   `params_schema` pero no en `args`. El formulario del frontend se inicializa
   con `args[k]`: el campo quedaba vacío («Valor numérico requerido») y el botón
   *Lanzar* inhabilitado, es decir, **no se podía lanzar desde la UI el
   experimento principal de la demostración** (captura
   `station/e2e/capturas/00-desajuste-n2-lanzar-deshabilitado.png`). El mock no lo
   mostraba. Arreglo: se añadieron los valores por defecto (`vel_max: 0`,
   `eirp_dbw: 9.0`, los mismos que `atp_loop.launch.py`, sin cambio de
   comportamiento) y la prueba del catálogo (`test_experiments.py`) exige ahora
   que todo parámetro editable tenga default.
2. **Frontend: cielo polar con trazo de antena inválido (corregido).** El
   backend omite los campos sin dato (CONTRATO §2) y la primera muestra solo trae
   `ref_az/ref_el`. `PolarSky` elegía `M` o `L` según el índice de la muestra, no
   según el contenido del trazo, así que la traza de la antena empezaba por `L…`:
   SVG inválido, no se dibujaba y el navegador registraba un error por cada
   render (decenas por corrida). Se cambió a «`M` en el primer punto válido de
   cada trazo» (`station/frontend/src/charts/PolarSky.jsx`, 2 líneas). El mock
   siempre traía todos los campos, por eso no se vio antes.
3. **Pruebas frágiles de `atp_hw` (corregidas):** sección 4.
4. **El autotrack N5 no era determinista con CPU escasa (CORREGIDO por el
   integrador; ver el cierre al final de este punto).** En el primer CI (2 vCPU) `validar.sh det` pasó el
   lazo N2 pero falló el autotrack: dos corridas idénticas dieron RMS 0.0045 y
   0.0043; en el segundo run salieron idénticas y en el tercero volvieron a
   diferir (2 de 3 runs): depende de la carga del momento. Se reprodujo en local limitando el contenedor a 2 CPU
   (`--cpuset-cpus`): cuatro corridas, cuatro cifras distintas (rms 0.0036–0.0044),
   mientras que con CPU holgada salen idénticas. Causa (medida): `sim_clock`
   avanza libre a tiempo real hasta la primera confirmación (diseño para HIL),
   así que el instante en que arranca el lockstep varía con la carga (primer
   par del *beacon* en `s = 3.80, 3.95, 4.35 s` en tres corridas), y `rf_beacon`
   calcula la fase del *conical scan* con el tiempo simulado **absoluto**
   (`fase = w·s`). La fase de arranque (3.14 rad frente a 5.50 rad) cambia el
   resultado; las dos corridas con la misma fase dieron cifras casi iguales. El
   lazo N2 no tiene esa dependencia y es determinista también en 2 vCPU.
   Arreglo propuesto (no hecho, es código de `atp_ros`): referir la fase del scan
   al inicio del pase (el primer par o `t_ini` del generador) en vez de al reloj
   absoluto, o arrancar el reloj en `k = 0` sin giro libre cuando hay montura
   simulada. **Cierre (integrador):** se hizo lo segundo y algo mas. El reloj
   ya no gira libre: una compuerta lo detiene en el primer paso hasta la
   primera confirmacion, y el generador solo publica cuando todos los nodos
   anunciaron en `/atp/listo` que sus suscripciones estan emparejadas (CONTRATO
   §8, regla 4). Ademas error, enlace y grabador emparejan por sello
   (`/atp/pointing_error`). Medido: 4 de 4 pares del autotrack con grabador
   dieron `telemetry.jsonl` y `metrics.json` identicos byte a byte (3115
   filas), y el lazo N2 tambien (511 filas). `det_n5` es ahora bloqueante en CI.
5. **El grabador perdió el arranque de un pase en CI (CORREGIDO: era la misma
   carrera de arranque del punto 4; el saludo de arranque la cierra).** En el
   segundo run de CI, `test_n2_real` (integración del backend, `time_scale=8`)
   falló porque `telemetry.jsonl` empezaba en `t = 15.85 s` y no en 0: el
   grabador no registró el primer tercio de un pase de 51 s (nótese que
   `metrics.json` se calcula aparte y sí estaba completo). Pasó en el primer run
   de CI, en el job E2E (que lanza el mismo pase) y 6 de 6 en local con el
   contenedor limitado a 2 CPU y carga, así que no se pudo reproducir ni medir la
   causa. Hipótesis sin confirmar: arranque en frío del primer `ros2 launch` en el
   runner; el generador espera a que las suscripciones a `/atp/sat_state` estén
   *emparejadas*, no a que el grabador ya esté girando, y con el reloj a ×8 dos
   segundos de retraso son 16 s de pase. Es comportamiento de `atp_ros`
   (`recorder.py` / `wait_subscribers`), no se tocó. **Mitigación en CI:** el
   paso de integración reintenta una vez solo lo que falló (`--lf`) y emite un
   `::warning::` visible. Si reaparece con frecuencia, la salida correcta es que el
   generador espere a una confirmación del grabador, no relajar la prueba.

6. **El grabador de ADCS cambia el saludo de arranque (diseño, no fallo).** Con el grabador la planta
   y el controlador esperan **3** suscriptores en vez de 2 (CONTRATO 8, regla 4); si no, el grabador
   se perdería el principio. `adcs.launch.py` lo pone solo cuando hay `record_dir`; sin él todo queda como estaba.
   Verificado con la corrida ROS real (4033 filas desde `t = 0` y `metrics.n` = filas).
7. **Una corrida a reloj máximo (n4a) agotó los 240 s del E2E con la máquina compartida (load average 36–45).**
   No es del producto: repetido con `E2E_RUN_TIMEOUT_MS=900000` pasó. Es la misma sensibilidad a la carga que ya
   describe la advertencia de fidelidad del reloj acelerado; por eso el CI pasa 600000.
8. **Aviso para `plan`:** `plan.json` pesa ~100 kB y el detalle lo trae entero (la lista no). Con muchas
   corridas de plan abiertas a la vez, conviene paginar o servirlo aparte; hoy no hace falta.

## 7. Estado verificado (2026-10-02)

**E2E real** (`./station/e2e/run.sh`, local, Node 24, Chromium de Playwright,
backend con ROS Jazzy en `docker compose`, puerto 8291): **61/61 comprobaciones
OK**; informe en `station/e2e/resultado.json`, capturas en `station/e2e/capturas/`.
Antes de los dos arreglos de la sección 6 el E2E fallaba en el primer paso (no se
podía lanzar n2 desde el formulario) y, ya lanzado, registraba errores de consola
del cielo polar. La misma prueba pasa también en CI (job `e2e`).

**Ronda 3 (equipo L), E2E real local** (`E2E_RUN_TIMEOUT_MS=900000 STATION_PORT=8391 ./station/e2e/run.sh`):
**100/100 comprobaciones OK** (61 de antes + 39 de montura 3D, plan → Gantt y ADCS → vista ADCS), con el
servicio sobre ROS Jazzy real y la máquina compartida (load average 36–45). Un primer intento con el tiempo por defecto
falló solo en la corrida larga n4a por la carga (hallazgo 6.7). Capturas nuevas en `station/e2e/capturas/`:
`03b-montura-3d`, `08b-montura-3d-12k`, `11-plan-gantt`, `12-plan-sin-estacion`, `13-adcs-en-vivo`, `14-adcs-terminada`, `15-vista-adcs`.

**CI** (`ubuntu-latest`, repositorio privado, minutos de Actions disponibles):

| Run | Resultado | Qué pasó |
|---|---|---|
| 36978538640 | falló | todo bien salvo `validar.sh det`: el autotrack N5 no fue determinista (6.4) |
| 36983558576 | falló | `test_n2_real` de integración: el grabador no registró el primer tercio del pase (6.5); el resto, verde |
| 36986108971 | **verde** (5 jobs) | `det_n5` informativo volvió a mostrar «NO DETERMINISTA» sin bloquear; la integración pasó sin reintento |

Tiempos aproximados (runner de 2 vCPU): imagen 4 min con caché de capas (9 min la
primera vez), pruebas 7 min, integración 5 min, E2E 7–8 min, frontend 15 s; el
workflow completo ronda los 12 min.

**No verificado / pendiente:** ~~el determinismo del autotrack N5 con CPU escasa y
la pérdida del arranque del grabador (6.4 y 6.5)~~ cerrados(ronda 2: medición sellada, compuerta y
saludo de arranque); confirmado otra vez el 2026-10-05 sobre el código integrado de la ronda 5 con el host cargado
(`validar.sh det`: N2 511 filas y N5 3115 filas idénticas byte a byte). Siguen pendientes: las pruebas HIL
de `atp_hw` no corren en CI; el frontend no tiene pruebas unitarias propias; el E2E
se probó solo en Chromium.
