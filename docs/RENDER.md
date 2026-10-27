# RENDER — del run al medio (equipo AC de la app)

Parte de la app de [APP](APP.md) (§4 y §7): los **renderizadores** convierten una corrida (`runs/<run_id>/`) en medios
(video, imágenes) con la marca Co.De Aerospace y con su procedencia. Cada uno es un **envoltorio no interactivo**
`scripts/render/<id>.sh` que el backend lanza sin humano, descrito en `render/<id>.yaml`. Este documento es el contrato de
los envoltorios, el catálogo, cómo añadir uno y lo aprendido con Gazebo por software.

```
render/<id>.yaml ──(backend: gobernador)──▶ scripts/render/<id>.sh <run_dir> <salida> [<pantalla>] [--clave valor …]
                                              │  _comun.sh: argumentos, pantalla del pool, carga, progreso, trap
                                              ├─ gz-demo.sh        ──▶ scripts/gz_demo.sh   (Xephyr + contenedor gz)
                                              ├─ manim-corrida.sh  ──▶ contenido/exportar.py + contenido/render.sh (contenedor Manim)
                                              └─ graficas.sh       ──▶ scripts/render/graficas.py (matplotlib)
                                              ▼
                              <salida>/  medios + media.json + miniatura.jpg   (+ logs/)
```

## 1. Catálogo

| id | Requiere (`requiere`) | Clase | Pantalla | Duración típica | Medio principal | Argumentos (`--clave valor`) |
|---|---|---|---|---|---|---|
| `gz-demo` | corrida de lazo (`kind: loop`: `telemetry.jsonl` + `metrics.json`) | `gazebo` | sí | 6 min (lo que dura el pase en tiempo real + arranque) | `demo.mp4` 1920×1080 + `eval.json` | `el_max`, `mask`, `band`, `h_km`, `az_culmination`, `res`, `lp_threads` |
| `manim-corrida` | lazo (`pase`) o campaña (`campana`) | `manim` | no | 4 min | `pase.mp4` / `campana.mp4` (480p15, 720p30 o 1080p60) + `cifras.json` | `escena`, `calidad`, `play_s`, `cpus` |
| `graficas` | cualquiera con `telemetry.jsonl` | `ligero` | no | 20 s | `lamina.png` 1920×1080 + una PNG 1600×900 por gráfica | `titulo` |

* **gz-demo** repite el pase de la corrida en Gazebo con la montura real (no reproduce su telemetría: la vuelve a simular
  con física; `el_max` y `mask` salen de su `params.json` o, si faltan, del máximo de `el` y el mínimo de `ref_el`; la banda,
  de su `metrics.json`). Las cifras rotuladas son las **medidas en Gazebo** (`eval.json`), no las de la corrida de origen.
* **manim-corrida** usa el puente de `contenido/` ([README](../contenido/README.md)): exporta la corrida a un dataset
  **temporal** (`contenido/datasets` no se toca), renderiza con `contenido/render.sh` (imagen `codeaerospace_contenido-manim`,
  repo `codeaerospace_contenido`, ambos en la máquina) y copia el registro de cifras rotuladas. La escena `autotrack`
  (dos corridas) no es de este render.
* **graficas** reconoce lazo, ADCS (`kind: adcs`) y cualquier otra telemetría (hasta 6 columnas). Usa matplotlib del
  host si lo hay; si no, del contenedor `ros2-jazzy-dev` (sin red, la corrida en solo lectura, 3 GB).

## 2. Contrato de un envoltorio

```
scripts/render/<id>.sh <run_dir> <salida> [<pantalla>] [--clave valor …]      # también --clave=valor
```

* `<run_dir>`: la corrida (solo lectura). `<salida>`: carpeta del trabajo, `runs/<run_id>/media/<job_id>/`; se crea.
  `<pantalla>` (opcional, `:80`–`:83`): el gobernador la asigna; sin ella el envoltorio toma la primera libre del pool.
* Las claves llevan guiones o guiones bajos indistintamente (`--n_estela` = `--n-estela`); el valor `true` si va sin valor.
* **`PROGRESO <0-1>`** en la salida estándar, una línea por avance (el último, `PROGRESO 1`, solo si terminó bien).
* **Siempre un `media.json`** (§3), incluso si falla: un trap de salida lo escribe con `estado` `error` o `parcial` y lo
  que haya. Si el uso es incorrecto antes de tener carpeta de salida no hay `media.json` (código 2).
* **Errores claros** a stderr, `ERROR: …`, y el mismo texto en `media.json.error`: «la corrida no cumple el requisito de
  gz-demo: debe ser un lazo (telemetry.jsonl + metrics.json); esta es: campaña (…)».
* **Códigos de salida:** `0` hecho · `2` uso o la corrida no cumple el requisito · `3` la carga del host no bajó (el
  envoltorio espera hasta `MAX_ESPERA_MIN`, 90, antes de rendirse) · `4` sin pantalla libre en el pool · **`143` interrumpido**
  (SIGTERM —cancelación del trabajador— o el vigilante del host que detuvo el contenedor; `media.json` trae `estado: parcial` y lo
  alcanzado; el backend reconoce 143/137 como muerte por señal externa y **reintenta**; se detienen el contenedor y la pantalla) ·
  otro: fallo (`estado: error`).
* **Espera de carga:** `gz-*` esperan carga ≤ 4 (como el gobernador, §3 de APP); `manim-corrida` ≤ 5; `SIN_ESPERA_CARGA=1` lo omite.
* **Nombres únicos:** el contenedor se llama `atp-render-<id>-<job>-<pid>` (job = `$RENDER_JOB_ID` —lo da el backend—, `$JOB_ID` o el nombre de la carpeta de salida) y es
  también su partición de gz-transport (`GZ_PARTITION`): dos trabajos no se ven entre sí. El dominio ROS es `$ROS_DOMAIN_ID` (83
  por omisión en los renders Gazebo).
* **Carpeta de trabajo:** lo intermedio (video crudo, rótulos, mundo SDF, datasets de Manim) va en `<salida>/_trabajo/` (dentro
  del repo: el contenedor lo monta en `/ws`) o, si la salida está fuera del repo, en `runs/_render/<job>/`; se borra al terminar
  (`CONSERVAR_TRABAJO=1` lo deja) y los logs útiles pasan a `<salida>/logs/`.
* **Miniatura:** `miniatura.jpg` 640×360 (con bandas si no es 16:9), de un fotograma al 92 % del video o de la imagen principal.

## 3. `media.json`

Lo escribe `scripts/render/media_json.py` (que también lo **valida**: `python3 scripts/render/media_json.py --validar <carpeta>/media.json`).
Es el de APP §4, con tres campos aditivos (`estado`, `error`, `bytes`):

```json
{"version": 1, "job_id": "prueba-demo", "render": "gz-demo", "run_id": "lazo-n2",
 "creado_utc": "2026-10-06T05:59:46Z", "duracion_pared_s": 62.0,
 "estado": "ok",                       // ok | parcial | error
 "archivos": [
   {"ruta": "demo.mp4", "tipo": "video", "ancho": 1920, "alto": 1080, "duracion_s": 23.3,
    "sha256": "…", "bytes": 638923, "principal": true},
   {"ruta": "miniatura.jpg", "tipo": "imagen", "ancho": 640, "alto": 360, "sha256": "…", "bytes": 17758}],
 "procedencia": {"commit": "git rev-parse HEAD", "sucio": true,
                 "comando": ["bash", "scripts/render/gz-demo.sh", "runs/lazo-n2", "…", ":80", "--el_max", "85"],
                 "args": {"el_max": 85, "mask": 70, "res": "1920x1080"},
                 "corrida": {"run_id": "lazo-n2", "experiment_id": "…", "args": {…}}},   // de params.json si existe
 "notas": "texto libre (qué se ve, líneas FIN del reproductor, cifras medidas)",
 "error": "solo si estado ≠ ok"}
```

`tipo` ∈ `video | imagen | datos`; `principal` marca **un** archivo (el medio que la galería muestra); `args` incluye los valores por
omisión aplicados, de modo que el medio es reproducible con ese comando sobre esa corrida y ese `commit`. `sucio` es verdadero si
el árbol tenía cambios sin confirmar (incluidos archivos nuevos). Con `estado: error` puede no haber principal.

## 4. Pool de pantallas

Cada render de Gazebo dibuja en una pantalla **Xephyr** propia de la resolución pedida (por omisión 1920×1080), para no depender de
ningún monitor. Pool `:80`–`:83`; `:77` (`scripts/gz_demo.sh` a mano) **no se usa** desde los
envoltorios (lo rechazan: `RENDER_PANTALLAS_FIJAS=1` los permite). El gobernador pasa la pantalla como tercer argumento; el
envoltorio toma además un cerrojo `flock` (`/tmp/atp-render-pantalla-<n>.lock`) y falla con código 4 si otro trabajo la tiene, o
elige la primera libre si no se la dan. `scripts/gz_demo.sh` solo ganó variables de entorno (`SALIDA`, `PANTALLA`,
`NOMBRE`, `DOMINIO_ROS`, `PROGRESO`, `DUR_EST`); sin ellas hacen exactamente lo de antes (`:77`, `runs/gz_demo`, contenedor `gz_demo`). `contenido/render.sh` ganó `MEDIA_DIR`, `DATASETS_DIR` y `NOMBRE_CONTENEDOR`.

## 4.1 Trabajador del host (`scripts/render/trabajador_host.py`)

El backend corre en el contenedor `station` (sin Docker ni pantalla), así que los renders de Gazebo, Manim y gráficas declaran
`ejecutor: host` en su YAML y los ejecuta este proceso del **host** (Python 3 estándar, sin dependencias) hablando con la API
(APP §4.1): `POST /api/renders/reclamar {trabajador, clases}` → el trabajo admitido por el gobernador (`job_id`, `render`, `comando`
ya resuelto, `salida`, `pantalla`, `entorno`) o 204; lo ejecuta **desde la raíz del repo** en un grupo de procesos propio con
`DISPLAY=<pantalla>` y el `entorno` del backend (`ROS_DOMAIN_ID`, `LP_NUM_THREADS`, `RENDER_ID`, `RENDER_JOB_ID`); `POST …/latido
{progreso, log}` cada 8 s (límite del backend, 10 s) con el último `PROGRESO` y las líneas nuevas de la salida (que además va a
`<salida>/render.log`); si el latido responde `{"cancelar": true}` (o el backend ya no espera el trabajo, o se agota `timeout_s`) manda
SIGTERM al grupo —los envoltorios limpian contenedor y pantalla— y SIGKILL a los 15 s; `POST …/fin {exit_code, senal?}` (una muerte
por señal se informa como `128+N` y `senal: N`, que el backend reconoce para reintentar). Sin backend durante `--tolerancia` (90 s) detiene el
trabajo. SIGTERM al trabajador detiene el trabajo en curso y sale.

```bash
python3 scripts/render/trabajador_host.py                       # permanente (--url http://127.0.0.1:8090, --clases gazebo,manim,ligero)
python3 scripts/render/trabajador_host.py --una-vez             # un reclamo
systemctl --user enable --now atp-render.service                # servicio de usuario: scripts/render/atp-render.service
python3 -m unittest scripts/render/test_trabajador.py -v        # prueba contra un backend falso (ciclo, progreso, cancelación, señal)
```

La carga la decide el gobernador (gazebo < 4, manim < 5) y los envoltorios la vuelven a esperar al arrancar; el trabajador no la mide.

## 5. Cómo añadir un render nuevo

1. **Envoltorio** `scripts/render/<id>.sh` (ejecutable), con el esqueleto:
   ```bash
   ID=mi-render
   source "$(dirname "${BASH_SOURCE[0]}")/_comun.sh"
   comun_args "$@"                                   # RUN, SALIDA, PANTALLA, A[clave]; trap que escribe media.json
   [ "$(python3 scripts/render/ayuda.py tipo-corrida "$RUN")" = lazo ] || fallo "la corrida no cumple … (explica qué falta)" 2
   defecto calidad m                                 # valores por omisión (entran en procedencia.args)
   esperar_carga 4; pantalla_tomar; trabajo_crear     # solo los que apliquen
   ejecutar <comando…>                               # en segundo plano: SIGTERM lo cancela
   mv "$TRABAJO/video.mp4" "$SALIDA/"; PRINCIPAL=video.mp4; MINI_DE=video.mp4; ARCHIVOS+=(otro.json); NOTAS="…"
   exit 0                                            # 143 si quedó parcial; ERROR="…" para el texto
   ```
   Imprima `progreso 0.4` en los hitos. No escriba el `media.json` a mano.
2. **Catálogo** `render/<id>.yaml` (formato de APP §4): `requiere`, `comando` con `{run_dir}`, `{salida}` y, si `pantalla: true`,
   `{pantalla}`; `recursos`; `ejecutor: host` si necesita Docker o pantalla (Gazebo, Manim) o los fondos de la marca del host (gráficas); `args` por omisión y `params_schema` (tipos `number`, `integer`, `enum`, `bool`: no hay `string`, use
   `enum` con `labels`). Cada argumento declarado debe aparecer en el script.
3. **Valide**: `python3 scripts/render/validar_catalogos.py` (formato, marcadores, que el script exista y conozca los argumentos) y,
   tras una corrida real, `… --media <salida>` (`media.json` y sha256, miniatura 640×360, un principal).
4. **Pruebe de verdad** con una corrida pequeña, mire 2-3 fotogramas (`ffmpeg -ss T -i video.mp4 -frames:v 1 f.png`) y anote la
   prueba aquí (§8).

## 6. Escenas `kind: proceso` nuevas (`experiments/`)

Formato de APP §2; las valida `validar_catalogos.py` (también que el comando exista y que cada argumento lo acepte la herramienta).
El backend que las ejecuta es del equipo AA.

| id | área | Qué lanza | Recursos | Artefactos | Duración real |
|---|---|---|---|---|---|
| `mision-corta-iss` | mision | `scripts/mision.py --n-od 0` (ISS sobre cdmx, 3 h por omisión, 1 contacto a la vez) | pesado | `informe.json`, `plan.json`, `metrics.json` | ver §8 |

Notas para el backend: (a) los args llevan **guiones bajos** (`--n_sats`); `mision.py` declara `--n-od` con guion, por eso va fijo en
el `comando`; (b) nombres de `contrato` fuera de APP §2: `mision_informe`, `mision_metricas`, `plan`.

## 7. Lecciones de Gazebo por software (el contenedor no tiene `/dev/dri`)

* **`--gui-config`**: la cámara y los paneles de la GUI se fijan con un `gui.config` generado (sin él, la ventana usa el que haya
  de la sesión anterior); la barra de Gazebo (48 px) se recorta en el rotulado.
* **Una escena a la vez** y con carga ≤ 4: `LP_NUM_THREADS=3` (llvmpipe). El vigilante del host detiene contenedores ROS con carga > 12:
  el envoltorio sale con 143 y `estado: parcial` (el backend reintenta).
* **Dominio y partición**: `ROS_DOMAIN_ID=83` y `GZ_PARTITION` por trabajo (red `host`: sin él dos Gazebo se descubren).
* **`stdin` cerrado**: `ros2 launch` se corta; se lanza con `docker compose run -d` y la salida a archivos dentro de `/ws`
  (los scripts ya lo hacen).
* **`colcon build` en el contenedor** (gz-demo): escribe `build/`, `install/`, `log/` en el repo; el cupo `gazebo` = 1 evita dos
  compilaciones simultáneas.

## 8. Pruebas reales (2026-10-06)

Todas con `ROS_DOMAIN_ID=83`, carga ≤ 4 al lanzar Gazebo y una corrida pequeña real. Los cuatro renders y las cuatro escenas nuevas se
probaron además **de punta a punta por el backend real** (`uvicorn` en el contenedor `station`, API en :8191 con `runs/_api`; el
trabajador del host reclamó, ejecutó, latió y terminó cada trabajo: `hecho`, `media.json` validado por el backend y `validar_catalogos.py`).

| Render | Corrida | Medio principal | Qué se vio en los fotogramas |
|---|---|---|---|
| `gz-demo` | lazo N2 cenital (`el_max` 85, `mask` 70) por el backend, 98 s de pared | `demo.mp4` 1920×1080, 66.8 s (directo: 64.6 s) + `eval.json` | antena y satélite con el arco del pase, haz cian; al pie «error de apuntamiento rms 0.0167° p95 0.0174° máx 0.1383° dentro de 0.1°: 99.3 %» = las cifras de la corrida de lazo (0.0167 / 0.0173 / 0.1382: mismo pase, medido ahora en Gazebo); en t = 30 s el satélite en la culminación y el plato vertical |
| `manim-corrida` | lazo de un pase completo (`el_max` 60, `mask` 5, banda S, 5834 muestras), `--calidad l` (480p15), 245 s de pared | `pase.mp4` 854×480, 33.0 s + `cifras.json` | «El pase, medido»: cielo polar con la traza y la antena (cian) sobre la referencia; panel en t = 344.7 s: EL 46.20°, AZ 192.11°, RANGO 736.0 km, DOPPLER −27.97 kHz, MARGEN 20.3 dB, ERROR 0.0027° — la telemetría original da, en 344.65 s, 46.23°, 192.07°, 735.7 km, −27.94 kHz, 20.29 dB, 0.0026° —; al final el resumen RMS 0.0015, P95 0.0033, MAX 0.0161, EN BANDA 100 %, MARGEN MÍN 10.75 dB, DOPPLER MÁX 50.55 kHz, `CUMPLE` (idénticos a `metrics.json`) |
| `graficas` | lazos (pase completo y cenital) | `lamina.png` 1920×1080 + 4-5 PNG 1600×900, 7-12 s | lámina con cabecera, siete tarjetas de cifras de `metrics.json`/`metricas.json`, línea de tiempo de antenas con handovers, satélites útiles, error con la banda y la tolerancia, cielo polar (con máscara alta solo el casquete), Doppler y rango, margen |
