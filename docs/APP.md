# Estación ATP · Estudio — la app alrededor del motor ROS

Arquitectura de la app que envuelve el motor ROS 2 de la Estación: **ejecutar** cualquier escena (lazo, misión, rover, dron,
ADCS, y lo que venga), **verla en vivo**, **renderizarla** (Gazebo, Manim, gráficas, visor 3D del rover) y **guardar la
evidencia** con su procedencia. Es la evolución de `station/` (no un reemplazo): todo lo que hoy funciona sigue igual
(CONTRATO §1–§6, E2E de la ronda 3).

Para quién: estudiantes y quien quiera aprender, usar y mejorar la Estación, y Co.De Aerospace
(divulgación con la marca). Regla de la casa intacta: cada corrida reproducible, determinista donde el motor lo es, y
cada medio con la procedencia exacta que lo generó.

## 1. Capas

```
 navegador (React + Vite + three.js, marca Co.De)
   │  REST + SSE (/api)
 ┌─▼──────────────────────────────────────────────────────────────────────────┐
 │ backend FastAPI (station/backend, 1 worker)                                │
 │  ├─ catálogo de ESCENAS   experiments/*.yaml   (kinds actuales + `proceso`)│
 │  ├─ catálogo de RENDERS   render/*.yaml        (Gazebo, Manim, gráficas…)  │
 │  ├─ GOBERNADOR de recursos: colas por clase, admisión por carga, reintentos│
 │  └─ SQLite: runs (existe) + renders + medios                               │
 └─┬───────────────────────────────┬──────────────────────────────────────────┘
   │ subprocess (grupo propio,      │ subprocess (pantalla virtual Xephyr del pool,
   │ ROS_DOMAIN_ID 100-199)         │ contenedor gz/manim, LP_NUM_THREADS)
 ┌─▼──────────────────────┐   ┌─────▼──────────────────────────────────────────┐
 │ MOTOR: ros2 launch …   │   │ RENDER: scripts/render/<id>.sh <run> <salida>  │
 │ scripts/*.py (lockstep)│   │ gz_demo / contenido (Manim) / gráficas│
 └─┬──────────────────────┘   └─────┬──────────────────────────────────────────┘
   ▼                                ▼
 runs/<run_id>/  (CONTRATO §1)   runs/<run_id>/media/<job_id>/ + media.json (§4)
```

Lo que **no** cambia: el motor (paquetes ROS), el lockstep (CONTRATO §8), los `kind` existentes y su API (§6), el
vigilante del host (`scripts/vigilante_host.sh`: la app **convive** con él, nunca lo desactiva).

## 2. Escenas: `kind: proceso` (genérico)

Hoy cada `kind` nuevo exige código en `runner.py`. Desde ahora una escena nueva es **solo un YAML**:

```yaml
id: mi-escena
title: Título corto de la escena
level: R1
kind: proceso
area: plataforma        # red | seguidor | control | orbita | rf | adcs | optico | mision | hardware | plataforma
description: >
  Qué se mide / qué se espera / por qué importa / validez (como los YAML actuales).
comando: [python3, scripts/mi_estudio.py, --salida, "{runs_dir}", --id, "{run_id}"]
estilo_args: cli        # cli: --clave valor (guiones bajos se respetan) | ros: clave:=valor
hecho_cuando: metricas.json          # archivo cuya existencia marca el fin correcto (además de exit 0)
metricas: metricas.json              # se lee como `metrics` de la corrida
artefactos:                          # lo que la app sabe mostrar
  - {ruta: metricas.json, contrato: metricas}
recursos: pesado        # ligero | pesado | gazebo | manim  (clase del gobernador, §3)
timeout_s: 1800
args: {n_sats: 72, planos: 8, h_km: 1200, inc_deg: 55, mascara_deg: 10, modo: mbb, politica: histeresis, dur_s: 2400}
params_schema: { ... como hoy ... }
```

Marcadores del comando: `{run_id}`, `{run_dir}` (= `{runs_dir}/{run_id}`), `{runs_dir}`, `{domain}` (ROS_DOMAIN_ID
asignado), `{root}`. Los `args` (validados contra `params_schema`) se añaden al final según `estilo_args`. El backend
escribe `params.json` y `launch.log` como siempre; si el proceso no crea `{run_dir}`, lo crea él antes.
Los `kind` actuales (`loop`, `campaign`, `plan`, `adcs`, `mision`) siguen tal cual.

## 3. Gobernador de recursos

Una sola autoridad decide qué corre. Clases y cupos por defecto (variables `STATION_CUPO_*`):

| Clase | Cupo | Admisión (carga 1 min del host) | Ejemplos |
|---|---|---|---|
| `ligero` | 2 | < 10 | plan, métricas, gráficas |
| `pesado` | 1 | < 6 | lockstep ROS, misión, estudios |
| `gazebo` | 1 | < 4 (render por software) | gz_demo |
| `manim` | 1 | < 5 | contenido/render.sh |

Reglas: (1) una tarea espera en estado `esperando_carga` mientras la carga supere su umbral (se reevalúa cada 10 s);
(2) si el proceso muere por señal externa (el vigilante detiene el contenedor) y no por error propio, la tarea pasa a
`interrumpida` y se **reintenta** hasta `STATION_REINTENTOS` (2) con espera creciente; (3) `GET /api/sistema` expone
carga, memoria, colas, cupos y las últimas acciones del vigilante (`journalctl -t atp-vigilante`, si se puede leer).
Las corridas del catálogo y los renders comparten el gobernador.

## 4. Renders: `render/*.yaml` y `media.json`

Un renderizador transforma una corrida en medios. Catálogo `render/*.yaml`:

```yaml
id: gz-demo
title: Demo en Gazebo de un lazo (1080p, marca Co.De)
requiere: {kind: loop}             # o {kind: loop}, {kind: [loop, campaign]}, {archivo: telemetry.jsonl}, {cualquiera: true}
comando: [bash, scripts/render/gz-demo.sh, "{run_dir}", "{salida}", "{pantalla}"]
estilo_args: cli
recursos: gazebo                   # clase del gobernador
pantalla: true                     # el gobernador asigna una pantalla Xephyr libre del pool :80-:83
duracion_estimada_s: 300
args: {el_max: 85, mask: 70}
params_schema: {el_max: {type: number, min: 5, max: 90, label: "Elevación máxima (deg)"}, ...}
```

Salida de cada trabajo: `runs/<run_id>/media/<job_id>/` con los archivos y un **`media.json`** (lo escribe el script;
si falta, el trabajo es `error`):

```json
{"version": 1, "job_id": "r20261005-…", "render": "gz-demo", "run_id": "…",
 "creado_utc": "ISO", "duracion_pared_s": 312.4,
 "archivos": [{"ruta": "visor.mp4", "tipo": "video", "ancho": 1920, "alto": 1080, "duracion_s": 80.0,
               "sha256": "…", "principal": true},
              {"ruta": "miniatura.jpg", "tipo": "imagen", "ancho": 640, "alto": 360}],
 "procedencia": {"commit": "git rev-parse HEAD", "sucio": false, "comando": ["…"], "args": {…},
                 "corrida": {"run_id": "…", "experiment_id": "…", "args": {…}}},
 "notas": "texto libre (p. ej. reloj leído en 1743 cuadros)"}
```

### 4.1 Trabajador de render en el host

El backend corre en el contenedor `station`, que **no** tiene Docker ni pantalla; Gazebo (servicio `gz`) y Manim
necesitan ambas cosas. Por eso un renderizador declara dónde se ejecuta:

```yaml
ejecutor: host        # host | backend (por defecto backend: lo lanza el propio backend como subprocess)
```

Los trabajos `ejecutor: host` los ejecuta `scripts/render/trabajador_host.py`, un proceso del **host** (lo arranca el
usuario o un servicio de systemd de usuario) que habla con la API. **El gobernador sigue decidiendo**: el trabajador
solo recibe un trabajo si el gobernador lo admite (clase, cupo, carga, pantalla libre).

| Método | Ruta | Notas |
|---|---|---|
| POST | `/api/renders/reclamar` `{trabajador, clases:[...]}` | 200 con el trabajo admitido (`job_id`, `render`, `comando` ya resuelto, `salida`, `pantalla`, `entorno`) y pasa a `corriendo`; 204 si no hay nada admisible ahora |
| POST | `/api/renders/{job_id}/latido` `{progreso?, log?: [líneas]}` | cada ≤ 10 s; sin latido en 60 s el backend lo da por `interrumpida` (y reintenta) |
| POST | `/api/renders/{job_id}/fin` `{exit_code, senal?}` | el backend valida `media.json` y decide `hecho`/`error`/`interrumpida` (señal externa) |

Las rutas en `comando` y `salida` son relativas a la raíz del repo, que es la misma en el host y en el contenedor (bind
mount `./:/ws`): el trabajador las ejecuta desde la raíz del repo. Cancelación: el siguiente latido recibe
`{"cancelar": true}` y el trabajador mata el grupo del proceso. Sin autenticación (127.0.0.1), igual que el resto de la API.

Toda salida de video lleva **miniatura** (`miniatura.jpg`, 640×360). Estados de un trabajo: `en_cola | esperando_carga |
corriendo | hecho | error | cancelado | interrumpida`.

## 5. API nueva (aditiva a CONTRATO §6)

| Método | Ruta | Notas |
|---|---|---|
| GET | `/api/escenas` | catálogo enriquecido: lo de `/experiments` + `area`, `recursos`, `artefactos`, `miniatura` (la del último medio de una corrida de esa escena, si hay) |
| GET | `/api/renders/catalogo` | `render/*.yaml` parseados |
| POST | `/api/runs/{id}/renders` `{render_id, args?}` | 201 `{job_id, estado}`; 409 si la corrida no cumple `requiere` o no está `done` |
| GET | `/api/renders?run_id=&estado=&limit=` | trabajos, más nuevos primero |
| GET | `/api/renders/{job_id}` | detalle + `media` (el `media.json`) |
| POST | `/api/renders/{job_id}/cancel` | |
| GET (SSE) | `/api/renders/{job_id}/stream` | `estado`, `log`, `progreso` (si el script imprime `PROGRESO <0-1>`), `hecho` |
| GET | `/api/medios?tipo=&escena=&limit=` | galería: archivos principales de todos los trabajos `hecho` |
| GET | `/api/medios/{job_id}/{ruta}` | sirve el archivo con **Range** (video en el navegador) |
| GET/HEAD | `/api/runs/{id}/artefactos/{ruta}` | cualquier artefacto DECLARADO en `artefactos` de la escena `proceso` y existente (p. ej. `telemetry.jsonl` o `metricas.json`), con `Range`; lo no declarado, 404 |
| GET | `/api/sistema` | §3 |
| GET | `/api/runs/{id}/export.mcap` | opcional: la telemetría como MCAP (abrible en Foxglove), si hay librería en la imagen; si no, 501 |

Errores con el formato actual (`{"detail": …}`), ids validados con regex, rutas de medios confinadas a `runs/`.

## 6. Frontend (station/frontend)

Marca **Co.De Aerospace** (`codeaerospace_contenido/marca/paleta.json`): fondo `#080F15`/marino `#0A0E27`, plata
`#F2F3F4`, tenue `#8E99A6`, **cian `#00D9FF` acento único**, ámbar `#F59E0B` solo para avisos/handover; títulos en
Rajdhani (empaquetar la fuente con `@fontsource` o el archivo local con su licencia OFL), cuerpo en Inter (ya está),
cifras en JetBrains Mono (ya está). Tema oscuro por defecto; legible en 1366×768 y en 1920×1080.

Vistas (las actuales se conservan y se reubican en la navegación):

| Vista | Ruta | Qué hace |
|---|---|---|
| Inicio | `#/` | estado del sistema (carga, colas, cupos, vigilante), últimas corridas y últimos medios |
| Escenas | `#/escenas` | catálogo en tarjetas con miniatura, filtros por área, botón **Lanzar** |
| Lanzar | `#/escenas/{id}` | formulario desde `params_schema`, descripción (qué/por qué/validez), corridas previas |
| Corrida | `#/runs/{id}` | lo de hoy (curvas en vivo, montura 3D, misión…) + panel **Renderizar** (elige render, args, ve la cola) + medios de la corrida |
| Render | `#/render` | cola de trabajos con estado, progreso y log en vivo |
| Medios | `#/medios` | galería (video con reproductor, imágenes), filtros, procedencia de cada medio y descarga |
| (existentes) | `#/experimentos`, `#/resultados`, `#/comparar`, `#/campanas`, `#/mision`… | intactas |

`VITE_MOCK=1` debe seguir funcionando: el mock cubre también la API nueva (§5), para desarrollar sin backend.

## 7. Equipos (ronda de la app) y fronteras

| Equipo | Dueño de | No toca |
|---|---|---|
| **AA backend** | `station/backend/` (gobernador, `kind: proceso`, renders, medios, sistema, MCAP), parser de `render/*.yaml`, pruebas | frontend, scripts de render |
| **AB frontend** | `station/frontend/` (marca, vistas nuevas, visor 3D del rover, mock de §5) | backend, scripts |
| **AC render y escenas** | `render/*.yaml`, `scripts/render/*.sh` (envoltorios no interactivos de `gz_demo`, Manim vía `contenido/`, gráficas con matplotlib), miniaturas, `media.json`; escenas `kind: proceso` nuevas en `experiments/` (estudios) | backend, frontend, motor |

Contratos compartidos: este documento (§2–§5). Si un equipo necesita cambiar un contrato, lo deja escrito en su
informe y el integrador decide. Integración: AA primero (la API), luego AC (sus YAML los valida el parser de AA) y AB
(contra el backend real y contra el mock).

## 8. Fuera de alcance (por ahora)

Autenticación y despliegue en el VPS (próximo frente del PLAN), edición de escenas desde la UI, render en GPU.
