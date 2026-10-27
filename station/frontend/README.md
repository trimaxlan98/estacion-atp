# Estación ATP · Estudio — frontend

App React 18 + Vite (JavaScript) con la marca **Co.De Aerospace** (paleta de `codeaerospace_contenido/marca/paleta.json`:
fondo `#080F15`, plata `#F2F3F4`, tenue `#8E99A6`, **cian `#00D9FF` acento único**, ámbar `#F59E0B` solo para avisos,
re-apuntado y handover; Rajdhani en títulos y navegación (`@fontsource/rajdhani`, licencia OFL), Inter en el cuerpo y
JetBrains Mono en las cifras; logo `src/assets/logo-cian.png`). Sin CDN ni librerías de gráficas: las gráficas son SVG
propio con submuestreo min/max (10 000+ muestras sin congelarse) y los visores 3D usan three.js empaquetado por npm.
Arquitectura de la app: [`docs/APP.md`](../../docs/APP.md).

## Uso

```bash
npm install
npm run dev          # http://localhost:5173, proxy /api -> http://127.0.0.1:8090 (backend real)
npm run dev:mock     # modo mock: la API (CONTRATO §6) en memoria, sin backend
npm run build        # genera dist/ (rutas relativas; el backend lo sirve en "/")
npm run preview      # sirve dist/
npm test             # pruebas de la logica pura (node --test, sin dependencias nuevas)
npm run shots        # capturas de las vistas clasicas en docs/ (necesita `npm run preview` y Chromium)
```

`ATP_API_PROXY=http://host:puerto` cambia el destino del proxy de desarrollo. Chromium: `PW_CHROMIUM=/ruta` o el de
`~/.cache/ms-playwright`.

## Vistas (rutas por hash, no requieren reescritura en el servidor)

Navegación: fila principal **Inicio · Escenas · Biblioteca · Render · Medios** y, debajo, la fila **Análisis**
con las vistas anteriores (Experimentos, Campañas, Plan de contactos, Misión, ADCS), que siguen intactas.

| Ruta | Vista |
|---|---|
| `#/` | **Inicio**: carga del host, cupos y colas del gobernador (clase, cupo, en uso, en cola, esperando carga, umbral), «En espera», vigilante (o aviso si el diario no se puede leer), últimas corridas, renders en curso y últimos medios |
| `#/escenas` | **Escenas**: catálogo en tarjetas con miniatura (la del último medio o una procedural de la marca), filtros por área, buscador |
| `#/escenas/:id` | **Lanzar**: descripción dividida en Qué se mide / Qué se espera / Por qué importa / Validez, formulario desde `params_schema` (validado), «Qué produce», corridas previas |
| `#/runs/:id` | **Corrida**: lo de siempre (curvas en vivo, montura 3D, misión…) + resultados de `kind: proceso` + panel **Renderizar** + medios de la corrida. Muestra con nota ámbar los estados `esperando_carga` e `interrumpida` |
| `#/render` | **Render**: cola de trabajos (estado, progreso, aviso «esperando al trabajador del host») y detalle con registro en vivo por SSE |
| `#/medios` | **Medios**: galería con filtros; reproductor de video (Range), descarga y procedencia (commit, comando y args exactos) |
| `#/experimentos` | Catálogo clásico por nivel (era `#/`) |
| `#/biblioteca`, `#/compare/:a/:b`, `#/campanas[/:id]`, `#/plan[/:id]`, `#/mision[/:id]`, `#/adcs[/:id]` | Como antes |

`#/runs/:id` elige la vista por el `kind` de la corrida (`detalle.kind`, o el catálogo): lazo y campaña como
antes, `plan`, `adcs` y `mision` con sus paneles, y `proceso` con las cifras de `metricas.json` (cada número una tarjeta).

### Mock (`VITE_MOCK=1`, `?mock=1` o sin backend)

`src/mock/` emula la API de CONTRATO §6 con la forma real del backend: escenas (con `kind: proceso`), catálogo de renders (`ejecutor: host`), trabajos con estados `en_cola → esperando_carga → corriendo →
hecho` y SSE, medios (miniaturas SVG y un `mp4` real de muestra en `public/mock/`), `/api/sistema` (forma de
`sistema.py`, con el diario del vigilante) y corridas `esperando_carga` e `interrumpida`.
El mock vive en memoria: al recargar la página se reinician las corridas y sus ids (navega con el hash, como las herramientas).

### Contra el backend real

El backend de `station/backend/` corre en el contenedor `station` (`./station.sh`). Para probarlo sin Docker ni ROS basta
FastAPI: `pip install --target X "fastapi<0.111" uvicorn pyyaml`, una raíz de pruebas con una escena `kind: proceso` y un render `requiere: {cualquiera: true}` (el de `station/backend/tests/fixtures/render/`),
`STATION_ROOT=… STATION_FRONTEND_DIST=$PWD/dist python3 -m uvicorn station.main:app --ws none`. Con la carga del host alta el gobernador deja los trabajos en
`esperando_carga` (justo lo que debe mostrar la UI); sube `STATION_UMBRAL_LIGERO` para la prueba.

### Plan de contactos

- **Gantt** (`src/charts/Gantt.jsx`, SVG): una fila por estación, una barra por contacto del plan elegido
  (óptimo, los tres codiciosos o «óptimo sin la estación»), con el GB y la elevación máxima dentro de la barra
  cuando cabe (zoom 12/6/3 h con deslizador) y siempre en el tooltip. En gris, bajo la fila, los pases que el
  plan descarta; borde rojo = pasa por el keyhole. Clic en un satélite de la leyenda lo aísla.
- **Color por satélite = identidad, no rol**: ocho tonos categóricos (`SAT_COLORS` en `colors.js`, la ISS en ámbar)
  que evitan el rojo y el verde (estado) y el cian (antena/estación). Con más de 8 satélites se reciclan.
- Resumen (contactos, minutos, GB, % de la cota de Shannon, **brecha del codicioso** frente al óptimo y si el
  óptimo está probado), GB por satélite y estación, y la comparación **con/sin una estación** (tarjetas y barras
  por satélite; el botón dibuja el plan sin ella en el Gantt gracias a `aporte_estacion.<est>.ids_sin`).

### ADCS

- **Modos como bandas de color** (`MODE_COLOR`): DETUMBLE ámbar (el satélite girando), NADIR violeta (la Tierra),
  TRACK_GS cian (la antena a la estación). Una tira con modos, eclipse y pase, y las mismas bandas detrás de cada gráfica
  (`xBands` de `Chart.jsx`).
- **Banda de umbral** siempre visible (`metrics.track.threshold_deg`, el que da la pérdida de 1 dB del enlace);
  el error de apuntamiento de TRACK_GS en el pase se pinta en rojo por encima de él. La gráfica general recorta el eje a
  5 × el umbral (el modo NADIR arranca con errores de decenas de grados) y lo dice en el subtítulo.
- Momento de cada rueda contra su saturación (`wheels.capacity_nms`, línea roja y zona de aviso al 80 %).

### Montura 3D

`src/components/Mount3D.jsx` (three.js empaquetado por npm; **sin CDN**, en su propio chunk de ~140 kB gz que se
carga con `React.lazy` solo al abrir un lazo): base, anillo de acimut con marca norte, brazos, eje y plato de elevación
orientados con `az`/`el` de la telemetría, el eje de la antena (cian) y la línea de vista al satélite (ámbar, con
`ref_az`/`ref_el`, a radio fijo: no está a escala). Ligera por diseño: **dibuja solo la muestra elegida y solo cuando
algo cambia** (muestra, cámara, tamaño; sin bucle `requestAnimationFrame` continuo), así 10 000 muestras cuestan lo
mismo que una. En vivo sigue la última muestra; el deslizador y *Reproducir* recorren una corrida terminada. El
contenedor expone `data-az/el/ref-az/ref-el/t/n/frames` (los usa el E2E). Sin WebGL muestra un aviso en vez de fallar.

Color por rol (`src/colors.js`): cian = cifra calculada/antena, ámbar =
satélite/referencia/p95, violeta = cielo, rojo = fuera de presupuesto, verde =
dentro de presupuesto.

## Modo mock (corridas clásicas)

Se activa con `VITE_MOCK=1`, con `?mock=1` en la URL, o automáticamente si
`GET /api/health` no devuelve JSON válido (2.5 s de plazo). El badge «MODO MOCK»
aparece en la cabecera. `src/mock/` genera un pase LEO esférico (551 km), Doppler
en S, keyhole, enlace y campañas Monte Carlo con el formato exacto de
`docs/CONTRATO.md`; `server.js` emula la REST y la SSE en memoria. Para `plan` y `adcs` el mock sirve
**corridas reales guardadas** (`src/mock/fixtures/`, hechas con `tools/mock_fixtures.py` a partir de un `s6-plan-24h` y
un `e1-adcs-funcube` reales; el ADCS con 1 fila de cada 8): no son datos inventados, pero el mock devuelve siempre
esos mismos resultados sea cual sea el argumento del formulario.

## Supuestos sobre el contrato

- El tipo de una corrida (lazo/campaña) se deduce de `kind` en `/api/experiments`
  (o de que el detalle traiga `campaign`).
- Al abrir una corrida activa se pide `/telemetry` y luego se abre la SSE; las
  muestras repetidas se descartan por `t`, así que el backend puede reenviar el
  historial.
- Tras `done` se vuelve a pedir `GET /api/runs/{id}` para leer `metrics`/`campaign`.
- Una corrida terminada con `metrics.n > 6000` se pide con `every=ceil(n/6000)`.
- `p95_uncertainty_pts` se muestra como «± pts» de percentil.
- El detalle de una corrida trae `kind`; `plan` solo en `kind: plan` y `metrics.kind = 'adcs'|'plan'` según el caso;
  la telemetría de `adcs` es la del CONTRATO 2.1 (una fila por paso de 1 s, no a 10 Hz) y `metrics.n` coincide con su
  número de filas.
- Un `enum` puede traer `labels` (`{valor: texto}`) y su valor puede ser `""` (el formulario lo muestra como «ninguna»).
- Todo parámetro de `params_schema` trae su valor por defecto en `args` (el formulario se
  inicializa con él), y la telemetría omite los campos sin dato (la 1.ª muestra solo trae `ref_*`).
