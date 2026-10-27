# contenido/ — el experimento produce el video

Sprint 6 de la Estación ATP: cualquier corrida real se convierte en una
animación de Manim con la identidad visual de Co.De Aerospace, **con cifras
medidas, no escritas a mano**.

```
corrida ──────────▶ dataset ──────────▶ escena ──────────▶ video
runs/<id>/          contenido/          contenido/         contenido/media/
 telemetry.jsonl    datasets/<id>.json  escenas/*.py       videos/<escena>/<calidad>/*.mp4
 metrics.json       (exportar.py)       (render.sh)
 campaign.json
```

## Regla: toda cifra en pantalla sale del dataset

Ningún número de resultado está escrito en una escena. Hay dos clases de cifra:

| Clase | Ejemplos | De dónde sale | Cómo se comprueba |
|---|---|---|---|
| **Resumen** | rms, p95, máx, % dentro de 0.1°, margen mín, Doppler máx, p50/p95/p99/peor de la campaña | `metrics.json` / `campaign.json`, copiados **literalmente** al dataset (`metricas`, `campana`) | Cada escena registra qué campo rotuló y qué texto dibujó (`media/cifras/<Escena>.json`); `verificar.py` lo contrasta con el archivo original de `runs/<id>/` |
| **En vivo** | el, az, rango, Doppler, margen y error en cada instante; las curvas | series de `telemetry.jsonl` en el dataset, interpoladas al tiempo del cuadro | `verificar.py --t <seg> <Escena>` reconstruye el instante del pase de ese cuadro y lista la telemetría original; se compara con el fotograma (`fotogramas.sh`) |

Lo que **no** es cifra medida y sí está en las escenas: rótulos de mobiliario
(«mascara 5 deg» sale de la menor elevación de la referencia; «banda S» de
`metrics.link.band`), la escala de los ejes (derivada de los datos) y los textos
fijos («PROGRAM TRACK», «AUTOTRACK»). El formato es el del curso 30: sin
subtítulos, rótulos de ≤ 5 palabras (`vigilar()` aborta el render si una cifra se
vuelve frase) y color por rol.

## Paleta por rol (curso 30, §8)

| Color | Papel aquí |
|---|---|
| cian `#22d3ee` | cifra calculada/medida, la **antena real**, la **banda de 0.1°** (siempre dibujada) |
| ámbar `#f59e0b` | el satélite, la **traza de referencia**, la curva de error, el **p95** |
| violeta `#a78bfa` | el cielo, las barras del histograma |
| rojo `#f43f5e` | error **fuera** de la banda, p99/peor, `NO CUMPLE` |
| verde `#34d399` | enlace cerrado, dentro de presupuesto, `CUMPLE` |
| gris | mobiliario (anillos, ejes) |

## Flujo

```bash
# 1. Corridas reales con el motor ROS (ROS_DOMAIN_ID=75; deterministas por el lockstep)
contenido/corridas.sh                    # demo-n2 demo-pase demo-n5a demo-n5b demo-campana
contenido/corridas.sh demo-n5b           # solo una
# (o cualquier corrida de la Estación: runs/<id>/ con el mismo contrato)

# 2. Dataset compacto y validado (Python puro; corre en el host)
python3 contenido/exportar.py demo-pase                  # -> contenido/datasets/demo-pase.json
python3 contenido/exportar.py runs/20261002-101500-ab12cd --id mi-pase

# 3. Video (imagen codeaerospace_contenido-manim, sin red)
contenido/render.sh -q l pase autotrack campana          # 480p15
contenido/render.sh -q m -m autotrack                    # 720p30 + miniatura en muestras/

# 4. Comprobar
python3 contenido/verificar.py                           # cifras de resumen vs runs/<id>/
contenido/fotogramas.sh contenido/media/videos/pase/480p15/ElPase.mp4 16 24
python3 contenido/verificar.py --t 16 ElPase             # qué debería verse en el segundo 16
python3 -m unittest discover -s contenido -p 'test_*.py' # pruebas de exportar.py
```

Cada escena toma su corrida de una variable de entorno (`render.sh` las pasa al
contenedor):

| Escena (`render.sh`) | Clase | Corrida(s) | Variables |
|---|---|---|---|
| `pase` | `ElPase` | un pase | `ATP_PASE` (def. `demo-pase`), `ATP_PLAY_S` (22 s) |
| `autotrack` | `ProgramContraAutotrack` | dos pases del mismo cielo | `ATP_PROGRAM` (`demo-n5a`), `ATP_AUTOTRACK` (`demo-n5b`), `ATP_PLAY_S` (20 s) |
| `campana` | `LaCampana` | una campaña Monte Carlo | `ATP_CAMPANA` (`demo-campana`), `ATP_PLAY_S` (16 s) |

Ejemplo con otra corrida de la Estación:
`ATP_PASE=20261002-101500-ab12cd contenido/render.sh -q l pase`.

## Las escenas

- **El pase** (`pase.py`): cielo polar con la máscara, la traza de referencia
  (ámbar) y la antena real (cian) avanzando en el tiempo del pase; panel con T+,
  el, az, rango, Doppler (kHz), margen (verde si el enlace cierra) y error; el
  error contra tiempo con la banda de 0.1° dibujada desde el primer cuadro y los
  tramos que la rebasan en rojo. Cierra con el resumen de `metrics.json`.
- **Program track contra autotrack** (`autotrack.py`): dos columnas, mismo cielo,
  misma escala vertical y misma banda. Cada una con su cielo, su **mira** (la
  antena vista desde el satélite, con el anillo del objetivo) y su error. En
  program track el error sale de la banda y se queda fuera; en autotrack queda
  encerrado. Si los datasets traen `params`, comprueba que `autotrack` coincide
  con el rol (falla si se cruzan las corridas).
- **La campaña** (`campana.py`): los N pases de `campaign.json` caen uno a uno en
  su barra; luego el objetivo (cian) y el p95 (ámbar, el mismo que calcula
  `histograma()`; la escena aborta si no coincide con `campaign.json`). La cola
  que queda fuera de la escala se cuenta y se rotula en rojo.

## Qué hay en un dataset (`atp-contenido/1`)

```json
{"esquema": "atp-contenido/1", "id": "demo-pase", "tipo": "pase",
 "params": {...},                      // params.json de la corrida, si existe
 "metricas": {...},                    // metrics.json, literal
 "fuente": {"muestras_telemetria": 5833, "muestras_dataset": 400, ...},
 "series": {"t": [...], "ref_az": [...], "az": [...], "el": [...], "e_pt": [...],
            "range_km": [...], "doppler_hz": [...], "margin_db": [...], ...},
 "campana": {...}}                     // campaign.json, literal (si hay)
```

`exportar.py` valida el contrato (`docs/CONTRATO.md` §1–§4) y **falla con un
mensaje que dice qué falta y dónde**: sin `metrics.json` («la corrida no
terminó»), línea de telemetría cortada (`archivo:línea`), campo obligatorio
ausente, tiempo que no crece, `rms_deg` con distinta longitud que `n`… Las
filas de arranque incompletas (antes del primer dato de la montura o del
enlace) se descartan y se cuentan (se exige ≥ 90 % completas). Los campos
opcionales entran solo si todas las filas los traen.

La telemetría (10 Hz) se **decima a 400 puntos** (tope de `traza_error`)
conservando el máximo de `e_pt` de cada bloque: ni el pico ni las salidas de
banda se pierden. El archivo es determinista (mismos bytes para la misma
corrida) y pesa ~60 KB, así que se versiona en git.

## Cómo se renderiza

`render.sh` hace `docker run --rm` con la imagen `codeaerospace_contenido-manim`,
sin red, sistema de archivos de solo lectura y tu usuario:

| Montaje | Dentro | Modo |
|---|---|---|
| repo `codeaerospace_contenido` | `/workspace` | solo lectura |
| `contenido/` | `/atp` | solo lectura (escenas y datasets) |
| `contenido/media/` | `/media` | lectura/escritura (única salida, fuera de git) |

Las librerías de dibujo (`atp.py`, `apuntado.py`, `code_brand.py`) **no se
copian**: `escenas/estilo.py` añade `/workspace/studio/content/manim_extensions`
a `sys.path` (`ATP_LIBS` lo cambia; `CONTENIDO_REPO` cambia la ruta del repo en el
host). `estilo.py` reproduce lo que el `style_block.py` de cada lección del
curso 30 hace: tipografía de marca, sombra de `Text`, guardián del formato mudo y
paleta. En git solo se guarda una miniatura PNG por escena en `muestras/`.

Las cifras que cambian cada cuadro se dibujan con un banco de glifos de Space
Mono (`BancoMono`): crear un `Text` por cifra y por cuadro hacía el render varias
veces más lento.

## Lo que hay renderizado (corridas `demo-*`, 2026-10-02)

| Escena | Video (`contenido/media/`, fuera del repo) | Duración | Miniatura |
|---|---|---|---|
| El pase (`demo-pase`: S, 60° máx., máscara 5°) | `videos/pase/480p15/ElPase.mp4` | 33.0 s | `muestras/pase.png` |
| Program track contra autotrack (`demo-n5a` / `demo-n5b`, Ka) | `videos/autotrack/480p15/…` y `720p30/ProgramContraAutotrack.mp4` | 32.4 s / 32.3 s | `muestras/autotrack.png` |
| La campana (`demo-campana`: lazo, N = 300, semilla 2029) | `videos/campana/480p15/LaCampana.mp4` | 28.7 s | `muestras/campana.png` |

Cifras de resumen en pantalla (todas contrastadas con `runs/<id>/`): pase S:
rms 0.0015°, p95 0.0033°, máx 0.0161°, 100.0 % dentro de 0.1°, margen mín
10.75 dB, Doppler máx 50.55 kHz; program track: rms 0.1572°, 0.0 % dentro de la
banda, `NO CUMPLE`; autotrack: rms 0.0042°, 100.0 %, `CUMPLE`; campaña: p50
0.0295°, p95 0.0423°, p99 1.73°, peor 2.49° (12 pases por encima de 0.15°).

## Limitaciones conocidas

- **Determinismo**: repetir `corridas.sh` dio las mismas cifras de resumen en
  `demo-campana` y casi las mismas en el pase de S (cambia `l_point_max_db`),
  pero **no** idénticas en `demo-n5b` (autotrack: rms 0.00408° la primera vez,
  0.00422° la segunda; los extremos también cambian) ni en `demo-n2`
  (`ref_az/ref_el` de las primeras filas del grabador difieren un paso). Las dos
  tandas se hicieron con la máquina muy cargada y de forma distinta; la causa no
  está aislada (el grabador y `link_budget` son nodos pasivos que emparejan «lo
  último que llegó» y no por sello). Las escenas no dependen de ello (leen lo que
  haya en `runs/`), pero un dataset refleja **esa** corrida, no «la» corrida.

- Las cifras de **resumen** (`metrics.json`) se calculan a 20 Hz y la telemetría
  se graba a 10 Hz: el máximo de la curva dibujada puede quedar ~1 % por debajo
  del máximo rotulado (p. ej. 0.1367 contra 0.1382 en `demo-n2`). Lo rotulado es
  siempre `metrics.json`.
- La lectura de **error en vivo** interpola entre muestras decimadas (con el
  pico de cada bloque de ~1.5 s en el pase de 9 min): puede quedar algo por
  encima de la muestra exacta de ese instante.
- En las corridas de autotrack, `ref_az/ref_el` del grabador es el mando
  corregido (`/atp/reference`), no el satélite; el error del monitor sí se mide
  contra la verdad. Las escenas dibujan el satélite como `medida + error`
  (la verdad), que en un lazo normal coincide con la referencia.
- Una corrida de la Estación puede traer más de 400 puntos solo si se exporta con
  `--max-points` mayor; las escenas lo rechazan (tope de `traza_error`).
- El grabador (`src/atp_ros/recorder.py`) no guarda la verdad ni la máscara
  de la estación como series/campos propios; las escenas la derivan
  (`med + error` y la menor elevación de la referencia).

## Desde la app (render `manim-corrida`)

`scripts/render/manim-corrida.sh <run_dir> <salida>` encadena `exportar.py` (a un dataset temporal, sin tocar
`contenido/datasets`) y `render.sh`, y deja el video, `cifras.json` y un `media.json` con la procedencia
([RENDER](../docs/RENDER.md)). `render.sh` acepta para ello `MEDIA_DIR`, `DATASETS_DIR` y `NOMBRE_CONTENEDOR`; sin ellas
todo funciona como arriba.
