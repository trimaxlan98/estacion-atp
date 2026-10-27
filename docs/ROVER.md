# Rover planetario (ronda 7)

**Objetivo**: modelar la dinámica y el movimiento de un rover de seis ruedas (rocker-bogie) que se pueda mandar por
`cmd_vel` o por metas en terreno difícil y en distintos planetas (Luna, Marte, Titán, Ceres, Tierra); después, fase 2,
comunicarlo por antenas (CONTRATO §19).

Supuesto de partida: «robot» = rover de ruedas. Si se quería otro (patas, brazo, dron), el núcleo de terreno/planeta se reutiliza.

## Equipos
- **R1 · dinámica**: `planetas.py`, `terreno.py`, `rover.py`, nodo `rover_dinamica`, pruebas, escena y validación.
- **R2 · control y navegación**: seguimiento de metas (pure pursuit + evasión por pendiente/rugosidad del terreno),
  controlador de tracción con anti-deslizamiento, seguridad (vuelco, atasco, pendiente, energía). Nodo `rover_navegador`.
- **R3 · Gazebo y visor**: modelo SDF/URDF del rocker-bogie con terreno por heightmap, planeta por gravedad, video;
  y vista del rover en el visor 3D web.

Orden: R1 define la API; R2 y R3 programan contra el contrato (CONTRATO §18) y se integran al final.

## Física (R1)
Cuerpo rígido 6-DOF; cada rueda con contacto vertical resorte-amortiguador sobre el heightmap; tracción por
Bekker–Wong simplificada (esfuerzo cortante vs. deslizamiento, hundimiento por presión), resistencia a la rodadura
por compactación, deslizamiento lateral por fricción de Coulomb saturada. La gravedad y el suelo salen del planeta.

## Validación
Pruebas de energía (cae en caída libre con g del planeta), subir pendientes (límite por φ del suelo), determinismo,
llano sin deslizamiento, vuelco cuando se supera el ángulo estático.

## Resultados R1
Paquete `src/atp_rover` (núcleo sin ROS + nodo `rover_dinamica` + CLI `atp_rover.cli`). Todas las cifras de abajo salen de
`python3 -m pytest -q src/atp_rover/test` (15 pruebas, 107 s) y de las tres escenas; son **en simulación, con estos
supuestos** (suelo de Marte, Titán y Ceres no medido; ver `atp_rover/planetas.py`).

**Modelo.** Rover de 150 kg, ruedas de 0.25 m (ancho 0.20 m), semivía 0.40 m, centro de masa 0.40 m sobre el suelo
(ángulo estático de vuelco lateral atan(0.40/0.40) = 45°). RK4 con dt = 0.01 s fijo; cuesta unos 6 ms por paso en
una CPU normal (100 s simulados ≈ 1 min). Simplificaciones: rocker-bogie como resorte-amortiguador por rueda
(20 kN/m, 990 N·s/m), sin cinemática de balancín ni choque del chasis; el hundimiento no baja el cubo; ruedas sin
dirección (viraje diferencial); fricción lateral tanh con ancho 0.1 m/s (deriva lenta en reposo sobre pendiente);
consumo eléctrico = 20 W fijos + max(0, τω)/0.7 + 0.02 τ² por rueda (supuestos).

**Misma orden, distinto cuerpo** (llano, 1 m/s mandado, 10 s tras asentar; regenerado con la física corregida `rodadura_en_motor: true`, defecto desde la ronda 8; los valores históricos de R1 —Luna 8.55 m, Marte 9.03 m, Tierra 8.89 m, Titán 7.10 m, Ceres 4.05 m— se reproducen con `rodadura_en_motor: false`):

| Cuerpo | g (m/s²) | Distancia (m) | Deslizamiento medio | Energía (Wh) |
|---|---|---|---|---|
| Luna | 1.62 | 8.45 | 0.106 | 0.2315 |
| Marte | 3.71 | 8.69 | 0.079 | 0.4106 |
| Tierra | 9.80665 | 7.81 | 0.099 | 0.9669 |
| Titán | 1.352 | 7.03 | 0.247 | 0.2338 |
| Ceres | 0.27 | 4.04 | 0.322 | 0.1377 |

Incluye el transitorio de arranque; con 1 m/s mandado la distancia ideal sería 10 m. Ceres es el caso extremo: con
g = 0.27 m/s² la carga por rueda es mínima y el suelo supuesto no da empuje.

**Pendiente** (Luna, φ = 25°, c = 100 Pa, 0.5 m/s mandado, 8 s): avance en x = +2.69 m a 4°, +0.53 m a 14°, −0.86 m
a 20° y −10.03 m a 32°. Con φ = 40° a 20°: +1.20 m. El límite de tracción del modelo queda por debajo de φ porque el
empuje máximo es T(1) = (c·b·L + Fn·tanφ)(1 − K/L·(1 − e^(−L/K))) < Fn·tanφ, y además actúa la compactación.

**Vuelco** (Tierra, φ = 60°, c = 3 kPa, rover de costado en la pendiente): a 35° queda estable con alabeo −35.9°;
a 52° vuelca (inclinación del eje z > 70°, `vuelcos` = 1).

**Caída libre**: v(1 s) = −g·1 s exacta a 1e-6 relativo con la g de cada cuerpo (sin atmósfera) y energía mecánica
conservada a 1e-9; con la atmósfera de la Tierra (Cd·A = 0.8 m²) la velocidad a 1 s está entre 0.9 g y g.

**Escenas** (`python3 -m atp_rover.cli --escena experiments/rover-….yaml`; `kind: rover`):

| Escena | Distancia (m) | Tiempo (s) | Energía (Wh) | Deslizamiento medio | Pendiente máx. (°) | Metas |
|---|---|---|---|---|---|---|
| `rover-luna-rocoso` (semilla 7) | 21.64 | 50.3 | 0.400 | 0.081 | 3.85 | alcanzadas |
| `rover-marte-dunas` (semilla 3) | 26.78 | 88.1 | 1.595 | 0.307 | 14.98 | alcanzadas |
| `rover-titan-pendiente` (12°) | 9.78 | 80.1 | 0.974 | 0.667 | 12.00 | alcanzadas |

Misma semilla → mismos bytes de `telemetry.jsonl` y `metrics.json` (prueba `test_cli_determinista_y_contrato`).

**Pendiente de integración.** El backend de la Estación (`station/backend/station/experiments.py`, `KINDS`) aún no
conoce `kind: rover`; el nodo ROS no se ejecutó (sin `rclpy` en el entorno de desarrollo): solo comprobado el núcleo y la CLI.
## Resultados R2 (control y navegación)
Módulos en `src/atp_rover/atp_rover/` (Python puro; el nodo ROS solo reenvía): `navegacion.py`, `seguridad.py`,
`nodo_navegador.py` (`rover_navegador`: suscribe `/rover/estado` y `/rover/meta`, publica `/rover/cmd_vel` y `/rover/alarma`).

- **Interfaz de terreno (duck-type)**: `altura(x,y)` y `normal(x,y) -> (nx,ny,nz)` unitaria. Pendiente = `acos(nz)`;
  rugosidad local = rms de `h(p)+h(-p)-2h(0)` en un anillo de 0.5 m (una rampa lisa vale 0).
- **Velocidad**: `limite_velocidad` baja linealmente de `v_max` a `v_min` entre 5° y 20° de pendiente y hasta `rug_max`=0.15 m de rugosidad.
- **Planificador**: A* 8-conexo en rejilla de 0.5 m, costo `1 + 3·pend/pend_max + 5·rug/rug_max`; celdas con pendiente ≥ 20° o
  rugosidad ≥ 0.15 m son intransitables; atajos libres para suavizar. Sin ruta (meta intransitable o encerrada) -> `sin_ruta`, el rover no se mueve.
- **Seguimiento**: pure pursuit (lookahead 1.5 m), gira en sitio si el objetivo queda a más de 80°, frena cerca de la meta (tol 0.3 m).
- **Tracción**: `ControlTraccion` baja un factor de par/velocidad en [0.2, 1] si el |deslizamiento| de cualquier rueda pasa 0.3, y recupera 0.05 por paso; `pares()` recorta más la rueda que patina.
- **Seguridad** (`/rover/alarma`, JSON del contrato): `vuelco` (|cabeceo| o |alabeo| > 35°), `pendiente` (> 25°), `atasco` (|v|<0.02 con
  orden ≥ 0.1 m/s durante 3 s, o `atascado` del estado), `energia` (`energia_wh` se interpreta como CONSUMIDA y pasa `energia_max_wh`).
  Se emite al activarse y se repite cada 5 s. Vuelco, pendiente y energía ordenan `cmd_vel = 0`; el atasco solo avisa.
- **Pruebas** (`src/atp_rover/test/test_navegacion.py`, 13, en `scripts/validar_todo.sh`): rover unicycle falso + terreno sintético:
  alcanza metas (incluida una detrás y secuencias), rodea una colina de 6 m (trayectoria con pendiente máx. < 20°), meta intransitable
  sin ruta, velocidad por pendiente/rugosidad, tracción, frenado ante vuelco/pendiente/energía, atasco a los ~3 s, determinismo (trayectoria idéntica).
- **Sin probar**: el nodo ROS (no se ejecutó contra `/rover/estado` real), integración con `terreno.py`/`rover.py` de R1 (el nodo
  intenta `Terreno(tipo=, semilla=)` y, si falla, navega en línea recta), ni con la dinámica real (deslizamiento y atascos reales).
## Resultados R3 (Gazebo y visor 3D)

Rama `worktree-agent-ae8c8443bd8f555be`. Todo lo de R3 vive en `src/atp_rover/gazebo/` (Python puro + SDF generado), `scripts/gz_rover*.sh`
y `station/frontend` (visor web); no toca `planetas.py`, `terreno.py`, `rover.py` ni la navegación.

![Visor 3D web del rover en modo mock](../station/frontend/docs/rover-visor-marte.png)

**Video y capturas de Gazebo (en local, fuera de git):** `runs/gz_rover/<planeta>_<terreno>/rover.mp4` (con cabecera Co.De y cifras),
`runs/gz_rover/montaje.png` (un cuadro por planeta y uno del modo `estado`). Se regeneran con los comandos de abajo.

### Qué hay
| Pieza | Archivo | Qué hace |
|---|---|---|
| Generador | `src/atp_rover/gazebo/generar.py` | Escribe `mundo.sdf`, `rover.sdf`, `terreno.png` (heightmap 16 bit, 257×257 sobre 60 m), `color.png` (suelo del planeta con el relieve sombreado), `gui.config` y `origen.json`. Determinista por `(planeta, terreno, semilla)`. |
| Rover | `sdf_rover()` | 6 ruedas, balancín (rocker) y bogie por lado con articulaciones `revolute` (±0.5 y ±0.6 rad), chasis 0.9×0.56 m, masa total ≈ 22 kg. Ruedas de 0.15 m con fricción lateral menor que la de rodadura (skid-steer). |
| Terrenos | `altura()` | `llano`, `rocoso` (ruido + 70 rocas), `dunas`, `crater` (cuenco de 9 m de radio y 2.2 m en x=13, y=4), `pendiente` (14° desde x=4). Siempre llano en 3.5 m alrededor del origen y z=0 exacto en (0,0). |
| Planetas | `PLANETAS` | Gravedad luna 1.62, marte 3.71, titán 1.35, ceres 0.28, tierra 9.81 m/s²; cielo y suelo distintos por planeta (Luna negro/gris, Marte butterscotch/óxido, Titán ocre/pardo, Ceres azul noche/gris pardo, Tierra azul/verde), con la intensidad del sol y la luz ambiente de cada uno. Colores en espacio lineal como `gz_demo.sh`. |
| Ejecución | `scripts/gz_rover.sh` (+ `gz_rover_contenedor.sh`, `conducir.py`, `estado_gz.py`) | Igual que `gz_demo.sh`: Xephyr `:79`, contenedor `gz`, grabación x11grab, recorte de la barra y rótulos. |
| Visor web | `station/frontend/src/{lib/rover3d.js,components/Rover3D.jsx,components/rover3dEngine.js,views/RoverPage.jsx}` | Ruta `#/rover` (mock, o un `telemetry.jsonl` cargado a mano) y `#/runs/<id>/rover` (la corrida `kind: rover`: su telemetría y los `args` planeta/terreno/semilla). |

### Cómo correrlo
```bash
bash scripts/gz_rover.sh                                         # marte, rocoso, ruta demo, 50 s simulados
PLANETA=luna TERRENO=crater DUR=30 bash scripts/gz_rover.sh
VIDEO=0 bash scripts/gz_rover.sh                                 # sin Xephyr ni grabación: ventana en tu pantalla
python3 src/atp_rover/gazebo/generar.py /tmp/mundo --planeta titan --terreno dunas   # solo el mundo
python3 -m pytest src/atp_rover/gazebo/test_generar.py           # 12 pruebas, sin Gazebo
cd station/frontend && npm test && npx vite build                # 34 pruebas (10 del visor del rover) y build
```
Modos (`MODO`): **`diffdrive`** (por defecto) Gazebo simula el rover con el plugin DiffDrive (skid-steer de las 6 ruedas) y `conducir.py`
publica `/rover/cmd_vel` (`Twist`) por `ros_gz_bridge`; rutas `demo` (recta, arcos a izquierda y derecha, giro en sitio), `recta`, `giro` o un
JSON `[[t_s, v, w], ...]` en `RUTA`; se mide en tiempo **simulado** (`/clock` puenteado). Escribe `telemetria.jsonl` en el formato de
`/rover/estado` (solo `t, x, y, rumbo_deg, v, w`; lo demás se omite, §18) a partir de `/rover/odom`. **`estado`** Gazebo solo *dibuja* lo que
publique la dinámica de R1 en `/rover/estado`: `estado_gz.py` manda la pose al modelo (estático) con `set_pose_vector`; sin R1,
`ESTADO_REPETIR=<telemetria.jsonl>` reproduce un archivo. **`externo`** otro nodo publica `/rover/cmd_vel` y el script solo graba `DUR` s.

### Cuánto tarda y por software
Render por **software** (llvmpipe, `LP_NUM_THREADS=3`; el contenedor no tiene `/dev/dri`), 1280×720, en este host (8 hilos, carga < 4):
una ruta de 14 a 25 s simulados tarda 17 a 30 s de pared (factor de tiempo real 0.78 a 0.89), más 13 a 20 s de arranque de Gazebo y la GUI y
unos 10 s de rotulado: **31 a 48 s de pared por corrida corta**. La ruta `demo` completa (50 s simulados) ronda 1 min de grabación y 1.5 min en
total. `resumen.json` guarda `sim_medido_s`, `pared_ruta_s` y `rtf` de cada corrida; a 1080p el factor baja (no medido). Antes de lanzar,
el script espera a que la carga del host sea < 4 (el vigilante mata contenedores con carga > 12); si el vigilante para el contenedor, el video
queda con lo grabado hasta entonces. El visor web va en el navegador (three.js) y no usa Gazebo.

### Lo que se comprobó
* **Modelo y mundos:** `gz sdf -k` válido; los cinco mundos se generan con su gravedad (se prueba en XML) y se renderizaron con el rover rodando; el rover se asienta a
  z=0.60 m sobre el suelo en `rocoso` y `llano`. El rover avanza por `/rover/cmd_vel` (500 muestras de odometría en 25 s, 12.7 m con la ruta `demo`).
* **Terreno:** el heightmap de colisión y el visual coinciden (el rover rueda sobre el relieve que se ve); el rover sube y baja por lomas del
  `rocoso` con el balancín y el bogie moviéndose.
* **Visor web:** `npm test` 34/34, `vite build` limpio, capturas con Chromium + SwiftShader de los cinco planetas sin errores de consola
  (`node tools/shots_rover.mjs`).

### Límites y decisiones (leer antes de integrar)
1. **Sin barra diferencial.** Los dos balancines comparten eje, y sin la barra que los acopla el chasis cabecea libre hasta el tope.
   Se sustituye por un resorte-amortiguador PD de posición en cada articulación (`JointPositionController`, 120 y 30 N·m/rad): el
   suspensión sigue el terreno y el chasis queda a nivel, pero **no es el mecanismo real**; no usar para dinámica fina (eso es R1).
2. **DiffDrive no es la dinámica de R1.** Es skid-steer con fricción de Coulomb de ODE: sin Bekker–Wong, sin hundimiento ni deslizamiento
   por suelo. La gravedad del planeta cambia la carga sobre las ruedas, no la aceleración (que limita el plugin a 1 m/s²). Sirve para ver el
   modelo moverse, no para validar tracción.
3. **El terreno dibujado no es el de `terreno.py`.** `generar.py` y `rover3d.js` generan relieve propio (mismos tipos y parámetros, pero
   no los mismos números). Para ver *la corrida de R1 sobre su terreno* hace falta que R1 exporte el campo de alturas
   (`normalizarTerreno` en el visor acepta `{tipo, lado_m, n, z[]}` en `args.terreno_datos`) o que `generar.py` lea `terreno.altura()`.
4. **Heightmap con desplazamiento.** Gazebo no trata igual el desplazamiento vertical del terreno en la vista y en la colisión, así que el
   terreno va de z=0 a su altura máxima y el origen del rover queda a `z0 = -hmin` (`origen.json`); `estado_gz.py --z0` lo suma. El estado
   de R1 usa z=0 en el origen del terreno.
5. **Convenciones del estado por confirmar con R1:** `rumbo_deg` como guinada antihoraria desde +x (este), `cabeceo_deg` positivo morro arriba,
   `alabeo_deg` positivo lado derecho abajo, `z` del centro del chasis. Está en `estado_gz.py` (`quat`) y `lib/rover3d.js`, con pruebas.
6. **Articulaciones del visor web aproximadas.** El estado de §18 no trae los ángulos del balancín; el visor los estima de la pendiente del
   terreno bajo las ruedas. En el modo `estado` de Gazebo las articulaciones se quedan en reposo. Si R1 los publica, se cablean directo.
7. En el modo `externo`/`estado` el video no mide distancias (`telemetria.jsonl` solo lo escribe `conducir.py`).

### Sin probar
Integración con el nodo real `rover_dinamica` de R1 y con `/rover/estado` en vivo (se probó `estado_gz.py` con un archivo repetido, no con ROS);
`#/runs/<id>/rover` contra el backend real (no hay corridas `kind: rover` registradas todavía; en mock solo funciona `#/rover`); 1080p; la
cámara de seguimiento de Gazebo (`/gui/follow`) no mantiene el rover centrado al girar; rendimiento del visor web en un navegador con GPU
(en SwiftShader da 1 fps); gravedad de Ceres/Titán sobre el rover en suelo suelto (no hay modelo de suelo aquí).

## Resultados R4 (investigación)

Lazo cerrado sin ROS: `src/atp_rover/atp_rover/lazo.py` (dinámica de R1 + `Navegador` de R2) y estudio `scripts/estudio_rover.py`
(resultados en `docs/rover_resultados/`: `estudio.json`, `estudio.md`, `corridas.jsonl` con las 180 corridas). Pruebas: `src/atp_rover/test/test_lazo.py`.

### Integración
- `python3 -m atp_rover.cli … --navegacion=basico|completa`. **`basico` es el defecto y es el lazo de R1 sin cambios**: las tres escenas
  (`rover-luna-rocoso`, `rover-marte-dunas`, `rover-titan-pendiente`) dan los mismos bytes de `telemetry.jsonl` y `metrics.json` antes y después (comprobado con `cmp`).
  Con `completa`, `metrics.json` añade `navegacion`, `fallo` (`null`, `sin_ruta`, `seguridad`) y `alarmas`.
- El `Terreno` de R1 ya cumple el duck-type de R2 (`altura`, `normal` con z arriba); no hizo falta adaptador. `energia_wh` = energía **consumida** (CONTRATO §18).
- El navegador se evalúa a 10 Hz (con cada estado muestreado) y mantiene la orden entre muestras; las metas de `ruta` se planifican una tras otra.
- **Tres ajustes de R2 hechos para esta dinámica** (con los valores de R2 el rover casi no avanzaba; se midió antes de cambiar): (1) el control de rueda de R1 es proporcional
  (par ∝ error de velocidad), de modo que una orden pequeña en cuesta no da par y el rover retrocede: `v_min` 0.1 → 0.3 y la rebaja por pendiente empieza a 10° (`NAV_LAZO`);
  (2) el pure pursuit giraba con `v ≈ 0.1` (w ∝ v) y una esquina de 90° tardaba ~40 s: ahora gira en sitio si el objetivo queda a más de 45° (parámetro nuevo `giro_sitio_deg`,
  defecto 80 como antes: las 13 pruebas de R2 siguen pasando); (3) el antideslizamiento recortaba la orden con deslizamiento > 0.3, que en una subida de suelo blando es lo normal
  (0.3 a 0.4): umbral 0.5 (`TRAC_LAZO`). Todo se puede pisar con `nav` y `traccion` en la escena. Estos valores se eligieron mirando 5 variantes en Marte 15°, no se optimizaron.

### Método
Matriz planeta (luna, marte, titan, ceres, tierra) × terreno (llano, rocoso, dunas, crater, pendiente de 15°) × 3 semillas (1, 2, 3) × 2 navegaciones = 150 corridas, **la misma ruta** `[[12,0],[12,8]]`
(recta de 20 m, una esquina de 90°), `v_max` 0.5 m/s, tope de 150 s simulados, rover de 150 kg del modelo R1. Más un barrido de pendiente: terreno `pendiente` de 5° a 30° (pasos de 5°),
ruta recta de 12 m cuesta arriba, navegación `basico` (para aislar la dinámica del suelo), 1 semilla, 30 corridas. Total 180 corridas, unos 50 min a 5 procesos en este host (cada corrida de 80 s simulados tarda ~65 s de pared). Métricas de `metrics.json`; «Wh/m trac.» resta el consumo fijo de 20 W × tiempo (la parte que depende de suelo y gravedad); CdT = E/(m g d).
`python3 scripts/estudio_rover.py --rapido` (2 planetas × 3 terrenos × 1 semilla + 4 puntos de pendiente, 16 corridas) tarda ~4 min con 4 procesos.

### Supuestos y límites (leer antes de citar una cifra)
1. **Parámetros del suelo.** Tierra (arena seca de Wong: n 1.1, kc 0.99, kφ 1528.43, c 1.04 kPa, φ 28°): contrastado con una búsqueda (valores coinciden con los citados en la literatura de terramecánica); **K = 0.025 m no se
   contrastó**. Luna: la búsqueda indica que **no existen parámetros Bekker medidos in situ del regolito lunar**; los de `planetas.py` (n 1, kc 1.4, kφ 820, c 0.17 kPa, φ 35°) son estimaciones de orden de magnitud
   (tipo Apollo/Surveyor) y **quedan sin verificar**, y K es supuesto. Marte, Titán y Ceres: supuestos de orden de magnitud (nada medido para este modelo). Fuentes consultadas:
   [Predicting the performances of rigid rover wheels](https://www.kiss.caltech.edu/papers/xterramechanics/papers/predicting.pdf), [NASA TM-20250006958](https://ntrs.nasa.gov/api/citations/20250006958/downloads/TM-20250006958.pdf).
   Las comparaciones **entre planetas** valen por la gravedad (verificada) y por una diferencia de suelo que es en buena parte supuesta.
2. Un solo rover, una sola ruta de 20 m, 3 semillas por celda (una celda es 3 corridas: 2/3 y 3/3 no son tasas). Sin ruido de sensores: el navegador ve el estado exacto y el terreno exacto.
3. El modelo de contacto es el simplificado de R1 (rueda esfera, sin balancín, sin hundimiento del cubo); el rover no vuelca en ninguna de las 180 corridas, así que **la reducción de vuelcos no se pudo medir**.
4. La navegación `completa` está **poco ajustada** (ver Integración); lo medido es «el navegador de R2 con estos ajustes», no «la navegación por A*».
5. Criterio de «pendiente segura» (supuesto mío): meta alcanzada, 0 atascos, 0 vuelcos y deslizamiento medio ≤ 0.5. Resolución de 5°.
6. `crater`: la meta (12, 0) cae dentro del cuenco (pendiente local > 20°); la navegación `completa` se niega (`sin_ruta`), la `basico` sube hasta 35° y no llega.

### Resultados
**P1. Costo de transporte con la gravedad y el suelo** (llano, navegación básica, media de 3 semillas, 20 m a ~0.4 m/s):

| Cuerpo | g (m/s²) | Wh/m | Wh/m de tracción | CdT |
|---|---|---|---|---|
| Ceres | 0.27 | 0.020 | 0.003 | 1.76 |
| Luna | 1.62 | 0.023 | 0.009 | 0.35 |
| Titán | 1.35 | 0.024 | 0.008 | 0.42 |
| Marte | 3.71 | 0.033 | 0.019 | 0.21 |
| Tierra | 9.81 | 0.057 | 0.041 | 0.14 |

- En simulación, con estos supuestos: el Wh/m total crece con la gravedad (Tierra 2.5× Luna), pero **menos que ella** porque a 0.5 m/s los 20 W fijos suponen ~0.011 Wh/m
  y dominan en Ceres, Luna y Titán (la parte de tracción de Tierra es 4.6× la lunar y 14× la de Ceres). El CdT adimensional **baja** con g (1.76 → 0.14): el costo fijo se reparte
  sobre un peso menor. El suelo pesa más que la gravedad en terreno difícil: de llano a dunas el Wh/m se multiplica por 1.5 (Luna), 1.8 (Marte) y 4.7 (Tierra); a 15° de pendiente por 2.7 (Luna), 3.1 (Marte) y 6.4 (Tierra)
  (navegación básica; `estudio.md`). Titán es el caso extremo del suelo (φ 26° supuesto): 0.224 Wh/m en la pendiente sin llegar a la meta (0.84 de deslizamiento).
- Lectura: la energía de un rover depende del consumo fijo mientras avanza lento y de la pendiente y del suelo blando cuando se complica; la gravedad por sí sola explica poco.

**P2. Navegación completa frente a básica** (60 corridas por navegación sin el cráter; las 15 del cráter aparte):

| Terreno | Metas básica | Metas completa | Atascos básica | Atascos completa |
|---|---|---|---|---|
| llano | 15/15 | 15/15 | 0 | 0 |
| rocoso | 15/15 | 15/15 | 0 | 0 |
| dunas | 13/15 | 7/15 | 7 | 16 |
| pendiente 15° | 12/15 | 3/15 | 9 | 6 |
| cráter | 0/15 | 0/15 (15 `sin_ruta`) | 17 | 0 |
| vuelcos (150 corridas) | 0 | 0 | | |

- **En esta integración la navegación completa no reduce los atascos ni mejora las metas; en dunas y cuestas es peor** (55/60 metas contra 40/60 sin cráter; 16 contra 22 atascos). Causa visible en las trazas:
  rebaja la velocidad en pendiente y el rover con control de rueda proporcional sube peor despacio (Luna 15°: 80 s básica, 145 s completa).
- **Dónde sí ayuda**: en el cráter evita meterse (0 atascos, pero tampoco llega: rechaza la meta), y en la Tierra `rocoso` rodea las rocas (pendiente máx. 4.4° contra 29.9°, 0.049 contra 0.069 Wh/m, 3/3 ambas).
- No se pudo medir el efecto sobre vuelcos (cero en ambas). Atascos «evitados» en el cráter son rechazos de la misión, no conducción mejor.

**P3. Pendiente máxima segura por planeta** (básica, criterio de §Supuestos 5, pasos de 5°; entre paréntesis el deslizamiento medio):

| Planeta | φ (°) | 5° | 10° | 15° | 20° | 25° | 30° | Máx. seguro |
|---|---|---|---|---|---|---|---|---|
| Luna | 35 | sí (0.16) | sí (0.29) | sí (0.46) | no (0.69) | atasco | atasco | 15° |
| Marte | 33 | sí (0.15) | sí (0.25) | sí (0.40) | no (0.60) | atasco | atasco | 15° |
| Ceres | 32 | sí (0.29) | sí (0.46) | no (0.70) | atasco | atasco | atasco | 10° |
| Tierra | 28 | sí (0.21) | sí (0.37) | no (0.58) | atasco | no (0.87) | atasco | 10° |
| Titán | 26 | sí (0.32) | no (0.57) | atasco | atasco | atasco | atasco | 5° |

- En simulación, con estos supuestos, el límite de tracción queda **por debajo** de φ en todos los cuerpos y 20° ya atasca en los cinco: el `pend_max_deg` = 20° por defecto de R2 y la alarma de pendiente (25°) **no protegen de
  quedarse atascado** en este modelo (solo del vuelco). Una cota práctica para planificar es 10° (5° en Titán), con los suelos supuestos. El orden Luna ≈ Marte > Ceres > Tierra > Titán sigue casi el de φ, salvo Tierra/Ceres.

### Sin probar
Nodo ROS con la dinámica real; sensibilidad a los parámetros del suelo (barrido de φ, c, K) y a las semillas más allá de 3; otras rutas y velocidades; ajuste fino de la navegación (el hallazgo de P2 puede cambiar);
vuelco (hace falta una pendiente lateral o rocas más altas); contraste de los parámetros lunares y marcianos con datos medidos.


## Resultados R6 (mecánica y navegación)

Rama `worktree-agent-acd065302acf4cad7`. Archivos: `rover.py`, `terreno.py`, `navegacion.py`, `seguridad.py`, `lazo.py`, `cli.py`; pruebas `test_rover_bogie.py` y
`test_navegacion_r6.py`; escenas `experiments/rover-marte-rocas.yaml`, `rover-luna-escalon.yaml`, `rover-tierra-lateral.yaml`; estudio repetido en `docs/rover_resultados_r6/`.
Todo es **en simulación, con los suelos supuestos de R4**. Regla de compatibilidad: con los defectos (`suspension: simple`, `rodadura_en_motor: false`, `navegacion: basico`) las tres
escenas de R1 dan los mismos bytes (comprobado con `cmp` y con huellas sha256 en la prueba `test_simple_conserva_los_bytes_de_r1`).

### 1. Rocker-bogie real (`suspension: rocker_bogie`)
- **Cinemática.** Por lado, el balancín gira sobre el chasis (pivote en x = 0, z = +0.05 m respecto al CM) con ángulo θ; la rueda delantera va en su extremo delantero; el bogie
  cuelga del extremo trasero (x = −0.25 m) con ángulo φ relativo al balancín y lleva la rueda media y la trasera. La **barra diferencial** impone θ_izq = +δ, θ_der = −δ.
  Estados extra: δ, φ_izq, φ_der. Mecanismo sin masa: fuerza y par sobre el chasis = suma de las fuerzas en los cubos (en su posición real) menos el par de los motores; cada
  articulación se mueve a Q/c_art (sobreamortiguada, c_art = 250 N·m·s/rad, constante de tiempo ~0.05 s) con Q = fuerza generalizada de los contactos (equilibrio de momentos).
  Topes ±30° (balancín) y ±40° (bogie): **supuestos**. Geometría de 2:1 elegida para que el reparto en llano sea 1/6 por rueda.
- **Reparto de carga** (`carga_n` en el estado): en llano 92.75 N por rueda en Marte (= m g / 6, a 0.02 %); en cuesta de 15° (Marte) 75 / 61 / 133 N (delantera / media / trasera, cuesta arriba hacia +x) con suma = m g cos 15° a 3 %.
- **Contacto por envolvente** (con `rocker_bogie`, `contacto: auto`): se muestrea el perfil del disco (13 puntos) y se toma el de mayor penetración, con la normal del contacto hacia el cubo; solo se impone
  sobre el contacto puntual de siempre si penetra > 1 cm más. Sin ancho de rueda (una línea).
- **Campos opcionales** (CONTRATO §18): `balancin_deg` [izq, der], `bogie_deg` [izq, der] (positivo = extremo delantero del eslabón hacia abajo), `carga_n` [6], `margen_vuelco_deg`; en `metrics.json`: `suspension`, `margen_vuelco_min_deg`. Con `simple` no aparece ninguno. `pendiente_deg` con rocker-bogie es la del plano medio de las normales de contacto (una roca no cuenta como pared bajo el CM).
- **Margen de vuelco**: ángulo que falta para que el CM quede sobre el borde más comprometido del polígono de apoyo (casco convexo de los contactos). En llano 45.0° (atan(0.40/0.40), el ángulo estático del modelo); en una pendiente lateral de 20°, 25° (±1.5°); la escena `rover-tierra-lateral` baja a 24.5°.
  `seguridad.py` lanza la alarma `vuelco` si el margen baja de 8° (parámetro). **Sin validar en campo de rocas**: en `rover-marte-rocas` el margen mínimo sale −42° (CM fuera del polígono con tres contactos), cifra poco creíble que dispara la alarma 41 veces aunque el rover llega; falta revisar cuándo el polígono es degenerado.
- **Altura máxima de escalón** (rueda de 0.25 m, 0.5 m/s mandado, escalón de borde vertical a 3 m, 45 s de tope; ok = pasa x = 5.5 m):

| Cuerpo | g (m/s²) | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | Máx. (m) | Máx./radio |
|---|---|---|---|---|---|---|---|---|
| Luna | 1.62 | sí | sí | sí | no | no | 0.20 | 0.8 |
| Marte | 3.71 | sí | sí | no | no | no | 0.15 | 0.6 |
| Tierra | 9.81 | sí | no | no | no | no | 0.10 | 0.4 |
  Con 0.35 m en la Luna y 0.40 m en Marte el rover vuelca de cabeza (cabeceo > 70°). Resolución de 0.05 m; una sola velocidad y un solo suelo. **El límite baja con la gravedad** (más carga por rueda, más agarre contra el borde),
  y queda muy por debajo del radio de la rueda: lo que la literatura de rocker-bogie suele citar (superar obstáculos del orden del diámetro de la rueda) **no se reproduce** con este suelo, par y velocidad. La suspensión `simple` (contacto puntual) en Marte da el mismo 0.15 m, así que en este ensayo **no se ve una ventaja del rocker-bogie**; su aporte medido es el reparto de carga y las articulaciones.

### 2. Terreno: rocas, escalones, pendiente lateral
- `terreno: rocas`: rocas discretas, semi-elipsoides de borde vertical, tamaños con la distribución de Golombek y Rapp (1997) para Marte: fracción de área cubierta por rocas de diámetro ≥ D, F(D) = k·exp(−q D), q = 1.79 + 0.152/k (D en m; la fórmula se confirmó en la búsqueda web como la del artículo, sin leer el PDF).
  Parámetros `abundancia` k (0.10), `d_min` 0.15, `d_max` 0.80, `radio_rocas` 30, zona libre de 3 m. Altura = D·U(0.3, 0.6) y razón de ejes U(0.6, 1): **supuestos de forma**. Determinista por semilla (rng PCG64, número de rocas Poisson). Con k = 0.10 hay 0.9 rocas/m² (D ≥ 0.15) y el área cubierta medida es ~4 % (esperado ~5–6 %: solapes y forma). Una semilla de 60 m de lado con k = 0.10 da ~3300 rocas con una de hasta 0.46 m.
- `terreno: escalones` (`altura`, `x_inicio`, `separacion`, `n`) y `pendiente` con `direccion_deg` (90° = sube hacia +y: pendiente **lateral** para un rover que avanza por +x). Sin `direccion_deg` los bytes del terreno son los de siempre.
- **Vuelco real** (prueba `test_vuelco_real_en_pendiente_lateral_sobre_el_angulo_estatico`, Tierra con φ = 70° para que no deslice antes): a 30° queda estable con margen > 5° y a 55° vuelca. Con el suelo real de la Tierra (φ = 28°) a 20° el rover **desliza antes de volcar** (`rover-tierra-lateral`: margen mínimo 24.5°, 1 atasco, no llega a la meta en 40 s). Es decir, el vuelco por pendiente lateral solo aparece con suelo de mucho agarre o con un obstáculo; el vuelco por escalón alto sí (ver arriba).

### 3. Navegación completa: qué se cambió y qué dio
Diagnóstico de R4 verificado: con control de rueda **proporcional** (par ∝ error de velocidad), bajar la orden en cuesta baja el par. Medido en pendiente de 15° (Marte): 0.3 m/s mandados dan 0.11 m/s; 0.9 dan 0.48; el deslizamiento queda fijo (~0.39) porque lo fija la fuerza que pide la cuesta, no la orden (Tierra: 0.61, Titán a 10°: 0.57). La velocidad lograda es ≈ (1 − s)·orden.
Cambios (todos con `navegacion: completa`; `completa_r4` conserva el comportamiento de R4: reproduce su telemetría byte a byte):
(a) **costo graduado y direccional** (`planificar_ruta_graduada`): subir, cruzar de costado y rugosidad cuestan más pero no prohíben; solo lo extremo (≥ 35°) se prohíbe, así la meta del cráter ya no da `sin_ruta`; atajos solo si no cuestan > 8 % más;
(b) replanificación cada 10 s y al desviarse 2 m; (c) **prealimentación + PI de velocidad** sobre la `v` medida (orden = v/(1 − s) con s ≤ 0.65, tope v + 0.9) y la pendiente ya no baja la velocidad; sin prealimentación cuando hay pendiente de costado > 4° (con alto deslizamiento el círculo de fricción deja sin agarre lateral: caso Titán dunas semilla 2) y con antideslizamiento a 0.5 en ese caso;
(d) guarda contra encabritarse (cabeceo > terreno + 8°) y límite de aceleración de la orden (1.5 m/s²; con poca gravedad un escalón de orden voltea el rover por el par de reacción); (e) **vaivén** (atrás 2.5 s, adelante 3 s, hasta 5 veces por meta) al detectar atasco, con límite de subida aprendido (−2° respecto a la cuesta donde se atascó) y sitio evitado; tras 5 intentos `fallo: inalcanzable`.

**Estudio repetido** (`scripts/estudio_rover.py`, mismos 75 casos y semillas que R4; las corridas `basico` se sembraron desde R4 porque su código y dinámica no cambiaron: bytes idénticos):

| Terreno | Metas básica | Metas completa R4 | Metas completa R6 | Atascos básica | Atascos R4 | Atascos R6 |
|---|---|---|---|---|---|---|
| llano | 15/15 | 15/15 | 15/15 | 0 | 0 | 0 |
| rocoso | 15/15 | 15/15 | 15/15 | 0 | 0 | 0 |
| dunas | 13/15 | 7/15 | 11/15 | 7 | 16 | 25 |
| pendiente 15° | 12/15 | 3/15 | 9/15 | 9 | 6 | 39 |
| cráter | 0/15 | 0/15 (15 `sin_ruta`) | 0/15 (0 `sin_ruta`) | 17 | 0 | 34 |
| total sin cráter | 55/60 | 40/60 | **50/60** | 16 | 22 | **64** |

- **El criterio «completa ≥ básica en metas y no más atascos» NO se cumplió**: 50/60 contra 55/60 metas y 64 contra 16 atascos (sin cráter). Mejoró mucho respecto a R4 en metas (40 → 50; pendiente 3/15 → 9/15 y Marte, Luna y Tierra en pendiente de 15° ya igualan a la básica) pero empeoró en atascos.
- **Dónde pierde** (todo con poca gravedad o suelo blando): Ceres pendiente 15° 0/3 (la básica 3/3), Titán pendiente 0/3 (la básica también 0/3), Titán dunas 1/3 (básica 2/3), Ceres dunas 2/3 (básica 3/3). Las trazas muestran el rover resbalando cuesta abajo (deslizamiento 1.0) y alarmas de cabeceo; la básica pasa en Ceres por la arrancada brusca a 0.5 m/s, que el límite de aceleración y las guardas quitan. Mi lectura (no probada con un experimento de ablación completo) es que en esos casos el rover está en el borde de lo que el modelo permite y la básica gana por inercia de arranque.
- **Los «atascos» se inflan**: el contador del rover cuenta cada episodio de 5 s con |v| < 20 % de la orden, y el vaivén repite intentos; no son más rover atascado, pero la métrica es la pedida y se reporta tal cual. Variantes intermedias sobre las mismas corridas: primera versión (aceleración 0.5 m/s², sin tope de vaivenes) 47/60 metas y 101 atascos; con 3 vaivenes y 1.2 m/s² 48/60 y 47 atascos (la de 5 vaivenes ganó 2 metas y perdió 17 atascos de margen).
- **Cráter**: la meta está en una pared de 19° a 36°; ahora la navegación completa intenta ir (ya no rechaza), 0/15 igual que la básica, y es la seguridad (pendiente > 25°) o el tope de tiempo lo que termina la corrida; ningún vuelco real. Con 1 vuelco en 75 corridas de la completa (Ceres cráter).
- **Vuelcos**: 0 en la básica; 1 en la completa (Ceres cráter). Sigue sin medirse una reducción de vuelcos por navegar.
- Ajustes hechos mirando estas mismas 75 corridas (iteré 4 veces sobre ellas): **el número de R6 está sobreajustado a este conjunto**; no se validó con otras rutas ni semillas.

### 4. Rodadura: el motor debe vencer la resistencia por compactación (hallazgo de R5)
En R1/R4 el par de la rueda usaba `fx = T − Rc`, así que la resistencia por compactación Rc ayudaba a girar la rueda en vez de frenarla; la potencia de tracción quedaba muy por debajo del mínimo Rc·v/η. Corrección **opt-in** `rover: {rodadura_en_motor: true}` (par = (T + Rc)·Rw); el defecto sigue siendo el de R1/R4 para no cambiar los bytes de las escenas.
Antes / después en llano, 20 m, 0.4 m/s mandado, Wh/m de tracción (energía − 20 W·t) / distancia:

| Cuerpo | Antes (R1/R4) | Después | Tiempo (s) antes → después |
|---|---|---|---|
| Ceres | 0.0010 | 0.0021 | 54.8 → 55.0 |
| Luna | 0.0010 | 0.0112 | 52.0 → 53.7 |
| Titán | 0.0011 | 0.0120 | 55.8 → 57.6 |
| Marte | 0.0010 | 0.0326 | 52.1 → 57.5 |
| Tierra | 0.0010 | 0.1159 | 53.8 → 77.7 |

Antes la tracción casi no dependía del cuerpo (0.001 Wh/m); después sigue la gravedad (Tierra 55× Ceres). **Hasta que R4 se repita con `rodadura_en_motor: true`, las cifras de CD-2026-30 P1 («Wh/m de tracción», CdT, «la gravedad explica poco») no deben citarse.** No se repitió el estudio completo con la física corregida. También se arregló el temporizador de atasco de `_acumular` (usa 10·DT en vez de 0.1: idéntico con DT = 0.01).

### Sin probar
Nodo ROS (ni `rclpy` ni la nueva alarma de margen); rocker-bogie en terreno `rocas` con la navegación (solo una escena: `rover-marte-rocas` llega a la meta en 75 s con `meta_alcanzada` pero margen −42° dudoso); sensibilidad a los topes, a c_art y a la geometría 2:1; escalones con otras velocidades y suelos; repetir el estudio de R4 con `rodadura_en_motor` y con rocker-bogie; contraste de la altura de escalón con rovers reales; rocas más allá de Marte (la distribución es de Marte).
## Resultados R7 (comunicación con el rover por antenas)

Fase 2 (CONTRATO §19). Módulo `src/atp_rover/atp_rover/comunicacion.py` (Python puro), nodo `nodo_enlace.py` (`rover_enlace`), escenas `experiments/rover-enlace-{marte,luna}.yaml`
(`kind: proceso`, `area: rf`), estudio `scripts/estudio_enlace_rover.py` y resultados en `docs/rover_resultados/enlace.{json,md}` (todas las tablas están ahí).
Pruebas: `src/atp_rover/test/test_comunicacion.py` (20, ~6 s). No se tocó `rover.py`, `navegacion.py` ni `lazo.py`; se reutiliza `Terreno.altura` y `PLANETAS`.
Se reutiliza de `atp_ros.model` la fórmula de la pérdida en espacio libre (92.45 + 20 log d_km + 20 log f_GHz, comprobada por una prueba) y se sigue el mismo criterio de
escalera de tasas con margen de DVB-S2; no se importan los módulos de constelaciones terrestres (SGP4 y TLE no aplican a una órbita de Marte).

### Qué modela
- **Relé**: órbita kepleriana de dos cuerpos (sin J2 ni precesión; vale para días, no meses) con los elementos de MRO, Odyssey y MAVEN de la guía JPL 2019 (época 2019); en la Luna, un relé polar circular de 100 km y una elipse «NRHO» de dos cuerpos con el periodo (6.56 d) y el perilunio (~1500 km de altura) de la NRHO, que **no es** la NRHO real (tres cuerpos). El cuerpo gira a su periodo sideral; el rover está en (lat, lon).
- **Visibilidad**: elevación del relé ≥ max(`el_min_deg`, horizonte local del terreno en ese azimut). El horizonte se traza sobre `Terreno.altura` hasta 60 m de una antena a 1 m del suelo (convención asumida: x = este, y = norte).
- **UHF Marte** (401.585625 MHz): potencia recibida = 7 W − 1 dB + G(rover) + G(relé, 4 dBi con caída fuera del nadir) − FSPL − 2 dB; tasa = la más alta de la escalera de Electra (1 kbps a 2 Mbps) cuyo umbral de la tabla 6.2-2 de la guía JPL se supera con 2 dB de margen; Odyssey topa en 256 kbps. Eficiencia útil 0.8 y pérdida de trama logística (1.8 % con 2 dB de margen): formas supuestas.
- **Antenas del rover** (supuestas): `omni_lineal` (monopolo, 2.15 dBi, nulo al cenit), `omni_helice` (cuadrifilar, +3 dBi cenit, −3 dBi horizonte), `direccional` (10 dBi, haz 60°, error de apuntamiento 12 (e/θ)² dB). X directa: `hga_x` (25 dBi, 15 W, DSN 34 m con G/T 51 dB/K) y `lga_x` (7 dBi).
- **Retardo de luz**: distancia entre órbitas circulares coplanares con las longitudes medias de J2000 de Standish; Marte 0.52–2.52 UA (4.4–21 min una vía; el rango real es ~0.37–2.67 UA, así que el modelo se equivoca hasta ~0.15 UA). Luna: 384 400 km fijos (1.28 s).
- **Comandos y telemetría** (`FiltroEnlace`, `Enlace.entrega_*`): un comando llega por el relé (retardo de luz + espera de ventana relé–Tierra + espera del siguiente contacto UHF) o por X directo si el rover ve la Tierra; gana el que llegue antes. La telemetría se guarda y sale en el siguiente contacto (DTN: `ColaDTN`, bundles FIFO, almacenamiento acotado, retransmisión en el siguiente contacto), luego espera la ventana del relé hacia la Tierra (turno DSN supuesto de 1/3 de sol) y el retardo de luz.

### Cifras (simulación, estas hipótesis; 20 soles, Gale 4.6 S 137.4 E, helicoidal, 10° de elevación mínima)
| Relé | Sesiones/sol sim. | Publicado | Mbit/sol sim. | Publicado | Mbit/sesión sim. | Publicado |
|---|---|---|---|---|---|---|
| Odyssey | 2.70 | 2.2 | 237 | 173 | 88 | 78.5 |
| MRO | 2.10 | 2.3 | 475 (70–987 según el sol) | 498.2 | 226 | 221.2 |
| Odyssey + MRO | 4.80 | 4.5 | 712 | 671.3 | | |

«Publicado» es el primer año de Curiosity en la guía JPL 2019, tabla 2-1 ([PDF](https://discovery.larc.nasa.gov/PDF_FILES/21_Proposers_Guide_To_Mars_Orbiters_-_Discovery_2019_AO_-_Rev_190411b.pdf)); otras fuentes consultadas: [arXiv 2211.14245](https://arxiv.org/pdf/2211.14245) (100–250 Mbit por orbitador y sol; Curiosity directo 500 bps–32 kbps) y [NASA Mars Relay Network](https://science.nasa.gov/mars/mars-relay-network/).
**Lectura honesta**: el orden de magnitud coincide sin haber ajustado nada al dato publicado, pero la potencia del rover (7 W), las ganancias y el plan de contactos son supuestos, así que **esto no valida el modelo**; el acuerdo puede ser casual (los errores de tasa y de número de pases pueden compensarse). El volumen varía hasta 14× de un sol a otro solo por la geometría del nodo.

- **Volumen por pase** (MRO): 58 Mbit con elevación máxima 10–25°, 241 con 25–40° y 430–480 con más de 40° (tasa pico 284 kbps contra 2 Mbps): la elevación del pase manda, no su duración (5 contra 9 min).
- **Elevación mínima de uso**: de 5° a 30°, las sesiones bajan de 2.70 a 1.00 por sol y el volumen de 511 a 285 Mbit/sol; la espera mediana de un comando por el relé sube de 321 a 714 min (DSN siempre disponible) y la del p95 de 670 a 4611 min. Con un turno DSN de 1/3 de sol (supuesto arbitrario) la mediana sube a 729–1294 min: domina la espera.
- **Inclinación orbital** (relé tipo MRO, mismo resto de elementos): en el ecuador (4.6 S) 30° da 4.00 sesiones/sol y 982 Mbit/sol contra 2.10 y 475 de 92.7°, y la espera mediana de un comando baja de 357 a 243 min; a 60° de latitud, 30° de inclinación no da ningún pase, 74.8° da 6.65 sesiones/sol (la guía JPL cita 1–6 sesiones/sol a 30–75° para MAVEN, de 74.8° de inclinación).
- **Relieve local**: un hoyo de 15 m de radio y 3 m de profundidad levanta el horizonte a 10.6°, casi la máscara de 10° (−1 % de volumen); uno de 15 m y 8 m lo levanta a 27.4° y recorta el volumen a 314 Mbit/sol (−34 %), deja 1.05 sesiones/sol y lleva el p95 de la espera de 687 a 4610 min. Es un hoyo de decenas de metros, no un cráter de kilómetros.
- **Antenas** (MRO, Gale, llano): helicoidal 475, monopolo 634, direccional de 10 dBi con 5° de error 1425 y con 25° de error 1326 Mbit/sol; sensibilidad a la ganancia: +6 dB × 2.02, +3 dB × 1.46, −3 dB × 0.65, −6 dB × 0.41. En este modelo conviene una antena de **ganancia moderada (≥ 6–10 dBi) con apuntamiento grueso** (con 25° de error aún casi triplica a la helicoidal), sobre todo en pases de baja elevación; la omni queda como respaldo sin mecanismo. Depende de la ganancia supuesta de la helicoidal y de un haz de 60° que no se verificó.
- **Directo en banda X** (HGA 25 dBi, 2 h por sol, supuesto): 0.5–8 kbps según la distancia, es decir 4–58 Mbit/sol, contra 475 por el relé: el relé da 8× (en el mejor caso, oposición) a ~100× más datos. La LGA de 7 dBi no cierra el enlace a más de 0.5 UA. El modelo da un máximo de 8 kbps, menos que los 32 kbps citados para Curiosity (posiblemente con DSN de 70 m o más potencia): subestima.
- **Luna** (banda S supuesta): relé polar de 100 km, polo sur 12.2 pases/día y 4760 Mbit/día; en el ecuador 1.8–1.9 pases/día y 319–357 Mbit/día; la elipse «NRHO» cubre el polo sur el 97 % del tiempo pero con ~3 Mbit/día por la distancia (la antena helicoidal del rover no sirve a 60 000 km; un relé real usaría otro enlace). La Tierra se ve el 100 % del tiempo en la cara visible y nunca en la oculta (sin libración).
- **DTN** (MRO, 300 Mbit/sol generados): se entregan 269 Mbit/sol, latencia mediana 17.0 h y p95 53.5 h (contacto UHF + turno DSN + luz); a 500 se entregan 418 con mediana 31.8 h y a 800 solo 454 con la cola creciendo (821 bundles en el rover tras 20 soles): el cuello es el volumen del enlace UHF en los soles malos y el turno DSN.

### Cómo reproducir
```bash
python3 -m pytest -q src/atp_rover/test/test_comunicacion.py      # 20 pruebas
python3 scripts/estudio_enlace_rover.py                            # ~45 s; --rapido: 6 soles
python3 scripts/enlace_rover.py --escena experiments/rover-enlace-marte.yaml --salida /tmp/rover --id e1
```

### Sin probar / límites
- El nodo `rover_enlace` se compiló (`colcon build`) y se probó en el contenedor con `reloj:=pared`, modo `retardo`, Luna: una orden de 0.7 m/s publicada en `/rover/cmd_vel_op` llegó a `/rover/cmd_vel` a los 1.23 s de pared (retardo de luz de 1.28 s, tick de 0.1 s) y un estado de `/rover/estado` salió por `/rover/estado_tierra` con `t_tierra`. **No se probó** con `/clock` (lockstep §8), ni en modo `ventanas`, ni encadenado con `rover_dinamica` y `rover_navegador`; la prueba fue de un solo mensaje y manual (no está en `validar_todo.sh`).
- Umbrales de Electra: el texto de la guía da «dB» sin signo; se interpretaron como dBm negativos (coherente con un piso de ruido de ~−108 dBm a 2 Mbps). Potencia del rover, ganancias, pérdidas, forma de la pérdida de trama, tasa del relé a la Tierra de la Luna, turno DSN y horas de transmisión directa: **supuestos propios, sin fuente**. Longitudes medias de Standish, periodo de la NRHO y su perilunio: recordados, no recontrastados.
- Sin precesión (MRO/Odyssey son heliosíncronas: el nodo se mueve ~0.5°/día), sin J2, sin eclipses del relé, sin ocultación del rover por su propio chasis o inclinación, sin libración lunar ni excentricidad de la órbita de la Luna, sin atmósfera (el UHF casi no la siente). Un solo rover por relé (sin contención).
- Un rover real planifica sus contactos y reparte el tiempo del relé entre varios usuarios; el modelo toma todos los pases geométricos.
(R5 hizo la sensibilidad, el contraste y encontró un defecto que afecta a P1: ver «Resultados R5».)

## Resultados R4b (repetición de R4 con la física corregida)

**Por qué.** R5 halló que en `rover.py` el motor no pagaba la resistencia por compactación (`Rc`), así que el Wh/m «de tracción» de R4 omitía el costo de rodar; R6 lo corrigió como opción (`rodadura_en_motor`, desactivada por defecto para no cambiar los bytes de las escenas históricas). Se repitió el estudio completo (mismas 180 corridas, ruta, semillas y criterios) con `python3 scripts/estudio_rover.py --rodadura-en-motor --trabajadores 4 --salida docs/rover_resultados_r4b` (~15 min con el host libre). Navegación `completa` = la de R6 (la de R4 no se repitió). Resultados en `docs/rover_resultados_r4b/` (estudio.json/md, corridas.jsonl). En simulación, con estos supuestos.

### Costo de transporte en llano, navegación básica (R4b)

| Cuerpo | Wh/m total | Wh/m tracción | CdT (adim.) |
|---|---|---|---|
| ceres | 0.021 | 0.004 | 1.86 |
| luna | 0.033 | 0.018 | 0.48 |
| titan | 0.034 | 0.018 | 0.61 |
| marte | 0.061 | 0.046 | 0.39 |
| tierra | 0.154 | 0.134 | 0.38 |

Lectura (con prudencia):
- Los Wh/m de **tracción** suben mucho respecto a R4 (Tierra 0.041 → 0.134, Marte 0.019 → 0.046, Luna 0.009 → 0.018): ahora incluyen el costo de rodar. El total de Tierra pasa de 0.057 a 0.154 Wh/m.
- La conclusión de R4 «el costo adimensional baja con la gravedad de 1.76 a 0.14» **no se sostiene**: con la física corregida el CdT de Luna, Marte y Tierra es casi el mismo (0.48, 0.39, 0.38), y solo Ceres (1.86) y Titán (0.61) quedan altos, y esos dependen de suelos **supuestos** de orden de magnitud. Con esta física la energía por metro crece con la gravedad en términos absolutos, pero el costo adimensional casi no depende de ella entre Luna y Tierra.
- **Pendiente máxima segura: sin cambio** (Luna 15°, Marte 15°, Ceres 10°, Tierra 10°, Titán 5°).
- **Básica contra completa (R6), 75 casos cada una:** metas sin cráter 52/60 (básica) contra 45/60 (completa); atascos sin cráter 20 contra 80 (inflados por el vaivén de recuperación, que el contador cuenta por episodio de 5 s); vuelcos 0 contra 2; 3 corridas de la completa abortadas por seguridad. La navegación completa de R6 **no** supera a la básica con esta física; el resultado negativo de R4 se mantiene.
- Límites: los mismos de R4 (suelos supuestos, una ruta, 3 semillas) y que R6 afinó la navegación sobre estos mismos casos.

## Resultados R5 (validación)

Equipo de validación científica del modelo de R1. No se tocó `rover.py`, `terreno.py` ni `planetas.py` (los tiene R6). Script `scripts/validacion_rover.py`
(`--rapido` ~6 min; completo ~45 min con 6 procesos), resultados en `docs/rover_resultados/validacion.{json,md}` (tablas completas e IC95 %), pruebas
`src/atp_rover/test/test_validacion.py` (8, ~1 min, en `scripts/validar_todo.sh`). Todo es **en simulación, con estos supuestos**; el muestreo es determinista
(Sobol con mezcla, semilla 20260507). Corridas de lazo abierto, 0.5 m/s mandado, 6 s (sensibilidad) o 10–16 s (convergencia), rover de 150 kg.

### 1. Convergencia numérica
Criterio de aceptación (fijado antes de mirar): el paso de producción dt = 0.01 s difiere del paso más fino (0.0025 s) menos de 2 % en distancia y Wh/m y menos de 0.01 en
deslizamiento medio; la resolución del terreno res = 0.1 m difiere de 0.05 m menos de 5 % (media de 3 semillas) y 0.02 en deslizamiento.
- **dt** (7 casos: 5 cuerpos en llano, Marte a 10° y Marte rocoso): error de la distancia ≤ 0.01 %, de Wh/m 0.24–0.25 % (0.05 % en la cuesta), del deslizamiento ≤ 2·10⁻⁴. **Cumple.**
  El orden observado de Wh/m es ~0.9, no el 4 del RK4: la energía se integra con la potencia del inicio del paso (regla del rectángulo); la distancia converge más rápido.
  Con dt = 0.02 s el error de Wh/m es ~0.5 % (no se probó la estabilidad con otras ganancias).
- **Terreno** (Marte, `rocoso` y `dunas`, 3 semillas cada uno, rover recorriendo 7 m de terreno ya rugoso): res 0.1 contra 0.05: dunas 0.17 % en distancia y 0.22 % en Wh/m; rocoso ≤ 0.05 %.
  res 0.2 contra 0.05: ≤ 0.8 %. **Cumple.** Limitación: la rejilla fija los puntos de la normal (diferencias a `res`) y las rocas gaussianas (σ ≥ 0.2 m) están resueltas con 0.1 m;
  rocas más pequeñas o ruedas más finas no se probaron.

### 2. Sensibilidad global del suelo y de g
Sobol–Saltelli (N = 32; 512–576 corridas por análisis) con seis parámetros del suelo (φ, c, K, n, kc, kφ). **Los rangos de incertidumbre son elecciones mías, no datos**: Marte ±5° en φ,
c ×/÷3, K ×/÷2, n ±0.1, kc y kφ ×/÷2; Titán y Ceres (suelo totalmente supuesto) ±8°, c ×/÷5, K ×/÷3, n ±0.15, kc y kφ ×/÷3.
- **Qué domina.** El índice total ST de **K** (módulo de deformación cortante de Janosi) es 0.63–0.72 (Marte) y 0.52–0.70 (Titán y Ceres) en el deslizamiento, la distancia y el Wh/m, tanto en llano como en subida de 10°;
  φ, c, n y kφ aportan ST ≤ 0.14 cada uno y **kc ≈ 0**. K es justo el parámetro que **ningún cuerpo tiene bien medido** (el de la Tierra no se contrastó; el de la Luna tiene un valor de Apolo distinto del del modelo, ver abajo).
  Los S1 de N = 32 son ruidosos (su suma va de 0 a 1.9; los IC están en el JSON): conviene leer ST.
- **Intervalos** (p05–p95, en el rango supuesto): deslizamiento medio en llano Marte 0.04–0.16 (nominal 0.08), Titán 0.08–0.57 (0.22), Ceres 0.13–0.63 (0.37); en la subida de 10° Marte 0.14–0.59 (0.27), Titán 0.22–0.98 (0.62), Ceres 0.22–0.82 (0.55);
  Wh/m en llano Marte 0.018–0.021, Titán 0.020–0.040, Ceres 0.026–0.072. Con el suelo de Titán y Ceres la distancia en la subida va de «retrocede» (negativa) a lo normal.
- **Pendiente máxima segura** (criterio mío: sin vuelco, deslizamiento medio ≤ 0.5 y velocidad de avance ≥ 0.1 m/s en los últimos 3 s de una subida de 8 s; bisección a ~1° de resolución; N = 24, Tierra 16):

| Cuerpo | nominal (°) | p05 | p50 | p95 | Spearman con K |
|---|---|---|---|---|---|
| Luna | 15.0 | 8.4 | 15.9 | 23.4 | −0.95 |
| Marte | 16.9 | 10.3 | 18.3 | 29.3 | −0.89 |
| Tierra | 12.2 | 7.3 | 13.6 | 18.8 | −0.98 |
| Ceres | 7.5 | 0.0 | 9.4 | 25.8 | −0.93 |
| Titán | 7.5 | 0.9 | 8.9 | 28.3 | −0.90 |

  Con estos supuestos, la cota para planificar de R4 (10°, 5° en Titán) cae dentro del intervalo de Marte y Luna pero **no se puede acotar para Titán y Ceres**: el intervalo va de 0–1° a ~26–28°.
  La lectura «el orden sigue el de φ» de R4 no se sostiene como conclusión de suelo: a φ parecido, K cambia el límite en más de 10°.
- **Gravedad.** Con el suelo de Marte y g de 0.27 a 9.81: el Wh/m en llano **baja** con g (0.0322 → 0.0186; elasticidad log-log −0.14), el deslizamiento baja (0.26 → 0.07, elasticidad −0.38) y el avance en 6 s sube un poco (1.64 → 2.74 m).
  En el Sobol conjunto (g log-uniforme en 0.27–9.81 más el suelo de Marte) g domina la distancia en llano (S1 0.97, ST 0.66) y el Wh/m (ST 0.61) y comparte el deslizamiento con K; en la subida de 10° pesan g y c.
  Como la g de cada cuerpo es conocida, la incertidumbre que importa es la del suelo; pero el signo del efecto de g sobre el Wh/m **contradice** lo esperado físicamente (más peso, más costo por metro) y se explica por el defecto de abajo.

### 3. Contraste con la literatura (acuerdos y desacuerdos, sin ajustar el modelo)
Tabla con URLs y estado de verificación en `validacion.md` (L1–L9). Resumen:
- **Tierra**: los parámetros son los de la arena seca de la Tabla 1 de [NASA/TM-20250006958](https://ntrs.nasa.gov/api/citations/20250006958/downloads/TM-20250006958.pdf) (acuerdo por construcción; K = 0.025 m sigue sin fuente).
- **Luna**: el [Apollo 17 Preliminary Science Report, sec. 8](https://www.nasa.gov/wp-content/uploads/static/history/alsj/WOTM//a17psr_Section8_SoilMechanics.pdf) da el «modelo B» del LRV (cuyo consumo de energía medido coincide con la predicción): φ 35°, c 0.17 kPa, kφ 0.81 N/cm³, n 1.0
  **coinciden con `planetas.py`**; pero **kc = 0.35 N/cm² (3.5 kPa; el modelo usa 1.4, o sea 0.4×) y K = 1.0 cm (el modelo usa 1.8 cm, 1.8×)**. Esto corrige a R4 («K supuesto, parámetros lunares sin verificar»): hay un K publicado para Apolo. Con ese K y kc el modelo agarra más:
  deslizamiento a 20° de 0.40 contra 0.69 (`luna_apollo17` en `validacion.md`). No se cambió `planetas.py`; queda para R6.
- **Marte**: φ 33° y c 0.8 kPa caen en los rangos de [Sullivan et al. 2011](https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2010JE003625) (φ 30–37°, c 0–2 kPa en zanjas de rueda; 0–11 kPa en rascaduras; solo el resumen, la página dio 403). kc, kφ, n y K no tienen contraste.
- **Titán y Ceres**: no hay terramecánica medida; **no contrastable**.
- **Deslizamiento contra pendiente** (modelo, Marte: 0.06 a 0°, 0.25 a 10°, 0.59 a 20°, ~1 a 30°; Tierra: 0.37 a 10°, 0.75 a 20°). *Acuerdo cualitativo*: el deslizamiento crece con la inclinación (en Curiosity la inclinación es el control principal, [Arvidson et al. 2014](https://agupubs.onlinelibrary.wiley.com/doi/full/10.1002/2013JE004605), cifra del buscador, no verificada);
  Opportunity quedó semanas atrapada en arena de 17–23° ([Maimone et al. 2007](https://www-robotics.jpl.nasa.gov/media/documents/rob-06-0081.R4.pdf), leído), y el modelo avanza ~0.15 m/s de 0.5 mandados a 20° en Marte y 0 en la Tierra.
  *Desacuerdo / no comparable*: «menos de 20 % de deslizamiento hasta 30°» de [Opportunity en 2016](https://www.jpl.nasa.gov/news/opportunity-mars-rover-goes-six-wheeling-up-a-ridge/) fue sobre afloramiento rocoso, no suelo suelto; el modelo en 30° de suelo suelto patina ~100 %. La cifra «91 % a 20° en suelo blando» (Lindemann y Voorhees 2005)
  **no se pudo verificar** (resumen de buscador); si fuera cierta, el modelo (0.59 a 20° en Marte) patina menos que el rover de pruebas, sin poder decir cuánto por las diferencias de rueda y suelo.
  Curiosity evita pendientes arenosas de más de 12.5° (norma operativa, no límite físico; no verificada): el modelo da 17° nominal en Marte (intervalo 10–29°), más optimista, coherente con que una norma operativa deja margen.
- **Resistencia al movimiento**: **no hay un coeficiente citable** que haya podido verificar (Wong y Bekker son libros de pago). El modelo da Rc/(m g) = 0.05 (Luna), 0.066 (Marte), 0.081 (Tierra); no se compara con nada.
- La URL del KISS/Caltech que citó R4 para «Predicting the performances of rigid rover wheels» hoy devuelve 404.

### 4. Invariancias (todas pasan salvo la marcada)
- **Carga estática**: Σ Fn = m g con error relativo ≤ 1·10⁻¹³ en los cinco cuerpos.
- **Similitud dinámica de g** (g × 4 y × 0.25 con k, c, τmax, kw, kc, kφ, c, velocidades y tiempo reescalados): trayectoria idéntica con error de posición ≤ 9·10⁻⁴ relativo, de rumbo ≤ 8·10⁻⁴° y de deslizamiento ≤ 5·10⁻⁵;
  la energía de tracción escala ×4.27 y ×0.25 (el término viscoso fijo 0.02·ω de la rueda y las pérdidas Joule no escalan). El modelo es consistente en su dependencia con g.
- **Simetría izquierda-derecha**: terreno reflejado + orden (v, −w) da la trayectoria reflejada con error 0.0 (llano, rocoso y dunas); recto en llano, |y| = 0.
- **Energía sin tracción** (φ = 0, c = 0, sin amortiguador ni motores, cuesta de 10°): deriva de 1·10⁻⁶ J sobre 773 J liberados (1·10⁻⁹); con el amortiguador real la energía no sube (mayor subida por paso 3·10⁻¹³ J);
  con suelo real y motores apagados, rodando por inercia, la energía total no sube (el suelo es pasivo).
- **Falla (reportada a R6, prueba `xfail`)**: *la resistencia por compactación no se cobra en la energía eléctrica.* En régimen (llano, 0.5 m/s) el par motor es 0.04 N·m y la potencia de tracción del modelo ~0.7 W en Luna, Marte y Tierra, contra una mínima Rc·v/η de 8.2, 24.7 y 77.4 W (razón 0.08, 0.03, 0.01).
  Causa (lectura de `rover.py`): `fx` incluye `−Rc·tanh(vx/V_ROD)` y el mismo `fx` entra en el par de la rueda (`−fx·Rw`), de modo que Rc frena el chasis y acelera la rueda en la misma cantidad; en régimen `fx = 0`.
  Una rueda rígida con presión normal radial no recibe par de Rc. Corolario medible: con motores apagados el rover desacelera 0.012 m/s² cuando Rc/m sería ~0.24 m/s² en Marte. Consecuencias: el Wh/m «de tracción» de R4 no incluye el costo de rodar
  (queda arranque, giro y deslizamiento), y su CdT y la lectura «la gravedad explica poco» (CD-2026-30) **no deben citarse** hasta corregirlo; el sentido de la elasticidad de Wh/m con g tampoco.
  Corrección sugerida: usar `Rw·T(s)` (sin el término Rc) en la dinámica de la rueda. Se verifica con `test_potencia_de_traccion_cubre_la_compactacion` (hoy `xfail`).
- **Defecto latente**: `Rover._acumular` muestrea cada 10 pasos y el temporizador de atasco suma 0.1 s por muestra, así que `metricas()` solo es correcta con DT = 0.01; el estudio calcula sus propias métricas.

### Sin probar
Otras masas, ruedas o velocidades; ruido de sensores; sensibilidad con la navegación completa (todo es lazo abierto); Sobol con N > 32 (los S1 son ruidosos); rangos de incertidumbre contrastados con datos; las cifras no verificadas de la literatura (L3, L6, L7, L8);
corregir el defecto de la compactación y repetir P1 de R4; estabilidad del paso con dt > 0.02 s.

## Resultados R8 (operación con retardo de enlace)

Fase 3 (CONTRATO §19 «Operación con retardo»). Módulo `src/atp_rover/atp_rover/operacion.py` (Python puro, determinista, sin ROS), envoltorio `scripts/operacion_rover.py`, escenas
`experiments/rover-operacion-{marte,luna}.yaml` (`kind: proceso`, `area: rf`), estudio `scripts/estudio_operacion_rover.py` y resultados en `docs/rover_resultados/operacion.{json,md}`
(todas las tablas están ahí) con la calibración en `operacion_calibracion.json`. Pruebas: `src/atp_rover/test/test_operacion.py` (16, ~30 s). No se tocó `rover.py`, `lazo.py`, `navegacion.py` ni `comunicacion.py`;
se reutilizan `cli.SeguidorBasico`, `Terreno`, `Enlace` y `CUERPOS`. Entrada de innovación: CD-2026-36.

### Qué es y qué no es (leer antes de citar una cifra)
La dinámica de R1 corre a ~130 pasos/s: ~11 min de pared por 90 s simulados. Un sol (88 775 s) es inalcanzable, así que **el rover del lazo operativo es un sustituto cinemático** (monociclo, tic de 1 s):
- La física de R1 (con `rodadura_en_motor: true`, navegación `basico`) se usa en **micro-corridas de 50 s** por (cuerpo, terreno, semilla) que dan la razón de velocidad efectiva, la potencia de marcha y el Wh/m
  (30 corridas, ~6 min con 6 procesos; `operacion_calibracion.json`). Ejemplos: Marte llano 0.87 de v_max, 70 W, 0.045 Wh/m; Luna llano 0.92, 38 W, 0.023 Wh/m; pendiente de 15° 0.46–0.50 (Marte 147 W).
- Sigue las metas con el `SeguidorBasico` real de `cli.py`. No tiene suspensión, deslizamiento ni vuelco físicos.
- La navegación `completa` de R6 **no se usa** en los soles (muy lenta). Extra, solo calibración (Marte, semilla 0, 4 terrenos): da v/v_max 0.96–1.00 contra 0.76–0.87 de `basico` en velocidad de crucero (sin atascos en 50 s), pero en `rocas` avanza
  menos (8 m en 40 s contra ~17 m); no se comparó en metas alcanzadas, que es donde R6 la encontró peor.
- **Peligros por regla** (supuestos propios, sin fuente): una celda con pendiente ≥ 25° del campo de pendientes del `Terreno` de R1 (teselado con desplazamiento aleatorio cada 100 m) es un obstáculo; el rover con alarmas locales lo ve a 1.5 m, se
  detiene, rodea (4 m al lado libre, 7 m) y sigue; sin alarmas entra y se atasca (con 35° o más, vuelco con p = 0.5, y el vuelco acaba la misión). Además hay zonas blandas ocultas (2 por km, 3 m): atasco que se detecta tras 3 s sin avanzar. Recuperarse (vaivén) tarda
  90 s y funciona con p = 0.6 (3 intentos locales; luego el rover queda en modo seguro esperando a Tierra).
- **Enlace**: el de R7 (retardo de luz de la fecha, ventanas del relé MRO en Gale, entrega de órdenes y DTN de telemetría de `Enlace`) o uno sintético (`owl_s` constante, ventanas de 10 min cada `intervalo_ventanas_s`). Pérdidas de 2 % de órdenes y de telemetría.
  Los supuestos del enlace son los de R7 (potencia del rover, ganancias, turno DSN, órbitas keplerianas, retardo de órbitas circulares). Luna: retardo constante de 1.28 s (2.6 s ida y vuelta), sin ventanas.
- Operación: rover de 0.5 m/s con 2 h de conducción por sol (un rover real recorre mucho menos), operador disponible las 24 h con 10 s de decisión, 3 soles por corrida, 3 semillas.

### Los cuatro modos
(a) **teleop**: «mover y esperar»: segmentos de 10 m, no manda el siguiente hasta que la telemetría confirma la llegada; sin alarmas locales. (b) **metas**: en cada telemetría manda metas hasta 100 m por delante de la última posición recibida; sin alarmas.
(c) **plan de sol**: compromete la jornada entera (~3.4 km) y el rover lleva alarmas locales (pendiente visible y atasco). (d) **gemelo**: como (c), más predicción (estima dónde estará el rover cuando llegue la orden, con un error de velocidad de ±15 %, y compromete solo
lo que el enlace obliga) y un MDE que conoce el 60 % de los obstáculos y los rodea de antemano. Se pueden separar los efectos con `seguridad_local`, `gemelo_prediccion`, `p_dem` y `adelanto_m`.

### Resultados (simulación, estos supuestos; media ± IC 95 % de Student con 3 semillas: los intervalos son anchos)
**Distancia útil (m/sol)**, tabla completa por terreno en `operacion.md`:

| Cuerpo y terreno | (a) teleop | (b) metas | (c) plan de sol | (d) gemelo |
|---|---|---|---|---|
| Luna, llano (2.6 s ida y vuelta) | 1934 ± 341 | 2791 ± 272 | 2834 ± 234 | 2834 ± 234 |
| Luna, rocoso | 1841 ± 195 | 2558 ± 257 | 2712 ± 249 | 2711 ± 237 |
| Luna, rocas | 148 ± 218 | 157 ± 255 | 1338 ± 984 | 1332 ± 1338 |
| Marte, llano (ventanas MRO, ~13 h de latencia mediana de una orden) | 9 ± 5 | 80 ± 88 | 2040 ± 1949 | 2205 ± 1262 |
| Marte, rocoso | 9 ± 5 | 75 ± 79 | 1953 ± 1857 | 2122 ± 1166 |
| Marte, rocas | 9 ± 5 | 53 ± 65 | 309 ± 611 | 361 ± 257 |

- **Hasta qué retardo se puede teleoperar** (Marte, retardo constante, sin ventanas, E2): la distancia útil cae a la mitad de la del enlace ideal con **17 s** de luz de una vía en llano (rango por semilla 16–17 s) y **18 s** en rocoso (17–18 s), es decir ~35 s de ida y vuelta;
  frente a la autonomía (c) con el mismo retardo, baja del 25 % a los **34 s** y del 10 % a **~120 s**. Con 150 s rinde 200 m/sol, con 900 s 38 y con 1440 s 29. «Metas» (b) cae a la mitad con **~200 s** (rango 141–232 s) por semilla. Con las ventanas reales del MRO la teleoperación llega a 9 m/sol y el
  extra de retardo de luz real de Marte (693 s) sin ventanas da 54 ± 8. Estos umbrales dependen de los 10 m por segmento y los 10 s de decisión que se supusieron.
- **Intervalo entre ventanas** (ventanas de 10 min, rocoso, E3): con 1.3 s de luz la teleoperación baja del 50 % de su valor ideal con ventanas cada **19 min o más** (en la rejilla de 15 a 960 min); con 60 s de luz o más ya está por debajo en el primer punto (15 min). (b) metas: 26 min con 1.3 s, 20 min con 60 s.
  (c) y (d) no bajan del 50 % en toda la rejilla (hasta 16 h entre ventanas con 900 s de luz) porque la primera jornada va precargada y las siguientes tienen margen; con 16 h entre ventanas ya aparece una semilla que pierde un sol (2284 ± 1384).
- **Cuánto vale la autonomía**: en la Luna (2.6 s) la teleoperación rinde 68 % del plan de sol en llano (1934 contra 2834); en Marte con ventanas reales rinde ~0.4 % (9 contra 2040). Lo que más pesa **no** es la decisión autónoma de avance sino las **alarmas locales**: sin ellas el mismo plan de sol baja
  en Marte-llano de 2040 a 312 m/sol y en Luna-rocas de 1338 a 157 (3 vuelcos en 3 semillas contra 1); comprometer más distancia sin alarmas casi no ayuda (Marte-llano, metas de 100 a 3400 m: 80 → 312 sin alarmas, 89 → 2026 con ellas, E7).
- **Tiempo hasta reaccionar a un peligro** (desde que el rover entra o lo detecta hasta que empieza la maniobra, E1): con alarma local, 0 s ante un obstáculo visto y 3 s ante un atasco (el umbral de detección); vía Tierra, 13–14 s en la Luna con 2.6 s de ida y vuelta (media por corrida 270–374 s por colas) y del orden de 18–32 h en Marte (4 a 9 eventos). El modo (b) en Marte termina con un peligro sin respuesta en 6 de 15 corridas.
- **Gemelo digital**: **sin ganancia medible en el plan de sol**: (d) 2205 ± 1262 contra (c) 2040 ± 1949 en Marte-llano (dentro del IC); el MDE (E4) no ayudó con rocas (Marte-rocas 309 ± 611 contra 361 ± 257; `p_dem` 0.3 dio 911 ± 2231, 0.9 dio 423 ± 722: ruido, no tendencia). Donde sí mueve la cifra es la
  **predicción en teleoperación continua con retardo**: a 150 s de luz «mover y esperar» 201 ± 12 m/sol contra 1836 ± 575 del gemelo sin alarmas y 2572 ± 212 con ellas (a 15 s: 1004 → 2420 → 2572; a 450 s: 73 → 1238 ± 1053 → 2567). Esto mide el valor de enviar órdenes por adelantado con un modelo del rover **correcto
  salvo por 15 % de velocidad y por los peligros ocultos**: es una cota superior, no una medida de un gemelo real.
- **Energía**: el consumo fijo se paga esperando. Wh por metro útil (2 h de conducción): teleoperación en Marte 4.9, (c) 0.13, (d) 0.11; en la Luna 0.05 para los cuatro.
- **Sensibilidad (Luna, llano, E6)**: con 0 zonas blandas por km (a) 2226 y (c) 3328; con 6 por km (a) 1651 y (c) 2113. Con 20 % de pérdida de órdenes y telemetría (a) baja a 1449 y (c) queda en 2833.

### Cómo reproducir
```bash
python3 -m pytest -q src/atp_rover/test/test_operacion.py                       # 16 pruebas, ~30 s
python3 scripts/estudio_operacion_rover.py --rapido --trabajadores 4            # ~35 s; escribe en runs/rover_operacion_rapido/
python3 scripts/estudio_operacion_rover.py --trabajadores 6                     # ~7 min; docs/rover_resultados/operacion.{json,md}
python3 scripts/operacion_rover.py --escena experiments/rover-operacion-marte.yaml --salida /tmp/rover --id o1
```

### Sin probar / límites
- **Nada se comparó con una operación real** ni con telemetría de un rover. Las magnitudes absolutas (km por sol) son de un rover de 0.5 m/s que conduce 2 h, no de uno real; solo las comparaciones relativas entre modos con los mismos supuestos tienen sentido, y con 3 semillas varias no son significativas (IC solapados).
- El rover sustituto **no reproduce la física**: no se verificó que un obstáculo de pendiente ≥ 25° realmente atasque a la dinámica de R1, ni que las zonas blandas existan con esa frecuencia. La calibración dura 50 s por celda y no ve terreno variado a escala de sol. La navegación `completa` no se evaluó en los soles.
- No hay nodo ROS (`rover_operador`): el módulo es Python puro y no se integró con `rover_enlace`, `rover_dinamica` ni `rover_navegador`.
- Sin turnos del operador, sin errores humanos, sin imágenes (el «ver 10 m» es un supuesto), sin planificación por energía o térmica, sin contención del relé, sin reintentos de DTN más allá de los de R7.
- E3 usa ventanas sintéticas periódicas de 10 min; las reales de Marte son irregulares (R7). En E1 el retardo de Marte (693 s) es el del modelo de R7, que se equivoca hasta ~0.15 UA.

## Resultados R9 (lockstep ROS)

Equipo R9 (investigación e ingeniería). Hasta R8 los nodos `rover_dinamica`, `rover_navegador` y `rover_enlace` solo habían pasado compilación y una prueba manual de un mensaje: **la cadena
no se había probado en lockstep ni se sabía si daba lo mismo que el lazo sin ROS**. Módulos: `src/atp_rover/atp_rover/lockstep.py` (Python puro: planta, navegador, operadores, enlace y `correr_cadena`, el equivalente sin ROS),
`nodos_lockstep.py` (tubería DDS), `launch/rover_lockstep.launch.py`, envoltorio `scripts/rover_lockstep.py`, escenas `experiments/rover-lockstep-{luna,marte}.yaml`, pruebas `src/atp_rover/test/test_lockstep.py` (13, ~1 min, sin ROS)
y `scripts/validar.sh rover` (la cadena con ROS en el contenedor). No se tocaron `rover.py`, `terreno.py`, `navegacion.py` ni `seguridad.py`. Semántica fijada: CONTRATO §18 «Lockstep ROS del rover». Entrada de innovación: CD-2026-43.

### Qué se fijó (y por qué hubo que cambiar los nodos)
- **Qué instante rige cada mando.** El modo antiguo (ahora `pasivo`) aplicaba «el último Twist recibido» al avanzar con cada `/clock`: según el orden de llegada en el DDS, el mando calculado de e_k regía [t_k, t_{k+1}) o [t_{k+1}, t_{k+2}), y no confirmaba `/atp/step_ack`, así que no era reproducible. En `lockstep` el mando calculado del estado e_k (sello t_k) rige **[t_k, t_{k+1}) completo** (retención de orden cero, 10 pasos de RK4 de 0.01 s con el mismo (v, w)): es exactamente lo que hace `lazo.correr`.
- **Un Twist no lleva sello**, así que se añadió una ficha por tópico de órdenes (`/rover/cmd_vel_listo`, `/rover/cmd_vel_op_listo`, `/rover/estado_tierra_listo`: `{"t", "n", "info"}`) y el consumidor espera la ficha y sus `n` mensajes. Los tópicos de §18-19 no cambian de tipo.
- **Con enlace**, el tiempo de la cadena es el de escena (el `t` del estado), no el `/clock` vigente al llegar el mensaje (que era el sello del `rover_enlace` pasivo). Una orden mandada en t_k entra al enlace en el tick k+1 con su sello t_k y se aplica en el primer tick t ≥ t_k + retardo: cuantización < 0.1 s y entrega mínima un paso después (en modo `ideal` también).
- Arranque (§8, regla 4): la dinámica espera suscriptores, atiende solo el `/clock` de k = 1 (el que deja pasar la compuerta de `sim_clock`) como instante 0 de la escena y lo repite hasta la primera ficha; navegador y enlace descartan sellos repetidos (un navegador con estado interno que respondiera dos veces al mismo estado ya no sería el del lazo). `sim_clock` se lanza con `ack_timeout_s: 3600`: con el defecto de 10 s de pared, un tick lento (el primer cálculo del horario del enlace de Marte) dejaría avanzar el reloj sin confirmar.

### Resultados (simulación, mismo contenedor y misma CPU para las dos corridas)
**Con y sin ROS: mismos bytes** de `telemetry.jsonl`, `metrics.json` y `mandos.jsonl` en todos los casos probados; no hubo que normalizar el cero negativo ni explicar ninguna diferencia (se compara con huella normalizada, como `_huella` de `test_rover_bogie.py`, y también byte a byte crudo).

| Escena (cadena en ROS vs `correr_cadena` sin ROS) | Duración | Filas | Resultado |
|---|---|---|---|
| Luna rocoso (semilla 7), 2 metas, navegación `basico`, `time_scale` 0 y 2 | 12 s (y 20 s en la prueba inicial) | 121 | idéntico; `time_scale` 0 y 2 idénticos entre sí |
| Luna rocoso, navegación `completa` (A*), `time_scale` 0 y 2 | 12 s | 121 | idéntico; ídem |
| Marte dunas (semilla 3), `basico` | 12 s | 121 | idéntico |
| Luna, enlace `retardo`, operador con guion (órdenes en t = 1.0 y 4.0 s) | 8 s | 81 | idéntico; las órdenes se aplican en t = 2.3 y 5.3 s (retardo de luz 1.28 s, cuantizado a 0.1 s) |
| Luna, enlace `retardo`, **navegador en Tierra** (ve el estado con 1.28 s de retraso) | 10 s | 101 | idéntico; la primera orden llega en t = 2.6 s (2 × 1.28 s) |
| Marte, enlace `ventanas` (MRO en Gale, 4.6 S 137.4 E), planta **cinemática**, orden en t = 1.0 s con `t0_s` = 117599 s | 2760 s | 27 601 | idéntico; la orden llega en t = **2701.1 s** (45 min: espera los contactos del relé; el retardo de luz solo son ~11 min) |

Con `completa` sin ROS, la cadena también iguala `lazo.correr` (pruebas: 3 escenas más la parada por meta alcanzada y una duración que no es múltiplo de 0.1 s; sin ruta).

- **Lo que habría cambiado con otra semántica** (emulado sin ROS, 20 s: el mando de e_k aplicado un paso después, que era lo que hacía el nodo pasivo cuando el `/clock` llegaba antes que el Twist): error máximo de posición **0.08 m** (`basico`, Luna rocoso), **0.16 m** (`completa`, Luna rocoso) y 0.03 m (`basico`, Marte dunas); de rumbo hasta 2.5°. Es una cota del orden de la diferencia, no una medición del nodo antiguo (que no era reproducible).
- **Retardo en tiempo simulado, no de pared**: se mide en tiempo de escena; con `time_scale` 0 la corrida de Luna de 8 s dura ~11 s de pared y la de 2760 s de Marte ~190 s (≈ 150 ticks/s con el host cargado, ~7 ms por tick, de los cuales casi todo es DDS entre cuatro procesos). La dinámica de R1 cuesta casi 1 s de pared por 1 s simulado, así que `time_scale` 2 **no frena nada** en estas escenas (el cálculo ya es más lento que el reloj): comprueba que el ritmo no cambia los bytes, no que el reloj de pared se respete.
- **La telemetría vuelve** por `/rover/estado_tierra` con `t_tierra` = `t` + retardo (Luna: `t_tierra − t` = 1.28 s, primer tick ≥ t + owl; probado con `EnlaceLock`). En `ventanas` de Marte la telemetría **no vuelve** en 46 min: el modelo de R7 la entrega a las ≥ 3.4 h (barrido cada 150 s durante un sol: mínimo 12 389 s), la cola de 5000 desborda (22 600 estados descartados) y eso se cuenta en `metrics.json`.

### Qué no cuadró / hallazgos (sin tocar tolerancias)
1. **`t0_s` no puede ser un desplazamiento de `jd0`.** Mi primer intento colocó la escena dentro del calendario del enlace moviendo `jd0`; el horario de contactos del relé y el giro del cuerpo de R7 se calculan desde t = 0 de `Enlace`, así que solo cambiaba el retardo de luz y la orden nunca se entregaba. Ahora `t0_s` desplaza el tiempo del filtro (T = t + t0) y `fecha` fija el retardo.
2. **En el modelo de R7 una orden por `ventanas` tarda 45 min como mínimo** (mejor caso de un barrido de un sol: 2700 s; el caso típico, horas): no se puede probar con la dinámica de R1 (≈ 1 s de pared por s simulado). Para ejercitar el protocolo se añadió `dinamica: cinematica` (monociclo ideal, **no es la física de R1**; la prueba mide el protocolo y el enlace, no el rover).
3. Hay un **paso de latencia mínima** también con enlace `ideal` y la entrega se cuantiza al paso de 0.1 s: es consecuencia de dar a cada tick una sola fase de enlace (si no habría un bucle estado → operador → orden → rover dentro del mismo instante).
4. `time_scale:=2` solo se probó en escenas donde el cálculo es más lento que el reloj (ver arriba).

### Cómo reproducir
```bash
python3 -m pytest -q src/atp_rover/test/test_lockstep.py                        # ~1 min, sin ROS
./run.sh bash scripts/validar.sh rover                                          # la cadena con ROS, 6 escenas y 8 corridas, ~10 min; imprime DETERMINISTA (rover)
ros2 launch atp_rover rover_lockstep.launch.py time_scale:=0 salida:=/tmp/rover id:=l1 \
    cfg:='{"planeta":"luna","terreno":"rocoso","semilla":7,"ruta":[[4,0],[4,3]],"duracion_s":20,"navegacion":"completa"}'
python3 scripts/rover_lockstep.py --escena_yaml experiments/rover-lockstep-luna.yaml --enlace retardo --salida /tmp/rl --id a   # ROS + sin ROS + equivalencia.json
```

### Sin probar / límites
- **No se probó** el rover del lockstep con Gazebo (R3) ni el visor; el modo `pasivo` de los tres nodos solo se comprobó que arranca y publica (`rover_dinamica` a reloj de pared, `rover_enlace` a 1 Hz), no su comportamiento con un controlador en el lazo (no es reproducible).
- Un solo host, una sola CPU y un contenedor: no se sabe si otra CPU (SIMD distinto) o DDS entre máquinas cambia los bytes más allá del cero negativo; las pruebas se hicieron con el host a carga 7-10, no bajo una carga extrema. Ejecutores de `rclpy` de un solo hilo.
- `time_scale:=1` (tiempo real) no se corrió; `lockstep:=false` (reloj libre) no aplica a esta cadena.
- Navegador en Tierra ingenuo (sin compensar retardo ni predecir) y sin `completa` a través del enlace (solo `basico` en ROS); en `ventanas` no se probó el navegador en Tierra (la telemetría tarda horas). Con pérdidas (`p_perdida_*` > 0) solo hay prueba sin ROS (pérdida total de órdenes), no con DDS.
- Las ventanas, el retardo y la potencia del enlace son los supuestos de R7, sin validar contra telemetría real; la planta cinemática no es el rover.
## Resultados R10 (validación fuera de muestra de la navegación)

Equipo R10 (investigación). Archivos: `src/atp_rover/atp_rover/validacion_nav.py` (casos, progreso, atasco real, bootstrap; **el pre-registro está en su docstring y se commiteó antes de correr un solo caso nuevo**), `scripts/estudio_navegacion_r10.py`, pruebas `src/atp_rover/test/test_validacion_nav.py` (7, < 1 s), resultados en `docs/rover_resultados_r10/` (`estudio_{validacion,ajuste,ajuste2,tercero}.{json,md}` y `corridas.jsonl`, 1300 corridas en caché). Todo es **en simulación, con los suelos supuestos de R4, la física corregida (`rodadura_en_motor: true`, defecto) y un rover de 150 kg de suspensión `simple`**; sin ruido de sensores.

### Diseño (fijado antes de mirar números)
- **Conjuntos disjuntos.** *Ajuste*: los 60 casos de R4/R6 sin cráter (semillas 1-3, ruta fija) más un *ajuste ampliado* (`ajuste2`: semillas 301-330, 50 casos del mismo generador que la validación). *Validación*: semillas 101-130, 150 casos = 5 cuerpos × 5 regímenes (`rocoso`, `dunas`, `rocas`, `escalones` de 6 y 10 cm, pendiente `lateral` de 8° y 12° con `direccion_deg` 90) × 6 casos; 4 rutas rotando (zigzag de 32 m con 3 giros, diagonal de 19.8 m, recta de 18 m que cruza las dos gradas en `escalones`, ida y vuelta de 21.5 m); tope de tiempo 3.5× el ideal + 20 s. *Tercero*: semillas 201-230, mismo diseño, usado **una sola vez**. Comparaciones por pares (mismo caso con ambas navegaciones); IC 95 % por bootstrap percentil (10 000 remuestreos, semilla 20261008).
- **Atasco real.** El contador del rover (episodios de 5 s con |v| < 20 % de la orden) cuenta cada pausa del vaivén de R6. Aquí un atasco real es *sin progreso*: 30 s sin que el progreso máximo a lo largo de la ruta (metas alcanzadas en orden a 1 m + proyección sobre el tramo actual) suba 0.25 m. 30 s es 6 veces el episodio de 5 s y más que 2 ciclos de vaivén (11 s); un vaivén que libera sube el progreso y no cuenta. Se reportan corridas con ≥ 1 episodio, episodios y atasco terminal (la corrida acaba sin llegar con un episodio abierto). Límite: es independiente del controlador, pero un rover muy lento (menos de 0.25 m en 30 s) también cuenta.
- **Criterios de `completa` contra `basico`** (C1 llegada: límite inferior del IC de la diferencia ≥ −0.05; C2 vuelcos totales ≤ y límite superior de la diferencia de tasa ≤ +0.02; C3 atasco real: límite superior ≤ +0.05; C4 energía por metro útil ≤ ×1.10; C5 tiempo ≤ ×1.15; C6 distancia/recta ≤ ×1.10, los tres últimos en pares donde ambas llegan; C7 ningún terreno ni cuerpo con `completa` significativamente peor). No se cambió ninguno después de ver números.

### 1. Validación: `completa` de R6 no generaliza (150 casos nuevos, semillas 101-130)
| | básica | completa (R6) |
|---|---|---|
| Metas | 141/150 (94.0 %) | 103/150 (68.7 %) |
| Vuelcos | 0 | 1 (Ceres, `rocas`) |
| Abortadas por seguridad / inalcanzable | 0 / 0 | 30 / 8 |
| Atascos nativos (contador del rover) | 43 | 64 |
| Corridas con atasco real (≥ 30 s sin progreso) / terminal | 39 / 5 | 34 / 11 |

- Llegada, completa − básica: **−25.3 puntos, IC95 [−32.7, −18.0]**; solo la completa llega en 2 casos, solo la básica en 40 (prueba exacta del signo p = 4·10⁻¹⁰, d_z = −0.54). Por terreno: `rocas` −83.3 [−96.7, −70.0] (13 % contra 97 %), `dunas` −23.3 [−40.0, −10.0], `escalones` −13.3 [−30.0, +3.3], `lateral` −6.7 [−16.7, 0.0], `rocoso` 0; **peor en los cinco cuerpos** (Ceres −36.7, Titán −26.7, Luna −23.3, Tierra −23.3, Marte −16.7) y en las cuatro rutas (−23.7 a −27.0).
- **Criterios**: C1 NO, C2 NO (1 vuelco contra 0; el IC de la diferencia de tasa, [0, +0.020], roza el límite), C3 **SÍ** (−3.3 puntos [−11.3, +4.7]: con la métrica de atasco real la completa no tiene más corridas atascadas que la básica), C4 NO (×1.100 [1.070, 1.133], n = 101), C5 SÍ (×0.903 [0.877, 0.930]: llega antes cuando llega), C6 SÍ (×1.032), C7 NO (dunas, rocas y los cinco cuerpos).
- **Lectura sobre el «atasco».** El contador nativo da 64 contra 43 (+49 %), pero las corridas con atasco real son 34 contra 39: la mayor parte de los «atascos» extra de R6 eran pausas del vaivén, como R6 sospechaba. Lo que sí es peor es que 47 corridas de la completa no llegan (contra 9 de la básica).
- **Mecanismo dominante** (trazas de la validación): de esas 47, 30 son abortos de `seguridad` por la alarma `pendiente` (> 25° en el punto bajo el CM; 24 en `rocas`, 5 en `escalones`, 1 mixta), 8 son `inalcanzable` (vaivén agotado, en `dunas`) y 9 se acaban por tiempo o vuelco. En `rocas` y `escalones` los bordes verticales dan > 25° en la normal puntual del heightmap, la seguridad ordena parar, el rover queda parado sobre el borde, la alarma sigue activa y a los 10 s se aborta. La básica no tiene seguridad y pasa. Es una propiedad de `seguridad.py` (ya presente en `completa_r4`), no de un componente que R6 haya añadido.
- Esto **no contradice** a R6 en su propio conjunto sino que lo completa: con la física corregida R4b ya mostraba 45/60 contra 52/60 (la cifra «50/60» de R6 era la de la física antigua, sobre un conjunto en el que se iteró 4 veces).

### 2. Diagnóstico por ablaciones (solo en el ajuste; nunca en la validación)
`ajuste2` (50 casos, semillas 301-330), cada fila apaga **un** componente de `completa` (llegada básica 96.0 %, completa 74.0 %); IC95 de la diferencia con la básica, en puntos:

| Variante | Llegada | dif. vs básica [IC95] |
|---|---|---|
| completa (R6) | 74 % | −22.0 [−36.0, −10.0] |
| sin costo graduado | 74 % | −22.0 [−36.0, −10.0] |
| sin prealimentación + PI | 76 % | −20.0 [−32.0, −10.0] |
| sin guarda anti-encabritado | 72 % | −24.0 [−38.0, −12.0] |
| sin vaivén | 78 % | −18.0 [−30.0, −6.0] |
| sin límite de aceleración | 80 % | −16.0 [−28.0, −6.0] |
| sin replanificación | 74 % | −22.0 [−34.0, −10.0] |
| umbral de antideslizamiento 0.5 | 72 % | −24.0 [−36.0, −12.0] |
| frenar en pendiente (como R4) | 78 % | −18.0 [−30.0, −6.0] |
| **sin la alarma de pendiente** (añadida *después* de ver la validación) | **88 %** | **−8.0 [−18.0, 0.0]** |

Con 50 casos las diferencias de 2-3 casos entre filas son ruido (IC solapados); lo único claro es la alarma de pendiente (+14 puntos). En el conjunto de R4 (60 casos), donde esa alarma casi no actúa, los pasos siguientes dieron: huella sola 45/60 (igual que la completa), huella + sin límite de aceleración 46/60, **huella + sin límite de aceleración + sin vaivén 54/60** (básica 52/60): ahí el daño es del vaivén (la completa tiene 12 corridas `inalcanzable` tras 5 maniobras y 14 atascos reales terminales, contra 0 y 3 de la variante sin vaivén). En `ajuste2` la misma combinación dio 48/50 (la básica también 48/50).

### 3. `completa_r10` y evaluación en el tercer conjunto (semillas 201-230, una sola vez)
`completa_r10` = `completa` de R6 con tres cambios decididos en el ajuste: (1) alarma de pendiente sobre la **huella** (normal media de una rejilla 3×3 de paso 0.6 m; parámetro `pendiente_huella_m`, 0 = como antes), (2) sin límite de aceleración de la orden (`acc_max` 0) y (3) sin vaivén. `completa` y `completa_r4` no cambian bytes (las pruebas de R2, R4 y R6 siguen pasando; el parámetro nuevo está apagado por defecto). Los tres cambios se escogieron entre pocas combinaciones mirando 110 casos de ajuste; un conjunto de ajuste así de pequeño puede sobreajustarse como el de R6, por eso existe el tercero.

| | básica | completa (R6) | completa_r10 |
|---|---|---|---|
| Metas (150 casos nuevos) | 141 (94.0 %) | 105 (70.0 %) | 140 (93.3 %) |
| Vuelcos | 0 | 0 | 1 (Ceres, `rocas`) |
| Abortadas por seguridad / inalcanzable | 0 / 0 | 31 / 6 | 0 / 0 |
| Corridas con atasco real / terminal | 34 / 5 | 33 / 10 | 36 / 5 |

- `completa_r10` − básica en llegada: **−0.7 puntos, IC95 [−5.3, +3.3]** (solo r10 llega en 5, solo la básica en 6; p = 1). `completa_r10` − `completa` (R6): **+23.3 [+16.7, +30.0]** (35 casos a favor, 0 en contra). Contra la básica, por terreno: `escalones` **+13.3 [+3.3, +26.7]** (4 a favor, 0 en contra; sin corregir por 10 comparaciones), `dunas` −6.7 [−20.0, +6.7], `rocas` −6.7 [−16.7, 0.0], `rocoso` −3.3 [−10.0, 0.0], `lateral` 0; por cuerpo de −6.7 a +6.7 (todos los IC contienen el 0 o lo tocan).
- **Criterios de `completa_r10` frente a básica en el tercero**: C1 **NO por 0.3 puntos** (límite inferior −0.053 contra −0.05; el criterio no se movió), C2 NO (1 vuelco contra 0; límite superior +0.020), C3 NO (+1.3 puntos [−6.0, +8.7]), C4 **NO** (energía por metro útil ×1.154 [1.120, 1.191], n = 135: sale más cara, por empujar más fuerte), C5 SÍ (×0.924), C6 SÍ (×1.014), C7 SÍ (ningún régimen significativamente peor). Cumple C5, C6 y C7. La `completa` de R6 en el mismo conjunto cumple C2, C5 y C6 y falla C1 (−24.0 [−31.3, −16.7]), C3, C4 y C7. El número de criterios cumplidos no resume bien nada: la diferencia está en la llegada.
- **Conclusión acotada**: en simulación y con estos supuestos, `completa_r10` recupera casi toda la llegada de la básica en casos nuevos (−0.7 puntos con un IC de ±4), pero **no la supera ni demuestra una ventaja**: cuesta ~15 % más de energía por metro útil, tiene un vuelco más y no reduce los atascos reales. La mejora de atascos y vuelcos que se esperaba de navegar con A* sigue **sin medirse**; lo que sí se mostró es que el vaivén y el límite de aceleración de R6 y la alarma de pendiente puntual de R2 perjudicaban.

### Límites y sin probar
- Suelos supuestos (Marte, Titán, Ceres; Luna con K y kc distintos de Apolo 17, ver R5), una masa, una velocidad (0.5 m/s), suspensión `simple`; sin ruido de sensores (el navegador ve el terreno exacto). La básica no tiene seguridad: comparar con ella premia también no frenar ante un borde, que con un peligro real sería un error.
- 150 casos por conjunto: una diferencia de ±5 puntos en llegada no es resoluble (semiamplitud del IC ~4-5 puntos). 10 comparaciones por régimen sin corrección. Cada semilla aparece 5 veces en celdas distintas (los casos no son independientes del todo).
- Las ablaciones de un componente en `ajuste2` (n = 50) no distinguen entre sí a los componentes de R6; las combinaciones de `completa_r10` se escogieron en 110 casos de ajuste (60 de R4 + 50), y la ablación de la alarma de pendiente se añadió tras ver la validación (pero se midió solo en el ajuste).
- Sin probar: `suspension: rocker_bogie`, el margen de vuelco con rocas (el valor dudoso de R6 sigue sin revisarse), rutas de más de 32 m, otras velocidades, el nodo ROS, `completa_r10` en el conjunto de validación (se reservó), pendientes laterales de más de 12° y un vaivén rediseñado (el de R6 sale mal; no se probó otro).

### Cómo reproducir
```bash
python3 -m pytest -q src/atp_rover/test/test_validacion_nav.py                                  # 7 pruebas, < 1 s
python3 scripts/estudio_navegacion_r10.py --rapido --trabajadores 4                              # ~3 min; runs/rover_r10_rapido/
python3 scripts/estudio_navegacion_r10.py --conjunto validacion --trabajadores 4                 # 300 corridas, ~1.5 h con 4 procesos
python3 scripts/estudio_navegacion_r10.py --conjunto ajuste2 --modos basico,completa,completa_r10 --ablaciones --candidatos
python3 scripts/estudio_navegacion_r10.py --conjunto tercero --modos basico,completa,completa_r10   # 450 corridas, ~2 h
```
La caché `docs/rover_resultados_r10/corridas.jsonl` hace que repetir el análisis tarde segundos; la caché del ajuste se migró a los nombres finales de `completa_r10` (se verificó re-corriendo dos casos).
