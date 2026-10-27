# sat_adcs — segmento espacio: ADCS de un CubeSat 3U

Control de actitud (*Attitude Determination and Control System*) de un CubeSat 3U,
en lockstep con el reloj de la Estación ATP (CONTRATO sec. 8). Cierra el lazo con el
segmento tierra: el error de apuntamiento de la **antena del satélite** hacia la
estación terrena durante un **pase real (SGP4)** se traduce en pérdida de enlace con
el mismo modelo de `atp_ros.model.presupuesto_enlace`.

```
DETUMBLE (B-dot, magnetopares) ─▶ NADIR (PD en cuaterniones, ruedas) ─▶ TRACK_GS (antena a la estación)
        solo magnetómetro              TRIAD + MEKF de 6 estados          preapunta 150 s antes del AOS
```

Paquetes: `src/sat_adcs` (este) y `src/sat_interfaces` (mensajes `AdcsState`, `AdcsCmd`).
No toca `atp_interfaces` ni `atp_ros` (solo importa `atp_ros.orbit` y `atp_ros.model`).

## Uso

```bash
docker compose build                               # una vez
./run.sh bash scripts/validar_adcs.sh              # pruebas + escenarios ROS + determinismo
./run.sh ros2 launch sat_adcs adcs.launch.py time_scale:=0
./run.sh ros2 launch sat_adcs adcs.launch.py time_scale:=0 start_utc:=2026-10-02T08:30:00   # pase en eclipse
./run.sh ros2 launch sat_adcs adcs.launch.py time_scale:=0 model_degree:=1                  # dipolo a bordo
./run.sh python3 -m sat_adcs.escenario --inicio_utc 2026-10-02T20:30:00                      # sin ROS (~1 min)
./run.sh python3 -m pytest -q src/sat_adcs/test                                              # ~6 min
```

Argumentos del launch: `sat`, `station`, `start_utc`, `pass_index`, `el_max_min`, `lead_s`
(el pase empieza al menos `lead_s` después del inicio), `seed`, `w0_deg_s` (tumbling inicial),
`step_s` (periodo del lazo, 1 s), `model_degree` (grado del campo magnético de a bordo),
`band`, `d_sat` (apertura equivalente de la antena del satélite). El resto de `misiones.Config`
se fija con `--ros-args -p` en cada nodo. Al final el monitor imprime el RESUMEN:

```
RESUMEN detumble: |w| 8.00 -> < 0.594 deg/s en t=... s (real); modo NADIR desde t=... s
RESUMEN NADIR (regimen, tras 207 s): n=... rms=... p95=... max=... deg
RESUMEN TRACK_GS (en pase, el>=5 deg): n=... rms=... p95=... max=... deg | umbral 2.40 deg (perdida 1 dB, haz 8.3 deg)
RESUMEN conocimiento (rms est-verdad, bore-sight): ... deg (1-sigma del filtro, eje peor: ... deg) | ruedas: h max ...
RESUMEN enlace X (antena del satelite 0.3 m): perdida por apuntamiento rms=... max=... dB | margen min=... | cerrado ...%
RESUMEN veredicto: CUMPLE (p95 TRACK_GS <= umbral)
```

## Lockstep (CONTRATO sec. 8)

```
/clock t_k ─▶ adcs_sim integra [t_{k-1}, t_k] con el mando vigente (ZOH) ─▶ /atp/adcs_state (sello t_k)
           ─▶ adcs_controller: estimador + modo + ley de control ─▶ /atp/adcs_cmd (MISMO sello t_k)
           ─▶ adcs_sim lo aplica y confirma en /atp/step_ack (Clock = t_k) ─▶ sim_clock publica t_{k+1}
```

| Nodo | Hace | Reglas del contrato que cumple |
|---|---|---|
| `adcs_sim` | planta: RK4, perturbaciones, sensores con ruido, verdad; publica `/atp/adcs_done` (latched) | sella con el instante del cálculo; confirma solo el mando del mismo sello (uno rancio se ignora); no arranca hasta tener `wait_subscribers` suscriptores y fija el inicio con el sello del primer mando aceptado (nada depende del descubrimiento DDS) |
| `adcs_controller` | TRIAD + MEKF, modos, B-dot, PD con ruedas, desaturación; solo ve sensores y la efeméride | responde a **cada** estado con el mismo sello; un estado repetido (arranque) recibe el mismo mando sin tocar el filtro; no responde hasta que su publicador tenga `wait_subscribers` suscriptores (planta y monitor), regla 4: sin eso, una corrida de determinismo mostró al monitor perdiendo las primeras 165 muestras |
| `adcs_monitor` | mide con la verdad, imprime el RESUMEN | pasivo: colas de 400, empareja estado y mando **por sello**, cierra con un temporizador de 3 s simulados tras `/atp/adcs_done` |

`AdcsState` lleva los sensores (lo único que usa el controlador) **y** la verdad (solo el monitor).
`sim_clock` es el de `atp_ros` con `step_s` = periodo del lazo (1 s: un ADCS de CubeSat corre a 1-10 Hz;
con 0.05 s el escenario de ~2 h simuladas serían 144 000 pasos). La planta y el controlador viven en
`misiones.py` sin ROS: `escenario.simular` los encadena en el mismo orden y da el mismo RESUMEN que ROS.

## Modelo (cada cifra dice dónde vale)

Convenciones: cuaternión escalar-primero, Hamilton, `q` = rotación **cuerpo → inercial (TEME)**;
`dq/dt = ½ q ⊗ [0, ω_B]`; error `dq = q_ref⁻¹ ⊗ q`. Cuerpo: +Z (eje largo) = bore-sight de la antena.
LVLH: +Z nadir, +Y = −normal de la órbita. Documentadas en `fisica.py`.

| Bloque | Qué es | Dónde deja de valer |
|---|---|---|
| Dinámica | Euler con N ruedas: `J ω̇ = τ − ω×(Jω + A h) − A u`, `ḣ = u`; saturación de par y de momento (la rueda no pasa de ±h_max dentro del paso); RK4 de paso fijo | cuerpo rígido; el momento de la rueda es el axial absoluto, sin fricción ni dinámica de rotor |
| Campo magnético | armónicos esféricos de Schmidt hasta grado 13, coeficientes **IGRF-13 reales** (`igrf13_datos.py`, generado del archivo de NOAA/NCEI), variación secular 2020-25. **Verdad = grado 13**; a bordo, grado configurable: 1 = **dipolo inclinado (NO es IGRF)** | solo campo principal (sin corteza ni campo externo: decenas de nT en calma); extrapolar la variación secular más allá de 2025 deriva decenas de nT |
| Gradiente de gravedad | `3μ/r³ ĉ×Jĉ` | Tierra puntual (sin J2) |
| Aerodinámico | placas planas, Cd = 2.2 supuesto, viento co-rotante | atmósfera exponencial por capas (tabla de Vallado de memoria, empalme verificado < 1 %); la densidad real varía ×2-×10. Solo orden de magnitud |
| Dipolo residual + magnetopares | `m × B` | residual supuesto |
| Sol / eclipse | Vallado alg. 29 (~0.01°); sombra cilíndrica | sin penumbra (~10 s); TEME ≈ MOD (< 0.4°) |
| Giróscopo | ARW + sesgo en paseo aleatorio (Farrenkopf) | sin escala/desalineación/temperatura |
| Magnetómetro, sensor solar | ruido blanco; Sol con error angular, 4π (6 sensores) | calibrado, sin campo propio de los magnetopares |
| TRIAD | actitud de dos vectores (inicializa el filtro) | vectores casi paralelos |
| MEKF | 6 estados (δθ, sesgo), Φ exacta de `exp(F dt)`, Q de primer orden, forma de Joseph | Q vale si ‖ω‖dt ≪ 1; el error del modelo magnético se trata como ruido blanco (no lo es) |
| B-dot | `m = −k Ḃ` saturado, `Ḃ` filtrado; `k = J_min/(τ B²)` derivado en el docstring | el magnetómetro se lee con el magnetopar activo (en vuelo se alternan) |
| PD | `ω̇_e = −2ζωₙ ω_e − 2ωₙ² δq_v`, prealimentación de `ω_ref`, `α_ref`, desacople de Euler y límite de velocidad `h_max/(2 J_max)` | un solo cuerpo rígido |
| Desaturación | `m = −(B×h)/(τ_d B²)` → `τ_m = −h⊥/τ_d` | no descarga la parte de `h` paralela a `B` en ese instante |

### Satélite de referencia

| Cifra | Valor | Origen |
|---|---|---|
| Dimensiones, masa | 100×100×340 mm, 4 kg | CubeSat Design Specification rev. 13 (de memoria, no re-verificado); el 4 kg es el techo: **supuesto** |
| Inercia | caja homogénea `m/12 (a²+b²)` → Ixx = Iyy ≈ 0.0419, Izz ≈ 0.0067 kg m² (calculada) + 2 % de productos de inercia | **supuesto** del reparto interno |
| Ruedas | 3 ortogonales, 0.23 mNm, 1.77 mNms | hoja de CubeSpace CubeWheel Small (verificada, satnow/CubeSpace) |
| Magnetopares | 0.2 A m² por eje | placa ISISpace iMTQ (verificada) |
| CG, dipolo residual | (1, 0.5, 2) cm; (0.01, −0.005, 0.008) A m² | **supuestos** |
| Sensores | magnetómetro 150 nT, giróscopo ARW 1e-4 rad/√s, RRW 1e-6 rad/s^1.5, sesgo inicial 5e-4 rad/s, sol 0.5° | **supuestos** típicos de COTS |
| Antena del satélite | banda X, 0.3 m equivalente (reflectarray desplegable), η = 0.5, 5 W, 100 Mbps | **supuestos**; haz `70λ/D` = 8.3° |
| Órbita | FUNCUBE-1 (TLE de `data/tle`) como órbita SSO de 600 km; el 3U es genérico, no el FUNcube real (1U) | |

Umbral de apuntamiento (derivado del enlace, no del resultado): `L = 12 (e/θ₃)²` → con 1 dB de pérdida,
`e = θ₃ √(1/12)` = **2.40°**. `test_umbral_de_apuntamiento…` lo verifica por construcción.

## Validación (`pytest src/sat_adcs/test`: 41 pruebas)

| Prueba | Contra qué | Cifra (medida, o cota que la prueba exige) |
|---|---|---|
| Conservación de momento y energía | invariantes exactos sin pares (RK4, h = 0.05 s, 600 s, 20°/s) | error relativo 4.6e-9 y 2.5e-9 (cota 1e-7) |
| Orden de RK4 | solución analítica de Euler, h = 0.5 → 0.0625 s | error ÷16.0 al duplicar el paso: pendiente **4.0** (la energía converge a ~h⁵) |
| Precesión libre axisimétrica | `λ = (Ia−It)/It ω_z`, cono inercial y `φ̇ = H/It` | ω_x, ω_y a 1e-8; ángulo del cono a 1e-8; precesión a 1e-6 rel |
| Gradiente de gravedad | `1.5 n²(Ix−Iz) sin 2θ`; equilibrio LVLH estable (Iy > Ix > Iz): libración de cabeceo con `ωp = n√(3(Ix−Iz)/Iy)` a < 1 %; inestable con el eje mayor en la vertical (crece > 10×) | acotado a 3× la perturbación en 3 órbitas |
| Saturación de ruedas | par y momento nunca exceden los límites; con `u = 0` posterior se conserva `J ω = −h` | exacto |
| Campo geomagnético | `B = −∇V` por diferencias finitas (grados 1-13); Legendre contra `scipy.special.lpmv`; dipolo (2×B₀ en el polo, B₀ en el ecuador magnético); anclas publicadas (SAA ~23 µT, polos ~55-60 µT) | gradiente a 2e-6 |
| Error del modelo de a bordo | contra IGRF-13 completo, órbita polar 600 km | grado 1: **32 % rms (12.7° rms, 32° máx)**; grado 2: 20 % (7.4°); grado 4: **3.8 % (1.7°)**; grado 8: 0.14 % (0.07°) |
| Sol | ejemplo 5-1 de Vallado (0.01°); declinación en equinoccio/solsticio; distancia perihelio/afelio; fracción de eclipse `asin(R/a)/π` | |
| TRIAD | exacto sin ruido; con ruido, error del eje primario `√2 σ₁` y del giro `σ₂/sin∠` (Shuster-Oh) | a 15-20 % |
| **MEKF: convergencia y consistencia (NEES)** | N = 60 corridas independientes con la misma estadística que el modelo del filtro; NEES medio contra la banda de dos colas al 99 % de χ²(6N)/N | NEES medio **6.22, 5.70, 6.09** en t = 60, 200, 500 s; banda **[4.91, 7.21]** (esperado 6); error de actitud 0.20 → 0.11° |
| PD con ruedas | giro de 120° sin pares: converge < 0.05°, nunca excede h_max, conserva el momento total, velocidad ≤ 1.25 w_lim; seguimiento de referencia móvil < 0.01° | |
| Desaturación y ganancia B-dot | igualdades algebraicas de la derivación | exactas |
| Geometría y referencias | elevación en AOS/LOS = máscara (0.05°); `q_ref_gs` apunta +Z a la estación (< 1e-9 rad); ω_ref de LVLH = −n (1e-6) | |
| **B-dot** (3 semillas + control negativo) | `|ω|` decae por bloques de 15 min, converge bajo el umbral y pasa a NADIR en < 2 h; con `k < 0` crece | |
| Escenario: DETUMBLE → NADIR → TRACK_GS | pase real de 79° en luz; pase de 36° en **eclipse** | ver abajo |
| Determinismo | misma semilla → muestras idénticas; otra semilla → distintas | |

## Resultados medidos del escenario

Condición: tumbling inicial 8°/s en dirección aleatoria, paso 1 s, pases reales de FUNCUBE-1 sobre
Ciudad de México (TLE de `data/tle`, época 2026-10-01). **Luz** = pase de 79° (AOS 22:16:53 UTC);
**eclipse** = pase de 36° (AOS 10:11:59 UTC) dentro de la sombra de la Tierra. Semilla 7.
p95 del error de apuntamiento de la antena durante el pase (el ≥ 5°); umbral 2.40°.

| Campo de a bordo | Luz: TRACK p95 | Luz: conocimiento rms | Eclipse: TRACK p95 | Eclipse: conocimiento rms |
|---|---|---|---|---|
| grado 1 (dipolo) | 0.98° | 8.7° | **2.46° (no cumple)** | 7.3° |
| grado 2 | **5.70° (no cumple)** | 4.6° | **6.11° (no cumple)** | 4.7° |
| **grado 4 (por omisión)** | **0.81°** | 0.89° | **0.28°** | 0.40° |
| grado 8 | 0.25° | 0.14° | 0.20° | 0.10° |
| grado 13 (= la verdad) | 0.22° | 0.10° | 0.28° | 0.10° |

Con el grado 4 (24 coeficientes IGRF, ~190 bytes) y la semilla 7, **corrida ROS completa**
(luz, `scripts/validar_adcs.sh escenario`): NADIR p95 1.79°, TRACK_GS rms 0.62°, p95 0.81°, máx 0.85°;
pérdida por apuntamiento de la antena del satélite rms 0.075 dB y máx 0.12 dB; margen de enlace mínimo
14.05 dB, enlace cerrado el 100 %; momento de ruedas máximo 0.82 mNms (46 % de la capacidad). En eclipse:
TRACK p95 0.28°, ruedas 1.06 mNms (60 %). Detumbling de 8°/s (semilla 7): |ω| real bajo el umbral
`h_max/(4 J_max)` = 0.594°/s (calculado de las ruedas) a t = 3111 s (luz) y 3260 s (eclipse); el modo NADIR empieza
en t = 4247 s en el pase de luz porque el TRIAD necesita el Sol y el satélite estaba en eclipse hasta ahí (el
RESUMEN lo avisa); en el otro es a t = 3395 s. Con otras semillas la entrada a NADIR va de 1.7 a 4.2 ks.

Monte Carlo corto (6 semillas = actitud inicial, velocidad inicial y ruido, mismo pase):

| Escenario | TRACK p95 por semilla |
|---|---|
| luz, grado 4 | 0.69, 0.72, 0.73, 0.76, 0.70, 0.74° (6/6 cumplen) |
| eclipse, grado 4 | 0.45, 0.41, 0.46, 0.43, 0.42, 0.50° (6/6) |
| luz, dipolo | 0.35, 1.29, 0.58, 0.74, 0.66, 0.69° (6/6 cumplen) |
| luz, grado 2 (3 semillas) | 5.70, 6.32, 6.03° (no cumple) |
| eclipse, dipolo (3 semillas) | 2.68, 2.97, 2.92° (no cumple) |

Determinismo (ROS): 4 corridas seguidas del escenario corto (`scripts/validar_adcs.sh det` hace 2) dan el
RESUMEN **idéntico byte a byte**, y la corrida ROS completa da el mismo RESUMEN que `adcs_offline`
(sin ROS), en luz y en eclipse.

### Qué dice esto (y qué no)

1. **El modelo de campo de a bordo es lo que fija la actitud.** El dipolo deja 32 % de error de
   campo; el filtro termina con 7-10° de error de conocimiento (en NADIR, en eclipse) y, peor, está
   **sobreconfiado**: reporta un 1-σ de 0.5° contra 8° reales. El error del modelo está correlacionado
   en el tiempo (no es ruido blanco) y el MEKF lo trata como blanco; inflar R no lo arregla. Con el
   grado 4 el filtro sigue algo sobreconfiado (0.12° reportado contra 0.9° real) y recién con el
   grado 8 es consistente (0.07-0.11° contra 0.10-0.14°).
2. **Que un pase "cumpla" con un modelo malo puede ser casualidad de la geometría.** En luz el sensor solar
   fija dos ejes y el tercero lo da el campo; si el error del campo en ese punto de la órbita cae
   sobre un eje que no mueve el bore-sight, el apuntamiento sale bien (dipolo: 0.35-1.3°) aunque el
   conocimiento sea malo; con el grado 2 el mismo pase da 5.7-6.3°. No es monótono en el grado ni depende de la
   semilla (cambia con el pase y el campo). Por eso el valor por omisión es el grado 4, que cumple en las
   dos geometrías y en las 6 semillas, y no el dipolo.
3. El umbral de 2.4° es **estrecho** solo para una antena de haz de 8°: con una antena de parche en banda S
   (`--banda S --d_sat_m 0.1 --p_tx_w 2 --tasa_bps 1e6` en `adcs_offline` → haz 95°) el umbral es 27.5° y,
   medido con el dipolo en el pase de luz, la pérdida por apuntamiento máxima es 0.001 dB (margen 8.96 dB):
   la precisión que exige el ADCS la fija la antena, no el satélite.
4. Los números valen para **estas dos pasadas, un satélite, un TLE y 6 semillas**: no es una campaña
   estadística de pases (eso sería un Monte Carlo sobre fechas, estaciones y elevaciones, como `campaign_server`).

## Supuestos y limitaciones (sin maquillar)

* La efeméride de a bordo es SGP4 con el mismo TLE que la planta (órbita conocida sin error, como un
  GPS); la actitud no afecta a la órbita (no hay arrastre ni acople). El error de SGP4 (~1 km) no importa para la actitud.
* **Verdad ≠ realidad**: IGRF-13 grado 13 sin campo externo ni de corteza; atmósfera exponencial;
  sin penumbra; sensor solar de 4π; magnetómetro calibrado y sin campo de los magnetopares.
* El 3U se simula con la inercia de la caja sin el reflectarray desplegable de 0.3 m de la antena
  (inconsistencia asumida: desplegarlo cambiaría la inercia y añadiría flexibilidad).
* Las ruedas no tienen fricción, curva par-velocidad, ni *jitter*; el lazo de velocidad de la rueda es ideal.
* El ambiente (posición, campo, densidad) se congela durante cada paso de 1 s (punto medio): error
  relativo ~1e-3 en el par; el equilibrio LVLH tiene una deriva numérica de 1e-6 rad con paso de 1 s.
* No hay modo seguro (*safe mode*), detección de fallas ni transición de vuelta a DETUMBLE si las ruedas
  saturan o el filtro diverge. El escenario no incluye reinicios ni pérdida de sensores.
* El enlace supone la estación bien apuntada (el segmento tierra mide ~0.001°) y el modelo gaussiano del
  haz, válido en el lóbulo principal (fuera, la pérdida se acota con la ganancia, como en `model`).
* El estimador necesita el Sol para arrancar (TRIAD): si el detumbling termina en eclipse, espera a la luz (el RESUMEN lo avisa).
* Una corrida ROS completa son ~7000 pasos, unos 2 minutos con `time_scale:=0`.

## Qué falta para hardware (plan, no implementación)

Objetivo: el controlador en un microcontrolador (p. ej. Cortex-M4F/M7 o ESP32-S3) conectado a ROS 2 por
**micro-ROS**, con la planta (`adcs_sim`) en el PC: simulación con *processor-in-the-loop* y luego con sensores reales.

1. **Portar el controlador** (`ControladorAdcs`: TRIAD, MEKF 6×6, B-dot, PD) a C/C++ sin memoria dinámica
   (Eigen o CMSIS-DSP, `float` o `double` según la FPU). Costo estimado: un MEKF de 6 estados son decenas de
   kflop por actualización, milisegundos en un M4F a 1 Hz. El modelo de campo grado 4 son 24 coeficientes en flash.
   **Prueba de equivalencia**: grabar sensores y mandos de esta simulación (`AdcsState`/`AdcsCmd`) y reproducirlos
   en el binario en el PC; deben coincidir dentro del redondeo de `float`.
2. **micro-ROS**: nodo `adcs_controller` con las mismas interfaces. `sat_interfaces` usa solo arreglos de tamaño
   fijo y escalares, compatibles con micro-ROS (se compila en el firmware con `micro_ros_setup`). El lockstep
   funciona igual con el enlace serie/UDP: la planta no avanza hasta recibir el mando con su sello, así que la
   latencia solo ralentiza la simulación; hay que medirla contra el periodo del lazo para el HIL a tiempo real (`time_scale:=1`).
3. **PIL → HIL**: primero el MCU con la planta simulada; luego magnetómetro, giróscopo y sensores solares
   reales (I²C/SPI) con **jaula de Helmholtz** y mesa de rodamiento de aire (o plataforma giratoria), con ruedas y magnetopares reales.
4. **Lo que el modelo no tiene y el hardware sí exige**: calibración del magnetómetro (hierro duro y blando,
   sesgo, no ortogonalidad) y del giróscopo (temperatura); **multiplexado de magnetopares y magnetómetro** (medir con el magnetopar
   apagado); marcas de tiempo del hardware; lazo de velocidad de la rueda y su fricción; modos seguros y FDIR
   (vigilante, límites de |ω| y de momento de ruedas, reintento de DETUMBLE); presupuesto de tiempo real y de memoria;
   estimación del error del modelo magnético (estados aumentados) en vez de tratarlo como ruido blanco.
5. **Orden**: (a) equivalencia numérica en el PC, (b) PIL con el escenario completo, (c) sensores reales en
   banco (solo lectura, comparar con la verdad simulada), (d) lazo cerrado en el banco con actuadores.

## Archivos

| Archivo | Qué contiene |
|---|---|
| `sat_adcs/fisica.py` | librería numérica pura (cuaterniones, dinámica, ambiente, sensores, estimación, control) |
| `sat_adcs/igrf13_datos.py` | coeficientes IGRF-13 (generado desde NOAA) |
| `sat_adcs/misiones.py` | `Config`, efeméride, planta, controlador (modos), medidor y RESUMEN, enlace |
| `sat_adcs/escenario.py` | elección del pase real, corrida sin ROS y CLI (`adcs_offline`) |
| `sat_adcs/nodo_*.py`, `nodos_comunes.py` | nodos ROS en lockstep |
| `launch/adcs.launch.py` | lanzador con `sim_clock` de `atp_ros` |
| `test/` | `test_fisica.py`, `test_misiones.py` |
| `../sat_interfaces/` | mensajes `AdcsState`, `AdcsCmd` |
| `../../scripts/validar_adcs.sh` | pruebas + escenarios ROS + determinismo |
