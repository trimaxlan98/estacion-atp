# Control avanzado de la montura (equipo H)

Este documento describe la planta realista, el LQR con acción integral, el
filtro de Kalman, el estudio comparativo contra el PID y lo que cambia al
pasar a un motor real. Cada cifra sale de código de este repositorio y se
puede regenerar (sección 7). Lo que es un **supuesto** está marcado así.

Piezas nuevas (todas opcionales; con los parámetros por defecto el
comportamiento anterior no cambia bit a bit, ver sección 6):

| Archivo | Qué es |
|---|---|
| `src/atp_ros/atp_ros/planta.py` | Física pura (numpy) compartida por el nodo y las campañas: fricción, backlash, gravedad, encoder, viento con espectro |
| `src/atp_ros/atp_ros/mount_sim.py` | Nodo de la montura; usa `planta.py` (parámetros nuevos apagados por defecto) |
| `src/atp_ros/atp_ros/control.py` | Riccati, LQR integral, márgenes, Kalman, `campana_lazo_ctrl` (Monte Carlo por controlador) |
| `src/atp_ros/atp_ros/lqr_controller.py` | Nodo LQR; hereda del PID el emparejamiento por sello |
| `src/atp_ros/atp_ros/pid_controller.py` | PID de siempre + `estimator:=kalman` opcional |
| `launch/atp_loop.launch.py` | `controller:=pid\|lqr`, `estimator:=none\|kalman` y los parámetros de planta |
| `scripts/estudio_control.sh`, `scripts/estudio_campana.py` | Reproducen las tablas de la sección 5 |
| `src/atp_ros/test/test_control.py` | 40 pruebas (sección 6) |

## 1. Planta

El eje de siempre, en el eje de carga (inercia reflejada, J = 2 kg m², b = 0.5 N m s/rad,
tope de par 8 N m):

    J θ'' + b θ' = u + d(t)

Capas opcionales (parámetros de `mount_sim`, argumentos del launch):

| Capa | Parámetros | Modelo | Fuente / condición |
|---|---|---|---|
| Fricción | `coulomb_nm`, `stribeck_nm`, `stribeck_vel_deg_s`, `friction_veps_deg_s` | F(ω) = [Fc + (Fs−Fc) e^{−(ω/ωs)²}] sgn ω, con adherencia tipo Karnopp (si \|ω\| < veps y \|par neto\| ≤ Fs, ω = 0) | Armstrong-Hélouvry, Dupont y Canudas de Wit (1994), Automatica 30(7). **Supuesto**: Fc, Fs y ωs no están medidos |
| Backlash | `backlash_deg`, `backlash_motor_frac`, `backlash_restitution` | Dos cuerpos (motor y carga) con holgura δ: libre dentro de ±δ/2, contacto rígido fuera, impacto con restitución e. J y b se reparten `frac`/(1−`frac`); u actúa en el motor; viento, gravedad y fricción en la carga | Nordin y Gutman (2002), Automatica 38. Contacto detectado por subpaso de 2 ms. **Supuesto**: reparto de inercia 50/50 y e = 0 |
| Desbalance | `unbalance_nm` | par −m g d cos(el) solo en elevación | m g d es un **supuesto** (parámetro) |
| Encoder | `encoder_bits`, `encoder_side` | cuantización mid-tread de 2π/2^bits (14 bits: 0.02197°); `load` mide la carga, `motor` mide detrás del reductor | exacto |
| Viento | `wind_sigma_nm`, `wind_model`, `wind_tau_s`, `wind_v10_m_s` | par aleatorio de RMS dado con espectro **Dryden** (Gauss-Markov de 1.er orden, τ = L/V; MIL-HDBK-1797) o **Davenport** (S(f) = 4κV10² x²/[f(1+x²)^{4/3}], x = 1200 f/V10; Davenport 1961, κ = 0.005) | La **forma** espectral es de la fuente; la **amplitud** es un supuesto: el par real de un plato de 3 m (78 N m a 10 m/s, `par_viento` de atp.py) excede por mucho el J didáctico de 2 kg m², así que se fija por parámetro. Se mantiene constante durante cada paso del reloj (20 Hz) |

Determinismo: el ruido del encoder y el viento usan generadores propios con
semilla (`seed` y `seed + 1000`), de modo que activar el viento no cambia la
secuencia del ruido del encoder.

`publish_truth:=true` (launch `truth:=true`) publica `/atp/mount_truth` con el
ángulo REAL de la carga, y `error_monitor` (`use_truth`) mide contra él. Con
backlash o cuantización el error medido por el encoder y el real difieren; ningún
controlador usa esta señal (tampoco `velocity` de `/joint_states`, que es la
velocidad real de la planta).

## 2. LQR con acción integral

Por eje, x = [θ, ω], A = [[0,1],[0,−b/J]], B = [0, 1/J]ᵀ. Discretización exacta con
retención de orden cero a Ts = 1/20 s (`discretizar_zoh`: exponencial de
[[A,B],[0,0]]Ts, sin scipy). Estado aumentado z = [θ−r, ω−ṙ, ζ] con ζ⁺ = ζ + Ts(θ−r):

    z⁺ = [[Ad,0],[Ts·C,1]] z + [Bd;0] v        v = u − u_ff

Riccati discreta resuelta por duplicación estructurada (SDA; Chu, Fan, Lin y Wang
2004) en `dare`; K = (R + BᵀXB)⁻¹BᵀXA. Ley:

    u = clip( J r̈ + b ṙ − K [θ̂−r, ω̂−ṙ, ζ] − d̂ )

* **Pre-alimentación**: u_ff = J r̈ + b ṙ (dinámica inversa). ṙ y r̈ salen de las
  muestras de la referencia con diferencias hacia atrás de 2.º orden
  (`DiferenciadorRef`, error O(Ts²), causales).
* **Anti-windup** por clamping, igual que el PID.
* **Pesos por defecto** (Bryson, sintonizados contra el estudio): Q = diag(10⁴, 1, 10³), R = 1 →
  K = [84.0, 17.8, 24.6], polos del lazo cerrado dentro del círculo unidad
  (|z| máx 0.984, el polo lento del integrador), margen de fase del lazo digital 52.0°
  en ωc = 9.8 rad/s, margen de retardo 92.5 ms (casi dos periodos), margen de ganancia
  hacia arriba infinito y hacia abajo 44 dB, pico de sensibilidad Ms = 1.29.
* **Elección de q_theta**: con 10⁵ (K = 207) el margen de fase baja a 44.5° y el
  margen de retardo a 49 ms; con el LQR sin Kalman (derivada filtrada) el lazo
  oscilaba en la simulación ROS (error 0.87° rms). 10⁴ es el compromiso adoptado.
* **Protocolo**: `lqr_controller` hereda de `PidController` `on_ref`/`on_meas` (mismo
  emparejamiento por sello, mismo respaldo con aviso) y responde a cada estado con
  `/atp/torque` sellado con ese instante; solo reemplaza `control()`.

## 3. Estimador

Kalman de [θ, ω, d] por eje, con d el par de perturbación (fricción, gravedad,
viento lento) como camino aleatorio:

    θ' = ω,  ω' = (u + d − bω)/J + wₐ,  d' = w_d      (densidades q_acc, q_par)

Discretización de Van Loan (`discretizar_ruido`). Medida: θ con ruido gaussiano σ y
cuantización de `bits` bits, con R = σ² + q²/12 (cuantizador uniforme; válido si el
error de cuantización es ≈ uniforme e incorrelado: requiere que la señal cruce varios
escalones por muestra o que haya ruido del orden de q/2). Forma de Joseph; la covarianza
es común a todos los ejes y corridas, se propaga una vez. Predice con el mando
previo u_{k−1} (el realmente aplicado, saturado) y corrige con la medida del mismo
sello. Defaults q_acc = 10⁻⁴ (rad/s²)² s, q_par = 10⁻² (N m)²/s, elegidos en un barrido sobre
planta realista (ver sección 5; son **supuestos de ajuste**, no medidos).

Dos usos: `controller:=lqr estimator:=kalman` (θ̂, ω̂ y la compensación d̂) y
`controller:=pid estimator:=kalman` (el PID usa e = r − θ̂ y la derivada ṙ − ω̂ en lugar
de diferenciar la medida). Sin `estimator:=kalman` el LQR usa la misma derivada filtrada
(τ = 0.08 s) que el PID.

Nota honesta: LQG **no** hereda los márgenes del LQR (Doyle 1978). Los márgenes de
la sección 2 son los del lazo con estado real; con el Kalman en el lazo no se
reclaman, y el estudio es la prueba de robustez práctica.

## 4. Funciones numéricas (`control.py`) contra el curso 30, lección 3.1

`lqr`, `lqr_doble_integrador`, `margenes`, `margen_fase_con_retardo` y
`controlabilidad` reproducen `atp.py`; las pruebas fijan las cifras y, si el repo del
curso está disponible, las contrastan en vivo:

| Cifra | Valor | Prueba |
|---|---|---|
| controlabilidad del eje | det = −0.25, rango 2 | `test_controlabilidad_curso` |
| q = 100, r = 1 | k1 = 10, k2 = 4.47, ωn = 3.16 rad/s, t_est = 1.75 s | `test_cifras_leccion_3_1` |
| r = 100 | ωn = 1 rad/s, 3.16× más lento (100^¼); ζ = 0.707 para cualquier q, r | idem |
| eje J = 2, b = 0.5 | K = [10, 5.844], MF 66.84° | `test_margenes_curso_lqr_continuo` |
| doble integrador | MG = ∞, MF = 65.53° en ωc = 4.91 rad/s | idem |
| retardo | 0.05 s → 51.45°; 0.1 s → 37.38°; 0.2 s → 9.23° | idem |

Nuevo: `dare`/`lqr_discreto` (contra forma cerrada escalar y contra
`scipy.linalg.solve_discrete_are`), `margenes_discreto` (MF, MG hacia arriba/abajo,
margen de retardo y Ms del lazo digital), `FiltroKalman` con `nees()`.

## 5. Estudio comparativo

Todos los controladores comparten planta, semilla, perturbaciones y pase. Métrica:
error de apuntamiento en el cielo sqrt((e_az cos el)² + e_el²) contra el objetivo de 0.1°.
Las corridas ROS son un pase cenital completo (1018 muestras a 20 Hz) en lockstep
(`time_scale:=0`), y **incluyen el transitorio de enganche**: la montura arranca en
reposo sobre una referencia que ya se mueve. `pid+kf` es el PID con `estimator:=kalman`.

### (a) Pase cenital N2 (9 °/s en acimut, keyhole), banda S, planta ideal

| Controlador | rms (°) | p95 (°) | máx (°) | dentro de 0.1° |
|---|---|---|---|---|
| PID (referencia de la escalera) | 0.0167 | 0.0173 | 0.138 | 99.3 % |
| PID + Kalman | 0.0161 | 0.0173 | 0.113 | 99.5 % |
| LQR | 0.0070 | 0.0027 | 0.097 | 100 % |
| LQR + Kalman | 0.0020 | 0.0001 | 0.038 | 100 % |

Tres corridas por celda: el PID dio la misma cifra las 3 veces; PID+KF, LQR y LQR+KF en 2 de 3
(la tercera cayó en la carrera de arranque, con n distinto de muestras; ver limitaciones). **Por qué**: el PID
es de tipo 2 (planta con integrador + integral) y sigue la rampa sin error, pero no
conoce la aceleración de la referencia; el LQR la pre-alimenta (J r̈ + b ṙ) y su
velocidad de referencia entra en el estado de error, y con Kalman la ω̂ no tiene el
retardo de la derivada filtrada de 80 ms. Gran parte del residuo que queda en todos es
el transitorio de enganche (en las corridas que perdieron las primeras muestras por la
carrera de arranque el rms del LQR+KF fue 0.0000, lo que lo indica; no lo separé
sistemáticamente).

### (b) N4b: cenital en Ka (haz 0.233°) con ráfaga de 3 N m a los 60 s

| Controlador | rms (°) | p95 (°) | máx (°) | dentro de 0.1° |
|---|---|---|---|---|
| PID | 0.466 – 0.471 | 0.2095 – 0.2120 | 9.47 – 9.55 | 92.2 % |
| PID + Kalman | 0.481 | 0.209 | 9.77 | 92.2 % |
| LQR | 0.140 | 0.0080 | 2.98 | 97.7 % |
| LQR + Kalman | 0.073 | 0.0009 | 1.68 | 99.5 % |

(El PID varía en la 3.ª cifra entre lanzamientos, ver limitaciones; las demás filas son
repetibles.) **Por qué**: una ráfaga de 3 N m es 28 veces el par de seguimiento (0.108 N m)
y en un resorte de rigidez k da un error de 3/k rad en pico: el PID (kp = 25) llega a
~7° (3/kp rad) y tarda ~kp/ki = 12 s en recuperarse; el LQR (K_θ = 84) a ~2 – 3° y recupera con
el polo de integrador de 0.98; con Kalman la estimación del par d̂ se compensa y el error cae a
1.7°. Ninguno cumple 0.1° *durante* la ráfaga (haría falta K_θ ≈ 1700 N m/rad); lo que gana el LQR es el
tiempo que pasa fuera del haz: 7.8 % → 2.3 % → 0.5 % de las muestras sobre 0.1°.

### (c) Planta realista: backlash 0.05°, Coulomb 0.05 + Stribeck 0.02 N m, encoder de 14 bits (error REAL)

| Controlador | rms (°) | p95 (°) | máx (°) | dentro de 0.1° |
|---|---|---|---|---|
| PID | 0.1045 | 0.215 | 0.283 | 67.8 % |
| PID + Kalman | 0.105 | 0.217 – 0.218 | 0.296 – 0.298 | 68.5 – 69.0 % |
| LQR | 0.0197 – 0.0200 | 0.044 – 0.052 | 0.102 – 0.106 | 99.9 % |
| LQR + Kalman | 0.0073 – 0.0074 | 0.0115 – 0.0131 | 0.067 – 0.068 | 100 % |

(c2) lo anterior + desbalance de elevación 0.05 N m + viento Dryden de 0.02 N m rms + ruido
de encoder 0.005°: PID rms 0.102 – 0.103 (p95 0.198, 62 – 63 % dentro); PID+KF 0.098 (p95 0.185,
64 – 67 %); LQR 0.023 – 0.026 (p95 0.051 – 0.060, 100 %); LQR+KF 0.0112 – 0.0118 (p95 0.021 – 0.024, 100 %).
Las filas con muestras faltantes por la carrera de arranque se excluyeron (PID c2 n = 900, LQR+KF c2 n = 1003).

**Por qué**: la fricción estática de 0.07 N m contra kp = 25 deja 0.16° de zona de adherencia en
la que el PID depende de su integral lenta (ki = 2): se atasca, se acumula y se suelta.
El LQR con K_θ = 84 la reduce a 0.048° y su integral es más rápida; el Kalman estima el
par de sesgo (la fricción y la gravedad se ven como d̂) y lo cancela, y de paso filtra la cuantización
de 14 bits (0.022°, ×20 Hz en la derivada). El PID + Kalman no gana nada: el cuello del PID es la
rigidez y la integral, no la derivada.

### (d) Campaña Monte Carlo (n = 300 pases al azar, semilla 2029, `control.campana_lazo_ctrl`)

Referencia: `model.campana_lazo(n=300)` da p50 0.0295, p95 0.0423, p99 1.73, peor 2.49
(el p95 = 0.042 del README), y `campana_lazo_ctrl('pid')` con la planta por defecto lo
reproduce idéntico. Incertidumbre del p95 de n = 300: ±1.3 puntos percentiles.

| Escenario | PID p50 / **p95** | LQR p50 / **p95** | LQR + Kalman p50 / **p95** |
|---|---|---|---|
| d1 nominal (sesgo ±0.03°, ruido 0.02°, latencia ≤ 45 ms, ráfaga) | 0.0295 / **0.0423** | 0.0378 / **0.0486** | 0.0345 / **0.0460** |
| d2 + backlash 0.05°, Coulomb/Stribeck, 14 bits | 0.0473 / **0.0689** | 0.0378 / **0.0485** | 0.0351 / **0.0460** |
| d3 + desbalance 0.05 N m + viento Dryden 0.02 N m | 0.0751 / **0.0975** | 0.0402 / **0.0506** | 0.0378 / **0.0484** |
| d4 = d3 sin sesgo de encoder ni latencia | 0.0707 / **0.0860** | 0.0312 / **0.0325** | 0.0280 / **0.0292** |

Los tres cumplen p95 < 0.1° en los cuatro escenarios, pero el PID en d3 queda a 0.0025° del
límite. El p99 (1.72 – 1.73°) y el peor (2.5°) son los pases que culminan sobre la
elevación de keyhole (~82.5°, el rotor topa a 6 °/s): el 4 % de los pases pierde el
satélite con cualquier controlador. Con ≥ 5 % de pases así el p95 también se desplomaría;
es una cota de la montura, no del control. **Lectura honesta**: en el escenario nominal el
PID es *el mejor* (0.0423 contra 0.0460 – 0.0486): el presupuesto lo dominan el sesgo del
encoder (±0.03°), la latencia (v·Δt) y el ruido, que ningún controlador corrige; el
LQR, de ancho de banda mayor, deja pasar más ruido. La ventaja del LQR + Kalman aparece cuando
la planta se ensucia (d2 – d4: p95 0.0460 – 0.0484 contra 0.0689 – 0.0975) y, sobre todo,
cuando se le quita al presupuesto lo que no depende del control (d4: 0.0292 contra 0.0860).

### Qué gana cada uno

* **PID**: simple y suficiente con planta ideal y ruido gaussiano (mejor en d1). Se degrada con
  fricción, backlash y cuantización (c: 68 % del pase dentro de 0.1°) porque su rigidez es baja
  (kp = 25) y su integral lenta.
* **LQR + integral + pre-alimentación**: mayor rigidez (K_θ = 84), polos con amortiguamiento
  garantizado y seguimiento sin retardo de la aceleración de la referencia. Gana contra
  fricción, ráfagas y pase cenital (a: 0.0167 → 0.0070). Pierde ante ruido de medida porque
  deriva la medida (mismo filtro de 80 ms que el PID, pero con ganancia de velocidad K_ω = 17.8 vs kd = 8).
* **LQR + Kalman**: el filtro da ω̂ sin retardo ni amplificación de cuantización y estima el
  par de sesgo; es el único que cumple con holgura (rms 0.007°, 100 % dentro) la planta realista de (c).
  Costo: dos densidades de ruido (q_acc, q_par) que ajustar y que son supuestos hasta medir el sensor.

## 6. Pruebas

`bash scripts/validar.sh test` (container): 69 passed, 1 skipped (29 anteriores + 40 de `test_control.py`; el skip es el previo).

Las pruebas de `test_control.py` cubren: Riccati continua y discreta contra forma
cerrada y scipy; estabilidad del lazo cerrado (|z| < 1); cero error estacionario con
par constante; márgenes (curso 3.1 y lazo digital); Kalman (covarianza estacionaria =
DARE dual, NEES medio dentro de 3 ± 0.4 con 400 corridas, estimación del par de sesgo);
fricción (adherencia por debajo de Fs, arranque por encima); backlash (holgura acotada
a ±δ/2, zona muerta, equivalencia con el cuerpo rígido para δ → 0); gravedad; cuantización;
viento (estadística de Dryden, la varianza de Davenport integra a 6κV10²); y que **con
los parámetros por defecto la planta es idéntica bit a bit** a la integración original
(`test_planta_por_defecto_identica_bit_a_bit`) y `campana_lazo_ctrl('pid')` es idéntica
a `model.campana_lazo` (`test_campana_pid_identica_a_model_campana_lazo`).

## 6b. Equivalencia por defecto y determinismo

Con los parámetros por defecto (`scripts/validar.sh test n2 n4b c1 c2 det`, mismo contenedor):
`n2` da `n=1018 ... apuntamiento: rms=0.0167 p95=0.0173 max=0.1382` y `n4b`
`n=6435 ... rms=0.4658 p95=0.2092 max=9.4559`, **idénticos a los del baseline previo a
los cambios**; `det` (N2 dos veces) da `DETERMINISTA: identicas`. `c1` (LQR en N2):
`rms=0.0070 p95=0.0027 max=0.0967`; `c2` (LQR+Kalman, planta realista, error real):
`rms=0.0073 p95=0.0131 max=0.0667`.

**Limitaciones del determinismo (previas a este trabajo, medidas aquí)**: el lockstep no es
bit-exacto entre lanzamientos cuando la máquina está cargada (load average ~60 durante parte
del estudio, por otros procesos). Síntomas observados: (1) un `n` distinto de muestras
(833, 745, 1036, 900, 1003 en lugar de 1018) con cifras casi nulas: el pase arranca con un
desfase de pasos respecto al monitor; (2) en N4b la ráfaga cae en otra fase del pase y el
PID varía en la 3.ª cifra (p95 0.2092 – 0.2120). Hipótesis (no verificada con trazas):
`reference_generator` espera `wait_subscribers` solo en `/atp/sat_state`, y las otras
suscripciones del monitor pueden emparejarse después de que el pase ya arrancó. El nivel
`det` del `validar.sh` para `atp_autotrack.launch.py` ya daba "NO DETERMINISTA" en el baseline
antes de tocar nada (n = 5985 vs 6226), y sigue igual. Esta rama no lo corrige; el estudio
de la sección 5 repite cada celda y reporta el rango.

## 7. Reproducir

```bash
./run.sh bash scripts/validar.sh test n2 n4b c1 c2 det
./run.sh bash scripts/estudio_control.sh        # tabla ROS (a, b, c, c2)
./run.sh python3 scripts/estudio_campana.py 300 # tabla Monte Carlo (d)
```

## 8. Qué cambia al pasar a un motor real

* **Driver y mando**: hoy `/atp/torque` es un par en N m con retención de orden cero. Un
  driver real (servo-amplificador) cierra un lazo de corriente propio y acepta par
  (modo torque) o velocidad; el reductor y el rendimiento ya no son parte del "eje
  reflejado". Con modo velocidad el LQR debe rediseñarse con la planta de
  velocidad (A = −1/τ_v), no con J y b.
* **PWM y cuantización del mando**: el DAC ideal pasa a ser un PWM de resolución
  finita y frecuencia fija; hay que añadir la cuantización de u al modelo (análoga a
  la del encoder) y, si el PWM es lento, un retardo equivalente: el margen de retardo
  calculado (93 ms) es el presupuesto disponible para cómputo + bus + driver.
* **Encoder absoluto**: el relativo pierde la cuenta al arrancar y exige referenciado
  (home); el absoluto da θ al encender, pero con comunicación serie (SSI/BiSS) trae
  una latencia y un periodo de muestreo propios. Con 14 bits la cuantización
  (0.022°) ya consume un quinto del presupuesto de 0.1°: el Kalman sirve, pero un
  encoder de 17–19 bits reduce el problema. Un encoder en el motor (detrás del
  reductor) hace invisible el backlash al lazo: elegir entre `encoder_side:=motor`
  (estable, apunta peor) y `load` (ve el backlash, riesgo de ciclo límite) o doble
  lazo.
* **ros2_control**: la montura pasaría a una interfaz de hardware con `command
  interface` de esfuerzo/velocidad y `state interface` de posición, y el LQR a un
  `controller` propio (ChainableController) en lugar de un nodo con tópicos. El
  lockstep deja de existir: sin `step_ack`, el lazo corre con el reloj del
  controller_manager (tiempo real o RT) y el emparejamiento por sello se sustituye por
  la lectura síncrona del `update()`. Hay que medir el jitter real y pasar de
  Ts fijo a Ts medido (el Kalman admite redescretizar por paso).
* **Identificación**: J, b, Fc, Fs, ωs, holgura y m g d son supuestos aquí. En el
  hardware se identifican (barrido senoidal para J y b, rampas lentas para Fc/Fs,
  inversión de sentido a par bajo para el backlash) y se re-sintonizan q_acc y q_par
  con el ruido real del sensor (varianza de Allan).
* **Seguridad**: topes de par y velocidad por hardware, final de carrera y
  paro de emergencia; el `u_max` simulado no sustituye al limitador del driver.

## 9. Limitaciones

* J = 2 kg m² es la inercia didáctica del curso; los pares de viento y gravedad reales
  son órdenes de magnitud mayores. Las amplitudes de fricción, backlash, gravedad y viento
  son supuestos, no medidas; solo las formas (Stribeck, Davenport, Dryden, hibrido de
  backlash) vienen de la literatura.
* El Kalman usa q_acc y q_par ajustados a mano en un barrido de 12 + 6 corridas de N2 realista
  (no una optimización) y R de la cuantización uniforme; los pesos del LQR (q_theta = 10⁴)
  se eligieron tras ver que 10⁵ oscilaba sin Kalman. Ambos se sobreajustaron un poco al
  pase cenital; no se validaron en pases oblicuos con ROS (sí en la campaña de 300 pases).
* El LQR es el mismo para acimut y elevación y desprecia el acoplamiento cos(el) de la
  cinemática; el estudio mide el error de apuntamiento real, que ya lo incluye.
* El backlash se detecta una vez por subpaso de 2 ms; el viento se mantiene constante
  cada 50 ms. No hay flexibilidad estructural ni retardo de cómputo/actuador.
* La campaña Monte Carlo y el nodo comparten `planta.py`, pero la campaña integra con
  dt = 10 ms (como `campana_lazo`) y el nodo con 2 ms.
* No se probó en hardware.
