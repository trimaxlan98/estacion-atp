# Hardware de la Estación ATP: qué comprar y por qué (equipo S)

Cómo pasar de «el firmware compila y la simulación pasa» a «hay una placa, un motor y un receptor sobre la mesa».
Cada requisito sale de **números calculados** con `scripts/requisitos_hw.py` (geometría de `atp_ros.model`, pases SGP4 reales de
`data/tle`, planta de [CONTROL](CONTROL.md)) y cada uno trae su fórmula y su fuente. Lo que es un **supuesto** o está **de memoria** (sin
contrastar con la hoja de datos) va rotulado *SUPUESTO* o *verificar*. Los precios no están: solo especificaciones (y, donde se nombra un
modelo, es uno real y conocido, siempre *verificar la hoja de datos vigente antes de comprar*).

> **Estado, con honestidad.** Nada de esto se ha probado con hardware: el firmware **nunca se ha cargado en una placa**
> ([firmware](../firmware/esp32_eje/README.md) §9). Las cifras de este documento son requisitos *derivados de la simulación* y propuestas de diseño;
> las pruebas que las confirmarían están por definir con el hardware real.

Reproducir todas las tablas: `PYTHONPATH=src/atp_ros python3 scripts/requisitos_hw.py` (≈ 25 s; `--json f.json` guarda todo; `--rapido` omite SGP4).
Pruebas: `src/atp_mcu/test/test_requisitos_hw.py` (23 pruebas, cada cifra contra una fuente independiente).

## 0. Resumen en una pantalla

| Escenario | Para qué | Haz −3 dB | Presupuesto de apuntamiento (θ/10) | Encoder mínimo → recomendado | Velocidad / aceleración de diseño (az) | Par de carga (con margen 1.5) | Motor (eje de carga, 24 V) |
|---|---|---|---|---|---|---|---|
| **A. Banco** (pan-tilt pequeño, láser/cámara, sin RF) | demostrar el lazo, el paro, el watchdog, el HIL | — (objetivo del curso 0.1°) | 0.1° | 12 bits (0.11°) → **14 bits** (0.022°) | 20°/s, 40°/s² | 0.21 N·m | ≈ 0.09 W mecánicos: cualquier motorreductor pequeño o NEMA 17 |
| **B. Yagi VHF/UHF** (radioaficionado) | NOAA/ISS/CubeSats con SDR | 51° (12 dBi) | 5.1° | 7 bits (5.6°) → **9 bits** | 6°/s, 2°/s² | 13 N·m (viento 10 m/s: 6.6 N·m, SUPUESTO área 0.3 m²) | ≈ 2 W: un rotor comercial basta |
| **C. Plato 3 m en X (8.4 GHz)** (el caso duro de la Estación) | la Estación completa | 0.833° | 0.083° | 12 bits (0.09°) → **14 bits+** (≥ 17 recomendado) | 30°/s, 30°/s² | 227 N·m | ≈ 140 W de pico, 6.5 A a 24 V, reductora 600:1 |

La cinemática del azimut **domina por el keyhole**: un pase con elevación máxima de 87° a 400 km pide 21°/s y 5°/s²; de 89°, 63°/s y 45°/s² (§2.1).
Un rotor de antena de ~6°/s (G-5500) pierde el 3.2 % de los pases a 550 km (§2.2): es una cota de la montura, no del control.

## 1. Escenarios

| | A. Banco | B. Yagi | C. Plato |
|---|---|---|---|
| Antena | ninguna; láser o cámara sobre la montura | Yagi ≈ 12 dBi (boom ≈ 3 m a 145.8 MHz, o ≈ 1.5 m a 437 MHz) | plato 1–3 m, S (2.2 GHz) o X (8.4 GHz) |
| Ancho de haz | — | θ ≈ √(41253/G) = 51° (G = 12 dBi; Kraus, *Antennas*; contraste: 70 λ/D con D = boom 3 m → 48°, `model.ancho_haz`) | θ = 70 λ/D (`model.ancho_haz`): 1 m S 9.54°, 3 m S 3.18°, 1 m X 2.50°, 3 m X **0.833°**, 3 m Ka 0.233° |
| Criterio | error rms total ≤ 0.1° (criterio de aceptación A1) | e ≤ θ/10 | e ≤ θ/10 (pérdida media 0.12 dB; A3) |
| Inercia de carga | J = 0.05 kg·m² (*SUPUESTO*: láser/cámara de 0.5 kg a 0.3 m) | J = 3 kg·m² (*SUPUESTO*: Yagi de 4 kg y 3 m, m L²/12) | J = 110 kg·m² (*SUPUESTO*: plato de 80 kg, ½ m R² + m d², d = 0.5 m; «cientos de kg·m²» según [CONTROL](CONTROL.md) §9) |
| Por qué importa | no se puede dañar nada; todo el software se prueba aquí | primera señal real | la meta |

Regla del presupuesto: e_total = θ3/10 corresponde a una pérdida de 12 (e/θ)² = 0.12 dB (`model.perdida_apuntamiento`). Se reparte
como suma **RSS** (fuentes independientes), propuesta de ingeniería, no cifra de una fuente:

| Fuente | Fracción del cuadrado | Plato 3 m X (e_total = 0.0833°) | Banco (0.1°) |
|---|---|---|---|
| encoder (cuantización) | 10 % | 0.026° | 0.032° |
| holgura (backlash) | 15 % | 0.032° | 0.039° |
| nivelación / orientación / cero (calibrable) | 20 % | 0.037° | 0.045° |
| control (seguimiento residual) | 25 % | 0.042° | 0.050° |
| latencia y sellado de tiempo | 10 % | 0.026° | 0.032° |
| flexión, térmica, viento | 20 % | 0.037° | 0.045° |

## 2. Requisitos calculados

### 2.1 Cinemática de pase (velocidad y aceleración por eje)

Fuente: `model.perfil_pase` (órbita circular, Tierra esférica, **sin** rotación terrestre; cada función de `model.py` dice su condición) con n = 40 000 puntos; la velocidad
de acimut en la culminación coincide con la fórmula cerrada `model.tasa_acimut` (v = ω/cos el, ω = v_circ/rango) y con SGP4 (abajo). La aceleración es el gradiente numérico de la velocidad.

| h (km) | el_max | v_az (°/s) | a_az (°/s²) | v_el (°/s) | a_el (°/s²) |
|---|---|---|---|---|---|
| 400 | 50 | 1.34 | 0.02 | 0.35 | 0.02 |
| 400 | 70 | 3.03 | 0.10 | 0.66 | 0.05 |
| 400 | 80 | 6.24 | 0.44 | 0.86 | 0.12 |
| 400 | 85 | 12.56 | 1.79 | 0.98 | 0.24 |
| 400 | 88 | 31.47 | 11.23 | 1.05 | 0.60 |
| 550 | 70 | 2.18 | 0.05 | 0.48 | 0.03 |
| 550 | 80 | 4.49 | 0.23 | 0.62 | 0.06 |
| 550 | 85 | 9.04 | 0.93 | 0.70 | 0.12 |
| 550 | 88 | 22.64 | 5.81 | 0.75 | 0.31 |
| 800 | 70 | 1.48 | 0.02 | 0.33 | 0.01 |
| 800 | 80 | 3.03 | 0.10 | 0.42 | 0.03 |
| 800 | 85 | 6.11 | 0.42 | 0.48 | 0.06 |
| 800 | 88 | 15.29 | 2.65 | 0.51 | 0.14 |

Más extremos (n = 200 000): 87° → 21.0°/s y 5.0°/s² (400 km), 15.1°/s y 2.6°/s² (550 km), 10.2°/s y 1.2°/s² (800 km); 89° → **63°/s y 45°/s²** (400 km), 45°/s y 23°/s² (550 km), 31°/s y 11°/s² (800 km).
**La elevación no es problema**: pico < 1.1°/s y < 0.6°/s² incluso en el cenit (`v/h` = 0.79°/s a 550 km). Todo el requisito mecánico es del **acimut** y crece como 1/cos(el).

**Distribución por elevación máxima.** En una estación fija los pases altos son raros: con lam_min ~ U(0, lam_máscara) (`model.elmax_aleatorio`), P(el_max > e) = lam(e)/lam(5°):

| h | > 60° | > 70° | > 80° | > 85° | > 88° |
|---|---|---|---|---|---|
| 400 km | 12.6 % | 8.0 % | 3.9 % | 1.9 % | 0.8 % |
| 550 km | 14.0 % | 8.9 % | 4.3 % | 2.2 % | 0.9 % |
| 800 km | 15.9 % | 10.2 % | 4.9 % | 2.5 % | 1.0 % |

**Pases SGP4 reales** (`orbit.predecir_pases` sobre CDMX, 72 h desde 2026-10-02, 117 pases completos de ISS, FUNcube-1, CO-57, NOAA 15/18/19/20/21 y METEOR-M2 3/4 de `data/tle`):

| el_max | n | v_az mediana (°/s) | v_az máx | a_az máx (°/s²) | v_el máx |
|---|---|---|---|---|---|
| 0–20° | 48 | 0.20 | 0.37 | 0.00 | 0.08 |
| 20–40° | 32 | 0.31 | 0.72 | 0.01 | 0.17 |
| 40–60° | 23 | 0.62 | 1.39 | 0.02 | 0.37 |
| 60–80° | 11 | 2.07 | 4.84 | 0.27 | 0.75 |
| 80–90° | 3 | 9.65 | 32.6 | 11.8 | 0.51 |

Validación de la geometría esférica contra SGP4 con rotación terrestre (estaciones sintéticas bajo la traza de NOAA 19, desplazadas 5/20/60/120 km): a el_max = 88.5° v_az = 19.0 (SGP4) contra 18.6 (esférico); a 85.4° 6.3 contra 6.2; a 80.8° 3.2 contra 3.1: **2–3 %**, coherente con la rotación omitida. Con el pase casi cenital (89.6°) las derivadas numéricas ya dependen del paso de muestreo (75.9 contra 57.1): es la física del keyhole, no un error.

### 2.2 Keyhole: hasta qué elevación llega cada clase de motor y de montura

Un pase con elevación máxima e_k exige v_az(e_k) en el acimut; sobre él la montura **pierde** el satélite (`model.elevacion_keyhole`). Se añade el límite por aceleración (mayor el_max cuyo pico de a_az cabe en la montura, barrido de 1°; tope del barrido 89°):

| Clase (v_az máx; a_az máx, *SUPUESTO de catálogo*) | 400 km | 550 km | 800 km | pases que pierde a 550 km |
|---|---|---|---|---|
| Rotor de antena tipo Yaesu G-5500 (6°/s; 2°/s²) | 79.6° (vel.) | **82.5°** | 84.9° | 3.2 % |
| Servo / paso a paso de banco (20°/s; 40°/s²) | 86.9° | 87.7° | 88.5° | 1.0 % |
| Servomotor con reductora, plato 1–3 m (30°/s; 60°/s²) | 87.9° | 88.5° | 89.0° | 0.6 % |
| Direct-drive / servo rápido (90°/s; 400°/s²) | ≥ 89° (acel. 89°) | ≥ 89° | ≥ 89° | 0.4 % |

(82.5° a 550 km con 6°/s coincide con [CONTROL](CONTROL.md) §d y con `test_cifras_conocidas_del_repositorio`.) **Cada clase pierde un 0.4–4 % de los pases**: ninguna llega al cenit exacto (la demanda diverge). Lo que se puede hacer cerca del cenit es **degradar con elegancia**: el pase cenital a 550 km (el_max 89.9°)
con un seguidor ideal de tope v/a deja, en el cielo, un error máximo de acimut de:

| Tope (v, a) | error máx en el cielo | tiempo fuera de 1° |
|---|---|---|
| 6°/s, 2°/s² | 16.5° | 27.9 s |
| 20°/s, 40°/s² | 4.25° | 6.9 s |
| 30°/s, 60°/s² | 2.66° | 4.0 s |
| 90°/s, 400°/s² | 0.62° | 0 s |

(Cota inferior del atraso: un lazo real atrasa más.) Para un haz de Yagi (51°) todo eso es irrelevante; para un plato de 0.83° solo el último renglón sigue al satélite **sin** perder el haz.

**Estrategia «sobre el cenit» (elevación hasta 180°).** Una montura con la elevación de **0 a 180°** pasa sobre el cenit sin girar el acimut 180° (en el cenit el acimut queda fijo y la elevación pasa de 90° a > 90°): la velocidad de elevación del pase cenital es la de la línea de vista, v/h = 0.79°/s a 550 km (calculada 0.789), **sin singularidad**. Es un **requisito mecánico opcional** y de software: rango de elevación 0–180°, límites de software
`lim_max` = 180° en el segundo eje (hoy el firmware usa 0–90°: `ATP_EJE2_LIM_MAX_CDEG`), cableado que admita el giro y una estrategia que decida de qué lado se vuelca. El **G-5500** tiene elevación 0–180° (*verificar* la hoja de Yaesu), de modo que ese camino existe ya en un rotor comercial; el equipo de keyhole (U) decide la estrategia. Con acimut de 450° (G-5500) además hay un solape de 90° que
evita el cruce del norte.

### 2.3 Re-apuntamiento entre satélites (handover)

Salto de acimut (camino corto) entre pases consecutivos del catálogo (`re_apuntamiento`, mismos 117 pases SGP4): **116 transiciones**, mediana 133°, p95 176°, máx 179.4°; en 17 el hueco es ≤ 60 s y 16 se solapan (hueco 0 s). El peor caso razonable es **180° de acimut**.
Velocidad pico de un perfil trapezoidal de aceleración a que recorre Δ en T: T = Δ/v + v/a ⇒ v = (a T/2)(1 − √(1 − 4Δ/(a T²))):

| Δ | T = 10 s | T = 20 s | T = 30 s |
|---|---|---|---|
| 90° | 9.1–10.0°/s | 4.5–4.6°/s | 3.0°/s |
| 180° | **18–24°/s** (a = 100…10°/s²) | 9.0–9.4°/s | 6.0–6.1°/s |

(`v_slew_necesaria`.) Conclusión: con un rotor de 6°/s, un cambio de 180° tarda ≥ 30 s; con 20–30°/s cabe en 10 s. La aceleración de diseño del plato ya sale del keyhole (30–60°/s²), más que suficiente para estos saltos.

### 2.4 Resolución de encoder

Regla: la cuantización de un cuantizador uniforme de paso q tiene rms q/√12 (Widrow y Kollar, *Quantization Noise*, 2008), así que q = √12 · e_enc con e_enc = e_total √0.10.
Cuentas por vuelta de **carga** N = 360/q; un incremental de PPR pulsos da 4·PPR cuentas (cuadratura x4, PCNT del firmware). «Recomendado» = +2 bits (4× más fino): el PID **deriva** la posición y la cuantización se amplifica (abajo).

| Antena | θ3 | e_total | q máx | N (cuentas/vuelta carga) | bits mínimo | bits recomendado |
|---|---|---|---|---|---|---|
| banco (0.1°) | — | 0.1° | 0.1095° | 3 286 | 12 | 14 |
| Yagi 10 dBi | 64.2° | 6.4° | 7.04° | 51 | 6 | 8 |
| Yagi 12 dBi | 51.0° | 5.1° | 5.59° | 64 | 7 | 9 |
| Yagi 15 dBi | 36.1° | 3.6° | 3.96° | 91 | 7 | 9 |
| 1 m S | 9.54° | 0.954° | 1.045° | 345 | 9 | 11 |
| 2 m S | 4.77° | 0.477° | 0.523° | 689 | 10 | 12 |
| 3 m S | 3.18° | 0.318° | 0.348° | 1 034 | 11 | 13 |
| 1 m X | 2.50° | 0.250° | 0.274° | 1 315 | 11 | 13 |
| 2 m X | 1.25° | 0.125° | 0.137° | 2 631 | 12 | 14 |
| **3 m X** | 0.833° | 0.0833° | 0.0912° | **3 946** | 12 | **14** |
| 3 m Ka | 0.233° | 0.0233° | 0.0255° | 14 104 | 14 | 16 |

Conclusiones y salvedades:

* **La cuantización no manda; manda la exactitud absoluta (INL), el backlash y la nivelación.** Un AS5048A de 14 bits (0.022°; *de memoria* su no linealidad integral es de ~décimas de grado, *verificar la hoja de ams* antes de contar con él en absoluto) cumple la cuantización del plato de 3 m en X pero su INL **no** cabe en 0.083°: se calibra con el modelo de apuntamiento (APUNTAMIENTO.md) o se usa un encoder óptico de ≥ 17 bits (131 072 cuentas, 0.0027°; [CONTROL](CONTROL.md) §8 ya recomienda 17–19 bits). El firmware solo soporta incremental (PCNT) o AS5048A (SPI): **un encoder SSI/BiSS requeriría código nuevo**.
* **Lado del encoder.** Un encoder en el motor (detrás de la reductora) **no ve el backlash**; con un incremental de 1 000 PPR en el motor y una reductora de 100:1 hay 400 000 cuentas/vuelta de carga (0.0009°), pero la holgura queda fuera del lazo (`encoder_side:=motor`, CONTROL §8). Para el plato: encoder de carga o doble lazo.
* **Tasa de cuentas.** El PCNT con el filtro de glitch de 1 µs del firmware admite ≈ 2 Mcuentas/s (cada canal necesita semiperiodo > 1 µs ⇒ 4 f < 2 M). A 30°/s con 3 946 cuentas/vuelta son 329 cuentas/s: **sobra por 4 órdenes**; con 400 000 cuentas y 30°/s son 33 kcuentas/s, también sobra.
* **Ruido de velocidad por cuantización.** Con el filtro de velocidad de 20 ms del firmware (τ_vel) y muestreo de 1 kHz, σ_v ≤ σ_q/√(dt τ), σ_q = q/√12 (hipótesis de ruido blanco): 14 bits ⇒ **1.42°/s**, 12 bits 5.67°/s, 17 bits 0.18°/s. Contrastado en simulación a 9°/s: 0.31°/s medido, **la fórmula es una cota alta** (a 9°/s la señal cruza 0.4 escalones por muestra y el ruido no es blanco). El término derivativo del PID (kd = 8, τ_d = 80 ms) multiplica un ruido mucho menor (σ_q/√(dt τ_d) ⇒ ≈ 0.7°/s con 14 bits): ≈ 0.1 N·m de ruido de par frente a 8 N·m de tope (1 %). Con 12 bits el ruido de par sube 4×; otra razón para los +2 bits.
* **Paso a paso (banco).** Sin encoder: paso de carga = 360/(pasos·micropasos·N); con motor de 200 pasos/vuelta, micropaso 16 y reducción 2:1 son 0.056° (cumple 0.1°) a 356 pasos/s para 20°/s. El micropaso (TMC2209) da resolución, **no exactitud** (par de retención y holgura la limitan).

### 2.5 Par, potencia, reductora y corriente

Par en el **eje de carga** para un mando (v, a), con la planta de [CONTROL](CONTROL.md) §1 (J θ″ + b θ′ = u + d):

    T = (J α + b ω + T_fricción + T_viento + T_desbalance) × 1.5          (margen: criterio de ingeniería)

| Término | Banco | Yagi | Plato 3 m |
|---|---|---|---|
| J α (a de diseño) | 0.035 | 0.10 | 57.6 (30°/s²) |
| b ω | 0.007 | 0.05 | 1.05 |
| fricción | 0.05 | 1.0 | 5.0 |
| viento 10 m/s (½ ρ v² Cd A r, `par_viento` de `atp.py` del curso; ρ = 1.225, Cd = 1.2) | 0 | 6.6 | **77.9** |
| desbalance m g d | 0.05 | 1.0 | 10.0 |
| total sin margen | 0.14 | 8.8 | 151.6 |
| **con margen 1.5** | **0.21 N·m** | **13.2 N·m** | **227 N·m** |

(Fricción, desbalance, J y áreas son *SUPUESTOS* de diseño de este documento; el viento reproduce el 78 N·m a 10 m/s del curso para un plato de 3 m, brazo 0.15 m.) **El viento manda en el plato**: a 20 m/s el término se cuadruplica (≈ 312 N·m, sin margen) y no cabe en este dimensionado: hace falta una posición de **resguardo** (stow) y un freno, y ese par es de supervivencia, no de seguimiento.

Reductora y motor (motor de 3 000 rpm, *SUPUESTO*; rendimiento de la cadena 0.85, *SUPUESTO* típico de un sinfín/planetario):

    N = ω_motor / ω_carga,máx        T_motor = T_carga / (N η)        P = T ω_carga / η        I_bus = P / (V η_elec),  V = 24 V, η_elec = 0.9 (SUPUESTO)

| Demanda del plato (v, a) | N | T_carga (con margen) | T_motor | P pico | I a 24 V |
|---|---|---|---|---|---|
| 6°/s, 2°/s² (rotor de antena) | 3 000 : 1 | 145 N·m | 0.06 N·m | 18 W | 0.83 A |
| 20°/s, 20°/s² | 900 : 1 | 198 N·m | 0.26 N·m | 81 W | 3.8 A |
| **30°/s, 30°/s²** | **600 : 1** | **227 N·m** | **0.45 N·m** | **140 W** | **6.5 A** |
| 30°/s, 60°/s² | 600 : 1 | 314 N·m | 0.62 N·m | 193 W | 9.0 A |
| 90°/s, 400°/s² (direct-drive) | 200 : 1 | 1 296 N·m | 7.6 N·m | 2.4 kW | 111 A |

Lectura: el plato con 30°/s y 30°/s² cabe en un **servomotor de ≈ 150–300 W, 24 V, con reductora total ≈ 600:1** (tren sinfín + etapa planetaria, que además retiene la carga sin energía) y una fuente de **24 V ≥ 10 A**. Inercia reflejada: el motor ve J/N² = 110/600² = 3.1·10⁻⁴ kg·m², unas 3 veces la del rotor si este es de 10⁻⁴ kg·m² (*SUPUESTO*): dentro de la regla práctica de 1–10:1 de dimensionado de servos (*de memoria*). La reductora de inercia adaptada es N_opt = √(J_carga/J_motor) = 1 049; una de 600:1 queda del lado «carga ligera», conveniente para la estabilidad, y la holgura (no la inercia) es lo que limita el ancho de banda.
El banco necesita **0.09 W** (0.004 A): el motor mínimo de un banco es el que cubra el par de fricción del mecanismo (≈ 0.05 N·m a la carga), no la dinámica. **El driver**: el BTS7960 (módulos de 43 A) es el predeterminado del firmware pero *de memoria* su tensión de trabajo llega a ≈ 27 V (*verificar* la hoja de Infineon); para 24 V es válido, para 48 V no. Un servoamplificador comercial con lazo de corriente propio ([CONTROL](CONTROL.md) §8) cambia la planta (modo par/velocidad): el firmware actual manda PWM.

### 2.6 Frecuencia del lazo y latencia ROS ↔ MCU

* **Latencia pura contra el presupuesto.** Con la parte de latencia del reparto (10 % del cuadrado ⇒ e_lat = 0.316 e_total) y una rampa a v: T_máx = e_lat / v (sin compensar). Para el plato de 3 m en X (e_lat = 0.0263°): **2.9 ms a 9°/s** (pase cenital N2) y **0.88 ms a 30°/s**. El serie de 921 600 baudios (1–3 ms estimados, [firmware](../firmware/esp32_eje/README.md) §4) **ya consume ese presupuesto**; Wi-Fi (decenas de ms de jitter) no sirve para el plato sin compensación.
* **Con la extrapolación del firmware** (usa la velocidad de la referencia, `eje_ref_nueva`) el error residual de un retardo T es ½ a T²: T_máx = √(2 e_lat/a): 230 ms (a = 1°/s²) y 42 ms (a = 30°/s²). Es lo que hace que el enlace serie (1–3 ms) y hasta un Wi-Fi bueno quepan; con el LQR el presupuesto total del lazo ROS es el **margen de retardo de 92.5 ms** ([CONTROL](CONTROL.md) §2, calculado para Ts = 50 ms, **no** para el lazo de 1 kHz del MCU).
* **Frecuencia del lazo.** fs ≥ 20 × ancho de banda (Franklin, Powell y Workman, *Digital Control of Dynamic Systems*, cap. 6): con ancho de banda ≈ 2 Hz (ωn ≤ 12 rad/s) basta **40 Hz**; el lazo de **1 kHz** del firmware (tick de FreeRTOS) tiene **25× de holgura** y no limita nada. La referencia llega a 20 Hz (Nyquist 10 Hz) y el firmware la interpola/extrapola: la frecuencia de la **referencia** y la latencia mandan, no la del lazo.

### 2.7 Radiofrecuencia: Doppler, enlace y banco conducido

**Doppler y re-sintonía** (`doppler_pase`, `model.doppler_hz`, `model.tasa_doppler_cenit`; Doppler máximo en AOS y tasa máxima en el cenit, geometría esférica):

| Banda | fd máx (400 / 550 / 800 km) | tasa máx (Hz/s) a 550 km | re-sintonía para residuo ≤ 100 Hz / 500 Hz / 1 kHz |
|---|---|---|---|
| VHF 137.5 MHz | 3.3 / 3.2 / 3.0 kHz | 44 | 2.3 s / 11 s / 23 s |
| VHF 145.8 MHz | 3.5 / 3.4 / 3.2 kHz | 47 | 2.1 s / 11 s / 21 s |
| UHF 437 MHz | 10.5 / 10.1 / 9.6 kHz | **140** | 0.71 s / 3.6 s / 7.1 s |
| S 2.2 GHz | 52.8 / 51.1 / 48.4 kHz | 707 | 0.14 s / 0.71 s / 1.4 s |

(La rotación terrestre cambia vr en ≈ ±6 %.) Un receptor que **re-sintoniza con el Doppler predicho** (TLE/SGP4) deja al lazo de portadora solo el residuo: en UHF basta una orden de sintonía por segundo para ≤ 140 Hz (`atp_sdr` `CadenaRF.paso(fd_pred_hz=…)`, [atp_sdr README](../src/atp_sdr/README.md) «Doppler predicho»); en S a 2.2 GHz hacen falta ≥ 7 por segundo, que **no** es razonable por USB: se rota con un NCO por software. T = tolerancia/tasa (el Doppler cambia casi linealmente en T). Un TCXO de 1 ppm (RTL-SDR Blog V4, *de memoria*) son 437 Hz en UHF: del orden de la tasa de un segundo, por lo que el seguimiento fino lo hace el PLL de `atp_sdr`, no la predicción sola.

**Enlace de un CubeSat de UHF a una Yagi** (`c_n0_enlace`, mismos términos que `model.presupuesto_enlace`: C/N0 = EIRP − FSPL − L + G/T + 228.6; FSPL = 92.45 + 20 log d_km + 20 log f_GHz): EIRP 0 dBW (*SUPUESTO*: 1 W en antena isótropa), Yagi 12 dBi, T_sys = 300 K (*SUPUESTO*, G/T = −12.8 dB/K), pérdidas 3 dB (polarización y cable), 550 km: **C/N0 = 60.7 dB-Hz a 5° de elevación (rango 2 205 km), 67.6 dB-Hz a 30°, 72.8 dB-Hz en el cenit**. Suficiente de sobra para telemetría de 1.2–9.6 kbit/s; con un haz de 51° el apuntamiento es trivial (12 (5°/51°)² = 0.12 dB con 5° de error).

**Banco de RF conducido (emulador de canal sin antena).** SDR transmisor → atenuadores y cables → SDR receptor. Se reproduce en el escritorio lo que en el cielo no se repite: el beacon de `atp_sdr.senal` (portadora o BPSK/QPSK con **Doppler**, tasa Doppler, **retardo** y **nivel** del pase) con el **ruido inyectado en banda base** (la cadena de `atp_sdr` ya fija el piso `N0·fs = 1` y la amplitud por el C/N0). Inyectar el ruido digitalmente evita el problema de los ~120 dB de atenuación: el nivel físico en el receptor se mantiene alto (el ruido inyectado domina al térmico del receptor) y las exigencias de aislamiento bajan a decenas de dB.

Cuentas (`banco_conducido`; ruido térmico N_rx = −174 + NF + 10 log₁₀ fs, con −174 dBm/Hz = 10 log₁₀(k T₀/1 mW), T₀ = 290 K):

| Magnitud | Valor (ejemplo UHF 437 MHz; fs = 2.4 MS/s; NF = 5 dB *SUPUESTO*) |
|---|---|
| ruido térmico del receptor en fs | −174 + 5 + 63.8 = **−105.2 dBm** |
| ruido inyectado en el receptor (≥ 20 dB sobre el térmico) | **−85.2 dBm** (−149 dBm/Hz) |
| señal a C/N0 = 45 dB-Hz sobre esa densidad | −104 dBm (SNR en banda −18.8 dB: la señal va bajo el ruido, como en el cielo) |
| potencia compuesta que sale del SDR transmisor (*SUPUESTO*: ajuste bajo de ganancia TX) | −20 dBm |
| **atenuación total** A = P_tx − P_rx | **65 dB** (p. ej. 20 + 20 + 20 dB fijos + 0–31.5 dB programable para variar el nivel con el pase) |
| aislamiento necesario TX → RX por fuga que **no** pasa por los atenuadores (portadora conducida − 20 dB) | ≥ **85 dB** |

Rango de C/N0 que reproducir: la tabla de la Yagi (§ arriba) da 60–73 dB-Hz con EIRP 0 dBW; para un CubeSat débil o una Yagi corta se emulan 30–55 dB-Hz. El **rango dinámico** de la variación del pase por distancia es 20 log(d_max/d_min) (550 km: ≈ 10.6 dB entre 5° y el cenit); se hace **en banda base** (ganancia digital), no moviendo atenuadores.

**Especificaciones del camino conducido** (todo lo de marcas/modelos, *verificar la hoja*):

| Ítem | Especificación | Por qué |
|---|---|---|
| Frecuencias | 437 MHz (típico CubeSat; `model.BANDAS`), 145.8 MHz y 137.5 MHz opcionales; **banda S 2.2 GHz** solo si el SDR TX/RX llegan (HackRF One 1 MHz–6 GHz; ADALM-Pluto 325 MHz–3.8 GHz de fábrica; el RTL-SDR **no** llega a 2.2 GHz) | las bandas de la Estación |
| TX | SDR con transmisión y 8–12 bits de DAC (HackRF One; ADALM-Pluto es full-duplex y puede cerrar el lazo TX→RX con un solo equipo, pero comparte reloj y no prueba el desplazamiento de oscilador) | generar IQ de `atp_sdr` en tiempo real o desde archivo |
| RX | RTL-SDR Blog V4 (24–1766 MHz, 8 bits, ≤ 2.4 MS/s) o similar; ≥ 2 MS/s | es el receptor de la fase 2: se prueba **el mismo** |
| Potencia máx. en la entrada del receptor | **≤ −30 dBm en operación**; **nunca > −10 dBm** (*SUPUESTO*: *de memoria* un front-end tipo R828D se daña hacia +10 dBm y se satura mucho antes; **verificar la hoja**). Poner **siempre** un atenuador de 10 dB como último elemento junto al receptor. | protección del front-end |
| Atenuación total | 40–70 dB conducidos según el nivel de salida del TX (ejemplo: 65 dB), repartidos en ≥ 3 atenuadores SMA de 50 Ω (3–30 dB, ≥ 1 W, CC–6 GHz) + uno **programable** de 0–31.5 dB, paso 0.5 dB (familia Mini-Circuits RUDAT/RCDAT-USB, *verificar*) | un solo atenuador de 60 dB no es fiable (fuga por el propio cuerpo); la cadena escalonada sí |
| Aislamiento | cada SDR en **caja metálica** separada (aluminio), cable coaxial de doble blindaje (RG-223 o semi-rígido), **ferritas** en los USB, tierra común en un punto, cables lo más cortos y separados posible | fuga radiada TX → RX ≥ 85 dB (cuenta arriba) |
| Comprobación de fuga | 1) cargar 50 Ω en la entrada del RX con el TX encendido: el piso no sube > 1 dB; 2) desconectar el cable de salida del TX: la portadora cae > 20 dB (si no, es fuga, no señal conducida); 3) medir con un analizador de espectro económico (p. ej. tinySA Ultra, *verificar*) | evitar medir fuga |
| Calibración del nivel | acoplador direccional de 20–30 dB (o divisor) hacia un analizador/medidor de potencia; calibrar la ganancia del RX con la señal conocida y **no** fiarse del AGC ([atp_sdr README](../src/atp_sdr/README.md): «la amplitud IQ solo es potencia si se calibra la cadena») | el C/N0 solo vale si el nivel es medido |
| Marcado de lo no verificado | todas las potencias máximas, el NF del RTL-SDR (5 dB), el ruido de fase y la linealidad de los DAC de 8 bits | *SIN PROBAR* |

## 3. Firmware de dos ejes (banco de escritorio con un ESP32)

Opción de compilación **`ATP_EJES=2`** (`menuconfig` → «Ejes que gobierna este ESP32», o `./build.sh dos` que usa `sdkconfig.defaults.2ejes`). Con **`ATP_EJES=1` (el valor por defecto) el firmware es el de siempre**: mismo lazo
(`core/` no cambió: `eje_paso` ocupa los mismos 1 215 B), mismos pines, mismos tópicos y mismo comportamiento (la prueba `prueba_micro_ros_host.sh` da las mismas 14 comprobaciones que antes). Con 2 ejes: acimut = eje 0, elevación = eje 1, **dos encoders PCNT, dos puentes H y un paro común**; el nodo micro-ROS
es uno solo (`esp32_banco`, espacio `/atp/mcu`) y publica `/atp/mcu/az/…` y `/atp/mcu/el/…` con la **misma interfaz** ([CONTRATO](CONTRATO.md) §7): `mcu_merge` no ve la diferencia.

### 3.1 Pines (ESP32-DevKitC WROOM-32; todo configurable)

| Función | Eje 0 (az) | Eje 1 (el) | Notas |
|---|---|---|---|
| Encoder A / B (PCNT, unidades distintas) | 32 / 33 | **25 / 26** | 3.3 V; 25 y 26 son GPIO de entrada/salida normales (no ADC2 en uso) |
| PWM RPWM (o PWM) | 18 | **22** | LEDC, 20 kHz, 10 bits, mismo temporizador, canales 0/1 y 2/3 |
| PWM LPWM (o DIR) | 19 | **23** | |
| EN (R_EN y L_EN juntos) | 21 | **4** | **pull-down externo de 10 kΩ en cada uno** |
| Paro de emergencia | **27, común a los dos ejes** (contacto NC a masa) | | una ISR baja los dos EN |
| Corriente R_IS / L_IS (opcional) | 34 / 35 | **36 / 39** | ADC1, entradas solo-lectura; el eje 1 se muestrea 5 ticks después |
| Consola / micro-ROS serie | UART1 TX 17 RX 16 / UART0 TX 1 RX 3 | | sin cambios |
| LED de estado | 2 | | muestra el **peor** estado de los dos ejes |

Evitados a propósito: GPIO 0, 2, 5, 12, 15 (*strapping*: fijan el modo de arranque, y 12 la tensión de la flash; *de memoria*, GPIO5 y GPIO15 además pueden sacar señal durante el arranque), 6–11 (flash), 16/17 en variante WROVER (PSRAM; la placa es WROOM), y ADC2 (36/39 son ADC1; ADC2 choca con el Wi-Fi). **El GPIO4 no es de arranque ni de flash**; su estado tras el reinicio (*de memoria*, entrada con pull-down débil) hay que **verificarlo en la hoja de datos de Espressif**; el pull-down externo es obligatorio de todos modos. Con 2 ejes el encoder SPI (AS5048A, GPIO 13/14/23/26) **no** está soportado (`Kconfig`: `depends on ATP_EJES = 1`) porque comparte pines con el eje 1; para un absoluto en cada eje haría falta un segundo CS y una segunda trama (diseñado, no escrito).
Los parámetros propios del eje 1 (`ATP_EJE2_*`): pines, cuentas por vuelta, signos de encoder y de motor, par al 100 % de PWM y límites (0–90° por defecto; 0–180° para la montura «sobre el cenit», §2.2). Las ganancias, el watchdog y la vigilancia se comparten.

### 3.2 Presupuesto de tiempo del tick de 1 kHz con 2 ejes

Medido **en el host** (`test_dos_ejes`, mejor de 5; incluye la planta de prueba en `double`, de modo que es una cota superior del costo del lazo en un x86 de escritorio **no** comparable con el Xtensa): **1 eje 94 ns, 2 ejes 148 ns por tick (×1.58)**: dos ejes cuestan menos del doble. Medido en el **ELF compilado** (`xtensa-esp32-elf-nm -S`, ESP-IDF v5.2.8, `-O2`): `eje_paso` 1 215 B (idéntico al de 1 eje: es el mismo código), `motor_aplicar` 256 B,
`tarea` 630 B, `eje_pid_paso` 185 B, `encoder_leer` 24 B, `eje_tel_publicar` 140 B, los buzones 68 B cada uno. Son cuerpos sin bucles: cada instrucción se ejecuta a lo sumo una vez por llamada.

| Paso del tick | 1 eje (README §4) | 2 ejes | Base |
|---|---|---|---|
| leer encoder (PCNT) | < 2 µs | < 4 µs | estimado |
| `eje_paso` | ≲ 12 µs | ≲ 24 µs | cota por tamaño de código (1 215 B) |
| `motor_aplicar` | ~5–10 µs | ~10–20 µs | estimado |
| telemetría y buzones (sección crítica) | ~1–2 µs | ~3–6 µs (2 tel, 2 ref, 2 cmd) | estimado |
| ADC de corriente | ~50 µs cada 10 ms | ~50 µs cada 5 ms (escalonado: nunca dos en el mismo tick) | estimado |
| **Cota de la suma** | ≲ 40 µs | **≲ 80 µs de 1 000 µs (≈ 8 %)** | **estimación**, no medición en el chip |

El firmware sigue publicando `t_lazo_us`, `t_lazo_max_us`, `jitter_max_us` y `ticks_perdidos` en `~/diag` (el tiempo es el del tick completo, el mismo en los dos ejes): **eso es lo que hay que leer en el primer encendido**. El presupuesto de cada tarea no cambia: el lazo sigue en el núcleo 1, prioridad 23, y la tarea micro-ROS en el 0.

Recursos del ESP32 con la compilación real (variante serie): 1 eje **304 384 B** de `.bin` (19.8 % de la partición de 1.5 MB), DRAM estática 40 924 B, IRAM 54 242 B; 2 ejes **304 720 B** (+336 B), 41 180 B (+256 B) y 54 278 B (+36 B). Contra la compilación de referencia del README (300 608 B; 39 300 B de DRAM) el 1 eje creció 3 776 B y 1 624 B de DRAM por los arreglos de dos ejes, el eco y las pozas de micro-ROS (`RMW_UXRCE_MAX_PUBLISHERS` 3 → 5 y `MAX_SUBSCRIPTIONS` 4 → 5; sin esto no caben las 4 publicaciones y las 5 suscripciones del banco).

### 3.3 Variante con motores paso a paso (STEP/DIR + TMC2209): diseñada, **no** escrita

Para el banco es más barata (un NEMA 17 + un TMC2209 por eje) y **no necesita encoder para empezar** (lazo abierto con micropaso). No se implementó en esta ronda porque el firmware cierra un lazo de **par/PWM** con realimentación de encoder, y un paso a paso es un actuador de **posición**: forzarlo en `eje_lazo.c` desordenaría el núcleo probado. Diseño que cabe sin desorden:

* Un driver alterno de `motor.c` seleccionado en `Kconfig` (`ATP_DRV_STEPDIR`): el «mando» `duty` (−1…1) se interpreta como **velocidad** (fracción de la velocidad máxima) y se genera con el periférico RMT o un temporizador de LEDC a la frecuencia de STEP, con la dirección en DIR y EN con pull-down externo (TMC2209: EN activo en bajo; *verificar* el módulo).
* El «encoder» sería un **contador de pasos** por software (`encoder_leer` devolvería los pasos emitidos): el lazo del core cierra sobre esa cuenta, sin cambios en `core/`. Cuentas por vuelta = 200 · micropasos · reducción (3 200·N con 1/16 de micropaso).
* Frecuencia de STEP a 20°/s con 0.056° por paso: **356 pasos/s** (`resolucion_paso_a_paso`); trivial para el ESP32. Par: un NEMA 17 típico da ~0.4–0.5 N·m de retención (*de memoria*), con una reducción 5:1 → 2 N·m a la carga, cuando el banco necesita 0.21 N·m (§2.5): sobra 10×.
* Pruebas en el host: las mismas de `core/` más una de `motor_stepdir` que cuente pasos emitidos contra la velocidad pedida.
Riesgo conocido: sin realimentación, un paso perdido es un error de apuntamiento permanente (el AS5048A o un encoder del eje lo cerraría).

## 4. Banco HIL sin placa (el ensayo general)

`firmware/esp32_eje/scripts/prueba_hil_lockstep.sh` corre el pase N2 en **lockstep** con el nodo micro-ROS del firmware compilado para Linux (`eje_host --ejes 2`: acimut y elevación) en lugar de la montura simulada: `sim_clock` (lockstep), `reference_generator`, `error_monitor` y el puente
**`atp_mcu/mcu_lockstep`** (en el lugar de `mount_sim` + `pid_controller`) con el agente `microros/micro-ros-agent:jazzy` por UDP, dominio 73. Mide la **latencia de ida y vuelta** (el firmware devuelve el sello de cada referencia en `/atp/mcu/eco`, opcional `ATP_ECO_LATENCIA`), el **jitter** del lazo (el firmware lo publica en `~/diag`; `eje_host` además guarda percentiles de los 62 750 periodos) y la **deriva contra el gemelo** (`atp_mcu/gemelo.py`: la
planta de `atp_ros` con su PID a 20 Hz, alimentada con las **mismas** referencias que recibió el MCU). Detalle, resultados y límites: [firmware](../firmware/esp32_eje/README.md) §7c. **Cuando llegue la placa: se cambia el transporte (agente en el puerto serie, `agente.sh serie`) y se omite `eje_host`; nada más** (`origen:=cmd` ya manda `cmd/origen` y `cmd/enable` como hará falta con una placa real).

## 5. Lista de compras por fases

Cada fila: cantidad, especificación mínima y **por qué** (requisito de §2). Modelos solo como ejemplo real; *verificar la hoja vigente*. Sin precios.

### Fase 1: banco de escritorio con un ESP32 (sin RF, sin riesgo)

Objetivo: el banco completo (arranque seguro, paro, signos, referenciado, identificación, sintonía, watchdog, jitter) con motor desacoplado o de banco.

| Cant. | Ítem | Especificación mínima | Por qué |
|---|---|---|---|
| 2 | Placa ESP32-DevKitC (ESP32-WROOM-32E, 4 MB) | ESP32 de doble núcleo a 240 MHz, **WROOM** (no WROVER: 16/17 son de PSRAM), puente USB–serie con CP2102 o CH340 hasta 921 600 baudios | el firmware compila para ESP32 (ESP-IDF v5.2.8); PCNT, LEDC y 2 núcleos; uno de repuesto. S3/C3 **no** están soportados |
| 2 | Motorreductor DC con encoder en cuadratura | 12 o 24 V, par a la salida ≥ 0.3 N·m (el banco pide 0.21 N·m con margen, §2.5), reducción 30–100:1, encoder Hall ≥ 11 PPR en el motor (ej.: Pololu 37D con encoder de 64 CPR y 131:1 ⇒ 8 400 cuentas/vuelta de salida = 0.043°, cumple 0.11° y no el 0.022° recomendado) | cierra el lazo de posición (§2.4); el requisito de cuantización es 12 bits, recomendado 14 |
| 2 | Alternativa: motor paso a paso NEMA 17 + TMC2209 | 200 pasos/vuelta, 1/16 de micropaso, reducción ≥ 2:1 (§2.4); corriente según la hoja | banco más barato y sin encoder (§3.3: **requiere código nuevo**) |
| 2 | Módulo puente H | BTS7960 (43 A, el predeterminado del firmware: RPWM+LPWM+EN) o PWM+DIR (Cytron MD10C, VNH5019; `ATP_DRV_PWMDIR`), tensión de trabajo ≥ la del bus; **comprobar que las entradas aceptan 3.3 V** en el módulo concreto (README §3 punto 3) | un puente por eje |
| 1 | Fuente de laboratorio | 0–30 V, 0–5 A, **límite de corriente regulable** | arrancar con el 10 % de la corriente nominal |
| 1 | Fuente de 12 o 24 V ≥ 5 A con fusible | la del bus del motor del banco | alimentación del motor **separada** de la lógica (README §3 punto 2) |
| 1 | Pulsador de paro de emergencia | seta enclavada, contacto **NC**, 22 mm | paro (README §3 punto 5) |
| 1 | **Contactor/relé de potencia** | 2 polos, bobina de 12/24 V, corriente ≥ 2× la nominal del motor, **en serie con la potencia del motor y accionado por el pulsador sin pasar por el ESP32** | el paro de software **no** sustituye al de potencia; la prueba correspondiente |
| 4 | Fines de carrera | microinterruptores NC, 2 por eje (az, el), **en serie con la bobina del contactor** | cableados a la potencia, no solo a un GPIO |
| 2–3 | Fusibles y portafusibles | rápidos al 1.5–2× de la corriente nominal del bus | protección (README §3) |
| 2 | Supresor TVS | tensión de trabajo ≥ 1.2 × V_bus (para 24 V: SMBJ28A o equivalente) | un motor que regenera sube el bus (README §3 punto 2) |
| 2 | Condensador de bulto | 470–1 000 µF, ≥ 1.5 × V_bus, junto al puente | corrientes pico del PWM |
| 2 | Resistencia de pull-down de **10 kΩ** (EN) | 1/4 W | EN apagado durante el arranque y el reinicio (README §3 punto 4) |
| — | Adaptación de nivel | divisores 1 k/2 k o conversor bidireccional (p. ej. TXS0108E) para encoders de 5 V; **los GPIO del ESP32 no toleran 5 V** | niveles lógicos 3.3 V (hoja de Espressif); *verificar* los encoders Hall: muchos piden ≥ 3.5–5 V de alimentación |
| — | Cable apantallado trenzado para encoder, ferritas, regletas, perfboard | par trenzado con malla, ≤ 1 m | ruido (README §3 punto 6; el PCNT cuenta cualquier flanco) |
| 1 | Mecanismo pan-tilt con rodamientos | carga ≥ 0.5 kg, holgura < 0.1° (no servos de aficionado), rango az 360°, **el 0–180°** (no 0–90°) | el modelo del banco (J = 0.05 kg·m²) y el requisito de 0.1°; la elevación hasta 180° permite medir en hardware la estrategia `sobre_cenit` del keyhole, la única que en simulación elimina la pérdida en pases casi cenitales ([SEGUIDOR](SEGUIDOR.md): error máx. 15.75° → 0.16° a 89.9°) |
| 1 | Láser puntero (clase ≤ 2) y blanco a ≥ 3 m | 0.1° a 3 m = **5.2 mm** (3 000 mm × tan 0.1°) | verdad de terreno del banco sin antena |

**Herramientas de medida**: osciloscopio ≥ 50 MHz de 2 canales o analizador lógico de 8 canales ≥ 24 MS/s (compatible con sigrok/PulseView) para EN, PWM y el flanco del paro (EN baja ≤ 1 ms tras el flanco); multímetro y **pinza amperimétrica**; adaptador USB–serie de 3.3 V (CP2102/FT232) para los logs en el GPIO17 con la variante serie; inclinómetro digital de **0.01° de resolución** (el nivel de burbuja no basta: la prueba correspondiente); cables USB de datos de calidad.

### Fase 2: recepción real con SDR (NOAA/ISS/CubeSats) y emulador de canal conducido

Objetivo: primera señal real y el banco de RF en el escritorio (§2.7). Sigue sin necesitar el plato.

| Cant. | Ítem | Especificación mínima | Por qué |
|---|---|---|---|
| 1–2 | SDR receptor | **RTL-SDR Blog V4** (24–1766 MHz, 8 bits, ≤ 2.4 MS/s, TCXO, bias-T) o equivalente | el receptor de `atp_sdr` (cualquier fuente SoapySDR); 137–437 MHz |
| 1 | SDR con transmisión | **HackRF One** (1 MHz–6 GHz, 8 bits, half-duplex) o **ADALM-Pluto** (325 MHz–3.8 GHz, 12 bits, full-duplex) | transmite el beacon de `atp_sdr` con Doppler, retardo y nivel (§2.7) |
| 1 | Antena de 137 MHz | **QFH** (cuadrifilar) o V-dipolo para APT/LRPT; *verificar que el satélite siga transmitiendo*: varios NOAA se han ido dando de baja | primera señal sin rotor |
| 1 | Antena Yagi de UHF/VHF | ≥ 10 dBi (≥ 12 dBi recomendado), **polarización circular** (cruzada) o lineal con 3 dB de pérdida asumida; haz ≈ 51° ⇒ presupuesto de 5° (§2.4) | pases de ISS (145.8 MHz) y CubeSats de 437 MHz |
| 1–2 | LNA con filtro y bias-T | NF ≤ 1 dB, ganancia 15–20 dB, filtro pasabanda de la banda (137 MHz; 435–438 MHz), cerca de la antena (p. ej. Nooelec SAWbird+ NOAA para 137 MHz) | G/T de §2.7 (T_sys 300 K *SUPUESTO*) |
| 10–15 m | Coaxial de bajas pérdidas | RG-213 / LMR-400 y conectores N/SMA (*de memoria*, el RG-58 pierde ≈ 0.7 dB/m a 400 MHz: no usarlo en UHF) | pérdidas de cable a 437 MHz |
| 1 | GNSS con PPS | u-blox NEO-M8/M9 con salida PPS | reloj de la estación ≤ 1 ms |
| 1 | PC con USB 3 y Docker | ≥ 4 núcleos, 8 GB | `atp_sdr` en tiempo real; la Estación |
| 1 | **Rotor/receptor con Doppler predicho** | el nodo ROS de re-sintonía (diseñado abajo) y un lado receptor controlable: SoapySDR `setFrequency` o **`gqrx` con control remoto de tipo rigctl (puerto 7356)**, o Hamlib `rigctld` con un SDR compatible | requisito de §2.7: UHF ≤ 1 orden/s |
| — | **Camino conducido** (emulador de canal) | | §2.7 |
| 3–5 | Atenuadores SMA fijos 50 Ω | 3, 6, 10, 20 y 30 dB, ≥ 1 W, CC–6 GHz | atenuación total 40–70 dB |
| 1 | Atenuador programable | 0–31.5 dB, paso 0.5 dB, USB o paralelo (familia Mini-Circuits RUDAT/RCDAT, *verificar*) | variar el nivel con el pase |
| 4 | Cargas de 50 Ω, adaptadores y cables | SMA, RG-316 o semi-rígido, doble blindaje | comprobación de fuga (§2.7) |
| 2 | Cajas metálicas con conectores SMA pasantes | aluminio | aislamiento TX/RX ≥ 85 dB (§2.7) |
| 1 | Acoplador direccional o divisor | 20–30 dB, 50 Ω, cubre 137–2200 MHz | calibración del nivel |
| 1 | Analizador de espectro económico | hasta 6 GHz (p. ej. tinySA Ultra) | verificar fugas y el nivel |
| 4–6 | Ferritas de abrazadera | para cables USB y de potencia | aislamiento |

**Arquitectura de la fase 2 con re-sintonía del Doppler predicho** (diseño; el nodo **no** está escrito: es de `atp_sdr`/`atp_hw`, fuera del área de este equipo):

    efeméride (TLE/SGP4: orbit.observar → vr)  ──▶  fd_pred = −vr/c · f0  (model.doppler_hz)
         │                                          cada Δt ≤ tolerancia/tasa  (UHF 140 Hz/s: 1 s ⇒ ≤ 140 Hz)
         ▼
    nodo `doppler_tuner` (ROS 2)  ──▶ ① SoapySDR setFrequency(f0 + fd_pred)         (RTL-SDR, HackRF, Pluto)
                                  └─▶ ② `F <Hz>` por rigctl/rigctld a gqrx o a un SDR con modelo de Hamlib
         │
         ▼
    SDR  ──IQ──▶ atp_sdr (adquisición FFT + FLL/PLL: el residuo y el oscilador) ──▶ /atp/sdr (SdrStatus)

El PLL de `atp_sdr` absorbe el residuo (rampa de ≤ 140 Hz/s entre órdenes y la incertidumbre del TCXO); el **Doppler residual** es la medida que cierra el lazo de apuntamiento por seguimiento de baliza (autotrack). Rutas de código existentes: `atp_ros.orbit.observar`, `atp_ros.model.doppler_hz`, `atp_sdr.cadena.CadenaRF.paso(fd_pred_hz=…)`. Compuerta de lockstep (CONTRATO §8): el nodo sin lockstep sella con el reloj del SDR/PPS.

### Fase 3: rotor comercial vía Hamlib o montura propia

| Opción | Cant. | Ítem y especificación | Por qué |
|---|---|---|---|
| **A. Rotor comercial** | 1 | Yaesu **G-5500** (acimut 0–450°, elevación 0–180°, ~6°/s y ~3°/s, *verificar la hoja de Yaesu*) + interfaz de PC (GS-232B), o SPID Rot2Prog | `atp_hw` ya habla con él por `rotctld` (modelo 603/901; [atp_hw README](../src/atp_hw/README.md)); suficiente para una **Yagi** (§2.2: pierde el 3.2 % de los pases a 550 km; el haz de 51° tolera cualquier error de la tabla) |
| | 1 | Mástil, soportes, cable de control apantallado de ≥ 7 hilos, **contactor y paro NC** aguas arriba de la alimentación del rotor | el paro de `rotctld` (`/atp/rotor/halt`) pasa por la red y **no** es de seguridad |
| **B. Montura propia con el ESP32** | 2 | Servomotor de **150–300 W, 24 V** (CC con escobillas o BLDC con driver de servo), reductora total ≈ 600:1 (sinfín + planetaria), **retención sin energía** | §2.5 (plato 3 m: 227 N·m, 140 W, 6.5 A a 24 V) |
| | 2 | Driver: BTS7960 si V_bus ≤ ~27 V (*verificar*), si no un servoamplificador con lazo de corriente (planta distinta: CONTROL §8) | corriente pico 9 A a 24 V con 60°/s² |
| | 1 | Fuente de 24 V ≥ 10 A, con fusible y TVS | §2.5 |
| | 2 | Encoder de **carga** absoluto ≥ 14 bits (AS5048A: soportado por el firmware, *SIN PROBAR*; INL a verificar) o óptico ≥ 17 bits (requiere código nuevo SSI/BiSS) o incremental de 10 000 PPR (40 000 cuentas/vuelta con PCNT) | §2.4: 12 bits mínimo, 14 recomendado, 17+ para absoluto |
| | 4 | Fines de carrera de **potencia** (2 por eje), un final de carrera de **referencia** (homing; hoy manual, firmware §9 punto 4) | |
| | 1 | Inclinómetro digital 0.01°, brújula/GNSS de doble antena, gnomon | |
| | 1 | Anemómetro | resguardo (stow) cuando el viento supere el diseño (§2.5) |

## 6. Seguridad

* **Paro de emergencia = contacto NC** y, además del paro por software del firmware (ISR en µs), un **contactor/relé** que corta la **potencia del motor sin pasar por el ESP32**: el firmware **no** está certificado como función de seguridad ([firmware](../firmware/esp32_eje/README.md) §3, §5). Con 2 ejes, **un solo paro corta los dos puentes** (ISR común), y aun así hay que cortar la potencia por contactor.
* **Fusibles** al 1.5–2× de la corriente nominal del bus, **TVS** o resistencia de frenado si el motor regenera, condensador de bulto cerca del puente, **fines de carrera cableados a la potencia**.
* **Niveles de 3.3 V**: los GPIO del ESP32 **no** toleran 5 V (hoja de datos de Espressif); encoders de 5 V o RS-422 con divisor, conversor o receptor diferencial; comprobar con el multímetro **antes** de conectar.
* **EN con pull-down externo de 10 kΩ en cada eje**; masa común en estrella; alimentación del motor separada de la lógica; cables del encoder trenzados/apantallados lejos del cable del motor.
* **RF**: nunca sobrepasar la potencia máxima de entrada del receptor (§2.7: ≤ −30 dBm en operación; atenuador de 10 dB siempre junto al receptor); cargar con 50 Ω las salidas sin uso; **no radiar** en bandas de radioaficionado sin licencia (el camino conducido no radia: con cajas y cables blindados es la forma de transmitir en el escritorio).
* **Primer encendido**: motor **desacoplado**, fuente al 10 % de la corriente nominal, y las cinco pruebas del paro tres veces cada una. Un plato real: resguardo por viento y freno.

## 7. Qué prueba exige cada compra

Cada compra de §6 habilita una prueba de banco o de campo: paros y fines de carrera (pulsador NC, contactor, osciloscopio), sentido de giro y cuentas por vuelta (motorreductor con encoder), referenciado (fin de carrera o AS5048A), identificación de J y b (pinza amperimétrica), watchdog y jitter (agente serie, osciloscopio), reloj (GNSS con PPS) y nivelación (inclinómetro de 0.01°). Los criterios de aceptación son propuestas razonadas, no cifras medidas.

## 8. Lo no verificado (resumen)

* **Todo lo que toca la placa**: PCNT, LEDC, ISR del paro, GPIO4 en el reinicio, ADC, UART a 921 600 y tiempos reales del tick (la tabla de §3.2 es una estimación por tamaño de código; solo hay medición **en el host**, no en el chip).
* Los **supuestos** de diseño: inercias (0.05 / 3 / 110 kg·m²), fricción y desbalance, área de la Yagi, motor de 3 000 rpm, rendimientos 0.85/0.9, margen 1.5, reparto RSS del presupuesto, EIRP de 0 dBW, T_sys 300 K, NF de 5 dB, 20 dB de ruido inyectado sobre el térmico y la potencia de salida del SDR transmisor.
* Datos **de memoria** que hay que contrastar con la hoja: INL del AS5048A, tensión y corriente del BTS7960, potencia máxima de entrada del RTL-SDR y TCXO de 1 ppm, potencias de salida del HackRF/Pluto, pasos de los atenuadores programables, las velocidades y el rango de elevación del G-5500, el par de retención de un NEMA 17, las pérdidas del RG-58.
* **No se escribió** la variante de paso a paso (solo diseño, §3.3), el soporte SSI/BiSS, ni el nodo `doppler_tuner` (diseño, §5 fase 2). El HIL mide la latencia **desde el receptor de la referencia en el puente** (callbacks de Python, ±ms) y la deriva contra el gemelo con las limitaciones de §4 y del README §7c.
