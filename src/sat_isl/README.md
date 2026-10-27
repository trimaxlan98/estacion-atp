# sat_isl — ATP óptico entre satélites (ISL) en una constelación

Enlaces láser intersatelitales (*inter-satellite links*, ISL) en una constelación Walker: geometría de los pares
vecinos, terminal óptico con arquitectura **gruesa/fina** (cardan + espejo de apuntamiento fino, FSM), **adquisición**
por espiral con baliza, **seguimiento** con sensor de cuadrante (QPD), presupuesto de enlace óptico (APD o coherente) y
un escenario en **lockstep** (CONTRATO sec. 8): búsqueda → apretón → seguimiento → enlace, con un `RESUMEN` determinista.

Es la contraparte de `atp_ros` (ATP de RF desde tierra, haz de grados) con otro régimen: haz de **decenas de µrad**,
**los dos extremos se mueven** (adelanto de apuntamiento `2 v⊥/c`), sin atmósfera pero con la Tierra y su borde en
medio, y adquisición **mutua** (cada terminal busca al otro con una baliza ancha). Ver curso 30 §5 y curso 18
(`isl.py`, módulo de ATP óptico): los números del curso (106.1 dBi, 272.2 dB, 50.7 µrad…) están como pruebas.

Paquetes: `src/sat_isl` (este) e `src/isl_interfaces` (mensajes `IslState`, `IslCmd`). Importa (sin modificar)
`atp_ros.propagador` (órbita) y `sat_adcs.fisica` (inercia del satélite 3U para la línea de microvibración de las
ruedas); no toca `atp_ros` ni `sat_adcs`.

## Uso

```bash
docker compose build                               # una vez
./run.sh bash scripts/validar_isl.sh               # pruebas + escenarios ROS + determinismo + tablas
./run.sh ros2 launch sat_isl isl.launch.py time_scale:=0                       # par del mismo plano (P0S0-P0S1)
./run.sh ros2 launch sat_isl isl.launch.py time_scale:=0 sat_b:=18 start_s:=1020 dur_s:=60   # otro plano: se corta la visibilidad
./run.sh ros2 launch sat_isl isl.launch.py time_scale:=0 detector:=apd         # APD de detección directa
./run.sh python3 -m sat_isl.escenario [--tabla]    # el mismo escenario sin ROS (o la tabla de vecinos)
./run.sh python3 scripts/isl_estudio.py todo       # las tablas de este README (geometria|haz|enlace|adquisicion|lazo|escenario)
./run.sh python3 -m pytest -q src/sat_isl/test     # 102 pruebas, ~75 s
```

Argumentos del launch: `n_sats`/`planos`/`fase`/`h_km`/`inc_deg` (Walker, 24/3/1 a 800 km y 55° por omisión),
`sat_a`/`sat_b` (índices plano-mayor, 8 por plano: 0 = P0S0, 1 = P0S1, 18 = P2S2), `start_s` y `dur_s` (segundo de la
propagación y duración), `step_s` (lockstep, 0.05 s), `seed`, `detector` (`coherente`|`apd`), `d_m`, `p_tx_w`,
`theta_baliza_urad`, `sigma_pos_km` (error de efemérides que **cree** el controlador, 2.1 km = TLE de un día) y
`sigma_actitud_urad`. El resto de `sat_isl.misiones.Config` se fija con `--ros-args -p` en cada nodo.

## Lockstep (CONTRATO sec. 8)

```
/clock t_k ─▶ isl_sim integra [t_{k-1}, t_k] con el mando vigente (ZOH) ─▶ /isl/state (sello t_k)
           ─▶ isl_controller: máquina de estados + efemérides estimadas ─▶ /isl/cmd (MISMO sello t_k)
           ─▶ isl_sim lo aplica y confirma en /atp/step_ack (Clock = t_k) ─▶ sim_clock publica t_{k+1}
```

| Nodo | Hace | Reglas del contrato |
|---|---|---|
| `isl_sim` | planta: órbitas, cardan, cámaras, QPD, lazos de 10 kHz, enlace; publica `/isl/done` (latched) | sella con el instante del cálculo; confirma solo el mando del mismo sello (uno rancio se ignora); no arranca hasta tener `wait_subscribers` y fija el inicio con el sello del primer mando (nada depende del descubrimiento DDS) |
| `isl_controller` | efemérides y actitud **estimadas** (con error sembrado), modos, barrido, tasa; solo ve las medidas del mensaje | responde a **cada** estado con el mismo sello; un estado repetido (arranque) recibe el mismo mando; espera `wait_subscribers` |
| `isl_monitor` | mide con la verdad e imprime el RESUMEN | pasivo: colas de 400, empareja por **sello**, cierra con un temporizador de 3 s simulados tras `/isl/done` |

Los nodos usan `atp_ros.nodo.correr` (saludo de arranque). Los lazos finos a 10 kHz corren **dentro** de la planta
(en un terminal real son electrónica/FPGA); el controlador es la supervisión a 20 Hz. Con `sim_clock` en lockstep la
misma corrida da las mismas cifras: `validar_isl.sh det` corre ROS dos veces y las compara con la corrida sin ROS.

## Modelo (cada cifra dice dónde vale)

**1. Geometría (`geometria.py`).** Walker delta T/P/F (Walker 1984) propagada con `atp_ros.propagador` (gravedad central +
J2, DP45, efeméride de Hermite quíntica de 30 s; sin arrastre). Por par: línea de vista, rango, velocidad relativa,
giro de la línea de vista `v⊥/ρ`, tiempo de luz, Doppler óptico `v/λ`, **adelanto** `2 v⊥/c` y visibilidad.

* *Adelanto.* El receptor se mueve `v⊥ t` mientras la luz va y vuelve: el haz debe salir `2 v⊥/c` adelantado respecto
  del haz recibido (el QPD ve la dirección **aparente** `u − v⊥/c`; se transmite a `u + v⊥/c`). `adelanto_exacto_rad`
  lo recalcula con tiempos de luz retardados y aberración de primer orden y coincide con la fórmula a 1e-3 relativo.
  Consistente con la aberración de un satélite medida desde tierra: Kuzkov et al. (arXiv:1711.10584) miden en ARTEMIS
  (3.07 km/s) `V/c` = 2.115″ = 10.2 µrad (un sentido; el viaje de ida y vuelta lo duplica).
* *Visibilidad.* La Tierra (esfera de 6378.137 km: conservador, la real es 21 km más baja en los polos) tapa el rayo si
  su **altura mínima < 100 km**. Justificación calculada (`refraccion_urad`, atmósfera de Vallado + refractividad
  `n−1 = 2.73e-4` a 1550 nm, `α = (n_t−1) √(2π r_t/H)`, contrastada con la integral de Abel numérica a 1-3 %): el rayo se
  desvía 0.01 µrad con tangente a 100 km, **1.5 µrad a 70 km, 16 µrad a 50 km y 307 µrad a 30 km** (el haz entero son
  ~30 µrad). 100 km deja ~30 km sobre el punto donde la refracción llega a 1 µrad, para la turbulencia y los aerosoles
  que el modelo no ve.
* *Por qué no 3 × 4.* A 800 km la separación máxima en un plano con el rayo libre es 51.0° (cuerda de 6184 km); con 4
  satélites por plano (90°) el rayo pasa a −1302 km, **dentro de la Tierra**: no hay enlaces intraplano. Se usa **3 × 8**
  (45°, cuerda 5494 km). El patrón 3 × 4 solo sirve a ≳ 2800 km de altura.

**2. Terminal óptico (`terminal.py`, `enlace.py`).** Montura **gruesa** (cardan, o el propio ADCS), **FSM** de ±1 mrad
con resonancia de 2 kHz, **QPD** (APD) en el plano focal y **cámara de adquisición** de campo ancho con baliza propia.
Haz de comunicación: gaussiano truncado por la apertura, `γ = (D/2)/w`. La ganancia sobre el eje es
`G_tx = (πD/λ)² · (2/γ²) tanh(γ²/2)` (integral exacta del campo truncado; la prueba la contrasta con integración
numérica) y el producto con la fracción de potencia que cruza la apertura se maximiza en **γ = 1.12**
(Klein y Degnan 1974; la prueba lo recupera). La anchura equivalente se define con la ganancia, `θ_h = √(8/G)`:
**0.956 λ/D** de semiángulo 1/e² (el "θ ≈ λ/D" pedido: 29.6 µrad para 5 cm a 1550 nm; ángulo completo 1.91 λ/D). La
prueba contrasta esa gaussiana con el campo lejano real del haz truncado (integral de Bessel): error < 4-10 % en
intensidad hasta θ_h; la gaussiana *sin truncar* de la misma apertura (0.71 λ/D) **subestimaría** la divergencia un 25 %.
Pérdida por apuntamiento `L = exp(−2e²/θ_h²)`; con error gaussiano de σ por eje la media es `1/(1 + 4σ²/θ_h²)` (Monte Carlo
a 0.5 %). Ecuación de enlace en vacío, `P_r = P_t η_t f G_t L_t (λ/4πR)² G_r η_r L_r` (reproduce la cascada del curso 18).
Detectores: **APD** con TIA (OOK, ruido térmico + de disparo con exceso de McIntyre; `Apd`) y **BPSK homodino**
(`BER = ½ erfc(√(2 N_p/L))`, `N_p` fotones por bit, `L` penalización de implementación sobre el límite cuántico de
**9 fotones por bit** para 1e-9: `Q(z)=1e-9, N = z²/4 = 8.99`).

**3. Adquisición (`adquisicion.py`).** A barre una **espiral de Arquimedes** (puntos a longitud de arco exacta `k·paso`)
con la baliza; B mira con la cámara. La baliza es gaussiana de semiángulo `θ_b` y pico `P0`; se detecta (≥ 90 %) dentro de
`r_hit = θ_b √(½ ln(P0/P_det))` (ec. 1), con `paso = √2 r_hit`. Cada paso "posee" `paso²` de área: tras `k` pasos el radio
barrido es `R = paso √(k/π)` (ec. 2) y, con la posición real gaussiana (error de efemérides y de actitud),
**`P_acq(t) = Pd (1 − exp(−R²/2σ²))`** (ec. 3, Rayleigh): la analítica con que se contrasta el Monte Carlo. La baliza que
minimiza el tiempo tiene `P0 = e·P_det` (`θ_b* = √(A/(e P_det))`, ec. 4). La incertidumbre por eje es
`σ² = (√2 σ_pos/ρ)² + σ_actitud² + σ_cardán²`. Tras detectar, B se apunta a A (centroide), emite su baliza (retardo de luz de ida
y vuelta) y ambos cierran los QPD (`t_apreton_s`).

**4. Seguimiento (`seguimiento.py`).** Lazo **maestro-esclavo**: QPD → integrador → FSM (2.º orden con resonancia) con
retención y 1 muestra de retardo a 10 kHz; la posición del FSM la integra la montura gruesa (`H_c = (ω_d/s) G_cardán`), de modo que
el lazo es de **tipo 2** (error nulo ante la rampa de 1-4 mrad/s de la línea de vista) y el FSM queda descargado.
`e = S d − T n`, `S = 1/(1+L)`, `y = G_f (e+n)`. Es lineal: la respuesta temporal (`lfilter` con estado continuo) y el rms
analítico (integral de `|S|²·PSD`) se contrastan. La perturbación de plataforma es una suma de modos de 2.º orden
(**SUPUESTO**, sin medición publicada que la fije): deriva del ADCS 400 µrad a 0.05 Hz, banda media 12 µrad a 5 Hz, modos
estructurales de 1 y 0.4 µrad a 35 y 330 Hz, y la línea de las ruedas a 133 Hz anclada en el desbalance dinámico (`U_d/I`
con la inercia del 3U de `sat_adcs`); ≈ 400 µrad rms antes del lazo. El ruido del QPD sale de la potencia recibida
(`NEA = σ_e w/1.596`, térmico + de disparo): crece al bajar `P_r`.

**5. Enlace por paso.** Para cada muestra de 10 kHz, `P(t) = P_nom exp(−2(|e_tx,A|² + |e_rx,B|²)/θ_h²)` (reciprocidad:
mismo `θ_h` al transmitir y recibir), con `e_tx = e_seguimiento + (adelanto mandado − verdadero) + calibración`; el BER del
paso es la media de las muestras y el sentido peor manda. El controlador elige la **tasa** de una escalera (10 Mb/s-10 Gb/s)
con BER 1e-6 y 3 dB de margen sobre la potencia medida. Enlace **cerrado** = seguimiento válido (error < rango del QPD, FSM
dentro de su carrera, `NEA < θ_h/3`) y BER ≤ 1e-6.

### Supuestos y fuentes

| Cifra | Valor | Origen |
|---|---|---|
| Walker, órbita | 24/3/1, 800 km, 55°; J2 sin arrastre | patrón de Walker 1984 (de memoria); propagador de la Estación |
| Altura mínima del rayo | 100 km | **calculada** (refracción), ver arriba |
| `n−1` del aire a 1550 nm | 2.73e-4 | Ciddor/Edlén **de memoria**, ±2 % |
| γ del haz | 1.12 | Klein y Degnan 1974 (de memoria; la prueba lo reproduce) |
| Terminal del escenario | 5 cm, 1 W, 1550 nm, η_tx 0.5, η_rx 0.4 | **SUPUESTO** (clase nanosatélite/ESPA) |
| Penalización del homodino | 5 dB | **SUPUESTO**: la del Tesat LCT no es pública |
| APD | η 0.8, M 10, k 0.4, 1 nA, 5 pA/√Hz | **SUPUESTO** típico de APD InGaAs + TIA de 10G (da −33 dBm a 10 Gb/s) |
| FSM, lazo | ±1 mrad, 2 kHz, ζ 0.3, cruce 400 Hz, 10 kHz | **SUPUESTO** típico de un FSM piezo/bobina |
| Montura gruesa | 1.5 Hz de descarga, servo 30 Hz | **SUPUESTO**; el ADCS (`LAZO_ADCS`) = 0.005 Hz |
| QPD | 10 % de la potencia, w = 100 µrad, NEP 1.1e-13 W/√Hz | **SUPUESTO** |
| Cámara | 5 ms, 150 e⁻ de lectura, oscuridad 5e4 e⁻/s, **fondo 1e5 e⁻/s**, Pfa 1e-6, η 0.8 | **SUPUESTO**; el fondo real (limbo, Sol) puede ser órdenes mayor |
| Baliza | 0.5 W, θ_b 300 µrad | **SUPUESTO**; el óptimo calculado es 8.8 mrad |
| Perturbación de plataforma | 400 µrad rms (0.05 Hz) + 12 µrad (5 Hz) + modos | **SUPUESTO** de forma típica; no verificado contra medición |
| Línea de ruedas | `U_d` = 3e-8 kg m² → 0.72 µrad | **SUPUESTO**; inercia de `sat_adcs.fisica.cubesat_3u` |
| Error de efemérides | TLE 1 día 2.1 km; 3 días 11 km; OD 0.036 km | `docs/OD.md` (el 0.036 km es el σ **formal** de la covarianza: optimista); GNSS a bordo 10 m **supuesto** |
| Error de actitud | star tracker 145 µrad (30″) **supuesto**; `sat_adcs` grado 8 0.14° y grado 4 0.89° rms | README de `sat_adcs` (conocimiento rms en el eje de la antena, luz), por eje = rms/√2 |
| Tesat LCT TSX-NFIRE | 1064 nm, 125 mm, 0.7 W, BPSK homodino 5.625 Gb/s, BER < 1e-9, ~5100 km | fichas y resúmenes (Smutny et al., Proc. SPIE 7199, 2009; Gregory et al., Opt. Eng. 51(3), 2012; Kaushal y Kaddoum 2017): **leído en resúmenes, no en el artículo** |
| Tesat LCT 135 | 135 mm, 1.8 Gb/s, 45 000 km LEO-GEO, "hasta 80 000 km" | notas de prensa de Tesat; **los 2.2 W son de memoria, sin verificar** |

## Validación (`pytest src/sat_isl/test`: 102 pruebas)

| Prueba | Contra qué | Cifra |
|---|---|---|
| Walker | patrón de RAAN, fase `360F/T`, elementos de Kepler exactos | exacto |
| Propagación con J2 | radio y cuerda dentro del plano | radio ±15 km; cuerda 5494 km a 0.5 %, constante a 1 % |
| **Point-ahead** | `4 v sin(φ/2)/c` en pares coplanares (cerrada); **tiempos de luz retardados + aberración** en pares Walker propagados | 1e-12 (cerrada); **1e-3 (fuerza bruta)**; 7.6 km/s → 50.7 µrad (curso 18) |
| Doppler óptico | `v/λ` | 7.6 km/s → 4.9 GHz |
| **Visibilidad** | altura del rayo `(R+h)cos(φ/2) − R` (cuerda), muestreo bruto del segmento, casos extremos (por el centro, extremo del segmento) | 1e-6 km; el cambio de visible ocurre en 51.03° ± 0.05° y la cuerda en 6184 km |
| **Refracción (100 km)** | integral de Abel numérica del rayo | 3 % (en 40-100 km: 0.1-1.2 %) |
| Haz truncado | integral numérica de la ganancia; `γ_opt`; campo lejano de Bessel | 1e-7; **1.121**; ≤ 4-10 % hasta θ_h |
| Pérdida por apuntamiento | puntos conocidos (−3 dB a 0.589 θ_h) y Monte Carlo del jitter gaussiano | 0.5 % |
| Enlace | cascada del curso 18 (106.1 dBi, 272.2 dB, −29.9 dBm); cascada en dB a mano | 1e-9 dB |
| Detectores | límite cuántico del homodino (**8.99** fotones/bit para 1e-9); el APD escala con √tasa; coherente gana > 20 dB a 10 Mb/s | |
| **Presupuesto contra el LCT publicado** | TSX-NFIRE a 5100 km y 5.625 Gb/s: margen +13 dB (rango máximo 24 000 km > 5500 demostrados); LCT 135 a 45 000 km y 1.8 Gb/s: +5.8 dB, rango máximo **87 000 km** (Tesat publica "hasta 80 000 km") | plausibilidad, **no** validación fina: las pérdidas y la penalización son supuestos |
| **Lazo FSM** | estable; cruce 400 Hz; **PM 68.5°, GM 7.5 dB, pico de \|S\| 1.73 (4.8 dB)**; BW de T (−3 dB) 1455 Hz; asíntota de tipo 2 `s²/(Ki(s+ω_d))` a 0.1-0.5 Hz; tono de 20 Hz = `\|S\|`; rampa de 4 mrad/s → error 0; escalón sin sobreimpulso > 40 % | 0.07-2 % |
| Control negativo | un lazo con 6 muestras de retardo y cruce a 900 Hz se detecta inestable | |
| **Rechazo de perturbaciones** | rms analítico (`∫\|S\|² PSD`) contra 60 s simulados | 0.4206 µrad contra 0.4218 µrad (0.3 %); FSM sin la deriva lenta a 6 % |
| QPD | `erf(√2 x/w)` contra la integral de la mancha; pendiente 1.596/w; NEA ∝ 1/P (térmico) y 1/√P (disparo) | 1e-5 |
| Carrera del FSM | la deriva lenta con montura de 1.5 Hz pide 17 µrad rms (cabe); con el ADCS (0.005 Hz) 417 µrad (**no cabe en ±1 mrad**) | |
| **Adquisición** | espiral: radio `paso√(k/π)` a 4 %, arco exacto, sin hueco central (regresión); cobertura 99.95 % con `√2·r_hit`; **Monte Carlo (4000) contra la ec. 3** | ≤ 0.03 (σ = 4.8 pasos), ≤ 0.015 (9.6 pasos); duplicar σ multiplica la mediana del tiempo por 4.2 (esperado 4) |
| Baliza óptima | `P0 = e·P_det` y máximo del área barrida | exacto |
| Relación con el error de órbita | mediana(OD 36 m) ≤ mediana(TLE 1 d) < mediana(TLE 3 d) | ×5 o más |
| **Planta completa contra el Monte Carlo puro** | 150 semillas (400 en el estudio): cuartiles del tiempo hasta el apretón | mediana 1.05 s contra 0.95 s, p90 3.40 contra 3.25 s |
| Escenario | adquisición < 1 s, error p95 < 0.1 θ_h, cerrado > 95 %; el controlador **no usa la verdad** (mismo mando con solo los sensores); corte por visibilidad; APD no cierra donde el coherente sí; enlace débil pierde y reintenta | |
| **Determinismo** | misma configuración → mismo `RESUMEN`; otra semilla → distinto; **ROS dos veces = sin ROS** | idéntico |

Un hallazgo del propio Monte Carlo del escenario: la primera espiral avanzaba el ángulo con el radio del paso anterior, saltaba
la vuelta interior y dejaba un anillo sin cubrir alrededor del centro (2 de 8 semillas no adquirían nunca). Está corregido
(arco exacto) y con prueba de regresión.

## Resultados (`scripts/isl_estudio.py`, todo calculado)

**Vecinos de P0S0** (Walker 24/3/1, 800 km, 55°, una órbita): los del plano son un enlace casi estático; los de otros planos son
rápidos y solo se ven un tercio de la órbita.

| par | rango (km) | visible (%) | v⊥ máx (km/s) | giro de la línea de vista máx (mrad/s) | adelanto máx (µrad) |
|---|---|---|---|---|---|
| P0S0 - P0S1 | 5436-5494 | 100 | 5.70 | 1.04 | 38.0 |
| P0S0 - P0S7 | 5490-5548 | 100 | 5.76 | 1.04 | 38.4 |
| P0S0 - P1S5 | 2673-10528 | 35 | 10.94 | 4.09 | 73.0 |
| P0S0 - P2S2 | 2594-10513 | 43 | 10.92 | 4.20 | 72.9 |

**Haz de comunicación a 1550 nm** (γ = 1.12): el costo del láser es la puntería.

| D (cm) | G_tx (dBi) | semiángulo 1/e² (µrad) | ángulo completo (µrad) | pérdida con 1 µrad (dB) | error para 3 dB (µrad) |
|---|---|---|---|---|---|
| 3 | 95.2 | 49.4 | 98.8 | 0.004 | 29.1 |
| 5 | 99.6 | 29.6 | 59.3 | 0.010 | 17.4 |
| 8 | 103.7 | 18.5 | 37.0 | 0.025 | 10.9 |
| 10 | 105.6 | 14.8 | 29.6 | 0.040 | 8.7 |
| 13.5 | 108.2 | 11.0 | 22.0 | 0.072 | 6.5 |

**Presupuesto de enlace** (terminal del escenario, BER 1e-6, 3 dB de margen): el coherente cierra Gb/s donde el APD llega a
decenas de Mb/s o no cierra.

| rango (km) | Pr (dBm) | fotones/bit a 1 Gb/s | tasa coherente (Mb/s) | tasa APD (Mb/s) | sens. coherente 1 Gb/s (dBm) | sens. APD 1 Gb/s (dBm) |
|---|---|---|---|---|---|---|
| 2600 | −44.1 | 302 | 5000 | 50 | −56.4 | −41.2 |
| 5500 | −50.6 | 67 | 1250 | 0 | −56.4 | −41.2 |
| 10500 | −56.3 | 19 | 250 | 0 | −56.4 | −41.2 |

**Lazo fino** (cruce 400 Hz, PM 68.5°, GM 7.5 dB a 1402 Hz, pico de |S| 4.8 dB, BW de T 1455 Hz, Ki = 2430 s⁻¹):

| f (Hz) | 0.1 | 1 | 5 | 35 | 133 | 330 | 1000 |
|---|---|---|---|---|---|---|---|
| \|S\| (dB) | −95.3 | −56.7 | −37.5 | −20.7 | −9.4 | −2.3 | +3.8 |

| componente de la plataforma (SUPUESTO) | f (Hz) | rms entrada (µrad) | rms residual (µrad) | rms posición del FSM (µrad) |
|---|---|---|---|---|
| deriva ADCS | 0.05 | 400.0 | 0.011 | 13.1 |
| banda media | 5 | 12.0 | 0.160 | 11.4 |
| estructura | 35 | 1.0 | 0.092 | 1.0 |
| rueda | 133 | 0.51 | 0.170 | 0.50 |
| estructura | 330 | 0.40 | 0.304 | 0.37 |
| **total** | | **400.2** | **0.394** | **17.4** |

Sin el lazo fino y solo con la parte rápida de la perturbación (12.1 µrad por eje) la pérdida media por apuntamiento sería
**2.2 dB por extremo**; con el lazo (0.48 µrad con el ruido del QPD) es **0.005 dB**. Montura gruesa y carrera del FSM:
con el cardán (1.5 Hz) el FSM necesita 69 µrad (4σ) de ±1000; **con el propio ADCS como montura (0.005 Hz) necesita 1668 µrad
y no cabe**: o un FSM de mayor carrera, o un cardán, o una deriva de actitud mucho menor que los 400 µrad supuestos.

**Adquisición** (rango 5500 km, baliza 0.5 W/300 µrad; mediana y p95 en s; con una espiral fuera del recorrido del FSM
el paso es el del cardán, 50 ms): la cámara detecta (≥ 90 %) hasta 591 µrad del eje del haz de 300 µrad.

| actitud | efemérides | σ (µrad) | pasos | mediana (s) | p95 (s) | P(<1 s) % | P(<10 s) % |
|---|---|---|---|---|---|---|---|
| star tracker 145 µrad | GNSS 10 m / OD 36 m | 153 | 3 | 0.01 | 0.01 | 100 | 100 |
| star tracker 145 µrad | TLE 1 día (2.1 km) | 561 | 19 | 0.10 | 0.50 | 100 | 100 |
| star tracker 145 µrad | TLE 3 días (11 km) | 2833 | 444 | 2.35 | 10.9 | 27 | 93 |
| `sat_adcs` grado 8 (0.14°) | TLE 1 día | 1811 | 182 | 1.00 | 4.4 | 51 | 100 |
| `sat_adcs` grado 4 (0.89°) | cualquiera | ~11000 | ~6700 | 36 | 166 | 1 | 18 |

Con la actitud de un ADCS de magnetómetro y sensor solar manda la actitud, no la órbita (la OD de la Estación no ayuda); con un
star tracker manda la órbita: de la OD (36 m) al TLE de 3 días el tiempo pasa de 0.01 s a 2.4 s (mediana) y 11 s (p95). Una baliza **óptima**
(8.8 mrad, `P0 = e·P_det`: poco margen de detección, que es lo que se paga) baja todos los casos a ≲ 1.5 s (p95); el de grado 4 de 166 s a 1.5 s.

**Escenario completo** (lockstep sin ROS, 8 semillas, 20 s; mediana (mín-máx)):

| caso | t adquisición (s) | error rms (µrad) | error p95 (µrad) | cerrado (% visible) | tasa |
|---|---|---|---|---|---|
| par del plano, TLE de 1 día, star tracker | 0.13 (0.10-0.45) | 0.683 (0.680-0.687) | 1.183 (1.173-1.188) | 99.1 (97.5-99.3) | 1.25 Gb/s |
| par de planos (2.7-3 Mm), TLE 1 día | 0.42 (0.10-1.80) | 0.567 (0.563-0.572) | 0.983 (0.974-0.990) | 97.6 (90.8-99.3) | 5 Gb/s |
| par del plano, TLE de 3 días | 1.95 (0.15-8.40) | 0.683 | 1.183 | 90.0 (57.9-99.0) | 1.25 Gb/s |
| par del plano, actitud `sat_adcs` grado 8 | 2.37 (0.10-6.65) | 0.682 | 1.179 | 96.0 (88.8-99.8) | 1.25 Gb/s |
| par del plano, detector APD | 0.13 | 0.683 | 1.183 | **0** | 0 (sigue sin enlace de datos) |

El error de seguimiento no depende de cómo se adquirió (lo fija el lazo y el ruido del QPD); lo que cambia es cuánto tiempo se
pierde buscando.

**RESUMEN de la corrida ROS** (`isl.launch.py time_scale:=0 dur_s:=60`, par del mismo plano, semilla 7; idéntico al de
`python3 -m sat_isl.escenario --dur_s 60`):

```
RESUMEN enlace BPSK-homodino 5 cm 1550 nm 1.0 W: rango 5492-5492 km, adelanto 38.0 urad, semiangulo del haz 29.6 urad, visible 100 % de 60 s
RESUMEN adquisicion: 0.45 s desde el inicio de la busqueda (t=0.05 s) hasta el seguimiento; 8 pasos de barrido, 0 barridos completos
RESUMEN seguimiento fino (tras 1 s): error radial rms 0.681 urad, p95 1.180 urad, max 2.288 urad (1171 pasos) | FSM |y| max 66.1 urad de 1000
RESUMEN enlace: perdida por apuntamiento media 0.020 dB (max 0.029) | Pr media -50.6 dBm (min -50.7) | BER max 8.69e-17
RESUMEN disponibilidad: enlace cerrado 99.2 % del tiempo visible (99.2 % de la corrida) | tasa 1250.0 Mb/s con el enlace cerrado, media 1239.6 Mb/s | 9.305 GB | perdidas de seguimiento 0, cortes por visibilidad 0
```

Y con el par de otro plano que sale de la vista (`sat_b:=18 start_s:=1020 dur_s:=60`; la ventana termina hacia t = 1063 s, el
controlador apaga el enlace a t = 42.45 s con **su** estimación de visibilidad, 0.35 s antes que la verdad (42.8 s) por el error de
efemérides de 2.1 km): adquisición 0.15 s, error rms 0.727 µrad (p95 1.258), enlace cerrado 98.8 % del tiempo visible (70.4 % de la
corrida), 1 corte por visibilidad.

El error fino es el **radial** (los dos ejes) del eje de recepción contra la dirección aparente del otro terminal, ambos terminales juntos,
tras 1 s del cierre del lazo (muestras de 1 kHz para el p95, todas las de 10 kHz para el rms).

## Limitaciones (lo no verificado y lo no modelado)

* **Las perturbaciones de plataforma son un supuesto** de forma típica; no hay medición publicada que las fije. El resultado
  fino (0.4-0.7 µrad) es tan bueno como esa PSD; lo que sí es robusto es la estructura (el lazo de tipo 2 con descarga, el
  rechazo `|S|` y su relación con la carrera del FSM).
* **Terminales publicados**: las cifras del Tesat LCT salen de fichas y resúmenes, no de los artículos completos. Las
  pérdidas ópticas, la penalización del receptor y los 2.2 W del LCT 135 son supuestos: el contraste es de **plausibilidad**
  (mismo orden de rango y de margen), no una validación fina del presupuesto.
* **ADCS**: no se co-simula la dinámica de `sat_adcs`. Entra por (1) la inercia del 3U para la línea de las ruedas, (2) sus errores
  de conocimiento como incertidumbre de adquisición (rms en el eje de la antena convertido a por-eje con /√2: aproximación) y (3) su
  ancho de banda como montura gruesa, analizado solo en baja frecuencia (la carrera del FSM), no en el tiempo.
* **Cámara de adquisición**: ruido, umbral y fondo son supuestos; **no hay Sol ni limbo terrestre en el campo** (en un enlace real
  son una limitación seria: ceguera o falsas alarmas).
* **Lazo**: lineal. La **saturación del FSM** no se simula, se vigila (`max|y|` contra la carrera) y se declara pérdida de enganche.
  Sin fricción, holgura ni dinámica fina del cardán; el *feed-forward* de efemérides para la rampa de la línea de vista es perfecto.
* **Receptor**: BER analítico (APD gaussiano; homodino ideal con penalización fija), sin FEC, sin entramado ni fluctuaciones de
  intensidad del láser; la tasa se elige de una escalera discreta.
* **Constelación**: órbitas circulares con J2, sin arrastre ni eclipses, Tierra esférica; un par a la vez (sin enrutado,
  traspaso ni evitación del Sol); terminales idénticos con roles fijos (A barre, B mira); tras perder el enganche se reinicia
  la búsqueda desde cero.
* El lockstep cuantiza la adquisición a pasos de 0.05 s (la planta completa da ~0.1 s más que el Monte Carlo puro).
* Todo se probó en el contenedor Docker del repositorio; el lazo de 10 kHz es simulación, **no** hay hardware.
