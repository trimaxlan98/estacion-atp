# Determinación de órbita con las mediciones de la estación (equipo N)

Qué responde: **¿cuánto mejora el apuntamiento predicho si la estación usa sus propias
mediciones (ángulos de la montura, rango, Doppler) para determinar la órbita, en vez de
fiarse de un TLE que envejece 1-3 km/día?**, y, para el autotrack: **¿cuándo deja de hacer
falta para *adquirir*?** Código: `src/atp_ros/atp_ros/{propagador,medidas,od,od_experimento,od_estimator}.py`
(numpy, sin ROS salvo el nodo; la parte central **no** depende de scipy), pruebas
`src/atp_ros/test/{test_propagador,test_od}.py`, experimento `scripts/experimento_od.py`,
cifras en `docs/od_resultados_iss.json`.

```
TLE ──SGP4──▶ a priori (envejecido N días)          verdad = propagador numérico (J4 + arrastre real)
                │                                          │
 mediciones ◀───┴── az/el (montura tras autotrack) ◀───────┘  + ruido
 selladas          rango, Doppler (receptor)
     │
     ▼  ajuste por lotes (Levenberg-Marquardt) / EKF  +  prior del TLE
 órbita estimada x(t0), P  ──propagador numérico──▶ pases siguientes ──▶ reference_generator (profile:=od)
```

## 1. Modelos

### 1.1 Propagador (`propagador.py`)
* **Marco: TEME de la época** (el de SGP4 y el de `orbit.py`), tratado como inercial. Decisión
  documentada: no hace falta ninguna rotación para comparar con SGP4 ni para pasar a
  ECEF/ENU. Lo que se desprecia: TEME rota respecto de J2000 ~50 arcsec/año (precesión y
  nutación): fuerza ficticia de Coriolis ~1e-10 km/s² (frente a 8e-3), que equivale a girar
  el plano orbital ~15 m en 3 días. El eje de J2 es el z de TEME.
* **Fuerzas**: gravedad central + zonales J2, J3, J4 (a partir del potencial zonal,
  Vallado 2013 Eq. 8-30/8-31; la prueba contrasta J2 con la forma explícita de Vallado y J2-J4
  con diferencias finitas del potencial) + **arrastre** `a = -½ (Cd A/m) ρ |v_rel| v_rel` con
  la atmósfera co-rotando. Constantes: las de `sgp4.earth_gravity` (WGS72 para comparar con
  un TLE, WGS84 por defecto; las pruebas las cotejan con la librería).
* **Atmósfera**: exponencial por capas ρ = ρ₀ exp(-(h-h₀)/H), Vallado y McClain (2013),
  Tabla 8-4 (28 capas, 0-1000 km; basada en CIRA-72). **Transcrita de memoria del libro**; la
  prueba comprueba la continuidad entre capas (una cifra mal copiada la rompe) y la contrasta con
  la US Standard Atmosphere 1976 (200-600 km, a mejor de 40 %: es una atmósfera *media*; a 400 km
  la real varía de 1e-12 a 1e-11 kg/m³ según el Sol). Altitud: primer orden en el aplanamiento.
* **Integradores**: RK4 de paso fijo y **Dormand-Prince 5(4)** adaptativo (FSAL, tolerancia
  mixta). Medido: RK4 h = 10 s conserva la energía a 5e-10 relativo en un día (central), DP45
  rtol 1e-11 a 1e-10; el orden de RK4 sale 16x por cada h/2 (orden 4); DP45 por defecto
  (rtol 1e-9) coincide con RK4 h = 2 s a < 1 cm en 6 h.
* **STM**: ecuaciones variacionales Φ' = AΦ con el **jacobiano analítico** de las tres fuerzas
  (incluye ∂ρ/∂r por la altitud y ∂a/∂p del factor de arrastre `p`). Se contrasta con el
  **paso complejo** (a precisión de máquina, 1e-15) y con diferencias finitas del flujo
  (1e-6 relativo en 100 min), y cumple la propiedad de semigrupo y det Φ = 1 (sin arrastre).
* **Validación física**: la deriva secular del nodo y del perigeo por J2 (órbita a = 8000 km,
  e = 0.1, i = 40°, 2 días) coincide con las fórmulas de Vallado (Eq. 9-41/9-42) a **0.12 %
  (nodo) y 0.18 % (perigeo)**; la inclinación heliosíncrona a 800 km da +0.9856°/día; el
  decaimiento de una órbita polar circular por arrastre coincide con `da/dt = -Bρ√(μa)` (con ρ
  promediada sobre la órbita) a 3 %; la atmósfera co-rotante frena menos al progrado.

### 1.2 Comparación con SGP4 (ISS, 3 días) y por qué difieren
Partiendo del estado SGP4 de la época del TLE (2026-10-01 19:41 UTC):

| Modelo del propagador | 1 h | 12 h | 24 h | 72 h |
|---|---|---|---|---|
| J2, sin arrastre | 0.04 km | 0.6 km | 2.9 km | 20 km |
| J2 + arrastre físico (Cd A/m = 0.0105) | 0.03 | 1.3 | 4.7 | 48 |
| J4 + arrastre físico | 0.09 | 0.9 | 4.6 | 48 |
| J4 + arrastre con BC *efectivo* (p = 0.406) | 0.10 | 0.25 | 0.16 | 7.9 |

La diferencia es **casi toda a lo largo de la pista** (> 90 % a 72 h) y crece con el cuadrado
del tiempo: un error de semieje δa da una deriva de fase 1.5 n δa t (ISS: 7 m de δa ≈ 1 km/día).
Causas: (i) SGP4 propaga elementos *medios* (Brouwer) con su propio arrastre (B* del TLE y
atmósfera de ley de potencias), y el estado osculante que entrega en la época, integrado con otro
modelo de arrastre, cambia unos metros el semieje medio; (ii) J3/J4 y los periódicos de J2 que
SGP4 trata analíticamente aportan ~0.3 km en 3 días; (iii) lo dominante es el arrastre: con el
Cd·A/m físico de la ISS (0.0105 m²/kg) la órbita decae demasiado y el error a 24 h es 4.6 km. Se
**calibra el factor `p` del BC** contra SGP4 en 24 h (sección áurea sobre el rms): `p = 0.406`
(BC efectivo 0.0043 m²/kg), rms 0.17 km en 24 h. No es un parámetro físico: absorbe que la
atmósfera de SGP4 y la de Vallado difieren.

### 1.3 Medidas (`medidas.py`)
`h(x)` = acimut, elevación, rango y tasa de rango desde la estación (WGS84), con la cadena
TEME → R3(GMST) → ECEF → ENU de `orbit.py` (**coincide con `orbit.observar` a 1e-13**) y su
**jacobiano analítico** (contrastado con diferencias finitas en `test_od.py`, 1e-6 relativo; no
vale en el cenit, donde el acimut no está definido: el nodo descarta el_max > 85°).
Doppler: `-f0 rr / c`. Se desprecia: tiempo de luz (2-7 ms ≈ 15-50 m, hasta 0.003° a 1000 km;
consistente entre verdad y modelo pero un sesgo en datos reales), aberración, **refracción**
(0.17° a 5° de elevación: corregir antes de ingerir ángulos reales), retardos ionosférico y
troposférico en el rango, deriva del oscilador en el Doppler, movimiento del polo y UT1-UTC.

**Sigmas por defecto (1σ) y su justificación** (supuestos de la casa, no medidas de una estación):

| Canal | σ | Fuente |
|---|---|---|
| az, el | 0.01° | tras el autotrack el apuntamiento da 0.004° rms (N5); el encoder de 14 bits (C2) cuantiza 0.022° → σ = LSB/√12 = 0.0063°; en cuadratura 0.0075°, redondeado a 0.01° |
| rango | 10 m | el límite térmico de un rango por tonos/PN (CRLB, C/N0 70 dB-Hz) es de centímetros; manda la calibración del retardo y la ionosfera residual (~2 m en S, cenit) |
| Doppler | 0.05 m/s (0.37 Hz en S) | un GPSDO de 1e-10 da 0.22 Hz = 0.03 m/s, más la ionosfera residual; el límite térmico del PLL (mHz) queda muy por debajo |

**El receptor de rango NO existe en la estación todavía**: las filas "con rango" suponen un
transpondedor con *ranging* (estándar en TT&C, no en un CubeSat amateur). El Doppler lo da la
cadena RF del equipo M.

## 2. Determinación de órbita (`od.py`)
* **Lotes**: mínimos cuadrados ponderados del estado (r, v) en la época del primer dato (y,
  opcional, el factor `p` del arrastre) con **Gauss-Newton escalado + Levenberg-Marquardt** y
  prior opcional; ajusta con la STM (integración DP45 rtol 1e-8). Reporta **covarianza formal**
  (inversa de la información), valores singulares de la información normalizada, residuos
  normalizados por canal y χ²/dof.
* **EKF** secuencial (actualización escalar por canal, forma de Joseph e iterada 3 veces, ruido de proceso de
  aceleración blanca q). El valor "físico" q = σ_a² τ = 3e-17 km²/s³ (σ_a = 7.5e-11 km/s², el 25 % del arrastre de la ISS;
  τ = 5600 s) dio un EKF 4 veces peor que los lotes: el error de arrastre es un sesgo correlacionado, no ruido blanco,
  y el blanco borra la información de velocidad de los pases anteriores. El defecto **q = 3e-21 se ajustó en este experimento**
  (mediana ≈ lotes; optimista para otros escenarios). Con las mismas medidas y el mismo prior un solo pase da lo mismo que los lotes.
* **Prior del TLE** (`prior_de_tle`): σ_pos = √(1² + (2 km/día × edad)²); σ_vel = (v/r) σ_pos (un
  error de posición a lo largo de la pista es un desfase de fase que gira la velocidad el mismo
  ángulo: 1 km ≈ 1.1 m/s) y **un término de información sobre la energía** (el TLE conoce el
  *periodo*: 14 m de semieje × max(1, edad) ≡ 2 km/día × max(1, edad) de deriva). **Es un supuesto
  calibrado a la escala del error del TLE del escenario**, y es decisivo con solo ángulos
  (ver ablación en §3.3).
* **Consistencia** (`monte_carlo_nees`, resultados en §3.4).

## 3. Experimento científico

### 3.1 Diseño (todo en `od_experimento.py`; cada supuesto, un parámetro)
* **Verdad**: estado SGP4 del TLE real de la ISS (`data/tle/stations.tle`, época 2026-10-01
  19:41 UTC) **más un desfase de época** de (0, 1.0, 0.3) km (R, T, N) y (0.3, 2.0, 0.3) mm/s: el
  error propio de un TLE en su época (~1 km a lo largo de la pista); el desfase radial de posición
  es 0 a propósito (1 km radial cambiaría el semieje 2 km y el TLE derivaría 100 km/día). Propagada
  con **J4 + arrastre con BC +25 %** del que conoce el estimador y **variación temporal de la
  densidad** (±25 % a 1.7 días y ±30 % al periodo orbital: el abultamiento diurno; órdenes de
  magnitud de Vallado cap. 8). Con esto el TLE sin tocar se aleja ~**2.1 km a 1 día, 6.4 km a 2
  días, 11 km a 3 y 40 km a 5** (lo documentado en `orbit.py`: 1-3 km/día al principio y creciente).
* **A priori conocido**: el TLE envejecido N días, propagado con SGP4.
* **Estimador**: J2 + arrastre nominal (menos física que la verdad: no hay "crimen inverso"). **No estima el
  factor de arrastre `p`** (con 3 pases en 12 h es inobservable y empeora la prediccion: ver 3.3); la fila "estimando el
  arrastre" es la ablación.
* **Medidas**: 1 cada 5 s con elevación 3°-88° (≈100 por pase), sigmas de §1.3. Estaciones: CDMX
  (máscara 5°), Madrid (5°), Quito (10°), de `data/estaciones.yaml`. Arranque: los primeros 8 pases
  de CDMX con elevación máxima ≥ 20° tras N días del TLE.
* **Métrica**: ángulo entre la dirección a la órbita predicha y a la verdad (incluye la
  compresión cos el del acimut) a lo largo de los pases de CDMX posteriores: el desapuntamiento que
  tendría la antena si siguiera la predicción (*program track* puro).
* **Métodos**: TLE viejo; OD con 1 pase de CDMX (3 conjuntos de medidas); OD con 3 pases de la
  red (el de arranque + 2 más en las 12 h siguientes) elegidos por la regla *rápida* (los 2
  siguientes con elevación ≥ 20°) o por el **planificador de información** (§3.5).
* **Evaluación**: el primer pase de CDMX tras 13 h del arranque ("siguiente"), el primero ≥ 24 h
  después ("+24 h") y ≥ 48 h ("+48 h"): los mismos para todos los métodos.

### 3.2 Resultados (ISS sobre CDMX, 32 escenarios)

Reproducir: `python3 scripts/experimento_od.py --sat ISS --edades 1,2,3,5 --repeticiones 8 --p-cal 0.406 --salida docs/od_resultados_iss.json`
(24 min con 7 procesos en una máquina cargada) y `python3 scripts/od_tablas.py` para estas tablas. Semilla fija por
(edad, repetición): mismo resultado. Todas las cifras de abajo son del JSON.

**Cómo envejece el TLE en este escenario** (distancia de posición entre SGP4 y la verdad, ISS):

| Edad del TLE | 0.5 d | 1 d | 2 d | 3 d | 5 d |
|---|---|---|---|---|---|
| Error de posición | 1.6 km | 2.1 km | 6.4 km | 11 km | 40 km |

Es 1-3 km/día al principio (lo documentado en `orbit.py`) y crece con el cuadrado del tiempo; el
caso de 5 días (40 km, 8 km/día de media) es un **caso de estrés** por encima de lo habitual. Los grupos de
las tablas se nombran por el día de *arranque* (N); la edad real del TLE en el primer pase de OD es mayor
porque se toman los 8 primeros pases de CDMX con elevación ≥ 20° tras N días (se indica el rango).

<!-- generado por scripts/od_tablas.py desde docs/od_resultados_iss.json: 32 escenarios -->

**Arranque a partir de 1 día(s) de la época del TLE** (8 escenarios; edad real 1.3-4.8 días; error de posición del TLE en el arranque, mediana 13.5 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 1.346 (6.104) | 3.139 (7.980) | 6.961 (8.130) | 0.449 (0.596) |
| OD 1 pase: az/el | 0.946 (8.600) | 2.646 (15.188) | 7.003 (14.320) | 0.244 (0.792) |
| OD 1 pase: az/el + Doppler | 0.411 (4.065) | 1.492 (7.715) | 3.556 (8.101) | 0.107 (0.394) |
| OD 1 pase: az/el + rango + Doppler | 0.290 (2.830) | 1.134 (6.172) | 2.452 (6.738) | 0.073 (0.309) |
| OD 3 pases de la red (regla rápida): az/el | 0.020 (0.084) | 0.130 (0.259) | 0.276 (0.662) | 0.008 (0.017) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.020 (0.107) | 0.109 (0.284) | 0.346 (0.674) | 0.007 (0.011) |
| OD 3 pases de la red (planificador): az/el | 0.020 (0.061) | 0.130 (0.253) | 0.344 (0.670) | 0.007 (0.013) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.018 (0.082) | 0.092 (0.263) | 0.330 (0.660) | 0.007 (0.012) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 0.054 (0.469) | 0.174 (1.638) | 0.442 (2.816) | 0.009 (0.100) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.020 (0.108) | 0.175 (0.294) | 0.413 (0.717) | 0.007 (0.011) |

**Arranque a partir de 2 día(s) de la época del TLE** (8 escenarios; edad real 2.2-5.7 días; error de posición del TLE en el arranque, mediana 25.4 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 3.139 (7.980) | 6.961 (10.134) | 6.961 (8.130) | 0.543 (0.892) |
| OD 1 pase: az/el | 2.639 (14.902) | 9.624 (21.747) | 11.325 (18.170) | 0.439 (1.694) |
| OD 1 pase: az/el + Doppler | 1.211 (5.205) | 2.832 (11.598) | 3.386 (9.160) | 0.168 (0.459) |
| OD 1 pase: az/el + rango + Doppler | 0.462 (3.641) | 1.921 (7.876) | 3.100 (6.823) | 0.089 (0.329) |
| OD 3 pases de la red (regla rápida): az/el | 0.019 (0.111) | 0.115 (0.307) | 0.299 (0.671) | 0.006 (0.016) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.021 (0.095) | 0.090 (0.289) | 0.259 (0.648) | 0.008 (0.013) |
| OD 3 pases de la red (planificador): az/el | 0.017 (0.060) | 0.112 (0.242) | 0.288 (0.685) | 0.006 (0.013) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.017 (0.080) | 0.092 (0.260) | 0.254 (0.658) | 0.007 (0.013) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 0.035 (0.469) | 0.128 (1.632) | 0.286 (2.974) | 0.010 (0.108) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.027 (0.308) | 0.134 (2.223) | 0.397 (1.506) | 0.007 (0.023) |

**Arranque a partir de 3 día(s) de la época del TLE** (8 escenarios; edad real 3.2-6.7 días; error de posición del TLE en el arranque, mediana 42.2 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 6.961 (8.130) | 6.961 (10.134) | 7.915 (18.219) | 0.624 (1.639) |
| OD 1 pase: az/el | 8.812 (17.190) | 16.376 (23.328) | 13.936 (73.868) | 0.740 (4.119) |
| OD 1 pase: az/el + Doppler | 2.212 (4.182) | 3.614 (12.270) | 6.267 (15.604) | 0.275 (0.712) |
| OD 1 pase: az/el + rango + Doppler | 1.310 (2.940) | 2.560 (7.605) | 4.153 (9.767) | 0.156 (0.434) |
| OD 3 pases de la red (regla rápida): az/el | 0.023 (0.174) | 0.085 (0.380) | 0.294 (0.689) | 0.008 (0.016) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.026 (0.093) | 0.086 (0.254) | 0.261 (0.632) | 0.007 (0.013) |
| OD 3 pases de la red (planificador): az/el | 0.016 (0.063) | 0.100 (0.232) | 0.319 (0.667) | 0.006 (0.012) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.023 (0.080) | 0.093 (0.232) | 0.326 (0.643) | 0.007 (0.013) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 0.079 (2.491) | 0.231 (8.356) | 0.478 (22.245) | 0.012 (0.320) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.032 (0.364) | 0.129 (2.663) | 0.415 (1.743) | 0.007 (0.027) |

**Arranque a partir de 5 día(s) de la época del TLE** (8 escenarios; edad real 5.2-8.7 días; error de posición del TLE en el arranque, mediana 85.3 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 6.961 (17.334) | 10.936 (22.290) | 18.250 (22.290) | 1.883 (3.434) |
| OD 1 pase: az/el | 14.940 (64.817) | 31.969 (113.805) | 92.833 (114.432) | 4.306 (10.288) |
| OD 1 pase: az/el + Doppler | 2.645 (17.260) | 8.843 (46.355) | 13.995 (51.338) | 0.583 (3.973) |
| OD 1 pase: az/el + rango + Doppler | 1.546 (10.455) | 5.957 (27.402) | 8.038 (30.320) | 0.353 (2.278) |
| OD 3 pases de la red (regla rápida): az/el | 0.019 (0.125) | 0.124 (0.346) | 0.287 (0.693) | 0.008 (0.012) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.018 (0.082) | 0.106 (0.245) | 0.294 (0.645) | 0.007 (0.014) |
| OD 3 pases de la red (planificador): az/el | 0.013 (0.046) | 0.133 (0.253) | 0.284 (0.676) | 0.006 (0.012) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.012 (0.052) | 0.114 (0.208) | 0.309 (0.646) | 0.006 (0.013) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 1.197 (4.764) | 6.263 (42.642) | 18.198 (63.630) | 0.290 (2.757) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.037 (0.391) | 0.180 (5.692) | 0.660 (7.111) | 0.011 (0.034) |

El planificador de información eligió pases distintos de la regla rápida en 20 de 32 escenarios.

**Fracción de pases (todas las edades, pase siguiente) con el máximo bajo el umbral:**

| Método | Ka 3 dB (0.117°) | Ka lock −10 dB (0.213°) | S 3 dB (1.59°) |
|---|---|---|---|
| TLE viejo (SGP4) | 0 % | 0 % | 28 % |
| OD 1 pase: az/el | 0 % | 3 % | 28 % |
| OD 1 pase: az/el + Doppler | 9 % | 22 % | 50 % |
| OD 1 pase: az/el + rango + Doppler | 12 % | 19 % | 62 % |
| OD 3 pases de la red (regla rápida): az/el | 91 % | 97 % | 100 % |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 91 % | 100 % | 100 % |
| OD 3 pases de la red (planificador): az/el | 100 % | 100 % | 100 % |
| OD 3 pases de la red (planificador): az/el + Doppler | 100 % | 100 % | 100 % |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 53 % | 62 % | 84 % |
| EKF 3 pases, rápida (az/el + Doppler) | 75 % | 88 % | 100 % |

**Fracción de pases con el error en AOS (donde se adquiere) bajo el umbral, todas las edades:**

| Método | < 0.117° (Ka 3 dB): pase siguiente / +24 h / +48 h | < 0.213° (captura del conical scan, Ka): pase siguiente / +24 h / +48 h |
|---|---|---|
| TLE viejo (SGP4) | 3 % / 0 % / 0 % | 12 % / 3 % / 0 % |
| OD 1 pase: az/el | 12 % / 0 % / 0 % | 22 % / 3 % / 0 % |
| OD 1 pase: az/el + Doppler | 25 % / 19 % / 16 % | 47 % / 25 % / 25 % |
| OD 1 pase: az/el + rango + Doppler | 44 % / 22 % / 12 % | 62 % / 38 % / 25 % |
| OD 3 pases de la red (regla rápida): az/el | 100 % / 100 % / 91 % | 100 % / 100 % / 100 % |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 100 % / 100 % / 91 % | 100 % / 100 % / 100 % |
| OD 3 pases de la red (planificador): az/el | 100 % / 100 % / 100 % | 100 % / 100 % / 100 % |
| OD 3 pases de la red (planificador): az/el + Doppler | 100 % / 100 % / 100 % | 100 % / 100 % / 100 % |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 75 % / 62 % / 53 % | 84 % / 78 % / 62 % |
| EKF 3 pases, rápida (az/el + Doppler) | 100 % / 88 % / 78 % | 100 % / 88 % / 88 % |


**Lectura** (cifras de las tablas; "máximo del pase" = peor instante del pase):

1. **TLE viejo**: 1.3° a 7° de máximo según la edad (6 a 30 veces el haz de Ka, 0.233°). Incluso en AOS,
   donde se adquiere, el error mediano es 0.45-0.62° con TLE de 1-4 días: el AOS queda bajo la mitad del haz
   (0.117°) en solo el **3 % de los pases** y bajo el cono de captura del conical scan (−10 dB: 0.213°) en el 12 %.
2. **OD con un solo pase de CDMX**: con **solo ángulos no mejora** el TLE (0.9-15° de máximo; peor que el TLE a
   partir de 3 días). Con Doppler baja 3-4 veces (0.41, 1.2, 2.2° en las tres primeras edades) y con rango y
   Doppler a 0.29, 0.46, 1.3°: mejor que el TLE pero **sin llegar al haz de Ka** (9-12 % de los pases bajo
   0.117°). Razón física: un pase de 8 min mide la velocidad a lo largo de la pista a ~5 mm/s (Doppler,
   σ = 0.05 m/s por muestra) y 1 m/s de velocidad a lo largo de la pista son ~250 km/día de deriva; en
   horas, 5 mm/s ya son ~0.1-0.2 km/h.
3. **OD con 3 pases de la red en 12 h** (el de arranque + 2 más): **0.02° de máximo en el pase siguiente (13 h
   después), ~0.1° a +24 h y ~0.3° a +48 h**, casi independiente de la edad del TLE (1-9 días) y de qué
   canales se usen (los ángulos de 3 pases con base larga bastan; el rango y el Doppler no añaden nada
   medible cuando hay varios pases). Eso es 70-370 veces mejor que el TLE en el pase siguiente. En AOS: 0.006-0.008° (p95 ≤ 0.023°)
   en el pase siguiente y p95 ≤ 0.05° a +24 h. Bajo el umbral de 3 dB de Ka (0.117°) el máximo del pase queda el 91-100 % de las veces en el
   pase siguiente.
4. **Por qué el error crece 0.02 → 0.1 → 0.3° al alejarse del dato**: es el error de modelo (la verdad tiene +25 % de
   arrastre y variaciones de densidad que el estimador no conoce) extrapolado: el estado en la época es
   exacto a decenas de metros, pero el semieje queda sesgado unos metros y la deriva de fase crece con el
   cuadrado del tiempo. Es la misma física que hace envejecer al TLE, con una constante ~100 veces menor.
5. **Estimar el arrastre empeora** (fila "estimando el arrastre"): con 3 pases en 12 h el factor `p` es casi
   inobservable y, con datos muy precisos y un modelo imperfecto, absorbe el error de modelo (ver 3.3). Por eso el
   experimento NO lo estima.
6. **EKF** (q = 3e-21 km²/s³, ajustado en este experimento): mediana cercana a los lotes (0.02-0.03° en el pase
   siguiente) pero **colas peores** (p95 hasta 0.4° en el pase siguiente y 0.3-5.7° a +24 h): es un filtro de una pasada linealizado sobre un
   a priori a 5-85 km. Los lotes re-linealizan hasta converger.
7. **Planificador de información** (3.5): elige pases distintos de la regla rápida en 20 de 32 escenarios, con
   medianas equivalentes y colas ligeramente mejores en el pase siguiente (p95 0.05-0.06° contra 0.08-0.17° con
   solo ángulos); a +24/+48 h empatan. Con 32 escenarios **no es una mejora demostrada**.

**Otro satélite: NOAA 19 (órbita polar, 850 km), red CDMX + Tromsø + Madrid** (`--sat 'NOAA 19'`
`--estaciones cdmx,tromso,madrid --bc 0.0119 --p-cal 1.0`; el arrastre a 850 km es ~100 veces menor, así que el
envejecimiento del TLE viene sobre todo del error de época; el BC es una suposición: ~2.2 × 8 m²/1479 kg):

<!-- generado por scripts/od_tablas.py desde docs/od_resultados_noaa19.json: 8 escenarios -->

**Arranque a partir de 2 día(s) de la época del TLE** (4 escenarios; edad real 2.3-3.8 días; error de posición del TLE en el arranque, mediana 6.3 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 0.535 (0.746) | 0.608 (0.769) | 0.579 (0.760) | 0.072 (0.098) |
| OD 1 pase: az/el | 0.278 (0.521) | 0.551 (0.939) | 0.650 (1.210) | 0.041 (0.067) |
| OD 1 pase: az/el + Doppler | 0.219 (0.594) | 0.363 (1.197) | 0.231 (1.511) | 0.042 (0.082) |
| OD 1 pase: az/el + rango + Doppler | 0.101 (0.363) | 0.178 (0.742) | 0.217 (0.980) | 0.023 (0.048) |
| OD 3 pases de la red (regla rápida): az/el | 0.034 (0.036) | 0.061 (0.073) | 0.076 (0.100) | 0.009 (0.013) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.045 (0.048) | 0.083 (0.090) | 0.106 (0.118) | 0.010 (0.015) |
| OD 3 pases de la red (planificador): az/el | 0.028 (0.044) | 0.054 (0.082) | 0.057 (0.100) | 0.010 (0.014) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.026 (0.039) | 0.051 (0.069) | 0.050 (0.082) | 0.010 (0.014) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 0.045 (0.048) | 0.083 (0.090) | 0.107 (0.118) | 0.010 (0.015) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.045 (0.048) | 0.083 (0.089) | 0.106 (0.118) | 0.010 (0.015) |

**Arranque a partir de 4 día(s) de la época del TLE** (4 escenarios; edad real 4.2-5.8 días; error de posición del TLE en el arranque, mediana 9.7 km). Desapuntamiento **máximo del pase** en grados, mediana (p95):

| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |
|---|---|---|---|---|
| TLE viejo (SGP4) | 0.579 (0.760) | 0.520 (0.695) | 0.551 (0.626) | 0.147 (0.206) |
| OD 1 pase: az/el | 0.343 (0.573) | 0.514 (0.939) | 0.684 (1.153) | 0.064 (0.119) |
| OD 1 pase: az/el + Doppler | 0.047 (0.103) | 0.089 (0.096) | 0.141 (0.179) | 0.012 (0.029) |
| OD 1 pase: az/el + rango + Doppler | 0.079 (0.180) | 0.079 (0.282) | 0.124 (0.356) | 0.023 (0.030) |
| OD 3 pases de la red (regla rápida): az/el | 0.029 (0.047) | 0.038 (0.056) | 0.062 (0.074) | 0.004 (0.015) |
| OD 3 pases de la red (regla rápida): az/el + Doppler | 0.034 (0.039) | 0.043 (0.068) | 0.067 (0.088) | 0.003 (0.012) |
| OD 3 pases de la red (planificador): az/el | 0.019 (0.033) | 0.030 (0.036) | 0.049 (0.050) | 0.004 (0.013) |
| OD 3 pases de la red (planificador): az/el + Doppler | 0.018 (0.031) | 0.026 (0.034) | 0.047 (0.052) | 0.008 (0.013) |
| OD 3 pases, rápida, estimando el arrastre (az/el + Doppler) | 0.034 (0.039) | 0.043 (0.068) | 0.067 (0.088) | 0.003 (0.012) |
| EKF 3 pases, rápida (az/el + Doppler) | 0.034 (0.040) | 0.044 (0.068) | 0.068 (0.088) | 0.003 (0.012) |



Lectura (8 escenarios, 4 por edad: indicativa): a 850 km casi no hay arrastre, el TLE de 2-6 días solo da 0.5-0.6° de máximo (6-10 km al
arranque, error de época que deriva) y 0.07-0.15° en AOS, **ya cerca del umbral de Ka**; la OD de 3 pases lo baja a 0.02-0.05° (12-30 veces)
y un solo pase con az/el + rango + Doppler a 0.08-0.18°. El estimar `p` da lo mismo que fijarlo (a esa altitud no se ve). Con 3 estaciones
y muchos pases por día (Tromsø) la red tiene más opciones: el planificador eligió pases distintos de la regla rápida en 8 de 8 escenarios y
quedó ligeramente mejor (0.019-0.028° contra 0.029-0.034°).

### 3.3 Ablaciones y sensibilidad
* **Prior de energía del TLE** (`test_sin_energia_en_el_prior_los_angulos_solos_empeoran`): con solo ángulos de
  un pase, quitar el término de energía del prior empeora el desapuntamiento del pase siguiente más de 3 veces
  (en un caso medido, 28° contra 0.45°). Sin prior alguno, ~30°. Con Doppler el término de energía casi no
  importa (0.12° sin él, 0.19° con él, mismo caso: el prior es un poco sobreconfiado).
* **Estimar el factor de arrastre `p`** y **inflar las sigmas de rango/Doppler** (12 escenarios, ISS,
  edades 2 y 3 días, 3 pases/12 h, az/el + Doppler; mediana / p95 del máximo en el pase siguiente;
  `scripts/experimento_od_ablacion.py`):

  | Variante | pase siguiente | +24 h |
  |---|---|---|
  | `p` fijo (la que usa el experimento) | 0.021 / 0.126 | 0.088 / 0.329 |
  | estimar `p`, σ_p = 50 % | 0.035 / 0.703 | 0.153 / 2.43 |
  | estimar `p`, σ_p = 30 % | 0.034 / 0.531 | 0.135 / 1.87 |
  | `p` fijo, σ de rango y Doppler × 3 | 0.020 / 0.127 | 0.100 / 0.330 |
  | estimar `p` (30 %), σ × 3 | 0.036 / 0.214 | 0.130 / 0.842 |
  | estimar `p` (30 %), σ × 10 | 0.025 / 0.137 | 0.136 / 0.347 |

  Estimar `p` con datos precisos lleva a `p = 6` (o negativo en el EKF) hasta que se acota a [0.2, 3] (`limites_p`) y aun así deja colas
  de 0.5-0.7° (pase siguiente) y 1.9-2.4° (+24 h): el error de modelo se carga sobre `p`. Solo se vuelve inocuo inflando mucho las σ (que le quita
  información a los datos). El arrastre real se estima con arcos de **días**, no de 12 h.
* **Sesgo de apuntamiento** (`test_sesgo_angular_se_absorbe_*`): un sesgo de 0.02° (2σ) en el acimut mueve la
  órbita estimada ~0.6 km (45 veces más error de posición) y χ²/dof pasa de 1.03 a 1.07: **no se ve en los residuos**. En un
  caso, el desapuntamiento del pase siguiente pasó de 0.3° a 1.0°. Es la limitación principal con datos reales:
  hay que calibrar la montura (sesgo de acimut/elevación, nivelación, refracción) antes de ingerir ángulos.
  Pendiente en esta versión: un procedimiento de calibración de la montura (p. ej. con el Sol) que reduzca ese sesgo.
* **Sigmas**: con la σ declarada, los residuos normalizados dan rms 0.7-1.3 por canal; con la mitad, χ²/dof ≈ 4
  (`test_residuos_normalizados_*`).

### 3.4 Consistencia: residuos y NEES con Monte Carlo
`python3 scripts/experimento_od.py --nees 40 --p-cal 0.406 --dias 5` (40 corridas; la verdad se sortea del prior; az/el + rango
+ Doppler). NEES medio de (r, v) con 6 grados de libertad; un estimador consistente cae en [4.97, 7.12] (95 %):

| Caso | NEES lotes | NEES EKF |
|---|---|---|
| (a) 1 pase, **modelo exacto** | 5.85 | 5.85 |
| (b) 3 pases/12 h, **modelo exacto** | 5.97 | 5.81 |
| (c) 3 pases/12 h, **verdad real** (J4, +25 % arrastre, variación de densidad) estimando el arrastre | 1008 | 2814 |

* Con el modelo exacto la covarianza formal es **consistente** (lotes y EKF). `test_od.py` repite (a) con 30 corridas.
* Con error de modelo la covarianza formal **no sirve**: el NEES es 170-470 veces el esperado. Es la advertencia central de la OD:
  la covarianza formal cubre el ruido de medición, no el modelo; en la práctica se mide el error contra la verdad
  (como aquí) o se calibra con consideración de parámetros / aceleraciones empíricas (no implementado).
* Residuos de la corrida ROS (S4, §5): χ²/dof = 0.44 (los ángulos de la montura son casi perfectos en el simulador, 0.001° rms, y se
  ponderaron con 0.01°: conservador).

### 3.5 Qué estaciones conviene usar para OD (planificador)
`od_experimento.candidatos_planificador` toma las ventanas del **planificador de contactos**
(`planificador.construir_ventanas` con el TLE envejecido: es todo lo que sabe el operador) y
`seleccionar_pases_od` elige golosamente los 2 pases que **minimizan el desapuntamiento formal esperado en el pase
objetivo**: información de Fisher de cada candidato (jacobiano de las medidas × STM sobre la órbita SGP4)
más el prior del TLE; la sensibilidad del pase objetivo es `d(az cos el, el)/dx`. Criterios que salen de ahí:
(i) la **base larga** vale más que el número de datos: pases de estaciones/geometrías distintas separados
por horas fijan el semieje (periodo) que un solo pase no puede medir; (ii) los pases rasantes (< 20°) aportan poco y los casi
cenitales (> 85°) pierden el acimut; (iii) para la ISS (51.6°) Tromsø no ve nunca el satélite y Quito/CDMX/Madrid sí; para una órbita polar
(NOAA 19) Tromsø es la mejor estación de OD por su número de pases. Las ventanas del planificador (hechas con el TLE
viejo) difieren de los pases verdaderos hasta ~1 min en AOS en pases rasantes (prueba), suficiente para decidir.
Honestidad: la ganancia **medida** sobre la regla de "los 2 siguientes pases" no es significativa (3.2, punto 7).

## 4. ¿Cuándo deja de hacer falta el autotrack para adquirir?

El autotrack (conical scan) engancha si la potencia media del haz cae por encima de `lock_db = −10 dB`: con
`L = 12 (e/θ3)²` eso es **e < 0.913 θ3**, o sea 0.213° en Ka (θ3 = 0.233°); la pérdida es 3 dB en e = θ3/2 = 0.117°.
En S (θ3 = 3.18°) esos números son 2.9° y 1.59°. Se compara el error de **AOS** (donde se adquiere) con esos umbrales:

| Fuente de la efeméride | Error en AOS (mediana, p95) | Adquiere sin autotrack en Ka (< 0.117°) |
|---|---|---|
| TLE de 1-4 días | 0.45-0.62° (p95 0.6-1.6°) | 3 % (12 % bajo el cono de captura de 0.213°) |
| OD de 1 pase, az/el + rango + Doppler (edad ≤ 6 d) | 0.07-0.16° (p95 0.3-0.4°) | 44 % |
| **OD de 3 pases de la red** (az/el) | **0.006-0.008° (p95 ≤ 0.023°)** | **100 %** (pase siguiente y +24 h), 91-100 % a +48 h (100 % bajo 0.213°) |

Conclusiones, con las salvedades de la sección 6:
* **Con TLE, en Ka el autotrack es necesario para ADQUIRIR** (solo el 3 % de los pases queda bajo 0.117°); en S el TLE basta casi siempre
  (AOS mediano 1.9° a 5-9 días de edad contra 2.9° de cono de captura; p95 3.4°).
* **Con una OD de varios pases de la red, el error en AOS cae 60-250 veces y el autotrack deja de ser necesario para
  adquirir en Ka durante unas 48 h** después de la OD (error en AOS p95 ~0.1° a +48 h). Esto
  vale mientras el modelo dinámico describa el arrastre como en el escenario (variaciones de densidad de ±25-30 %);
  en otro régimen solar habría que repetir el experimento.
* Para **seguir** el pase completo con < 3 dB en Ka, el máximo del pase pasa de 0.02° (13 h) a ~0.1° (+24 h, 50 % de los pases
  sobre el umbral) y 0.3° (+48 h): el autotrack sigue haciendo falta para seguir más allá de ~1 día después de la OD, y
  **para producir los propios ángulos de 0.01° que alimentan la OD** (sin enganche, la montura apunta a la efeméride y
  sus ángulos no miden al satélite). El lazo es: TLE + autotrack → ángulos → OD de la red → siguientes pases adquiridos
  con la OD (autotrack solo para refinar) → nueva OD con datos frescos.
* Una OD con **un solo pase** no alcanza: 9-12 % de pases bajo el umbral de Ka.

## 5. Nodo ROS `od_estimator` y el lazo en el simulador

`atp_loop.launch.py ... od:=true od_range:=true od_doppler:=true od_out:=<dir>` (todo apagado por defecto: sin `od`, la
corrida es exactamente la de antes; `bash scripts/validar.sh n2 s4 det` da lo de siempre, ver abajo). El nodo cumple el
CONTRATO §8: empareja por **sello** `/joint_states` con `/atp/sat_state`, usa colas de 200 (pasivo, no frena el reloj), anuncia
`/atp/listo` y el generador lo espera (`wait_nodes`, `wait_subscribers`). El "receptor" de rango y Doppler es la verdad del generador
más ruido gaussiano con **semilla fija** (`noise_seed`): repetible. Al llegar `/atp/pass_done` ajusta (lotes, o `mode:=ekf`) en un hilo que
el cierre del nodo espera; publica `/atp/od_result` (JSON, `transient_local`) y escribe `<od_out>/od.json` (CONTRATO §9).
`profile:=od od_file:=<od.json>` hace que `reference_generator` prediga el pase con esa órbita (`od.perfil_od`: propagador numérico +
`pases_efemeride`) y `od_prior:=od` que el siguiente `od_estimator` encadene pases (propaga estado y covarianza con la STM más ruido de proceso).

Verificación de punta a punta (`bash scripts/validar.sh od`, salida real):

```
pase real ISS sobre Ciudad de Mexico: AOS 2026-10-02 15:52:33 UTC, duracion 489.5 s, el max 34.2 deg
RESULTADO n=9789 | apuntamiento: rms=0.0010 max=0.0204 deg | dentro de 0.1 deg: 100.0%
OD lotes: 98 medidas (az+el+rng+rr), sigma_pos formal 0.036 km, chi2/dof 0.44, 2 iter; a priori: TLE ISS (ZARYA) (0.84 dias de edad)
pase por ORBITA ESTIMADA (epoca 2026-10-02T15:52:33Z, sigma_pos 0.036 km) sobre Ciudad de Mexico: AOS 2026-10-03 00:07:57 UTC, el max 14.5 deg
RESULTADO n=7844 | apuntamiento: rms=0.0010 max=0.0312 deg | dentro de 0.1 deg: 100.0%
OD lotes: 79 medidas (az+el+rng+rr), sigma_pos formal 0.005 km, chi2/dof 0.39, 2 iter; a priori: orbita previa /tmp/od/1/od.json
ORBITA ESTIMADA vs TLE (pase a 0.34 dias de ella): AOS dif +0.07 s; DESAPUNTAMIENTO OD-vs-TLE: aos=0.0037 rms=0.0101 max=0.0132 deg
ORBITA ESTIMADA vs TLE (pase a 1.03 dias de la orbita encadenada): AOS dif +0.03 s; DESAPUNTAMIENTO OD-vs-TLE: aos=0.0079 rms=0.0549 max=0.1069 deg
```
Ojo con lo que NO prueba: en el lazo ROS la "verdad" es el propio SGP4 del TLE, así que esto verifica la **cadena** (medidas selladas →
OD → archivo → efeméride → montura) y que el propagador numérico ajustado reproduce a SGP4 en 1 día a 0.11° (Ka: 0.233°; la σ formal de 5 m de la órbita encadenada es, como en 3.4, muy optimista). La
mejora frente a una verdad distinta del TLE es la del experimento de 3.2.

## 6. Limitaciones (qué no se verificó)
* **Todo el experimento es simulado**: la "verdad" es el propio propagador numérico con supuestos (+25 % de BC, variación de
  densidad de ±25/±30 %, desfase de época). No hay datos reales de una estación ni efemérides precisas de la ISS (p. ej. de
  NASA/CPF) contra las que contrastar; la mejora real dependerá de cuánto se parezca la física real a la del escenario.
* **Modelo de estimador y de verdad comparten** marco, constantes gravitatorias WGS84, atmósfera exponencial de Vallado y
  geometría de medidas. No hay presión de radiación, tercer cuerpo, mareas ni armónicos tesserales (decenas a cientos de
  metros por día en LEO) en ninguno de los dos; la atmósfera real (NRLMSISE-00) no se usó.
* **Tabla de atmósfera** transcrita de memoria de Vallado y McClain (2013): verificada por continuidad entre capas y
  contra la US76, no contra el libro.
* **Medidas**: sin refracción, tiempo de luz, aberración, sesgos de montura ni de oscilador (el sesgo de acimut se absorbe en la órbita: 3.3);
  el rango requiere un transpondedor con *ranging* que la estación no tiene; las σ son supuestos, no medidas. UT1-UTC igual que `orbit.py`.
* **Prior del TLE** (σ y término de energía): calibrado a la escala del error del TLE del propio escenario; con otro
  régimen de arrastre habría que recalibrarlo. La covarianza formal **no cubre el error de modelo** (3.4).
* **EKF**: de una pasada, sensible al a priori (colas); `q` se ajustó en este experimento. **No hay UKF.**
* **Experimento**: 32 escenarios de la ISS (6-8 por edad) y 8 de NOAA 19; las colas (p95) con n = 8 son indicativas. El
  arranque usa los primeros pases con elevación ≥ 20° (condiciones favorables) y las medidas de 5 s.
* **ROS**: la "estación" del lazo es el simulador; los ángulos de la montura son casi perfectos (0.001° rms), no los 0.01° declarados.
  El ajuste puede tardar segundos de reloj de pared tras el fin del pase (se espera en el cierre del nodo, ~2 s medidos).
* `prior_de_orbita` encadena pases con ruido de proceso blanco `q = 3e-21`; con intervalos de varios días (modelo de arrastre) no se validó.

## 7. Fuentes
* D. Vallado y W. McClain, *Fundamentals of Astrodynamics and Applications*, 4.ª ed., 2013: zonales (cap. 8), tasas
  seculares de J2 (Eq. 9-41/9-42), atmósfera exponencial (Tabla 8-4, CIRA-72), elementos clásicos (Alg. 9), GMST (Alg. 15), SGP4.
* Dormand y Prince, *A family of embedded Runge-Kutta formulae*, J. Comp. Appl. Math. 6 (1980).
* Squire y Trapp, *Using complex variables to estimate derivatives of real functions*, SIAM Review 40 (1998) (paso complejo).
* Tapley, Schutz y Born, *Statistical Orbit Determination*, 2004 (mínimos cuadrados por lotes, STM, consistencia, NEES);
  Bar-Shalom, Li y Kirubarajan, *Estimation with Applications to Tracking and Navigation*, 2001 (NEES, χ²).
* Jazwinski, *Stochastic Processes and Filtering Theory*, 1970 (EKF iterado).
* Hoots y Roehrich, *Spacetrack Report No. 3* y la librería `sgp4` (constantes WGS72/WGS84 de `sgp4.earth_gravity`).
* US Standard Atmosphere 1976 (NOAA-NASA-USAF), contraste de la atmósfera.
* Cifras internas: N5 (autotrack 0.004° rms), C2 (encoder de 14 bits), `model.ancho_haz` (Ka 0.233°, S 3.18°), `orbit.py` (1-3 km/día).

## 8. Archivos
`propagador.py` (fuerzas, integradores, STM, `Efemeride`), `medidas.py`, `od.py` (lotes, EKF, prior, serialización, `perfil_od`),
`od_experimento.py` (escenario, planificador, Monte Carlo), `od_estimator.py` (nodo), `scripts/experimento_od.py`,
`scripts/experimento_od_ablacion.py`, `scripts/od_tablas.py`, `scripts/od_comparar.py`, `docs/od_resultados_iss.json`,
`docs/od_resultados_noaa19.json`, `docs/od_nees_iss.json`; pruebas `test_propagador.py` (25), `test_od.py` (19).

