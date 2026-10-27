# Enlace óptico tierra–espacio (equipo Y, ronda 6)

El origen de la Estación es el curso de **sistemas ATP** (adquisición, apuntamiento y seguimiento) de Co.De Aerospace, y lo óptico es donde el ATP es más exigente:
haces de microradianes. La casa ya tenía el ATP óptico **entre satélites** (`src/sat_isl`: sin atmósfera, los dos extremos se mueven). Este documento es el otro
paradigma, el de la tendencia real (LCRD, OPALS, SOTA, TBIRD a 200 Gb/s): el **enlace óptico tierra–espacio**, donde manda la **atmósfera** (turbulencia, centelleo,
*beam wander*, nubes). Une el seguidor de la estación (la montura gruesa de `atp_ros` más un espejo de dirección fina) con la red (un gateway óptico de alta
capacidad como alternativa al RF, con disponibilidad limitada por las nubes).

Paquete nuevo `src/atp_optico`; **nada existente cambia** (`sat_isl` y `atp_ros` solo se reutilizan, el CONTRATO gana la §17, `data/` gana `clima_optico.yaml`).
Todo corre sin ROS (`python3 -m atp_optico.escenario`) y en **lockstep** ROS (§8 del CONTRATO) con cifras idénticas.

**Cada cifra dice de dónde sale.** Las fuentes marcadas "(buscado)" se leyeron en resúmenes de búsqueda en la web durante esta ronda, **no** en el artículo completo;
las "(de memoria)" son fórmulas de los libros clásicos que se recuerdan y se verifican con pruebas contra formas cerradas; **SUPUESTO** es un valor de diseño sin fuente.

## 1. Cifras clave

| Qué | Calculado | Contraste |
|---|---|---|
| HV 5/7, r₀ a 500 nm en el cenit | **4.96 cm** | 5 cm (Andrews y Phillips; es el nombre del perfil) |
| HV 5/7, ángulo isoplanático θ₀ a 500 nm | **6.89 µrad** | 7 µrad |
| Escala de r₀ con λ | r₀(1550)/r₀(500) = 3.887 | λ^(6/5) = 3.887 |
| r₀ a 1550 nm, cenit | 19.3 cm (nivel del mar), 21.1 cm (CDMX, 2240 m) | — |
| Ganancia del haz de OPALS (FWHM 1 mrad) | **70.4 dB** | 70.4 dB publicado (buscado) |
| Potencia recibida de OPALS a 30° (820 km, 2.5 W, telescopio de 1 m) | **430 nW** | "cientos de nW" predichos, decenas de nW medidos (buscado) |
| Índice de centelleo, 40 cm, 1550 nm, CDMX | 0.0024 (cenit), 0.015 (30°), **0.16 (10°)** | promediado por apertura 4–17× menor que el de un receptor puntual (4.5× a 10°, 17× en el cenit) |
| Tilt de llegada (un eje, 40 cm) | 2.8 µrad (cenit), 6.8 µrad (10°) | 0.42 (λ/D)(D/r₀)^(5/6), independiente de λ |
| *Beam wander* de la subida (un eje, 40 cm) | 3.5 µrad (cenit), 8.4 µrad (10°) | 2.42 Cn² L³ W₀^(-1/3) con Cn² uniforme (prueba) |
| Red alemana de 4 estaciones ópticas (25–65 % cada una) | modelo 83.8 % | 83.68 % publicado (buscado; **la longitud de correlación se ajustó a este dato**) |
| ATP: residuo del FSM, pase ISS 0 (34°), por eje | **0.55 µrad** rms (p95 0.60), 0.046 del campo de 12 µrad | sin FSM: 31.7 µrad rms y 87 GB; con FSM **423 GB** en el pase |
| ATP hacia el cenit (pases de 60° a 89.5°) | montura **sin tope de velocidad**: FSM enganchado 100 % hasta 88°, pierde 0.4 % a 89.5° (pico de 2.1 mrad); **con un rotor de 6°/s**: 100 % a 80° y **pierde el 14.7 % del pase a 85°** | el FSM de ±1 mrad no sustituye a la estrategia de keyhole |
| Lockstep ROS ×2 y sin ROS | `optico.json` y 1831 filas de serie **idénticas byte a byte** entre las dos corridas ROS y la corrida sin ROS | `validar.sh optico` |

## 2. Mapa del código

| Archivo | Qué hace |
|---|---|
| `src/atp_optico/atp_optico/atmosfera.py` | perfil Cn²(h) de Hufnagel–Valley (HV 5/7 y paramétrico: viento, Cn² en tierra, altitud de la estación), r₀, θ₀, Rytov, centelleo con promediado por apertura, tilt de llegada, *beam wander*, Strehl, viento de Bufton, τ₀ y frecuencia de Greenwood, transmisión (Kruse, Rayleigh, Kasten–Young) |
| `.../enlace.py` | desvanecimiento (log-normal y gamma-gamma), modems OOK (APD) y PPM (conteo de fotones), FEC Reed–Solomon, presupuesto de **bajada** y de **subida**, outage, tasa adaptada; terminales y las referencias OPALS |
| `.../atp.py` | el ATP de la estación: lazo del **FSM** (10 kHz), espectro y generador del **tilt**, ruido del sensor de cuadrante, motor de un paso del lockstep, medidor |
| `.../montura.py` | la montura gruesa de `atp_ros` (PID de 20 Hz + `PlantaMontura`) sin ROS, con el pase SGP4 real |
| `.../escenario.py` | el escenario completo sin ROS y su CLI; `ConfigOptico`; análisis casi estático (curvas vs elevación, volumen por pase) |
| `.../nubes.py`, `.../diversidad.py` | cobertura de nubes con semilla, diversidad de estaciones, volumen por día, comparación con el RF Ka con lluvia |
| `.../nodo_atp.py`, `launch/optico.launch.py` | el nodo pasivo `optico_atp` y el lanzamiento (CONTRATO §17) |
| `data/clima_optico.yaml` | climatología de nubes por estación (cada valor dice su fuente o SUPUESTO) |
| `scripts/optico_estudio.py` | el estudio: las tablas de este documento salen de aquí; JSON en `docs/optico_resultados/` |

## 3. Modelos

### 3.1 Atmósfera (`atmosfera.py`)

**Perfil.** Hufnagel–Valley (Andrews y Phillips, *Laser Beam Propagation through Random Media*, 2.ª ed., cap. 3; Valley 1980):
`Cn²(h) = 0.00594 (v/27)² (10⁻⁵ h)¹⁰ e^(−h/1000) + 2.7·10⁻¹⁶ e^(−h/1500) + A e^(−h/100)`, con v = 21 m/s y A = 1.7·10⁻¹⁴ m^(−2/3) en HV 5/7. Parametrizable por
viento (`viento_rms`) y Cn² en tierra (`cn2_suelo`). Para una estación a altitud h_s los dos primeros términos (chorro, troposfera libre) se evalúan a la altura sobre el
mar (la tropopausa no baja con el terreno) y la capa limite con la altura sobre la estación (A es la Cn² *local*). La alternativa de medirlo todo desde la
estación (`modo_altitud='agl'`) es la que sigue el informe del DLR sobre HV a altitud no nula (buscado, **resumen poco fiable**: el modelo que lo leyó dio cifras de r₀ que
no coinciden con HV 5/7, no se usó) y su diferencia se tabula.

**Integrales** (todas con la regla del trapecio a 0.5 m de paso hasta 30 km sobre la estación; plano-paralelo, sec ζ = 1/sin el, válido para el ≳ 10°):

* Fried (1966): `r₀ = [0.423 k² sec ζ ∫Cn² dh]^(−3/5)`; θ₀ = `[2.914 k² sec^(8/3) ζ ∫Cn² h^(5/3) dh]^(−3/5)`.
* Rytov (onda plana, bajada): `σ_R² = 2.25 k^(7/6) sec^(11/6) ζ ∫Cn²(h) h^(5/6) dh`. Para la subida desde tierra a un satélite lejano el peso de la onda esférica
  `(z(L−z)/L)^(5/6)` se reduce a `z^(5/6)`: da lo mismo (comprobado con la forma cerrada: 1.23 Cn² k^(7/6) L^(11/6) con Cn² uniforme y 0.4965 con la función beta).
* **Centelleo con promediado por apertura** (A&P cap. 8–9; Andrews, Phillips y Hopen 2001): σ_I² = exp(σ_lnx² + σ_lny²) − 1 con
  `σ_lnx² = 0.49 σ_R²/(1 + 0.65 d² + 1.11 σ_R^(12/5))^(7/6)` y `σ_lny² = 0.51 σ_R² (1 + 0.69 σ_R^(12/5))^(−5/6)/(1 + 0.90 d² + 0.62 d² σ_R^(12/5))^(5/6)`, d² = kD²/(4L) y L
  = (altura efectiva de la turbulencia ponderada por su aporte al centelleo, ≈ 7.4 km)/sin el. Régimen **débil** si σ_R² < 0.3 (log-normal), si no moderado–fuerte (**gamma-gamma**,
  α = 1/(e^σ_lnx² − 1), β = 1/(e^σ_lny² − 1); Al-Habash, Andrews y Phillips 2001). Contraste independiente: la integral **por capas** de Churnside (1991) da un factor de promediado
  hasta 2.4 veces menor (A&P decae como d^(−5/3), Churnside como d^(−7/3)); se usa la de A&P porque es la más pesimista (tabla en §5.1).
* **Tilt de llegada** (un eje, tilt-Z): σ² = 0.182 (D/r₀)^(5/3) (λ/D)² (Tyler 1994; con la varianza 0.448 de Noll 1976). Independiente de λ. Kolmogorov (escala externa infinita): **conservador**
  (la escala externa de decenas de m la reduce 10–30 % a D = 0.4 m; no se modeló).
* ***Beam wander*** de un haz colimado que sube a un satélite a R: `⟨r_c²⟩ = 7.25 W₀^(−1/3) ∫₀^R Cn²(z sin el)(R − z)² dz` (A&P cap. 6 y 12), W₀ = D/(2γ), γ = 1.12; por eje, σ = √(⟨r_c²⟩/2)/R.
  Con Cn² uniforme se reduce a 2.42 Cn² L³ W₀^(−1/3) (la prueba lo verifica).
* **Strehl de corta exposición** (sin tilt): `[1 + 0.112 (D/r₀)^(5/3)]^(−6/5)`. **No es una fórmula publicada tal cual**: interpola 1 − 0.134 (D/r₀)^(5/3) (la varianza sin tilt de Noll, a D/r₀ pequeño) con la
  asíntota (r₀/D)² de la ley de Fried; se declara como interpolación.
* **Escala temporal**: viento de Bufton `v(h) = ω_s h/sin el + v_g + 30 e^(−((h−9400)/4800)²)` (con el barrido ω_s de la línea de vista, que domina en LEO), τ₀ = 0.314 r₀/v_eff (Roddier), f_G = 0.134/τ₀ (Greenwood 1977),
  y la velocidad del tilt v_tilt = [∫Cn² v² / ∫Cn²]^(1/2) (Tyler) fija su espectro.
* **Transmisión** (cielo despejado): aerosol de Kruse (β = (3.912/V)(λ/550 nm)^(−q), V = 23 km, escala 1.2 km) + Rayleigh (0.008569 λ_µm^(−4)/km, escala 8 km) + una absorción molecular residual de la ventana
  (**SUPUESTO**: 0.02 a 1550 nm), por la masa de aire de Kasten y Young (1989). Da 0.977 en el cenit en CDMX y 0.876 a 10°.

### 3.2 Desvanecimiento y enlace (`enlace.py`)

La potencia instantánea es P̄·I con I de media 1 y varianza σ_I². La CDF de la gamma-gamma (producto de dos gammas de media 1) se calcula como `P(I < t) = E_Y[gammainc(α, αt/Y)]`, sin funciones
de Bessel de orden grande; el **outage** es P(I < P_req/P̄), el margen de desvanecimiento es el cuantil. El BER medio es E_I[BER(P̄ I)] (siempre mayor que el BER a potencia media: convexidad de la cola Q).

**Bajada** (satélite → telescopio): `P_r = P_t η_t G_t L_t (λ/4πR)² T_atm G_r η_r L_acople`, con G_t = 8/θ_h² (haz gaussiano de semiángulo 1/e² θ_h: el modelo de `sat_isl`,
θ_h = 0.956 λ/D si es difractivo), L_t = 1/(1 + 4σ_t²/θ_h²) (pérdida **media** por la inestabilidad de apuntamiento del satélite), G_r = (πD/λ)². El acople del telescopio al detector es la parte
nueva: con un **detector de campo** (APD de área grande, como el de OPALS) es la fracción de la mancha de corta exposición (radio w = θ_d/√S_ST) que cae dentro del campo θ_f,
ensanchada por el error residual del ATP: `1 − exp(−2θ_f²/(w² + 4σ²))`; con **fibra monomodo** (acoplo para coherente/EDFA, como el OCTL de TBIRD con óptica adaptativa) `0.81 S_ST/(1 + 4σ²/θ_d²)`.
El "ancho de haz de recepción" contra el que se mide el ATP es el campo (12 µrad por defecto: APD de 50 µm a f = 2 m) o λ/D (3.7 µrad con fibra).

**Subida** (estación → satélite): el haz transmitido tiene semiángulo efectivo θ_eff² = θ_up² + θ_d² (1/S_ST − 1) (la turbulencia ensancha el haz en cuadratura) y la pérdida
media por apuntamiento usa la varianza **total**: residuo del ATP + *beam wander* no compensado + error de calibración del *point-ahead*. El haz se **ensancha a propósito**
(θ_up = 12 µrad, no los 3.7 µrad difractivos): el beam wander (3.5–8 µrad) es del tamaño del haz difractivo. `corr_tilt_up` es la fracción del beam wander que el FSM
**pre-compensa** con el tilt medido de la baliza de bajada: depende de la anisoplanaticidad (el point-ahead de ~40–50 µrad supera θ₀ ≈ 20–35 µrad a 1550 nm), por lo que **0** es el valor conservador
y 0.8 el de un sistema ideal; ninguno está medido aquí.

### 3.3 Modems y FEC

* **OOK** con APD (CCSDS 141.0-B, capa física óptica: OOK-NRZ para tasas altas) y la FEC de Reed–Solomon de la familia de CCSDS 142.0-B: el detector es el `Apd` de `sat_isl` (ruido térmico del TIA + de disparo con exceso
  de McIntyre; escalera de 10 Mb/s a 10 Gb/s). Se exige un BER **bruto** ≤ 1e-4; `ber_salida_rs` lo traduce con RS(255,239) (corrige 8 símbolos; Sklar): 1e-4 → 2e-14, 1e-3 → 4.5e-6.
  Como OPALS (RS + OOK, 50 Mb/s, APD de espacio libre de 200 µm).
* **PPM-16** (CCSDS 141.0-B: PPM de alta eficiencia fotónica; la codificación SCPPM es la de 142.0-B) con detector de **conteo de fotones** (SNSPD o APD Geiger: η = 0.7, fondo 5000 cuentas/s,
  **SUPUESTOS**): tasa de error de símbolo **exacta** con estadística de Poisson (señal n_s por pulso, fondo n_b por ranura, empates al azar) y BER = SER·M/(2(M−1)); la capacidad de
  decisión dura del canal M-ario simétrico es una cota inferior de la capacidad que alcanza SCPPM (que decodifica en blando: **de memoria, sin verificar** que opera a ~1 dB de la capacidad, Moision y Hamkins).
  Las pruebas contrastan el SER con Monte Carlo y con la forma cerrada sin fondo, e^(−n_s)(M−1)/M.

Las referencias exactas de los Libros Azules de CCSDS (números de página, parámetros de las tablas de OOK y de SCPPM) **no se abrieron**: lo que se usó es la arquitectura (OOK + RS, PPM + SCPPM), no sus tablas.

### 3.4 ATP de la estación óptica (`atp.py`)

```
perturbación d = (satélite − montura) + tilt de llegada + vibración de la torre + error del modelo de apuntamiento     [µrad, por eje]
e = d − y (+ ruido del cuadrante)  →  C(z) = Ki Ts/(1 − z⁻¹)  →  FSM P(s) = ωn²/(s² + 2ζωn s + ωn²) con retención y retardo  →  y          (10 kHz)
```

* **Montura gruesa**: la de `atp_ros` (J = 2 kg m², b = 0.5 N m s/rad, PID de 20 Hz kp 25, ki 2, kd 8, derivada filtrada y antiwindup) sigue el pase SGP4; su error real contra la **verdad** (no contra el encoder)
  entra al FSM por `/atp/pointing_error` (CONTRATO §17) o, sin ROS, de `montura.simular_montura` (idéntica, ver §6).
* **Lazo fino**: parámetros de `sat_isl.terminal.Lazo` (10 kHz, FSM de 2 kHz con ζ 0.3, cruce a 400 Hz, ±1 mrad) y la conversión ZOH de `sat_isl.seguimiento`; se **adapta** la planta: aquí no hay
  descarga del FSM a un cardán rápido (el lazo de tipo 2 de `sat_isl`) porque la montura de la estación es un servo de programa de ~0.5 Hz; el lazo es de **tipo 1** (|S| = f/f_c a baja frecuencia) y el FSM carga
  con **todo** el error de la montura: tiene que caber en su recorrido (**±1 mrad**) o se pierde el enganche (`enganchado` falso). Margen de fase 68°, de ganancia 7.5 dB, pico de |S| 1.73 (prueba).
* **Tilt atmosférico** como perturbación: rms de `atmosfera.tilt_llegada_rad`, espectro de la forma de Conan et al. (1995) (plano bajo f_L = v_tilt/L₀ con L₀ = 25 m **SUPUESTO**, f^(−2/3) hasta f_c = 0.3 v_tilt/D, f^(−17/3) después),
  generado con un banco de 14 filtros pasa bajos de 3.er orden cuyas varianzas salen de un ajuste no negativo (la prueba: forma a ±30 % entre 2f_L y 1.2f_c, varianza total a 3 %). El barrido de la línea
  de vista (6–20 mrad/s) sube f_c a 37–48 Hz (tabla de §5.3): un cruce de 400 Hz deja un factor ~10 de margen sobre la banda del tilt (el lazo rechaza |S| = f/400 en ella).
* **Sensor de seguimiento** (cuadrante o cámara): ruido por la potencia recibida (`sat_isl.seguimiento.ruido_qpd_urad`, NEP 1.1e-13 W/√Hz, 10 % de la señal de bajada, mancha de 50 µrad): sube cuando baja la potencia.
* **Fuentes de error propias** (todas **SUPUESTOS**, nombradas para que el estudio las barra): vibración de la torre 0.5 µrad rms a 6 Hz; error de camino no común sensor–detector ±0.5 µrad
  (fijo en el pase); y el **residuo del modelo de apuntamiento** tras calibrar con el Sol, **89 µrad por eje** (0.0072° rms en el cielo, la calibración con el Sol §7), un sesgo constante que el FSM absorbe.
  La **adquisición** (la espiral con baliza de `sat_isl.adquisicion`) no se simula: se supone el satélite ya dentro del campo del sensor en AOS (89 µrad ≪ ±1 mrad).
* **Métrica**: error residual por eje (rms, p95 por paso) en µrad contra el campo de recepción, **con y sin FSM sobre las mismas perturbaciones**, fracción del pase con enlace cerrado
  (algún escalón de la escalera con outage ≤ 1 % y FSM enganchado) y GB entregados (útil × (1 − outage)).

### 3.5 Nubes y diversidad (`nubes.py`, `diversidad.py`)

Una nube opaca **corta** el enlace óptico (la lluvia solo debilita el de RF): la disponibilidad de una estación es su probabilidad de línea de vista libre de nubes (CFLOS).
Modelo con semilla: estado latente AR(1) gaussiano por estación (τ = 8 h, **SUPUESTO**), innovaciones con correlación espacial exp(−d/L) (circulo máximo, L = 500 km) y umbral Φ⁻¹(p_claro) (cópula gaussiana):
marginales **exactas** y, para dos estaciones, P(ambas despejadas) = Φ₂(·, ·; ρ) cerrada (las pruebas la verifican). **L se AJUSTÓ a un dato** (la red óptica alemana de 4 estaciones, §6) y p_claro de cada estación
sale de `data/clima_optico.yaml`, donde cada valor dice su origen: Madrid 63.7 % (Fuchs y Moll 2015, buscado); **CDMX 55 %, Tromsø 30 % y Quito 35 % son SUPUESTOS** (sin estadística leída; sustituir
por CFLOS de ISCCP/MODIS/ERA5 del sitio real: es el parámetro que más manda y el que menos se conoce).
La diversidad se mide de dos formas: la fracción del **tiempo** con alguna estación despejada, y los **GB/día** de pases SGP4 reales (NOAA 20, polar, 824 km) repartidos entre las estaciones, cada pase con el estado de nubes de su
estación a la hora del pase.

**RF en Ka** (la alternativa): disponibilidad por **lluvia** (UIT-R P.618, resúmenes buscados): en clima templado (Vigo) 4 dB de margen dan 99.9 % y 11 dB dan 99.99 %; en clima ecuatorial 99.9 % pide
31 dB (otro estudio). Se interpola log-lineal (una década por 7 dB, **SUPUESTO** la extrapolación). La tasa del RF es fija, **0.5 Gb/s** (**SUPUESTO**: clase de un Ka de microsatélite), sobre los mismos pases; no es un presupuesto
de enlace de RF (el de `atp_ros` es el de la montura de 3 m).

## 4. Supuestos y fuentes

| Cifra | Valor | Origen |
|---|---|---|
| Perfil Cn² | HV 5/7: A = 1.7e-14, v = 21 m/s | Andrews y Phillips cap. 3 (de memoria; **las pruebas lo reproducen: r₀ = 5 cm, θ₀ = 7 µrad**) |
| Escala externa L₀ | 25 m | **SUPUESTO** (solo fija f_L del espectro del tilt) |
| Viento de Bufton | v_g = 5 m/s, 30 m/s a 9.4 km | Bufton 1977 (de memoria); v_g **SUPUESTO** |
| Visibilidad | 23 km | OMM "cielo despejado" (de memoria); absorción molecular 0.02 **SUPUESTO** |
| Satélite | 1 W, haz θ_h = 100 µrad, η_t 0.5, 3 µrad de inestabilidad | **SUPUESTO** (microsatélite); el de 35 mW es el de SOTA (35 mW, 5 cm, 1549 nm, 10 Mb/s, buscado) |
| Telescopio | 40 cm, η_r 0.35, campo 12 µrad (APD de 50 µm a f = 2 m) | **SUPUESTO**; OPALS usó 1 m + APD de 200 µm (buscado) |
| APD + TIA | η 0.8, M 10, k 0.4, 1 nA, 5 pA/√Hz | **SUPUESTO** típico (el de `sat_isl`) |
| PPM-16, detector de conteo | η 0.7, fondo 5000 cps | **SUPUESTO** (SNSPD, noche) |
| FSM, lazo | ±1 mrad, 2 kHz, ζ 0.3, cruce 400 Hz, 10 kHz | **SUPUESTO** típico (el de `sat_isl`) |
| Sensor de seguimiento | 10 % de la señal, mancha de 50 µrad, NEP 1.1e-13 W/√Hz | **SUPUESTO** |
| Vibración de la torre, camino no común | 0.5 µrad rms a 6 Hz; ±0.5 µrad | **SUPUESTO** |
| Residuo del modelo de apuntamiento | 89 µrad por eje (0.0072° en el cielo) | de la casa, simulado, no medido en hardware |
| Montura | J 2, b 0.5, PID 25/2/8 a 20 Hz, sin viento ni fricción | la de `atp_ros` (defectos de `atp_loop`) |
| Subida | 20 W, θ_up 12 µrad, error de *point-ahead* 0.5 µrad | **SUPUESTO** |
| CFLOS: Madrid | 63.7 % | Fuchs y Moll 2015 (buscado) |
| CFLOS: CDMX, Tromsø, Quito | 55 %, 30 %, 35 % | **SUPUESTOS** |
| Longitud de correlación de nubes | 500 km | **ajustada** a la red alemana (83.68 %), no validada |
| Ka con lluvia | 4 dB → 99.9 %, 11 dB → 99.99 % (templado); 31 dB → 99.9 % (ecuatorial) | resúmenes buscados (Vigo; un estudio ecuatorial) |
| Tasa del RF Ka | 0.5 Gb/s | **SUPUESTO** |
| OPALS | 410 km, 2.5 W, 1550 nm, FWHM ~1 mrad, G_t 70.4 dB, 50 Mb/s OOK + RS, OCTL 1 m, APD de 200 µm | JPL (buscado) |
| TBIRD | 200 Gb/s, 6U, OCTL de 1 m con óptica adaptativa a fibra monomodo | NASA/MIT-LL (buscado); **no se modeló** (coherente y óptica adaptativa) |

## 5. Resultados (`python3 scripts/optico_estudio.py todo`)

### 5.1 Atmósfera

**Atmosfera: HV 5/7 (A = 1.7e-14, v = 21 m/s)**

| sitio | lam nm | r0 cenit cm | theta0 urad | Rytov cenit | tau0 ms (sin barrido) | f_G Hz |
|---|---|---|---|---|---|---|
| nivel del mar | 500 | 4.96 | 6.9 | 0.2351 | 1.86 | 72 |
| nivel del mar | 1064 | 12.28 | 17.1 | 0.0974 | 4.60 | 29 |
| nivel del mar | 1550 | 19.28 | 26.8 | 0.0628 | 7.23 | 19 |
| CDMX 2240 m | 500 | 5.43 | 9.0 | 0.1557 | 1.60 | 84 |
| CDMX 2240 m | 1064 | 13.44 | 22.3 | 0.0645 | 3.97 | 34 |
| CDMX 2240 m | 1550 | 21.12 | 35.1 | 0.0416 | 6.23 | 22 |

Contraste: HV 5/7 publicado r0 = 5 cm y theta0 = 7 urad a 500 nm: calculado 4.96 cm y 6.89 urad. Escala lam^(6/5): r0(1550)/r0(500) = 3.887 (formula 3.887).

**Efecto de la altitud (modo asl, por defecto, contra agl) a 1550 nm, cenit y 30 deg**

| modo | r0 cenit cm | r0 30 deg cm | Rytov 30 deg |
|---|---|---|---|
| asl | 21.12 | 13.93 | 0.148 |
| agl | 19.28 | 12.72 | 0.224 |

**Por elevacion (CDMX, 1550 nm, telescopio 40 cm, barrido de la linea de vista 10 mrad/s)**

| el | r0 cm | theta0 urad | sigma_R^2 | sigma_I^2 puntual | sigma_I^2 (40 cm) | regimen | tilt urad (1 eje) | beam wander urad | tau0 ms | T atm |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 7.4 | 2.1 | 1.031 | 0.719 | 0.1591 | moderado-fuerte | 6.76 | 8.42 | 0.20 | 0.876 |
| 15 | 9.4 | 4.0 | 0.496 | 0.430 | 0.0734 | moderado-fuerte | 5.53 | 6.90 | 0.37 | 0.914 |
| 20 | 11.1 | 6.3 | 0.297 | 0.278 | 0.0388 | debil | 4.81 | 6.00 | 0.56 | 0.933 |
| 30 | 13.9 | 11.6 | 0.148 | 0.145 | 0.0151 | debil | 3.98 | 4.97 | 0.97 | 0.954 |
| 45 | 17.2 | 20.1 | 0.079 | 0.078 | 0.0061 | debil | 3.35 | 4.18 | 1.57 | 0.967 |
| 60 | 19.4 | 27.9 | 0.054 | 0.054 | 0.0036 | debil | 3.03 | 3.77 | 2.05 | 0.973 |
| 90 | 21.1 | 35.1 | 0.042 | 0.042 | 0.0024 | debil | 2.82 | 3.51 | 2.47 | 0.977 |

Promediado por apertura de 40 cm (A&P con L efectiva / Churnside por capas): 20 deg: 0.139 / 0.074; 45 deg: 0.078 / 0.035; 90 deg: 0.058 / 0.024

### 5.2 Tasa, outage y margen contra la elevación

**Bajada: satelite de 550 km, 1 W, theta_h 100 urad, telescopio 0.4 m, campo 12 urad, error residual 0.6 urad, outage <= 1 %**

| el | R km | P_rx dBm | sigma_I^2 | regimen | margen desv. dB (1 %) | OOK Gb/s (tope 10) | outage | margen s/ 10 Gb/s dB | margen PPM-16 a 250 Mb/s dB | BER 1 Gb/s sin centelleo | BER 1 Gb/s con centelleo |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 1815 | -34.3 | 0.1591 | moderado-fuerte | 4.63 | 2.50 | 2.0e-03 | 2.0 | 35.8 | 4.3e-60 | 4.4e-08 |
| 15 | 1518 | -32.6 | 0.0734 | moderado-fuerte | 3.01 | 10.00 | 2.2e-03 | 3.7 | 37.5 | 4.4e-103 | 2.3e-15 |
| 20 | 1294 | -31.1 | 0.0388 | debil | 2.05 | 10.00 | 8.4e-10 | 5.2 | 39.0 | 9.2e-161 | 4.5e-32 |
| 30 | 993 | -28.7 | 0.0151 | debil | 1.27 | 10.00 | 4.1e-46 | 7.6 | 41.4 | 0.0e+00 | 5.0e-76 |
| 45 | 749 | -26.2 | 0.0061 | debil | 0.80 | 10.00 | 1.0e-194 | 10.1 | 43.9 | 0.0e+00 | 6.7e-178 |
| 60 | 627 | -24.6 | 0.0036 | debil | 0.61 | 10.00 | 0.0e+00 | 11.7 | 45.5 | 0.0e+00 | 1.9e-295 |
| 90 | 550 | -23.5 | 0.0024 | debil | 0.50 | 10.00 | 0.0e+00 | 12.8 | 46.6 | 0.0e+00 | 0.0e+00 |

A la misma tasa bruta de ~250 Mb/s: sensibilidad OOK-APD -45.6 dBm contra PPM-16 con conteo de fotones -70.1 dBm (BER 1e-4): el PPM gana 24.5 dB, a costa de detector de conteo (SNSPD) y de la limitacion de ~1e9 ranuras/s.

**Bajada de un microsatelite de 35 mW (clase SOTA: 1549 nm, 35 mW, 5 cm; mismo telescopio y mismo haz de 100 urad)**

| el | R km | P_rx dBm | margen desv. dB (1 %) | OOK Mb/s | outage | margen PPM-16 a 250 Mb/s dB |
|---|---|---|---|---|---|---|
| 10 | 1815 | -48.9 | 4.63 | 0 | 1.0e+00 | 21.2 |
| 15 | 1518 | -47.1 | 3.01 | 10 | 5.5e-06 | 23.0 |
| 20 | 1294 | -45.6 | 2.05 | 100 | 6.2e-03 | 24.5 |
| 30 | 993 | -43.3 | 1.27 | 250 | 2.0e-06 | 26.9 |
| 45 | 749 | -40.8 | 0.80 | 1250 | 5.9e-04 | 29.4 |
| 60 | 627 | -39.2 | 0.61 | 2500 | 2.3e-04 | 30.9 |
| 90 | 550 | -38.0 | 0.50 | 2500 | 2.2e-22 | 32.1 |

**Subida: laser de 20 W de la estacion, desde el telescopio de 40 cm (receptor del satelite de 5 cm)**

| variante | el | R km | P_rx dBm | beam wander urad | perdida apunt. dB | sigma_I^2 | margen s/ 10 Mb/s dB | outage a 10 Mb/s |
|---|---|---|---|---|---|---|---|---|
| theta_up 12 urad, sin precompensar | 10 | 1815 | -25.4 | 8.42 | -4.15 | 0.1591 | 27.6 | 1.7e-25 |
| theta_up 12 urad, sin precompensar | 20 | 1294 | -20.4 | 6.00 | -2.81 | 0.0388 | 32.6 | 0.0e+00 |
| theta_up 12 urad, sin precompensar | 30 | 993 | -17.2 | 4.97 | -2.18 | 0.0151 | 35.8 | 0.0e+00 |
| theta_up 12 urad, sin precompensar | 60 | 627 | -12.3 | 3.77 | -1.45 | 0.0036 | 40.7 | 0.0e+00 |
| theta_up 12 urad, sin precompensar | 90 | 550 | -11.0 | 3.51 | -1.29 | 0.0024 | 42.0 | 0.0e+00 |
| theta_up 12 urad, 80 % precompensado | 10 | 1815 | -22.5 | 8.42 | -1.24 | 0.1591 | 30.5 | 7.4e-29 |
| theta_up 12 urad, 80 % precompensado | 20 | 1294 | -18.4 | 6.00 | -0.77 | 0.0388 | 34.6 | 0.0e+00 |
| theta_up 12 urad, 80 % precompensado | 30 | 993 | -15.6 | 4.97 | -0.58 | 0.0151 | 37.4 | 0.0e+00 |
| theta_up 12 urad, 80 % precompensado | 60 | 627 | -11.2 | 3.77 | -0.38 | 0.0036 | 41.8 | 0.0e+00 |
| theta_up 12 urad, 80 % precompensado | 90 | 550 | -10.0 | 3.51 | -0.34 | 0.0024 | 43.0 | 0.0e+00 |
| theta_up 6 urad | 10 | 1815 | -24.3 | 8.42 | -7.02 | 0.1591 | 28.7 | 8.0e-27 |
| theta_up 6 urad | 20 | 1294 | -18.5 | 6.00 | -5.77 | 0.0388 | 34.5 | 0.0e+00 |
| theta_up 6 urad | 30 | 993 | -14.9 | 4.97 | -4.97 | 0.0151 | 38.1 | 0.0e+00 |
| theta_up 6 urad | 60 | 627 | -9.2 | 3.77 | -3.80 | 0.0036 | 43.8 | 0.0e+00 |
| theta_up 6 urad | 90 | 550 | -7.6 | 3.51 | -3.52 | 0.0024 | 45.4 | 0.0e+00 |
| theta_up 25 urad | 10 | 1815 | -28.5 | 8.42 | -1.56 | 0.1591 | 24.5 | 6.0e-22 |
| theta_up 25 urad | 20 | 1294 | -24.5 | 6.00 | -0.89 | 0.0388 | 28.5 | 6.5e-247 |
| theta_up 25 urad | 30 | 993 | -21.8 | 4.97 | -0.64 | 0.0151 | 31.2 | 0.0e+00 |
| theta_up 25 urad | 60 | 627 | -17.5 | 3.77 | -0.39 | 0.0036 | 35.5 | 0.0e+00 |
| theta_up 25 urad | 90 | 550 | -16.3 | 3.51 | -0.34 | 0.0024 | 36.7 | 0.0e+00 |

Contraste OPALS (ISS, 2.5 W, FWHM 1 mrad, OCTL de 1 m, 30 deg, 820 km): G_tx = 70.4 dB (publicado 70.4) y P_rx = 430 nW (prediccion publicada: cientos de nW; medida: decenas).

Lectura: el desvanecimiento por centelleo cuesta **0.5 dB en el cenit y 4.6 dB a 10°** a un outage del 1 %; con un telescopio de 40 cm el promediado por apertura lo mantiene en el régimen débil por
encima de 20°. El **BER medio con centelleo** es muchos órdenes de magnitud mayor que el de potencia media (4e-8 contra 4e-60 a 10°): un enlace sin margen de desvanecimiento no cierra. La tasa **cae con la elevación** por el rango
(1815 km a 10° contra 550 km en el cenit, 10 dB) y por la atmósfera (T, centelleo, r₀); con el microsatélite de 35 mW de SOTA la tasa sube de 0 a 2.5 Gb/s del horizonte al cenit: la **elevación mínima útil** manda
la duración del contacto. El PPM con conteo de fotones gana ~25 dB de sensibilidad frente al OOK con APD a la misma tasa bruta pero está limitado a ~1e9 ranuras/s por el detector.
La subida (20 W, un satélite con receptor de 5 cm) sobra en potencia (−11 a −25 dBm); lo que cuesta es el **beam wander**: **1.3–4.2 dB** de pérdida por apuntamiento con un haz de 12 µrad, y un haz demasiado estrecho
(6 µrad) pierde 3.5–7 dB, uno demasiado ancho (25 µrad) pierde ganancia (5 dB menos de potencia en el satélite). La pre-compensación del tilt con la baliza recupera 1 dB en el cenit y 3 dB a 10° si funciona (80 %).

### 5.3 ATP: error residual con y sin FSM

**ATP optico en pases SGP4 reales sobre CDMX (mascara 10 deg; montura de atp_ros, PID de 20 Hz, sin viento)**

| pase | el max | dur s | montura rms/eje urad | tilt rms urad | sin FSM rms urad | con FSM rms urad | con FSM p95 | rms/campo | y max urad | FSM enganchado % | cerrado sin FSM % | cerrado con FSM % | GB sin FSM | GB con FSM |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ISS #0 | 34.2 | 374 | 16.3 | 4.85 | 31.7 | 0.55 | 0.60 | 0.046 | 415 | 100.0 | 99.8 | 100.0 | 87.1 | 422.9 |
| ISS #1 | 14.5 | 233 | 24.4 | 5.98 | 37.8 | 0.62 | 0.66 | 0.052 | 824 | 100.0 | 98.8 | 100.0 | 5.8 | 250.2 |
| ISS #2 | 23.8 | 330 | 17.0 | 5.22 | 30.6 | 0.56 | 0.62 | 0.047 | 519 | 100.0 | 99.8 | 100.0 | 42.7 | 371.6 |
| ISS #3 | 78.4 | 404 | 56.1 | 4.45 | 60.9 | 0.52 | 0.58 | 0.044 | 352 | 100.0 | 99.8 | 100.0 | 165.9 | 460.3 |
| NOAA 20 #0 | 13.8 | 314 | 14.0 | 6.05 | 28.9 | 0.56 | 0.63 | 0.046 | 507 | 100.0 | 98.8 | 100.0 | 1.2 | 77.0 |

**Variantes sobre dos pases (la misma montura): receptor, viento en la montura, camino no comun**

| pase | variante | sin FSM rms | con FSM rms | rms/campo | y max | enganchado % | cerrado con FSM % | GB con FSM | GB sin |
|---|---|---|---|---|---|---|---|---|---|
| ISS #3 | base (campo 12 urad) | 60.9 | 0.52 | 0.044 | 352 | 100.0 | 100.0 | 460.3 | 165.9 |
| ISS #3 | campo 4 urad | 60.9 | 0.52 | 0.131 | 352 | 100.0 | 100.0 | 414.4 | 32.1 |
| ISS #3 | fibra monomodo (campo = 3.7 urad) | 60.9 | 0.52 | 0.044 | 352 | 100.0 | 100.0 | 353.1 | 8.6 |
| ISS #3 | sin camino no comun ni modelo de apuntamiento | 56.3 | 0.16 | 0.013 | 376 | 100.0 | 100.0 | 460.3 | 321.3 |
| ISS #3 | modelo de apuntamiento SIN calibrar (0.52 deg) | 1838.7 | 45.64 | 3.803 | 2378 | 0.0 | 0.0 | 0.0 | 0.0 |
| ISS #3 | sensor de camara de 1 kHz (cruce del lazo a 80 Hz) | 60.9 | 0.82 | 0.069 | 352 | 100.0 | 100.0 | 460.0 | 165.9 |
| ISS #3 | viento en la montura (0.02 N m rms, el doble del de la campana de atp_ros) | 591.1 | 7.46 | 0.622 | 2295 | 82.4 | 82.4 | 377.8 | 3.7 |
| ISS #0 | base (campo 12 urad) | 31.7 | 0.55 | 0.046 | 415 | 100.0 | 100.0 | 422.9 | 87.1 |
| ISS #0 | campo 4 urad | 31.7 | 0.55 | 0.137 | 415 | 100.0 | 100.0 | 371.9 | 1.9 |
| ISS #0 | fibra monomodo (campo = 3.7 urad) | 31.7 | 0.55 | 0.046 | 415 | 100.0 | 100.0 | 300.1 | 0.0 |
| ISS #0 | sin camino no comun ni modelo de apuntamiento | 17.0 | 0.22 | 0.018 | 388 | 100.0 | 100.0 | 422.9 | 370.8 |
| ISS #0 | modelo de apuntamiento SIN calibrar (0.52 deg) | 1841.3 | 51.39 | 4.283 | 2706 | 0.0 | 0.0 | 0.0 | 0.0 |
| ISS #0 | sensor de camara de 1 kHz (cruce del lazo a 80 Hz) | 31.7 | 1.00 | 0.083 | 415 | 100.0 | 100.0 | 422.6 | 87.1 |
| ISS #0 | viento en la montura (0.02 N m rms, el doble del de la campana de atp_ros) | 628.4 | 10.48 | 0.873 | 2402 | 78.8 | 78.8 | 335.5 | 0.7 |

**Linea base analitica: montura cerrada sobre el sensor optico (sin FSM), tilt de 40 cm**

| el | barrido mrad/s | tilt rms urad | f_c tilt Hz | montura cerrada sobre sensor urad | FSM (analitico, solo tilt) urad |
|---|---|---|---|---|---|
| 20 | 6 | 4.81 | 38 | 5.15 | 0.217 |
| 45 | 12 | 3.35 | 37 | 3.58 | 0.151 |
| 60 | 20 | 3.03 | 48 | 3.22 | 0.157 |

**Hacia el cenit: pases sinteticos de 550 km, mascara 10 deg, montura de atp_ros sin estrategia de keyhole**

| el max | dur s | az max deg/s | montura rms/eje urad | montura pico urad | y max urad | FSM enganchado % | con FSM rms urad | cerrado con FSM % | GB con FSM | GB sin FSM |
|---|---|---|---|---|---|---|---|---|---|---|
| 60.0 | 469 | 1.4 | 21 | 366 | 324 | 100.0 | 0.52 | 100.0 | 503 | 128 |
| 80.0 | 476 | 4.5 | 41 | 345 | 328 | 100.0 | 0.52 | 100.0 | 512 | 152 |
| 85.0 | 476 | 9.0 | 52 | 343 | 346 | 100.0 | 0.52 | 100.0 | 512 | 177 |
| 88.0 | 476 | 22.6 | 65 | 625 | 637 | 100.0 | 0.52 | 100.0 | 512 | 201 |
| 89.5 | 476 | 90.4 | 94 | 2074 | 1916 | 99.6 | 0.63 | 99.6 | 510 | 215 |

**Hacia el cenit: pases sinteticos de 550 km, mascara 10 deg, montura de atp_ros sin estrategia de keyhole, rotor de 6 deg/s por eje **

| el max | dur s | az max deg/s | montura rms/eje urad | montura pico urad | y max urad | FSM enganchado % | con FSM rms urad | cerrado con FSM % | GB con FSM | GB sin FSM |
|---|---|---|---|---|---|---|---|---|---|---|
| 60.0 | 469 | 1.4 | 21 | 366 | 324 | 100.0 | 0.52 | 100.0 | 503 | 128 |
| 80.0 | 476 | 4.5 | 41 | 345 | 328 | 100.0 | 0.52 | 100.0 | 512 | 152 |
| 85.0 | 476 | 9.0 | 3912 | 33245 | 41140 | 85.3 | 254.99 | 85.3 | 430 | 120 |
| 88.0 | 476 | 22.6 | 12883 | 122412 | 224848 | 85.5 | 3527.12 | 85.5 | 431 | 145 |

Lectura: (1) **con FSM el error residual es de ~0.5 µrad por eje** en todos los pases, de 0.04–0.05 del campo de 12 µrad; el piso lo ponen el camino no común (±0.5 µrad) y la vibración (0.5 µrad), **no** la turbulencia
(sin ellos: 0.16–0.22 µrad); sin FSM (la montura por programa de efemérides) el error es de 30–60 µrad (**el modelo de apuntamiento ya calibrado con el Sol, 89 µrad por eje, es una realización por pase y domina**), 2.5–5 veces el campo → el enlace
solo cierra a las tasas más bajas: **423 GB con FSM contra 87 sin él** en el pase de 34°, hasta 77 contra 1 en el de 14°. (2) Con un campo más estrecho (4 µrad) el residuo ocupa 0.13–0.14 del campo: el enlace sigue cerrado
pero pierde 10–12 % de datos; con **fibra monomodo** (campo = λ/D = 3.7 µrad, el camino de TBIRD y LCRD) el ATP debe ser submicrorradián y se pierde un 23–30 % de datos con el error actual. (3) Un sensor de **cámara de
1 kHz** (cruce a 80 Hz en vez de 400 Hz) sube el residuo de 0.55 a 0.8–1.0 µrad: aún cabe, pero ya no hay holgura para una fibra monomodo. (4) Con el modelo de apuntamiento **sin calibrar (0.52°)** el
error (1.8 mrad rms en esta realización) se sale de los ±1 mrad del FSM y del campo del sensor: **el enlace no cierra nunca**; es el requisito que la calibración de la calibración con el Sol cubre y la razón de la cámara de adquisición.
(5) El **viento** en la montura (0.02 N m rms, el doble del de la campaña de `atp_ros`) mueve la montura (kp = 25 N m/rad: una montura blanda de plato de 3 m) 0.6 mrad rms por eje con picos de 2.3–2.4 mrad: el FSM pierde el enganche el **18–21 % del pase** (82 y 79 % enganchado), el residuo con FSM sube a 7–10 µrad y los GB bajan de 460 a 378 (ISS #3) y de 423 a 336 (ISS #0): **la rigidez de la montura es un requisito que el FSM no esconde** (±1 mrad de recorrido). (6) **Hacia el cenit** el error de la montura crece (21 µrad rms por eje a 60°, 94 µrad a 89.5°); el pico
de 340–370 µrad hasta 85° es el **arranque** desde el reposo (la montura parte en AOS con el satélite ya moviéndose), no el *keyhole*. Con la montura de `atp_loop` **sin tope de velocidad** el keyhole real aparece a 88° (pico 625 µrad) y a 89.5° (az a 90°/s) el pico de 2.1 mrad
sale del recorrido (se pierde el 0.4 % del pase); pero con el **rotor de 6°/s** el rotor se satura a 85° (az 9°/s): el error es de **3.9 mrad rms** y el FSM pierde el enganchado el **14.7 % del pase**
(430 GB en vez de 512). **El FSM de ±1 mrad no sustituye a la estrategia de keyhole**: absorbe los errores de decenas–cientos de µrad del seguimiento normal, no la saturación del rotor; la estrategia (anticipada / sobre el cenit) sigue siendo necesaria
por encima de ~80° (la ISS a 428 km: 80.3°). (7) El pase de validación del lockstep (ISS 78°, solo sobre 50°) pierde el 0.6 % por el mismo **arranque** (2 mrad de pico: `|y| max` 2042 µrad), no por el cenit.

### 5.4 Sensibilidad a Cn² en tierra y a la apertura

**Sensibilidad a Cn2 en tierra y a la apertura (bajada a 30 deg y a 15 deg de un satelite de 550 km, OOK, error residual 0.6 urad)**

| el | Cn2 suelo | D m | D/r0 | sigma_I^2 | margen desv. dB | P_rx dBm | OOK Gb/s | tilt urad | residuo FSM (solo tilt) urad |
|---|---|---|---|---|---|---|---|---|---|
| 30 | 1.0e-15 | 0.2 | 0.49 | 0.0423 | 2.15 | -34.7 | 5.00 | 1.83 | 0.391 |
| 30 | 1.0e-15 | 0.4 | 0.98 | 0.0148 | 1.26 | -28.7 | 10.00 | 1.63 | 0.179 |
| 30 | 1.0e-15 | 0.6 | 1.47 | 0.0072 | 0.87 | -25.2 | 10.00 | 1.52 | 0.126 |
| 30 | 5.0e-15 | 0.2 | 0.80 | 0.0431 | 2.17 | -34.8 | 5.00 | 2.74 | 0.432 |
| 30 | 5.0e-15 | 0.4 | 1.60 | 0.0149 | 1.26 | -28.7 | 10.00 | 2.44 | 0.196 |
| 30 | 5.0e-15 | 0.6 | 2.39 | 0.0073 | 0.88 | -25.2 | 10.00 | 2.28 | 0.122 |
| 30 | 1.7e-14 | 0.2 | 1.44 | 0.0451 | 2.22 | -34.8 | 5.00 | 4.47 | 0.405 |
| 30 | 1.7e-14 | 0.4 | 2.87 | 0.0151 | 1.27 | -28.7 | 10.00 | 3.98 | 0.203 |
| 30 | 1.7e-14 | 0.6 | 4.31 | 0.0073 | 0.88 | -25.2 | 10.00 | 3.72 | 0.117 |
| 30 | 5.0e-14 | 0.2 | 2.62 | 0.0487 | 2.31 | -34.9 | 5.00 | 7.37 | 0.453 |
| 30 | 5.0e-14 | 0.4 | 5.23 | 0.0153 | 1.28 | -28.7 | 10.00 | 6.56 | 0.199 |
| 30 | 5.0e-14 | 0.6 | 7.85 | 0.0074 | 0.88 | -25.2 | 10.00 | 6.13 | 0.133 |
| 30 | 1.0e-13 | 0.2 | 3.91 | 0.0511 | 2.36 | -35.3 | 5.00 | 10.31 | 0.439 |
| 30 | 1.0e-13 | 0.4 | 7.83 | 0.0153 | 1.28 | -28.8 | 10.00 | 9.18 | 0.219 |
| 30 | 1.0e-13 | 0.6 | 11.74 | 0.0073 | 0.88 | -25.3 | 10.00 | 8.58 | 0.125 |
| 15 | 1.0e-15 | 0.2 | 0.73 | 0.1796 | 4.96 | -38.6 | 0.25 | 2.54 | 0.914 |
| 15 | 1.0e-15 | 0.4 | 1.46 | 0.0736 | 3.02 | -32.6 | 10.00 | 2.26 | 0.446 |
| 15 | 1.0e-15 | 0.6 | 2.19 | 0.0374 | 2.10 | -29.1 | 10.00 | 2.12 | 0.297 |
| 15 | 5.0e-15 | 0.2 | 1.18 | 0.1828 | 5.01 | -38.6 | 0.25 | 3.81 | 0.981 |
| 15 | 5.0e-15 | 0.4 | 2.37 | 0.0737 | 3.02 | -32.6 | 10.00 | 3.39 | 0.461 |
| 15 | 5.0e-15 | 0.6 | 3.55 | 0.0372 | 2.09 | -29.1 | 10.00 | 3.17 | 0.313 |
| 15 | 1.7e-14 | 0.2 | 2.13 | 0.1901 | 5.12 | -38.7 | 0.25 | 6.21 | 0.980 |
| 15 | 1.7e-14 | 0.4 | 4.26 | 0.0734 | 3.01 | -32.6 | 10.00 | 5.53 | 0.521 |
| 15 | 1.7e-14 | 0.6 | 6.39 | 0.0366 | 2.07 | -29.1 | 10.00 | 5.17 | 0.341 |
| 15 | 5.0e-14 | 0.2 | 3.88 | 0.1983 | 5.24 | -39.1 | 0.25 | 10.24 | 1.068 |
| 15 | 5.0e-14 | 0.4 | 7.77 | 0.0709 | 2.96 | -32.7 | 10.00 | 9.12 | 0.473 |
| 15 | 5.0e-14 | 0.6 | 11.65 | 0.0344 | 2.01 | -29.1 | 10.00 | 8.53 | 0.333 |
| 15 | 1.0e-13 | 0.2 | 5.81 | 0.1947 | 5.19 | -39.9 | 0.25 | 14.33 | 1.089 |
| 15 | 1.0e-13 | 0.4 | 11.62 | 0.0656 | 2.84 | -33.4 | 10.00 | 12.76 | 0.481 |
| 15 | 1.0e-13 | 0.6 | 17.43 | 0.0312 | 1.90 | -29.7 | 10.00 | 11.93 | 0.298 |

Lectura: la Cn² en tierra casi no mueve el **centelleo** (lo fijan las capas altas: σ_I² a 30° pasa de 0.0148 a 0.0153 al multiplicar A por 100) pero sí el **tilt** (1.6 → 9 µrad) y r₀; el FSM lo absorbe y el residuo sigue por debajo
de 1.1 µrad. La **apertura** sí manda en el centelleo (promediado: de 0.2 a 0.6 m σ_I² baja 6×) y en la potencia (6 dB por duplicar D en la ganancia de recepción, menos el coste del tilt/Strehl):
20 cm cierra solo 5 Gb/s a 30° con los supuestos de §4, 40 cm llega al tope de 10 Gb/s.

### 5.5 Nubes, diversidad y el RF Ka

**Disponibilidad de linea de vista (fraccion del TIEMPO) con 1..4 estaciones (1 ano horario, 40 semillas)**

| N | estaciones | disponibilidad % (media) | desv. entre anos % | cota independiente % | mejor sola % |
|---|---|---|---|---|---|
| 1 | cdmx | 55.0 | 1.6 | 55.0 | 55.0 |
| 2 | cdmx+madrid | 83.7 | 0.9 | 83.7 | 63.7 |
| 3 | cdmx+madrid+ecuador | 89.5 | 0.7 | 89.4 | 63.7 |
| 4 | cdmx+madrid+ecuador+tromso | 92.6 | 0.6 | 92.6 | 63.7 |

**Pases de NOAA 20 (824 km, polar) en 14 dias: GB entregados por dia con el enlace optico de bajada (288 pases, 24 s de calculo)**

| estacion | pases | el max media | duracion media s | GB por pase (sin nubes) |
|---|---|---|---|---|
| cdmx | 46 | 38.2 | 510 | 397 |
| madrid | 60 | 37.2 | 500 | 341 |
| ecuador | 44 | 36.9 | 510 | 407 |
| tromso | 138 | 40.4 | 556 | 391 |

| N | estaciones | GB/dia posible (sin nubes) | GB/dia optico (media) | % de lo posible | dias sin ningun GB % | GB/dia optico (p10) | GB/dia Ka 0.5 Gb/s 99.9 % (media) | GB/dia Ka (p10) |
|---|---|---|---|---|---|---|---|---|
| 1 | cdmx | 1306 | 747 | 57.2 | 18.0 | 0 | 105 | 80 |
| 2 | cdmx+madrid | 2768 | 1668 | 60.2 | 1.6 | 679 | 238 | 215 |
| 3 | cdmx+madrid+ecuador | 4047 | 2075 | 51.3 | 1.1 | 1038 | 338 | 318 |
| 4 | cdmx+madrid+ecuador+tromso | 7897 | 3249 | 41.1 | 0.4 | 1525 | 681 | 644 |

| estacion sola | CFLOS % | GB/dia posible | GB/dia optico | dias sin datos % |
|---|---|---|---|---|
| cdmx | 55 | 1306 | 747 | 18.0 |
| madrid | 64 | 1462 | 920 | 12.0 |
| ecuador | 35 | 1279 | 408 | 43.4 |
| tromso | 30 | 3850 | 1174 | 23.9 |

RF Ka: margen de lluvia para 99.9 % = 4.0 dB, para 99.99 % = 11.0 dB (templado); ecuatorial 99.9 % = 31 dB.

Lectura: (1) **una estación sola** (CDMX, CFLOS 55 % supuesto) entrega en promedio 747 GB/día con el enlace óptico, **7 veces los 105 GB/día del Ka** de 0.5 Gb/s, pero el 10 % de sus días peores no entrega nada y el
**18 % de los días no entrega ni un GB**; el Ka entrega 80 GB/día en ese mismo 10 % peor (la lluvia lo corta el 0.1 % del tiempo con 4 dB de margen). (2) Con **dos estaciones separadas** (CDMX + Madrid, 9000 km: nubes
decorreladas) la disponibilidad sube de 55 a 84 %, los días sin datos bajan de 18 a 1.6 % y el 10 % peor entrega 679 GB/día (el Ka con las mismas dos estaciones: 215); con tres 90 % y con cuatro 93 % (0.4 % de los días sin datos).
(3) La fracción de lo **posible** que llega baja de 57 a 41 % al añadir estaciones porque las que se añaden son más nubladas (Quito 35 %, Tromsø 30 %, esta última con 138 pases de un satélite polar): se gana volumen y se
pierde certeza por pase. (4) El enlace de **RF** en clima ecuatorial necesita 31 dB de margen de lluvia para 99.9 % (Quito): no hay tabla de GB/día de Ka ahí; las cifras del Ka de arriba son de clima templado (4 dB → 99.9 %) y se aplican a las
cuatro estaciones por simplicidad (**SUPUESTO que favorece al Ka**). (5) La conclusión de red: el gateway óptico es **capacidad con disponibilidad por diversidad** (nubes, decorreladas por estaciones) y el Ka es **disponibilidad con capacidad
limitada** (lluvia, cubierta con margen de enlace); un diseño real combina ambos (el óptico para el volumen, el RF para el mando y el respaldo).

## 6. Contrastes y validación (`pytest src/atp_optico/test`: 67 pruebas)

| Prueba | Contra qué | Cifra |
|---|---|---|
| HV 5/7 | el nombre del perfil (r₀ = 5 cm, θ₀ = 7 µrad a 500 nm) | 4.96 cm, 6.89 µrad (±3 %) |
| Escalas de r₀ y θ₀ | λ^(6/5), sin(el)^(3/5), sin(el)^(8/5) | exacto (1e-9) |
| Rytov, r₀, *beam wander* con Cn² uniforme | formas cerradas 1.23 Cn² k^(7/6) L^(11/6), (0.423 k² Cn² L)^(−3/5), 2.42 Cn² L³ W₀^(−1/3) | 0.5 % |
| Tilt de llegada | independiente de λ; 0.427 (λ/D)(D/r₀)^(5/6) | exacto; 1 % |
| Centelleo | σ_I² → σ_R² a σ_R² ≪ 1; baja con D; A&P contra Churnside por capas | 5 %; factor ≤ 2.5 |
| Gamma-gamma | media 1 y varianza σ_I² por cuadratura; CDF contra Monte Carlo (4e5 muestras); límite log-normal | 3e-3; 1e-2 |
| PPM | SER sin fondo = e^(−n_s)(M−1)/M; contra Monte Carlo con fondo | 1e-6; 4e-3 |
| FEC RS(255,239) | 1e-4 → <1e-12; 1e-3 → ~5e-6 | binomial exacta |
| OPALS | ganancia 70.4 dB; potencia recibida 100 nW–1 µW a 30° | 70.45 dB; 430 nW |
| Cuadre del presupuesto | suma de las partidas en dB = P_rx en dBm | 1e-6 dB |
| Lazo FSM | estable, PM > 45°, GM > 6 dB, \|S\| = f/f_c, S + T = 1; tono contra \|S(f)\| | 2 % |
| Banco del tilt | varianza total, forma de la PSD; rms del residuo simulado contra la integral de \|S\|² PSD | 3 %; 10 % |
| Motor de un paso | reproducible por semilla (campo a campo); el FSM reduce >30×; se satura a 2.5 mrad | — |
| Nubes | marginales exactas; P(ambas) contra la cópula bivariada; persistencia contra arccos(φ)/π | 0.012; 0.01 |
| Red alemana | 4 sitios de 25–65 % → 83.68 % publicado; independiente 88.2 % | 83.8 % (L ajustada a este dato: no es validación independiente) |
| Escenario | determinista byte a byte (JSON y serie); la montura reproduce el error de la de `atp_ros` | idéntico |

**Lockstep ROS (`./run.sh bash scripts/validar.sh optico_ros`)**: ISS sobre CDMX, el pase de 78° (solo el tramo sobre 50°, 1831 pasos de 0.05 s): **dos corridas ROS dan `optico.json` y `optico_serie.jsonl` idénticos byte a byte, y los dos
coinciden con la corrida sin ROS** (`python3 -m atp_optico.escenario`): la montura reproducida sin ROS es **exacta**, no una aproximación (diferencia relativa 0.0 en cada campo del resumen y la serie de 1831 filas igual byte a byte).
La primera versión del nodo **no** era determinista: atendía el motor desde un temporizador del mismo ejecutor y, a máxima velocidad, el lockstep (~3 ms por paso) le sacaba ventaja al motor (~10 ms por paso), así que las colas de 2000 mensajes
se desbordaban y se perdía el principio del pase (1787 y 1781 pasos de 1831 en dos corridas bajo carga). Se corrigió con un hilo que consume la cola, suscripciones de 50 000 y un cierre que espera a emparejar el último paso;
la prueba que lo detectó es exactamente la de determinismo (CONTRATO §17).

## 7. Cómo reproducir

```bash
./run.sh bash scripts/validar.sh optico_test      # 67 pruebas (~1.5 min)
./run.sh bash scripts/validar.sh optico           # pruebas + cifras clave + lockstep ROS x2 + comparacion con la corrida sin ROS
./run.sh ros2 launch atp_optico optico.launch.py time_scale:=0 sat:=ISS station:=cdmx pass_index:=3 D_m:=0.6 salida:=/ws/runs/optico
./run.sh python3 -m atp_optico.escenario --pase_idx 3 --campo_urad 4 --modem ppm     # el mismo escenario sin ROS
./run.sh python3 scripts/optico_estudio.py todo   # las tablas de este documento (~20 min); JSON en docs/optico_resultados/
```

## 8. Qué haría falta en hardware (sin precios)

* **Telescopio**: apertura de 20–60 cm (el estudio cubre 20, 40 y 60 cm) con montura acimutal de **encoder de alta resolución** (la de 14 bits de `atp_ros`, 0.022° = 384 µrad, no basta: el FSM tiene ±1 mrad y el campo es de 12 µrad);
  la montura debe seguir un pase de LEO a hasta varios °/s con un error que quepa en el recorrido del FSM (con la montura sin tope de velocidad el error cabe en los ±1 mrad hasta 88° de elevación máxima; con el rotor de 6°/s se sale a 85°: §5.3).
* **Espejo de dirección fina (FSM)**: piezo o bobina móvil, ±1 mrad en el cielo, resonancia ≥ 2 kHz, lazo de ≥ 400 Hz de cruce (la tabla de §5.3 muestra qué pasa con un sensor de cámara de 1 kHz: cruce de ~80 Hz).
* **Sensor de seguimiento**: cuadrante de InGaAs (QPD) con muestreo de 10 kHz, o cámara de InGaAs con ventana de pocos píxeles a ≥ 2–5 kHz; campo de captura de cientos de µrad; la **adquisición** (espiral con baliza,
  `sat_isl.adquisicion`) necesita además una cámara de campo ancho (mrad) y la efeméride calibrada (el residuo de 89 µrad por eje de la calibración con el Sol cabe en el campo del sensor, pero el modelo **sin calibrar** de 0.52° no).
* **Receptor**: detector de campo (APD de 30–200 µm) para OOK hasta 10 Gb/s, o fibra monomodo con óptica adaptativa/EDFA para coherente (el camino de TBIRD; **no modelado**); detector de conteo de fotones (SNSPD) para PPM de alta sensibilidad.
* **Láser de baliza / subida**: 1 µm o 1.55 µm de decenas de W con divergencia ajustable (el estudio muestra que la divergencia óptima ronda 12–25 µrad con 40 cm), con **coordinación de espacio aéreo y seguridad láser** (ANSI Z136 / normativa
  aeronáutica local) antes de cualquier emisión; apagado automático si hay una aeronave.
* **Sitio**: altitud (la de CDMX baja el centelleo y sube la transmisión), cielo despejado, y **más de una estación** (la nube manda: §5.5).

## 9. Qué aporta a la Estación

* **ATP fino desde tierra**: la montura gruesa de la Estación, tal cual, pasa de cumplir un presupuesto de grados (RF) a ser el generador del error que absorbe un FSM: el residuo medido baja de 30–60 µrad a **~0.5 µrad por eje** (el piso lo ponen el camino no común y la vibración, no la turbulencia). El FSM no resuelve el *keyhole*: con una montura sin tope de velocidad cabe hasta ~88° de elevación máxima, pero con un rotor de 6°/s el enganchado se pierde en una fracción del pase.
* **Capacidad del enlace frente a disponibilidad**: un gateway óptico entrega ~7× los GB/día del Ka a una estación pero un 18 % de los días no entrega nada; con dos estaciones separadas la disponibilidad sube de 55 a 84 %: el costo de la nube se paga con **estaciones**, no con margen de enlace.

## 10. Limitaciones y lo no verificado

* **Ninguna cifra de hardware**: todo es simulación; los parámetros del satélite, del telescopio, del FSM y del sensor son **SUPUESTOS** declarados en §4. La atmósfera (HV 5/7) es un perfil medio: un sitio real puede ser 2–3× peor o mejor.
* **CFLOS de CDMX, Tromsø y Quito: SUPUESTOS**; la longitud de correlación de nubes (500 km) está **ajustada** a un solo dato publicado (red alemana), y las series son **sintéticas** (cópula gaussiana con memoria AR(1)), no cobertura nubosa real; no hay ciclo diurno ni estacional.
* Las fuentes marcadas "(buscado)" son **resúmenes**, no los artículos; las fórmulas "de memoria" se verificaron con pruebas contra formas cerradas, pero **no** contra el texto de A&P; la fórmula de promediado por apertura de A&P y la de Churnside
  difieren hasta un factor 2.4 (se usó la pesimista).
* **No modelado**: óptica adaptativa y acople a fibra monomodo en bajada con D/r₀ > 3 (solo la aproximación del Strehl de §3.1); recepción coherente (TBIRD); el modelo de anisoplanatismo del tilt de la subida (parámetro `corr_tilt_up`); la **adquisición** (espiral con baliza);
  nubes finas (cirros) que atenúan sin cortar; turbulencia de capa limite del propio telescopio (cúpula); el ruido de fondo diurno del sensor; la escala externa en el tilt (conservador); la refracción diferencial.
* La **montura** es la de `atp_ros` sin viento, fricción ni holgura (variante con viento en §5.3); su error se muestrea a 20 Hz (el contenido sobre 10 Hz no se ve; el servo es de ~0.5 Hz, y la serie de 2 ms no se publica por ROS).
* El **PPM** usa la capacidad de decisión dura (cota) y un detector de conteo con η y fondo supuestos; los parámetros de los Libros Azules de CCSDS no se abrieron.
* La **comparación con Ka** es de orden de magnitud: tasa fija supuesta (0.5 Gb/s), lluvia por una interpolación entre dos puntos de la literatura y sin modelar nubes sobre el Ka ni la dependencia de la elevación de la lluvia.
