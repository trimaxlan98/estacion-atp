# Planificador de contactos de la red (equipo F)

Qué responde: dada una red de M estaciones y N satélites, **qué contactos
hacer, con qué antena y cuántos datos bajan**. Código: `src/atp_ros/atp_ros/planificador.py`
(numpy, sin ROS), nodo `contact_scheduler`, script `scripts/planificar.py`,
pruebas `src/atp_ros/test/test_planificador.py`.

## 1. Modelo

```
TLE ─SGP4─▶ pases (orbit.predecir_pases) ─▶ ventanas (satélite, estación, AOS, LOS)
                          │
                          ├─ rango oblicuo real ─▶ C/N0 (model.presupuesto_enlace)
                          │                      ─▶ Es/N0 = C/N0 − 10 log10 Rs
                          │                      ─▶ MODCOD DVB-S2 (ACM) ─▶ tasa = Rs·η
                          │                      ─▶ ∫ tasa dt  = volumen del contacto
                          └─ az/el(t) ─▶ keyhole del rotor (marcar / recortar / descartar)
ventanas ─▶ restricciones ─▶ plan  (codicioso | óptimo)
```

### Ventanas
Un pase por (satélite, estación) con máscara `max(máscara de la estación, el_min_deg)`;
los tiempos son exactamente los de `orbit.predecir_pases` (la prueba compara AOS, LOS,
elevación máxima y acimutes pase por pase). Se ordenan por (AOS, satélite, estación).

### Enlace y volumen de datos
* `C/N0 = model.presupuesto_enlace(d_oblicuo, error_apunt, …)` con el rango del pase
  muestreado cada 1 s. `Es/N0 = C/N0 − 10 log10(Rs)`.
* **ACM ideal**: en cada instante se elige el MODCOD de mayor eficiencia espectral cuya
  Es/N0 ideal **+ margen** (1.0 dB por defecto: pérdida de implementación e histéresis)
  se cumple. Tasa de información = `Rs · η`. La eficiencia no es monótona en Es/N0
  (16APSK 2/3 domina a 8PSK 5/6), por eso se toma el máximo de los factibles.
* **Volumen** = integral por trapecios de la tasa sobre `[AOS, LOS]` (menos `t_adq_s`
  de adquisición, 0 por defecto). La tasa ACM es escalonada: el error de trapecios es
  ≈ salto·dt/2 por cambio de MODCOD; la prueba contra una integración independiente
  (punto medio, malla de 0.25 s, fórmulas de definición) coincide a mejor de 0.5 %.
* **Cota de Shannon** para la misma `Rs`: `Rs·log2(1 + Es/N0)`, integrada igual. Todo
  volumen queda por debajo (en el estudio, el ACM logra el 74 % de la cota).

**Fuente de la tabla MODCOD**: ETSI EN 302 307 V1.2.1 (2009-08), Tabla 13 (Es/N0 ideal
para PER 10⁻⁷ en AWGN, FECFRAME normal de 64 800 bits, sin pilotos; η por unidad de tasa
de símbolos, incluye el overhead de trama). **Verificada**: los 28 valores se extrajeron
con `pdftotext` del PDF oficial de ETSI (`etsi.org/deliver/etsi_en/302300_302399/302307/01.02.01_60/`),
no de memoria. Como control independiente, la prueba `test_modcod_bajo_shannon` exige
que cada η quede bajo `log2(1+Es/N0)` a su Es/N0 y a menos de 1.5 dB de ella.
La norma advierte que con FECFRAME corto hay 0.2–0.3 dB de degradación adicional (no
modelado: se usa FECFRAME normal).

### Restricciones
1. **Una antena por estación.** Entre dos contactos consecutivos `a → b` de la misma
   estación: `AOS_b ≥ LOS_a + max(|ΔAz|/vel_az, |ΔEl|/vel_el) + t_asent`, con `ΔAz` por el
   camino corto (≤ 180°), `vel_az = 6 °/s`, `vel_el = 3 °/s`, `t_asent = 5 s` (suposiciones
   del escenario; rotor de acimut sin tope de giro).
2. **Un satélite no se descarga a dos estaciones a la vez** (un solo enlace de bajada,
   datos distintos): dos ventanas del mismo satélite que se solapan chocan (`sep_sat_s`
   opcional).
3. **Keyhole.** `model.elevacion_keyhole(h, vel_az)` con `h` = altitud en el TCA
   marca como `keyhole` los pases que culminan por encima del umbral (ISS ≈ 80°). Además
   se mide la demanda real de la trayectoria SGP4 (|az'| > `vel_az` o |el'| > `vel_el`):
   `t_perdido_s` es el tiempo en que el rotor no puede seguir; con `keyhole=recortar` ese
   tramo no cuenta en el volumen; con `descartar` el pase no entra. La prueba verifica que
   la marca de la fórmula cerrada coincide con la demanda real salvo ±1.5° del umbral.
   El re-apuntamiento tras un pase cenital NO modela la recuperación del keyhole.

### Algoritmos (peso = prioridad × GB)
* **Codicioso**: ordena por `peso`, `densidad` (peso/s) o `aos` y acepta cada ventana si
  cabe en su estación (a ambos lados) y en su satélite. Es el que usaría una operación
  sencilla; no es óptimo.
* **Óptimo propio (`bb`)**: ramificación y acotamiento sobre los choques de satélite. Quitando
  esa familia de restricciones, cada estación es independiente y se resuelve **exacta** por
  programación dinámica (secuencia de peso máximo con tiempos de re-apuntamiento); su suma es
  una cota superior válida. Si la solución relajada tiene dos ventanas simultáneas del mismo
  satélite `(a, b)`, se ramifica en "sin a" / "sin b" (mejor cota primero, determinista).
  Solo Python; el corte es por nodos, no por reloj.
* **Óptimo por programación entera (`milp`)**: `scipy.optimize.milp` (HiGHS, scipy 1.11.4 en
  la imagen) con `x_i + x_j ≤ 1` para cada par que no puede convivir. Se usa cuando `bb`
  no demuestra el óptimo en `limite_nodos`. `auto` = `bb` y, si no prueba, `milp`.
* **Fuerza bruta** (`optimo_fuerza_bruta`, 2ⁿ, n ≤ 22): solo como referencia de las pruebas.

`optimo_probado` es `True` solo si el algoritmo demostró el óptimo; si no, `cota_sup` acota.

## 2. Validación (`pytest src/atp_ros/test/test_planificador.py`: 29 pruebas)

| Qué | Cómo |
|---|---|
| Tabla MODCOD | valores puntuales de ETSI; cada η < Shannon a su Es/N0 |
| Selección ACM | umbrales exactos (1.00 dB → QPSK 1/2, 0.99 → QPSK 2/5), margen, vectorizado = escalar, 16APSK 2/3 domina a 8PSK 5/6 |
| Enlace | C/N0 contra el cálculo a mano (FSPL = 20 log 4πdf/c, G = η(πDf/c)², k = 1.380649e-23) a 0.01 dB |
| Ventanas | AOS/LOS/el_max/acimutes iguales a `predecir_pases` en 3 sats × 3 estaciones; Tromsø no ve la ISS |
| Volumen | integración independiente (punto medio, 0.25 s) a < 0.5 %; reimplementación escalar con las mismas muestras a 1e-9; EIRP saturado × 2 Rs = 2× volumen; nunca > Shannon |
| Keyhole | pase cenital (el_max 89.98°) marcado, recortado y descartado; el volumen recortado = completo − t_perdido·tasa tope (2 %) |
| Restricciones | planes reales (codicioso ×3, óptimo) revisados con una función escrita aparte en la prueba; `verificar_plan` detecta solape, re-apuntamiento corto (34 s vs 35 s) y satélite doble |
| Re-apuntamiento | camino corto (350°→10° = 20°, no 340°), eje de elevación, frontera exacta de 35 s |
| Óptimo | 60 instancias aleatorias de 11–14 ventanas: `bb` = fuerza bruta y ≥ cada codicioso (el codicioso queda estrictamente por debajo en > 5); 30 instancias de 40 ventanas: `bb` = `milp`; 15 ventanas reales SGP4 contra fuerza bruta; trampa clásica (10 vs 6+6) |
| Determinismo | `calcular_plan` dos veces → JSON idéntico byte a byte; `subred` = construir directo |

Medido además (no en las pruebas): sobre las instancias reales del estudio, `bb` y `milp`
dan el mismo valor en todas las subredes que `bb` logra probar (97 ventanas: 404 nodos, 0.1 s;
57 ventanas: 2146 nodos). Con las 173 ventanas, tras 5000 nodos la cota de `bb` sigue 4.6 % sobre el óptimo
(365.8 contra 349.6 GB) y **no** lo prueba: ahí manda `milp` (0.03–0.5 s). Es el límite honesto de
mi óptimo propio sin scipy.

Salida real de la suite completa: ver el informe del equipo (sección "Pruebas").

## 3. Estudio de caso (24 h)

Reproducible: `./run.sh python3 scripts/planificar.py --sats "ISS,NOAA 15,NOAA 18,NOAA 19,NOAA 20,NOAA 21,METEOR-M2 3,METEOR-M2 4" --estaciones cdmx,tromso,ecuador,madrid --inicio 2026-10-02T12:00:00 --horas 24 --sin tromso`
(≈ 35 s; guarda `runs/<id>/plan.json` y `resumen.txt`; copia del resumen en
[`estudio-planificador-resumen.txt`](estudio-planificador-resumen.txt)).

Escenario: TLE de `data/tle` (época 2026-10-01, edad 0.5–0.7 d al inicio), estaciones de
`data/estaciones.yaml` (CDMX, Tromsø, Quito con máscara 10°, Madrid), carga útil **hipotética
idéntica en todos los satélites**: banda X (8.4 GHz), EIRP 12 dBW, 10 Mbaud, plato de 3 m,
G/T 24.4 dB/K, pérdidas fijas 3 dB, ACM con margen 1 dB, sin atmósfera. Prioridades iguales
(peso = GB). **Los GB son del escenario, no telemetría real de esos satélites**: ni la ISS ni los
NOAA/METEOR operativos bajan en X a 10 Mbaud con ese EIRP.

| | contactos | minutos | GB |
|---|---:|---:|---:|
| Ventanas disponibles (sin la restricción de una antena) | 173 | 1758.2 | 516.3 |
| Codicioso por peso | 105 | 1073.0 | 320.4 |
| Codicioso por densidad | 108 | 1101.8 | 329.2 |
| Codicioso por orden de llegada | 107 | 1063.8 | 314.7 |
| **Óptimo (HiGHS, probado)** | **115** | **1178.4** | **349.6** |

Cota de Shannon del plan óptimo: 474.8 GB (el ACM obtiene el 73.6 %).
**Brecha del codicioso por peso: 8.35 %** (densidad: 5.84 %; llegada: 9.98 %).

GB por satélite y por estación en el plan óptimo:

| satélite | contactos | min | GB | | estación | contactos | min | GB |
|---|---:|---:|---:|---|---|---:|---:|---:|
| ISS (ZARYA) | 12 | 79.8 | 26.31 | | CDMX | 26 | 255.8 | 75.41 |
| METEOR-M2 3 | 14 | 153.2 | 45.88 | | Tromsø | 42 | 485.3 | 142.80 |
| METEOR-M2 4 | 14 | 138.9 | 41.16 | | Quito | 18 | 150.2 | 46.54 |
| NOAA 15 | 11 | 113.6 | 33.13 | | Madrid | 29 | 287.2 | 84.83 |
| NOAA 18 | 17 | 183.6 | 53.38 | | | | | |
| NOAA 19 | 16 | 179.4 | 52.83 | | | | | |
| NOAA 20 (JPSS-1) | 17 | 182.9 | 54.06 | | | | | |
| NOAA 21 (JPSS-2) | 14 | 147.1 | 42.84 | | | | | |

### Beneficio de Tromsø

| Red | contactos | minutos | GB (óptimo) | brecha del codicioso |
|---|---:|---:|---:|---:|
| CDMX + Quito + Madrid | 81 | 758.0 | 224.98 | 0.13 % |
| + Tromsø | 115 | 1178.4 | 349.58 | 8.35 % |
| **ganancia** | **+34** | **+420.4** | **+124.6 GB (+55.4 %)** | |

Tromsø *sola* (76 ventanas) lograría 150.4 GB: más que cualquiera de las otras tres y el 67 % de lo
que consiguen las otras tres juntas. **Por qué**: (1) los NOAA/METEOR son heliosíncronos
(i ≈ 98.7°) y sus órbitas convergen hacia el polo: a 69.6° N Tromsø ve **casi cada una de las
~14 órbitas diarias** (10–12 pases por satélite) frente a 2–6 en CDMX/Quito/Madrid; (2) esos
pases son más largos y altos (media 672 s y 37.8° de elevación máxima contra 513–585 s y
31–38° en las demás) porque la traza pasa cerca del cenit; (3) hay 76 ventanas (851 min) en una
sola antena, lo que además **hace el problema duro**: la brecha del codicioso pasa de 0.13 % a
8.35 %, porque con ventanas densas y solapadas escoger mal bloquea dos buenas. (4) Contrapunto:
la ISS (i = 51.6°) **no** es visible desde Tromsø (a 5° de máscara su cobertura llega a ~67.5° de latitud, geometría esférica) y no
aporta nada allí; el beneficio es solo para órbitas polares. Tromsø también concentra 3 pases por
encima del keyhole de 6 °/s (el rotor los marca).

### Sensibilidad (misma red, 4 estaciones)
* Rotor lento (1 °/s en ambos ejes, asentamiento 15 s): óptimo 341.9 GB (−2.2 %), brecha del
  codicioso 7.81 %: con estos pases el re-apuntamiento lento cuesta poco (no se analizó
  por qué en detalle).
* Prioridades ISS = 3, METEOR-M2 3 y 4 = 2: óptimo 346.7 GB, valor ponderado 502.7, brecha
  3.04 %; la ISS recupera sus 29.0 GB posibles (de 26.3).

## 4. Supuestos y límites (cada cifra vale solo dentro de esto)

* **Enlace**: espacio libre + pérdidas fijas, sin gases, lluvia, nubes ni refracción (a baja
  elevación en X/Ka el volumen está sobrestimado). Apuntamiento perfecto (0°; los experimentos
  S4/N5 dan 0.001–0.004° rms). ACM instantáneo, sin latencia de adaptación. Rs fija; `η` no
  incluye protocolos de capas superiores (CCSDS, IP).
* **Carga útil**: la misma para todos; con perfiles por satélite (`perfiles=`) o por EIRP se
  cambian. Sin memoria a bordo: el satélite siempre tiene datos que bajar.
* **Órbitas**: TLE con error ~1 km en la época y 1–3 km/día después; AOS/LOS a ~1–2 s por pase
  (ver `orbit.py`). No se modela maniobras ni eclipses (potencia).
* **Rotor**: velocidades constantes, acimut sin tope de giro, sin aceleración. Aproximación
  adecuada con rotores de 6 °/s y pases de ≥ 100 s; con contactos de < 30 s, la desigualdad
  triangular del re-apuntamiento puede fallar y el MILP (restricciones por pares) sería algo
  más estricto que el `bb` (restricciones entre consecutivos).
* **Red**: una antena por estación, sin pre-posicionamiento previo al primer pase, sin antenas
  compartidas ni enlaces entre satélites; la restricción de satélite supone un solo enlace
  de bajada con datos distintos por estación.
* **Óptimo**: exacto para el objetivo (suma de pesos) y estas restricciones; no minimiza
  contactos ni reparte equitativamente entre satélites (usar `prioridades`).
* Tiempo de cómputo: 24 h × 4 estaciones × 8 satélites ≈ 35 s, dominado por SGP4 en `predecir_pases`.

## 5. Uso

```bash
./run.sh python3 scripts/planificar.py --sats "ISS,NOAA 19" --estaciones cdmx,tromso \
    --inicio 2026-10-02T12:00:00 --horas 24 --sin tromso
./run.sh ros2 run atp_ros contact_scheduler --ros-args -p sats:="ISS,NOAA 19" \
    -p stations:="cdmx,tromso" -p start_utc:=2026-10-02T12:00:00 -p hours:=24.0
./run.sh ros2 service call /atp/plan_contacts std_srvs/srv/Trigger "{}"
./run.sh ros2 topic echo --once --qos-durability transient_local /atp/contact_plan
```

El nodo publica `/atp/contact_plan` (String JSON, latched) y expone `/atp/plan_contacts`
(Trigger). Parámetros: `sats`, `stations`, `hours`, `start_utc` (vacío = ahora; con fecha fija
el JSON es idéntico), `band`, `eirp_dbw`, `rs_baud`, `margen_acm_db`, `vel_az_deg_s`,
`vel_el_deg_s`, `t_asent_s`, `keyhole`, `el_min_deg`, `sep_sat_s`, `algoritmo`
(`codicioso|optimo|ambos`), `metodo` (`auto|bb|milp`), `prioridades`, `tle_dir`,
`stations_file`. Corre con el reloj de pared (no usa `/clock`; CONTRATO 8 no aplica).

## 6. En la Estación (equipo L, ronda 3)

Hecho: `kind: plan` en el catálogo y el lanzador (`experiments/s6-plan-24h.yaml`: la red de 4 estaciones, los
NOAA/METEOR/ISS y la fecha fija del estudio). El backend ejecuta `scripts/planificar.py --k=v … --id=<run_id>
--salida=<runs>`, el resultado es `runs/<id>/plan.json` y las `metrics` de la corrida son el bloque
`planes.optimo.resumen` más `brecha_codicioso_pct` (CONTRATO 3.2 y 4.1). La vista **Plan de contactos**
(`#/plan`) dibuja el Gantt (una fila por estación, barras por contacto coloreadas por satélite, con GB y elevación
máxima; en gris los pases descartados), la tabla de GB por satélite y estación y la comparación con/sin una estación.

Un cambio mínimo en este equipo: `scripts/planificar.py` guarda además `aporte_estacion.<est>.ids_sin` (el plan óptimo
sin esa estación, con los ids de `ventanas`; `subred` los renumera), para poder dibujar «el plan sin Tromsø» en el Gantt.
El resto de `plan.json` no cambió (comprobado: idéntico salvo esa clave).
