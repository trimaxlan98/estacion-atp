# La misión integrada: un día de operaciones de la red (equipo P)

Qué responde: **¿funcionan encadenadas todas las piezas de la Estación?** Un operador de una red real
planifica el día, ejecuta cada contacto, determina la órbita con lo que midió y la usa en los pases siguientes,
y al final cuenta cuántos datos bajó de verdad. Aquí eso corre de punta a punta, con cada contacto como una
corrida en lockstep con el pase real SGP4 de ese satélite sobre esa estación.

Código: `src/atp_mision/` (paquete nuevo, Python puro más un launch), `scripts/mision.py`, pruebas
`src/atp_mision/test/` (29, con corridas simuladas), experimento `experiments/m1-dia-operaciones.yaml`
(`kind: mision` de la Estación, vista `#/mision`), contrato en `docs/CONTRATO.md` (§1, 3.3, 4.2, 10).

```
                 ┌──────────── 1. plan (atp_ros.planificador: óptimo DVB-S2, cargas por satélite) ─────────────┐
                 ▼                                                                                            │
 TLE ─ SGP4 ─▶ contactos asignados ──▶ 2. UNA corrida por contacto (ros2 launch atp_mision mision_pase)        │
 (lo que el                         en paralelo, ROS_DOMAIN_ID propio (pool 200-229), lockstep (CONTRATO 8)    │
  operador sabe)                      reference_generator: predice con TLE u órbita estimada (profile tle|od)  │
                                      + satélite VERDADERO distinto (truth_file) ─▶ sat_state / truth          │
                                      autotrack (rf_beacon+conical scan) si la banda lo pide o es calibración  │
                                      sdr_receiver (IQ, compuerta del reloj) en la calibración UHF/S           │
                                      monitor + enlace + grabador: error de apuntamiento y C/N0 MEDIDOS        │
                 ┌────────────────────────────────────────────────────────────────────────────────────────────┘
                 ▼
 telemetry.jsonl de cada corrida ─▶ 3. OD (atp_ros.od, por lotes) con los ángulos de la montura enganchada
                 │                  (+ Doppler del receptor) de los n_od primeros pases útiles de cada satélite
                 │                  ─▶ od.json ─▶ los contactos siguientes del satélite vuelan con profile:=od
                 ▼
 4. GB = ∫ tasa ACM(C/N0 medido) dt  ─▶ 5. informe: plan contra ejecutado, minutos, GB, pérdidas por apuntamiento,
                                           mejora por la OD (gemelos con TLE), incidencias, verificación de totales
```

## 1. Qué se reutiliza y qué es nuevo

| Pieza | De quién | Cómo entra |
|---|---|---|
| Planificador de contactos | `planificador.py` (F) | `calcular_plan` con una `PerfilEnlace` por satélite (banda, EIRP, Rs); se ejecuta el plan **óptimo** |
| Pase real SGP4 | `orbit.py`, `profile:=tle` | cada contacto: `start_utc` = AOS − 40 s, `pass_index 0` |
| Lazo en lockstep | `atp_loop`: reloj, generador, montura, PID, monitor, enlace, grabador | los mismos nodos, en `launch/mision_pase.launch.py` |
| Autotrack | `autotrack`, `rf_beacon` (N5) | con verdad externa; banda X/Ka y calibración |
| Receptor SDR | `atp_sdr` (M) | `sdr_receiver` + `sdr_gate` en los contactos de calibración UHF/S |
| OD | `od.py`, `propagador.py`, `profile:=od` (N) | ajuste por lotes sobre la telemetría registrada |
| Backend de la Estación | `runner.py` (pool de dominios, subproceso sin shell, SIGINT→SIGKILL) | mismo patrón en `ejecucion.py`, con hilos y pool 200-229 |
| **Nuevo** | | verdad distinta del TLE (`mundo.py`), contabilidad con lo medido (`contabilidad.py`), OD desde la telemetría (`od_mision.py`), orquestador (`mision.py`), informe, `kind: mision` |

Cambios en archivos compartidos (todos aditivos, apagados por defecto: sin `truth_file` el lazo es el de siempre):
`reference_generator` (parámetro `truth_file`: publica `/atp/truth` y pone el rango y la tasa del satélite
verdadero en `/atp/sat_state`), `rf_beacon` (parámetro `external_truth`), `od.py` (`guardar_efemeride`,
`cargar_efemeride`, `rho_var` en el modelo serializado).

## 2. El mundo: lo que se sabe y lo que es

Una misión solo tiene sentido si la predicción y la realidad difieren (si no, el seguimiento no tiene nada que
corregir ni la OD nada que mejorar). Cada satélite tiene el **TLE** (lo que el operador sabe) y la **verdad**: el
escenario de `docs/OD.md` 3.1 (estado SGP4 de la época del TLE más un desfase de época de ~1 km a lo largo de la pista,
propagado con J4 y arrastre con BC +25 % y variación temporal de la densidad). El código de la misión no usa la verdad
para estimar ni decidir: solo la corrida (lo que ve la estación) y el informe (para medir errores).

En la corrida el generador predice con el TLE (o la órbita estimada) y mueve la **referencia**; el satélite que ve la
estación es el verdadero: `/atp/sat_state` (rango, tasa de rango), `/atp/truth` (dirección). El monitor mide contra la
verdad, el enlace calcula con el error y el rango de ese instante, el receptor SDR y el beacon del autotrack usan lo mismo
(CONTRATO 11). Si la efeméride está dentro del cono de captura (0.913 θ3) el autotrack engancha y la montura sigue al
satélite; si no, queda en *program track* con el error de efeméride.

## 3. Contactos, papeles y modos

* **Cargas útiles hipotéticas** (ni la ISS ni los NOAA/METEOR bajan así), una por satélite, en bandas verosímiles por
  familia: ISS y NOAA 15/18/19 en **S** (EIRP 10 dBW, 4 Mbaud, plato de 3 m, θ3 = 3.18°); NOAA 20/21 y METEOR-M2 en **X**
  (12 dBW, 10 Mbaud, θ3 = 0.83°). ACM DVB-S2 con margen de 1 dB, sin atmósfera. `mundo.CARGAS_M1`.
* **Calibración** (`rol: cal`): los `n_od = 3` primeros contactos de cada satélite con elevación máxima de 15° a 85° y
  sin keyhole. Autotrack forzado (la montura mide al satélite solo si engancha) y receptor SDR si la banda es UHF/S.
* **Seguimiento**: autotrack si el contacto es de calibración o la banda es X/Ka (haz < 1°: con el TLE el program track
  pierde la mitad del haz); program track en UHF/S.
* **Efeméride**: un contacto vuela con la órbita estimada cuando hay `n_min_od = 3` calibraciones con medidas válidas
  que terminaron al menos `latencia_od_s = 600 s` antes de su AOS y la OD se acepta; si no, con el TLE. Un contacto
  espera a las calibraciones de su satélite que pueden alimentarlo (los demás corren libres).
* **Rotor**: el de la planificación (6 °/s; `vel_max` de `mount_sim`), así los pases por el keyhole pierden seguimiento de
  verdad y el plan (que los *marca*, `keyhole: marcar`) se compara con lo medido.
* **Ventanas cortadas por el horizonte**: un pase ya en curso al inicio arranca en el AOS del plan; uno que cruza el
  final cuenta solo hasta el horizonte (la ventana del plan está cortada igual).

## 4. La OD con lo registrado (`od_mision.py`)

De la telemetría de cada calibración (`telemetry.jsonl`, no de la verdad):

* **Acimut y elevación**: los ángulos de la montura (`az`, `el`), una muestra cada 5 s, solo si el autotrack anunció
  ENGANCHE (el aviso del log; si no, la corrida no aporta medidas: los ángulos serían los de la efeméride), pasados 15 s del
  AOS (convergencia del lazo), con menos de 0.5 dB de pérdida por apuntamiento y elevación de 3° a 85° (cerca del cenit
  el acimut no sirve). σ = 1.5 % del ancho de haz de la banda (0.048° en S, 0.012° en X; el autotrack mide 1.3 % en las
  corridas, 1.7 % en N5).
* **Doppler** (solo con receptor SDR en ese contacto): el del enlace más 0.05 m/s con semilla fija (`docs/OD.md` 1.3).
  El rango **no** se usa (la estación no tiene transpondedor con *ranging*).
* **Ajuste** por lotes (`od.ajuste_lotes`, Levenberg-Marquardt) con todas las calibraciones disponibles, el modelo del
  estimador (J2 + arrastre nominal) y el prior del TLE; no estima el arrastre (`docs/OD.md` 3.3).
* **Aceptación**: converge, χ²/dof < 5, σ de posición formal < 5 km, órbita a menos de 100 km del TLE; si no, el día sigue
  con el TLE y queda como incidencia. La covarianza formal no cubre el error de modelo (`docs/OD.md` 3.4): la mejora se
  mide contra la verdad, en el informe.

## 5. Contabilidad, informe y verificación

* **GB** de cada contacto: la telemetría de su corrida (10 Hz) trae el C/N0 que el enlace calculó con el error de
  apuntamiento medido y el rango verdadero; Es/N0 = C/N0 − 10 log Rs, ACM ideal (el MODCOD de mayor eficiencia con Es/N0 ≥
  ETSI + 1 dB), tasa = Rs · η, volumen = ∫ tasa dt. `gb_sin_apunt` repite la cuenta quitando `l_point_db` de cada muestra:
  `perdida_apunt_gb` = lo que costó el apuntamiento (error de efeméride + seguimiento + rotor que no alcanza).
* **Mejora por la OD**: cada contacto volado con la órbita estimada y en *program track* tiene un **gemelo**: el mismo
  contacto con TLE (misma verdad, misma geometría; solo cambia la efeméride). Para todos los contactos, además, el error de
  efeméride se mide analíticamente contra la verdad (`od.resumen_error_pase`).
* **Informe** (`informe.json`, CONTRATO 4.2; `informe.txt`): plan contra ejecutado (contactos, minutos, GB), por satélite y
  por estación, cada contacto, la OD de cada satélite (con su error real de posición contra el del TLE), la mejora por la
  OD, incidencias. No lleva ningún tiempo de pared ni ids aleatorios: con la misma configuración es idéntico byte a byte.
* **Verificación**: los totales se vuelven a calcular desde cero (`contabilizar_a_mano`: bucle escalar, tabla ETSI fila a fila)
  sobre el `telemetry.jsonl` de cada carpeta y deben cuadrar con los del informe (`verificacion.cuadra_gb`).

## 6. Ejecución en paralelo

`ejecucion.EjecutorROS`: un subproceso `ros2 launch atp_mision mision_pase.launch.py` por corrida, con `ROS_DOMAIN_ID`
tomado del pool **200-229** (propio: la Estación usa 100-199 y las pruebas 61-99) y devuelto al terminar; grupo de procesos
propio, SIGINT y luego SIGKILL al vencer el plazo, barrido final. `Mision.ejecutar` respeta `--concurrencia` y las
dependencias de calibración; una corrida que falla se reintenta una vez y, si vuelve a fallar, queda como incidencia sin
tumbar el día. El lockstep hace que la carga de la máquina no cambie las cifras, solo el tiempo.

## 7. Uso

```bash
./run.sh python3 scripts/mision.py --id m1 --salida runs                      # el día de 24 h (M1)
./run.sh python3 scripts/mision.py --sats "ISS,NOAA 20" --estaciones cdmx,madrid --horas 3 \
    --n-od 2 --n-min-od 2 --el-cal-min 8 --latencia-od 300 --id corta --salida runs      # integración corta
./run.sh bash scripts/validar.sh mision_test     # pruebas de orquestación (sin ROS)
./run.sh bash scripts/validar.sh mision          # misión corta real dos veces + determinismo
```

Desde la Estación: experimento `m1-dia-operaciones` (nivel M1), vista **Misión** (`#/mision`).

## 8. Operación segura (no congelar el host)

El 2-oct-2026 una misión con `--concurrencia 6` lanzada con `docker compose run -d`
(sin límites) dejó la máquina de 8 hilos / 11 GB sin responder y hubo que reiniciar.
Reglas desde entonces:

- **Lanzar por la Estación** (`./station.sh`, kind `mision`): tiene cola, tope de
  corridas, timeout y limpieza de procesos. No lanzar misiones largas con
  `docker compose run -d` sueltos.
- `scripts/mision.py` recorta `--concurrencia` a la mitad de los hilos (aviso por
  stderr); `--forzar` o `ATP_MAX_CONCURRENCIA=N` lo cambian a conciencia.
- `docker-compose.yml` fija `cpus`, `mem_limit`, `pids_limit` y `nofile` en `ros`,
  `station` y `gz`: una corrida desbocada se ahoga en su contenedor, no en el host.
- `scripts/vigilante_host.sh` (servicio `scripts/atp-vigilante.service`) detiene los
  contenedores ROS si la carga o la memoria libre se pasan de umbral 3 lecturas
  seguidas.
- En el host conviene `earlyoom` (`sudo apt install earlyoom`) como red de seguridad.
