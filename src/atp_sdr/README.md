# atp_sdr — la cadena de RF a nivel de muestras IQ

Hasta ahora el enlace de la Estación era un **presupuesto**: C/N0 y Eb/N0 en
dB, el Doppler como fórmula y el autotrack con una potencia ideal en dB. Este
paquete baja a la **señal**: un beacon de satélite generado como banda base
compleja a nivel de muestras (Doppler y tasa Doppler de un pase real, ruido
AWGN fijado por el C/N0 del presupuesto, defectos del oscilador) y un receptor
como el de una estación con SDR (adquisición por FFT, seguimiento FLL/PLL,
detector de enganche, estimación de C/N0; en BPSK, filtro adaptado, reloj de
símbolo y Costas). Cada cifra se contrasta con su fuente o su derivación.

Todo el código del receptor es numpy puro y consume arreglos de muestras: es el
mismo que correría sobre un SDR real (ver [Qué cambia con un SDR real](#qué-cambia-con-un-sdr-real)).

## Qué hay

| Archivo | Qué hace |
|---|---|
| `atp_sdr/senal.py` | generador: portadora / BPSK / QPSK con RRC, Doppler lineal por bloque con fase continua, piso de ruido fijo, offset, deriva y ruido de fase del oscilador; Q(x), BER teórica, intervalo de Wilson |
| `atp_sdr/receptor.py` | adquisición FFT (CFAR), `SeguidorPortadora` (NCO + integración y descarga + lazo de 2.º/3.er orden con ayuda de FLL), detector de enganche, M2M4, NWPR, C/N0 espectral, B_L efectivo del lazo digital |
| `atp_sdr/bpsk.py` | filtro adaptado, Gardner, Costas, `ReceptorBPSK` (etapa FLL por cuadrado + reloj + Costas), alineación y BER |
| `atp_sdr/cadena.py` | `CadenaRF`: un paso de 50 ms = un bloque IQ generado y procesado; `simular_pase`; CLI `sdr_offline` |
| `atp_sdr/nodo_sdr.py` | nodo `sdr_receiver` (lockstep, pasivo, mensaje `sdr_interfaces/SdrStatus` en `/atp/sdr`) |
| `atp_sdr/nodo_gate.py` | compuerta `sdr_gate`: el reloj espera al receptor |
| `atp_sdr/nodo_beacon.py` | nodo `sdr_beacon`: el beacon del autotrack con la potencia medida sobre la amplitud IQ |
| `launch/sdr_loop.launch.py` | lazo ATP (N2/N4/S4) con el receptor SDR |
| `launch/sdr_autotrack.launch.py` | autotrack N5 con `beacon:=sdr` o `beacon:=rf` |
| `test/` | 62 pruebas contra teoría (ver abajo) |
| `../../scripts/informe_sdr.py` | imprime las tablas de cifras-contra-teoría de este README |
| `../../scripts/validar_sdr.sh` | pruebas + pases en el lazo + determinismo |

Paquete nuevo de mensajes: `src/sdr_interfaces` (`SdrStatus`). No se tocó
`atp_interfaces` ni `atp_ros`.


## Modelo

**Banda base.** El oscilador local del receptor está sintonizado a la frecuencia
nominal `f0` del beacon (la de `model.BANDAS`): lo que se ve en banda base es
el Doppler `fd = -vr/c f0` (más el desplazamiento del oscilador, si lo hay).
Dentro de cada bloque de 50 ms el Doppler varía linealmente entre el valor del
paso anterior y el del actual (tasa constante: el error de fase es
≤ jerk·dt³/24, despreciable) y la fase se acumula entre bloques, así que el pase
entero es una sola realización continua.

**Ruido y C/N0.** Piso de ruido FIJO: ruido blanco gaussiano complejo de
varianza 1 por muestra (`N0·fs = 1`, "AGC apagado"). La amplitud sale del C/N0
del presupuesto de enlace (`model.presupuesto_enlace`, **con** la pérdida por
desapuntamiento del paso): `A² = (C/N0)/fs`. Así la amplitud IQ medida es una
medida de la potencia recibida. Para BPSK/QPSK con raíz de coseno alzado
(β = 0.35, 8 símbolos) el filtro se normaliza a `Σh² = sps` y tras el filtro
adaptado `h/sps` el símbolo sale con amplitud A y ruido `σ²/sps`:
`Es/N0 = (C/N0)/Rs`, BER = Q(√(2Eb/N0)) (Proakis & Salehi cap. 4).

**Oscilador.** Desplazamiento constante (`offset_lo_hz`; 1 ppm de un TCXO a
2.2 GHz = 2.2 kHz), deriva lineal y ruido de fase de frecuencia blanca
(caminata aleatoria, `var(Δφ) = 2π·lw·dt`, ancho de línea `lw`; Rubiola 2009).

## Receptor

1. **Adquisición** (`adquirir_fft`): FFT con relleno de ceros ×4 de una ventana
   de `T_acq = min(50 ms, 1/√tasa_max)` (el Doppler no debe moverse más de un
   bin: tasa·T² < 1), búsqueda del pico en la ventana de Doppler esperada
   (`|vr| ≤` velocidad circular de la órbita), interpolación parabólica y umbral
   CFAR por probabilidad de falsa alarma (piso = mediana/ln 2). En BPSK se busca
   el pico de x² (quita la modulación) y se divide entre 2.
2. **Seguimiento** (`SeguidorPortadora`): un NCO continuo mezcla, se integra y
   descarga cada `T_int = 1 ms` (la "época") y un lazo digital de 2.º o 3.er
   orden corrige fase y frecuencia del NCO. Coeficientes de Kaplan & Hegarty /
   Ward: 2.º orden `ωn = B_L/0.53`; 3.er orden `ωn = B_L/0.7845`, `a3 = 1.1`,
   `b3 = 2.4`. Un FLL (discriminador `atan2(cross, dot)/T`) ayuda a la captura
   hasta que el detector declara el enganche. La salida del NCO es la
   **estimación del Doppler**.
3. **Detector de enganche**: `<I²−Q²>/S`, que vale `E[cos 2φ] = exp(−2σ²)` sin
   sesgo por ruido (el ruido se cancela en `I²−Q²`; `S = <|p|²> − N` con el
   piso espectral). Histéresis 0.6 / 0.3 (σ ≈ 29° / 44°). Si se pierde el
   enganche, la cadena vuelve a BÚSQUEDA y readquiere.
4. **C/N0**: M2M4 sobre las muestras post-integración (Pauluzzi & Beaulieu
   2000; `S = √(2M2²−M4)`, `N = M2−S`, `C/N0 = S/(N·T)`), NWPR (Van Dierendonck
   1992) y el estimador **espectral** (potencia de banda ancha menos piso de
   ruido leído del espectro). `cn0_est` usa M2M4 por debajo de 45 dB-Hz y el
   espectral por encima.
5. **BPSK** (`ReceptorBPSK`): etapa 1 = el mismo seguidor en modo FLL de 2.º
   orden con discriminador de la señal elevada al cuadrado (quita el Doppler y su
   rampa, deja la fase libre); filtro adaptado RRC; **Gardner** (interpolación
   lineal, ganancias de Rice con Kd medido = 1.00); **Costas** de 2.º orden a
   la tasa de símbolo (`atan(Q/I)`). La ambigüedad de 180° se resuelve en el
   banco de pruebas correlacionando con los bits transmitidos; un receptor real
   usaría una palabra de sincronía o codificación diferencial.

## Supuestos

- Portadora con Doppler lineal por bloque, ruido blanco gaussiano, un solo
  trayecto (sin multitrayecto, centelleo, Faraday ni interferencia).
- Receptor sin cuantización ni desbalance IQ ni DC (el SDR "ideal"); piso de
  ruido calibrado (σ² = 1) y receptor con el piso desconocido (lo lee del espectro).
- Fase y frecuencia de reloj de muestreo perfectas (la deriva del reloj de
  símbolo la absorbe Gardner).
- El oscilador de la estación se supone sintonizado a `f0` sin conocer el Doppler
  (adquisición ciega en la ventana); con `fd_pred_hz` (beacon por amplitud) se
  pre-corrige con la efeméride, como una estación real.
- Un bloque IQ por paso del lockstep (50 ms): la modulación del conical scan
  (2.5 Hz) se muestrea una vez por bloque, igual que en `rf_beacon`.

## Qué cambia con un SDR real

El receptor (`receptor.py`, `bpsk.py`) solo ve arreglos `complex` de muestras:
sustituir el generador por un SDR es cambiar la fuente de `x`.

- **Fuente de muestras.** USRP / LimeSDR / RTL-SDR con SoapySDR
  (`readStream` en bloques de `fs·50 ms` muestras `complex64`) o un bloque de
  GNU Radio (`sync_block` cuyo `work()` llame a `SeguidorPortadora.procesar`;
  hay hasta un PLL/Costas ya hecho en GNU Radio, aquí está hecho a mano para poder
  validarlo contra la teoría). El nodo `sdr_receiver` sin lockstep (CONTRATO §8.5)
  sellaría cada bloque con el reloj del SDR/GPS-PPS en lugar del reloj simulado.
- **Frontal.** Ganancia y figura de ruido del LNA/mezclador; un ADC de 8 bits
  (RTL-SDR) limita la SQNR a ~50 dB y recorta señales fuertes; el AGC cambia el
  nivel: la amplitud IQ solo es potencia si se calibra la cadena de ganancia
  (aquí el piso es 1 por construcción). En Ka hace falta un convertidor (LNB): su
  oscilador pone ±30 kHz por ppm.
- **Cero-IF.** Pico de DC y desbalance IQ (imagen) justo en banda base; el Doppler
  de un pase LEO **cruza el cero** al pasar por el cenit, justo sobre el pico de
  DC. Se sintoniza con un desplazamiento (low-IF) o se corrige DC/IQ antes del
  lazo (`quitar_dc`, `corregir_iq` del curso SDR).
- **Oscilador y reloj.** Un TCXO de 1 ppm son 2.2 kHz en S: la ventana de
  búsqueda y `fs` deben cubrirlo (el generador ya lo modela: `offset_lo_hz`). Su
  ruido de fase no es de frecuencia blanca (hay flicker y suelo de fase): el
  ancho de lazo debe elegirse con la densidad espectral real. El reloj de
  muestreo tiene su propio ppm (deriva del reloj de símbolo, que Gardner absorbe,
  y escala del Doppler).
- **Doppler predicho.** Una estación real sintoniza el LO con el Doppler de la
  efeméride (TLE) y deja al receptor el residuo; `CadenaRF.paso(fd_pred_hz=...)`
  lo hace y baja `fs` de 2 MS/s a 20 kHz en Ka. Reprogramar el sintonizador por
  USB tarda milisegundos: se rota por software con un NCO.
- **Pérdidas de muestras.** Un overrun USB/Ethernet es un hueco que el lazo ve
  como salto de fase (deslizamiento de ciclo): `CadenaRF` ya cuenta los huecos
  de sello y readquiere.
- **Canal.** Multitrayecto, centelleo ionosférico (afecta a M2M4 y a la
  amplitud del autotrack), polarización/Faraday en UHF/S, interferencia.
- **Tiempo real.** El lazo de portadora a 1 kHz cuesta ~40 µs por época en Python
  (4 % de un núcleo); Gardner + Costas cuestan ~10 µs por símbolo, por lo que
  más de ~50 ksps necesita C++/GNU Radio/numba. La FFT y el mezclado están
  vectorizados en numpy.

## Validación contra teoría

`python3 -m pytest src/atp_sdr/test` (62 pruebas, 1.5-3 min según la carga de
la máquina; cada una fija su semilla y es determinista). Las cifras salen de
`python3 scripts/informe_sdr.py` (imprime cada tabla, medida junto a su teoría);
aquí, medidas junto a su fuente o derivación.

| Qué | Medido | Teoría | Fuente / derivación |
|---|---|---|---|
| B_L del lazo de 2.º / 3.er orden | 0.530 ωn / 0.7838 ωn (integral numérica de \|H\|²) | 0.53 / 0.7845 | Kaplan & Hegarty cap. 5; Ward |
| Varianza de fase del PLL, C/N0 34-40 dB-Hz, 60 s | cociente medido/teoría 1.03 · 1.04 · 1.07 · 1.09 | 1 (σ² = B_L/(C/N0)(1 + 1/(2T C/N0)) con el B_L **efectivo** del lazo digital: +1…+8 % sobre el nominal) | Kaplan & Hegarty cap. 8; B_L efectivo = Σh²/(2T) simulando el lazo lineal |
| Error de arrastre de una rampa de 100 / 500 Hz/s, B_L = 20 Hz | 2.º orden: 0.4416 / 2.2098 rad · 3.er orden: 0.0000 / 0.0008 rad | 2π a/ωn² = 0.4418 / 2.2089 · 0 | Kaplan & Hegarty cap. 5 (lazo tipo 2 vs tipo 3) |
| Error por jerk de 100 / 400 Hz/s² (3.er orden) | 0.0378 / 0.1515 rad | 2π j/ωn³ = 0.0379 / 0.1517 | ídem |
| Ruido de fase del oscilador (lw = 1 y 2 Hz) | 0.0831 / 0.1270 rad² | 2π lw T Σg² = 0.0856 / 0.1260 | Rubiola 2009 cap. 1 + respuesta al escalón de fase del lazo |
| Detector de enganche vs. σ de fase | 0.917 · 0.944 · 0.975 · 0.996 a σ = 14° · 10° · 6.6° · 2.7° | E[cos 2φ] = exp(−2σ²) = 0.888 · 0.939 · 0.974 · 0.996 | derivación (φ gaussiana) |
| Umbral de pérdida de enganche (B_L = 20 Hz, T = 1 ms) | primeros deslizamientos de ciclo con σ teórico ≈ 42° (22 dB-Hz): 3/20; 13/20 con 64° (20 dB-Hz); 0/20 desde 24 dB-Hz | regla 3σ < 45° ⇒ σ < 15° ⇒ C/N0 ≥ 27.4 dB-Hz (conservadora ≈ 5 dB) | Kaplan & Hegarty cap. 8; el detector (ruido compensado) sin falsas caídas desde 28 dB-Hz |
| Adquisición FFT: error de frecuencia, T = 10 ms, 45 dB-Hz | rms 2.30 Hz | cota de Cramér-Rao 2.19 Hz | Rife & Boorstyn 1974 |
| Falsas alarmas de la adquisición (Pfa de diseño 1 %) | 1.07 % (10 000 pruebas) | 1 % | CFAR exponencial; con relleno ×4 hay 2.8× más intentos (medido) |
| C/N0 por M2M4, 2000 muestras, 34-80 dB-Hz | sesgo −0.25 … −0.07 dB, σ 0.14 … 0.10 dB | 0; σ ≈ 4.34√(2/K) dB | Pauluzzi & Beaulieu 2000 |
| C/N0 por NWPR | sesgo −0.35 … −0.16 dB | 0 (sesgo = jitter de fase del lazo) | Van Dierendonck 1992 |
| C/N0 espectral, bloque de 50 ms | sesgo −0.03 dB, σ 0.2 dB (≥ 60 dB-Hz); σ 1.2 dB a 45 | 0 | derivación (potencia total − piso) |
| BER BPSK ideal, Eb/N0 0-8 dB | dentro del IC 99.9 % de Q(√(2Eb/N0)) en los 5 puntos (p. ej. 6 dB: 2.316e-3 contra 2.388e-3; 8 dB: 2.005e-4 contra 1.909e-4) | Q(√(2Eb/N0)) | Proakis & Salehi cap. 4 |
| BER QPSK (Gray) ideal, 0-8 dB | ídem, 5 puntos | misma BER por bit | ídem |
| BER cadena completa (FLL por cuadrado, Gardner, Costas) con Doppler 40 kHz + 400 Hz/s | Eb/N0 efectivo −0.16 / −0.13 / −0.08 / −0.12 dB a 2 / 4 / 6 / 8 dB | pérdida de implementación < 0.3 dB | medido |
| Pendiente del detector de Gardner | 1.003 por símbolo | 1.0 (dimensiona los lazos) | Rice 2009 cap. 8 |
| Costas: varianza de fase a Es/N0 = 10 dB | dentro del 12 % de B_L/(C/N0)(1 + 1/(2Es/N0)) | ídem PLL | Proakis & Salehi cap. 6 |
| Doppler del pase: tasa cenital | 707.3 Hz/s a 2.2 GHz | (v²/h)(R/(R+h)) f0/c = 707.3; `model.tasa_doppler_cenit` (v²/h) da 768, un 8 % más (Tierra plana) | derivación |

**Doppler estimado contra verdadero** (tramo central de un pase de 60° a
550 km, donde la tasa es máxima: 390 Hz/s en S, 5.3 kHz/s en Ka; error del NCO
contra la verdad a mitad de cada época, receptor enganchado, 92 dB-Hz):

| Banda | Modo | B_L | Tramo | Error rms | Sesgo | Máx. | C/N0 medido − presupuesto | Enganchado |
|---|---|---|---|---|---|---|---|---|
| S | portadora, 3.er orden | 30 Hz | 40 s | **0.084 Hz** | 0.000 | 1.91 | −0.002 dB (rms 0.18) | 99.4 % |
| Ka | portadora, 3.er orden | 100 Hz | 15 s | **0.045 Hz** | −0.001 | 0.99 | −0.001 dB (rms 0.04) | 99.0 % |
| Ka | portadora, **2.º orden** | 100 Hz | 6 s | pierde el enganche | | | | 0.0 % |
| S | BPSK 12.5 ksps | | 10 s | 0.234 Hz | 0.012 | 5.80 | +0.005 dB | 99.5 % |
| S | portadora con TCXO de 1 ppm (2.2 kHz) | 30 Hz | 12 s | sesgo +2199.995 Hz = el offset | | | | 97.1 % |
| S | portadora, lw = 0.5 Hz | 30 Hz | 12 s | 2.27 Hz = √(lw B_L/π) = 2.25 | −0.06 | 8.59 | −0.32 dB | 97.9 % |

El 2.º orden de B_L = 100 Hz en Ka deja un error de fase constante de
2π·5300/(100/0.53)² = 0.94 rad (> 45°): el detector lo declara sin enganche; el
3.er orden lo anula. El error "máx." es el de la época en que se declara el
enganche. El ruido de frecuencia del oscilador (blanco, lw/(2π) Hz²/Hz) lo sigue
el NCO y se cuenta como Doppler con varianza lw·B_L/π.

## En el lazo ROS

```bash
./run.sh bash scripts/validar_sdr.sh                 # pruebas + N2 + S4 + determinismo + Ka + autotrack
./run.sh ros2 launch atp_sdr sdr_loop.launch.py el_max:=85 mask:=70 time_scale:=0          # N2, banda S
./run.sh ros2 launch atp_sdr sdr_loop.launch.py profile:=tle sat:=ISS station:=cdmx \
    pass_index:=0 start_utc:=2026-10-02T15:00:00 time_scale:=0 band:=S                        # S4: ISS real
./run.sh ros2 launch atp_sdr sdr_loop.launch.py el_max:=30 mask:=5 time_scale:=0 band:=Ka    # N4 en Ka (2 MS/s)
./run.sh ros2 launch atp_sdr sdr_loop.launch.py mode:=bpsk offset_lo_hz:=2200 lw_lo_hz:=0.5 delta_cn0_db:=-30
```

`sdr_receiver` (CONTRATO 8): por cada paso de 50 ms empareja por sello
`/atp/sat_state` y `/atp/pointing_error`, genera el bloque IQ de ese instante
(Doppler del estado del satélite, C/N0 del presupuesto con el desapuntamiento
real), lo procesa y publica `/atp/sdr` (`sdr_interfaces/SdrStatus`: Doppler
verdadero y estimado, C/N0 del presupuesto y medido por M2M4 y espectral,
amplitud, métrica y estado de enganche BÚSQUEDA/SEGUIMIENTO/ENGANCHADO, BER).
Usa `atp_ros.nodo.correr` (saludo de arranque) y figura en `wait_nodes` del
generador. Al final del pase imprime el `RESUMEN SDR` (solo cifras
deterministas) y el `RENDIMIENTO SDR` (CPU, no determinista) y cierra el
lanzamiento.

**Compuerta `sdr_gate` (desviación consciente del CONTRATO 8.3).** Un bloque IQ
cuesta más CPU que un paso de la montura (con el reloj a máxima velocidad, 3-18 ms
contra ~2 ms). Como nodo pasivo, la cola crece y, con la máquina cargada, DDS
perdió pasos (en una corrida BPSK cargada: la mitad), y el pase terminaba antes
de que el receptor acabara. Por eso, por defecto (`gate:=true`) el launch
redirige la confirmación de la montura a `/atp/step_ack_raw` y `sdr_gate` la
reemite como `/atp/step_ack` cuando `sdr_receiver` confirmó el mismo sello en
`/atp/sdr_ack`: el reloj espera al receptor. No hay ciclos (el bloque de t_k
solo necesita `sat_state` y `pointing_error` de t_k) y el apuntamiento no cambia
(N2: 0.0167° rms, idéntico a `atp_loop`). `gate:=false` lo deja pasivo.

Resultados (`time_scale:=0`, receptor de 3.er orden, ver `runs/` al ejecutar):

| Pase | Bloques | Doppler rms | Sesgo | C/N0 medido − presupuesto | Enganchado |
|---|---|---|---|---|---|
| N2 cenital 85°, banda S | 1018 | **0.049 Hz** | −0.003 Hz | +0.005 dB (rms 0.17) | 99.3 % |
| S4 ISS real sobre CDMX (SGP4), S | 9789 | **0.022 Hz** | −0.001 Hz | −0.000 dB (rms 0.17) | 100 % |
| N4 30° banda S, BPSK 12.5 ksps | 10 898 | **0.012 Hz** | −0.000 Hz | +0.001 dB (rms 0.12) | 100 %, BER 0 en 6.8 millones de bits |
| N4 30° **Ka**, 2.05 MS/s | 10 898 | **0.074 Hz** | −0.001 Hz | −0.000 dB (rms 0.04) | 100 % |
| N4 30° S, enlace degradado −50 dB (C/N0 ≈ 27-37 dB-Hz) | 10 898 | 7.46 Hz | −0.001 Hz | +0.11 dB (rms 1.18) | 100 % |
| N4 30° S, TCXO de 1 ppm + lw 0.5 Hz | 10 898 | 2200.011 Hz (2.1 Hz alrededor del offset = √(lw B_L/π)) | +2200.010 Hz = el offset | −0.037 dB (rms 0.13) | 100 % |

Todas con `huecos=0` y 1 adquisición, 0 pérdidas. El Doppler rms es por época
de 1 ms (la frecuencia del NCO, que incluye su ruido de mando: con el enlace
degradado, c₂σ_θ/2π = 7.4 Hz, que es lo que sale); promediada por bloque de
50 ms es del orden de 7 veces menor (√50, si el ruido fuera blanco). En el pase N2 de la tabla (nominal) el Doppler tiene
tasa máxima de 705 Hz/s. **Determinismo**: la misma corrida N2 dos veces da
la línea `RESUMEN SDR` idéntica carácter por carácter (comprobado dos veces,
con la compuerta y sin ella).

### Rendimiento (CPU del hilo, un solo núcleo; máquina de medida con carga ≈ 50 sobre 8 núcleos, así que son cifras pesimistas)

Por bloque de 50 ms (CPU): generador (simulación, no cuenta contra un SDR
real) + receptor. Margen = muestras por segundo de CPU de la cadena entre las
que exige el tiempo real (fs):

| Banda | Modo | fs | Generador | Receptor | MS/s del receptor | Margen sobre tiempo real (cadena / solo receptor) |
|---|---|---|---|---|---|---|
| UHF | portadora | 50 kHz | 0.36 ms | 2.49 ms | 1.00 | ×17.6 / ×20 |
| S | portadora | 150 kHz | 0.66 ms | 2.91 ms | 2.58 | ×14.0 / ×17 |
| X | portadora | 600 kHz | 2.0 ms | 4.6 ms | 6.6 | ×7.6 / ×11 |
| Ka | portadora | 2.05 MS/s | 6.3 ms | 9.1 ms | 11.2 | ×3.2 / ×5.5 |
| S | BPSK 12.5 ksps | 300 kHz | 4.3 ms | 17.3 ms | 0.87 | ×2.3 / ×2.9 |

En el lazo (Ka, N4): 10.5 ms de receptor + 10.8 ms de generador por bloque =
×2.3 sobre el tiempo real; en S carrera completa 3.0 + 0.7 ms (×13.5). El cuello
del BPSK es Python escalar por símbolo (Gardner + Costas ≈ 10 µs por símbolo,
pase de 625 símbolos por bloque): por encima de ~50 ksps hace falta C++ o
GNU Radio.

### Autotrack con la amplitud medida sobre la señal IQ (bonus)

`rf_beacon` genera P_dB como la potencia ideal del lóbulo gaussiano más un ruido
gaussiano **inventado** de 0.2 dB. `sdr_beacon` (launch `sdr_autotrack.launch.py
beacon:=sdr`) lo sustituye: suma la potencia del conical scan al C/N0 del
presupuesto del rango de ese instante, genera el bloque IQ (portadora con el
Doppler residuo respecto de la efeméride; fs de 20 kHz), mide la amplitud
`A² = <|x|²> − σ²` (piso leído del espectro) y publica `P_dB = 10 log10(A² fs/C/N0_pico)`
(solo con la portadora enganchada). El `autotrack` de `atp_ros` NO cambia: es el
mismo escenario N5 (Ka, haz de 0.233°, efeméride 0.1 s atrasada, sesgo mecánico),
solo cambia de dónde sale la potencia.

| Beacon | Error de la potencia medida (rms) | Apuntamiento (error real contra la verdad) |
|---|---|---|
| sin autotrack (program track) | — | 0.1572° rms, 0 % dentro de 0.1° |
| `rf_beacon`, ruido gaussiano 0.2 dB | 0.2 dB (por construcción) | 0.0041° rms |
| IQ, enlace nominal (C/N0 del pase ≈ 80-90 dB-Hz) | **0.0010 dB** | **0.0032°** rms |
| IQ, enlace −35 dB | 0.0758 dB | 0.0033° |
| IQ, enlace −45 dB | 0.5639 dB | 0.0083° (el ruido de amplitud ya cuenta) |
| IQ, enlace −55 dB (bajo el umbral de adquisición) | receptor enganchado 15 % | **0.2296°: peor que el program track** |

Lectura: el 0.2 dB de `rf_beacon` equivale a un enlace ~40 dB más débil que el
de diseño (entre las filas de −35 dB y −45 dB); con el enlace de diseño la
medida real es 200 veces mejor y el autotrack llega a
0.0032°. Pero el autotrack hereda las limitaciones del receptor: por debajo del
umbral de adquisición (~40 dB-Hz con T_acq = 9 ms y Pfa = 10⁻³) no hay amplitud y el lazo
oscilante de enganche/pérdida lo empeora respecto del program track; una
estación real tendría que volver al program track cuando el receptor pierde la
portadora (el `autotrack` de `atp_ros` solo congela la corrección cuando la
potencia media cae bajo `lock_db`, y aquí el receptor intermitente la
vuelve a subir por momentos). Limitación del modelo: la amplitud del bloque de
50 ms es la del instante del sello (la modulación del scan se muestrea una vez
por bloque, como en `rf_beacon`).

## Limitaciones y lo que NO está verificado

- Todo es simulación: **ninguna cifra se midió con un SDR real**. Las cifras
  contra teoría validan el receptor y el generador entre sí y contra fórmulas;
  no el canal ni el hardware (ver arriba).
- El tramo de Doppler de las pruebas sobre pases es de 6-40 s alrededor del
  cenit (donde la tasa es máxima) para que la suite quepa en minutos; el pase
  completo se valida en el lazo ROS (N2, N4, S4: 1 000-11 000 bloques).
- QPSK solo se valida con receptor ideal (BER); no hay cadena de recepción
  QPSK (Costas de 4 fases y 4.ª potencia en la adquisición).
- El BPSK resuelve la ambigüedad de 180° correlacionando con los bits que el
  propio generador transmitió (banco de pruebas). Sin codificación, tramas ni FEC.
- Adquisición ciega a un solo pico y sin desfase de código; en BPSK hace falta
  fs ≥ 4.2 veces el semiancho de la ventana (el cuadrado duplica la frecuencia),
  y Gardner usa interpolación lineal (sps ≥ 8).
- El ruido de fase del oscilador es de frecuencia blanca (lorentziano); no hay
  flicker ni suelo de fase. Con lw = 0.5 Hz el C/N0 medido sale ~0.3 dB bajo
  (las faldas del ruido de fase ensanchan la línea y suben el piso espectral).
- El detector de enganche con T = 1 ms y K = 100 épocas es fiable desde ~28
  dB-Hz; por debajo, falsas caídas (a 24 dB-Hz: 7/20 corridas sin enganche al
  final con σ teórico 29°). El C/N0 por M2M4 con 100 muestras tiene 0.65 dB de
  dispersión por bloque.
- El autotrack por amplitud falla bajo el umbral de adquisición (ver arriba);
  no se probó con el scan muestreado dentro del bloque.
- Rendimiento medido con la máquina cargada (carga ≈ 50 en 8 núcleos) y CPU del
  hilo; no hay una medida en máquina en reposo. Los nodos no se probaron a
  `time_scale` distinto de 0 (lockstep: da lo mismo por construcción, pero no se
  midió) ni con el grabador de la Estación.
- `model.tasa_doppler_cenit` (atp_ros, no tocado) usa v²/h y sobrestima la
  tasa Doppler cenital un 8 % a 550 km (falta el factor R/(R+h)); se documenta en
  `test_senal.py`.
