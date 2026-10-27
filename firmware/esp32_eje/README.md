# Firmware del eje — ESP32 + micro-ROS (equipo J)

Primer firmware real de la Estación ATP: **un ESP32 gobierna un eje** de la
montura de la antena (acimut o elevación) o, con la opción `ATP_EJES=2` (banco de escritorio,
[HARDWARE](../../docs/HARDWARE.md) §3), **los dos**. Cierra el lazo de posición a **1 kHz**
con un encoder y un puente H propios y habla ROS 2 Jazzy por **micro-ROS**
(serie o UDP/Wi-Fi), con la interfaz que ya usa el resto de la Estación
([CONTRATO](../../docs/CONTRATO.md) §7). Cumple el plan de
[`src/atp_hw`](../../src/atp_hw/README.md) («micro-ROS en un ESP32»).

> **Estado, con honestidad.** Compila en Docker, la lógica del lazo está probada
> en el host contra la planta de `atp_ros`, y el mismo nodo micro-ROS corre
> sobre Linux hablando con un agente y con nodos ROS 2 normales. **Nunca se ha
> cargado en una placa**: no había hardware. Todo lo que dependa de la placa, de
> los periféricos o de la mecánica está marcado *SIN PROBAR* o *SUPUESTO*; la
> lista de lo que falta para el primer encendido está al final.

## 1. Arquitectura

```
 PC (Estación, ROS 2 Jazzy, dominio N)                       ESP32 (un eje)
┌─────────────────────────────────────┐            ┌────────────────────────────────────────┐
│ reference_generator ─▶ /atp/reference│  XRCE-DDS  │ tarea micro-ROS (núcleo 0, prio 5)     │
│ error_monitor ◀────── /joint_states  │◀──────────▶│   rclc: 3 subs + 2 publicadores        │
│ atp_mcu/mcu_merge: /atp/mcu/<eje>/…  │  agente    │   reconexión automática                │
└──────────────▲──────────────────────┘ micro-ROS  │        │ buzones (sección crítica ~1 µs) │
               │  UART 921600 8N1  o  UDP 8888      │ tarea del lazo (núcleo 1, prio 23, 1 kHz)
   microros/micro-ros-agent:jazzy (Docker)          │   encoder ─▶ eje_paso() ─▶ PWM/EN       │
                                                    │ ISR del paro de emergencia ─▶ EN = 0    │
                                                    └──────┬──────────────┬──────────────┬────┘
                                                       encoder        BTS7960/driver   paro (NC)
```

* **`core/`** — la lógica del lazo en **C puro** (sin ESP-IDF ni ROS): PID discreto,
  anti-windup, referencia con tope de velocidad, límites, watchdog, máquina de
  estados, fallas, conversión cuentas↔rad, encoder absoluto (paridad y desenrollado)
  y la planta J/b en C. La misma fuente corre en el ESP32, en las pruebas del host y
  en el nodo de Linux.
* **`uros/`** — el nodo `rclc` (entidades, reconexión con el agente, callbacks). Igual en
  el ESP32 y en Linux.
* **`main/`** — solo ESP32: periféricos (PCNT, SPI, LEDC, GPIO, ADC), transporte, la tarea del lazo.
* **`host/`** — nodo micro-ROS sobre Linux con la planta simulada (el «hardware en el lazo» sin hardware).
* **`test/`** — `ctest` en el host y los vectores de `atp_ros`.
* **`../../src/atp_mcu`** — paquete ROS 2: `mcu_merge` une los ejes en `/joint_states`.

### El lazo (igual al de `atp_ros`)

`pid_controller.py` línea a línea: `d_filt += α(Δe/dt − d_filt)`, `α = dt/(τ+dt)`,
`u = clip(kp·e + ki·∫e + kd·d_filt)`, anti-windup por *clamping* (no integra si satura y el
error empuja más). Ganancias por defecto de `atp_ros`: kp 25, ki 2, kd 8, τ 80 ms, u_max 8 N·m.
**Esas ganancias son de la planta didáctica (J = 2 kg·m²): con la montura real hay que
identificar y re-sintonizar** ([CONTROL §8](../../docs/CONTROL.md)). El firmware solo porta el
PID; el LQR+Kalman del equipo H no está portado.

Entre la referencia y el PID hay una etapa que el PID de `atp_ros` no necesita (allí hay lockstep):
la referencia llega a 20 Hz con saltos; el lazo la **extrapola** con su velocidad (la del mensaje si
`velocity` viene, o la estimada de dos muestras con sus sellos), la **limita** a los límites de software
y la pasa por un **tope de velocidad** (`vel_max`), y frena con perfil `v = √(2·a·d)` al acercarse
a un límite. Al armar, la referencia comandada **parte de la posición actual** (sin salto).

## 2. Interfaz ROS 2

Espacio de nombres `/atp/mcu/<eje>`, `<eje>` = `az` | `el` (un ESP32 por eje; se elige en `menuconfig`).
Con `ATP_EJES=2` el mismo ESP32 publica y escucha **los dos** espacios (`/atp/mcu/az/…` y `/atp/mcu/el/…`) desde un solo nodo
(`esp32_banco`): la interfaz de abajo es idéntica por eje. Opcional (`ATP_ECO_LATENCIA`, apagado por defecto): `/atp/mcu/eco`
(`std_msgs/Int64`) devuelve el sello, en ns, de cada `/atp/reference` recibida: sirve para medir la latencia de ida y vuelta (§7c).

| Tópico | Tipo | Sentido | Qué |
|---|---|---|---|
| `/atp/reference` | `sensor_msgs/JointState` | ESP32 **suscribe** | **El del contrato.** `position[i]` en rad; `i` = lugar de `<eje>` en `name` (o el índice fijo si `name` viene vacío). `velocity[i]` opcional |
| `~/joint_states` | `sensor_msgs/JointState` | ESP32 publica (reliable, 50 Hz) | `name=[<eje>]`, `position`/`velocity` del encoder (rad, rad/s), `effort` = mando en N·m (como `/joint_states` del contrato). Sello = hora del agente (PC) tras `rmw_uros_sync_session` |
| `~/diag` | `std_msgs/Float32MultiArray` | publica (best effort, 2 Hz) | estado, fallas, tiempos del lazo: ver `DIAG_*` en [`uros/eje_uros.h`](uros/eje_uros.h) |
| `~/cmd/enable` | `std_msgs/Bool` | suscribe | `true` = armar (y **rearmar** tras una falla); `false` = desarmar |
| `~/cmd/origen` | `std_msgs/Float64` | suscribe | «el eje está AHORA en este ángulo (rad)»: referenciado manual; solo con el driver apagado |

**Por qué un espacio de nombres por eje y no `/joint_states` directo:** el contrato pide un único
`/joint_states` con `name=[az, el]`; dos ESP32 no pueden compartirlo. `atp_mcu/mcu_merge` los une:

```bash
ros2 run atp_mcu mcu_merge                              # dos ESP32
ros2 launch atp_mcu mcu_merge.launch.py simulated:=el   # un solo ESP32 (az); el = su referencia
```

Un eje real sin dato fresco (> 0.5 s) **no se rellena**: `mcu_merge` deja de publicar y avisa. El eje
`simulated` toma la referencia (error 0 por construcción: sirve para probar un eje, no para medir apuntamiento).

**Dominio ROS:** micro-ROS manda al agente el `ROS_DOMAIN_ID` (`CONFIG_ATP_ROS_DOMAIN_ID`, defecto **42**, el de
`docker-compose.yml`; las corridas de la Estación usan 100–199). Debe coincidir en los tres lados:
firmware, agente (`ROS_DOMAIN_ID` de `agente.sh`) y la Estación. **Tiempo:** sin lockstep ([CONTRATO §8.5](../../docs/CONTRATO.md)),
`time_scale = 1`; el sello de `~/joint_states` es hora de pared del PC (vía el agente), el de `/atp/reference` el del generador:
solo se usan sus *diferencias*.

### Estados y fallas

| Estado | Driver | Significado |
|---|---|---|
| `DESARMADO` (0) | apagado | espera `cmd/enable=true` (por defecto **no** arma solo: `ATP_AUTOARMAR=n`) |
| `ESPERA` (1) | apagado | armado, sin referencia fresca |
| `ACTIVO` (2) | PID | sigue la referencia |
| `RETENCION` (3) | PID | perdió la referencia: **mantiene la posición** `t_retencion` (2 s) |
| `FALLA` (4) | apagado, **enclavada** | no se quita sola: `cmd/enable=true` con la causa ya resuelta |

Fallas (máscara en `diag[1]`): `ESTOP` 1, `LIMITE` 2, `VEL` 4, `ATASCO` 8, `ENCODER` 16, `CORR` 32, `CFG` 64.
LED (GPIO2): 1 Hz desarmado · destello cada 1 s en espera · fijo activo · 4 Hz retención · 10 Hz falla.

## 3. Pines (ESP32-DevKit, WROOM-32; todo configurable en `menuconfig`)

| Función | GPIO | Notas |
|---|---|---|
| micro-ROS serie | UART0: TX 1, RX 3 | es el puente USB de la placa (`/dev/ttyUSB0`) |
| Consola (logs) | UART1: TX **17**, RX 16 | con serie, UART0 es del agente; los logs salen por 17 (adaptador USB-serie aparte, opcional). En la variante UDP, UART0 vuelve a ser consola y `idf.py monitor` sirve |
| Encoder A / B (PCNT) | **32 / 33** | cuadratura x4, filtro de glitch 1 µs. **3.3 V**: ver cableado |
| Encoder absoluto SPI (opción) | SCLK 14, MOSI 13, MISO 26, CS 23 | AS5048A, modo 1, 1 MHz. *SIN PROBAR* |
| PWM RPWM (o PWM) | **18** | LEDC, 20 kHz, 10 bits |
| PWM LPWM (o DIR) | **19** | |
| EN (R_EN y L_EN juntos) | **21** | **pull-down externo de 10 kΩ** |
| Paro de emergencia | **27** | contacto **NC** a masa + pull-up; abierto = paro |
| Corriente R_IS / L_IS (opción) | 34 / 35 | ADC1 (nunca ADC2: choca con Wi-Fi). Entradas solo-lectura |
| LED de estado | 2 | |

Evitados a propósito: GPIO 0, 2 (solo LED), 12, 15 (*strapping*), 6–11 (flash), 1/3 (UART0).

**Con `ATP_EJES=2`** (acimut = eje 0 con los pines de arriba; elevación = eje 1; un paro común): encoder del segundo eje A/B = **25 / 26**, PWM RPWM/LPWM = **22 / 23**,
EN = **4** (pull-down externo de 10 kΩ en cada EN), corriente R_IS/L_IS = **36 / 39** (ADC1, solo entrada). El paro (GPIO27) baja los dos EN en la misma ISR. El encoder SPI no se admite con 2 ejes
(comparte pines con el eje 1). Tabla completa y razones: [HARDWARE](../../docs/HARDWARE.md) §3.1.

### Cableado y protección (**leer antes de conectar potencia**)

1. **Masa común** entre ESP32, driver y encoder, en estrella (un punto), con el retorno de potencia del
   motor por cable aparte y grueso. Sin masa común las señales no tienen referencia.
2. **Alimentación separada**: el motor de su fuente (con **fusible** y, si el motor regenera, un **TVS/resistencia de frenado**);
   el ESP32 por USB o de un regulador, **no** de la fuente del motor sin regulador. Condensador de bulto cerca del puente.
3. **Niveles lógicos 3.3 V** (hoja de datos de Espressif: VDD 3.3 V; los GPIO **no** son tolerantes a 5 V). Un encoder de
   salida a 5 V o *line driver* (RS-422) necesita divisor/adaptador de nivel o un receptor diferencial. Que las
   entradas del módulo BTS7960 acepten 3.3 V hay que **comprobarlo** en el módulo concreto (*no verificado*).
4. **EN con pull-down externo**: durante el arranque y tras un reinicio los GPIO flotan; el pull-down mantiene el
   puente apagado. El firmware pone EN a 0 *antes* que nada, antes de inicializar el PWM.
5. **Paro de emergencia = contacto normalmente cerrado.** Un cable cortado abre el circuito y para, igual que
   pulsar. 10 nF–100 nF a masa contra rebotes. **El paro de software NO sustituye al de potencia**: el pulsador debe
   además **cortar la fuente del motor con un contactor/relé** que no pase por el ESP32. El firmware protege contra
   fallos de *software*; no está certificado como función de seguridad.
6. Cables de encoder trenzados/apantallados lejos del cable del motor (el PCNT cuenta cualquier flanco; el filtro
   de 1 µs solo descarta glitches muy cortos). Para una montura de 3 m, **fines de carrera cableados** a la potencia, no
   solo los límites de software.

## 4. Tiempos: presupuesto de la tarea de 1 kHz

Tick de FreeRTOS 1 kHz (`CONFIG_FREERTOS_HZ=1000`), `vTaskDelayUntil(1)`, núcleo 1, prioridad fija 23
(`configMAX_PRIORITIES − 2`), reloj de CPU 240 MHz (hoja de datos de Espressif): **1 ms = 240 000 ciclos**.
Wi-Fi y lwIP viven en el núcleo 0; la tarea micro-ROS (prioridad 5, núcleo 0) nunca puede retrasar al lazo.

| Paso del tick | Costo estimado | Base de la estimación |
|---|---|---|
| leer encoder | PCNT: un registro, < 2 µs · SPI: ~16 µs de reloj a 1 MHz + overhead | *no medido* |
| `eje_paso` (estados, PID, límites, conversiones) | unos pocos µs | 1215 B de código sin bucles (~400 instrucciones) + ~7 divisiones/1 `sqrtf` por software (la FPU del ESP32 no divide ni saca raíces): **≲ 3 000 ciclos ≈ 12 µs** como cota gruesa |
| `motor_aplicar` (LEDC, 2–3 llamadas de driver) | ~5–10 µs | *no medido* |
| telemetría (sección crítica, copia de ~60 B) | ~1–2 µs | *no medido* |
| ADC de corriente (solo cada 10 ticks) | ~50 µs cada 10 ms | *no medido* |

Cota de la suma: **≲ 40 µs de 1000 µs (≈ 4 %)**: holgura de sobra, **pero es una estimación por tamaño de código, no
una medición**. El firmware mide cada tick con `esp_timer` y publica `t_lazo_us`, `t_lazo_max_us`, `jitter_max_us` y
`ticks_perdidos` en `~/diag` (y `mcu_merge` los registra): **eso es lo que hay que leer en el primer encendido**. Un tick «perdido» es
un periodo real > 1.5 ms. Si el lazo se cuelga > 100 ms, el *watchdog de tareas* reinicia el chip (y el puente queda apagado
por el pull-down).

**Con dos ejes** el tick hace el trabajo dos veces (cota ≲ 80 µs de 1 000 µs, ≈ 8 %, *estimada* por el tamaño del código compilado; `eje_paso` ocupa 1 215 B igual que con un eje) y el
ADC del segundo eje se muestrea 5 ticks después del primero para no sumar dos lecturas en el mismo tick. En el host el tick de dos ejes cuesta ×1.58 el de uno (94 → 148 ns, `test_dos_ejes`; cota superior en x86, **no** es el tiempo del ESP32).
Desglose en [HARDWARE](../../docs/HARDWARE.md) §3.2.

Latencias de la cadena (estimadas): referencia → buzón → aplicada en el siguiente tick (≤ 1 ms) + enlace (serie 1–3 ms;
Wi-Fi: jitter de decenas de ms, vigilar: [atp_hw README](../../src/atp_hw/README.md), punto 6). El margen de retardo del lazo LQR es
93 ms ([CONTROL §8](../../docs/CONTROL.md)): el enlace cabe, el Wi-Fi hay que medirlo. Una telemetría de ~100 B (estimado) a 50 Hz ocupa ~6 % de
un enlace de 921 600 baudios y ~43 % de uno de 115 200: por eso el valor por defecto es 921 600.

## 5. Seguridad

Todo es del **firmware**, no de ROS ([atp_hw README](../../src/atp_hw/README.md), punto 4), y cada regla tiene prueba en `test/test_lazo.c`.

| Protección | Cómo | Prueba |
|---|---|---|
| **Paro de emergencia** | ISR en cualquier flanco: baja EN en µs; el lazo lo lee cada tick y enclava `FALLA`; no arranca solo al cerrarse; rearme explícito y solo con el contacto cerrado | `test_estop` |
| **Watchdog de referencia** | 500 ms sin referencia → `RETENCION` (mantiene posición 2 s) → driver apagado; vuelve solo si llega referencia fresca | `test_watchdog_*` |
| **Límites de software** | la referencia se recorta a `[lim_min, lim_max]`, frena con perfil √ al acercarse, y no se empuja hacia fuera del límite; fuera del límite + margen: falla | `test_limite_*`, `test_falla_fuera_de_limites` |
| **Velocidad** | tope de la referencia (12°/s); velocidad medida > 30°/s: falla | `test_falla_sobrevelocidad`, `test_arranque_sin_salto` |
| **Encoder** | salto imposible entre dos ticks: falla (cable suelto, ruido); el AS5048A comprueba paridad y bandera de error | `test_falla_salto_encoder`, `test_abs` |
| **Atasco** | par > 90 % de u_max con velocidad ~0 durante 1 s: falla | `test_falla_atasco` |
| **Sobrecorriente** | con sensor: > `i_max` durante 50 ms: falla | `test_falla_sobrecorriente` |
| **Arranque seguro** | EN a 0 antes que nada; `ATP_AUTOARMAR=n`; configuración inválida → no arma | `test_cfg_invalida_no_arma` |
| **Reinicio del chip** | handler de apagado baja EN; el *watchdog de tareas* reinicia si el lazo se cuelga | *SIN PROBAR en placa* |

## 6. Compilar, flashear y correr

Todo se compila **en Docker**; no hace falta instalar ESP-IDF.

```bash
cd firmware/esp32_eje
./build.sh            # variante SERIE  -> build/atp_esp32_eje.bin
./build.sh udp        # variante UDP/Wi-Fi -> build_udp/atp_esp32_eje.bin
./build.sh dos        # variante SERIE de dos ejes (banco: az y el) -> build_2ejes/atp_esp32_eje.bin
./build.sh shell      # shell en el contenedor (idf.py menuconfig, etc.)
```

* Imagen: **`espressif/idf:v5.2.8`** (fijada también por digest en el `Dockerfile`). **Descarga: 4.7 GB comprimida
  (11.9 GB en disco).** `microros/micro-ros-agent:jazzy`: 160 MB (747 MB en disco).
* Componente: `micro_ros_espidf_component`, rama `jazzy`, commit `5cd71f5` (fijado). Su `libmicroros.mk` clona ~25
  repos de micro-ROS/ROS 2 **por rama** al compilar la librería (la primera vez, 10–20 min y red); los commits que
  resultaron quedan en `build/microros-commits.txt` (copia de la compilación de referencia: [`microros-commits.txt`](microros-commits.txt)). La cache vive en `.cache/` (no se versiona).
* Configurar (pines, ganancias, límites, transporte, Wi-Fi): `./build.sh shell` → `idf.py -B build -DSDKCONFIG=build/sdkconfig menuconfig`
  → menú **Estacion ATP - eje de la montura** y **micro-ROS Settings**. Para UDP: SSID/clave y la IP del agente (`CONFIG_MICRO_ROS_AGENT_IP`).
  **No subir claves de Wi-Fi al repo.**
* Elegir el eje: `ATP_EJE_AZ` / `ATP_EJE_EL`. Un firmware por eje.

### Flashear (el contenedor no flashea: cuando haya placa, desde el PC con ESP-IDF o `esptool`)

```bash
# con ESP-IDF instalado en el PC
idf.py -B build -p /dev/ttyUSB0 flash monitor
# sin ESP-IDF: esptool (pip install esptool), usando lo que dejó la compilación
cd firmware/esp32_eje/build && python -m esptool --chip esp32 -b 460800 -p /dev/ttyUSB0 write_flash "@flash_args"
# o desde Docker, pasando el dispositivo
docker run --rm --device /dev/ttyUSB0 -v "$PWD":/project -w /project atp-esp32-idf:5.2.8 \
    idf.py -B build -p /dev/ttyUSB0 flash
```

Permisos del puerto: grupo `dialout`. **Con la variante serie, `monitor` en `/dev/ttyUSB0` muestra tramas binarias de
micro-ROS, no texto**, y además compite con el agente por el puerto: para ver logs, usar la variante UDP o un
adaptador en el GPIO17.

### Agente micro-ROS

```bash
./agente.sh serie /dev/ttyUSB0 921600     # imagen oficial microros/micro-ros-agent:jazzy
./agente.sh udp 8888
./agente.sh ambos                         # los dos en segundo plano; ./agente.sh parar
ROS_DOMAIN_ID=42 ./agente.sh udp          # el dominio por variable (defecto 42)
```

Red del host (`--net=host`) para que su DDS vea a los nodos del PC. Con Wi-Fi: abrir UDP/8888 en el cortafuegos y que
`CONFIG_MICRO_ROS_AGENT_IP` sea la IP del PC.

### Primera conexión (con placa)

```bash
./agente.sh serie
docker compose run --rm ros bash -c 'ros2 topic echo /atp/mcu/az/diag'          # estado DESARMADO, t_lazo_us...
ros2 topic pub --once /atp/mcu/az/cmd/enable std_msgs/msg/Bool "{data: true}"   # armar
```

## 7. Pruebas sin hardware

### a) Lógica del lazo en C puro, contra la planta de `atp_ros` (rápido, ~5 s, está en `scripts/validar_todo.sh`)

```bash
./run.sh bash firmware/esp32_eje/scripts/pruebas_host.sh
```

`gen_vectores.py` usa **`atp_ros.planta.PlantaMontura`** (J = 2, b = 0.5 de `atp_ros.model`) con el PID de `pid_controller.py`
reimplementado en numpy (float64) a 1 kHz, y escribe CSV en `test/vectores/`; el C (PID en **float**, planta en double)
corre los mismos escenarios y se compara fila a fila. El script también comprueba que los CSV del repo siguen siendo los
que genera `atp_ros` hoy. Resultado de la última corrida (`ctest`, 3 suites, 100 % pasadas):

| Escenario (1 kHz) | máx \|Δθ\| C vs `atp_ros` | máx \|Δω\| | máx \|Δu\| |
|---|---|---|---|
| planta en lazo abierto (u = 1 N·m, 5 s) | 5.0e-13 rad | 4.9e-13 rad/s | 0 |
| escalón 5° | 2.0e-9 rad | 1.3e-8 | 3.4e-7 N·m |
| rampa 9°/s (20 s) | 1.7e-9 rad | 9.1e-9 | 1.8e-7 |
| escalón 90° saturado (anti-windup) | 1.3e-7 rad | 3.4e-7 | 6.9e-6 |
| ráfaga de 1 N·m | 1.1e-8 rad | 5.6e-8 | 1.1e-6 |
| rampa con encoder de 16 bits | 7.5e-7 rad | 5.4e-6 | 1.2e-2 (un escalón del cuantizador) |

Es decir: la trayectoria del C coincide con la de `atp_ros` hasta el límite de la aritmética de 32 bits, y la planta
es idéntica. Además, `test_lazo.c` (6 549 comprobaciones) cubre conversión cuentas↔rad, anti-windup (la integral
llega a 0.20 rad·s en vez de 1.02 sin *clamping*), seguimiento de rampa (régimen 0.093° máx < 0.1° de objetivo, transitorio de enganche
1.7°), watchdog (RETENCION a 452 ms, driver apagado a 2.45 s), límites, paro, todas las fallas, mapeo a PWM, signos y referenciado;
`test_abs.c` (478), el encoder absoluto. Umbrales y su fuente están en cada prueba.

`atp_mcu` (la fusión y la decodificación del diagnóstico, que lee los índices del propio `.h` del firmware para que no se
desincronicen): `python3 -m pytest -q src/atp_mcu/test`.

### b) micro-ROS **sobre Linux**: el mismo nodo rclc, contra el agente y ROS 2 de verdad

`host/` compila el lazo y `uros/eje_uros.c` (los mismos que el firmware) contra una `libmicroros` de x86_64, con la planta J/b
en vez del encoder y el motor. Habla UDP con el agente oficial.

```bash
docker compose run --rm -T ros bash firmware/esp32_eje/host/build_libmicroros.sh   # una vez (~20-30 min, red)
bash firmware/esp32_eje/scripts/prueba_micro_ros_host.sh                            # agente + nodo + verificación ROS 2
```

Resultado real de `prueba_micro_ros_host.sh` (agente `microros/micro-ros-agent:jazzy` + `eje_host` + `mcu_merge` + `verificar_host.py`, que hace de Estación; dominio 81):

```
OK    el micro-ROS publica ~/diag (conexion con el agente)  [tras 3.1 s]
OK    nace DESARMADO
OK    desarmado no se mueve aunque llegue referencia  [pos 0.00000 -> 0.00000 rad, ref 0.200 rad]
OK    joint_states trae name=[<eje>], position/velocity/effort  [name=['az']]
OK    cmd/enable=true -> ACTIVO  [en 0.46 s]
OK    llega a la referencia fija de 0.2 rad  [pos=0.2001 rad]
OK    sigue la rampa (regimen): error max < 0.5 deg  [n=398 rms=0.0871 deg max=0.2887 deg]
OK    joint_states ~50 Hz  [49.8 Hz]
OK    /joint_states fusionado: name=[az, el]
OK    referencia cortada -> RETENCION (~0.5 s)  [0.97 s]
OK    sigue sin referencia -> ESPERA, driver apagado (~2.5 s)  [2.96 s desde el corte]
OK    vuelve la referencia -> ACTIVO  [en 0.50 s]
OK    cmd/enable=false -> DESARMADO
TODO OK
```

Es decir: el nodo `rclc` aparece como un nodo ROS 2 normal (`ros2 topic list` ve `/atp/mcu/az/{joint_states,diag,cmd/enable,cmd/origen}`),
obedece al agente, arma, sigue la rampa y aplica el watchdog; `mcu_merge` lo une en `/joint_states` con `name=[az, el]`.
**Cautela con los tiempos:** el host no es de tiempo real y estaba muy cargado (carga ~50 en 8 núcleos por otros equipos): el lazo de Linux
llegó a 48 ms de *jitter* y los umbrales de la prueba son anchos (RETENCION a 0.97 s en vez de 0.5 s). Los tiempos del lazo de 1 kHz
**solo** valen medidos en el ESP32.

Y el lazo completo con `atp_ros` (`prueba_hil_host.sh`, ~1 min, tiempo real): `reference_generator` (rampa de 5°/s durante 40 s) →
agente → `eje_host` (acimut real simulado con J/b; elevación = `simulated`) → `mcu_merge` → `error_monitor`:

```
[reference_generator] lazo cerrado: arranca el pase      <- lo anuncia mcu_merge (/atp/torque), como rotctld_bridge
[error_monitor] RESULTADO n=781 | az: rms=0.2118 max=1.1960 | apuntamiento: rms=0.1835 p95=0.3854 max=1.0358 deg | dentro de 0.1 deg: 52.8%
eje_host[az]: fin. estado=ACTIVO pos=210.06 deg ref_cmd=210.00 deg refs_recibidas=873 error_rms_vs_ref_cmd=0.208 deg
```

**No es una cifra de apuntamiento de la montura**: la planta es la didáctica, el host no es de tiempo real y estaba cargado, la
referencia llega por DDS → agente → UDP, y el PID no tiene pre-alimentación (con `--kff` mejora el error de la rampa, ver `test_seguimiento_con_feedforward`).
Demuestra que **el lazo cierra de punta a punta con los nodos reales de la Estación**, no su precisión.

`prueba_micro_ros_host.sh` y `prueba_hil_host.sh` usan la imagen del agente fijada por digest y dejan el contenedor del agente parado al terminar.

`EJES=2 ROS_DOMAIN_ID=73 bash firmware/esp32_eje/scripts/prueba_micro_ros_host.sh` hace la misma verificación con **dos ejes en un solo nodo** (`eje_host --ejes 2`): un verificador por eje
(az a 9°/s, el a 1°/s) y los 10 tópicos `/atp/mcu/{az,el}/…` + `/joint_states` fusionado; las 15 comprobaciones de cada eje pasan (**TODO OK** ×2). Con `EJES=1` (el predeterminado) el resultado es el de siempre:
mismas 15 comprobaciones, mismas transiciones (ACTIVO en 0.46 s, RETENCION a 0.99 s, ESPERA a 2.99 s, ACTIVO en 0.50 s): **el firmware de 1 eje no cambió de comportamiento** con la generalización a 2 ejes.

### c) Banco HIL en **lockstep** (el ensayo general del primer encendido)

`prueba_hil_lockstep.sh` sustituye a `mount_sim` + `pid_controller` de la Estación por el nodo micro-ROS del firmware para Linux con **los dos ejes** (`eje_host --ejes 2 --eco --tel-hz 100`) y corre el pase N2 (`el_max:=85`, `mask:=70`: 52.8 s de pase) con
los nodos de siempre (`sim_clock` en lockstep, `reference_generator`, `error_monitor`) y un puente nuevo, **`atp_mcu/mcu_lockstep`**, que:

1. al llegar la primera referencia declara el origen (`cmd/origen` = referencia inicial) y arma (`cmd/enable`), exactamente como habrá que hacerlo con una placa real;
2. publica `/joint_states(t_k)` con el **estado del MCU interpolado en el instante de pared T_k** (T_k = T_0 + (t_k − t_0), ×1), y confirma `/atp/step_ack(t_k)` cuando llegó el **eco** de la referencia de t_k (el MCU ya la tiene) y la pared alcanzó T_{k+1}.
   Un control proporcional adelanta el ack lo que tarden `sim_clock` y el generador en reaccionar, para que la referencia salga en T_{k+1} y no después;
3. al terminar el pase calcula, con las **mismas** referencias, el **gemelo digital** (`atp_mcu/gemelo.py`: `PlantaMontura` + el PID de `pid_controller.py` a 20 Hz) y escribe `.cache/hil/hil.json` y `hil.csv` (t, ref, HIL, gemelo, RTT).

El MCU gira en **tiempo real**: el lockstep garantiza que cada estado y cada referencia llevan el mismo sello, pero **ya no** que el resultado sea independiente de la carga de la máquina.

Resultado de una corrida (agente `microros/micro-ros-agent:jazzy`, dominio 73, UDP; **host con carga 5.6 → 10.1 en 8 hilos por los otros equipos**: es una medida pesimista y repetible solo en orden de magnitud: en 3 corridas la mediana de la diferencia con el gemelo fue 0.015° (sin el adelanto del ack) y 0.004° (con él), y el máximo varió de 0.17° a 0.27°):

| Qué | Valor | Cómo se mide |
|---|---|---|
| pasos cerrados / duración | 1 056 pasos de 50 ms, 52.8 s | `mcu_lockstep` |
| **RTT de ida y vuelta** (receptor de la referencia en el puente → eco del MCU) | p50 **0.94 ms**, p95 19.4 ms, p99 24.8 ms, máx 28.0 ms (n = 1 051; 5 ecos > 0.3 s) | sellos de pared; mínimo −5.3 ms: el ejecutor de Python procesó el eco antes que la referencia (ruido de la medida, ±ms, **contado, no oculto**) |
| cierre del paso respecto de T_{k+1} | p50 0.17 ms (con el adelanto adaptativo; sin él 14 ms), p95 24 ms, máx 362 ms | idem; los máximos son tirones del planificador |
| **jitter del lazo** del MCU (periodo real del tick de 1 ms, `eje_host`, 62 569 ticks) | mín 7 µs, p50 1 000 µs, **p99 2 287 µs**, p99.9 3 817 µs, **máx 17 400 µs**; > 1.5 ms: 2 931 (4.7 %), > 2 ms: 860 (1.4 %) | `clock_nanosleep` absoluto sin tiempo real (`--stats`) |
| duración del cuerpo del tick (2 ejes, host x86) | p50 1.8 µs, p99 3.9 µs, máx 2.6 ms | los 2.6 ms son una expulsión del hilo, no el costo del lazo |
| `~/diag` | `t_lazo_max_us` 1 507, `jitter_max_us` 16 400, `ticks_perdidos` 6 399 de 62 569 | firmware (mismo código que el ESP32) |
| **deriva contra el gemelo** (diferencia en el cielo, √((Δaz cos el)² + Δel²)) | p50 **0.0039°**, p95 0.059°, p99 0.19°, rms 0.038°, máx 0.27°; la diferencia de **acimut** llega a 3.0° en un tirón de 360 ms (a 9°/s) | `gemelo.deriva` |
| error de apuntamiento propio | HIL rms **0.042°**, p95 0.098°, máx 0.28°, **95.0 %** del pase dentro de 0.1°; gemelo rms 0.0164°, máx 0.138° | `error_monitor` y `gemelo` |
| tendencia de la diferencia | 0.027° por minuto (pendiente de mínimos cuadrados) | `gemelo.deriva` (ruidosa: la dominan los tirones) |

**Lectura honesta.** (1) El enlace UDP por el agente cuesta **≈ 1 ms en la mediana** y decenas de ms en la cola: con el host cargado el RTT de la cola es el del planificador de Linux, no el del protocolo; el serie a 921 600 baudios
(1–3 ms estimados) y el Wi-Fi **hay que medirlos con la placa**. (2) El HIL sigue al gemelo a 4 milésimas de grado en la mediana; el 2.6× de diferencia de rms con el gemelo (0.042° contra 0.016°) viene de lo que **no** es igual: PID a 1 kHz con la referencia extrapolada y filtro de velocidad
del encoder, referencia retrasada ≈ 1 ms + cola, y tirones del host. (3) **Linux no es tiempo real**: el 4.7 % de los ticks llega > 0.5 ms tarde y el máximo fue 17 ms con carga 10; `--rt` (SCHED_FIFO) existe en `eje_host` pero **no se usó** (el contenedor no tiene CAP_SYS_NICE). Los tiempos del lazo de 1 kHz **solo** valen medidos en el ESP32 (`~/diag`).
(4) La planta de `eje_host` avanza **un paso por tick**, no por tiempo de pared: un tirón del host congela la planta simulada (en un motor real el eje seguiría girando). Por eso los máximos de la tabla son cotas pesimistas del efecto de un tirón del *control*.

Cuando llegue la placa, **cambia el transporte y nada más**: se arranca `agente.sh serie` (o `udp`) en lugar del agente de Docker de la prueba, se omite `eje_host`, se compila con `ATP_ECO_LATENCIA=y` (el eco ya existe) y `ATP_EJES=2` (`./build.sh dos`), y se lanza el mismo `ros2 launch atp_mcu mcu_hil_lockstep.launch.py` (`origen:=cmd`): `mcu_lockstep` y el gemelo son los mismos.
(Faltan: un motor de verdad en el lazo, el `ATP_MODO_SIMULADO` de la sección 9 punto 1 en el chip, y la medida de latencia con el serie.)

## 8. Recursos del ESP32

Compilación real (`./build.sh`, ESP-IDF v5.2.8, `-O2`, ESP32, 240 MHz) contra los límites de la hoja de datos de Espressif
(520 KB de SRAM, 448 KB de ROM, 4 MB de flash en el módulo típico: `codeaerospace_contenido/.../esp32.py`, `HOJA`):

| Recurso | Límite (hoja de datos / partición) | Variante **serie** | Variante **UDP/Wi-Fi** |
|---|---|---|---|
| `.bin` de la aplicación | flash 4 MB (4 194 304 B); partición de app 1.5 MB (1 536 000 B) | **300 608 B** (7.2 % de la flash; 19.6 % de la partición) | **848 896 B** (20.2 % de la flash; 55.3 % de la partición) |
| Imagen total (`esp_idf_size`) | — | 300 493 B | 848 781 B |
| Flash usada (`.text` + `.rodata`) | — | 228 183 B (173 711 + 54 216) | 728 791 B (618 091 + 110 444) |
| DRAM estática (`.data` + `.bss`) | 520 KB de SRAM en total; 180 736 B de DRAM estática utilizable (cifra de `esp_idf_size`) | 39 300 B (7.4 % de 520 KB; 21.7 % de la utilizable) | 60 904 B (11.4 %; 33.7 %) |
| IRAM estática | sale de los mismos 520 KB; 131 072 B utilizables | 54 202 B (10.2 %; 41.4 %) | 94 878 B (17.8 %; 72.4 %) |
| SRAM estática total (DRAM + IRAM) | 520 KB = 532 480 B | **93 502 B (17.6 %)** | **155 782 B (29.3 %)** |
| Bootloader | — | 27 408 B (0x6b10) | 26 816 B (0x68c0) |

**Tras generalizar a 1–2 ejes** (compilación real, `./build.sh`, `./build.sh udp`, `./build.sh dos`; 0 advertencias en `main/`, `uros/` y `core/`): serie 1 eje **304 384 B** de `.bin` (19.8 % de la partición), DRAM 40 924 B, IRAM 54 242 B;
**serie 2 ejes 304 720 B**, 41 180 B, 54 278 B; UDP 1 eje **852 496 B** (55.5 % de la partición), DRAM 62 536 B, IRAM 94 918 B. El crecimiento frente a la tabla (+3 776 B y +1 624 B de DRAM en serie) lo ponen los arreglos de `ATP_EJES_MAX = 2`, el eco de latencia (compilado, apagado en ejecución)
y las pozas de micro-ROS ampliadas (`RMW_UXRCE_MAX_PUBLISHERS` 3 → 5, `MAX_SUBSCRIPTIONS` 4 → 5 en `app-colcon.meta` y `host/colcon.meta`): con 2 ejes hacen falta 4 publicadores (+1 del eco) y 5 suscripciones.

* «DRAM estática» es `.data + .bss` (lo que ve el enlazador); el **heap libre en ejecución no se midió** (sin placa): los
  mensajes micro-ROS (`JointState` ×2, `Float32MultiArray`) y la pila de la tarea micro-ROS (12 KB) salen del heap. El
  Wi-Fi reserva además unos 40–60 KB de heap (cifra orientativa de Espressif, no medida aquí).
* La partición de aplicación es de 1.5 MB (`PARTITION_TABLE_SINGLE_APP_LARGE`): sin OTA.
* Desglose por componente: `build/tamano_componentes.txt`.

## 9. Qué falta para el primer encendido real

Ordenado por el riesgo que cierra:

1. **Probar primero sin potencia.** Compilar con `ATP_MODO_SIMULADO=y` y cargarlo: el ESP32 corre el lazo contra la planta J/b
   en firmware, sin encoder ni motor (el «HIL sin potencia» del plan). Ahí se **miden `t_lazo_us`, `jitter_max_us` y la
   latencia real del enlace** (serie y Wi-Fi), y se comprueba la reconexión con el agente. Nada de esto se ha podido hacer en el chip;
   **el ensayo en Linux sí está hecho** (§7c: `mcu_hil_lockstep.launch.py` con el mismo lanzador y el eco de latencia), de modo que con la placa solo cambia el transporte.
   Para el banco de dos ejes: `./build.sh dos` (`ATP_EJES=2`) y [HARDWARE](../../docs/HARDWARE.md) §3 (pines, tiempos) y §5 (qué comprar).
2. **Verificar contra las hojas de datos** lo que aquí está de memoria o supuesto: trama del AS5048A (paridad, `0xFFFF`, modo SPI 1),
   BTS7960 (kILIS = 8500, R_IS del módulo, frecuencia de PWM máxima, umbral lógico a 3.3 V), límites de corriente por GPIO.
3. **Identificar la montura** (J, b, fricción, holgura; [CONTROL §8](../../docs/CONTROL.md)) y **calibrar `u_pleno`** (el par al 100 % de PWM,
   hoy 8 N·m *supuesto*): las ganancias kp 25 / ki 2 / kd 8 son de J = 2 kg·m² y con el motor real pueden oscilar o ser flojas. Subir de
   a poco con la fuente limitada en corriente.
4. **Referenciado.** El encoder incremental no sabe dónde está al encender: hoy el origen se declara a mano (`cmd/origen`) y se pierde
   al reiniciar. Falta un procedimiento de *homing* con final de carrera/índice (y, si se quiere, guardar el origen en NVS). El encoder
   absoluto lo evita, pero está *SIN PROBAR*.
5. **Seguridad de potencia:** contactor/relé del paro independiente del ESP32, fines de carrera cableados, fusible, pull-down de EN
   (sección 3). Probar el paro, el corte de cable del paro, el corte del cable de datos (watchdog) y el reinicio del chip **con el motor desacoplado**.
6. Medir el **ruido real del encoder** y re-sintonizar el filtro de velocidad (`tau_vel`, 20 ms supuesto) y la derivada (80 ms).
7. Resolver **cómo se asigna el dominio ROS** en producción (hoy compilado: 42) y si hace falta un parámetro/servicio para cambiar
   ganancias en caliente (no hay servidor de parámetros: ahorra memoria).
8. Fuera de alcance de esta rama: LQR/Kalman en el ESP32, OTA, interferencia con Wi-Fi de otros equipos, certificación de seguridad funcional,
   `ros2_control` (equipo K) y el cierre de lazo con `reference_generator` en tiempo real (`atp_hw/hil.launch.py` aún habla con `rotctld`).

## Lo que **no** está verificado (resumen)

Todo lo que toca la placa: PCNT, LEDC, SPI, ADC, GPIO/ISR del paro, UART a 921 600 y Wi-Fi/UDP en el chip, tiempos reales, consumo de heap
y estabilidad con el motor. El código **compila** (ambas variantes, sin advertencias) y la lógica pura y el nodo micro-ROS están probados
en Linux, pero un periférico bien compilado no es un periférico probado.
