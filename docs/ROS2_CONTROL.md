# La montura en ros2_control y Gazebo (equipo K)

La vía estándar de la robótica ROS 2 para el mismo problema que resuelven
`mount_sim` + `pid_controller` / `lqr_controller`: la montura Az/El como un
`hardware_interface` de `ros2_control` y el lazo como un controlador. Con eso **el
mismo controlador mueve una montura simulada, una montura en Gazebo o una real con solo
cambiar el plugin de hardware** (una línea del URDF). Cada cifra de este documento sale de
una salida real que se reproduce con `scripts/ros2_control.sh` (sección 8). Lo que es un
**supuesto** o **no se verificó** está marcado así.

| Paquete | Qué es |
|---|---|
| `src/atp_description` | xacro de la montura (inercias, límites, `<ros2_control>`), meshes simples y mundo de Gazebo |
| `src/atp_control` | plugin de hardware `AtpMountSystem` (backends `sim` y `serial`), controlador `AtpTrackingController`, lazo en lockstep (`lockstep_node`), lanzadores, evaluador y pruebas |
| `docker/Dockerfile.gz` + servicio `gz` | imagen con `ros2_control` y Gazebo Harmonic (sección 2) |

No se modificó nada de `src/atp_ros` ni de `docker/Dockerfile`. Único archivo compartido
tocado: `docker-compose.yml` (servicio nuevo `gz`, aditivo).

## 1. Arquitectura

```
                       ┌────────── controller_manager ───────────┐
 /atp/reference ─────▶ │ AtpTrackingController  (PID | LQR, 20 Hz)│ ─▶ /atp/torque (JointState)
 (generador de atp_ros)│    ▲ position            │ effort        │
                       │    │                     ▼               │
                       │  ┌────────── hardware (UN plugin) ─────┐ │
                       │  │ AtpMountSystem  backend=sim    ──── dinamica de planta.py en read()
                       │  │                 backend=serial ──── microcontrolador (protocolo ASCII)
                       │  │ GazeboSimSystem (gz_ros2_control) ── montura fisica en gz sim
                       │  └─────────────────────────────────────┘ │
                       │ joint_state_broadcaster ─▶ /joint_states │
                       └───────────────────────────────────────────┘
 sin tocar: reference_generator, link_budget, error_monitor, recorder (atp_ros)
```

**Interfaces** (idénticas en todos los hardware; las declara el URDF, `antena.ros2_control.xacro`):

| Junta `az`, `el` | Nombre | Unidad | Quién la usa |
|---|---|---|---|
| comando | `effort` (límite ±`u_max`, 8 N m) | N m | `AtpTrackingController`, `effort_controllers` |
| comando | `position` | rad | `joint_trajectory_controller` (el hardware cierra un servo PD interno) |
| estado | `position` | rad | todos |
| estado | `velocity` | rad/s | broadcaster (el controlador **no** la usa: la estima, como el PID/LQR de Python) |
| estado | `effort` | N m | el mando realmente aplicado (lo recoge el grabador como `u_az`, `u_el`) |

El hardware se elige con `hw:=sim|serial|gz|mock` del xacro; lo demás (geometría,
inercias, límites, controladores y su YAML) no cambia.

### 1.1 `AtpMountSystem` (hardware_interface::SystemInterface)

* **`sim`**: integra en `read()` la misma dinámica que `mount_sim`,
  J θ'' + b θ' = u (+ fricción de Coulomb/Stribeck con adherencia Karnopp, backlash de dos
  cuerpos, desbalance de elevación, cuantización del encoder). Es un port, operación por
  operación, de `planta.py` (`mount_plant.cpp`), con el mismo subpaso de 2 ms. `write()`
  retiene el mando (retención de orden cero) para el siguiente `read()`.
* **`serial`**: habla con un microcontrolador por el protocolo de la sección 5. Para
  probar sin hardware, `serial_port: fake` arranca un **microcontrolador falso en proceso**
  que implementa el protocolo y la misma planta.
* **Modos de comando**: con `effort` reclamado el mando va directo; con `position`
  reclamado un servo PD (`servo_kp`, `servo_kd`) lo convierte en par; sin controlador, par
  cero. El cambio de modo usa `prepare/perform_command_mode_switch`.
* Parámetros de `<hardware>` (todos con valor por defecto): `backend`, `J`, `b`,
  `vel_max_deg_s`, `substep_s`, `coulomb_nm`, `stribeck_nm`, `stribeck_vel_deg_s`,
  `friction_veps_deg_s`, `backlash_deg`, `backlash_motor_frac`, `backlash_restitution`,
  `unbalance_nm`, `encoder_bits`, `servo_kp`, `servo_kd` y, para `serial`: `serial_port`,
  `serial_baud`, `serial_cpr`, `serial_timeout_ms`.
* No portado de `mount_sim` (apagado por defecto allí también): ruido gaussiano del
  encoder, ráfaga y viento con espectro (RNG de numpy no reproducible en C++) y
  `/atp/mount_truth`. Ver sección 9.

### 1.2 `AtpTrackingController` (controller_interface::ControllerInterface)

Lee `/atp/reference` (`JointState`, rad) y cierra el lazo con la ley de la casa a
`rate_hz` = 20 Hz: **PID** (`pid_controller.py`: derivada filtrada, anti-windup por clamping),
**LQR integral** con pre-alimentación (`control.py`: Riccati discreta por duplicación
estructurada calculada en C++, K = [83.97, 17.77, 24.60]) y, para ambos, `estimator:=kalman`
(Kalman de [θ, ω, par de sesgo]). Escribe el par en `effort`, lee `position` y publica el
mando en `/atp/torque` (sellado), que el generador de referencia necesita para arrancar el pase
y el grabador para `u_az`/`u_el`.

Dos decisiones que importan fuera del lockstep (donde la referencia del instante t llega
*después* de que el controlador necesita calcular con ella):

* **Extrapolación de la referencia**: con `extrapolate_ref` la referencia se lleva al
  instante de cálculo con el polinomio de Lagrange de sus 3 últimas muestras *con sus sellos
  reales* (no se supone paso uniforme). Si el sello coincide (lockstep) no se extrapola y el
  cálculo es idéntico al de Python. Con sellos que tiemblan ~1 ms, el LQR (que deriva la
  referencia dos veces, ÷ Ts²) es muy sensible: con paso uniforme supuesto el LQR en reloj
  de pared daba p95 0.0094° y con Lagrange 0.0027° (sección 8.3).
* **Ticks de la ley**: uno cada 50 ms de tiempo del `controller_manager`, con tolerancia de
  medio ciclo de este. Una rejilla absoluta de 50 ms (probada) hace que un sello 1 ms
  adelantado cambie el ciclo del tick y el paso entre ticks salte de 50 a 41/59 ms: el LQR
  (diseñado para Ts = 50 ms) lo paga con picos de par de 2 N m.

### 1.3 Lazo en lockstep sin Gazebo: `lockstep_node`

`ros2_control_node` (el estándar) mueve el lazo con el reloj de pared y no espera a nadie.
`lockstep_node` es el mismo `controller_manager` movido por el `/clock` de `sim_clock`, para
correr el **mismo plugin y el mismo controlador** en la simulación determinista del
CONTRATO §8. En cada `/clock` t_k:

```
read(t_k) → [espera la referencia con sello t_k] → update → write → [espera /joint_states(t_k)] → /atp/step_ack(t_k)
```

Es el protocolo de `mount_sim` (el generador, el monitor y el reloj no notan la diferencia),
con `mode:=free` no espera ni confirma, y con `mode:=realtime` ni siquiera se usa: corre el
`ros2_control_node` estándar con reloj de pared a 100 Hz. Tres defectos de ros2_control
Jazzy 4.48 que el lockstep tuvo que rodear (los tres medidos, no supuestos):

1. `update()` **se salta el ciclo sin avisar** si otro hilo (diagnósticos, servicios) tiene
   la lista de controladores. En tiempo real es un ciclo perdido; en lockstep es un paso sin
   mando ni estado y el LQR se perturbaba durante ~4 s. Se detecta (el controlador anota el
   sello del último `update`) y se reintenta: un ciclo saltado no ejecuta nada.
2. El `joint_state_broadcaster` publica desde un hilo propio y **descarta la muestra** si el
   hilo sigue ocupado con la anterior (probablemente por su QoS de sistema por defecto, que en Fast DDS
   es de profundidad 1: no lo verifiqué).
   `lockstep_node` espera ver pasar el estado de cada instante por `/joint_states` antes de
   confirmar el paso y, si no llega, repite `update()` (idempotente para el controlador).
3. `get_loaded_controllers()` desde el hilo del lazo se interbloquea con `load_controller`
   (que espera a que ese hilo avance): el arranque consulta los controladores desde un hilo
   aparte.

## 2. Imagen de Docker

La imagen `ros2-jazzy-dev` **no trae `ros2_control` ni Gazebo** (`ros2 pkg list` no muestra
`controller_manager`, `hardware_interface`, `ros_gz*` ni `gz_ros2_control`). Sin tocar
`docker/Dockerfile`, `docker/Dockerfile.gz` hereda de ella y añade `ros2_control`,
`ros2_controllers`, `xacro`, `ros-jazzy-ros-gz` y `ros-jazzy-gz-ros2-control` (Gazebo
Harmonic, gz sim 8.15, ros2_control 4.48). El servicio nuevo es `gz` (`ros2-jazzy-gz`):

```bash
docker compose build gz        # una vez
docker compose run --rm -T -e ROS_DOMAIN_ID=82 gz bash scripts/ros2_control.sh test n2
```

**Tamaño**: la capa añadida pesa **637 MB descomprimida** (≈170 MB comprimida, lo que
`docker image inspect` da como diferencia de tamaño de contenido: 1.45 GB contra 1.28 GB de
`ros2-jazzy-dev`); la imagen completa ocupa ≈ 6.4 GB en disco frente a los 5.73 GB de la
base. En la imagen base `colcon build` **omite `atp_control`** con un aviso de CMake (no falla: CI y
`validar.sh` siguen verdes; comprobado construyendo los 8 paquetes en `ros2-jazzy-dev`) y
`atp_description` se construye igual (solo instala archivos). Detalle de red: con la red `host` del contenedor, gz-transport no descubre servicios
por la interfaz por defecto; hay que fijar `GZ_IP=127.0.0.1` (lo hacen el servicio `gz` y los
lanzadores).

## 3. Cómo correrlo

```bash
# dentro del servicio gz (arriba): lockstep, sin Gazebo
ros2 launch atp_control atp_ros2_control.launch.py el_max:=85 mask:=70 time_scale:=0
ros2 launch atp_control atp_ros2_control.launch.py controller:=lqr estimator:=kalman \
    backlash_deg:=0.05 coulomb_nm:=0.05 stribeck_nm:=0.02 encoder_bits:=14
ros2 launch atp_control atp_ros2_control.launch.py backend:=serial        # MCU falso en proceso
ros2 launch atp_control atp_ros2_control.launch.py mode:=realtime         # ros2_control_node estandar

# Gazebo (gz sim headless, gz_ros2_control)
ros2 launch atp_control atp_gz.launch.py                                   # tiempo real
ros2 launch atp_control atp_gz.launch.py stepped:=true                     # paso a paso
ros2 launch atp_control atp_gz.launch.py headless:=false                   # con GUI (necesita X11)

# modo posicion (joint_trajectory_controller) e identificacion de la planta
ros2 launch atp_control atp_position.launch.py backend:=sim
ros2 launch atp_control atp_identify.launch.py backend:=gz axis:=0
```

### Animación: la antena siguiendo al satélite (`scripts/gz_demo.sh`)

```bash
bash scripts/gz_demo.sh                    # en el host: pase al cenit, máscara 45° (139 s de pase), graba video
bash scripts/gz_demo.sh el_max:=60 mask:=20   # cualquier argumento de atp_gz.launch.py
VIDEO=0 bash scripts/gz_demo.sh            # sin grabar
```

Abre la ventana de Gazebo en tu pantalla (X11, render por software: el contenedor no tiene
`/dev/dri`) y deja en `runs/gz_demo/` (fuera de git) `demo.mp4` y `eval.json`, el error de
apuntamiento medido en Gazebo (`parcial: true` si algo detuvo la corrida antes de acabar el pase;
corrida de referencia: rms 0.0090°, p95 0.0174°, máx 0.0858°, 100 % dentro de 0.1°). Con `escena:=true` el lanzador genera una copia del mundo con un **satélite
visible** (esfera con paneles), el **arco del pase** (puntos celestes) y la cámara, y arranca
`atp_gz_escena.py`. Es una ayuda visual **a escala**: el satélite real está a 550+ km; aquí se
dibuja a `radio_escena` (15 m) del cruce de las juntas, en la misma dirección (az, el) de
`/atp/reference`. La antena la mueve la física de Gazebo con el lazo de siempre (PID/LQR), de
modo que, si el lazo fallara, el eje del plato se separaría de la esfera. Solo es una vista:
no entra en ninguna cifra ni en el lockstep.

Los lanzadores aceptan los argumentos de `atp_loop.launch.py` (`el_max`, `mask`, `controller`,
`estimator`, planta realista, `record_dir`…). `pointing_eval.py` (parte de todos) imprime
`EVAL …`: el error de apuntamiento con la misma fórmula que `error_monitor`, pero
interpolando la referencia en el sello de cada medida (en lockstep da **exactamente** las
mismas cifras que el monitor; en Gazebo, donde los sellos casi nunca coinciden, es la cifra
válida: el monitor solo empareja sellos idénticos y vio 172–243 de 5095 instantes).

## 4. Del simulador al hardware real

El cambio es **una línea del URDF** (`hw:=serial`) más los parámetros del puerto; el
controlador, su YAML y los lanzadores no cambian:

```bash
ros2 launch atp_control atp_ros2_control.launch.py mode:=realtime backend:=serial \
    serial_port:=/dev/ttyUSB0 serial_baud:=921600
```

Lista de verificación (lo que sí está hecho y lo que debe hacerse en el banco):

1. **Firmware** que implemente el protocolo de la sección 5 (o adaptar `serial_protocol.cpp`,
   un solo archivo): recibe `$C` con el par (N m) de los dos ejes, aplica su propio límite de
   corriente/par, devuelve `$S` con las cuentas del encoder a ≥ 20 Hz y **corta el par si
   deja de recibir `$C`** (watchdog del MCU). Hecho: protocolo, codec con XOR, resincronización,
   `PosixSerialPort` y MCU falso. **No probado con hardware ni con el firmware del ESP32** de
   otro equipo (no estaba en esta rama): el protocolo es un borrador de interfaz.
2. **Identificar la planta**: J, b, Fc, Fs, ωs, holgura y m g d son supuestos aquí
   ([CONTROL](CONTROL.md) §8). `atp_identify.launch.py` + `identify_plant.py` hacen el escalón
   de par (J = τ b) con cualquier hardware; con la planta de `sim` da J = 1.9999, b = 0.49998
   y con Gazebo J = 2.010, b = 0.50001 (sección 8.4).
3. **Re-sintonizar** `kp, ki, kd`, `q_*`, `kf_q_acc`, `kf_q_par` con la planta identificada y
   el ruido real del encoder (`enc_sigma_deg`, `enc_bits`); el presupuesto de retardo del LQR
   es 92.5 ms ([CONTROL](CONTROL.md) §2) y el lazo serie debe caber en él.
4. **Seguridad**: `<command_interface name="effort">` lleva `min/max` en el URDF y el
   plugin satura; el servo de posición también. El `serial_timeout_ms` del host pasa el
   hardware a ERROR si el MCU calla (medido en el tiempo del *lazo*, no de pared, para que
   valga igual en lockstep); una falla del MCU (`flags` bit 0) hace lo mismo. Falta
   final de carrera y paro de emergencia físicos.
5. **Tiempo real**: `ros2_control_node` con `update_rate: 100` y prioridad RT (en el
   contenedor no hay permiso: "Could not enable FIFO RT scheduling policy"). Sin lockstep el
   monitor empareja con "la última referencia" y avisa: use `pointing_eval`.
6. **Encoder**: con `encoder_bits` y el lado del encoder (`encoder_side`) el modelo `sim` ya
   reproduce la cuantización de [CONTROL](CONTROL.md) §1; con el MCU real las cuentas por vuelta
   son `serial_cpr` (65536 por defecto = 0.0055°).

Otras vías al mismo hardware, ya preparadas por la interfaz: **Gazebo** (`hw:=gz`, sección 7) y,
cuando el equipo J entregue el firmware micro-ROS, un tercer backend que en lugar de bytes
hable con los tópicos del agente (el resto del plugin no cambia: `MountBackend` tiene cuatro
métodos).

## 5. Protocolo serie (borrador)

ASCII, una trama por línea, comprobación XOR al estilo NMEA (`serial_protocol.hpp`):

```
$<tipo>,<campos>*<HH>\n            HH = XOR de los bytes entre '$' y '*', 2 hex
host → MCU  $C,<seq>,<dt_us>,<tau_az>,<tau_el>*HH     mando de par (N m)
            $Z*HH                                      fijar el cero de los encoders
MCU → host  $S,<seq>,<t_us>,<az_cnt>,<el_cnt>,<az_rate>,<el_rate>,<flags>*HH
            flags: bit0 falla/paro, bit1 con cero fijado, bit2 *_rate valida (cuentas/s)
```

Si el MCU no mide velocidad (`bit2` en 0) el host la deriva de la posición con un filtro de
primer orden (`serial_vel_tau_s`). Las tramas mal formadas se descartan y se cuentan; el
puerto falso inyecta basura y tramas corruptas en las pruebas.

## 6. Pruebas (gtest, `scripts/ros2_control.sh test`)

31 pruebas gtest (`test_plant`, `test_laws`, `test_serial`, `test_controller_ref`) que no
necesitan ROS en marcha. Las referencias numéricas las genera el código **Python** de la casa
(`src/atp_control/test/gen_ref.py`, que usa `atp_ros.planta` y `atp_ros.control`) y se versionan en
`test/data/`, de modo que gtest no depende de Python.

| Prueba | Qué contrasta | Resultado medido |
|---|---|---|
| `Planta.ReproducePlantaPython` | el backend `sim` (`MountPlant`) contra `PlantaMontura` con el mismo mando (retención a 20 Hz, subpaso 2 ms, ráfaga de 1 N m): ideal, tope de velocidad, fricción Stribeck/Karnopp, backlash, backlash con restitución 0.4 y "realista" (fricción + backlash + desbalance); 12 s por caso | max\|Δθ\| = max\|Δω\| = max\|Δθ_motor\| = **0.0** en los 6 casos |
| `Planta.IdealIdenticaBitABit` | caso ideal con igualdad exacta (`==`) en 240 pasos | idéntico bit a bit |
| `Leyes.GananciaLqrIgualQuePython` | K del LQR integral calculado en C++ (Riccati SDA) contra `control.lqr_integral` y contra las cifras de CONTROL.md | K = [83.974583656, 17.766218521, 24.596515859] en C++ y en Python; \|z\|max = 0.984313 |
| `Leyes.LazoCerrado` (×4) | planta + ley en lazo cerrado a 20 Hz durante 20 s, contra Python: PID, PID + Kalman, LQR, LQR + Kalman (los dos últimos con fricción, backlash y encoder de 14 bits) | max\|Δu\| = 0, 4.4e-16, 8.9e-16, 8.1e-14 N m |
| `Leyes.DisenoKalmanIgualQuePython`, `RiccatiContraFormaCerradaEscalar`, `LeyDesconocidaLanza`, `SaturaEnUMax` | modelo y covarianza del Kalman (1e-12), Riccati escalar de forma cerrada, errores de configuración, saturación a ±8 N m | pasan |
| `Serial.*` (12) | codec y XOR, resincronización con basura y tramas partidas, MCU falso, **equivalencia `serial` contra `sim`** con el mismo mando durante 10 s, velocidad derivada si el MCU no la mide, silencio del MCU (falla en el 5.º–6.º ciclo mudo de 50 ms, con timeout de 200 ms), falla reportada por el MCU, ruido de línea, puerto inexistente | `serial` contra `sim`: max\|Δpos\| = 4.79e-5 rad = media cuenta del encoder de 65536 cuentas (4.79e-5); el resto pasa |
| `RefHistory.*` (5) | historial de la referencia y extrapolación: exacta para un polinomio de 2.º orden con sellos uniformes e **irregulares** (1e-12), lineal con 2 muestras, sin extrapolar si el sello coincide o la espera es > 250 ms | pasan |

**Tolerancia justificada**: se fijó en 1e-9 rad (rad/s) para la planta y 1e-7 N m para las leyes.
Sin capas opcionales el código C++ repite las *mismas* operaciones en el *mismo* orden que numpy
y el resultado es idéntico bit a bit (la prueba exacta lo exige); con fricción (`exp` de libm contra
el de numpy), backlash y Kalman se esperaba una diferencia de redondeo (1e-16 – 1e-13) y solo una
conmutación distinta en un subpaso (el mismo evento un paso antes o después) la haría crecer a
~1e-5: la tolerancia detecta eso. El resultado observado (0.0) es mejor que la tolerancia.
La prueba `serial` contra `sim` no puede ser exacta por construcción: el MCU transmite el
ángulo en cuentas enteras.


## 7. Gazebo

`atp_gz.launch.py` arranca, todo dentro del servicio `gz`:

| Proceso | Qué hace |
|---|---|
| `gz sim -s -r atp.sdf` | servidor de Gazebo Harmonic **sin pantalla** (DART, paso de 1 ms, tiempo real). Con `headless:=false` abre la GUI (no probado: el contenedor no tiene X) |
| `robot_state_publisher` | publica `/robot_description` (xacro con `hw:=gz`) |
| `ros_gz_sim create` | crea la montura en el mundo desde `/robot_description`; el plugin `gz_ros2_control` que lleva el URDF levanta **dentro del proceso de gz** un `controller_manager` con el `atp_controllers.yaml` |
| `ros_gz_bridge` | `/clock` de Gazebo a ROS |
| spawners | `joint_state_broadcaster` y `atp_tracking_controller`: **los mismos** que con `AtpMountSystem` |
| `reference_generator`, `link_budget`, `error_monitor`, `pointing_eval` | los del lazo de `atp_ros`, con `use_sim_time`; sin `sim_clock` (el reloj es el de Gazebo) |

El `<ros2_control>` del URDF solo cambia el plugin (`gz_ros2_control/GazeboSimSystem`); las
juntas, interfaces y límites son las mismas. **Mundo** (`worlds/atp.sdf`): suelo, sol y una losa de
concreto bajo el pedestal; la montura (pedestal, yugo, plato paraboloidal de 3 m con f/D = 0.4
y alimentador, meshes STL generados por `scripts/gen_meshes.py`) se crea al lanzar. Colisión con
primitivas (cilindros y cajas).

**Planta física**: J = 2 kg m² por eje y b = 0.5 N m s/rad, como la planta de `planta.py`. Se
logran con inercia esférica de 2.0 kg m² en el cuerpo de elevación (J_el = 2.0; su aporte al eje
de acimut es 2.0 a cualquier elevación) más 0.01 kg m² en el yugo (**J_az = 2.01**), amortiguamiento
de junta `b` y centro de masa sobre los ejes (montura equilibrada, sin carga gravitatoria; el
desbalance de elevación es un parámetro aparte). Esa elección es un **supuesto de modelado**: un
plato real de 3 m tiene una inercia órdenes de magnitud mayor ([CONTROL](CONTROL.md) §9).

**Identificación** (escalón de par de 1 N m en acimut, `atp_identify.launch.py`, ajuste por mínimos
cuadrados de w(t) = w∞(1 − e^−t/τ) con la latencia del par como parámetro, y caída libre al quitarlo):

| Eje | Hardware | J subida / caída libre (kg m²) | b (N m s/rad) | τ subida / caída (s) |
|---|---|---|---|---|
| az | `AtpMountSystem` sim con `ros2_control_node` estándar | 1.9999 / 1.9994 | 0.49998 | 4.000 / 3.999 |
| az | **Gazebo** | 2.0101 / 2.0103 | 0.50001 | 4.020 / 4.020 |
| el | `AtpMountSystem` sim | 1.9998 / 1.9993 | 0.49996 | 4.000 / 3.999 |
| el | **Gazebo** | 1.99997 / 2.0002 | 0.49999 | 4.000 / 4.000 |

(Detalle y una anomalía no explicada en la sección 8.4.)

**Modo paso a paso** (`stepped:=true`): `gz sim` arranca en pausa y `atp_gz_stepper.py` lo avanza
con `multi_step` por el servicio `/world/atp/control` (bindings Python de gz-transport) en pasos
de 50 ms. Fase A (arranque): avanza sin esperar hasta ver el primer mando de `/atp/torque` (el
`controller_manager` solo activa un controlador mientras la simulación avanza). Fase B: tras
cada paso espera, en orden, a que `/clock`, `/joint_states` y `/atp/reference` lleguen al instante
del paso y deja 4 ms de pared para que lleguen a los demás nodos. El orden de llegada de los
mensajes entre procesos sigue siendo del sistema operativo, así que **no es bit-exacto**:
reduce la dispersión (sección 8.5) pero no la elimina; lo que queda es el instante en que el
generador de referencia ve el primer mando (arranca el pase en el reloj que tenga en ese
momento, dentro de la ventana del paso).


## 8. Resultados

Todas las corridas: pase cenital N2 (el_max 85°, máscara 70°, 9 °/s en acimut, keyhole,
banda S, 50.9 s, planta ideal salvo que se diga), `time_scale:=0`, `ROS_DOMAIN_ID=82`. Máquina
de desarrollo con carga media de 27–35 sobre 8 núcleos (otros equipos corriendo a la vez):
condiciones peores que las del CI. Error de apuntamiento sqrt((e_az cos el)² + e_el²) en grados,
objetivo 0.1°. `error_monitor` imprime con 4 decimales.

### 8.1 N2 con ros2_control contra el lazo de referencia (lockstep, sin Gazebo)

`bash scripts/ros2_control.sh n2_ref n2 c1_ref c1 c2_ref c2 serial` (salidas literales de `error_monitor`):

| Escenario | Lazo de referencia (`mount_sim` + nodo Python) | **ros2_control** `AtpMountSystem` backend `sim` |
|---|---|---|
| PID, planta ideal (N2) | `n=1018 rms=0.0167 p95=0.0173 max=0.1382` | `n=1018 rms=0.0167 p95=0.0173 max=0.1382` |
| LQR, planta ideal (C1) | `n=1018 rms=0.0070 p95=0.0027 max=0.0967` | `n=1018 rms=0.0070 p95=0.0027 max=0.0967` |
| LQR + Kalman, planta realista (C2: backlash 0.05°, Coulomb 0.05 + Stribeck 0.02 N m, encoder 14 bits) | `n=1018 rms=0.0099 p95=0.0172 max=0.0655` | `n=1018 rms=0.0099 p95=0.0172 max=0.0655` |
| PID, backend `serial` (MCU falso, 65536 cuentas/rev) | — | `n=1018 rms=0.0169 p95=0.0185 max=0.1403` |

(az/el también coinciden: PID az rms 0.0599 max 0.1391, el rms 0.0142 max 0.1336.) Este
`rms=0.0167` es el que fija `scripts/validar.sh n2`.

**Explicación de la diferencia: no hay, y es lo esperado.** El lazo ros2_control ejecuta la misma
cadena del CONTRATO §8 con las mismas piezas: la planta del plugin es un port de `planta.py` con
resultados idénticos bit a bit (sección 6), las leyes coinciden a 1e-13 N m, el controlador
calcula con medida y referencia **del mismo sello** (en lockstep no se extrapola nada) y el mando
se retiene 50 ms igual que en `mount_sim`. Las cifras impresas coinciden en los 4 decimales;
**bit a bit entre los dos lazos completos no se comparó** (la comparación fila a fila de
`telemetry.jsonl` de una corrida contra otra sí: sección 8.2). La única diferencia real aparece
con `serial`, y es la **cuantización del encoder** que impone el protocolo: el MCU transmite
cuentas enteras de 0.0055° (65536 por vuelta), la derivada filtrada del PID las amplifica
y el p95 sube de 0.0173° a 0.0185° (rms 0.0167° → 0.0169°); es lo que costaría un encoder así,
no un defecto del plugin.

`pointing_eval` (el evaluador que no exige sellos iguales) da, en lockstep, exactamente las mismas
cifras que el monitor en las cuatro corridas (n2, c1, c2, serial): es su comprobación cruzada.

### 8.2 Determinismo

`bash scripts/ros2_control.sh det` (N2 dos veces con grabador, `telemetry.jsonl` y `metrics.json`
comparados con `cmp`): **idénticos byte a byte, 511 filas, en 6 de 6 ejecuciones** del nivel (5 de ellas con el
código final; cada ejecución compara 2 corridas) y en una comprobación manual de 3 corridas. Antes de los
arreglos de la sección 1.3 (ciclos saltados por el `controller_manager`, muestras descartadas por
el broadcaster y pausa mínima de 10 ms por paso) las 2 primeras ejecuciones dieron "NO DETERMINISTA"
y el LQR se perturbaba durante ~4 s tras cada ciclo saltado: con la máquina cargada el
lockstep sobre `ros2_control` es **más frágil** que sobre nodos Python con QoS profundo; se ganó con
esas tres defensas, no es gratis. Pasos mínimos de pared: `min_step_wall_s:=0.01` por defecto
(el pase tarda ~10 s de pared); a 0 el monitor perdió de 1 a 10 de 1018 muestras con la máquina cargada.

### 8.3 Sin lockstep: `ros2_control_node` estándar con reloj de pared (`mode:=realtime`)

Mismo plugin y mismo controlador, `controller_manager` a 100 Hz de pared, la referencia llega por
el tópico cuando llega. `EVAL` (el monitor, con sellos que no coinciden, empareja con "la última
referencia" y da 0.0233° rms: **no es válido aquí**; ver `no coinciden` en el log):

| Controlador | `EVAL` n=5095–5100 (muestras a 10 ms) |
|---|---|
| PID | rms 0.0167, p95 0.0173, max 0.1379 |
| LQR | rms 0.0071, p95 0.0027, max 0.0983 |
| LQR, referencia extrapolada con paso uniforme supuesto (antes del arreglo) | rms 0.0075, **p95 0.0094** |

El PID no nota la diferencia de tiempo (es de tipo 2 y la referencia llega con un retardo
de ~10 ms que la extrapolación corrige). El LQR deriva la referencia dos veces (÷ Ts² = ×400) y es
sensible a cualquier error en ella: la extrapolación con Lagrange sobre los sellos reales la deja
en el valor del lazo de referencia. `mode:=free` (el reloj de `sim_clock` a 20 Hz sin confirmar
pasos) es peor: PID `EVAL` rms 0.0177 y p95 0.0307, porque el estado y la referencia de cada
instante nunca se emparejan (el controlador usa la referencia del paso anterior).

### 8.4 Identificación de la planta (`atp_identify.launch.py`)

| Eje | Hardware | J de la subida / de la caída libre (kg m²) | b (N m s/rad) | τ subida / caída (s) |
|---|---|---|---|---|
| az | `AtpMountSystem` sim, `ros2_control_node` estándar | 1.9999 / 1.9994 | 0.49998 | 4.000 / 3.999 |
| az | **Gazebo** | 2.0101 / 2.0103 | 0.50001 | 4.020 / 4.020 |
| el | `AtpMountSystem` sim | 1.9998 / 1.9993 | 0.49996 | 4.000 / 3.999 |
| el | **Gazebo** | 1.99997 / 2.0002 | 0.49999 | 4.000 / 4.000 |

(El J_az de Gazebo, 2.010, es el diseñado: 2.0 del cuerpo de elevación + 0.01 del yugo.)
Una de cuatro identificaciones de `el` en Gazebo dio una **anomalía no explicada** (J = 0.47,
b = 0.75, deriva en reposo de 0.03 rad/s; la repetición dio los valores de la tabla), y una
de `az` con un ajuste anterior mostró una deriva inicial de 0.48 rad/s: probablemente un
transitorio del arranque de Gazebo bajo carga (la posición inicial se impone al crear la
montura); no lo aislé.

### 8.5 N2 en Gazebo (gz sim headless + gz_ros2_control)

`bash scripts/ros2_control.sh gz gz_step gz_lqr` (3 corridas de cada una; el monitor solo empareja los
sellos que coinciden, 172–243 de 5095: la cifra es la de `EVAL`, medida en los 5095 instantes de 10 ms):

| Corrida | n | rms (°) | p95 (°) | max (°) | dentro de 0.1° |
|---|---|---|---|---|---|
| **referencia: lazo PID** (`validar.sh n2`) | 1018 | 0.0167 | 0.0173 | 0.1382 | 99.3 % |
| Gazebo, tiempo real, PID (×3) | 5095 | 0.0168, 0.0168, 0.0168 | 0.0173 (×3) | 0.1388, 0.1379, 0.1388 | 99.3 % |
| Gazebo, paso a paso, PID (×3) | 5094–5100 | 0.0168 (×3) | 0.0173 (×3) | 0.1385, 0.1381, 0.1388 | 99.3 % |
| **referencia: lazo LQR** (C1) | 1018 | 0.0070 | 0.0027 | 0.0967 | 100 % |
| Gazebo, tiempo real, LQR (×3) | 5095–5100 | 0.0070, 0.0072, 0.0070 | 0.0031, 0.0029, 0.0033 | 0.0977, 0.0987, 0.0974 | 100 % |
| Gazebo, paso a paso, LQR (×3) | 5094–5095 | 0.0071 (×3) | 0.0031, 0.0032, 0.0032 | 0.0976, 0.0974, 0.0980 | 100 % |
| Gazebo, tiempo real, LQR + Kalman (×3) | 5095–5100 | 0.0022, 0.0021, 0.0021 | 0.0010 (×3) | 0.0466, 0.0440, 0.0440 | 100 % |

Con Gazebo el apuntamiento es **el del lazo de referencia**: PID 0.0168° contra 0.0167° (+0.6 %),
LQR 0.0070–0.0072° contra 0.0070°, LQR + Kalman 0.0021–0.0022° (CONTROL.md da 0.0020° para el
mismo caso en el lazo con planta ideal). Lo que no es idéntico y por qué:

* **n**: 5095 instantes a 10 ms (periodo del `controller_manager`) contra 1018 a 50 ms.
* **max** +0.0006° y **p95 del LQR** 0.0031° contra 0.0027°: la planta de Gazebo (J_az = 2.01,
  DART con paso de 1 ms y amortiguamiento implícito, frente a semi-implícito de 2 ms) no es la del
  modelo del controlador, y la referencia que usa la ley está extrapolada ~9 ms (no hay lockstep).
* **Dispersión entre corridas** (la de Gazebo): max del PID 0.1379–0.1388°, rms ±0.0001°, p95 del
  LQR 0.0029–0.0033°; n de 5094 a 5100 (el generador arranca el pase en el reloj que ve al
  recibir el primer mando). El paso a paso **no la redujo de forma medible** una vez corregido el
  muestreo del controlador (sección 1.2): antes de ello, con la ley corriendo en una rejilla
  desfasada según cuándo llegó la primera referencia, el tiempo real daba rms 0.0167–0.0169 y
  max 0.1373–0.1377 y el paso a paso 0.0167 (×3) con max 0.1386–0.1390. Lo que sí da el
  paso a paso es **independencia de la carga de la máquina**: avanza cuando el lazo cierra el paso,
  no cuando el reloj de pared lo exige.
* Una corrida anómala con LQR (az max 0.35°, p95 0.0185°) apareció antes de arreglar la
  tolerancia del tick de la ley (sección 1.2) y no se ha repetido con el código final (6
  corridas LQR con el código final, sin ella).
* Factor de tiempo real (una medición): el pase de 50.9 s duró ~55 s de pared con la carga
  descrita (≈ 0.93).


## 9. Limitaciones y lo no verificado

* **Gazebo no es determinista** como el lockstep: el reloj es el del simulador y el orden de
  llegada de los mensajes entre procesos es del sistema operativo. Dispersión medida entre
  corridas: sección 8.5. `error_monitor` solo empareja los sellos que coinciden y en Gazebo cubre
  ~4 % de los instantes: para medir se usa `pointing_eval` (interpolación lineal de la referencia
  a 20 Hz: error ≈ 1e-5°, no comparado contra una referencia analítica).
* **Paso a paso**: avanza con el lazo y no con el reloj de pared, pero no es bit-exacto (sección 7)
  y no se midió a una velocidad distinta de la de tiempo real. El paso es de 50 ms; con otro
  `step_s` la fase del primer mando cambia.
* **Lockstep sobre `ros2_control` con la máquina cargada**: necesita `min_step_wall_s` (10 ms por
  defecto) y los reintentos de la sección 1.3; sigue habiendo una probabilidad pequeña de
  que el broadcaster descarte una muestra (el log avisa `no llego a /joint_states`; en 6
  corridas de `det` no hubo ninguna dentro del pase). No probé el lockstep en una máquina
  descargada ni en el CI de 2 vCPU.
* **Hardware real**: nada se probó con un motor, un encoder ni un microcontrolador. El protocolo
  serie es un borrador mío (el firmware del ESP32 de otro equipo no estaba en esta rama), el MCU falso
  responde de forma síncrona (un MCU real tiene latencia propia que el lazo debe caber en el
  margen de retardo de 92.5 ms) y no hay prueba de pérdida de bytes a 921600 baudios ni de
  `PosixSerialPort` contra un puerto real (solo el caso "no existe"). El plugin no tiene
  `/atp/mount_truth`: con backlash o cuantización el error medido por el encoder y el real
  difieren y aquí solo se ve el medido (como en C2 con el lazo sin `truth:=true`).
* **Planta de Gazebo**: J = 2 kg m² es la inercia didáctica ([CONTROL](CONTROL.md) §9) y el centro
  de masa está sobre los ejes por construcción; no hay viento, fricción, backlash ni encoder
  en Gazebo (los parámetros `coulomb_nm`, etc. solo afectan al backend `sim`). Una de cuatro
  identificaciones de `el` dio un valor anómalo no explicado (sección 8.4).
* **No portado a C++**: ruido gaussiano del encoder, ráfaga, viento con espectro y `mount_truth`
  (las dos primeras eran de `mount_sim` con valor cero por defecto). Los niveles N4b (ráfaga en Ka)
  y N5 (autotrack) no se corrieron con ros2_control.
* **Perfil `tle`**: los lanzadores solo cablean `profile:=pass|ramp`.
* **Gazebo con GUI** (`headless:=false`), RViz y `hw:=mock` (probado solo con `atp_position`,
  que es lo que dice "solo depuración") no se verificaron más allá de lo indicado. El tiempo real
  con prioridad RT no está disponible en el contenedor (`Could not enable FIFO RT scheduling`),
  así que no hay medida de jitter del lazo.
* **Regresión**: `bash scripts/validar.sh n2` y `test` se corrieron en el servicio `gz` con los 8 paquetes
  construidos: `n=1018 rms=0.0167 p95=0.0173 max=0.1382` y 98 pruebas de pytest pasan, 1 se salta (la de
  siempre). El resto de la escalera (`validar.sh` completo, E2E del frontend) no se corrió.
* **Cambios a archivos compartidos**: solo `docker-compose.yml` (servicio `gz` nuevo). No se tocó
  `docker/Dockerfile`, `src/atp_ros`, README ni PLAN.

