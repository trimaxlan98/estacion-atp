# atp_hw — hardware en el lazo (Sprint 5)

Puente entre el pase de `atp_ros` y un **rotor real** (o simulado) a través del
demonio `rotctld` de Hamlib. El lazo de posición lo cierra el controlador del
rotor; la Estación solo manda posiciones y lee la posición real, de modo que
el mismo `error_monitor` y `link_budget` miden el rotor verdadero.

```
 reference_generator ──/atp/reference (rad, az continuo)──▶ rotctld_bridge ──TCP "P az el"──▶ rotctld ──▶ rotor
        ▲                                                         │  ▲                         (G-5500, Rot2Prog...)
        └────────────/atp/torque (señal "listo")──────────────────┘  └──TCP "p"────────────────────┘
 error_monitor ◀── /joint_states (rad, posición REAL leída) ◀────────┘
 link_budget   ◀── /atp/error_deg ◀── error_monitor
```

Sin `mount_sim` ni `pid_controller`: en HIL no hay planta simulada.

## Contenido

| Pieza | Qué es |
|---|---|
| `rotctld_bridge` | Nodo ROS 2: `/atp/reference` → `P`, `p` → `/joint_states` |
| `fake_rotctld` | Rotor falso con protocolo `rotctld` y velocidad máxima por eje (6°/s az, 3°/s el) |
| `launch/hil.launch.py` | `sim_clock` + `reference_generator` + `link_budget` + `error_monitor` + puente (+ rotor falso o `rotctld -m 1`) |
| `atp_hw/protocol.py` | Formato/parseo del protocolo (lógica pura) |
| `atp_hw/rotor_logic.py` | Acimut continuo → rango del rotor, flip, límites, banda muerta, tasa (lógica pura) |
| `atp_hw/rotctl_client.py` | Cliente TCP bloqueante (timeouts, resincronización) |
| `atp_hw/bridge_core.py` | Hilo de E/S, backoff, estados; sin ROS (se prueba contra el rotor falso) |

## Uso

Todo en el contenedor (`ROS_DOMAIN_ID` propio por corrida):

```bash
docker compose run --rm -T -e ROS_DOMAIN_ID=53 ros bash -c \
  'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build && source install/setup.bash && \
   ros2 launch atp_hw hil.launch.py el_max:=85 mask:=70'
```

| Argumento | Defecto | Significado |
|---|---|---|
| `rotor` | `fake` | `fake` rotor falso de `atp_hw`; `dummy` lanza `rotctld -m 1`; `none` usa un `rotctld` que ya corre en `host:port` (hardware real) |
| `host`, `port` | `127.0.0.1`, `4690` | dónde escucha `rotctld` (el estándar de Hamlib es 4533) |
| `el_max`, `mask`, `band`, `eirp_dbw` | `85`, `70`, `S`, `9` | el pase y el enlace, igual que `atp_loop.launch.py` |
| `rot_az_min/max`, `rot_el_min/max` | `0..360`, `0..90` | rango del rotor (ver "Acimut y flip") |
| `rot_az_res`, `rot_el_res` | `0.1`, `0.1` | resolución del rotor = banda muerta (deg) |
| `cmd_rate_hz`, `read_rate_hz` | `5`, `10` | tasa máxima de `P` y tasa de lectura `p` |
| `az_prefer` | `low` | primera orden: `low`, `high` o `near` (ver abajo) |
| `preposition` | `true` | ir primero al punto de AOS y esperar al rotor |
| `fake_az_speed`, `fake_el_speed` | `6`, `3` | deg/s del rotor falso |
| `fake_az_res`, `fake_el_res` | `0`, `0` | cuantización de la lectura del falso (GS-232: `1.0`) |
| `fake_az0`, `fake_el0`, `fake_delay` | `0`, `0`, `0` | posición inicial y latencia por comando (s) |

**`time_scale` no existe como argumento y vale 1.** El rotor real vive en
tiempo real: no se puede acelerar el reloj y esperar que el motor lo siga. El
launch fija `sim_clock` a ×1 y el puente vigila además `/clock` contra el reloj
de pared; si difieren más de 10 % emite `EL RELOJ NO ES TIEMPO REAL` y
detiene el rotor (`require_realtime`). Un pase HIL dura lo mismo que el pase
real (más el viaje del rotor hasta AOS).

### Cómo arranca un pase

`reference_generator` congela la referencia en AOS hasta ver el primer
`/atp/torque` (en simulación lo publica el PID). Sin PID, el puente:

1. conecta con `rotctld` (reintenta con backoff 0.5 → 5 s si no está);
2. recibe la referencia de AOS y manda `P` (estado `PREPOSITION`);
3. cuando la lectura `p` está dentro de `ready_tol_deg` (0.5°, nunca menos que la
   resolución) durante `ready_hold_s` (1 s) pasa a `TRACKING`, publica
   `/atp/torque` y empieza a publicar `/joint_states`.

Hasta entonces **no** publica `/joint_states` (parámetro
`hold_joint_states_until_ready`) para que `error_monitor` no cuente el viaje
desde la posición de reposo. Si el rotor no llega en `preposition_timeout_s`
(180 s) sigue de todos modos y lo grita en el log. Con una referencia viva (un
satélite real en vez del generador) usa `preposition:=false`.

## Tópicos del puente

| Tópico | Dir. | Tipo | Nota |
|---|---|---|---|
| `/atp/reference` | in | `JointState` rad | az continuo, el |
| `/atp/rotor/halt` | in | `Bool` | `true` = parada de emergencia (envía `S` y no manda más `P`) |
| `/joint_states` | out | `JointState` rad | posición leída, az en el marco continuo de la referencia |
| `/atp/rotor/raw` | out | `JointState` rad | posición leída tal cual (rango del rotor), siempre |
| `/atp/torque` | out | `Float64MultiArray` | `[0, 0]` a 1 Hz desde `TRACKING`; es solo la señal de "lazo cerrado" |

Parámetros del nodo (`ros2 param`): `host`, `port`, `timeout_s`, `az_min`,
`az_max`, `el_min`, `el_max`, `az_res_deg`, `el_res_deg`, `cmd_rate_hz`,
`read_rate_hz`, `backoff_min_s`, `backoff_max_s`, `ref_timeout_s`, `preposition`,
`ready_tol_deg`, `ready_hold_s`, `preposition_timeout_s`, `az_prefer`,
`allow_flip`, `query_limits`, `stop_on_exit`, `require_realtime`.

Al terminar imprime `PUENTE ...` (órdenes, banda muerta, rechazos, flips,
reconexiones) y `ROTOR ...` (error máximo y tiempo > 1° según las lecturas del
rotor).

## Qué hace el puente con la referencia

- **Acimut y flip.** La referencia es continua (350, 359, 361...); el rotor solo
  acepta `[az_min, az_max]`.
  - Rango de **360°** (0..360): cada acimut tiene un solo valor. Al cruzar el
    norte la orden salta 360° y el rotor desenrosca entero: es el *flip*, y con
    360° exactos es inevitable.
  - Rango con **solape** (0..450 del G-5500, −180..540, 0..720): el mismo
    acimut tiene dos valores (a y a+360) y se elige el **más cercano a la última
    orden**, así un pase que cruza el norte no gira atrás. El flip solo ocurre si
    la referencia sale por el extremo del rango.
  - Primera orden (rotor en reposo): `az_prefer=low` toma el menor valor válido
    (deja el solape por delante para pases de acimut creciente), `high` el
    mayor, `near` el más cercano a la posición leída. Ojo: la dirección del pase
    no se conoce al preposicionar, porque la referencia está congelada.
  - Si hay flip se cuenta (`flips`), se avisa en el log y, con
    `allow_flip:=false`, se bloquea (el rotor se queda quieto).
  - Rango < 360°: lo que cae en la zona sin cobertura va al límite más cercano
    (`az_recortado`).
- **Límites.** Se intersecan con los que declara el rotor (`\dump_state`, si lo
  soporta) y la elevación se recorta a `[el_min, el_max]` (`el_recortada`).
- **Banda muerta.** No se manda `P` si el cambio respecto a la **última orden
  aceptada** es menor que la resolución en ambos ejes (no contra la lectura: un
  rotor lento iría siempre rezagado y se mandaría sin parar).
- **Tasa máxima.** `cmd_rate_hz` (5 Hz); las referencias intermedias se
  descartan, solo se manda la última.
- **Errores.** `RPRT -n` se cuenta (`rechazadas`) y no tira la conexión; un
  timeout o cierre del socket cierra y **reconecta** (el protocolo no tiene
  secuencia: tras un timeout hay que resincronizar). Tras reconectar se vuelve
  a mandar la posición; si estaba en `TRACKING` sigue ahí.
- **Sin referencia** durante `ref_timeout_s` (2 s; p. ej. fin del pase) → `S`.
- **Al salir** (Ctrl-C, fin del launch) → `S` y se cierra el socket.
- **Medida en el marco continuo.** `/joint_states` reubica el acimut leído ±360°
  junto a la referencia, de modo que `ref − meas` es el error físico.

## Conectar un rotor real

1. Conecta el controlador al PC (USB-serie) y comprueba el puerto:
   `ls /dev/ttyUSB*`. Dentro del contenedor hay que pasar el dispositivo
   (`docker compose run --device /dev/ttyUSB0 ...`) y el usuario `dev` necesita
   permiso (`dialout`).
2. Arranca `rotctld` con el modelo de tu controlador (números de Hamlib 4.5.5 de
   `rotctld -l`):

   | Hardware | Modelo | Ejemplo |
   |---|---|---|
   | Yaesu G-5500 + interfaz GS-232B | `603` | `rotctld -m 603 -r /dev/ttyUSB0 -s 9600 -T 127.0.0.1 -t 4533` |
   | Yaesu G-5500 + GS-232A | `601` | `rotctld -m 601 -r /dev/ttyUSB0 -s 9600 -t 4533` |
   | SPID Rot2Prog / MD-01 | `901` / `903` | `rotctld -m 901 -r /dev/ttyUSB0 -s 600 -t 4533` |
   | Easycomm II (SatNOGS, etc.) | `202` | `rotctld -m 202 -r /dev/ttyUSB0 -s 19200 -t 4533` |
   | Rotor dummy (pruebas) | `1` | `rotctld -m 1 -t 4533` |

   Las velocidades serie de la tabla son las habituales y **no están
   verificadas aquí**; mira el manual de tu interfaz. `-T 127.0.0.1` evita
   exponer el rotor a toda la red (`rotctld` no tiene autenticación).
3. Comprueba a mano antes de conectar ROS:
   `echo "p" | nc -q1 127.0.0.1 4533` debe devolver dos líneas (az, el).
   Mueve con `echo "P 90 10" | nc -q1 127.0.0.1 4533` → `RPRT 0`.
4. Lanza: `ros2 launch atp_hw hil.launch.py rotor:=none port:=4533
   rot_az_max:=450 rot_el_max:=180 rot_az_res:=1.0 rot_el_res:=1.0`.
   Ajusta rango y resolución al controlador: un GS-232 reporta enteros, así que
   una banda muerta de 0.1° solo manda órdenes que el rotor no distingue.
5. Primera vez: `el_max:=30 mask:=20` (un pase lento, lejos del keyhole), con el
   rotor a la vista y la mano en el interruptor.

### Seguridad

- **Parada de emergencia por software:** `ros2 topic pub --once
  /atp/rotor/halt std_msgs/msg/Bool "{data: true}"` envía `S` y deja de mandar
  posiciones hasta `false`. Es el *software*: no sustituye el paro físico.
  **Instala siempre un paro físico / fin de carrera** (la secuencia de `S` pasa
  por ROS, TCP, `rotctld` y el serie; un nodo colgado no detiene nada).
- Límites: `rot_az_*` y `rot_el_*` son la envolvente del puente; el rotor y
  `rotctld` aplican los suyos. No los pongas más anchos que el cableado físico
  (el G-5500 tiene tope en 450° de acimut; un rango de 720° enrosca el cable).
- El puente envía `S` al salir, ante un timeout de referencia y ante `halt`; **no**
  lo envía si el proceso muere con `SIGKILL` o se corta la corriente.
- Un pase HIL puede incluir un *flip* de acimut de hasta 360° (ver arriba):
  revisa el cableado y el rango antes de lanzar pases que crucen el norte.
- `rotctld` sin `-T 127.0.0.1` escucha en todas las interfaces y sin
  autenticación; cualquiera en la red puede mover la antena.
- `require_realtime` detiene el rotor si el reloj no es tiempo real.

## Pruebas

```bash
# lógica pura + puente contra el rotor falso, sin ROS (39 pruebas, ~10 s)
python3 -m pytest -q src/atp_hw/test
# integración: pase N2 completo contra el rotor falso (necesita ROS, ~2.5 min)
ATP_HW_INTEGRATION=1 python3 -m pytest -q -s src/atp_hw/test/test_integration_hil.py
```

## Resultados medidos (rotor falso, tiempo real)

Todas las cifras salen de `ros2 launch atp_hw hil.launch.py ...` con la
salida real de `error_monitor` (`RESULTADO`), `link_budget` (`ENLACE`) y del
puente. Rotor falso 6°/s (az) y 3°/s (el), sin cuantización, salvo donde se
indica. Puente por defecto: `P` a 5 Hz, banda muerta 0.1°, lectura a 10 Hz.

**Pase N2** (`el_max:=85 mask:=70`, 50.9 s, demanda de acimut 9.04°/s en la
culminación). Análisis: la demanda supera 6°/s durante 9.0 s (t = 21.0..30.0 s) y
el keyhole analítico a 6°/s es `el_max` = 82.5°. Un rotor ideal, solo con tope de
velocidad (cálculo con `atp_ros.model.perfil_pase`), acumula 16.8° de error de
acimut, 1.90° de apuntamiento máximo y 9.0 s por encima de 1°.

| Medido (banda Ka, 0.2° de haz) | Valor |
|---|---|
| Error de acimut, máx | 18.8° |
| Apuntamiento rms / p95 / máx | 0.79° / 2.01° / 2.15° |
| Dentro de 0.1° | 20.1 % |
| Margen de enlace mín | −34.6 dB |
| Enlace cerrado | 81.6 % del pase (se pierde ~9 s) |
| Tiempo con error > 1° (lecturas del rotor) | ~9.9 s, la primera vez a los 27.5 s de `TRACKING` |
| Órdenes `P` / rechazadas / flips | 251 / 0 / 0 |

En banda S (haz de 3.18°) el mismo pase **no** pierde el enlace: apuntamiento
máx 2.1°, margen mín 17.3 dB, cerrado 100 % (`L_point` máx 5.4 dB). El
error de acimut de 18° se proyecta al cielo como `18° · cos(85°)`; solo la
banda Ka (y la X, según el margen) se queda sin enlace. Es decir: el keyhole
**sí** hace perder el satélite en términos de acimut (el rotor queda ~19° atrás
durante ~9 s), pero cuánto cuesta en el enlace depende del haz.

**Línea base del puente y de la resolución** (mismas condiciones; la
carga del host era alta, ver más abajo, así que ±varios puntos de variación):

| Corrida | Dentro de 0.1° | Apuntamiento p95 / máx | Enlace |
|---|---|---|---|
| Control `el_max:=60 mask:=50`, banda S, puente por defecto (5 Hz, 0.1°) | 34–47 % | 0.17–0.21° / 0.25–0.46° | S: cerrado 100 %, margen mín 20.7 dB |
| Control `el_max:=60`, banda Ka, `cmd_rate_hz:=20 rot_*_res:=0.02 read_rate_hz:=20` | 96.4 % | 0.096° / 0.52° | Ka: cerrado 100 %, margen mín +2.4 dB |
| Control `el_max:=60`, Ka, rotor tipo GS-232 (lectura entera: `fake_*_res:=1 rot_*_res:=1`) | 4.0 % | 0.89° / 1.20° | Ka: cerrado 36.6 % |
| N2, Ka, rotor falso rápido 20/10°/s, 20 Hz, 0.02° | 92–95 % | 0.11° / 0.30–0.40° | Ka: cerrado 99.8–100 %, margen mín −13..+2.9 dB |

Lecturas: (1) un pase **por debajo** del keyhole se sigue bien con un rotor de
6°/s, y lo que manda es la tasa de órdenes y la resolución, no la velocidad:
con 5 Hz/0.1° el seguimiento queda en ~0.11–0.15° rms (escalonado) y con 20 Hz/0.02° en
~0.05° rms; (2) la resolución del controlador manda: con 1° de resolución
el objetivo de 0.1° es inalcanzable; (3) con un rotor 3× más rápido el N2 se sigue
(el error residual es latencia + escalones, no saturación).

**`rotctld -m 1` (rotor dummy) NO sirve para medir dinámica.** Hamlib 4.5.5:
se mueve a 6°/s en *ambos* ejes si se le manda un solo `P`, pero cada `P` nuevo
reinicia su movimiento: reenviando `P 300 80` cada 0.5 s avanzó 0.6°/s, cada
0.2 s 0.24°/s y cada 0.1 s 0.13°/s (medido). Sirve para probar el protocolo y
la conexión (`rotor:=dummy`), no la dinámica; por eso existe `fake_rotctld`. Las
corridas con dummy dieron cifras peores y distintas (apuntamiento máx 4.9°, cerrado
77 %) y no se usan arriba.

**Advertencia de medición.** Durante estas corridas el host tenía una carga
de 35–50 (8 núcleos) por trabajos de otros equipos. Los nodos de `atp_ros` usan
temporizadores en tiempo simulado: bajo carga `error_monitor` pierde ticks (`n` osciló
entre 600 y 1700 en pases de duración fija), así que los porcentajes tienen
ruido. Las conclusiones cualitativas (keyhole, efecto de tasa/resolución) se
repitieron en varias corridas; las cifras exactas no. Con el host libre deben
repetirse.


## micro-ROS en un ESP32 (plan; primer firmware en `firmware/esp32_eje`)

Objetivo: sustituir `rotctld` + controlador comercial por un ESP32 que cierre
el lazo de posición con encoders y motores propios, y hable ROS 2 directamente.

**Actualización (equipo J):** el firmware de un eje (lazo de 1 kHz, encoder PCNT,
PWM LEDC, watchdog, paro, micro-ROS por serie/UDP) existe y compila en Docker:
[`firmware/esp32_eje/README.md`](../../firmware/esp32_eje/README.md); el paquete
`atp_mcu` une los ejes en `/joint_states`. **Nunca se cargó en una placa.** Lo
que sigue es el plan original, tal como lo dejó este equipo.

1. **Firmware** (ESP-IDF o PlatformIO + `micro_ros_platformio`) con
   `rclc`: un `rclc_executor` con dos *timers* (control a 100–200 Hz; telemetría a
   20–50 Hz) y transporte serie (USB-CDC) o UDP (WiFi) hacia un
   `micro_ros_agent` en el PC/contenedor. Sin agente no hay tópicos: el agente
   es un proceso más del launch.
2. **Interfaz ROS idéntica a la del puente** para no tocar `error_monitor`:
   suscribe `/atp/reference` (JointState) y publica `/joint_states` (posición
   leída del encoder, velocidad, `effort` = mando). Con eso `hil.launch.py` solo
   cambia `rotctld_bridge` por el agente.
3. **Lazo en el ESP32:** PID (o el compensador del curso 31) en coordenadas del
   eje, con anti-windup y saturación por eje; lectura de encoder absoluto
   (SSI/SPI, AS5600/AS5048) o incremental con índice; PWM/driver por eje
   (o paso/dirección para steppers).
4. **Seguridad en el firmware, no en ROS:** *watchdog* de referencia (si no
   llega en 500 ms, frena y mantiene), fines de carrera cableados al
   hardware, límites de velocidad y de aceleración, y rango de acimut con la
   misma lógica de flip que `rotor_logic.py` (portar a C o mandar ya el acimut
   dentro del rango del rotor).
5. **Tiempo:** ni micro-ROS ni el reloj del ESP32 son del dominio simulado;
   sincronizar con `rmw_uros_sync_session` o ignorar el sello y usar el del PC
   al recibir. Mantener `time_scale=1`.
6. **Qué hay que decidir/medir antes:** latencia real del enlace
   (serie ≈ 1–3 ms; WiFi: jitter de decenas de ms, vigilar), memoria
   disponible (el tipo `JointState` con `name` de longitud variable necesita
   buffers estáticos configurados en `colcon.meta`), y QoS *best effort* para la
   telemetría.
7. **Pruebas sin motor:** *hardware-in-the-loop* del propio ESP32 con una
   planta simulada en firmware (el `mount_sim` en C) antes de conectar
   potencia; después, el rotor real.

## Limitaciones conocidas

- Solo protocolo simple de `rotctld` (sin `+` extendido); no lee estado de
  movimiento (`s`/`get_status`) ni usa `M` (velocidad) ni `K` (aparcar).
- No hay compensación de latencia (adelanto): la orden se manda con la posición
  del satélite *ahora*, así que un rotor lento siempre llega tarde. El
  adelanto (`P` a la posición en *t + latencia*) es la mejora obvia.
- Si se pierde la conexión mitad del pase, `/joint_states` deja de publicarse y
  `error_monitor` conserva la última lectura (no sabe de datos viejos).
- La elevación > 90° (voltear el rotor por encima del cenit: az+180°, el=180°−el
  en G-5500, la técnica clásica para esquivar el keyhole) no está implementada.
- Al terminar el pase, `reference_generator` y `link_budget` de `atp_ros` a veces
  mueren con un `RuntimeError: Unable to convert call argument` al recibir el
  SIGINT del launch (carrera de `rclpy.spin` en el apagado, ya presente en
  `atp_ros`; no se tocó). El resultado ya se imprimió antes; el puente lo
  tolera. Bajo estas condiciones la salida estándar de un nodo se pierde tras
  el SIGINT en `ros2 launch`: el puente imprime su resumen por `stderr`.
- Los 6°/s y 3°/s del rotor falso son un supuesto cercano al G-5500, no un
  dato medido.
