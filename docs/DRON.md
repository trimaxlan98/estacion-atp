# Dron multirrotor

**Objetivo**: base de simulación de drones en la Estación: un cuadrirrotor con dinámica 6-DOF, atmósfera por cuerpo, viento y
control en cascada, reproducible (misma semilla, mismos bytes). Es el núcleo sobre el que crecen la ruta/misión y Gazebo.

## Paquete `src/atp_dron` (núcleo sin ROS)
- `cuerpos.py`: g y densidad de Tierra, Marte, Titán y Luna (fuentes en el docstring).
- `dron.py`: cuadrirrotor en X, RK4 a dt = 4 ms, motores de primer orden, arrastre cuadrático, ráfagas Ornstein-Uhlenbeck,
  contacto con el suelo, energía por teoría de cantidad de movimiento (`T^1.5 / (sqrt(2 rho A) · FoM · eta)` + aviónica).
- `control.py`: posición → velocidad (PI) → atitud geométrica (Lee 2010) → tasa → mezclador.
- `lazo.py` / `cli.py`: ruta de metas `[[x,y,z],...]`, despegue vertical, `telemetry.jsonl` + `metrics.json`.

```
python3 scripts/dron.py --escena experiments/dron-ruta-viento.yaml --salida /tmp/dron --id d1
python3 -m pytest -q src/atp_dron/test      # ~2.7 min
python3 scripts/estudio_dron.py [--rapido]  # banco de experimentos de D2 -> docs/dron_resultados/
```

Escenas de la app (`experiments/dron-*.yaml`, `kind: proceso`): `dron-ruta-viento`, `dron-hover-marte`, `dron-titan-vuelo`,
`dron-carga-bateria`, `dron-viento-limite`. **Corrección de D2**: la escena de D1 no cargaba en la app (el catálogo exige
`params_schema` con valor por defecto en `args`); ahora las cinco cargan. Contratos `dron_telemetria`/`dron_metricas`: [CONTRATO §21](CONTRATO.md).

## Resultados (en simulación, con estos supuestos)
- Ruta de tres metas a 10 m con viento 2 m/s + ráfagas 1 m/s (`dron-ruta-viento`, semilla 5): llega en 23.2 s, 107 m,
  error transversal rms 0.86 m, inclinación máx. 38.9°, 1.23 Wh, sin choque.
- Mantener posición con viento constante de 4 m/s: deriva < 0.2 m (el término integral lo absorbe).
- Hover terrestre ≈ 182 W (a mano, con FoM 0.6 y eficiencia 0.8: 4·T^1.5/(√(2ρA)·FoM·η) + 15 W; D1 decía 210 W, era un error de la cuenta). La corrida da el mismo orden.
- **Marte**: con rotores de tamaño terrestre el empuje/peso efectivo es 0.09 y no despega; con `rotor_escala: 25` sí (2.4).
  El 25 es un SUPUESTO sin fuente. **Titán**: empuje/peso 70 con los mismos rotores (aire denso, g baja).

## D2: física y banco de experimentos
Física añadida (opcional, apagada = bytes de D1): **efecto suelo** de Cheeseman–Bennett (`dron.efecto_suelo`) y **batería**
(`dron.bateria_wh`: la corrida termina al agotarla y reporta `soc_final`). `atp_dron/estudios.py` + `scripts/estudio_dron.py`
corren el banco con semillas; cifras completas en `docs/dron_resultados/estudio_dron.{md,json}` (en simulación, con estos supuestos):
- **Escalón de 5 m**: subida 10–90 % en 1.06 s, sobreimpulso 5.7 %, asentamiento al 2 % en 5.1 s (guiado + cascada, no la planta sola).
- **Viento sostenible**: sin deriva hasta 12 m/s (inclinación 18°); a 20 m/s se llega a 35.2° con 0.7 m de deriva; a 24 m/s la
  inclinación máxima (35°) ya no basta y deriva 17 m en 30 s. **Contraste con fórmula cerrada**: m·g·tan 35° = ½ρ·CdA·v² da 23.7 m/s.
- **Carga útil** (Tierra, batería de 60 Wh): de 0 a 1 kg de carga el empuje/peso baja de 2.2 a 1.32, el hover sube de 182 a 374 W y la
  autonomía en hover cae de 19.8 a 9.6 min. En Marte (rotores ×14, supuesto) el límite es ~0.5 kg: con 0.5 kg ya no despega.
- **Cuerpos**: rotor mínimo para empuje/peso 1.3 es ×0.59 en la Tierra, ×13.7 en Marte y ×0.02 en Titán; la Luna no vuela con rotores
  (sin atmósfera). Hover: 182 W (Tierra), 319 W (Marte ×14), 19 W (Titán).
- **Ráfagas** (σ = 1 m/s, viento 2 m/s, 30 semillas): 30/30 llegan, error transversal rms 0.848 m (desv. 0.011, p95 0.863).
- **Efecto suelo**: +11 % de empuje a z = 0.1 m (0.79 R), < 1 % a z ≥ 0.5 m.

## D3: sensores y estimador
`sensores.py` (IMU con ruido y paseo aleatorio de sesgo, GPS 5 Hz, barómetro, magnetómetro) y `estimador.py` (ESKF de 15 estados,
Solà 2017, forma de Joseph). Con `sensores: true` el control vuela sobre la estimación; la dinámica verdadera usa su propio
generador y no cambia. Resultados (en simulación, con sensores supuestos de bajo costo):
- Ruta de tres metas con viento y ráfagas (`dron-sensores-ruta`): error de posición 0.25 m rms (GPS crudo: 0.8 m horizontal, 1.5 m
  vertical), actitud 0.9°, sesgo de giroscopio estimado a 0.002 rad/s; el error transversal pasa de 0.86 a 0.93 m. Banco Monte Carlo en
  `docs/dron_resultados/`.
- **Pérdida de GPS** (`dron-perdida-gps`, corte a los 15 s): la posición verdadera deriva ~50 m en 45 s. Cuadra con la física: un
  sesgo de acelerómetro residual b integra a ½·b·t² (b = 0.05 m/s² da 50 m a 45 s); la altura se sostiene con el barómetro.
- Probado: el acelerómetro mide +g en reposo y en hover y 0 en caída libre; sensores apagados reproducen los bytes de D2.

## D5: nodos ROS 2 en lockstep
`lazo.correr`, los nodos y `lockstep.correr_cadena` son **la misma cadena** (`atp_dron/lockstep.py`): la **planta** (`PlantaLock`) es el
dron con su controlador de vuelo a bordo (sensores, ESKF, cascada, motores) y el **guiado** (`Guiado`) es la computadora de misión
(metas, despegue, velocidad deseada). Un tick dura 0.02 s (5 pasos de 4 ms, el lazo externo de la cascada); `sim_clock` avanza con
`step_s = 0.02` solo cuando la planta confirma en `/atp/step_ack` (CONTRATO §8).
```
/clock -> dron_dinamica publica /dron/estado (verdad + lo que ve a bordo) y /dron/pose (PoseStamped, 'world', ENU)
       -> dron_navegador responde UN /dron/setpoint {k, t, v, yaw, guia}
       -> dron_dinamica integra [t_k, t_k+1) con él (retención de orden cero) y confirma el /clock
```
```
ros2 launch atp_dron dron_lockstep.launch.py escena:=experiments/dron-lockstep-ruta.yaml time_scale:=0 salida:=/tmp/dron id:=a
python3 scripts/dron_lockstep.py --escena_yaml experiments/dron-lockstep-ruta.yaml    # ROS + sin ROS + equivalencia.json
./run.sh bash scripts/validar.sh dron_test     # 13 pruebas, sin ROS, ~10 s
./run.sh bash scripts/validar.sh dron          # CON ROS: time_scale 0 y 2, sin y con sensores, byte a byte
```
- **Contrato de `/dron/setpoint`** (cualquier nodo —teleoperación, Gazebo, otro planificador— puede sustituir a `dron_navegador`): CONTRATO §21.
  El guiado solo ve la observación a bordo (`obs_p`, `obs_v`: la estimación con sensores, la verdad sin ellos), nunca la verdad.
- **Cambio de semántica respecto a D4** (para que ROS y sin ROS coincidan): el guiado decide una vez por tick, no en cada paso de 4 ms.
  Efecto medido en cinco escenas: la llegada se redondea al tick (≤ 20 ms), el error transversal cambia en la 4.ª cifra (0.8587 → 0.8586 m
  en `dron-ruta-viento`) y la corrida dura exactamente `duracion_s` (antes perdía un paso de 4 ms por meta alcanzada).
- **Qué se probó y qué no.** *Probado aquí* (sin ROS): la cadena con y sin DDS simulado da los mismos bytes en telemetry, metrics y
  setpoints; los nodos REALES (`nodos_lockstep.py`, `nodo_dinamica.py`, `nodo_navegador.py`) y el `SimClock` REAL de `atp_ros` corren
  sobre un `rclpy` en memoria (`test/fake_rclpy.py`) con entrega aleatoria entre tópicos, emparejamiento tardío y datos tardíos, con
  y sin sensores; siete mutaciones del código (sello rancio, ack equivocado, arranque, estado inicial, deduplicación, lectura de la verdad,
  tick mal contado) hacen fallar las pruebas. *No probado*: ROS 2 / DDS reales (este entorno no los tiene), QoS reales, el ejecutor de
  rclpy, el launch ni `colcon build`; `validar.sh dron` es la prueba que falta y hay que correrla en el contenedor `ros2-jazzy-dev`.
  El `rclpy` falso no es DDS: prueba la lógica de sellos y fichas, no la red.

## Opciones de física y control (D6)
Las opciones del rotor (modo coherente / energía-par, Reynolds, potencia en avance), del guiado con autoridad limitada y del estimador
(ESKF consistente) se activan desde la escena (CONTRATO §21) y están **apagadas por defecto**: las escenas del dron dan los mismos bytes
que antes.

## Barrida de robustez del rotor y estado de Gazebo (2026-10-18)
- **Curva en U:** `scripts/estudio_rotor_robustez.py` barre k_mu ∈ {0, 2, 4.65, 6, 8} y el factor c sobre P0 ∈ {0.5, 1, 2}. El mínimo interior
  existe en las 15 combinaciones, a 7.5–8.8 m/s, y el control sin Glauert no tiene mínimo. **La ubicación del mínimo es robusta; la potencia no**
  (86–174 W, casi lineal en P0).
- **Gazebo:** **no se puede verificar aquí.** No hay `gz` instalado y el repositorio de OSRF da 403 por el proxy; Ubuntu 24.04 no trae
  gz-harmonic. Lo que sí se hizo: `src/atp_dron/models/cuadrirrotor/model.sdf` (SDF 1.9, masa, inercias y rotores de `dron.py`) y
  `scripts/comprobar_modelo_dron.py` que verifica que coincidan (prueba de regresión incluida; detecta desviaciones de masa y de posición de rotor).
  **Pendiente de verdad:** cargarlo en `gz-sim` y conectar `/dron/pose`; eso requiere el contenedor con Gazebo.

## Límites conocidos
- D6: la integración corrió sin ROS 2 real; el modelo de energía cuenta solo ~5 % del costo físico de subir y `k_M` implica FoM 0.505 en vez de 0.6.
- D5: un solo guiado y una sola planta; sin enlace Tierra-dron; sin Gazebo ni visor
  todavía (publica `/dron/pose` para ellos); `/dron/estado` lleva la verdad además de la observación, un nodo de misión real no debería leerla.

- Sensores: ruidos y sesgos supuestos; GPS sin retardo ni multitrayectoria; magnetómetro sin perturbaciones; el yaw inicial se da
  conocido y la actitud inicial es nivelada (el dron parte del suelo). Sin cámara ni flujo óptico que acoten la deriva sin GPS.
- **Titán** (`dron-titan-vuelo`): la inclinación medida llega a 51.6° con un límite de 35°. **Corrección (D6):** no es por ganancias sin adaptar al empuje/peso: el sobreimpulso de
  actitud es ~11 % en la Tierra y en Titán; el guiado frena con 2 m/s² y el cuerpo solo da g·tan 35° = 0.947 m/s². Con `autoridad_guiado` + `autoridad_acel: rampa` + `antiwindup` llega en
  18.7 s contra 28.7 s y el sobreimpulso a la meta baja de 6.29 a 0.08 m (10 semillas). Las banderas están apagadas por defecto.
- La inercia no se recalcula con la carga (solo la masa).
Sin efecto suelo, sin arrastre de rotor ni Reynolds bajo (sobrestima el empuje en Marte), inercias y CdA supuestos, suelo
plano inelástico, ganancias sin sintonía fina (el error transversal en esquinas viene de la velocidad de paso de 1.5 m/s).
Pendiente: probar el lockstep con ROS 2 real (`validar.sh dron`), modelo
Gazebo, vista en el visor.
