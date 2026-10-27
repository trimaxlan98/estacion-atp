# Estación ATP (versión educativa)

Plataforma de simulación aeroespacial sobre **ROS 2 Jazzy**: una estación terrena que **adquiere, sigue y apunta**
(Acquisition, Tracking, Pointing) a satélites LEO, simulada de punta a punta y preparada para cargarse en hardware.
Cada cifra se calcula y se contrasta con una fuente (una fórmula cerrada, una norma, una efeméride publicada o una segunda
implementación); cada corrida se guarda y se puede repetir con los mismos bytes.

Está pensada para que **estudiantes la clonen, la usen, aprendan y la mejoren**. Es un proyecto de
[Co.De Aerospace](mailto:contacto@codeaerospace.com) bajo licencia Apache-2.0.

> Esta es la versión pública y educativa: incluye la plataforma base y los estudios de apoyo (rover, dron, enlace óptico,
> misión, ADCS, firmware, app web). No incluye trabajos de investigación de tesis ni sus datos.

## Qué hay

| Pieza | Dónde | Qué hace |
|---|---|---|
| Física y nodos | `src/atp_ros` | pase LEO (esférico y SGP4 real), montura de 2 ejes, PID, Doppler, presupuesto de enlace, Monte Carlo (acción ROS), autotrack por *conical scan*, estrategias de keyhole, reloj en lockstep |
| Interfaces | `src/atp_interfaces` | `SatState`, `LinkStatus`, acción `RunCampaign` |
| Hardware | `src/atp_hw` | puente Hamlib `rotctld` para rotores comerciales y rotor falso con dinámica |
| ros2_control | `src/atp_description`, `src/atp_control` | montura como `hardware_interface` (backends `sim`, `serial`, Gazebo) y controlador de seguimiento PID/LQR/Kalman en C++: [ROS2_CONTROL](docs/ROS2_CONTROL.md) |
| Control | `src/atp_ros` (`lqr_controller`, `planta`, `control`) | planta realista (holgura, fricción, encoder cuantizado, viento), LQR integral y Kalman contra el PID: [CONTROL](docs/CONTROL.md) |
| Firmware | `firmware/esp32_eje`, `src/atp_mcu` | ESP32 + micro-ROS que gobierna un eje de la montura (lazo de 1 kHz, encoder, PWM, paro, watchdog); compila en Docker y su lógica se prueba contra la planta de `atp_ros`. Sin probar en placa |
| Estación (app) | `station/` | backend FastAPI (cola, SQLite, SSE) + frontend React: escenas, lanzar, ver en vivo (con montura 3D), biblioteca, comparar, campañas, plan de contactos (Gantt), ADCS, rover 3D, renders y medios |
| Renders | `render/*.yaml`, `scripts/render/*.sh` | de la corrida al medio, sin humano: Gazebo con la antena, video de Manim con cifras medidas y gráficas con la marca; dejan `media.json` con la procedencia: [RENDER](docs/RENDER.md), [APP](docs/APP.md) |
| Planificador | `src/atp_ros/atp_ros/planificador.py`, `scripts/planificar.py` | contactos N satélites × M estaciones con volumen de datos (ACM DVB-S2), codicioso y óptimo: [PLANIFICADOR](docs/PLANIFICADOR.md) |
| ISL óptico | `src/sat_isl` | ATP láser entre satélites: constelación Walker, visibilidad con refracción, haz truncado, adquisición en espiral, lazo fino FSM + montura gruesa, presupuesto óptico |
| Óptico tierra–espacio | `src/atp_optico`, `scripts/optico_estudio.py`, `data/clima_optico.yaml` | enlace óptico con atmósfera: perfil Hufnagel–Valley, centelleo, tilt y *beam wander*, OOK + RS y PPM, FSM de 10 kHz, nubes y diversidad de estaciones: [OPTICO](docs/OPTICO.md) |
| RF | `src/atp_sdr` | cadena IQ: beacon con Doppler real, FFT + FLL + PLL de 3.er orden, Costas, Gardner, C/N0; validada contra teoría |
| Órbita | `propagador.py`, `medidas.py`, `od.py`, `od_estimator.py` | propagador numérico (J2-J4, arrastre, RK4/DP45, STM), medidas az/el/rango/Doppler, determinación de órbita por lotes y EKF: [OD](docs/OD.md) |
| Misión | `src/atp_mision`, `scripts/mision.py` | un día de operaciones de extremo a extremo: planifica, corre cada contacto en lockstep con SGP4 real, determina la órbita y cuenta los GB: [MISION](docs/MISION.md) |
| ADCS | `src/sat_adcs` | actitud de un CubeSat 3U en el mismo lockstep |
| Rover | `src/atp_rover`, `scripts/rover*.py` | rover planetario de seis ruedas (terramecánica Bekker–Wong en cinco cuerpos), navegación, enlace con relés y visor 3D: [ROVER](docs/ROVER.md) |
| Dron | `src/atp_dron`, `scripts/dron*.py` | cuadrirrotor 6-DOF reproducible (Tierra, Marte, Titán, Luna) con viento, control en cascada, ESKF y nodos ROS 2 en lockstep: [DRON](docs/DRON.md) |
| Catálogo | `experiments/` | experimentos versionados con su hipótesis y lo que se espera |
| Datos | `data/` | TLE reales de CelesTrak y estaciones terrenas |
| Documentos | `docs/` | [APP](docs/APP.md), [CONTRATO](docs/CONTRATO.md) (formatos, API, lockstep), [CALIDAD](docs/CALIDAD.md), [HARDWARE](docs/HARDWARE.md), además de los citados arriba |

## Escalera de validación

| Nivel | Experimento | Resultado de referencia |
|---|---|---|
| N0 | pub/sub `hello_ros` | DDS y colcon funcionando |
| N1 | lazo de un eje sobre rampa | arrastre del PID |
| N2 | pase cenital con keyhole (9 °/s en acimut) | 0.017° rms de apuntamiento |
| N3 | gemelo URDF + TF2 | boresight en TF |
| N4 | Doppler, enlace, Monte Carlo | ±47 kHz en S; p95 0.042° (lazo, n=300) |
| S4 | pase real de la ISS sobre CDMX (SGP4) | 0.001° rms; SGP4 contra Vallado: 1.6e-7 km |
| N5 | autotrack en Ka (haz 0.233°) | program track 0.16° → autotrack 0.004° rms |
| C1 | LQR integral + pre-alimentación en el pase N2 (`controller:=lqr`) | 0.007° rms (PID 0.017°); con Kalman y planta realista, ver [CONTROL](docs/CONTROL.md) |

`scripts/validar.sh` recorre la escalera dentro del contenedor e incluye una prueba de **determinismo** (la misma corrida
dos veces da cifras idénticas).

## Uso

```bash
git clone <url-de-este-repositorio> && cd estacion-atp
docker compose build                        # imagen ros2-jazzy-dev (una vez)
./run.sh bash scripts/validar.sh            # validar la escalera
./run.sh ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0
./run.sh ros2 launch atp_ros atp_autotrack.launch.py autotrack:=true
./station.sh                                # la Estación en http://127.0.0.1:8090
```

Para el frontend: `cd station/frontend && npm install && npm run build` (el backend sirve `dist/`). Sin ROS también se
pueden correr las pruebas de Python de cada paquete (`python3 -m pytest -q src/<paquete>/test`) y las del backend
(`cd station/backend && python3 -m pytest -q`). Hardware: ver [`src/atp_hw/README.md`](src/atp_hw/README.md).

## Qué no está probado

- Ninguna entrada está validada en hardware: el firmware nunca se cargó en una placa y los criterios de banco son propuestas.
- `scripts/validar.sh dron` y los nodos del dron con ROS 2 real, y el modelo `src/atp_dron/models/cuadrirrotor/model.sdf` en `gz-sim`.
- Los modelos de rover, dron y enlace óptico usan parámetros supuestos (marcados `SUPUESTO` en el código y la documentación).

> Los resultados de los estudios (`docs/*_resultados/`) no se incluyen: los scripts de `scripts/estudio_*.py` los regeneran.

## Para aprender y mejorar

- Empieza por la escalera N0–N5 y el [CONTRATO](docs/CONTRATO.md): ahí está lo que cada pieza escribe y lee.
- Cada módulo trae sus pruebas; una buena primera contribución es cerrar un «SUPUESTO» contrastándolo con una fuente.
- Ideas abiertas: un procedimiento de calibración de la montura, un E2E de navegador propio para la app, probar el firmware en placa,
  contrastar el modelo del rover con datos reales.
- Abre un *issue* o un *pull request*; mantén el determinismo (misma semilla, mismos bytes) y declara los supuestos.

## Licencia

Apache-2.0 (`LICENSE`), Copyright 2026 Co.De Aerospace. Alcance y terceros en `NOTICE`.
