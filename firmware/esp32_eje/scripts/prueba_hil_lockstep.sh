#!/usr/bin/env bash
# Banco HIL en LOCKSTEP sin placa: el nodo micro-ROS del firmware compilado para Linux
# (eje_host --ejes 2: acimut y elevacion) en lugar de la montura simulada, con el pase
# N2 de la Estacion. Mide la latencia de ida y vuelta del enlace, el jitter del lazo y
# la deriva contra el gemelo digital (atp_mcu/gemelo.py).
#
#   bash firmware/esp32_eje/scripts/prueba_hil_lockstep.sh [el_max] [mask]   (defecto 85 70)
#
# Dominio ROS 73, UDP 8888 (agente oficial microros/micro-ros-agent:jazzy fijado por digest).
# Requiere haber compilado antes libmicroros para Linux:
#   docker compose run --rm -T ros bash firmware/esp32_eje/host/build_libmicroros.sh
# Los resultados quedan en firmware/esp32_eje/.cache/hil/ (hil.json, hil.csv, eje_host.json, logs).
# Cuando llegue la placa: mismo lanzador, sin eje_host y con el agente en el puerto serie
# (agente.sh serie): ver README, seccion 7c.
set -uo pipefail
cd "$(dirname "$0")/../../.."
DOMINIO="${ROS_DOMAIN_ID:-73}"
AGENTE="microros/micro-ros-agent:jazzy@sha256:76b7a4382ea1d34f65027449c518335dea5641154bcaa62bfe84b1691eb33c53"
NOMBRE=atp-agente-hil-lockstep
EL_MAX="${1:-85}"
MASK="${2:-70}"
mkdir -p firmware/esp32_eje/.cache/hil
docker rm -f "$NOMBRE" > /dev/null 2>&1 || true
docker run -d --rm --net=host --name "$NOMBRE" -e ROS_DOMAIN_ID="$DOMINIO" "$AGENTE" udp4 --port 8888 > /dev/null || exit 1
trap 'docker rm -f "$NOMBRE" > /dev/null 2>&1' EXIT
docker compose run --rm -T -e ROS_DOMAIN_ID="$DOMINIO" -e EL_MAX="$EL_MAX" -e MASK="$MASK" ros \
    bash /ws/firmware/esp32_eje/scripts/_dentro_ros_hil_lockstep.sh
