#!/usr/bin/env bash
# Eje micro-ROS (Linux) contra el generador y el monitor de atp_ros, en tiempo real (~1 min).
#   bash firmware/esp32_eje/scripts/prueba_hil_host.sh      (dominio 81, UDP 8888)
set -uo pipefail
cd "$(dirname "$0")/../../.."
DOMINIO=81
AGENTE="microros/micro-ros-agent:jazzy@sha256:76b7a4382ea1d34f65027449c518335dea5641154bcaa62bfe84b1691eb33c53"
NOMBRE=atp-agente-hil
docker rm -f "$NOMBRE" > /dev/null 2>&1 || true
docker run -d --rm --net=host --name "$NOMBRE" -e ROS_DOMAIN_ID=$DOMINIO "$AGENTE" udp4 --port 8888 > /dev/null || exit 1
trap 'docker rm -f "$NOMBRE" > /dev/null 2>&1' EXIT
docker compose run --rm -T -e ROS_DOMAIN_ID=$DOMINIO ros bash /ws/firmware/esp32_eje/scripts/_dentro_ros_hil.sh
