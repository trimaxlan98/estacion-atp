#!/usr/bin/env bash
# micro-ROS sobre Linux de punta a punta: agente oficial (Docker) + nodo rclc
# del eje (el MISMO codigo del firmware, con la planta J/b) + nodos ROS 2 normales.
#
#   bash firmware/esp32_eje/scripts/prueba_micro_ros_host.sh
#
# Requiere: firmware/esp32_eje/.cache/host_uros (host/build_libmicroros.sh) y la
# imagen microros/micro-ros-agent:jazzy. Usa por defecto ROS_DOMAIN_ID=81 y el puerto
# UDP 8888 del host (ROS_DOMAIN_ID=<n> en el entorno lo cambia). EJES=2 prueba el firmware de dos ejes
# (eje_host --ejes 2: az y el, un verificador por eje). No deja procesos ni contenedores corriendo.
set -uo pipefail
cd "$(dirname "$0")/../../.."
DOMINIO="${ROS_DOMAIN_ID:-81}"
AGENTE="microros/micro-ros-agent:jazzy@sha256:76b7a4382ea1d34f65027449c518335dea5641154bcaa62bfe84b1691eb33c53"
NOMBRE=atp-agente-prueba

docker rm -f "$NOMBRE" > /dev/null 2>&1 || true
docker run -d --rm --net=host --name "$NOMBRE" -e ROS_DOMAIN_ID=$DOMINIO \
    "$AGENTE" udp4 --port 8888 -v4 > /dev/null || exit 1
trap 'docker rm -f "$NOMBRE" > /dev/null 2>&1' EXIT
echo "agente micro-ROS (UDP 8888, dominio $DOMINIO) en marcha"

docker compose run --rm -T -e ROS_DOMAIN_ID=$DOMINIO -e EJES="${EJES:-1}" ros bash /ws/firmware/esp32_eje/scripts/_dentro_ros.sh
estado=$?

echo "--- ultimas lineas del agente"
docker logs "$NOMBRE" 2>&1 | tail -8
exit $estado
