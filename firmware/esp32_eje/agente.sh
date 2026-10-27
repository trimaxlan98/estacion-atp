#!/usr/bin/env bash
# Agente micro-ROS (imagen oficial microros/micro-ros-agent:jazzy, fijada por
# digest) para el ESP32. Une el XRCE-DDS del microcontrolador con el DDS de la
# Estacion: los topicos del ESP32 aparecen como topicos ROS 2 normales.
#
#   ./agente.sh serie [DISPOSITIVO] [BAUDIOS]   defecto: /dev/ttyUSB0 921600
#   ./agente.sh udp   [PUERTO]                  defecto: 8888
#   ./agente.sh ambos [DISPOSITIVO] [BAUDIOS]   los dos, en segundo plano
#   ./agente.sh parar                           detiene los agentes de este script
#
# Variables: ROS_DOMAIN_ID (defecto 42, el de docker-compose.yml; DEBE coincidir
# con CONFIG_ATP_ROS_DOMAIN_ID del firmware y con la Estacion), VERBOSO (defecto 4).
# Red del host (--net=host): el DDS de la imagen descubre a los nodos del PC y
# el UDP del ESP32 llega al puerto sin NAT. En Wi-Fi, abrir UDP/8888 en el cortafuegos.
set -euo pipefail

IMG="microros/micro-ros-agent:jazzy@sha256:76b7a4382ea1d34f65027449c518335dea5641154bcaa62bfe84b1691eb33c53"
DOMINIO="${ROS_DOMAIN_ID:-42}"
VERBOSO="${VERBOSO:-4}"
CMD="${1:-}"

case "$CMD" in
    serie)
        dev="${2:-/dev/ttyUSB0}"; baud="${3:-921600}"
        [ -e "$dev" ] || { echo "no existe $dev (conectado? permisos? grupo dialout)"; exit 1; }
        echo "agente SERIE en $dev a $baud baudios, dominio $DOMINIO"
        exec docker run --rm --net=host --name atp-agente-serie -e ROS_DOMAIN_ID="$DOMINIO" \
            --device "$dev" "$IMG" serial --dev "$dev" -b "$baud" -v"$VERBOSO"
        ;;
    udp)
        puerto="${2:-8888}"
        echo "agente UDP en el puerto $puerto, dominio $DOMINIO"
        exec docker run --rm --net=host --name atp-agente-udp -e ROS_DOMAIN_ID="$DOMINIO" \
            "$IMG" udp4 --port "$puerto" -v"$VERBOSO"
        ;;
    ambos)
        dev="${2:-/dev/ttyUSB0}"; baud="${3:-921600}"
        docker run -d --rm --net=host --name atp-agente-udp -e ROS_DOMAIN_ID="$DOMINIO" \
            "$IMG" udp4 --port 8888 -v"$VERBOSO" > /dev/null
        docker run -d --rm --net=host --name atp-agente-serie -e ROS_DOMAIN_ID="$DOMINIO" \
            --device "$dev" "$IMG" serial --dev "$dev" -b "$baud" -v"$VERBOSO" > /dev/null
        echo "agentes en marcha (udp 8888 y serie $dev@$baud, dominio $DOMINIO); ./agente.sh parar"
        ;;
    parar)
        docker rm -f atp-agente-udp atp-agente-serie 2>/dev/null || true
        ;;
    *)
        sed -n '2,16p' "$0"
        exit 2
        ;;
esac
