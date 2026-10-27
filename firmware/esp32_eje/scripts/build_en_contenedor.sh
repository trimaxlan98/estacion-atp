#!/usr/bin/env bash
# Corre DENTRO del contenedor atp-esp32-idf (lo lanza build.sh). No usar a mano.
#   build_en_contenedor.sh serie|udp|dos
set -euo pipefail
VAR="${1:-serie}"
mkdir -p "${HOME}" /cache

# libmicroros se compila dentro de la carpeta del componente: copia escribible
# y persistente (la primera vez tarda; despues se reutiliza).
if [ ! -d /cache/micro_ros_espidf_component ]; then
    cp -r /opt/micro_ros_espidf_component /cache/micro_ros_espidf_component
fi
export MICRO_ROS_COMPONENT_DIR=/cache/micro_ros_espidf_component
export IDF_COMPONENT_CACHE_PATH=/cache/idf_components

cd /project
case "$VAR" in
    serie) B=build;     DEFAULTS="sdkconfig.defaults" ;;
    udp)   B=build_udp; DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.udp" ;;
    dos)   B=build_2ejes; DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.2ejes" ;;
    *) echo "variante desconocida: $VAR (serie|udp|dos)"; exit 2 ;;
esac

echo "=== idf.py build (${VAR}) con $(idf.py --version)"
idf.py -B "$B" -DSDKCONFIG="$B/sdkconfig" -DSDKCONFIG_DEFAULTS="$DEFAULTS" build

echo "=== tamano (esp_idf_size sobre el .map; idf.py size falla en v5.2.8 con -B y -D)"
python -m esp_idf_size "$B/atp_esp32_eje.map" | tee "$B/tamano.txt"
python -m esp_idf_size --archives "$B/atp_esp32_eje.map" > "$B/tamano_componentes.txt" 2>&1 || true

# commits de los repositorios que libmicroros.mk clono por rama (auditoria)
if [ -d "$MICRO_ROS_COMPONENT_DIR/micro_ros_src/src" ]; then
    for d in "$MICRO_ROS_COMPONENT_DIR"/micro_ros_src/src/*/; do
        printf '%s %s\n' "$(git -C "$d" rev-parse HEAD 2>/dev/null || echo ?)" "$(basename "$d")"
    done > "$B/microros-commits.txt"
fi
ls -l "$B"/atp_esp32_eje.bin "$B"/atp_esp32_eje.elf
