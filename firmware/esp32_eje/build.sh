#!/usr/bin/env bash
# Compila el firmware del eje en Docker (imagen oficial espressif/idf v5.2.8).
#
#   ./build.sh            variante SERIE  -> build/atp_esp32_eje.bin
#   ./build.sh udp        variante UDP/Wi-Fi -> build_udp/atp_esp32_eje.bin
#   ./build.sh dos        variante SERIE de dos ejes (banco) -> build_2ejes/atp_esp32_eje.bin
#   ./build.sh shell      shell dentro del contenedor (menuconfig, etc.)
#
# La primera vez baja la imagen (4.7 GB comprimida, 11.9 GB en disco) y compila
# libmicroros (~10-20 min, necesita red para clonar repos de GitHub); las
# siguientes reutilizan la cache en .cache/. Flashear: ver README.md.
set -euo pipefail
cd "$(dirname "$0")"
IMG="atp-esp32-idf:5.2.8"
VAR="${1:-serie}"

docker build -q -t "$IMG" . > /dev/null
mkdir -p .cache

run() {
    docker run --rm --user "$(id -u):$(id -g)" \
        -e HOME=/tmp/home -e CMAKE_BUILD_PARALLEL_LEVEL -e MAKEFLAGS \
        -v "$PWD":/project -v "$PWD/.cache":/cache \
        "$@"
}

if [ "$VAR" = "shell" ]; then
    run -it "$IMG" bash
else
    run "$IMG" bash /project/scripts/build_en_contenedor.sh "$VAR"
fi
