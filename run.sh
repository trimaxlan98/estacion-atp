#!/usr/bin/env bash
# Uso: ./run.sh            -> shell interactiva en el contenedor
#      ./run.sh <comando>  -> ejecuta comando (ej: ./run.sh ros2 topic list)
cd "$(dirname "$0")"
xhost +local:docker >/dev/null 2>&1 || true
docker compose run --rm ros "${@:-bash}"
