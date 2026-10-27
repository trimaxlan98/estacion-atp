#!/usr/bin/env bash
# Entrypoint del servicio `station` (corre DENTRO del contenedor, ver docker-compose.yml).
# Compila el workspace solo si hace falta, carga el entorno y levanta uvicorn.
# (sin set -u: los setup.bash de ROS usan variables sin definir)
set -e
cd /ws
source /opt/ros/jazzy/setup.bash
marca=build/.station_built
if [ ! -f "$marca" ] || [ -n "$(find src -type f -newer "$marca" -print -quit)" ]; then
  echo "[station] colcon build..."
  colcon build 2>&1 | tail -3
  mkdir -p build && touch "$marca"
fi
source install/setup.bash
cd station/backend
# UN solo worker: la cola de corridas y los procesos viven en memoria
exec python3 -m uvicorn station.main:app --host 127.0.0.1 --port "${STATION_PORT:-8090}" --workers 1
