#!/usr/bin/env bash
# Ejecuta un comando dentro de la imagen ros2-jazzy-dev con el workspace montado en /ws
# (lo usa .github/workflows/ci.yml; sirve igual en local). Corre como el usuario `dev`
# de la imagen, que en CI se construye con el UID del runner (--build-arg UID=...), asi
# que build/ install/ log/ quedan con el dueno correcto.
#   scripts/ci_ros.sh 'colcon build && python3 -m pytest -q src/atp_ros/test'
# Carga /opt/ros/jazzy y, si existe, install/setup.bash. Falla al primer error.
set -eu
cd "$(dirname "$0")/.."
exec docker run --rm --network host --ipc host ${DOCKER_RUN_ARGS:-} \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}" \
  -e ATP_HW_INTEGRATION="${ATP_HW_INTEGRATION:-0}" \
  -v "$PWD:/ws" -w /ws "${ROS_IMAGE:-ros2-jazzy-dev}" \
  bash -c "set -eo pipefail; source /opt/ros/jazzy/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; $*"
