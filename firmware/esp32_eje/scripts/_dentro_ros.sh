#!/usr/bin/env bash
# Corre DENTRO del contenedor ROS (lo lanza prueba_micro_ros_host.sh).
#   EJES=1 (defecto): eje_host de un eje (az) y su verificacion de siempre.
#   EJES=2: eje_host --ejes 2 (az y el en un solo proceso, como el firmware ATP_EJES=2) y un
#           verificador por eje, mas la comprobacion de que los topicos de los dos ejes existen.
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
echo "--- colcon build atp_mcu"
colcon build --packages-select atp_mcu 2>&1 | tail -2
source install/setup.bash

echo "--- compilar el nodo del eje para Linux"
cmake -S firmware/esp32_eje/host -B /tmp/eje_host_build -G "Unix Makefiles" > /dev/null || exit 1
cmake --build /tmp/eje_host_build 2>&1 | grep -E "warning|error|Linking"
BIN=/tmp/eje_host_build/eje_host

if [ "${EJES:-1}" = "2" ]; then
    $BIN --agente 127.0.0.1:8888 --dominio "${ROS_DOMAIN_ID}" --ejes 2 > /tmp/eje_host.log 2>&1 &
    PID_EJE=$!
    ros2 run atp_mcu mcu_merge > /tmp/mcu_merge.log 2>&1 &
    PID_MERGE=$!
    echo "=== eje az (rampa de 9 deg/s)"
    python3 firmware/esp32_eje/scripts/verificar_host.py --eje az
    estado=$?
    echo "=== eje el (rampa de 1 deg/s: limite de 90 deg)"
    python3 firmware/esp32_eje/scripts/verificar_host.py --eje el --vel-deg-s 1
    estado=$((estado + $?))
else
    $BIN --agente 127.0.0.1:8888 --dominio "${ROS_DOMAIN_ID}" --eje az > /tmp/eje_host.log 2>&1 &
    PID_EJE=$!
    ros2 run atp_mcu mcu_merge --ros-args -p simulated:=el > /tmp/mcu_merge.log 2>&1 &
    PID_MERGE=$!
    python3 firmware/esp32_eje/scripts/verificar_host.py
    estado=$?
fi

echo "--- topicos micro-ROS vistos por ROS 2 (dominio ${ROS_DOMAIN_ID})"
ros2 topic list 2>/dev/null | grep -E "atp/mcu|joint_states|atp/reference"
ros2 topic info /atp/mcu/az/joint_states 2>/dev/null | head -4
kill -TERM $PID_EJE $PID_MERGE 2>/dev/null
sleep 1
echo "--- eje_host"; tail -4 /tmp/eje_host.log
echo "--- mcu_merge"; tail -6 /tmp/mcu_merge.log
exit $estado
