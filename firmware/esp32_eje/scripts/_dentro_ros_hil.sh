#!/usr/bin/env bash
# Dentro del contenedor ROS (lo lanza prueba_hil_host.sh): el eje de micro-ROS (Linux)
# contra reference_generator y error_monitor de atp_ros, via agente y mcu_merge.
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --packages-select atp_interfaces atp_ros atp_mcu 2>&1 | tail -1
source install/setup.bash
cmake -S firmware/esp32_eje/host -B /tmp/eje_host_build -G "Unix Makefiles" > /dev/null || exit 1
cmake --build /tmp/eje_host_build 2>&1 | grep -E "error|Linking"
# la rampa de atp_ros arranca en az = 10 deg: la montura simulada parte alineada
/tmp/eje_host_build/eje_host --agente 127.0.0.1:8888 --dominio "${ROS_DOMAIN_ID}" --eje az \
    --armar --theta0 10 > /tmp/eje_host.log 2>&1 &
PID_EJE=$!
sleep 3
timeout 120 ros2 launch atp_mcu mcu_hil.launch.py ramp_deg_s:=5 ramp_duration_s:=40 2>&1 \
    | grep -E "RESULTADO|apuntamiento|emparejamiento|enganchado|pase|WARN|ERROR" | head -30
kill -TERM $PID_EJE 2>/dev/null
sleep 1
tail -3 /tmp/eje_host.log
