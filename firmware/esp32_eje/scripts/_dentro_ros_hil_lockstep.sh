#!/usr/bin/env bash
# Dentro del contenedor ROS (lo lanza prueba_hil_lockstep.sh): eje_host de dos ejes con eco
# + lanzador mcu_hil_lockstep (sim_clock lockstep, generador, error_monitor, mcu_lockstep).
set -o pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
RES=/ws/firmware/esp32_eje/.cache/hil
mkdir -p "$RES"
rm -f "$RES"/hil.json "$RES"/hil.csv "$RES"/eje_host.json
colcon build --packages-select atp_interfaces atp_ros atp_mcu 2>&1 | tail -1
source install/setup.bash
cmake -S firmware/esp32_eje/host -B /tmp/eje_host_build -G "Unix Makefiles" > /dev/null || exit 1
cmake --build /tmp/eje_host_build 2>&1 | grep -E "warning|error|Linking"
echo "--- carga al empezar: $(cut -d' ' -f1-3 /proc/loadavg)"
# telemetria a 100 Hz: la interpolacion del estado en el instante exacto necesita muestras densas
/tmp/eje_host_build/eje_host --agente 127.0.0.1:8888 --dominio "${ROS_DOMAIN_ID}" --ejes 2 \
    --eco --tel-hz 100 --stats "$RES/eje_host.json" > "$RES/eje_host.log" 2>&1 &
PID_EJE=$!
sleep 3
timeout 400 ros2 launch atp_mcu mcu_hil_lockstep.launch.py el_max:="${EL_MAX:-85}" mask:="${MASK:-70}" \
    salida:="$RES/hil" > "$RES/launch.log" 2>&1
echo "--- ros2 launch termino con codigo $?"
grep -E "RESULTADO|apuntamiento|lazo cerrado|enganch|fin del pase|WARN|ERROR" "$RES/launch.log" | head -30
kill -TERM $PID_EJE 2>/dev/null
sleep 2
echo "--- eje_host"
tail -6 "$RES/eje_host.log"
echo "--- carga al terminar: $(cut -d' ' -f1-3 /proc/loadavg)"
[ -f "$RES/hil.json" ] && python3 -c "
import json; r = json.load(open('$RES/hil.json'))
print('rtt_ms', r['rtt_ms']); print('retraso_cierre_ms', r['retraso_de_cierre_ms'])
print('diag', r['diag_mcu']); print('deriva', r['deriva_vs_gemelo'])
print('eco perdidos', r['eco_perdidos'], 'pasos con MCU tarde', r['pasos_con_mcu_tarde'])"
