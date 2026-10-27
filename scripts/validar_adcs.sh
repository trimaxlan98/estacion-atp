#!/usr/bin/env bash
# Valida el segmento espacio (ADCS del CubeSat) dentro del contenedor.
# Uso: ./run.sh bash scripts/validar_adcs.sh [paso...]   (sin argumentos: todo)
# (sin set -u: los setup.bash de ROS usan variables sin definir)
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sat_interfaces sat_adcs 2>&1 | tail -2
source install/setup.bash
pasos=${*:-"test escenario eclipse dipolo det"}
filtro='RESUMEN|pase real|campo de a bordo|modo |Traceback|ERROR|sin confirmar'
for n in $pasos; do
  echo "=================== $n"
  case $n in
    test) python3 -m pytest -q src/sat_adcs/test 2>&1 | tail -5 ;;
    # DETUMBLE -> NADIR -> TRACK_GS, pase real de FUNCUBE-1 sobre CDMX en luz (campo de a bordo grado 4)
    escenario) timeout 1500 ros2 launch sat_adcs adcs.launch.py time_scale:=0 2>&1 | grep -E "$filtro" ;;
    # el mismo, con un pase DENTRO del eclipse (sin sensor solar)
    eclipse) timeout 1500 ros2 launch sat_adcs adcs.launch.py time_scale:=0 start_utc:=2026-10-02T08:30:00 2>&1 | grep -E "$filtro" ;;
    # el mismo pase con el dipolo inclinado a bordo (limitacion: ver src/sat_adcs/README.md)
    dipolo) timeout 1500 ros2 launch sat_adcs adcs.launch.py time_scale:=0 model_degree:=1 2>&1 | grep -E "$filtro" ;;
    # Determinismo del lockstep: la misma corrida dos veces debe dar el mismo RESUMEN exacto.
    # Escenario corto (tumbling de 3 deg/s, arranca 56 min antes del pase) para que sean 2 corridas rapidas.
    det)
      for i in 1 2; do
        timeout 1500 ros2 launch sat_adcs adcs.launch.py time_scale:=0 w0_deg_s:=3 start_utc:=2026-10-02T21:20:00 lead_s:=3000 2>&1 | grep -o "RESUMEN.*" > /tmp/det_adcs$i.txt
        cat /tmp/det_adcs$i.txt
      done
      if [ -s /tmp/det_adcs1.txt ] && cmp -s /tmp/det_adcs1.txt /tmp/det_adcs2.txt; then
        echo "DETERMINISTA (adcs.launch.py): RESUMEN identico"
      else
        echo "NO DETERMINISTA (adcs.launch.py): difieren o vacio"
      fi ;;
  esac
done
