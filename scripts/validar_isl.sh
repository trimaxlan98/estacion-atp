#!/usr/bin/env bash
# Valida el ATP optico entre satelites (src/sat_isl) dentro del contenedor.
# Uso: ./run.sh bash scripts/validar_isl.sh [paso...]   (sin argumentos: todo)
# (sin set -u: los setup.bash de ROS usan variables sin definir)
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isl_interfaces sat_isl 2>&1 | tail -2
source install/setup.bash
pasos=${*:-"test escenario planos det estudio"}
filtro='RESUMEN|modo |Traceback|ERROR|sin confirmar'
for n in $pasos; do
  echo "=================== $n"
  case $n in
    test) python3 -m pytest -q src/sat_isl/test 2>&1 | tail -5 ;;
    # par del mismo plano (P0S0-P0S1, 5.5 Mm): busqueda -> apreton -> seguimiento -> enlace
    escenario) timeout 900 ros2 launch sat_isl isl.launch.py time_scale:=0 dur_s:=60 2>&1 | grep -E "$filtro" ;;
    # par de planos distintos (P0S0-P2S2): la ventana de visibilidad termina hacia t=1060 s
    planos) timeout 900 ros2 launch sat_isl isl.launch.py time_scale:=0 sat_b:=18 start_s:=1020 dur_s:=60 2>&1 | grep -E "$filtro" ;;
    # Determinismo del lockstep: la misma corrida dos veces en ROS da el mismo RESUMEN exacto,
    # y es el de la corrida sin ROS (sat_isl.escenario)
    det)
      for i in 1 2; do
        timeout 900 ros2 launch sat_isl isl.launch.py time_scale:=0 dur_s:=20 2>&1 | grep -o "RESUMEN.*" > /tmp/det_isl$i.txt
        cat /tmp/det_isl$i.txt
      done
      python3 -m sat_isl.escenario --dur_s 20 2>/dev/null > /tmp/det_isl_offline.txt
      if [ -s /tmp/det_isl1.txt ] && cmp -s /tmp/det_isl1.txt /tmp/det_isl2.txt; then
        echo "DETERMINISTA (isl.launch.py): RESUMEN identico"
      else
        echo "NO DETERMINISTA (isl.launch.py): difieren o vacio"
      fi
      if cmp -s /tmp/det_isl1.txt /tmp/det_isl_offline.txt; then
        echo "ROS = SIN ROS: RESUMEN identico a sat_isl.escenario"
      else
        echo "ROS != SIN ROS: difieren"
      fi ;;
    # las tablas del README (geometria, haz, enlace, adquisicion, lazo)
    estudio) python3 scripts/isl_estudio.py todo 2>&1 | head -80 ;;
  esac
done
