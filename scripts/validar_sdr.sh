#!/usr/bin/env bash
# Valida la cadena de RF a nivel IQ (equipo M) dentro del contenedor.
# Uso: ./run.sh bash scripts/validar_sdr.sh [test n2 s4 det ka at]
#   test  pruebas contra teoria (pytest, ~3 min)
#   n2    pase cenital N2 en banda S con el receptor SDR en el lazo
#   s4    pase REAL de la ISS sobre CDMX (SGP4) en banda S
#   det   la misma corrida N2 dos veces: el RESUMEN SDR debe ser identico
#   ka    pase oblicuo completo N4 en Ka (el caso caro: 2 MS/s)
#   at    autotrack N5: program track, rf_beacon (dB ideal + ruido) y beacon IQ
# (sin set -u: los setup.bash de ROS usan variables sin definir)
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select atp_interfaces sdr_interfaces atp_ros atp_sdr 2>&1 | tail -2
source install/setup.bash
pasos=${*:-"test n2 s4 det ka at"}
filtro='RESUMEN SDR|RENDIMIENTO SDR|RESULTADO|ENLACE|BEACON IQ|Traceback|ERROR|sin confirmar'
lanzar() { timeout 1700 ros2 launch "$@" 2>&1 | grep -E "$filtro|pase real" | cut -c1-600; }
for n in $pasos; do
  echo "=================== $n"
  case $n in
    test) python3 -m pytest -q src/atp_sdr/test 2>&1 | tail -3 ;;
    n2)  lanzar atp_sdr sdr_loop.launch.py el_max:=85 mask:=70 time_scale:=0 ;;
    s4)  lanzar atp_sdr sdr_loop.launch.py profile:=tle sat:=ISS station:=cdmx pass_index:=0 start_utc:=2026-10-02T15:00:00 time_scale:=0 band:=S ;;
    ka)  lanzar atp_sdr sdr_loop.launch.py el_max:=30 mask:=5 time_scale:=0 band:=Ka ;;
    det)
      for i in 1 2; do
        timeout 900 ros2 launch atp_sdr sdr_loop.launch.py el_max:=85 mask:=70 time_scale:=0 \
          > /tmp/sdr_det_$i.log 2>&1
        grep "RESUMEN SDR" /tmp/sdr_det_$i.log | sed 's/^.*RESUMEN SDR/RESUMEN SDR/' > /tmp/sdr_det_$i.txt
      done
      if [ -s /tmp/sdr_det_1.txt ] && cmp -s /tmp/sdr_det_1.txt /tmp/sdr_det_2.txt; then
        echo "DETERMINISTA: $(cat /tmp/sdr_det_1.txt)"
      else echo "NO DETERMINISTA:"; cat /tmp/sdr_det_1.txt /tmp/sdr_det_2.txt; fi ;;
    at)
      lanzar atp_sdr sdr_autotrack.launch.py beacon:=sdr autotrack:=false
      lanzar atp_sdr sdr_autotrack.launch.py beacon:=rf autotrack:=true
      lanzar atp_sdr sdr_autotrack.launch.py beacon:=sdr autotrack:=true
      lanzar atp_sdr sdr_autotrack.launch.py beacon:=sdr autotrack:=true delta_cn0_db:=-35 ;;
  esac
done
