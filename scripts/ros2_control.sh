#!/usr/bin/env bash
# Valida la montura en ros2_control (equipo K) dentro del servicio `gz` (imagen
# ros2-jazzy-gz: ros2_control + Gazebo Harmonic). Uso, desde la raiz del repo:
#
#   docker compose run --rm -T -e ROS_DOMAIN_ID=82 gz bash scripts/ros2_control.sh [nivel...]
#
# Niveles (sin argumentos: test n2_ref n2 c1 c1_ref serial det):
#   test    pruebas gtest de atp_control (planta contra planta.py, leyes contra control.py,
#           protocolo serie, extrapolacion de la referencia)
#   n2_ref  el pase N2 del lazo de REFERENCIA (mount_sim + pid_controller, atp_ros)
#   n2      el mismo pase con ros2_control (AtpMountSystem sim + AtpTrackingController, PID)
#   c1_ref  N2 con el LQR de atp_ros;  c1  N2 con el LQR en ros2_control
#   c2_ref / c2  LQR + Kalman con planta realista (backlash, friccion, encoder 14 bits)
#   serial  N2 con el backend serial (microcontrolador falso en proceso)
#   servo   modo de POSICION: joint_trajectory_controller sobre el plugin (sim y serial)
#   rt      N2 con el ros2_control_node ESTANDAR (reloj de pared a 100 Hz, sin lockstep)
#   ident   identificacion de J y b con un escalon de par (sim y Gazebo)
#   det     el pase n2 dos veces con grabador: telemetry.jsonl identico byte a byte
#   free    N2 en reloj libre (mode:=free, tiempo real x1): lo que veria un banco sin lockstep
#   gz      pase N2 contra Gazebo a tiempo real (REP=3 lo repite 3 veces); gz_step en modo
#           paso a paso; gz_lqr con el LQR. error_monitor solo empareja sellos exactos: en
#           Gazebo la cifra es la de pointing_eval (EVAL)
# (sin set -u: los setup.bash de ROS usan variables sin definir)
cd /ws
source /opt/ros/jazzy/setup.bash
# --cmake-force-configure: un build previo en la imagen base (station) deja atp_control vacio con la cache de CMake
colcon build --packages-up-to atp_control --cmake-force-configure 2>&1 | tail -3
source install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-82}
niveles=${*:-"test n2_ref n2 c1_ref c1 c2_ref c2 serial servo det"}
filtro='RESULTADO|EVAL|Traceback|ERROR|sin confirmar|respaldo|no coinciden|DETERMIN'
N2="el_max:=85 mask:=70 time_scale:=0"
REAL="backlash_deg:=0.05 coulomb_nm:=0.05 stribeck_nm:=0.02 encoder_bits:=14"
for n in $niveles; do
  echo "=================== $n"
  case $n in
    test)
      for t in test_plant test_laws test_serial test_controller_ref; do
        ./build/atp_control/$t 2>&1 | grep -E "^\[planta\]|^\[lqr\]|^\[ley|^\[serial\]|PASSED|FAILED"
      done ;;
    n2_ref) timeout 300 ros2 launch atp_ros atp_loop.launch.py $N2 2>&1 | grep -E "$filtro" ;;
    n2)     timeout 300 ros2 launch atp_control atp_ros2_control.launch.py $N2 2>&1 | grep -E "$filtro" ;;
    c1_ref) timeout 300 ros2 launch atp_ros atp_loop.launch.py $N2 controller:=lqr 2>&1 | grep -E "$filtro" ;;
    c1)     timeout 300 ros2 launch atp_control atp_ros2_control.launch.py $N2 controller:=lqr 2>&1 | grep -E "$filtro" ;;
    c2_ref) timeout 300 ros2 launch atp_ros atp_loop.launch.py $N2 controller:=lqr estimator:=kalman $REAL 2>&1 | grep -E "$filtro" ;;
    c2)     timeout 300 ros2 launch atp_control atp_ros2_control.launch.py $N2 controller:=lqr estimator:=kalman $REAL 2>&1 | grep -E "$filtro" ;;
    servo)  for b in sim serial; do echo "-- backend $b"; timeout 120 ros2 launch atp_control atp_position.launch.py backend:=$b 2>&1 | grep -E "POSICION|Traceback|ERROR" | grep -v "pal_stat\|FIFO"; done ;;
    serial) timeout 300 ros2 launch atp_control atp_ros2_control.launch.py $N2 backend:=serial 2>&1 | grep -E "$filtro" ;;
    free)   timeout 300 ros2 launch atp_control atp_ros2_control.launch.py el_max:=85 mask:=70 time_scale:=1 mode:=free 2>&1 | grep -E "$filtro" ;;
    det)
      rm -rf /tmp/det; for i in 1 2; do
        timeout 300 ros2 launch atp_control atp_ros2_control.launch.py $N2 record_dir:=/tmp/det/$i > /tmp/det_$i.log 2>&1
      done
      if cmp -s /tmp/det/1/telemetry.jsonl /tmp/det/2/telemetry.jsonl && cmp -s /tmp/det/1/metrics.json /tmp/det/2/metrics.json; then
        echo "DETERMINISTA (ros2_control n2): $(wc -l < /tmp/det/1/telemetry.jsonl) filas identicas"
      else
        echo "NO DETERMINISTA (ros2_control n2): difieren"
        diff /tmp/det/1/telemetry.jsonl /tmp/det/2/telemetry.jsonl | head -4 | cut -c1-200
        grep -h "reintentos\|no llego" /tmp/det_1.log /tmp/det_2.log | cut -c1-200
      fi ;;
    rt)     for c in pid lqr; do timeout 300 ros2 launch atp_control atp_ros2_control.launch.py mode:=realtime controller:=$c 2>&1 | grep -E "$filtro" | grep -v "no coinciden"; done ;;
    ident)  for a in "backend:=sim axis:=0" "backend:=gz axis:=0"; do echo "-- $a"; timeout 200 ros2 launch atp_control atp_identify.launch.py $a 2>&1 | grep -E "IDENT|Traceback" | sed 's/^.*IDENT //'; done ;;
    gz|gz_step|gz_lqr)
      extra=""; [ "$n" = gz_step ] && extra="stepped:=true"; [ "$n" = gz_lqr ] && extra="controller:=lqr"
      for i in $(seq ${REP:-1}); do
        timeout 900 ros2 launch atp_control atp_gz.launch.py headless:=true $extra 2>&1 | grep -E "$filtro" | grep -v "error_monitor"
      done ;;
  esac
done
