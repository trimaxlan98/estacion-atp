#!/usr/bin/env bash
# Estudio comparativo PID / LQR / LQR+Kalman (docs/CONTROL.md) dentro del contenedor.
#   ./run.sh bash scripts/estudio_control.sh            # los tres casos de lazo ROS
#   ./run.sh bash scripts/estudio_control.sh a c        # solo algunos
# Cada linea: caso | controlador | apuntamiento rms / p95 / max (grados) y % dentro de 0.1.
# (d) la campana Monte Carlo es Python puro: python3 scripts/estudio_campana.py
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build 2>&1 | tail -1
source install/setup.bash

# controladores: etiqueta -> argumentos de launch
declare -A CTL=(
  [pid]="controller:=pid"
  [pid+kf]="controller:=pid estimator:=kalman"
  [lqr]="controller:=lqr"
  [lqr+kf]="controller:=lqr estimator:=kalman"
)
ORDEN="pid pid+kf lqr lqr+kf"

# casos: nombre -> argumentos de launch
declare -A CASO=(
  # (a) pase cenital N2 (9 deg/s en acimut, keyhole), banda S
  [a]="el_max:=85 mask:=70"
  # (b) N4b: el mismo cenital en Ka (haz 0.233 deg) con una rafaga de 3 N m a los 60 s
  [b]="el_max:=85 mask:=20 band:=Ka gust_nm:=3.0 gust_start:=60"
  # (c) planta realista: backlash 0.05 deg, Coulomb 0.05 + Stribeck 0.02 N m, encoder 14 bits
  [c]="el_max:=85 mask:=70 backlash_deg:=0.05 coulomb_nm:=0.05 stribeck_nm:=0.02 encoder_bits:=14 truth:=true"
  # (c2) lo anterior + desbalance de elevacion 0.05 N m + viento Dryden de 0.02 N m rms + ruido de encoder 0.005 deg
  [c2]="el_max:=85 mask:=70 backlash_deg:=0.05 coulomb_nm:=0.05 stribeck_nm:=0.02 encoder_bits:=14 truth:=true unbalance_nm:=0.05 wind_sigma_nm:=0.02 noise_deg:=0.005"
)

# Cada celda se corre NREP veces y se imprimen TODAS: el lockstep del baseline
# no es bit-exacto entre lanzamientos cuando la maquina esta cargada (el arranque
# del pase depende del descubrimiento DDS; ver docs/CONTROL.md, limitaciones), y
# una corrida suelta puede perder las primeras muestras (n menor) o correr el
# instante de la rafaga unos pasos. La tabla del documento usa la corrida con el n
# completo mas frecuente.
NREP=${NREP:-3}
casos=${*:-"a b c c2"}
for c in $casos; do
  for k in $ORDEN; do
    for i in $(seq 1 $NREP); do
      sleep 2     # que los nodos de la corrida anterior ya hayan muerto
      r=$(timeout 600 ros2 launch atp_ros atp_loop.launch.py ${CASO[$c]} ${CTL[$k]} time_scale:=0 2>&1 \
          | grep -o "RESULTADO.*\|Traceback.*\|ERROR.*" | head -1)
      echo "caso $c | $k | rep $i | $r"
    done
  done
done
