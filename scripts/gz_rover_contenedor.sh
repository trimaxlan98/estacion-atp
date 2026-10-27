#!/usr/bin/env bash
# Parte de gz_rover.sh que corre DENTRO del contenedor `gz`: Gazebo con el mundo generado, el puente
# ROS<->gz (/clock, /rover/cmd_vel, /rover/odom), la camara que sigue al rover y quien lo maneja.
# Entorno (lo pone gz_rover.sh): GZ_SALIDA (carpeta de salida), MUNDO (nombre del world), MODO
# (diffdrive | estado | externo), RUTA, DUR (s simulados), GZ_PARTITION.
source /opt/ros/jazzy/setup.bash
cd /ws
S=${GZ_SALIDA:?}
G=/ws/src/atp_rover/gazebo
export GZ_PARTITION=${GZ_PARTITION:-gz_rover} GZ_IP=127.0.0.1
export PYTHONPATH=$G:$PYTHONPATH

gz sim -r -v 2 --gui-config $S/gui.config $S/mundo.sdf > $S/gz.log 2>&1 &
gzpid=$!
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /rover/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry > $S/puente.log 2>&1 &

# espera a que el mundo corra (hay /clock) y a que la GUI exista; en software tarda
for _ in $(seq 1 600); do
  kill -0 $gzpid 2>/dev/null || { echo "gz sim termino (mundo invalido?); ver gz.log" > $S/error; exit 1; }
  gz topic -l 2>/dev/null | grep -q "^/clock" && break
  sleep 1
done
sleep 5
# camara de seguimiento (GUI): sigue al modelo `rover` desde atras y arriba
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'data: "rover"' > $S/camara.log 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'x: -4.2 y: -3.2 z: 2.4' >> $S/camara.log 2>&1
echo LISTO > $S/listo
# espera la orden de arrancar (gz_rover.sh ya esta grabando)
while [ ! -e $S/go ]; do sleep 0.1; done

case ${MODO:-diffdrive} in
  diffdrive)
    python3 $G/conducir.py --ruta "${RUTA:-demo}" --salida $S --dur "${DUR:-50}" > $S/conducir.log 2>&1 ;;
  estado)
    Z0=$(python3 -c "import json;print(json.load(open('$S/origen.json'))['z0'])")
    if [ -n "${ESTADO_REPETIR:-}" ]; then      # sin R1: reproduce un telemetria.jsonl (formato /rover/estado)
      python3 $G/estado_gz.py --mundo "$MUNDO" --z0 "$Z0" --repetir "/ws/$ESTADO_REPETIR" > $S/estado_gz.log 2>&1
    else                                        # R1 publica /rover/estado; aqui solo se espera el tiempo (real) pedido
      python3 $G/estado_gz.py --mundo "$MUNDO" --z0 "$Z0" > $S/estado_gz.log 2>&1 &
      sleep "${DUR:-50}"
    fi ;;
  *)
    sleep "${DUR:-50}" ;;      # otro nodo publica /rover/cmd_vel
esac
touch $S/fin_ok
sleep 2
kill $gzpid 2>/dev/null
sleep 1
pkill -f "gz sim" 2>/dev/null
exit 0
