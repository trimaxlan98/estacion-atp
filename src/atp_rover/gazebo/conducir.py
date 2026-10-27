#!/usr/bin/env python3
"""Maneja el rover de Gazebo por /rover/cmd_vel con una ruta de comandos y registra su telemetria.

Corre DENTRO del contenedor `gz` (lo lanza scripts/gz_rover_contenedor.sh) con use_sim_time: la ruta se
mide en tiempo SIMULADO (el render por software va mas lento que el tiempo real, por eso). Escribe
`telemetria.jsonl` con filas en el formato de /rover/estado (CONTRATO §18; solo los campos que se miden:
t, x, y, rumbo_deg, v, w; un campo sin dato se omite) a partir de /rover/odom (puente de ros_gz_bridge).

    conducir.py --ruta demo|recta|giro|<archivo.json> --salida <dir> [--dur 60]
La ruta es una lista [[t_inicio_s, v_m_s, w_rad_s], ...] ordenada; el ultimo tramo dura hasta --dur.
"""
import argparse
import json
import math
import os
import sys
import time

RUTAS = {
    # un trazado que muestra el balancin: recta, arco a la izquierda, recta, arco a la derecha, giro en sitio
    'demo': [[0, 0.0, 0.0], [2, 0.6, 0.0], [10, 0.5, 0.35], [18, 0.6, 0.0], [24, 0.5, -0.35], [32, 0.6, 0.0],
             [38, 0.0, 0.5], [44, 0.6, 0.0]],
    'recta': [[0, 0.0, 0.0], [2, 0.8, 0.0]],
    'giro': [[0, 0.0, 0.0], [2, 0.0, 0.6]],
}


def cargar_ruta(r):
    if r in RUTAS:
        return RUTAS[r]
    with open(r) as fh:
        return json.load(fh)


def comando(ruta, t):
    c = ruta[0]
    for tramo in ruta:
        if t >= tramo[0]:
            c = tramo
    return c[1], c[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ruta', default='demo')
    ap.add_argument('--salida', required=True)
    ap.add_argument('--dur', type=float, default=50.0, help='duracion simulada de la corrida, s')
    a = ap.parse_args()
    ruta = cargar_ruta(a.ruta)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    nodo = rclpy.create_node('rover_conducir')
    pub = nodo.create_publisher(Twist, '/rover/cmd_vel', 10)
    os.makedirs(a.salida, exist_ok=True)
    fh = open(os.path.join(a.salida, 'telemetria.jsonl'), 'w')
    pos0 = {}

    def odom(m):
        if 't0' not in pos0:
            return                       # aun no arranca la ruta
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - pos0['t0']
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        fila = {'t': round(t, 3), 'x': round(m.pose.pose.position.x, 4), 'y': round(m.pose.pose.position.y, 4),
                'rumbo_deg': round(math.degrees(yaw), 3), 'v': round(m.twist.twist.linear.x, 4),
                'w': round(m.twist.twist.angular.z, 4)}
        fh.write(json.dumps(fila) + '\n')
        fh.flush()
        pos0['n'] = pos0.get('n', 0) + 1

    nodo.create_subscription(Odometry, '/rover/odom', odom, 20)
    # espera el reloj simulado (puente de /clock) para empezar la ruta
    t0 = None
    espera = time.monotonic()
    while t0 is None and time.monotonic() - espera < 600:
        rclpy.spin_once(nodo, timeout_sec=0.2)
        ahora = nodo.get_clock().now().nanoseconds * 1e-9
        if ahora > 0:
            t0 = ahora
    if t0 is None:
        print('conducir: no llego /clock', flush=True)
        return 2
    print(f'conducir: inicio de la ruta {a.ruta} (t sim {t0:.2f} s)', flush=True)
    pos0['t0'] = t0
    ult = None
    while True:
        rclpy.spin_once(nodo, timeout_sec=0.05)
        t = nodo.get_clock().now().nanoseconds * 1e-9 - t0
        v, w = comando(ruta, t)
        if (v, w) != ult or int(t * 10) % 5 == 0:
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            pub.publish(msg)
            ult = (v, w)
        if t >= a.dur:
            break
    pub.publish(Twist())
    for _ in range(10):
        rclpy.spin_once(nodo, timeout_sec=0.1)
    fh.close()
    print(f'conducir: FIN t={t:.1f} s sim, {pos0.get("n", 0)} muestras de odometria', flush=True)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
