#!/usr/bin/env python3
"""Modo `estado`: Gazebo solo DIBUJA el rover donde lo dice la dinamica de R1 (/rover/estado, CONTRATO §18).

Suscribe /rover/estado (std_msgs/String, JSON) y, por cada mensaje, manda la pose del modelo estatico
`rover` a Gazebo con el servicio `/world/<mundo>/set_pose_vector` (gz-transport, igual que
reproductor). La fisica es la de R1; Gazebo no calcula nada. Las articulaciones del
balancin y el bogie no se mueven (el estado no las trae todavia: se quedan en la posicion de reposo).

Convenciones (a confirmar con R1 cuando integre): `rumbo_deg` = guinada alrededor de +z desde +x (este)
en sentido antihorario (ENU, como el cuaternion de §18); `cabeceo_deg` positivo = morro arriba (en Gazebo,
rotacion sobre +y positiva baja el morro: se invierte); `alabeo_deg` positivo = lado derecho abajo (= rotacion
sobre +x de Gazebo). z del estado = z del centro del chasis con z = 0 en el origen del terreno; si falta, 0.62 m
(el relieve lo pone R1). `--z0` = altura del terreno en el origen en el mundo de Gazebo (origen.json).
Con --repetir <telemetria.jsonl> reproduce un archivo (sin ROS), util sin el nodo de R1.

    estado_gz.py --mundo rover_marte [--repetir telemetria.jsonl --factor 1.0]
"""
import argparse
import json
import math
import sys
import time

Z_CHASIS = 0.62


def quat(rumbo_deg, cabeceo_deg=0.0, alabeo_deg=0.0):
    """Cuaternion (w,x,y,z) de Rz(yaw) Ry(pitch_gz) Rx(roll), con la convencion del encabezado."""
    y, p, r = math.radians(rumbo_deg) / 2, math.radians(-cabeceo_deg) / 2, math.radians(alabeo_deg) / 2
    cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def pose_de_estado(e, z0=0.0):
    """(posicion, cuaternion) de una fila de estado; None si no trae x/y."""
    if 'x' not in e or 'y' not in e:
        return None
    z = e.get('z')
    z = Z_CHASIS if z is None else z
    return (e['x'], e['y'], z + z0), quat(e.get('rumbo_deg', 0.0), e.get('cabeceo_deg', 0.0), e.get('alabeo_deg', 0.0))


class Enviador:
    def __init__(self, mundo):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self.Boolean, self.Pose_V = Boolean, Pose_V
        self.nodo = Node()
        self.servicio = f'/world/{mundo}/set_pose_vector'

    def enviar(self, pose):
        (x, y, z), (w, qx, qy, qz) = pose
        msg = self.Pose_V()
        m = msg.pose.add()
        m.name = 'rover'
        m.position.x, m.position.y, m.position.z = x, y, z
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = w, qx, qy, qz
        ok, rep = self.nodo.request(self.servicio, msg, self.Pose_V, self.Boolean, 1000)
        return ok and rep.data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mundo', required=True)
    ap.add_argument('--repetir', default=None, help='telemetry.jsonl a reproducir en lugar de escuchar ROS')
    ap.add_argument('--z0', type=float, default=0.0)
    ap.add_argument('--factor', type=float, default=1.0)
    a = ap.parse_args()
    env = Enviador(a.mundo)
    if a.repetir:
        filas = [json.loads(s) for s in open(a.repetir) if s.strip()]
        w0, t0 = time.monotonic(), filas[0]['t']
        for e in filas:
            time.sleep(max(0.0, (e['t'] - t0) / a.factor - (time.monotonic() - w0)))
            p = pose_de_estado(e, a.z0)
            if p:
                env.enviar(p)
        print('estado_gz: FIN', flush=True)
        return 0
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    nodo = rclpy.create_node('rover_estado_gz')
    n = [0]

    def cb(m):
        try:
            p = pose_de_estado(json.loads(m.data), a.z0)
        except ValueError:
            return
        if p and env.enviar(p):
            n[0] += 1

    nodo.create_subscription(String, '/rover/estado', cb, 20)
    print('estado_gz: escuchando /rover/estado', flush=True)
    rclpy.spin(nodo)
    return 0


if __name__ == '__main__':
    sys.exit(main())
