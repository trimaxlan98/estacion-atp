#!/usr/bin/env python3
"""Mueve el satelite VISIBLE y el haz de la antena en la escena de Gazebo.

El haz (cilindro delgado cian de `radio` m) sale del cruce de las juntas a lo largo del eje REAL
de la antena (`/joint_states`): si el lazo se retrasa, se ve separarse del satelite.

El satelite de `atp_gz.launch.py escena:=true` es una esfera que NO tiene dinamica: este
nodo la coloca, a `radio` metros del centro de la montura, en la direccion (az, el) de
`/atp/reference` (lo que persigue el controlador). Es una ayuda visual a escala: el
satelite real esta a 550+ km, aqui se dibuja a unos metros en la MISMA direccion, y la
antena (que mueve la fisica de Gazebo con el lazo de siempre) apunta a el. Si el lazo
fallara, el eje de la antena se separaria de la esfera.

Convencion de la montura (atp_description): la junta `az` gira sobre +z y `el` sobre -y,
de modo que el eje de apuntamiento es (cos el cos az, cos el sin az, sin el); el cruce de
las juntas esta a `z_centro` (1.0 + 0.9 m de pedestal).

Parametros: radio (m), z_centro (m), modelo y haz (nombres en el mundo), mundo.
"""
import math
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import JointState


def direccion(az, el):
    """Vector unitario del eje de la antena para az/el (rad)."""
    return (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def cuaternion_eje_z(az, el):
    """Cuaternion (w, x, y, z) que lleva +z (el eje de un cilindro) a `direccion(az, el)`:
    Rz(az) * Ry(pi/2 - el)."""
    cy, sy = math.cos(az / 2), math.sin(az / 2)
    cp, sp = math.cos((math.pi / 2 - el) / 2), math.sin((math.pi / 2 - el) / 2)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


class Escena(Node):
    def __init__(self):
        super().__init__('atp_gz_escena')
        self.radio = float(self.declare_parameter('radio', 15.0).value)
        self.zc = float(self.declare_parameter('z_centro', 1.9).value)
        self.modelo = str(self.declare_parameter('modelo', 'satelite').value)
        self.haz = str(self.declare_parameter('haz', 'haz').value)
        mundo = str(self.declare_parameter('mundo', 'atp').value)
        self.servicio = f'/world/{mundo}/set_pose'
        self.gz = GzNode()
        self.ultimo = 0.0
        self.ultimo_haz = 0.0
        self.fallos = 0
        self.create_subscription(JointState, '/atp/reference', self.on_ref, 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.get_logger().info(f'escena: {self.modelo} a {self.radio} m, servicio {self.servicio}')

    def poner(self, nombre, x, y, z, q=(1.0, 0.0, 0.0, 0.0)):
        p = Pose()
        p.name = nombre
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = q
        ok, rep = self.gz.request(self.servicio, p, Pose, Boolean, 200)
        if not (ok and rep.data):
            self.fallos += 1
            if self.fallos in (1, 20, 200):
                self.get_logger().warn(f'set_pose({nombre}) fallo ({self.fallos}); sigue intentando')

    def on_js(self, msg):
        ahora = time.monotonic()
        if ahora - self.ultimo_haz < 0.04 or 'az' not in msg.name or 'el' not in msg.name:
            return
        self.ultimo_haz = ahora
        az, el = msg.position[msg.name.index('az')], msg.position[msg.name.index('el')]
        dx, dy, dz = direccion(az, el)
        m = self.radio / 2.0                   # el cilindro esta centrado: a medio radio
        self.poner(self.haz, m * dx, m * dy, self.zc + m * dz, cuaternion_eje_z(az, el))

    def on_ref(self, msg):
        ahora = time.monotonic()
        if ahora - self.ultimo < 0.04:          # <= 25 Hz: la escena no necesita mas
            return
        self.ultimo = ahora
        az, el = msg.position[0], msg.position[1]
        dx, dy, dz = direccion(az, el)
        self.poner(self.modelo, self.radio * dx, self.radio * dy, self.zc + self.radio * dz)


def main():
    rclpy.init()
    nodo = Escena()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()


if __name__ == '__main__':
    main()
