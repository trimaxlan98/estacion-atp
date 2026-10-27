"""Nodo ROS 2 `rover_navegador` (R2): sigue /rover/meta y publica /rover/cmd_vel y /rover/alarma.

Suscribe /rover/estado (String JSON) y /rover/meta (PoseStamped); publica /rover/cmd_vel (Twist) al recibir cada
estado y /rover/alarma (String JSON). Todo el calculo esta en `navegacion.Navegador` (Python puro).
Terreno: si `atp_rover.terreno` (R1) existe se construye con los parametros `terreno_tipo` y `semilla`; si no,
se navega en linea recta (sin evasion de terreno).

Dos modos (`modo`):
  * `pasivo` (defecto): lo de arriba; responde a cada /rover/estado en el orden en que llegue (sin sello ni ficha).
  * `lockstep` (CONTRATO sec. 8, R9; `atp_rover.nodos_lockstep.NavegadorROS`): el controlador de `lazo` (basico | completa) con la
    ruta de la escena (parametro `cfg`, JSON). Sin enlace responde a CADA estado e_k con el mando de e_k y la ficha
    `/rover/cmd_vel_listo`; con enlace es el operador en Tierra: consume `/rover/estado_tierra` y publica `/rover/cmd_vel_op`
    (o sigue el `guion` si `operador` es `guion`). Nunca responde dos veces al mismo sello.
"""
import json

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from atp_rover.navegacion import Navegador, ParamsNav
from atp_rover.seguridad import MonitorSeguridad, ParamsSeguridad


def _terreno(tipo, semilla):
    try:
        from atp_rover.terreno import Terreno
        return Terreno(tipo=tipo, semilla=semilla)
    except Exception:
        return None


class RoverNavegador(Node):
    def __init__(self):
        super().__init__('rover_navegador')
        self.declare_parameter('modo', 'pasivo')
        if self.get_parameter('modo').value == 'lockstep':
            from atp_rover.nodos_lockstep import NavegadorROS
            self.declare_parameter('cfg', '{}')
            self.declare_parameter('wait_subscribers', 1)
            self.lock = NavegadorROS(self, json.loads(self.get_parameter('cfg').value),
                                     self.get_parameter('wait_subscribers').value)
            return
        for k, d in (('terreno_tipo', 'llano'), ('semilla', 0), ('v_max', 1.0),
                     ('pend_max_deg', 20.0), ('energia_max_wh', 1e9)):
            self.declare_parameter(k, d)
        g = self.get_parameter
        terr = _terreno(g('terreno_tipo').value, g('semilla').value)
        self.nav = Navegador(terr, ParamsNav(v_max=g('v_max').value, pend_max_deg=g('pend_max_deg').value),
                             MonitorSeguridad(ParamsSeguridad(energia_max_wh=g('energia_max_wh').value)))
        self.estado = None
        self.pub_cmd = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.pub_alarma = self.create_publisher(String, '/rover/alarma', 10)
        self.create_subscription(String, '/rover/estado', self._on_estado, 10)
        self.create_subscription(PoseStamped, '/rover/meta', self._on_meta, 10)

    def _on_meta(self, msg):
        if self.estado is None:
            self.get_logger().warn('meta recibida sin estado; ignorada')
            return
        self.nav.nueva_meta(self.estado, (msg.pose.position.x, msg.pose.position.y))
        if self.nav.sin_ruta:
            self.get_logger().warn('sin ruta transitable a la meta')

    def _on_estado(self, msg):
        self.estado = json.loads(msg.data)
        (v, w), alarmas = self.nav.paso(self.estado)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub_cmd.publish(cmd)
        for a in alarmas:
            self.pub_alarma.publish(String(data=json.dumps(a)))


def main(args=None):
    from atp_ros.nodo import correr
    correr(RoverNavegador)


if __name__ == '__main__':
    main()
