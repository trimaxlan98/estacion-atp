"""Controlador del enlace optico: efemerides y actitud estimadas (con su error),
maquina de estados ESPERA -> BUSQUEDA -> APRETON -> SEGUIMIENTO y eleccion de
tasa.

En lockstep responde a CADA estado de la planta con un mando sellado con el
MISMO instante (/isl/cmd), que es lo que la planta confirma para que el reloj
avance. Solo usa los sensores del mensaje (nunca la verdad) y su propia
estimacion de las orbitas.
"""
from rclpy.node import Node

from atp_ros.nodo import correr
from atp_ros.params import declare_num
from isl_interfaces.msg import IslCmd, IslState
from sat_isl.misiones import NOMBRE_MODO, ControladorIsl
from sat_isl.nodos_comunes import declarar_config, est_de_msg, msg_de_cmd


class IslController(Node):
    def __init__(self):
        super().__init__('isl_controller')
        cfg = declarar_config(self)
        # suscriptores del mando (planta, monitor) que deben estar conectados antes de
        # responder: CONTRATO sec. 8, regla 4
        declare_num(self, 'wait_subscribers', 2)
        self.wait = int(self.get_parameter('wait_subscribers').value)
        self.listo = False
        self.ctrl = ControladorIsl(cfg)
        self.modo = None
        self.pub = self.create_publisher(IslCmd, '/isl/cmd', 50)
        self.create_subscription(IslState, '/isl/state', self.on_state, 50)

    def on_state(self, msg):
        if not self.listo:
            if self.pub.get_subscription_count() < self.wait:
                return          # la planta repite el estado inicial hasta recibir el primer mando
            self.listo = True
        s = est_de_msg(msg)
        c = self.ctrl.paso(s['t'], s)
        if c['modo'] != self.modo:
            self.modo = c['modo']
            self.get_logger().info(f"t={s['t']:.2f} s: modo {NOMBRE_MODO[c['modo']]}")
        # mismo sello que el estado al que responde (CONTRATO sec. 8, regla 1)
        self.pub.publish(msg_de_cmd(IslCmd(), c, s['t'], msg.header.stamp))


def main():
    correr(IslController)
