"""Planta del enlace optico en LOCKSTEP (CONTRATO.md sec. 8), la misma disciplina
de `mount_sim` y `adcs_sim`.

En cada /clock integra EXACTAMENTE el tiempo transcurrido con el mando vigente
(retencion de orden cero) y publica el estado con ese sello (/isl/state:
medidas de los terminales y verdad). Al recibir el mando CON EL MISMO SELLO lo
aplica y lo confirma en /atp/step_ack, y solo entonces el reloj avanza.

Arranque determinista: no empieza hasta que haya `wait_subscribers` suscriptores
del estado (controlador y monitor). Mientras no reciba el primer mando repite el
estado inicial con el sello del reloj vigente, y el instante de arranque es el
sello del PRIMER mando aceptado: nada depende del descubrimiento DDS. Un mando
con otro sello es rancio y se ignora. Al llegar a `dur_s` publica /isl/done
(t del ultimo paso, latched) y sigue paso a paso para no frenar al resto.
"""
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from atp_ros.nodo import correr
from atp_ros.params import declare_num
from isl_interfaces.msg import IslCmd, IslState
from sat_isl.misiones import PlantaIsl
from sat_isl.nodos_comunes import cmd_de_msg, declarar_config, msg_de_est, sello_ns


class IslSim(Node):
    def __init__(self):
        super().__init__('isl_sim')
        self.cfg = declarar_config(self)
        declare_num(self, 'wait_subscribers', 2)
        self.wait = int(self.get_parameter('wait_subscribers').value)
        self.planta = PlantaIsl(self.cfg)
        c = self.cfg
        self.get_logger().info(
            f'constelacion Walker {c.n_sats}/{c.planos}/{c.fase} a {c.h_km:.0f} km: par '
            f'{self.planta.const.elementos[c.sat_a].nombre} - {self.planta.const.elementos[c.sat_b].nombre}, '
            f'terminal {self.planta.term.nombre} ({c.detector}, {c.d_m * 100:.0f} cm), '
            f'dur {c.dur_s:.0f} s, paso {c.paso_s:g} s')
        self.pend = None            # sello del estado que espera su mando
        self.t_prev_ns = None       # sello del ultimo estado (None: aun no arranca)
        self.done = False
        self.pub = self.create_publisher(IslState, '/isl/state', 50)
        self.ack = self.create_publisher(Clock, '/atp/step_ack', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Float64, '/isl/done', latched)
        self.create_subscription(IslCmd, '/isl/cmd', self.on_cmd, 50)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.cmd = None

    def publicar(self, stamp):
        msg = msg_de_est(IslState(), self.planta.medir(), stamp)
        self.pend = sello_ns(stamp)
        self.pub.publish(msg)

    def on_clock(self, msg):
        s = sello_ns(msg.clock)
        if self.t_prev_ns is None:                      # aun no arranca
            if self.pub.get_subscription_count() >= self.wait:
                self.publicar(msg.clock)                # estado inicial (t_sim = 0)
            return
        elapsed = (s - self.t_prev_ns) * 1e-9
        if elapsed <= 0.0:
            return
        self.planta.avanzar(elapsed, self.cmd)
        self.t_prev_ns = s
        self.publicar(msg.clock)
        if not self.done and self.planta.t >= self.cfg.dur_s - 1e-9:
            self.done = True
            self.done_pub.publish(Float64(data=float(self.planta.t)))
            self.get_logger().info('fin del escenario')

    def on_cmd(self, msg):
        s = sello_ns(msg.header.stamp)
        if self.pend is None or s != self.pend:
            return                                      # rancio: de otro instante
        self.pend = None
        self.cmd = cmd_de_msg(msg)
        if self.t_prev_ns is None:
            self.t_prev_ns = s                          # arranque: el sello del primer mando
            self.get_logger().info('lazo cerrado: arranca el escenario')
        # se confirma el instante PARA EL QUE se calculo el mando
        self.ack.publish(Clock(clock=msg.header.stamp))


def main():
    correr(IslSim)
