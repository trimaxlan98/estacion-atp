"""Planta ADCS en LOCKSTEP (CONTRATO.md sec. 8), la misma disciplina de `mount_sim`.

En cada /clock integra EXACTAMENTE el tiempo transcurrido con el mando
vigente (retencion de orden cero) y publica el estado con ese sello
(/atp/adcs_state: sensores y verdad). Al recibir el mando CON EL MISMO SELLO
lo aplica y lo confirma en /atp/step_ack, y solo entonces el reloj avanza.

Arranque determinista: no empieza hasta que haya `wait_subscribers`
suscriptores del estado (controlador y monitor). Mientras no reciba el
primer mando, repite el estado inicial con el sello del reloj vigente, y el
instante de arranque es el sello del PRIMER mando aceptado: asi nada
depende de cuanto tarde el descubrimiento DDS. Un mando con otro sello es
rancio y se ignora. Al llegar al fin del escenario (LOS + cola) publica
/atp/adcs_done (t del ultimo paso, latched) y sigue paso a paso para no frenar al resto.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from sat_adcs.escenario import preparar
from sat_adcs.misiones import PlantaAdcs
from sat_adcs.nodos_comunes import declarar_config, msg_de_sens, sello_ns
from sat_interfaces.msg import AdcsCmd, AdcsState
from atp_ros.nodo import correr
from atp_ros.params import declare_num


class AdcsSim(Node):
    def __init__(self):
        super().__init__('adcs_sim')
        cfg = declarar_config(self)
        declare_num(self, 'wait_subscribers', 2)
        self.wait = int(self.get_parameter('wait_subscribers').value)
        self.esc = preparar(cfg)
        self.planta = PlantaAdcs(cfg, self.esc.sat, self.esc.efem)
        e = self.esc
        self.get_logger().info(
            f"pase real {e.tle.nombre} (NORAD {e.tle.norad}) sobre {e.estacion.nombre}: "
            f"AOS t={e.aos_s:.0f} s ({e.pase['aos']:%Y-%m-%d %H:%M:%S} UTC), "
            f"duracion {e.pase['duracion_s']:.0f} s, el max {e.pase['el_max_deg']:.1f} deg; "
            f"fin del escenario t={e.t_fin_s:.0f} s")
        if abs(e.edad_tle_dias) > 7.0:
            self.get_logger().warn(f'el TLE tiene {e.edad_tle_dias:.1f} dias de edad en el pase: '
                                   'refrescar con scripts/actualizar_tle.sh')
        self.u = [0.0] * 3
        self.m = [0.0] * 3
        self.pend = None            # sello del estado que espera su mando
        self.t_prev_ns = None       # sello del ultimo estado (None: aun no arranca)
        self.done = False
        self.pub = self.create_publisher(AdcsState, '/atp/adcs_state', 50)
        self.ack = self.create_publisher(Clock, '/atp/step_ack', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.done_pub = self.create_publisher(Float64, '/atp/adcs_done', latched)
        self.create_subscription(AdcsCmd, '/atp/adcs_cmd', self.on_cmd, 50)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def publicar(self, stamp):
        msg = msg_de_sens(AdcsState(), self.planta.medir(), stamp)
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
        self.planta.avanzar(elapsed, self.u, self.m)
        self.t_prev_ns = s
        self.publicar(msg.clock)
        if not self.done and self.planta.t >= self.esc.t_fin_s - 1e-9:
            self.done = True
            self.done_pub.publish(Float64(data=float(self.planta.t)))   # ultimo instante del escenario
            self.get_logger().info('fin del escenario')

    def on_cmd(self, msg):
        s = sello_ns(msg.header.stamp)
        if self.pend is None or s != self.pend:
            return                                      # rancio: de otro instante
        self.pend = None
        self.u = list(msg.wheel_torque_nm)
        self.m = list(msg.mtq_dipole_am2)
        if self.t_prev_ns is None:
            self.t_prev_ns = s                          # arranque: el sello del primer mando
            self.get_logger().info('lazo cerrado: arranca el escenario')
        # se confirma el instante PARA EL QUE se calculo el mando
        self.ack.publish(Clock(clock=msg.header.stamp))


def main():
    correr(AdcsSim)
