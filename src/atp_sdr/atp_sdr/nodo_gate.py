"""Compuerta del reloj: el paso k solo se confirma cuando el receptor SDR lo termino.

mount_sim confirma cada paso en /atp/step_ack y sim_clock espera esa
confirmacion para publicar el instante siguiente (CONTRATO 8). Con este nodo,
el launch redirige la salida de la montura a /atp/step_ack_raw y esta compuerta
reemite /atp/step_ack (rosgraph_msgs/Clock, mismo sello) solo cuando tambien
llego /atp/sdr_ack de `sdr_receiver` para ese mismo sello. Es la misma regla de
"emparejar por sello" del contrato: el paso k se cierra con la montura Y el
receptor.

Sin ciclos: el bloque IQ de t_k necesita /atp/sat_state y /atp/pointing_error
con sello t_k, que existen antes de que se confirme el paso k (el error lo
calcula el monitor con la medida, la referencia y el estado de ese instante).

Red de seguridad (de PARED, solo contra interbloqueos): si una confirmacion de
la montura espera mas de `timeout_s` sin la del receptor, se reemite igual y se
avisa; la corrida deja de ser repetible pero no se cuelga.
"""
import time

from rclpy.clock import Clock as RelojROS, ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from atp_ros.nodo import correr
from atp_ros.params import declare_num


def _sello(msg):
    return msg.clock.sec * 1_000_000_000 + msg.clock.nanosec


class SdrGate(Node):
    def __init__(self):
        super().__init__('sdr_gate')
        declare_num(self, 'timeout_s', 20.0)
        self.timeout = float(self.get_parameter('timeout_s').value)
        self.raw = {}              # sello -> (mensaje, instante de pared)
        self.sdr = set()
        self.reenviados = 0
        self.forzados = 0
        self.pub = self.create_publisher(Clock, '/atp/step_ack', 10)
        self.create_subscription(Clock, '/atp/step_ack_raw', self.on_raw, 200)
        self.create_subscription(Clock, '/atp/sdr_ack', self.on_sdr, 30000)
        self.create_timer(1.0, self.vigilar,
                          clock=RelojROS(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('compuerta del reloj: el paso espera al receptor SDR')

    def on_raw(self, msg):
        s = _sello(msg)
        self.raw[s] = (msg, time.monotonic())
        self.intentar(s)

    def on_sdr(self, msg):
        s = _sello(msg)
        self.sdr.add(s)
        self.intentar(s)

    def intentar(self, s):
        if s in self.raw and s in self.sdr:
            msg, _ = self.raw.pop(s)
            self.sdr.discard(s)
            self.reenviados += 1
            self.pub.publish(msg)

    def vigilar(self):
        ahora = time.monotonic()
        for s, (msg, t) in list(self.raw.items()):
            if ahora - t > self.timeout:
                self.raw.pop(s)
                self.forzados += 1
                self.get_logger().warn(
                    f'sin confirmacion del receptor SDR para el paso {s * 1e-9:.2f} s '
                    f'tras {self.timeout:g} s: se reemite igual (no repetible)')
                self.pub.publish(msg)
        # los sellos del receptor que ya no se emparejaran (la montura no los
        # confirma) no deben crecer sin limite
        if len(self.sdr) > 5000:
            for s in sorted(self.sdr)[:len(self.sdr) - 2000]:
                self.sdr.discard(s)


def main():
    correr(SdrGate)
