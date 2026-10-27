"""El cielo verdadero y la senal RF del satelite.

La referencia de program track (/atp/program) sale de una efemeride que
nunca es perfecta. Este nodo fabrica la VERDAD: el satelite real esta
adelantado `ephem_dt_s` segundos a lo largo de su traza (TLE envejecido) y
desplazado una constante (desalineacion de la montura, cero del encoder).

Publica:
  /atp/truth   JointState: donde esta de verdad el satelite (rad)
  /atp/beacon  JointState sellado, position = [P_dB, fase_scan_rad]: lo que
               recibe el alimentador que nuta (conical scan) apuntado donde
               esta la montura. Una muestra por instante del lockstep, con la
               verdad y la medida del MISMO sello (CONTRATO 8).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num


class RfBeacon(Node):
    def __init__(self):
        super().__init__('rf_beacon')
        self.declare_parameter('band', 'Ka')
        declare_num(self, 'dish_m', model.DIAMETRO_PLATO_M)
        declare_num(self, 'ephem_dt_s', 0.1)        # adelanto de la verdad
        declare_num(self, 'bias_xel_deg', 0.10)     # sesgo cross-elevacion
        declare_num(self, 'bias_el_deg', -0.08)
        declare_num(self, 'scan_hz', 2.5)        # 8 muestras por vuelta a 20 Hz
        declare_num(self, 'scan_frac', 0.2)          # radio del scan / theta3
        declare_num(self, 'noise_db', 0.2)
        declare_num(self, 'seed', 5)
        # verdad EXTERNA (mision integrada): el satelite verdadero llega por /atp/truth
        # (lo publica reference_generator con truth_file); no se sintetiza a partir
        # de la efemeride y este nodo no la publica
        self.declare_parameter('external_truth', False)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        f0 = model.BANDAS[str(g('band'))]
        self.th3 = model.ancho_haz(float(g('dish_m')), f0)
        self.g_max = model.ganancia_plato(float(g('dish_m')), f0)
        self.r = float(g('scan_frac')) * self.th3
        self.dt_err = float(g('ephem_dt_s'))
        self.bias = (float(g('bias_xel_deg')), float(g('bias_el_deg')))
        self.w = 2.0 * math.pi * float(g('scan_hz'))
        self.noise = float(g('noise_db'))
        self.rng = np.random.default_rng(int(g('seed')))
        self.prev = None          # (t, az, el) de la efemeride, en grados
        self.rate = (0.0, 0.0)    # velocidad filtrada (grados/s)
        self.truth = None         # (az, el) grados
        self.externa = bool(g('external_truth'))
        if self.externa:
            self.create_subscription(JointState, '/atp/truth', self.on_truth_ext, 200)
            self.truth_pub = None
        else:
            self.create_subscription(JointState, '/atp/program', self.on_program, 10)
            self.truth_pub = self.create_publisher(JointState, '/atp/truth', 10)
        self.create_subscription(JointState, '/joint_states', self.on_meas, 200)
        self.beacon_pub = self.create_publisher(JointState, '/atp/beacon', 200)
        self.truths = {}          # sello -> (az, el) verdad, grados
        self.meass = {}           # sello -> (az, el) medida, grados
        self.get_logger().info(
            f'theta3={self.th3:.3f} deg, scan r={self.r:.3f} deg '
            f'(cruce {model.perdida_cruce_conscan(self.r, self.th3):.2f} dB), '
            + ('verdad externa (/atp/truth)' if self.externa else
               f'error de efemeride {self.dt_err:g} s + sesgo {self.bias} deg'))

    def on_program(self, msg):
        # La velocidad de la efemeride se deriva con el sello del EMISOR, no
        # con el reloj de este nodo: con el reloj acelerado dos mensajes
        # pueden llegar casi juntos y la derivada explotaria.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        az, el = (math.degrees(x) for x in msg.position[:2])
        if self.prev is not None and t - self.prev[0] > 1e-3:
            dt = t - self.prev[0]
            a = 0.3   # filtro paso bajo de la velocidad
            self.rate = (self.rate[0] + a * ((az - self.prev[1]) / dt - self.rate[0]),
                         self.rate[1] + a * ((el - self.prev[2]) / dt - self.rate[1]))
        if self.prev is None or t - self.prev[0] > 1e-3:
            self.prev = (t, az, el)
        daz, dele = self.rate
        cos_el = max(math.cos(math.radians(el)), 1e-3)
        t_az = az + daz * self.dt_err + self.bias[0] / cos_el
        t_el = el + dele * self.dt_err + self.bias[1]
        self.truth = (t_az, t_el)
        s = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.truths[s] = self.truth
        self._muestrear(s)
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = ['az', 'el']
        out.position = [math.radians(t_az), math.radians(t_el)]
        self.truth_pub.publish(out)

    def on_truth_ext(self, msg):
        s = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.truths[s] = tuple(math.degrees(x) for x in msg.position[:2])
        self._muestrear(s)

    def on_meas(self, msg):
        s = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.meass[s] = [math.degrees(x) for x in msg.position[:2]]
        self._muestrear(s)

    def _muestrear(self, s):
        if s not in self.truths or s not in self.meass:
            return
        truth, meas = self.truths.pop(s), self.meass.pop(s)
        for d in (self.truths, self.meass):      # lo mas viejo ya no se emparejara
            for k in [k for k in d if k < s]:
                del d[k]
        # la fase del scan es funcion del tiempo simulado: deterministica
        fase = (self.w * s * 1e-9) % (2.0 * math.pi)
        el = math.radians(meas[1])
        dx = (truth[0] - meas[0]) * math.cos(el)
        dy = truth[1] - meas[1]
        p = float(model.potencia_conscan(dx, dy, fase, self.r, self.th3))
        # fuera del lobulo principal: solo lobulos laterales
        p = max(p, -self.g_max) + self.rng.normal(0.0, self.noise)
        out = JointState()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(s, 1_000_000_000)
        out.name = ['p_db', 'fase']
        out.position = [p, fase]
        self.beacon_pub.publish(out)


def main():
    correr(RfBeacon)
