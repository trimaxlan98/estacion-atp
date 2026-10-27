"""El beacon con la potencia medida sobre la AMPLITUD del bloque IQ.

Reemplaza a `rf_beacon` (atp_ros) en el lazo de autotrack (launch
sdr_autotrack.launch.py, `beacon:=sdr`): misma verdad (el
satelite esta `ephem_dt_s` adelantado y desplazado una constante) y misma
interfaz (/atp/truth y /atp/beacon con position = [P_dB, fase_scan]), pero P_dB
ya no es la potencia ideal en dB del lobulo gaussiano mas un ruido gaussiano
inventado (`noise_db`): sale de un bloque IQ real.

Por paso: la potencia del conical scan P_scan(dx, dy, fase) (dB bajo el pico)
se suma al C/N0 del presupuesto de enlace del rango de ese instante, el
`CadenaRF` genera el bloque IQ (portadora con el Doppler del pase, ruido AWGN
de piso fijo), la estacion sintoniza su oscilador al Doppler PREDICHO por la
efemeride (`fd_pred_hz`: queda solo el residuo, fs de unos 20 kHz) y de la
amplitud medida, A^2 = <|x|^2> - sigma^2 (el piso sigma^2 leido del espectro),
sale P_dB = 10 log10(A^2 fs / C/N0_pico): potencia relativa al pico, como la de
rf_beacon. Solo importa su variacion dentro de la vuelta del scan (el
discriminador resta la media) y su nivel para el umbral de enganche.

Limitacion: el bloque IQ de 50 ms tiene amplitud constante (la del instante del
sello), es decir, la modulacion del scan se muestrea una vez por bloque, como
en rf_beacon; un receptor real veria la amplitud variar DENTRO del bloque.
"""
import math

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from atp_interfaces.msg import SatState
from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num
from atp_sdr.cadena import CadenaRF
from atp_sdr.senal import doppler_de_vr


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class SdrBeacon(Node):
    def __init__(self):
        super().__init__('sdr_beacon')
        self.declare_parameter('band', 'Ka')
        declare_num(self, 'dish_m', model.DIAMETRO_PLATO_M)
        declare_num(self, 'ephem_dt_s', 0.1)
        declare_num(self, 'bias_xel_deg', 0.10)
        declare_num(self, 'bias_el_deg', -0.08)
        declare_num(self, 'scan_hz', 2.5)
        declare_num(self, 'scan_frac', 0.2)
        declare_num(self, 'seed', 5)
        declare_num(self, 'fs_hz', 20e3)             # solo el residuo de Doppler
        declare_num(self, 'window_hz', 4000.0)
        declare_num(self, 'order', 3)
        declare_num(self, 'bn_hz', 0.0)
        declare_num(self, 'delta_cn0_db', 0.0)       # degrada el enlace
        declare_num(self, 'eirp_dbw', 9.0)
        declare_num(self, 'other_losses_db', 3.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        banda = str(g('band'))
        f0 = model.BANDAS[banda]
        self.th3 = model.ancho_haz(float(g('dish_m')), f0)
        self.g_max = model.ganancia_plato(float(g('dish_m')), f0)
        self.r = float(g('scan_frac')) * self.th3
        self.dt_err = float(g('ephem_dt_s'))
        self.bias = (float(g('bias_xel_deg')), float(g('bias_el_deg')))
        self.w = 2.0 * math.pi * float(g('scan_hz'))
        self.cad = CadenaRF(
            banda, 'portadora', int(g('seed')), fs_hz=float(g('fs_hz')),
            ventana_hz=float(g('window_hz')), orden=int(g('order')),
            bn_hz=float(g('bn_hz')) or None, delta_cn0_db=float(g('delta_cn0_db')),
            eirp_dbw=float(g('eirp_dbw')), otras_perdidas_db=float(g('other_losses_db')),
            dish_m=float(g('dish_m')))
        self.prev = None          # (t, az, el) de la efemeride, en grados
        self.rate = (0.0, 0.0)
        self.truth = None
        self.vr_prev = None       # (t, vr) para extrapolar la verdad
        self.truths, self.meass, self.sats = {}, {}, {}
        self.err_p = []           # error de la potencia medida (dB) contra la ideal
        self.n_bloq = 0
        self.fin = False
        self.create_subscription(JointState, '/atp/program', self.on_program, 200)
        self.create_subscription(JointState, '/joint_states', self.on_meas, 200)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, 200)
        self.truth_pub = self.create_publisher(JointState, '/atp/truth', 10)
        self.beacon_pub = self.create_publisher(JointState, '/atp/beacon', 200)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        c = self.cad
        self.get_logger().info(
            f'beacon IQ banda {banda}: theta3={self.th3:.3f} deg, scan r={self.r:.3f} deg, '
            f'fs {c.fs/1e3:.0f} kHz, ventana +-{c.ventana/1e3:.1f} kHz, error de '
            f'efemeride {self.dt_err:g} s + sesgo {self.bias} deg')

    def on_program(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        az, el = (math.degrees(x) for x in msg.position[:2])
        if self.prev is not None and t - self.prev[0] > 1e-3:
            dt = t - self.prev[0]
            a = 0.3
            self.rate = (self.rate[0] + a * ((az - self.prev[1]) / dt - self.rate[0]),
                         self.rate[1] + a * ((el - self.prev[2]) / dt - self.rate[1]))
        if self.prev is None or t - self.prev[0] > 1e-3:
            self.prev = (t, az, el)
        daz, dele = self.rate
        cos_el = max(math.cos(math.radians(el)), 1e-3)
        t_az = az + daz * self.dt_err + self.bias[0] / cos_el
        t_el = el + dele * self.dt_err + self.bias[1]
        self.truth = (t_az, t_el)
        s = _sello(msg)
        self.truths[s] = self.truth
        self._muestrear(s)
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = ['az', 'el']
        out.position = [math.radians(t_az), math.radians(t_el)]
        self.truth_pub.publish(out)

    def on_meas(self, msg):
        s = _sello(msg)
        self.meass[s] = [math.degrees(x) for x in msg.position[:2]]
        self._muestrear(s)

    def on_sat(self, msg):
        s = _sello(msg)
        self.sats[s] = msg
        self._muestrear(s)

    def on_done(self, msg):
        if msg.data:
            self.fin = True

    def _muestrear(self, s):
        if s not in self.truths or s not in self.meass or s not in self.sats:
            return
        truth, meas, sat = self.truths.pop(s), self.meass.pop(s), self.sats.pop(s)
        for d in (self.truths, self.meass, self.sats):
            for k in [k for k in d if k < s]:
                del d[k]
        fase = (self.w * s * 1e-9) % (2.0 * math.pi)
        el = math.radians(meas[1])
        dx = (truth[0] - meas[0]) * math.cos(el)
        dy = truth[1] - meas[1]
        p_ideal = max(float(model.potencia_conscan(dx, dy, fase, self.r, self.th3)),
                      -self.g_max)
        # velocidad radial VERDADERA: la de la efemeride extrapolada ephem_dt_s
        vr = sat.range_rate_km_s
        t = s * 1e-9
        vr_true = vr
        if self.vr_prev is not None and t - self.vr_prev[0] > 1e-3:
            vr_true = vr + (vr - self.vr_prev[1]) / (t - self.vr_prev[0]) * self.dt_err
        self.vr_prev = (t, vr)
        pred = float(doppler_de_vr(vr, self.cad.f0))
        m = self.cad.paso(t, sat.range_km, vr_true, 0.0, extra_db=p_ideal,
                          fd_pred_hz=pred)
        cn0_pico = float(model.presupuesto_enlace(sat.range_km, 0.0, **self.cad.kw)
                         ['cn0_dbhz']) + self.cad.delta_cn0
        amp = m['amp_est']
        # solo con la portadora ENGANCHADA: sin enganche la 'amplitud' es el
        # ruido del piso, no una medida de la senal
        if m['locked'] and amp == amp and amp > 0.0:
            p_med = 20.0 * math.log10(amp) - (cn0_pico - 10.0 * math.log10(self.cad.fs))
        else:
            p_med = -60.0                 # sin senal: el autotrack no engancha
        self.n_bloq += 1
        if p_med > -30.0:
            self.err_p.append(p_med - p_ideal)
        out = JointState()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(s, 1_000_000_000)
        out.name = ['p_db', 'fase']
        out.position = [p_med, fase]
        self.beacon_pub.publish(out)
        if self.fin and not hasattr(self, '_cierre'):
            self._cierre = self.create_timer(0.5, self.resumen)   # antes del cierre del monitor (1 s)

    def resumen(self):
        self._cierre.cancel()
        e = np.array(self.err_p)
        r = self.cad.resumen()
        if len(e):
            self.get_logger().info(
                f'BEACON IQ bloques={self.n_bloq} | potencia medida contra ideal: '
                f'sesgo={e.mean():+.4f} dB rms={np.sqrt(np.mean(e ** 2)):.4f} dB '
                f'(rf_beacon por defecto: ruido gaussiano de 0.2 dB) | Doppler residuo '
                f'rms={r["doppler_rms_hz"]:.3f} Hz | enganchado {r["enganchado_pct"]:.1f}%')


def main():
    correr(SdrBeacon)
