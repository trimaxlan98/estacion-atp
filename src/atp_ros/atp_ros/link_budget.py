"""Doppler y presupuesto de enlace de bajada, en vivo durante el pase.

Entradas: /atp/sat_state (rango y velocidad radial) y /atp/pointing_error
(error de apuntamiento real), emparejados por SELLO: cada instante del pase
se calcula con el rango y el error de ese mismo instante (CONTRATO 8), asi
que la corrida es repetible. Salida: /atp/link (LinkStatus, mismo sello).
Al terminar el pase resume margen minimo, fraccion de tiempo con enlace
cerrado y Doppler. Sin lockstep (hardware real) se empareja con lo ultimo.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool

from atp_interfaces.msg import LinkStatus, PointingError, SatState
from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.qos import MEDICION
from atp_ros.params import declare_num


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class LinkBudget(Node):
    def __init__(self):
        super().__init__('link_budget')
        self.declare_parameter('band', 'S')
        declare_num(self, 'f0_hz', 0.0)               # 0: la de la banda
        declare_num(self, 'eirp_dbw', 9.0)
        declare_num(self, 'other_losses_db', 3.0)
        declare_num(self, 'bitrate_bps', 1e6)
        declare_num(self, 'ebn0_req_db', 10.5)        # QPSK, BER 1e-6
        declare_num(self, 'dish_m', model.DIAMETRO_PLATO_M)
        declare_num(self, 'eta', 0.6)
        declare_num(self, 'tsys_k', model.T_SISTEMA_K)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.band = str(g('band'))
        f0 = float(g('f0_hz'))
        self.f0 = f0 if f0 > 0 else model.BANDAS[self.band]
        self.kw = dict(f_hz=self.f0, eirp_dbw=float(g('eirp_dbw')),
                       perdidas_db=float(g('other_losses_db')),
                       tasa_bps=float(g('bitrate_bps')),
                       ebn0_req_db=float(g('ebn0_req_db')),
                       diametro_m=float(g('dish_m')), eta=float(g('eta')),
                       t_sys_k=float(g('tsys_k')))
        self.sats = {}                # sello -> SatState
        self.errs = {}                # sello -> error de apuntamiento (deg)
        self.prev = None              # (t, fd) para la tasa Doppler
        self.hist = []                # (margen, fd, tasa, l_point)
        self.finished = False
        self.cierre = None
        self.vistos = 0
        self.exactos = 0
        self.modo_ultimo = False
        self.err_ultimo = 0.0
        q = 200
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        self.create_subscription(PointingError, '/atp/pointing_error', self.on_err, MEDICION)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.pub = self.create_publisher(LinkStatus, '/atp/link', MEDICION)
        self.get_logger().info(
            f'banda {self.band} {self.f0/1e9:.3f} GHz, theta3 '
            f'{model.ancho_haz(self.kw["diametro_m"], self.f0):.3f} deg')

    def on_sat(self, msg):
        s = _sello(msg)
        en_pase = 0.0 < msg.t_since_aos_s < msg.pass_duration_s
        if not en_pase:
            return            # fuera del pase no hay error que emparejar
        self.vistos += 1
        if not self.modo_ultimo and self.vistos > 200 and self.exactos == 0:
            self.modo_ultimo = True
            self.get_logger().warn('los sellos no coinciden (sin lockstep): '
                                   'se empareja con el ultimo error')
        if self.modo_ultimo:
            self.calcular(msg, self.err_ultimo)
            return
        self.sats[s] = msg
        self.intentar(s)

    def on_err(self, msg):
        s = _sello(msg)
        self.err_ultimo = msg.e_pt_deg
        self.errs[s] = msg.e_pt_deg
        self.intentar(s)

    def intentar(self, s):
        if s in self.sats and s in self.errs:
            self.exactos += 1
            sat, err = self.sats.pop(s), self.errs.pop(s)
            for d in (self.sats, self.errs):     # lo anterior ya no se emparejara
                for k in [k for k in d if k < s - 5_000_000_000]:
                    del d[k]
            self.calcular(sat, err)

    def calcular(self, s, err):
        if self.finished:
            return
        lb = model.presupuesto_enlace(s.range_km, err, **self.kw)
        fd = float(model.doppler_hz(s.range_rate_km_s, self.f0))
        t = s.t_since_aos_s
        tasa = 0.0
        if self.prev is not None and t > self.prev[0]:
            tasa = (fd - self.prev[1]) / (t - self.prev[0])
        self.prev = (t, fd)
        m = LinkStatus()
        m.header = s.header
        m.band, m.f0_hz = self.band, self.f0
        m.doppler_hz, m.doppler_rate_hz_s = fd, float(tasa)
        m.theta3_deg = lb['theta3_deg']
        m.pointing_error_deg = float(err)
        m.l_point_db = float(lb['l_point_db'])
        m.fspl_db = float(lb['fspl_db'])
        m.g_t_db = lb['g_t_db']
        m.cn0_dbhz = float(lb['cn0_dbhz'])
        m.ebn0_db = float(lb['ebn0_db'])
        m.margin_db = float(lb['margen_db'])
        m.link_closed = m.margin_db >= 0.0
        self.pub.publish(m)
        self.hist.append((m.margin_db, fd, tasa, m.l_point_db))

    def on_done(self, msg):
        if not msg.data or self.finished or self.cierre is not None:
            return
        # como el monitor: 1 s simulado para recibir los ultimos instantes
        self.cierre = self.create_timer(1.0, self.resumen)

    def resumen(self):
        self.cierre.cancel()
        if self.finished or not self.hist:
            return
        self.finished = True
        h = np.array(self.hist)
        self.get_logger().info(
            f'ENLACE banda={self.band} | margen min={h[:,0].min():.2f} '
            f'max={h[:,0].max():.2f} dB | cerrado {100*np.mean(h[:,0] >= 0):.1f}% '
            f'| Doppler {h[:,1].max()/1e3:+.2f}..{h[:,1].min()/1e3:+.2f} kHz, '
            f'tasa max {np.abs(h[1:,2]).max() if len(h) > 1 else 0.0:.1f} Hz/s '
            f'| L_point max {h[:,3].max():.3f} dB')


def main():
    correr(LinkBudget)
