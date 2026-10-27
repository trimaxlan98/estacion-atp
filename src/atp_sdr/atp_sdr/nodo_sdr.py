"""Receptor SDR en el lazo: un bloque IQ por paso de 50 ms.

Entradas: /atp/sat_state (rango y velocidad radial) y /atp/pointing_error
(desapuntamiento real), emparejados por SELLO (CONTRATO 8): cada bloque IQ se
genera con el Doppler y el C/N0 (presupuesto de enlace, con la perdida por
desapuntamiento) de ese mismo instante. Cada bloque pasa por la cadena
`CadenaRF` (adquisicion FFT, seguimiento FLL/PLL, detector de enganche, C/N0 por
M2M4 y espectral; en modo bpsk tambien reloj de simbolo y Costas) y el estado
sale en /atp/sdr (sdr_interfaces/SdrStatus, mismo sello).

Por defecto va DENTRO del lockstep: confirma cada paso en /atp/sdr_ack y el
`sdr_gate` solo deja pasar la confirmacion de la montura (/atp/step_ack) cuando
el receptor termino el bloque de ese instante. Asi el reloj espera al receptor,
la corrida es identica aunque la maquina vaya cargada y no se pierde ningun
paso. Sin el gate (`gate:=false`, nodo PASIVO como pide el CONTRATO 8.3) un
bloque cuesta mas CPU que un paso de la montura: con el reloj a maxima velocidad
se acumula cola, las suscripciones (profundidad 30 000) aguantan un pase de 10
min, pero con la maquina cargada DDS puede perder pasos: la cadena cuenta los
huecos de sello y el resumen los declara. El fin del pase lo decide el propio nodo cuando
ha procesado todos los pasos en pase que vio (los otros nodos del lazo ya habran
terminado), y con `exit_on_done` cierra el lanzamiento tras el resumen.
Determinista: misma semilla y mismo pase, mismo resumen.
"""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock as RelojMsg
from std_msgs.msg import Bool

from atp_interfaces.msg import PointingError, SatState
from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num
from atp_sdr.cadena import CadenaRF
from sdr_interfaces.msg import SdrStatus

COLA = 30000


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def formatear_resumen(r):
    """Linea RESUMEN SDR: solo cifras deterministas (nada de tiempo de pared)."""
    ber = ('' if math.isnan(r['ber'])
           else f" | BER={r['ber']:.2e} ({r['ber_n']} bits)")
    return (f"RESUMEN SDR banda={r['banda']} modo={r['modo']} fs={r['fs_hz']/1e3:.0f} kHz | "
            f"bloques={r['bloques']} huecos={r['huecos']} | Doppler rms="
            f"{r['doppler_rms_hz']:.3f} Hz sesgo={r['doppler_sesgo_hz']:+.3f} Hz "
            f"max={r['doppler_max_hz']:.2f} Hz | C/N0 medido-presupuesto sesgo="
            f"{r['cn0_sesgo_db']:+.3f} dB rms={r['cn0_rms_db']:.3f} dB | enganchado "
            f"{r['enganchado_pct']:.1f}% (adquisiciones={r['adquisiciones']} "
            f"perdidas={r['perdidas']}){ber}")


class SdrReceiver(Node):
    def __init__(self):
        super().__init__('sdr_receiver')
        self.declare_parameter('band', 'S')
        self.declare_parameter('mode', 'portadora')
        self.declare_parameter('seed', 1)
        self.declare_parameter('exit_on_done', False)
        declare_num(self, 'f0_hz', 0.0)                 # 0: la de la banda
        declare_num(self, 'order', 3)
        declare_num(self, 'bn_hz', 0.0)                 # 0: segun la banda
        declare_num(self, 'bn_fll_hz', 10.0)
        declare_num(self, 'tint_s', 1e-3)
        declare_num(self, 'offset_lo_hz', 0.0)
        declare_num(self, 'lw_lo_hz', 0.0)
        declare_num(self, 'drift_lo_hz_s', 0.0)
        declare_num(self, 'delta_cn0_db', 0.0)          # degrada el enlace
        declare_num(self, 'symbol_rate', 12.5e3)
        declare_num(self, 'h_km', 550.0)
        # el MISMO presupuesto que el nodo link_budget
        declare_num(self, 'eirp_dbw', 9.0)
        declare_num(self, 'other_losses_db', 3.0)
        declare_num(self, 'dish_m', model.DIAMETRO_PLATO_M)
        declare_num(self, 'eta', 0.6)
        declare_num(self, 'tsys_k', model.T_SISTEMA_K)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.exit_on_done = bool(g('exit_on_done'))
        f0 = float(g('f0_hz'))
        self.cad = CadenaRF(
            str(g('band')), str(g('mode')), int(g('seed')), dt_s=0.05,
            orden=int(g('order')), bn_hz=float(g('bn_hz')) or None,
            bn_fll_hz=float(g('bn_fll_hz')), t_int_s=float(g('tint_s')),
            offset_lo_hz=float(g('offset_lo_hz')), lw_lo_hz=float(g('lw_lo_hz')),
            deriva_lo_hz_s=float(g('drift_lo_hz_s')),
            delta_cn0_db=float(g('delta_cn0_db')), eirp_dbw=float(g('eirp_dbw')),
            otras_perdidas_db=float(g('other_losses_db')), dish_m=float(g('dish_m')),
            eta=float(g('eta')), tsys_k=float(g('tsys_k')),
            tasa_simbolos=float(g('symbol_rate')), h_km=float(g('h_km')),
            f0_hz=f0 if f0 > 0 else None)
        self.sats = {}                # sello -> SatState (en pase)
        self.errs = {}                # sello -> error de apuntamiento (deg)
        self.n_esperados = 0          # pasos en pase vistos en /atp/sat_state
        self.n_procesados = 0
        self.fin_visto = False        # llego un estado ya fuera del pase
        self.done = False
        self.finished = False
        self.t_actividad = time.monotonic()
        q = QoSProfile(depth=COLA, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE)
        qm = QoSProfile(depth=COLA, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        self.create_subscription(PointingError, '/atp/pointing_error', self.on_err, qm)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.pub = self.create_publisher(SdrStatus, '/atp/sdr', qm)
        self.ack_pub = self.create_publisher(RelojMsg, '/atp/sdr_ack', q)
        # de PARED: con el reloj parado (compuerta) o a maxima velocidad, el
        # tiempo simulado no sirve para vigilar la cola
        self.create_timer(0.5, self.vigilar, clock=Clock(clock_type=ClockType.STEADY_TIME))
        c = self.cad
        self.get_logger().info(
            f'SDR banda {c.banda} {c.f0/1e9:.3f} GHz modo {c.modo}: fs {c.fs/1e3:.0f} kHz, '
            f'{c.n_blk} muestras por paso, ventana Doppler +-{c.ventana/1e3:.1f} kHz, '
            f'lazo orden {c.seg.orden} B_L {c.seg.bn:.0f} Hz T_int {c.t_int*1e3:g} ms')

    def on_sat(self, msg):
        if self.finished:
            self.ack(msg.header.stamp)
            return
        if 0.0 < msg.t_since_aos_s < msg.pass_duration_s:
            self.n_esperados += 1
            self.sats[_sello(msg)] = msg
            self.intentar(_sello(msg))
        else:
            if self.n_esperados > 0:
                self.fin_visto = True
            self.ack(msg.header.stamp)       # fuera del pase no hay bloque que esperar

    def on_err(self, msg):
        if self.finished:
            return
        s = _sello(msg)
        self.errs[s] = msg.e_pt_deg
        self.intentar(s)

    def on_done(self, msg):
        if msg.data:
            self.done = True

    def intentar(self, s):
        if s in self.sats and s in self.errs:
            self.procesar(self.sats.pop(s), self.errs.pop(s))

    def procesar(self, sat, err):
        m = self.cad.paso(sat.t_since_aos_s, sat.range_km, sat.range_rate_km_s, err)
        self.n_procesados += 1
        self.t_actividad = time.monotonic()
        out = SdrStatus()
        out.header = sat.header
        out.band, out.mode = self.cad.banda, self.cad.modo
        out.f0_hz, out.fs_hz, out.n_samples = self.cad.f0, self.cad.fs, m['n_samples']
        out.doppler_true_hz = m['doppler_true_hz']
        out.doppler_est_hz = m['doppler_est_hz']
        out.doppler_err_hz = m['doppler_err_hz']
        out.cn0_budget_dbhz = m['cn0_budget_dbhz']
        out.cn0_est_dbhz = m['cn0_est_dbhz']
        out.cn0_m2m4_dbhz = m['cn0_m2m4_dbhz']
        out.cn0_spec_dbhz = m['cn0_spec_dbhz']
        out.amp_est = m['amp_est']
        out.lock_metric = m['lock_metric']
        out.state = int(m['estado'])
        out.locked = bool(m['locked'])
        out.ber = m['ber']
        out.proc_ms = m['proc_ms']
        self.pub.publish(out)
        self.ack(sat.header.stamp)
        self.intentar_fin()

    def ack(self, stamp):
        """Confirma el paso al `sdr_gate` (si lo hay): el reloj no avanza hasta
        que el receptor termino el bloque de ese instante."""
        c = RelojMsg()
        c.clock = stamp
        self.ack_pub.publish(c)

    def vigilar(self):
        """Cierra el pase cuando se proceso todo lo visto, o por inactividad (un
        paso sin pareja no debe colgar el lanzamiento)."""
        if self.finished or not (self.fin_visto or self.done):
            return
        if self.n_procesados >= self.n_esperados:
            self.resumen()
        elif time.monotonic() - self.t_actividad > 20.0:
            self.get_logger().warn(
                f'SDR: {self.n_esperados - self.n_procesados} pasos sin pareja; se cierra')
            self.resumen()

    def intentar_fin(self):
        if self.fin_visto and self.n_procesados >= self.n_esperados:
            self.resumen()

    def resumen(self):
        if self.finished:
            return
        self.finished = True
        r = self.cad.resumen()
        if r['bloques'] == 0:
            self.get_logger().error('SDR: sin pasos en pase')
        else:
            self.get_logger().info(formatear_resumen(r))
            self.get_logger().info(
                f"RENDIMIENTO SDR (CPU del hilo, no determinista) | receptor "
                f"{r['ms_por_bloque_rx']:.2f} ms/bloque = {r['muestras_por_s_rx']/1e6:.2f} MS/s, "
                f"generador {r['ms_por_bloque_gen']:.2f} ms/bloque, cadena "
                f"{r['muestras_por_s_total']/1e6:.2f} MS/s contra {r['fs_hz']/1e6:.2f} MS/s "
                f"de tiempo real (margen x{r['muestras_por_s_total']/r['fs_hz']:.1f})")
        if self.exit_on_done:
            raise SystemExit(0)


def main():
    correr(SdrReceiver)
