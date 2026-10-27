"""Mide el error de apuntamiento contra el presupuesto de 0.1 deg.

Publica /atp/error_deg = [e_az, e_el, e_apuntamiento] y, al terminar el pase,
imprime el veredicto. El error de apuntamiento es la separacion angular real
en el cielo: sqrt((e_az cos el)^2 + e_el^2).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from atp_interfaces.msg import PointingError, SatState

from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.qos import MEDICION
from atp_ros.params import declare_num


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class ErrorMonitor(Node):
    def __init__(self):
        super().__init__('error_monitor')
        declare_num(self, 'objective_deg', model.OBJETIVO_DEG)
        declare_num(self, 'rate_hz', 20.0)
        self.declare_parameter('exit_on_done', False)
        # mide contra el angulo REAL de la carga (/atp/mount_truth), no contra
        # el encoder (que tiene ruido, cuantizacion y backlash invisible)
        self.declare_parameter('use_truth', False)
        # keyhole: contra QUE se mide
        # (con una estrategia de keyhole la referencia es la orden a las
        # articulaciones y la verdad sale por /atp/truth), y como: `esferico`
        # usa la separacion angular exacta en la esfera; `montura_sobre_cenit`
        # convierte una articulacion con el' > 90 a la direccion del cielo
        self.declare_parameter('ref_topic', '/atp/reference')
        self.declare_parameter('esferico', False)
        self.declare_parameter('montura_sobre_cenit', False)
        self.obj = float(self.get_parameter('objective_deg').value)
        self.exit_on_done = bool(self.get_parameter('exit_on_done').value)

        self.samples = []      # (e_az, e_el, e_point) en grados
        self.finished = False

        q = 200   # el monitor es pasivo: el reloj no lo espera, que no pierda nada
        self.create_subscription(JointState, str(self.get_parameter('ref_topic').value),
                                 self.on_ref, q)
        self.esferico = bool(self.get_parameter('esferico').value)
        self.sobre_cenit = bool(self.get_parameter('montura_sobre_cenit').value)
        self.use_truth = bool(self.get_parameter('use_truth').value)
        self.create_subscription(
            JointState, '/atp/mount_truth' if self.use_truth else '/joint_states',
            self.on_meas, q)
        self.create_subscription(SatState, '/atp/sat_state', self.on_sat, q)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/atp/pass_done', self.on_done, latched)
        self.pub = self.create_publisher(Float64MultiArray, '/atp/error_deg', q)
        # la misma cifra con su sello, para emparejar por instante (CONTRATO 8)
        self.pe_pub = self.create_publisher(PointingError, '/atp/pointing_error', MEDICION)
        # Se mide cada INSTANTE (sello) cuando estan sus tres piezas: medida,
        # referencia y estado del satelite. Solo cuenta lo que pasa ENTRE AOS
        # y LOS: la espera previa y el transitorio tras LOS dependen del
        # arranque, no de la estacion. Si los sellos nunca coinciden (banco
        # sin lockstep, p. ej. un rotor real) se pasa a "ultimo disponible".
        self.buf = {'ref': {}, 'meas': {}, 'sat': {}}
        self.exactos = 0
        self.vistos = 0
        self.modo_ultimo = False
        self.ref_ultima = None
        self.sat_ultimo = None
        self.cierre = None

    def _guardar(self, clave, s, valor):
        b = self.buf[clave]
        b[s] = valor
        if len(b) > 400:
            b.pop(min(b))
        self.intentar(s)

    def on_ref(self, msg):
        self.ref_ultima = np.array(msg.position[:2])
        self._guardar('ref', _sello(msg), self.ref_ultima)

    def on_sat(self, msg):
        self.sat_ultimo = 0.0 < msg.t_since_aos_s < msg.pass_duration_s
        self._guardar('sat', _sello(msg), self.sat_ultimo)

    def on_meas(self, msg):
        meas = np.array(msg.position[:2])
        self.vistos += 1
        if not self.modo_ultimo and self.vistos > 200 and self.exactos == 0:
            self.modo_ultimo = True
            self.get_logger().warn('los sellos no coinciden (sin lockstep): '
                                   'se empareja con la ultima referencia')
        if self.modo_ultimo:
            if self.ref_ultima is not None and self.sat_ultimo is not False:
                self.medir(meas, self.ref_ultima, msg.header.stamp)
            return
        self._guardar('meas', _sello(msg), meas)

    def intentar(self, s):
        b = self.buf
        if s in b['meas'] and s in b['ref'] and s in b['sat']:
            self.exactos += 1
            meas, ref, en_pase = b['meas'].pop(s), b['ref'].pop(s), b['sat'].pop(s)
            if en_pase:
                self.medir(meas, ref, s)

    def medir(self, meas, ref, sello):
        if self.finished:
            return
        if self.esferico or self.sobre_cenit:
            e, e_pt = self._error_cielo(meas, ref)
        else:
            e = np.degrees(ref - meas)
            e_pt = math.hypot(e[0] * math.cos(ref[1]), e[1])
        self.samples.append((float(e[0]), float(e[1]), e_pt))
        self.pub.publish(Float64MultiArray(data=[float(e[0]), float(e[1]), e_pt]))
        pe = PointingError()
        if isinstance(sello, int):
            pe.header.stamp.sec, pe.header.stamp.nanosec = divmod(sello, 1_000_000_000)
        else:
            pe.header.stamp = sello
        pe.e_az_deg, pe.e_el_deg, pe.e_pt_deg = float(e[0]), float(e[1]), e_pt
        self.pe_pub.publish(pe)

    def _error_cielo(self, meas, ref):
        """Error en el cielo de la montura (articulaciones `meas`, rad) con
        respecto a la direccion verdadera `ref` (rad): con `montura_sobre_cenit`
        el' > 90 grados es la direccion (az' + 180, 180 - el'); la separacion es
        la distancia angular exacta (`keyhole.separacion_deg`) y las
        componentes se reportan con el acimut envuelto a [-180, 180]."""
        from atp_ros import keyhole
        az_m, el_m = (keyhole.a_cielo(math.degrees(meas[0]), math.degrees(meas[1]))
                      if self.sobre_cenit else
                      (math.degrees(meas[0]), math.degrees(meas[1])))
        az_v, el_v = math.degrees(ref[0]), math.degrees(ref[1])
        e_az = (az_v - float(az_m) + 180.0) % 360.0 - 180.0
        e_el = el_v - float(el_m)
        e_pt = float(keyhole.separacion_deg(az_v, el_v, float(az_m), float(el_m)))
        return np.array([e_az, e_el]), e_pt

    def on_done(self, msg):
        if not msg.data or self.finished or self.cierre is not None:
            return
        # el aviso de LOS puede adelantarse a los ultimos instantes del pase
        # (otro topico): se espera 1 s SIMULADO antes de cerrar la cuenta
        self.cierre = self.create_timer(1.0, self.resumen)

    def resumen(self):
        self.cierre.cancel()
        if self.finished:
            return
        self.finished = True
        s = np.array(self.samples)
        if len(s) == 0:
            self.get_logger().error('sin muestras')
            return
        pt = s[:, 2]
        dentro = float(np.mean(pt <= self.obj)) * 100.0
        self.get_logger().info(
            f'RESULTADO n={len(s)} | az: rms={np.sqrt(np.mean(s[:,0]**2)):.4f} '
            f'max={np.max(np.abs(s[:,0])):.4f} | el: rms={np.sqrt(np.mean(s[:,1]**2)):.4f} '
            f'max={np.max(np.abs(s[:,1])):.4f} | apuntamiento: '
            f'rms={np.sqrt(np.mean(pt**2)):.4f} p95={np.percentile(pt,95):.4f} '
            f'max={np.max(pt):.4f} deg | dentro de {self.obj} deg: {dentro:.1f}%')
        if self.exit_on_done:
            raise SystemExit(0)


def main():
    correr(ErrorMonitor)
