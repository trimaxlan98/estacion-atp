"""PID digital por eje: derivada filtrada, saturacion y anti-windup por clamping.

En lockstep: calcula cuando tiene estado y referencia con el MISMO sello de
tiempo (nada de emparejar la medida de un instante con la referencia de
otro, que mete un sesgo de v*dt) y responde a cada estado con un mando,
que es lo que la montura confirma para que el reloj avance. Si las
referencias no traen el mismo sello (bancos sin lockstep) usa la ultima.

Opcional (`estimator:=kalman`, apagado por defecto: el PID de siempre queda
igual): el error y su derivada salen de un filtro de Kalman [theta, omega,
par de sesgo] en vez de diferenciar la medida; sirve con encoder cuantizado.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num


def _sello(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class PidController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        declare_num(self, 'kp', 25.0)
        declare_num(self, 'ki', 2.0)
        declare_num(self, 'kd', 8.0)
        declare_num(self, 'tau_derivada', 0.08)
        declare_num(self, 'u_max', 8.0)
        self.declare_parameter('antiwindup', True)
        declare_num(self, 'rate_hz', 20.0)
        # estimador opcional (por defecto none: sin cambios respecto al PID clasico)
        self.declare_parameter('estimator', 'none')       # none | kalman
        declare_num(self, 'J', model.J_EJE)
        declare_num(self, 'b', model.B_EJE)
        declare_num(self, 'enc_sigma_deg', 0.0)
        declare_num(self, 'enc_bits', 0)
        declare_num(self, 'kf_q_acc', 1.0e-4)
        declare_num(self, 'kf_q_par', 1.0e-2)

        p = self.get_parameter
        self.kp, self.ki, self.kd = (float(p(k).value)
                                     for k in ('kp', 'ki', 'kd'))
        self.tau = float(p('tau_derivada').value)
        self.u_max = float(p('u_max').value)
        self.aw = bool(p('antiwindup').value)
        self.dt = 1.0 / float(p('rate_hz').value)
        self.dt_ns = int(round(self.dt * 1e9))

        self.refs = {}                 # sello -> referencia (rad)
        self.ref_ultima = None
        self.meas_pend = None          # (sello, medida) esperando su referencia
        self.t_ultimo = None           # sello del ultimo calculo
        self.u = np.zeros(2)
        self.integral = np.zeros(2)
        self.e_prev = np.zeros(2)
        self.d_filt = np.zeros(2)
        self.pares = {'exacto': 0, 'respaldo': 0}   # calidad del lockstep
        self.kalman = str(p('estimator').value) == 'kalman'
        if self.kalman:
            from atp_ros import control
            self._control = control
            self.est = None
            self.dif = control.DiferenciadorRef(self.dt)
            self.pj = (float(p('J').value), float(p('b').value),
                       math.radians(float(p('enc_sigma_deg').value)),
                       int(p('enc_bits').value),
                       {'q_acc': float(p('kf_q_acc').value),
                        'q_par': float(p('kf_q_par').value)})

        self.create_subscription(JointState, '/atp/reference', self.on_ref, 10)
        self.create_subscription(JointState, '/joint_states', self.on_meas, 10)
        self.pub = self.create_publisher(JointState, '/atp/torque', 10)

    def on_ref(self, msg):
        s = _sello(msg)
        self.ref_ultima = np.array(msg.position[:2])
        self.refs[s] = self.ref_ultima
        if len(self.refs) > 64:
            self.refs.pop(min(self.refs))
        if self.meas_pend is not None and self.meas_pend[0] == s:
            self.pares['exacto'] += 1
            self.control(*self.meas_pend, self.ref_ultima)

    def on_meas(self, msg):
        s, meas = _sello(msg), np.array(msg.position[:2])
        if s in self.refs:
            self.pares['exacto'] += 1
            self.control(s, meas, self.refs[s])
        elif self.ref_ultima is not None and self.refs and max(self.refs) > s:
            # la referencia de este instante ya no llegara: usar la ultima
            self.pares['respaldo'] += 1
            self._avisar(s, 'referencia mas nueva que el estado')
            self.control(s, meas, self.ref_ultima)
        else:
            if self.meas_pend is not None and self.ref_ultima is not None \
                    and self.meas_pend[0] < s:
                self.pares['respaldo'] += 1
                self._avisar(self.meas_pend[0], 'estado sin su referencia')
                self.control(*self.meas_pend, self.ref_ultima)
            self.meas_pend = (s, meas)

    def _avisar(self, s, motivo):
        n = self.pares['respaldo']
        if n in (1, 10, 100) or n % 1000 == 0:
            self.get_logger().warn(f'emparejamiento de respaldo #{n} en t={s*1e-9:.2f}: '
                                   f'{motivo} (exactos {self.pares["exacto"]})')

    def control(self, s, meas, ref):
        self.meas_pend = None
        if self.t_ultimo is None or s - self.t_ultimo >= self.dt_ns - 1000:
            self.t_ultimo = s
            if self.kalman:
                # error y derivada del error desde el estado estimado
                if self.est is None:
                    J, b, sig, bits, kw = self.pj
                    self.est = self._control.crear_estimador(
                        True, J, b, self.dt, meas, sig, bits, (2,), **kw)
                    x = self.est.x
                else:
                    x = self.est.paso(meas, self.u)
                rv, _ = self.dif.paso(ref)
                e = ref - x[0]
                self.d_filt = rv - x[1]      # ya filtrada por el Kalman
            else:
                e = ref - meas
                d_bruta = (e - self.e_prev) / self.dt
                alfa = self.dt / (self.tau + self.dt)
                self.d_filt += alfa * (d_bruta - self.d_filt)
            u_raw = self.kp * e + self.ki * self.integral + self.kd * self.d_filt
            self.u = np.clip(u_raw, -self.u_max, self.u_max)
            sat = np.abs(u_raw) > self.u_max + 1e-12
            # clamping: no integra si el mando esta saturado y el error empuja mas
            frozen = self.aw & sat & (u_raw * e > 0.0)
            self.integral += np.where(frozen, 0.0, e * self.dt)
            self.e_prev = e
        # siempre se responde, con el sello del estado al que responde: la
        # montura solo confirma el paso si el mando es para SU instante
        out = JointState()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(int(s), 1_000_000_000)
        out.name = ['az', 'el']
        out.effort = [float(x) for x in self.u]
        self.pub.publish(out)


def main():
    correr(PidController)
