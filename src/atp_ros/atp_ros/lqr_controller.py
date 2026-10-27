"""LQR digital con accion integral por eje, con Kalman opcional.

Mismo protocolo que `pid_controller` (lockstep): hereda de `PidController`
SU emparejamiento por sello (estado y referencia del MISMO instante, respaldo
con aviso si no coinciden) y responde a cada estado con un /atp/torque
sellado con ese instante, que es lo que la montura confirma para que el reloj
avance. Solo cambia la ley de control (`control`).

Ley (ver control.py): estado aumentado [theta, omega, integral del error],
ganancia K del LQR discreto a la cadencia del lazo (20 Hz), pre-alimentacion
de velocidad y aceleracion de la referencia (u_ff = J r'' + b r'), integral
con anti-windup por clamping. Estimador: `estimator:=none` (derivada filtrada
de la medida, como el PID) o `estimator:=kalman` ([theta, omega, par de
sesgo]; sirve con encoder cuantizado/ruidoso).

El controlador NO usa `velocity` de /joint_states (es la velocidad real de
la planta: un sensor que no existe); estima omega de la medida de posicion.
"""
import math

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

from atp_ros import control, model
from atp_ros.nodo import correr
from atp_ros.params import declare_num
from atp_ros.pid_controller import PidController


class LqrController(PidController):
    def __init__(self):
        # NO se llama a PidController.__init__ (declararia los parametros del
        # PID y se llamaria 'pid_controller'): se arma aqui el MISMO estado de
        # emparejamiento que usan on_ref/on_meas/_avisar heredados.
        Node.__init__(self, 'lqr_controller')
        declare_num(self, 'J', model.J_EJE)
        declare_num(self, 'b', model.B_EJE)
        declare_num(self, 'u_max', 8.0)
        declare_num(self, 'rate_hz', 20.0)
        declare_num(self, 'q_theta', control.LQR_DEFECTO['q_theta'])
        declare_num(self, 'q_omega', control.LQR_DEFECTO['q_omega'])
        declare_num(self, 'q_int', control.LQR_DEFECTO['q_int'])
        declare_num(self, 'r_u', control.LQR_DEFECTO['r'])
        self.declare_parameter('estimator', 'none')       # none | kalman
        self.declare_parameter('feedforward', True)
        self.declare_parameter('antiwindup', True)
        self.declare_parameter('use_dhat', True)          # compensa el par de sesgo estimado
        declare_num(self, 'enc_sigma_deg', 0.0)           # ruido del encoder que ve el filtro
        declare_num(self, 'enc_bits', 0)
        declare_num(self, 'kf_q_acc', control.KALMAN_DEFECTO['q_acc'])
        declare_num(self, 'kf_q_par', control.KALMAN_DEFECTO['q_par'])

        p = self.get_parameter
        J, b = float(p('J').value), float(p('b').value)
        self.u_max = float(p('u_max').value)
        self.dt = 1.0 / float(p('rate_hz').value)
        self.dt_ns = int(round(self.dt * 1e9))
        self.kalman = str(p('estimator').value) == 'kalman'
        self.J, self.b = J, b
        self.sigma = math.radians(float(p('enc_sigma_deg').value))
        self.bits = int(p('enc_bits').value)
        self.kf_kw = {'q_acc': float(p('kf_q_acc').value),
                      'q_par': float(p('kf_q_par').value)}
        self.dis = control.lqr_integral(
            J, b, self.dt, float(p('q_theta').value), float(p('q_omega').value),
            float(p('q_int').value), float(p('r_u').value))
        self.lazo = control.LazoLQR(
            self.dis, J, b, self.u_max, forma=(2,),
            antiwindup=bool(p('antiwindup').value),
            usar_ff=bool(p('feedforward').value),
            usar_dhat=self.kalman and bool(p('use_dhat').value))
        self.dif = control.DiferenciadorRef(self.dt)
        self.est = None

        # estado de emparejamiento (identico al del PID)
        self.refs = {}
        self.ref_ultima = None
        self.meas_pend = None
        self.t_ultimo = None
        self.u = np.zeros(2)
        self.pares = {'exacto': 0, 'respaldo': 0}

        self.create_subscription(JointState, '/atp/reference', self.on_ref, 10)
        self.create_subscription(JointState, '/joint_states', self.on_meas, 10)
        self.pub = self.create_publisher(JointState, '/atp/torque', 10)
        K = self.dis['K'].ravel()
        self.get_logger().info(
            f'LQR integral a {1.0 / self.dt:g} Hz: K=[{K[0]:.2f}, {K[1]:.2f}, {K[2]:.2f}] '
            f'polos |z| max {np.max(np.abs(self.dis["polos"])):.3f} | estimador '
            f'{"kalman" if self.kalman else "derivada filtrada"}')

    def control(self, s, meas, ref):
        self.meas_pend = None
        if self.t_ultimo is None or s - self.t_ultimo >= self.dt_ns - 1000:
            self.t_ultimo = s
            if self.est is None:
                self.est = control.crear_estimador(
                    self.kalman, self.J, self.b, self.dt, meas, self.sigma,
                    self.bits, (2,), **self.kf_kw)
                estimado = self.est.x
            else:
                estimado = self.est.paso(meas, self.u)
            rv, ra = self.dif.paso(ref)
            self.u = self.lazo.calcular(estimado, ref, rv, ra)
        # siempre se responde, con el sello del estado al que responde
        out = JointState()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(int(s), 1_000_000_000)
        out.name = ['az', 'el']
        out.effort = [float(x) for x in self.u]
        self.pub.publish(out)


def main():
    correr(LqrController)
