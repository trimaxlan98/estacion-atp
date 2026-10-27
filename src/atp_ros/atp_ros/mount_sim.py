"""Planta de la montura: dos ejes, J theta'' + b theta' = u_sat + perturbacion.

Se alinea con la referencia en el primer mensaje (enganche en AOS), como
`simular_pase` de atp.py. Integracion semi-implicita en radianes.

Va en lockstep con `sim_clock`: en cada /clock integra EXACTAMENTE el tiempo
transcurrido con el mando vigente (retencion de orden cero, como un DAC) y
publica su estado con ese sello; al recibir el mando nuevo lo confirma en
/atp/step_ack para que el reloj avance.

Fisica opcional (ver `planta.py`; todo APAGADO por defecto, y apagado el
resultado es identico bit a bit al de la planta ideal): friccion de Coulomb
+ Stribeck, backlash del reductor, desbalance gravitatorio en elevacion,
cuantizacion del encoder y viento con espectro (Dryden o Davenport).
Con `publish_truth` publica tambien /atp/mount_truth (angulo REAL de la
carga y velocidad real, sin ruido ni cuantizacion) para medir el apuntamiento
verdadero; ningun controlador debe usarlo.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from atp_ros import model
from atp_ros.nodo import correr
from atp_ros.params import declare_num
from atp_ros.planta import PlantaMontura, VientoTurbulento, cuantizar


class MountSim(Node):
    def __init__(self):
        super().__init__('mount_sim')
        declare_num(self, 'J', model.J_EJE)
        declare_num(self, 'b', model.B_EJE)
        declare_num(self, 'u_max', 8.0)
        declare_num(self, 'encoder_noise_deg', 0.0)
        declare_num(self, 'gust_start_s', -1.0)   # <0: sin rafaga
        declare_num(self, 'gust_duration_s', 1.0)
        declare_num(self, 'gust_torque_nm', 0.0)
        declare_num(self, 'seed', 7)
        declare_num(self, 'vel_max_deg_s', 0.0)        # <=0: sin tope de velocidad
        declare_num(self, 'substep_s', 0.002)
        # --- fisica opcional (por defecto todo apagado) ---
        declare_num(self, 'coulomb_nm', 0.0)           # friccion de Coulomb (carga)
        declare_num(self, 'stribeck_nm', 0.0)          # exceso estatico Fs - Fc
        declare_num(self, 'stribeck_vel_deg_s', 0.5)   # velocidad de Stribeck (supuesto)
        declare_num(self, 'friction_veps_deg_s', 0.0573)   # banda de adherencia (1e-3 rad/s)
        declare_num(self, 'backlash_deg', 0.0)         # holgura total del reductor
        declare_num(self, 'backlash_motor_frac', 0.5)  # fraccion de J en el motor
        declare_num(self, 'backlash_restitution', 0.0)
        declare_num(self, 'unbalance_nm', 0.0)         # m g d de la elevacion
        declare_num(self, 'encoder_bits', 0)           # 0: sin cuantizar
        declare_num(self, 'meas_delay_s', 0.0)         # retardo de la medida (comunicacion/filtro)
        self.declare_parameter('encoder_side', 'load')  # load | motor
        declare_num(self, 'wind_sigma_nm', 0.0)        # RMS del par de viento; 0: sin viento
        self.declare_parameter('wind_model', 'dryden')  # dryden | davenport
        declare_num(self, 'wind_tau_s', 5.0)
        declare_num(self, 'wind_v10_m_s', 10.0)
        self.declare_parameter('publish_truth', False)

        p = self.get_parameter
        self.J, self.b = float(p('J').value), float(p('b').value)
        self.u_max = float(p('u_max').value)
        self.noise = math.radians(float(p('encoder_noise_deg').value))
        self.gust = (float(p('gust_start_s').value),
                     float(p('gust_duration_s').value),
                     float(p('gust_torque_nm').value))
        self.rng = np.random.default_rng(int(p('seed').value))
        v = float(p('vel_max_deg_s').value)
        self.w_max = math.radians(v) if v > 0.0 else math.inf
        self.h = float(p('substep_s').value)
        self.t_prev = None

        self.planta = PlantaMontura(
            self.J, self.b, self.w_max, self.h,
            coulomb_nm=float(p('coulomb_nm').value),
            stribeck_nm=float(p('stribeck_nm').value),
            v_stribeck=math.radians(float(p('stribeck_vel_deg_s').value)),
            v_eps=math.radians(float(p('friction_veps_deg_s').value)),
            backlash_rad=math.radians(float(p('backlash_deg').value)),
            frac_motor=float(p('backlash_motor_frac').value),
            restitucion=float(p('backlash_restitution').value),
            mgd_nm=float(p('unbalance_nm').value))
        self.bits = int(p('encoder_bits').value)
        # retardo de la lectura del encoder (gemelo vs real,: el
        # sello es el del instante, el VALOR es el de hace `meas_delay_s`; 0: sin cambios
        self.delay_ns = int(round(float(p('meas_delay_s').value) * 1e9))
        self._buf_meas = []
        self.enc_side = str(p('encoder_side').value)
        self.viento = None
        if float(p('wind_sigma_nm').value) > 0.0:
            # generador propio: el del encoder no cambia con o sin viento
            self.viento = VientoTurbulento(
                float(p('wind_sigma_nm').value), str(p('wind_model').value),
                float(p('wind_tau_s').value), float(p('wind_v10_m_s').value),
                semilla=int(p('seed').value) + 1000)
        self.par_viento = 0.0
        self.truth = bool(p('publish_truth').value)

        self.u = np.zeros(2)
        self.t0 = None

        self.create_subscription(JointState, '/atp/reference', self.on_ref, 10)
        self.create_subscription(JointState, '/atp/torque',
                                 self.on_torque, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.truth_pub = (self.create_publisher(JointState, '/atp/mount_truth', 10)
                          if self.truth else None)
        self.ack_pub = self.create_publisher(Clock, '/atp/step_ack', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.stamp = None              # sello del ultimo estado publicado

    @property
    def theta(self):
        return self.planta.theta        # [az, el] rad de la carga; None hasta enganchar

    def on_ref(self, msg):
        if self.planta.theta is None:
            self.planta.enganchar(msg.position[:2])
            self.t0 = rclpy.time.Time.from_msg(msg.header.stamp)
            self.t_prev = self.t0
            # el estado del instante del enganche sale YA: con la compuerta
            # de arranque el reloj espera este primer intercambio
            self.publish(msg.header.stamp)
            self.get_logger().info(
                f'enganchado en AOS: az={math.degrees(self.planta.theta[0]):.2f} '
                f'el={math.degrees(self.planta.theta[1]):.2f} deg')

    def on_torque(self, msg):
        self.u = np.clip(np.array(msg.effort[:2], dtype=float),
                         -self.u_max, self.u_max)
        # se confirma el instante PARA EL QUE se calculo el mando, no el
        # ultimo publicado: un mando atrasado no puede adelantar el reloj
        self.ack_pub.publish(Clock(clock=msg.header.stamp))

    def on_clock(self, msg):
        now = Time.from_msg(msg.clock)
        if self.t_prev is None or self.planta.theta is None:
            self.t_prev = now
            if self.planta.theta is None:
                return
        elapsed = (now - self.t_prev).nanoseconds * 1e-9
        if elapsed <= 0.0:
            return            # /clock repetido (latido) o viejo: nada que integrar
        self.t_prev = now
        t_rel = (now - self.t0).nanoseconds * 1e-9 - elapsed
        t_ini, dur, nm = self.gust
        if self.viento is not None:
            # par de viento retenido durante el intervalo (cadencia del reloj)
            self.par_viento = self.viento.par(t_rel + elapsed, elapsed)
        n = int(round(elapsed / self.h, 9))
        resto = elapsed - n * self.h
        for h in [self.h] * n + ([resto] if resto > 1e-9 else []):
            par_ext = nm if (t_ini >= 0.0 and t_ini <= t_rel < t_ini + dur) else 0.0
            if self.viento is not None:
                par_ext = par_ext + self.par_viento
            self.planta.paso(self.u, par_ext, h)
            t_rel += h
        self.publish(msg.clock)

    def publish(self, stamp):
        meas = self.planta.angulo_encoder(self.enc_side).copy()
        if self.noise > 0.0:
            meas += self.rng.normal(0.0, self.noise, 2)
        if self.bits > 0:
            meas = cuantizar(meas, self.bits)
        if self.delay_ns > 0:
            t_ns = Time.from_msg(stamp).nanoseconds
            self._buf_meas.append((t_ns, meas.copy()))
            while len(self._buf_meas) > 1 and self._buf_meas[1][0] <= t_ns - self.delay_ns:
                self._buf_meas.pop(0)
            meas = self._buf_meas[0][1].copy()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = ['az', 'el']
        msg.position = [float(x) for x in meas]
        msg.velocity = [float(x) for x in self.planta.omega]
        msg.effort = [float(x) for x in self.u]
        self.stamp = stamp
        self.pub.publish(msg)
        if self.truth_pub is not None:
            tr = JointState()
            tr.header.stamp = stamp
            tr.name = ['az', 'el']
            tr.position = [float(x) for x in self.planta.theta]
            tr.velocity = [float(x) for x in self.planta.omega]
            self.truth_pub.publish(tr)


def main():
    correr(MountSim)
