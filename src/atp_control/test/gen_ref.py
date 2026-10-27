#!/usr/bin/env python3
"""Genera las referencias numericas de las pruebas de atp_control con el codigo
PYTHON de la casa (atp_ros.planta y atp_ros.control):

  python3 src/atp_control/test/gen_ref.py src/atp_control/test/data

  planta_ref.csv  trayectorias de PlantaMontura con un mando dado (lo que el
                  backend `sim` debe reproducir)
  laws_ref.csv    lazo cerrado planta + ley (PID, LQR, con y sin Kalman)
  gains_ref.txt   ganancias del LQR y covarianza del Kalman

No es parte de la compilacion: se corre a mano cuando cambia planta.py o
control.py, y el resultado se versiona para que gtest no dependa de Python.
"""
import math
import os
import sys

import numpy as np

from atp_ros import control, model
from atp_ros.planta import PlantaMontura, cuantizar

DT = 0.05            # paso del lazo (20 Hz)
H = 0.002            # subpaso de la planta (mount_sim.substep_s)
DEG = math.pi / 180.0

# Casos de planta: (nombre, kwargs de PlantaMontura, amplitud del mando)
CASOS = [
    ('ideal', dict(J=2.0, b=0.5), 3.0),
    ('tope_vel', dict(J=2.0, b=0.5, w_max=math.radians(6.0)), 3.0),
    ('friccion', dict(J=2.0, b=0.5, coulomb_nm=0.05, stribeck_nm=0.02,
                      v_stribeck=0.5 * DEG, v_eps=0.0573 * DEG), 0.12),
    ('backlash', dict(J=2.0, b=0.5, backlash_rad=0.05 * DEG), 0.5),
    ('backlash_rest', dict(J=2.0, b=0.5, backlash_rad=0.2 * DEG, restitucion=0.4), 0.5),
    ('realista', dict(J=2.0, b=0.5, coulomb_nm=0.05, stribeck_nm=0.02,
                      v_stribeck=0.5 * DEG, v_eps=0.0573 * DEG,
                      backlash_rad=0.05 * DEG, mgd_nm=0.05), 0.4),
]


def mando(k, amp):
    """Mando de prueba determinista (N m): senoides + escalon + inversion."""
    t = k * DT
    u0 = amp * math.sin(0.9 * t) + 0.5 * amp * (1.0 if (k // 40) % 2 == 0 else -1.0)
    u1 = amp * math.sin(0.55 * t + 1.0) - 0.4 * amp * (1.0 if k > 60 else 0.0)
    return np.array([u0, u1])


def planta_ref(out):
    n = 240   # 12 s
    with open(os.path.join(out, 'planta_ref.csv'), 'w') as f:
        f.write('caso,k,u_az,u_el,th_az,th_el,w_az,w_el,thm_az,thm_el\n')
        for nombre, kw, amp in CASOS:
            p = PlantaMontura(**kw)
            p.enganchar([math.radians(20.0), math.radians(35.0)])
            for k in range(1, n + 1):
                u = mando(k, amp)
                ext = 1.0 if 100 <= k < 120 else 0.0       # rafaga de 1 s (mount_sim)
                for _ in range(25):
                    p.paso(u, ext, H)
                thm = p.theta_m if p.theta_m is not None else p.theta
                f.write(f'{nombre},{k},{u[0]!r},{u[1]!r},{p.theta[0]!r},{p.theta[1]!r},'
                        f'{p.omega[0]!r},{p.omega[1]!r},{thm[0]!r},{thm[1]!r}\n')


# ---------------------------------------------------------------- leyes

class PidRef:
    """Copia literal de PidController.control (sin ROS)."""

    def __init__(self, kalman, bits, sigma):
        self.kp, self.ki, self.kd, self.tau, self.umax = 25.0, 2.0, 8.0, 0.08, 8.0
        self.dt = DT
        self.u = np.zeros(2)
        self.integral = np.zeros(2)
        self.e_prev = np.zeros(2)
        self.d_filt = np.zeros(2)
        self.kalman = kalman
        self.est = None
        self.bits, self.sigma = bits, sigma
        self.dif = control.DiferenciadorRef(DT)

    def paso(self, meas, ref):
        if self.kalman:
            if self.est is None:
                self.est = control.crear_estimador(True, 2.0, 0.5, DT, meas, self.sigma,
                                                   self.bits, (2,))
                x = self.est.x
            else:
                x = self.est.paso(meas, self.u)
            rv, _ = self.dif.paso(ref)
            e = ref - x[0]
            self.d_filt = rv - x[1]
        else:
            e = ref - meas
            d_bruta = (e - self.e_prev) / self.dt
            alfa = self.dt / (self.tau + self.dt)
            self.d_filt += alfa * (d_bruta - self.d_filt)
        u_raw = self.kp * e + self.ki * self.integral + self.kd * self.d_filt
        self.u = np.clip(u_raw, -self.umax, self.umax)
        sat = np.abs(u_raw) > self.umax + 1e-12
        frozen = True & sat & (u_raw * e > 0.0)
        self.integral += np.where(frozen, 0.0, e * self.dt)
        self.e_prev = e
        return self.u


class LqrRef:
    def __init__(self, kalman, bits, sigma):
        self.dis = control.lqr_integral(2.0, 0.5, DT)
        self.lazo = control.LazoLQR(self.dis, 2.0, 0.5, 8.0, forma=(2,), usar_dhat=kalman)
        self.dif = control.DiferenciadorRef(DT)
        self.est = None
        self.kalman, self.bits, self.sigma = kalman, bits, sigma
        self.u = np.zeros(2)

    def paso(self, meas, ref):
        if self.est is None:
            self.est = control.crear_estimador(self.kalman, 2.0, 0.5, DT, meas, self.sigma,
                                               self.bits, (2,))
            est = self.est.x
        else:
            est = self.est.paso(meas, self.u)
        rv, ra = self.dif.paso(ref)
        self.u = self.lazo.calcular(est, ref, rv, ra)
        return self.u


def referencia(k):
    """Referencia suave tipo pase: rampa de acimut con una curva y elevacion senoidal."""
    t = k * DT
    az = math.radians(40.0 + 9.0 * t + 6.0 * math.sin(0.4 * t))
    el = math.radians(30.0 + 25.0 * math.sin(0.12 * t))
    return np.array([az, el])


def laws_ref(out):
    n = 400   # 20 s
    casos = [('pid', PidRef, False, 0, 0.0), ('pid_kf', PidRef, True, 14, 0.0),
             ('lqr', LqrRef, False, 0, 0.0), ('lqr_kf', LqrRef, True, 14, 0.0)]
    with open(os.path.join(out, 'laws_ref.csv'), 'w') as f:
        f.write('ley,k,u_az,u_el,th_az,th_el\n')
        for nombre, clase, kal, bits, sig in casos:
            ley = clase(kal, bits, sig)
            p = PlantaMontura(2.0, 0.5, coulomb_nm=0.05, stribeck_nm=0.02,
                              v_stribeck=0.5 * DEG, v_eps=0.0573 * DEG,
                              backlash_rad=0.05 * DEG) if kal else PlantaMontura(2.0, 0.5)
            p.enganchar(referencia(1))
            for k in range(1, n + 1):
                if k > 1:
                    for _ in range(25):
                        p.paso(ley.u, 0.0, H)
                meas = p.angulo_encoder('load').copy()
                if bits:
                    meas = cuantizar(meas, bits)
                u = ley.paso(meas, referencia(k))
                f.write(f'{nombre},{k},{u[0]!r},{u[1]!r},{p.theta[0]!r},{p.theta[1]!r}\n')


def gains(out):
    d = control.lqr_integral(2.0, 0.5, DT)
    kd = control.diseno_kalman(2.0, 0.5, DT, 0.0, 14)
    P = control.FiltroKalman(kd, np.zeros(2))
    for _ in range(400):
        P.paso(np.zeros(2), 0.0)
    with open(os.path.join(out, 'gains_ref.txt'), 'w') as f:
        f.write('K ' + ' '.join(repr(float(x)) for x in d['K'].ravel()) + '\n')
        f.write('polo_max ' + repr(float(np.max(np.abs(d['polos'])))) + '\n')
        f.write('kalman_R ' + repr(float(kd['R'])) + '\n')
        f.write('kalman_Ad ' + ' '.join(repr(float(x)) for x in kd['Ad'].ravel()) + '\n')
        f.write('kalman_Qd ' + ' '.join(repr(float(x)) for x in kd['Qd'].ravel()) + '\n')
        f.write('kalman_K ' + ' '.join(repr(float(x)) for x in P.K.ravel()) + '\n')
    print('K =', d['K'].ravel(), 'polo max', np.max(np.abs(d['polos'])), 'J, b =', model.J_EJE, model.B_EJE)


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    os.makedirs(out, exist_ok=True)
    planta_ref(out)
    laws_ref(out)
    gains(out)
    print('referencias en', out)
