"""Estimador de navegacion del dron: filtro de Kalman de error de estado (ESKF) de 15 estados (Sola 2017).

Nominal: posicion p, velocidad v, cuaternion q (cuerpo->mundo), sesgos del acelerometro ba y del giroscopio bg.
Error (cuerpo, q_verdadero = q (x) dq): [dp, dv, dtheta, dba, dbg]. Prediccion con la IMU a 250 Hz; correcciones con GPS
(posicion y velocidad), barometro (altura) y magnetometro (rumbo). Sin el GPS el rumbo y los sesgos no son observables
en vuelo estacionario: por eso el magnetometro.
"""
import numpy as np

from atp_dron.dron import DT, rot


def _cruz(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def _qmul(a, b):
    return np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                     a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                     a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                     a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])


def _q_exp(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([1.0, *(0.5 * v)])
    return np.concatenate([[np.cos(n / 2)], np.sin(n / 2) * v / n])


class ESKF:
    def __init__(self, g, p0, q0, sens):
        self.g = np.array([0.0, 0.0, -g])
        self.p, self.v, self.q = np.array(p0, float), np.zeros(3), np.array(q0, float)
        self.ba, self.bg = np.zeros(3), np.zeros(3)
        S = sens
        self.P = np.diag(np.r_[np.full(3, 0.25), np.full(3, 0.01), np.full(3, 0.05 ** 2), np.full(3, S['acel_sesgo0'] ** 2 * 4),
                               np.full(3, S['gyro_sesgo0'] ** 2 * 4)])
        # q_corregida (P7, apagada): el ruido de la IMU es BLANCO POR MUESTRA (sigma por muestra, sin escalar con la tasa), asi que
        # la varianza que entra por paso en v y theta es sigma^2 * DT^2 (incremento = sigma * DT), no sigma^2 * DT.
        dt_r = DT * DT if S.get('q_corregida') else DT
        rw_a = S['acel_sesgo_rw'] if S.get('est_acel_sesgo_rw') is None else S['est_acel_sesgo_rw']
        rw_g = S['gyro_sesgo_rw'] if S.get('est_gyro_sesgo_rw') is None else S['est_gyro_sesgo_rw']
        self.Q = np.diag(np.r_[np.zeros(3), np.full(3, S['acel_ruido'] ** 2 * dt_r), np.full(3, S['gyro_ruido'] ** 2 * dt_r),
                               np.full(3, rw_a ** 2 * DT), np.full(3, rw_g ** 2 * DT)])
        self.S = S

    def predecir(self, gyro, acel):
        am, wm = acel - self.ba, gyro - self.bg
        R = rot(self.q)
        a = R @ am + self.g
        self.p = self.p + self.v * DT + 0.5 * a * DT * DT
        self.v = self.v + a * DT
        self.q = _qmul(self.q, _q_exp(wm * DT))
        self.q /= np.linalg.norm(self.q)
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * DT
        F[3:6, 6:9] = -R @ _cruz(am) * DT
        F[3:6, 9:12] = -R * DT
        F[6:9, 6:9] = np.eye(3) - _cruz(wm) * DT
        F[6:9, 12:15] = -np.eye(3) * DT
        self.P = F @ self.P @ F.T + self.Q

    def _corregir(self, y, H, Rm):
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T          # forma de Joseph
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = _qmul(self.q, _q_exp(dx[6:9]))
        self.q /= np.linalg.norm(self.q)
        self.ba += dx[9:12]
        self.bg += dx[12:15]

    def gps(self, pos, vel):
        S = self.S
        H = np.zeros((6, 15))
        H[0:6, 0:6] = np.eye(6)
        Rm = np.diag([S['gps_pos_h'] ** 2, S['gps_pos_h'] ** 2, S['gps_pos_v'] ** 2, *([S['gps_vel'] ** 2] * 3)])
        self._corregir(np.r_[pos - self.p, vel - self.v], H, Rm)

    def baro(self, z):
        H = np.zeros((1, 15))
        H[0, 2] = 1.0
        self._corregir(np.array([z - self.p[2]]), H, np.array([[self.S['baro_sigma'] ** 2]]))

    def mag(self, yaw):
        from atp_dron.dron import euler
        e = yaw - euler(self.q)[2]
        e = (e + np.pi) % (2 * np.pi) - np.pi
        H = np.zeros((1, 15))
        if self.S.get('h_mag_exacto'):
            # P7 (apagada): d(yaw) = (sin(roll) dtheta_y + cos(roll) dtheta_z) / cos(pitch) para dtheta en el cuerpo (angulos de Euler ZYX)
            r_, p_, _ = euler(self.q)
            H[0, 7:9] = np.array([np.sin(r_), np.cos(r_)]) / np.cos(p_)
        else:
            H[0, 6:9] = rot(self.q)[2, :]
        self._corregir(np.array([e]), H, np.array([[self.S['mag_sigma_rad'] ** 2]]))


class VistaEstimada:
    """Lo que el control lee (p, v, q, w, I, m, g, A_inv) tomado del estimador y del dron (parametros conocidos)."""

    def __init__(self, dron, est):
        self._d, self._e = dron, est
        self.w = np.zeros(3)

    p = property(lambda s: s._e.p)
    v = property(lambda s: s._e.v)
    q = property(lambda s: s._e.q)
    I = property(lambda s: s._d.I)
    m = property(lambda s: s._d.m)
    g = property(lambda s: s._d.g)
    A_inv = property(lambda s: s._d.A_inv)
