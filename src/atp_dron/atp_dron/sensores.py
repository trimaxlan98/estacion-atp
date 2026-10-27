"""Sensores del dron con ruido y sesgo: IMU (giroscopio y acelerometro), GPS, barometro y magnetometro (rumbo).

Todos los valores por defecto son SUPUESTOS de orden de magnitud de modulos de bajo costo (MEMS + GNSS de consumo), no de un
sensor concreto. El generador es propio (semilla + 7919): encender los sensores no cambia la dinamica verdadera.
"""
import numpy as np

from atp_dron.dron import DT, euler

SENSORES = {
    'gyro_ruido': 0.004,          # rad/s por muestra a 250 Hz (SUPUESTO)
    'gyro_sesgo0': 0.01,          # rad/s, desv. del sesgo inicial
    'gyro_sesgo_rw': 2e-4,        # rad/s/sqrt(s), paseo aleatorio del sesgo
    'acel_ruido': 0.06,           # m/s2 por muestra
    'acel_sesgo0': 0.05,          # m/s2
    'acel_sesgo_rw': 2e-3,        # m/s2/sqrt(s)
    'gps_hz': 5.0, 'gps_pos_h': 0.8, 'gps_pos_v': 1.5, 'gps_vel': 0.1,   # m, m, m/s (SUPUESTO)
    'baro_hz': 25.0, 'baro_sigma': 0.3,                                   # m
    'mag_hz': 10.0, 'mag_sigma_rad': 0.05,
    'gps_corte_s': None,          # s: a partir de aqui el GPS deja de entregar medidas (prueba de perdida de GNSS)
    # --- P7: todo APAGADO por defecto (con estos valores los bytes de D3 no cambian) ---
    'gps_gm_tau_s': 0.0,          # s: tiempo de correlacion Gauss-Markov del error de POSICION del GPS (0 = blanco, como antes)
    'gps_latencia_s': 0.0,        # s: la medida GPS llega con este retraso (su contenido es el de t - latencia)
    'q_corregida': False,         # ESKF: Q de la IMU = sigma^2 DT^2 en vez de sigma^2 DT
    'h_mag_exacto': False,        # ESKF: Jacobiano del rumbo [0, sin(roll), cos(roll)]/cos(pitch)
    'est_acel_sesgo_rw': None,    # ESKF: paseo aleatorio del sesgo del acelerometro que ASUME el filtro (None = el del sensor)
    'est_gyro_sesgo_rw': None,
}


class Sensores:
    def __init__(self, dron, params=None, semilla=0):
        self.d = dron
        self.P = {**SENSORES, **(params or {})}
        self.rng = np.random.default_rng(int(semilla) + 7919)
        P = self.P
        self.sesgo_g = P['gyro_sesgo0'] * self.rng.standard_normal(3)
        self.sesgo_a = P['acel_sesgo0'] * self.rng.standard_normal(3)
        self.k = 0
        self._gm = np.zeros(3)                       # P7: estado Gauss-Markov del error de posicion del GPS
        self._cola_gps = []                          # P7: (t_entrega, pos, vel) pendientes por latencia
        self._cada = {n: max(1, int(round(1.0 / (P[n + '_hz'] * DT)))) for n in ('gps', 'baro', 'mag')}

    def imu(self):
        P, r = self.P, self.rng
        raiz = np.sqrt(DT)
        self.sesgo_g += P['gyro_sesgo_rw'] * raiz * r.standard_normal(3)
        self.sesgo_a += P['acel_sesgo_rw'] * raiz * r.standard_normal(3)
        g = self.d.w + self.sesgo_g + P['gyro_ruido'] * r.standard_normal(3)
        a = self.d.fuerza_especifica() + self.sesgo_a + P['acel_ruido'] * r.standard_normal(3)
        return g, a

    def otros(self):
        """Medidas que tocan en este paso: {'gps': (p, v), 'baro': z, 'mag': yaw}. Llamar una vez por paso."""
        P, r, d, out = self.P, self.rng, self.d, {}
        if self.k % self._cada['gps'] == 0 and not (P['gps_corte_s'] is not None and d.t >= P['gps_corte_s']):
            sp = np.array([P['gps_pos_h'], P['gps_pos_h'], P['gps_pos_v']])
            z = r.standard_normal(3)
            if P['gps_gm_tau_s'] > 0.0:              # Gauss-Markov de primer orden, varianza estacionaria = sigma^2
                phi = np.exp(-(self._cada['gps'] * DT) / P['gps_gm_tau_s'])
                self._gm = phi * self._gm + np.sqrt(1.0 - phi * phi) * z
                z = self._gm
            medida = (d.p + sp * z, d.v + P['gps_vel'] * r.standard_normal(3))
            if P['gps_latencia_s'] > 0.0:
                self._cola_gps.append((d.t + P['gps_latencia_s'], *medida))
            else:
                out['gps'] = medida
        if self._cola_gps and self._cola_gps[0][0] <= d.t + 1e-9:
            _, pm, vm = self._cola_gps.pop(0)
            out['gps'] = (pm, vm)
        if self.k % self._cada['baro'] == 0:
            out['baro'] = float(d.p[2] + P['baro_sigma'] * r.standard_normal())
        if self.k % self._cada['mag'] == 0:
            out['mag'] = float(euler(d.q)[2] + P['mag_sigma_rad'] * r.standard_normal())
        self.k += 1
        return out
