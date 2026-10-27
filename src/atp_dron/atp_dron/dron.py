"""Dinamica 6-DOF de un cuadrirrotor en X (marco mundo ENU, z arriba; cuerpo FLU: x adelante, y izquierda, z arriba).

Estado: posicion p, velocidad v (mundo), cuaternion q (w,x,y,z; cuerpo->mundo), velocidad angular w (cuerpo) y velocidad
de los cuatro motores om (rad/s). Integracion RK4 con dt fijo; el viento se mantiene constante dentro del paso.

Modelo (todo SUPUESTO de orden de magnitud salvo lo que dice la fuente):
  * empuje por rotor T_i = kT * om_i^2, con kT proporcional a la densidad del aire (rho/1.225) y a `rotor_escala`;
  * par de reaccion en guinada tau_z = -sum(s_i * kM * T_i), con kM = 0.016 m (orden tipico de helices de 5");
  * motor de primer orden, om' = (om_cmd - om)/tau_m;
  * arrastre cuadratico del cuerpo F = -1/2 rho CdA |v-viento| (v-viento), sin arrastre de rotor ni efecto suelo;
  * efecto suelo opcional (`efecto_suelo`): T/T_inf = 1/(1-(R/4z)^2) de Cheeseman y Bennett (1955), con z >= R/2 (tope 1.33);
  * contacto con el suelo: plano z=0 inelastico (v_z>=0 al tocar); choque si baja a mas de `v_choque` m/s.
Cuerpo rigido: I w' = tau - w x (I w) (Euler); atitud q' = 1/2 q (x) (0,w).
"""
import numpy as np

from atp_dron.cuerpos import RHO_REF, cuerpo, viscosidad
from atp_dron import rotor_fisica as RF

DT = 0.004
G0 = 9.80665
S_GIRO = np.array([1.0, -1.0, 1.0, -1.0])          # sentido de giro de cada rotor (alternado)
PARAMS = {
    'masa_kg': 1.5,
    'brazo_m': 0.23,                  # distancia centro-rotor; los rotores estan a 45 grados de los ejes
    'inercia': [0.0123, 0.0123, 0.0224],   # kg m2 (SUPUESTO: cuerpo + 4 motores puntuales)
    'empuje_peso': 2.2,               # empuje maximo total / peso, a nivel del mar en la Tierra
    'om_max': 1000.0,                 # rad/s (~9550 rpm)
    'tau_motor_s': 0.05,
    'k_guinada_m': 0.016,
    'cda_m2': 0.03,                   # SUPUESTO
    'radio_rotor_m': 0.127,
    'fom': 0.6,                       # figura de merito del rotor (Leishman 2006: 0.5-0.75)
    'eta_motor': 0.8,
    'p_avionica_w': 15.0,
    'rotor_escala': 1.0,              # multiplica kT: rotores mas grandes a igual om_max (Marte)
    'v_choque': 3.0,
    'efecto_suelo': False,            # Cheeseman-Bennett sobre el empuje a igual velocidad de giro (apagado: bytes de D1)
    'bateria_wh': None,               # energia util; None = sin limite
    # --- P8, todo apagado por defecto (ver rotor_fisica.py) ---
    'modo_rotor': 'legacy',           # 'coherente': kM se deriva de FoM (FoM_implicita = fom en hover); 'energia_par': energia = kM T Omega
    'f_reynolds': False,              # factor parametrico f(Re) sobre kT y FoM (solo cuerpos con viscosidad verificada)
    'cuerda_m': 0.02,                 # SUPUESTO de geometria (cuerda de pala) para Re
    're_ref': 9.0e4,                  # Re por encima del cual f=1 (PARAMETRICO)
    'exp_kT': 0.2, 'exp_fom': 0.297,  # exponentes parametricos de f(Re) (ver rotor_fisica.exponente_desde_caida)
    'avance': False,                  # energia con flujo inducido de Glauert + perfil + parasita
    'k_mu': 4.65,                     # coeficiente de perfil en avance (PARAMETRICO)
    'vrs': False,                     # anillo de vortice: NO implementado (True lanza ValueError)
}


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def q_de_euler(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2),
                              np.cos(yaw / 2), np.sin(yaw / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def euler(q):
    w, x, y, z = q
    return (np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _cruz(a, b):
    """Producto cruz de dos 3-vectores sin la sobrecarga de np.cross (mismas operaciones, mismos bytes)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def _qmul_w(q, w):
    a, b, c, d = q
    wx, wy, wz = w
    return 0.5 * np.array([-b * wx - c * wy - d * wz, a * wx + c * wz - d * wy,
                           a * wy - b * wz + d * wx, a * wz + b * wy - c * wx])


class Dron:
    def __init__(self, cuerpo_nombre='tierra', params=None, pos=(0.0, 0.0, 0.0), yaw_deg=0.0,
                 viento=(0.0, 0.0, 0.0), rafaga_sigma=0.0, rafaga_tau_s=2.0, semilla=0):
        c = cuerpo(cuerpo_nombre)
        self.g, self.rho = c['g'], c['rho']
        self.P = {**PARAMS, **(params or {})}
        P = self.P
        if P['vrs']:
            raise ValueError('vrs: estado de anillo de vortice no implementado (coeficientes NO VERIFICADO)')
        if P['modo_rotor'] not in ('legacy', 'coherente', 'energia_par'):
            raise ValueError(f"modo_rotor desconocido: {P['modo_rotor']!r}")
        self._mu = viscosidad(cuerpo_nombre) if P['f_reynolds'] else None
        self._fre = self._mu is not None and self.rho > 0.0
        self.m = float(P['masa_kg'])
        self.I = np.diag(P['inercia']).astype(float)
        self.Iinv = np.linalg.inv(self.I)
        d = P['brazo_m'] / np.sqrt(2.0)
        self.rx = np.array([d, d, -d, -d])           # rotor 0 delante-derecha, 1 delante-izquierda, 2 atras-izq., 3 atras-der.
        self.ry = np.array([-d, d, d, -d])
        t_max_ref = P['empuje_peso'] * self.m * G0 / 4.0
        self.kT = t_max_ref / P['om_max'] ** 2 * (self.rho / RHO_REF) * P['rotor_escala']
        self.t_max = self.kT * P['om_max'] ** 2      # empuje maximo por rotor aqui y ahora
        self.empuje_peso_efectivo = 4.0 * self.t_max / (self.m * self.g)
        self.k_guinada_declarado = P['k_guinada_m']
        if P['modo_rotor'] == 'coherente' and self.rho > 0.0 and self.kT > 0.0:
            T_h = self.m * self.g / 4.0
            P['k_guinada_m'] = RF.k_m_coherente(T_h, np.sqrt(T_h / self.kT), self.rho, P['radio_rotor_m'], P['fom'])
        self.p = np.array(pos, float)
        self.v = np.zeros(3)
        self.q = q_de_euler(0.0, 0.0, np.radians(yaw_deg))
        self.w = np.zeros(3)
        self.om = np.zeros(4)
        self.t = 0.0
        self.viento_medio = np.array(viento, float)
        self.rafaga = np.zeros(3)
        self.sigma, self.tau_r = float(rafaga_sigma), float(rafaga_tau_s)
        self.rng = np.random.default_rng(semilla)
        self.choque = False
        self.en_suelo = self.p[2] <= 1e-9
        self.energia_j = 0.0
        self.empuje = np.zeros(4)
        # matriz de asignacion [T, tau_x, tau_y, tau_z] = A @ [T_0..T_3]
        self.A = np.array([np.ones(4), self.ry, -self.rx, -S_GIRO * P['k_guinada_m']])
        self.A_inv = np.linalg.inv(self.A)

    @property
    def viento(self):
        return self.viento_medio + self.rafaga

    def empuje_a_om(self, T):
        return np.sqrt(np.clip(T, 0.0, self.t_max) / self.kT) if self.kT > 0 else np.zeros(4)

    def _f_re(self, om, expo):
        P = self.P
        return RF.f_reynolds(RF.reynolds(self.rho, self._mu, om, P['radio_rotor_m'], P['cuerda_m']), P['re_ref'], expo)

    def _empuje_efectivo(self, om, z):
        """Empuje por rotor que ven la dinamica y el acelerometro (una sola ruta: efecto suelo y f(Re) en un lugar)."""
        T = self.kT * om * om
        if self._fre:
            T = T * self._f_re(om, self.P['exp_kT'])
        if self.P['efecto_suelo']:
            T = RF.aplicar_suelo(T, z, self.P['radio_rotor_m'])
        return T

    def _deriv(self, s, om_cmd, wind):
        p, v, q, w, om = s[0:3], s[3:6], s[6:10], s[10:13], s[13:17]
        T = self._empuje_efectivo(om, p[2])
        R = rot(q)
        vr = v - wind
        arr = -0.5 * self.rho * self.P['cda_m2'] * np.linalg.norm(vr) * vr
        F = R @ np.array([0.0, 0.0, T.sum()]) + arr
        a = F / self.m + np.array([0.0, 0.0, -self.g])
        tau = np.array([self.ry @ T, -(self.rx @ T), -(S_GIRO * self.P['k_guinada_m']) @ T])
        wd = self.Iinv @ (tau - _cruz(w, self.I @ w))
        return np.concatenate([v, a, _qmul_w(q, w), wd, (om_cmd - om) / self.P['tau_motor_s']])

    def paso(self, T_cmd, dt=DT):
        """Avanza un paso con empuje mandado por rotor `T_cmd` (N); lo recorta a [0, t_max]."""
        om_cmd = self.empuje_a_om(np.asarray(T_cmd, float))
        if self.sigma > 0.0:                       # Ornstein-Uhlenbeck exacto, determinista por semilla
            a = np.exp(-dt / self.tau_r)
            self.rafaga = a * self.rafaga + self.sigma * np.sqrt(1 - a * a) * self.rng.standard_normal(3)
        wind = self.viento
        s = np.concatenate([self.p, self.v, self.q, self.w, self.om])
        k1 = self._deriv(s, om_cmd, wind)
        k2 = self._deriv(s + 0.5 * dt * k1, om_cmd, wind)
        k3 = self._deriv(s + 0.5 * dt * k2, om_cmd, wind)
        k4 = self._deriv(s + dt * k3, om_cmd, wind)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.p, self.v, self.q, self.w, self.om = s[0:3], s[3:6], s[6:10] / np.linalg.norm(s[6:10]), s[10:13], s[13:17]
        self.om = np.clip(self.om, 0.0, None)
        self.empuje = self.kT * self.om ** 2
        if self._fre:
            self.empuje = self.empuje * self._f_re(self.om, self.P['exp_kT'])
        self._energia(dt)
        self._suelo()
        self.t += dt

    def _energia(self, dt):
        A = np.pi * self.P['radio_rotor_m'] ** 2
        # potencia inducida ideal T^1.5 / sqrt(2 rho A), entre FoM y eficiencia del motor (rho de la atmosfera)
        if self.rho > 0 and self.P['modo_rotor'] == 'energia_par':
            p = (self.P['k_guinada_m'] * self.empuje * self.om).sum() / self.P['eta_motor']     # P_eje = kM T Omega
        elif self.rho > 0 and self.P['avance']:
            fom = self.P['fom'] * (np.maximum(self._f_re(self.om, self.P['exp_fom']), 1e-12) if self._fre else 1.0)
            fom = np.broadcast_to(fom, (4,))
            V = float(np.linalg.norm(self.v - self.viento))
            p = 0.0
            for i in range(4):
                pi, _ = RF.potencia_avance(self.empuje[i], V, self.rho, self.P['radio_rotor_m'], fom[i], self.om[i],
                                           self.P['k_mu'], self.P['cda_m2'])
                p += pi / self.P['eta_motor']
        elif self.rho > 0 and self._fre:       # misma expresion que el caso base, con FoM(Re); f=1 da los mismos bytes
            fom = self.P['fom'] * np.maximum(self._f_re(self.om, self.P['exp_fom']), 1e-12)
            p = (self.empuje ** 1.5 / (np.sqrt(2 * self.rho * A) * fom * self.P['eta_motor'])).sum()
        elif self.rho > 0:
            p = (self.empuje ** 1.5 / (np.sqrt(2 * self.rho * A) * self.P['fom'] * self.P['eta_motor'])).sum()
        else:
            p = 0.0
        self.energia_j += (p + self.P['p_avionica_w']) * dt

    def _suelo(self):
        if self.p[2] >= 0.0:
            self.en_suelo = False
            return
        if self.v[2] < -self.P['v_choque']:
            self.choque = True
        self.p[2] = 0.0
        self.v[2] = max(self.v[2], 0.0)
        self.en_suelo = True
        if self.empuje.sum() < self.m * self.g:     # apoyado: sin deslizar ni girar (SUPUESTO)
            self.v[:] = 0.0
            self.w[:] = 0.0
            r, pi, y = euler(self.q)
            self.q = q_de_euler(0.0, 0.0, y)

    def fuerza_especifica(self):
        """Lo que mide un acelerometro (fuerza no gravitatoria / masa, en el cuerpo) con el estado y el viento actuales."""
        R = rot(self.q)
        if self.en_suelo and self.empuje.sum() < self.m * self.g:     # apoyado: el suelo sostiene el peso
            return R.T @ np.array([0.0, 0.0, self.g])
        T = self._empuje_efectivo(self.om, self.p[2])
        vr = self.v - self.viento
        arr = -0.5 * self.rho * self.P['cda_m2'] * np.linalg.norm(vr) * vr
        return (np.array([0.0, 0.0, T.sum()]) + R.T @ arr) / self.m

    @property
    def bateria_agotada(self):
        b = self.P['bateria_wh']
        return b is not None and self.energia_j / 3600.0 >= b

    def estado(self):
        r, pi, y = euler(self.q)
        return {'t': round(self.t, 6), 'x': float(self.p[0]), 'y': float(self.p[1]), 'z': float(self.p[2]),
                'vx': float(self.v[0]), 'vy': float(self.v[1]), 'vz': float(self.v[2]),
                'roll_deg': float(np.degrees(r)), 'pitch_deg': float(np.degrees(pi)), 'yaw_deg': float(np.degrees(y)),
                'empuje_n': float(self.empuje.sum())}
