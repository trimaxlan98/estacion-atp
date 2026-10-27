"""Control en cascada del cuadrirrotor: posicion -> velocidad (PI) -> atitud geometrica (Lee 2010) -> tasa (P) -> mezclador.

Sin ROS. `Cascada.mando(dron_estado, ref)` devuelve el empuje por rotor (N). Lazo interno cada paso (250 Hz con DT=0.004),
lazo de velocidad cada `n_ext` pasos (50 Hz). Ganancias de partida, sin sintonia fina (SUPUESTO).
"""
import numpy as np

from atp_dron.dron import DT, rot


def _cruz(a, b):
    """Producto cruz de dos 3-vectores sin pasar por `np.cross` (que gasta mas en despacho que en aritmetica con vectores de 3).
    Mismas operaciones de punto flotante que `np.cross`: mismos bits."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def _vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


V_MAX, A_BRAKE, KP_POS = 5.0, 2.0, 1.2        # los de Cascada; el guiado (lockstep.Guiado) usa la misma funcion


def velocidad_deseada(p, ref_p, v_max=V_MAX, a_brake=A_BRAKE, kp_pos=KP_POS, frenar=True, v_esquina=1.5):
    """Velocidad que lleva de `p` a `ref_p`: perfil de frenado sqrt(2 a d) si `frenar`; si no, pasa a `v_esquina` en la meta."""
    e = np.asarray(ref_p, float) - np.asarray(p, float)
    dist = np.linalg.norm(e)
    if dist < 1e-9:
        return np.zeros(3)
    v = v_max
    if frenar:                                   # llega a velocidad cero sobre la meta
        v = min(v, np.sqrt(2.0 * a_brake * dist), kp_pos * dist)
    else:                                        # meta intermedia: pasa a `v_esquina` en vez de frenar del todo
        v = min(v, np.sqrt(v_esquina ** 2 + 2.0 * a_brake * dist))
    return e / dist * v


class Cascada:
    def __init__(self, dron, v_max=V_MAX, a_brake=A_BRAKE, inclinacion_max_deg=35.0, n_ext=5,
                 kp_pos=KP_POS, kv=3.0, ki_v=0.8, k_att=6.0, k_rate=12.0,
                 antiwindup=False, mezcla_par=False, mezcla_dT_g=0.1):
        self.d = dron
        self.v_max, self.a_brake = v_max, a_brake
        self.tilt_max = np.radians(inclinacion_max_deg)
        self.n_ext, self.k = n_ext, 0
        self.kp_pos, self.kv, self.ki_v, self.k_att, self.k_rate = kp_pos, kv, ki_v, k_att, k_rate
        # Banderas de P6 (apagadas = los bytes de siempre): integracion condicionada del lazo de velocidad y mezclador con
        # prioridad de par (`guiado_autoridad.mezclar_prioridad_par`; el empuje colectivo puede cambiar a lo mas `mezcla_dT_g`*m*g).
        self.antiwindup, self.mezcla_par, self.mezcla_dT_g = bool(antiwindup), bool(mezcla_par), float(mezcla_dT_g)
        self.int_v = np.zeros(3)
        self.z_b_des = np.array([0.0, 0.0, 1.0])
        self.T_des = dron.m * dron.g

    def velocidad_deseada(self, ref_p, frenar=True, v_esquina=1.5):
        return velocidad_deseada(self.d.p, ref_p, self.v_max, self.a_brake, self.kp_pos, frenar, v_esquina)

    def _lazo_externo(self, v_des, dt_ext):
        d = self.d
        ev = v_des - d.v
        int_nuevo = np.clip(self.int_v + ev * dt_ext, -3.0, 3.0)
        a_des = self.kv * ev + self.ki_v * int_nuevo
        F = d.m * (a_des + np.array([0.0, 0.0, d.g]))
        # limite de inclinacion: componente horizontal <= F_z tan(tilt_max); F_z siempre positivo
        piso = F[2] < 0.2 * d.m * d.g                    # empuje vertical en el piso: tampoco se integra (mando de bajada imposible)
        F[2] = max(F[2], 0.2 * d.m * d.g)
        h = np.hypot(F[0], F[1])
        hmax = F[2] * np.tan(self.tilt_max)
        saturado = bool(h > hmax) or bool(piso)
        if h > hmax:
            F[0:2] *= hmax / h
        # antiwindup: con el limite de inclinacion activo la integral no sigue creciendo (integracion condicionada)
        self.int_v = self.int_v if (self.antiwindup and saturado) else int_nuevo
        self.T_des = float(np.linalg.norm(F))
        self.z_b_des = F / self.T_des

    def mando(self, v_des, yaw_des_rad=0.0):
        d = self.d
        dt = DT
        if self.k % self.n_ext == 0:
            self._lazo_externo(np.asarray(v_des, float), dt * self.n_ext)
        self.k += 1
        R = rot(d.q)
        zb = self.z_b_des
        xc = np.array([np.cos(yaw_des_rad), np.sin(yaw_des_rad), 0.0])
        yb = _cruz(zb, xc)
        yb /= np.linalg.norm(yb)
        Rd = np.column_stack([_cruz(yb, zb), yb, zb])
        e_R = 0.5 * _vee(Rd.T @ R - R.T @ Rd)
        w_des = -self.k_att * e_R
        tau = d.I @ (self.k_rate * (w_des - d.w)) + _cruz(d.w, d.I @ d.w)
        T = self.T_des * max(0.0, float(zb @ R[:, 2]))     # proyecta el empuje en el eje real del cuerpo
        if self.mezcla_par:
            from atp_dron.guiado_autoridad import mezclar_prioridad_par
            return mezclar_prioridad_par(d.A_inv, T, tau, d.t_max, self.mezcla_dT_g * d.m * d.g)
        return d.A_inv @ np.array([T, tau[0], tau[1], tau[2]])
