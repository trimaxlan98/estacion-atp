"""Cascada con rechazo de rafagas por INDI (inversion dinamica incremental, P7): subclase de `control.Cascada`, sin tocarla.

Idea (Smeur, Chu y de Croon 2016, JGCD 39(3):450-461, DOI 10.2514/1.G001490; Tal y Karaman, IEEE TCST, arXiv:1809.04048;
de ambas solo se leyo el resumen): en vez de modelar la perturbacion, se MIDE la aceleracion que no es de los rotores y se
resta del mando. El acelerometro mide fuerza especifica f = (T z_b + F_aero) / m (la gravedad no entra), asi que

    a_np = R (f - (T/m) e_z)          aceleracion no propulsiva en el mundo (arrastre aerodinamico + rafaga)

y el lazo de velocidad manda F = m (a_des + g e_z - ganancia * a_np_filtrada). `ganancia = 1` es INDI; `ganancia = 0` es la
Cascada de siempre; `ganancia = -1` es el CONTROL NEGATIVO (suma la perturbacion en vez de restarla: debe empeorar).

Que se mide y que se supone:
  * `fuente()` devuelve (f_cuerpo, T_total): con VERDAD, f = fuerza especifica del dron sin ruido; con SENSORES, la salida de
    la IMU menos el sesgo estimado por el ESKF. T_total es la telemetria de empuje de los rotores (en un vehiculo real, de
    las RPM de los ESC y la curva kT; aqui el empuje verdadero: SUPUESTO que lo hace optimista).
  * a_np se filtra con un pasabajos de primer orden a `fc_hz` (5 Hz por defecto).
  * en el suelo (peso apoyado) a_np se mantiene en cero.
Instalacion sin editar `lockstep.py`: `instalar(planta, ...)` reemplaza `planta.ctl` y envuelve `planta.sens.imu`.
"""
import numpy as np

from atp_dron.control import Cascada
from atp_dron.dron import DT, rot


class CascadaINDI(Cascada):
    def __init__(self, dron, fuente, ganancia=1.0, fc_hz=5.0, **kw):
        super().__init__(dron, **kw)
        self.fuente = fuente
        self.ganancia = float(ganancia)
        self.alpha = 1.0 - float(np.exp(-2.0 * np.pi * fc_hz * DT))
        self.a_np = np.zeros(3)                  # estimacion filtrada, mundo (m/s2)

    def _actualizar(self):
        m = self.fuente()
        if m is None:                            # en el suelo: no hay perturbacion que medir
            self.a_np *= 1.0 - self.alpha
            return
        f_cuerpo, T = m
        a = rot(self.d.q) @ (np.asarray(f_cuerpo, float) - np.array([0.0, 0.0, T / self.d.m]))
        self.a_np += self.alpha * (a - self.a_np)

    def mando(self, v_des, yaw_des_rad=0.0):
        self._actualizar()                       # a DT (250 Hz): el pasabajos corre en el lazo interno
        return super().mando(v_des, yaw_des_rad)

    def _lazo_externo(self, v_des, dt_ext):
        # Igual que Cascada._lazo_externo con el termino -ganancia * a_np en la aceleracion de mando.
        d = self.d
        ev = v_des - d.v
        self.int_v = np.clip(self.int_v + ev * dt_ext, -3.0, 3.0)
        a_des = self.kv * ev + self.ki_v * self.int_v - self.ganancia * self.a_np
        F = d.m * (a_des + np.array([0.0, 0.0, d.g]))
        F[2] = max(F[2], 0.2 * d.m * d.g)
        h = np.hypot(F[0], F[1])
        hmax = F[2] * np.tan(self.tilt_max)
        if h > hmax:
            F[0:2] *= hmax / h
        self.T_des = float(np.linalg.norm(F))
        self.z_b_des = F / self.T_des


def instalar(planta, ganancia=1.0, fc_hz=5.0):
    """Cambia el controlador de una `lockstep.PlantaLock` por `CascadaINDI` (misma cadena, mismo guiado)."""
    d = planta.d

    if planta.sens is None:
        def fuente():
            return None if d.en_suelo else (d.fuerza_especifica(), float(d.empuje.sum()))
    else:
        sens, est, ultimo = planta.sens, planta.est, {}
        imu0 = sens.imu

        def imu():
            g_m, a_m = imu0()
            ultimo['a'] = a_m
            return g_m, a_m
        sens.imu = imu

        def fuente():
            return None if d.en_suelo or 'a' not in ultimo else (ultimo['a'] - est.ba, float(d.empuje.sum()))

    c0 = planta.ctl
    planta.ctl = CascadaINDI(planta.obs, fuente, ganancia=ganancia, fc_hz=fc_hz, v_max=c0.v_max, a_brake=c0.a_brake,
                             kp_pos=c0.kp_pos, kv=c0.kv, ki_v=c0.ki_v, k_att=c0.k_att, k_rate=c0.k_rate)
    return planta.ctl


def correr_indi(cfg, ganancia=1.0, fc_hz=5.0, carpeta=None):
    """`lockstep.correr_cadena` con la `CascadaINDI` puesta (ganancia 0 reproduce la Cascada de siempre). Devuelve (metricas, filas, planta)."""
    import json

    from atp_dron.lockstep import Guiado, PlantaLock, cargar_cfg
    cfg = cargar_cfg(None, cfg)
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    instalar(pl, ganancia, fc_hz)
    while not pl.fin:
        sp = gu.decidir(json.loads(pl.estado_txt()))
        pl.aplicar(json.loads(json.dumps(sp, separators=(',', ':'))))
    return pl.cerrar(carpeta), pl.filas, pl
