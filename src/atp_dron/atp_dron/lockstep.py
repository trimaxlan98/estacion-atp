"""Cadena del dron en LOCKSTEP (CONTRATO sec. 8 y 21; docs/DRON.md «D5»). Python puro, sin ROS.

`lazo.correr`, los nodos `dron_dinamica` y `dron_navegador` y `correr_cadena` son LA MISMA cadena: `lazo.correr` la ejecuta en
un proceso y los nodos (`nodos_lockstep.py`) solo ponen tuberia DDS alrededor de `PlantaLock` y `Guiado`.

Reparto (como un autopiloto con computadora de mision): la PLANTA es el dron y su controlador de vuelo a bordo (sensores, estimador,
cascada de velocidad/actitud, motores); el GUIADO es la computadora de mision (metas, despegue, velocidad deseada).

Un tick k dura PASO_S = 5 pasos de la dinamica = 0.02 s (el lazo externo de la cascada, 50 Hz); t_k = k * PASO_S:

  1. La planta publica el estado e_k (verdad + lo que ve a bordo: `obs_p`, `obs_v`, la estimacion si hay sensores) y espera.
  2. El guiado decide con e_k SOLO lo que ve a bordo y responde UN mensaje con el sello k: el setpoint {k, t, v, yaw, guia}.
     Un solo mensaje, sin fichas: no hay nada que emparejar entre topicos.
  3. La setpoint de k rige [t_k, t_{k+1}) COMPLETO (retencion de orden cero): la planta corre 5 pasos (sensores, estimador, cascada,
     RK4 a 4 ms) y confirma el reloj. El lazo externo de la cascada solo usa la velocidad deseada del primer paso de cada
     bloque de 5, asi que decidirla una vez por tick no pierde nada.

Si el guiado dice `terminado` (llego con `parar_al_llegar`), la planta no integra ese tick y cierra.
"""
import json
import os

import numpy as np

from atp_dron.control import A_BRAKE, KP_POS, Cascada, velocidad_deseada
from atp_dron.dron import DT, Dron, rot
from atp_dron.estimador import ESKF, VistaEstimada
from atp_dron.sensores import Sensores

N_EXT = 5
PASO_S = N_EXT * DT                       # 0.02 s: intervalo de decision y de muestreo (= step_s del reloj)


def cargar_cfg(escena=None, extra=None):
    """cfg completo: DEFECTOS de lazo + `args` del YAML `escena` + `extra` (dict o JSON)."""
    from atp_dron.lazo import DEFECTOS
    cfg = dict(DEFECTOS)
    if escena:
        import yaml
        with open(escena) as f:
            cfg.update((yaml.safe_load(f).get('args')) or {})
    if isinstance(extra, str):
        extra = json.loads(extra or '{}')
    cfg.update(extra or {})
    return cfg


def _angulo_cero(q):
    return float(np.degrees(np.arccos(np.clip(rot(q)[2, 2], -1, 1))))


class Guiado:
    """Computadora de mision: de la observacion a bordo al setpoint de velocidad. Sin estado fisico, solo el indice de meta."""

    def __init__(self, cfg):
        from atp_dron.lazo import DEFECTOS
        self.c = c = {**DEFECTOS, **cfg}
        self.ruta = [np.array(m, float) for m in c['ruta']]
        self.p_ini = np.array([c['x0'], c['y0'], 0.0])
        self.seg0_corr, self.p_desp = bool(c.get('seg0_corregido', False)), None   # P6: el tramo 0 empieza donde acaba el despegue
        self.i = 0
        self.t_llegada = None
        self.yaw = float(np.radians(c['yaw_deg']))
        self.ultimo = None
        # P6, apagado por defecto: limites de frenado y de crucero segun la autoridad del vehiculo (guiado_autoridad.py)
        self.a_brake, self.v_tope, self.ev_max, self.a_acel, self.v_cmd, self.vz_cmd = A_BRAKE, None, None, None, None, None
        self.acel_modo = c.get('autoridad_acel', False)    # False | 'recorte' | True/'rampa'
        if c.get('autoridad_guiado', False):
            from atp_dron.guiado_autoridad import limites_autoridad
            self.a_brake, self.v_tope, info = limites_autoridad(c['cuerpo'], c['dron'], c.get('inclinacion_max_deg', 35.0),
                                                                A_BRAKE, c['v_max'])
            if self.acel_modo == 'recorte':
                self.ev_max = info['ev_max_ms']
            elif self.acel_modo:
                self.a_acel, self.acel_modo = info['a_acel_ms2'], 'rampa'
                self.ev_cap = 3.0 * info['ev_max_ms']
                self.az_acel = info['az_acel_ms2']

    def _meta_lejos(self, p):
        return self.i < len(self.ruta) and float(np.hypot(*(self.ruta[self.i][:2] - p[:2]))) > 1.0

    def decidir(self, e):
        c, ruta = self.c, self.ruta
        if self.ultimo is not None and e['k'] <= self.ultimo:
            return None                                   # duplicado: nadie responde dos veces al mismo sello
        self.ultimo = e['k']
        p, v = np.array(e['obs_p'], float), np.array(e['obs_v'], float)
        terminado = False
        while self.i < len(ruta):                         # metas alcanzadas con lo que se ve a bordo
            ultima = self.i == len(ruta) - 1
            if np.linalg.norm(ruta[self.i] - p) < c['radio_meta_m'] and (not ultima or np.linalg.norm(v) < 0.5):
                self.i += 1
                if self.i == len(ruta):
                    self.t_llegada = e['t']
                    terminado = bool(c['parar_al_llegar'])
                    break
            else:
                break
        # `c['v_max']` se lee en cada tick (un guiado derivado puede cambiarlo por tramo, p. ej. aire_keyhole.GuiadoPase); el tope por
        # autoridad (P6) solo lo recorta
        v_max = c['v_max'] if self.v_tope is None else min(c['v_max'], self.v_tope)
        if not ruta:                                      # ruta vacia (antes IndexError): se queda donde esta
            v_des, seg0 = np.zeros(3), p
        elif self.i < len(ruta):
            v_des = velocidad_deseada(p, ruta[self.i], v_max, self.a_brake, KP_POS, frenar=self.i == len(ruta) - 1)
            seg0 = ruta[self.i - 1] if self.i > 0 else self.p_ini
            if self.seg0_corr and self.i == 0:
                if self.p_desp is None and not (p[2] < c['z_despegue_m'] and self._meta_lejos(p)):
                    self.p_desp = p.copy()               # primer tick fuera del despegue vertical
                seg0 = p if self.p_desp is None else self.p_desp
        else:
            v_des = velocidad_deseada(p, ruta[-1], v_max, self.a_brake, KP_POS)
            seg0 = ruta[-1]
        if self.ev_max is not None:                       # P6: el lazo de velocidad no pide mas aceleracion de la que el cuerpo da
            ev = v_des - v
            ne = float(np.linalg.norm(ev))
            if ne > self.ev_max:
                v_des = v + ev * (self.ev_max / ne)
        if self.a_acel is not None:                       # P6, 'rampa': v_des cambia a lo mas a_acel (ni salto de 0 a v_max ni frenazo)
            # el plano horizontal y la vertical se rampean aparte: la vertical tiene otra autoridad (bajar solo da ~g, no g tan i)
            if self.vz_cmd is None:
                self.vz_cmd = float(v[2])
            lz = self.az_acel * PASO_S
            self.vz_cmd += min(max(float(v_des[2]) - self.vz_cmd, -lz), lz)
            self.vz_cmd = float(v[2]) + min(max(self.vz_cmd - float(v[2]), -self.ev_cap), self.ev_cap)
            vz = self.vz_cmd
            if self.v_cmd is None:
                self.v_cmd = v[:2].copy()
            dv = v_des[:2] - self.v_cmd
            nd, lim = float(np.linalg.norm(dv)), self.a_acel * PASO_S
            self.v_cmd = self.v_cmd + (dv * (lim / nd) if nd > lim else dv)
            er = self.v_cmd - v[:2]                       # y no se aleja de la velocidad real mas de ev_cap (si el cuerpo no sigue)
            ne = float(np.linalg.norm(er))
            if ne > self.ev_cap:
                self.v_cmd = v[:2] + er * (self.ev_cap / ne)
            v_des = np.array([self.v_cmd[0], self.v_cmd[1], vz])
        if p[2] < c['z_despegue_m'] and self._meta_lejos(p):    # despegue vertical antes de avanzar en horizontal
            v_des = np.array([0.0, 0.0, min(2.0, c['v_max'])])
            self.v_cmd = self.vz_cmd = None
        n = len(ruta)
        return {'k': e['k'], 't': e['t'], 'v': [float(x) for x in v_des], 'yaw': self.yaw,
                'guia': {'i': self.i, 'meta': max(0, int(min(self.i, n - 1))), 'n': n, 'seg0': [float(x) for x in seg0],
                         'dest': None if self.i >= n else [float(x) for x in ruta[self.i]],
                         't_llegada': self.t_llegada, 'terminado': terminado}}


class PlantaLock:
    """El dron con su controlador de vuelo a bordo, avanzado por ticks de 0.02 s con el setpoint recibido."""

    def __init__(self, cfg):
        from atp_dron.lazo import DEFECTOS
        self.c = c = {**DEFECTOS, **cfg}
        self.d = d = Dron(c['cuerpo'], c['dron'], pos=(c['x0'], c['y0'], 0.0), yaw_deg=c['yaw_deg'], viento=c['viento'],
                          rafaga_sigma=c['rafaga_sigma'], rafaga_tau_s=c['rafaga_tau_s'], semilla=c['semilla'])
        self.sens = self.est = self.vista = None
        self.obs = d                                       # lo que lee el control y el guiado: la verdad o la estimacion
        if c['sensores']:
            self.sens = Sensores(d, c['sensores_params'], c['semilla'])
            self.est = ESKF(d.g, d.p, d.q, self.sens.P)
            self.vista = self.obs = VistaEstimada(d, self.est)
        kw = {k: c[k] for k in ('antiwindup', 'mezcla_par', 'mezcla_dT_g') if k in c}      # P6: banderas de la cascada (apagadas por defecto)
        if 'inclinacion_max_deg' in c:
            kw['inclinacion_max_deg'] = c['inclinacion_max_deg']
        self.ctl = Cascada(self.obs, v_max=c['v_max'], **kw)
        assert self.ctl.n_ext == N_EXT
        self.ruta = [np.array(m, float) for m in c['ruta']]
        self.n = int(round(c['duracion_s'] / DT))
        self.cada = max(1, int(round(1.0 / (c['hz_telemetria'] * DT))))
        self.pasos = 0
        self.k = 0
        self.fin = self.n <= 0
        self.filas, self.setpoints = [], []
        self.guia = {'i': 0, 'meta': 0, 'n': len(self.ruta), 't_llegada': None}
        self.dist = self.xt2 = 0.0
        self.nxt = 0
        self.tilt_max = self.v_max_alc = 0.0
        self.e2_p = self.e2_v = self.e2_a = 0.0

    # --- lo que se publica
    def estado(self):
        d, o = self.d, self.obs
        e = d.estado()
        e.update({'k': self.k, 'obs_p': [float(x) for x in o.p], 'obs_v': [float(x) for x in o.v]})
        return e

    def estado_txt(self):
        return json.dumps(self.estado(), separators=(',', ':'))

    # --- un tick
    def aplicar(self, sp):
        """Aplica el setpoint del tick actual (`sp['k'] == self.k`). Devuelve nada; mira `fin`."""
        c, d = self.c, self.d
        g = sp['guia']
        self.guia = g
        self.setpoints.append({'k': sp['k'], 't': sp['t'], 'v': sp['v'], 'yaw': sp['yaw']})
        self.k += 1
        if g['terminado']:
            self.fin = True
            return
        v_des, yaw = np.array(sp['v'], float), float(sp['yaw'])
        for _ in range(min(N_EXT, self.n - self.pasos)):
            k = self.pasos
            if self.sens is not None:
                g_m, a_m = self.sens.imu()
                self.est.predecir(g_m, a_m)
                for nombre, val in self.sens.otros().items():
                    if nombre == 'gps':
                        self.est.gps(*val)
                    elif nombre == 'baro':
                        self.est.baro(val)
                    else:
                        self.est.mag(val)
                self.vista.w = g_m - self.est.bg
            T = self.ctl.mando(v_des, yaw)
            antes = d.p.copy()
            d.paso(T)
            self.pasos += 1
            self.dist += float(np.linalg.norm(d.p - antes))
            if g['dest'] is not None and d.p[2] >= c['z_despegue_m']:   # error transversal a la recta del tramo (tras el despegue)
                u = np.array(g['dest']) - np.array(g['seg0'])
                nu = np.linalg.norm(u)
                if nu > 1e-6:
                    u /= nu
                    r = d.p - np.array(g['seg0'])
                    self.xt2 += float(np.linalg.norm(r - (r @ u) * u) ** 2)
                    self.nxt += 1
            if self.est is not None:
                self.e2_p += float(np.sum((self.est.p - d.p) ** 2))
                self.e2_v += float(np.sum((self.est.v - d.v) ** 2))
                self.e2_a += float(_ang(self.est.q, d.q) ** 2)
            e = d.estado()
            if self.est is not None and k % self.cada == 0:
                e.update({'ex': float(self.est.p[0]), 'ey': float(self.est.p[1]), 'ez': float(self.est.p[2])})
            self.tilt_max = max(self.tilt_max, _angulo_cero(d.q))
            self.v_max_alc = max(self.v_max_alc, float(np.linalg.norm(d.v)))
            if k % self.cada == 0:
                e['meta'] = g['meta']
                self.filas.append(e)
            if d.choque or d.bateria_agotada:
                self.fin = True
                break
        if self.pasos >= self.n:
            self.fin = True

    def metricas(self):
        c, d, g = self.c, self.d, self.guia
        n = len(self.ruta)
        llego = bool(g['i'] >= n)
        t_ll = g.get('t_llegada')
        met = {'cuerpo': c['cuerpo'], 'semilla': c['semilla'], 'tiempo_s': round(d.t, 6),
               'distancia_m': round(self.dist, 4), 'metas_alcanzadas': int(min(g['i'], n)), 'metas_total': n,
               'llego': llego, 't_llegada_s': None if t_ll is None else round(t_ll, 6),
               'error_transversal_rms_m': round(float(np.sqrt(self.xt2 / self.nxt)) if self.nxt else 0.0, 4),
               'energia_Wh': round(float(d.energia_j) / 3600.0, 5), 'inclinacion_max_deg': round(self.tilt_max, 3),
               'velocidad_max_ms': round(self.v_max_alc, 4), 'choque': bool(d.choque),
               'empuje_peso_efectivo': round(d.empuje_peso_efectivo, 4),
               'vuelo_posible': bool(d.empuje_peso_efectivo > 1.0)}
        est, sens = self.est, self.sens
        if est is not None and d.t > 0:
            nk = max(1, int(round(d.t / DT)))
            met.update({'est_error_pos_rms_m': round(float(np.sqrt(self.e2_p / nk)), 4),
                        'est_error_vel_rms_ms': round(float(np.sqrt(self.e2_v / nk)), 4),
                        'est_error_actitud_rms_deg': round(float(np.degrees(np.sqrt(self.e2_a / nk))), 4),
                        'est_error_pos_final_m': round(float(np.linalg.norm(est.p - d.p)), 4),
                        'est_sesgo_gyro_error_rads': round(float(np.linalg.norm(est.bg - sens.sesgo_g)), 5),
                        'est_sesgo_acel_error_ms2': round(float(np.linalg.norm(est.ba - sens.sesgo_a)), 5)})
        if d.P['bateria_wh'] is not None:
            met['bateria_agotada'] = bool(d.bateria_agotada)
            met['soc_final'] = round(max(0.0, 1.0 - d.energia_j / 3600.0 / d.P['bateria_wh']), 4)
        return met

    def cerrar(self, carpeta=None):
        met = self.metricas()
        if carpeta:
            os.makedirs(carpeta, exist_ok=True)
            with open(os.path.join(carpeta, 'telemetry.jsonl'), 'w') as f:
                for r in self.filas:
                    f.write(json.dumps(r, sort_keys=True) + '\n')
            with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
                json.dump(met, f, sort_keys=True, indent=1)
            with open(os.path.join(carpeta, 'setpoints.jsonl'), 'w') as f:
                for r in self.setpoints:
                    f.write(json.dumps(r, sort_keys=True) + '\n')
        return met


def _ang(q1, q2):
    """Angulo (rad) del giro que lleva q1 a q2."""
    return 2.0 * float(np.arccos(min(1.0, abs(float(np.dot(q1, q2))))))


def correr_cadena(cfg, carpeta=None):
    """La cadena completa en un proceso, pasando por JSON como la tuberia DDS. Devuelve (metricas, filas, setpoints)."""
    cfg = cargar_cfg(None, cfg)
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    while not pl.fin:
        sp = gu.decidir(json.loads(pl.estado_txt()))
        pl.aplicar(json.loads(json.dumps(sp, separators=(',', ':'))))
    met = pl.cerrar(carpeta)
    return met, pl.filas, pl.setpoints
