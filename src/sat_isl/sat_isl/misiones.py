"""Planta, controlador y medidor de un enlace optico entre dos satelites vecinos
(sin ROS: los nodos y `escenario.simular` los encadenan igual).

Secuencia (CONTRATO 8, lockstep): cada paso k la planta integra [t_{k-1}, t_k]
con el mando vigente (retencion de orden cero) y publica el ESTADO (lo que
miden los terminales: camaras, QPD, potencia recibida, y la verdad para el
monitor); el controlador responde con el MANDO (modo, direcciones de
apuntamiento, barrido, tasa); la planta lo aplica y el reloj avanza.

MODOS: 0 EN ESPERA, 1 BUSQUEDA (A barre con la baliza, B mira), 2 APRETON
(B apunto a A y emite su baliza), 3 SEGUIMIENTO (lazos QPD+FSM+montura cerrados,
enlace de datos). Los lazos finos corren a 10 kHz DENTRO de la planta (en un
terminal real son electronica/FPGA); el controlador es la supervision de
~20 Hz (efemerides, maquina de estados, tasa).

CONVENCIONES. Terminales A (indice sat_a) y B (sat_b), identicos. Direccion de
VISTA de un terminal (eje de recepcion) = direccion APARENTE de la fuente
(desviada v_perp/c por el movimiento relativo); eje de TRANSMISION = eje de
recepcion + adelanto (point-ahead = 2 v_perp / c, ver geometria.py). Todos los
angulos pequenos van en urad en la base (e1, e2) del plano normal a la
direccion mandada (`geometria.base_perpendicular`).

Que sabe cada quien: la planta tiene la VERDAD (orbitas propagadas, cardan con
su error de exactitud, perturbaciones); el controlador solo ve las medidas del
estado y su PROPIA estimacion de efemerides (verdad + error de posicion
constante sembrado) y de actitud (error de conocimiento sembrado).
"""
import math
import dataclasses
from dataclasses import dataclass, replace

import numpy as np

from sat_isl import adquisicion as A
from sat_isl import enlace as E
from sat_isl import geometria as G
from sat_isl import seguimiento as S
from sat_isl import terminal as TM

MODO_ESPERA, MODO_BUSQUEDA, MODO_APRETON, MODO_SEGUIMIENTO = 0, 1, 2, 3
NOMBRE_MODO = {0: 'ESPERA', 1: 'BUSQUEDA', 2: 'APRETON', 3: 'SEGUIMIENTO'}
DECIMACION = 10          # muestras del lazo (10 kHz) por muestra de error enviada (1 kHz)


@dataclass
class Config:
    # --- constelacion Walker delta T/P/F (propagador numerico con J2)
    n_sats: int = 24
    planos: int = 3
    fase: int = 1
    h_km: float = 800.0
    inc_deg: float = 55.0
    sat_a: int = 0
    sat_b: int = 1                      # P0S1: vecino en el plano; 11 = vecino de otro plano (ver CLI)
    inicio_s: float = 600.0             # segundos desde la epoca de la propagacion
    dur_s: float = 60.0
    paso_s: float = 0.05                # periodo del lockstep
    semilla: int = 7
    # --- terminal (los dos iguales)
    detector: str = 'coherente'         # 'coherente' | 'apd'
    d_m: float = 0.05
    p_tx_w: float = 1.0
    theta_baliza_urad: float = 300.0
    # --- lo que el controlador sabe (SUPUESTOS, ver README)
    sigma_pos_km: float = 2.1           # TLE de 1 dia (docs/OD.md: 2.1 km); 0.036 con OD de la Estacion
    sigma_actitud_urad: float = 145.0   # conocimiento de actitud (~30 arcsec 1 sigma: star tracker de nanosat)
    # --- criterios
    ber_obj: float = 1e-6
    margen_db: float = 3.0
    perdida_t_s: float = 0.2            # seguimiento perdido tanto tiempo -> a busqueda
    apreton_max_s: float = 2.0
    regimen_s: float = 1.0              # tras el cierre del lazo, antes de contar estadisticas


def config_campos():
    return [(f.name, f.type, f.default) for f in dataclasses.fields(Config)]


def terminal_de(cfg):
    base = TM.ISL_APD if cfg.detector == 'apd' else TM.ISL_COHERENTE
    return replace(base, D_m=cfg.d_m, p_tx_w=cfg.p_tx_w, theta_baliza_urad=cfg.theta_baliza_urad)


def constelacion_de(cfg):
    return G.Constelacion(T=cfg.n_sats, Pl=cfg.planos, Fase=cfg.fase, h_km=cfg.h_km,
                          inc_deg=cfg.inc_deg, t_fin_s=cfg.inicio_s + cfg.dur_s + 120.0)


def _proj(v3, base):
    """Componentes (urad) de un vector pequeno 3D en la base (e1, e2)."""
    return np.array([float(np.dot(v3, base[0])), float(np.dot(v3, base[1]))]) * 1e6


def _unit(v):
    return v / np.linalg.norm(v)


def _geom_directa(g):
    """(app_a, tx_a, app_b, tx_b, pa_a, pa_b): direcciones aparentes y de
    transmision (3D) y adelantos (3D) de cada terminal desde la geometria."""
    c = G.C_LUZ_KM_S
    u, vp = g['u'], g['v_perp']
    app_a, tx_a = u - vp / c, u + vp / c
    app_b, tx_b = -u + vp / c, -u - vp / c
    return app_a, tx_a, app_b, tx_b, tx_a - app_a, tx_b - app_b


# ====================================================================== planta

class PlantaIsl:
    """La verdad: orbitas, terminales (cardan, camaras, QPD, lazos) y enlace."""

    def __init__(self, cfg, const=None):
        self.cfg = cfg
        self.term = terminal_de(cfg)
        self.const = const or constelacion_de(cfg)
        self.rng = np.random.default_rng(cfg.semilla)
        self.t = 0.0
        self.modo = MODO_ESPERA
        self.card = {k: self.rng.normal(0.0, self.term.sigma_cardan_urad, 2) for k in 'ab'}
        self.off = {k: np.zeros(2) for k in 'ab'}          # barrido / pasos de centroide (urad)
        self.pa = {k: np.zeros(2) for k in 'ab'}           # adelanto mandado (urad)
        self.canal = None
        self.eps_cal = {k: np.zeros(2) for k in 'ab'}
        self.locked = False
        self.t_lock = None
        self._reset_busqueda()
        self.ev = {'det_a': False, 'det_b': False, 'cen_a': np.zeros(2), 'cen_b': np.zeros(2)}
        self.nea = {'a': 0.0, 'b': 0.0}
        g = self._geom()                       # el estado inicial ya trae la geometria de t = 0
        self.ultimo = self._vacio()
        self.ultimo.update(visible=bool(g['visible']), rango_km=g['rango_km'],
                           tasa_los_urad_s=g['tasa_los_rad_s'] * 1e6, adelanto_urad=g['adelanto_rad'] * 1e6)

    # -- auxiliares --------------------------------------------------------
    def _vacio(self):
        return {'t': 0.0, 'modo': MODO_ESPERA, 'visible': False, 'rango_km': 0.0,
                'tasa_los_urad_s': 0.0, 'adelanto_urad': 0.0, 'det_a': False, 'det_b': False,
                'cen_a': np.zeros(2), 'cen_b': np.zeros(2), 'qpd_ok': False,
                'pr_a_dbm': -200.0, 'pr_b_dbm': -200.0, 'ber': 0.5, 'tasa_bps': 0.0,
                'cerrado': False, 'perdida_db': 0.0, 'err_a_urad': np.zeros(0),
                'err_b_urad': np.zeros(0), 'err_a_ms': 0.0, 'err_b_ms': 0.0,
                'fsm_y_max_urad': 0.0, 'busq_k': 0, 'barridos': 0, 'enganchado': False,
                'nea_urad': 0.0, 'theta_h_urad': self.theta_h_urad}

    @property
    def theta_h_urad(self):
        return E.semiangulo_haz_rad(self.term.D_m, self.term.lam_m, self.term.gamma) * 1e6

    def _reset_busqueda(self):
        self.k = 0
        self.t_acum = 0.0
        self.pts = None
        self.pts_clave = None
        self.barridos = 0
        self.t_hs = 0.0
        self.det_a_hecho = False
        self.det_b_hecho = False

    def _geom(self):
        return self.const.enlace(self.cfg.sat_a, self.cfg.sat_b, self.cfg.inicio_s + self.t,
                                 self.term.lam_m)

    def _p_nom_w(self, rango_km):
        tm = self.term
        return E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, rango_km * 1e3,
                                     tm.eta_tx, tm.eta_rx, tm.gamma)

    # -- avance ------------------------------------------------------------
    def avanzar(self, dt, cmd):
        """Integra `dt` con el mando `cmd` (dict del controlador)."""
        tm, cfg = self.term, self.cfg
        t0 = self.t
        self.t = t0 + dt
        g = self._geom()
        modo = int(cmd['modo'])
        visible = g['visible']
        out = self._vacio()
        out.update(t=self.t, modo=modo, visible=bool(visible), rango_km=g['rango_km'],
                   tasa_los_urad_s=g['tasa_los_rad_s'] * 1e6, adelanto_urad=g['adelanto_rad'] * 1e6)
        ev = {'det_a': False, 'det_b': False, 'cen_a': self.ev['cen_a'], 'cen_b': self.ev['cen_b']}
        if modo != self.modo:                                  # transicion de modo
            if modo == MODO_BUSQUEDA:
                self._reset_busqueda()
                self.off = {k: np.zeros(2) for k in 'ab'}
                self.locked = False
                self.canal = None
            elif modo == MODO_APRETON:
                self.off['b'] = self.off['b'] + np.asarray(cmd['steer_b'])
                self.t_hs = 0.0
            elif modo == MODO_SEGUIMIENTO:
                self.off['a'] = self.off['a'] + np.asarray(cmd['steer_a'])
            elif modo == MODO_ESPERA:
                self.locked = False
            self.modo = modo
        self.pa = {'a': np.asarray(cmd['pa_a'], float), 'b': np.asarray(cmd['pa_b'], float)}
        dirs = {'a': np.asarray(cmd['dir_a'], float), 'b': np.asarray(cmd['dir_b'], float)}
        bases = {k: G.base_perpendicular(dirs[k]) for k in 'ab'}
        app_a, tx_a, app_b, tx_b, pa3_a, pa3_b = _geom_directa(g)
        if visible and modo == MODO_BUSQUEDA:
            self._busqueda(dt, cmd, g, dirs, bases, tx_a, app_b, ev, out)
        elif visible and modo == MODO_APRETON:
            self._apreton(dt, cmd, g, dirs, bases, tx_b, app_a, ev, out)
        elif modo == MODO_SEGUIMIENTO:
            self._seguimiento(dt, cmd, g, dirs, bases, app_a, app_b, pa3_a, pa3_b, visible, out)
        out['det_a'], out['det_b'] = ev['det_a'], ev['det_b']
        out['cen_a'], out['cen_b'] = ev['cen_a'], ev['cen_b']
        self.ev = ev
        out['busq_k'], out['barridos'] = self.k, self.barridos
        out['enganchado'] = self.locked
        self.ultimo = out

    def medir(self):
        return self.ultimo

    # -- busqueda ----------------------------------------------------------
    def _busqueda(self, dt, cmd, g, dirs, bases, tx_a, app_b, ev, out):
        tm = self.term
        pitch, rmax = float(cmd['pitch_urad']), float(cmd['rmax_urad'])
        if pitch <= 0.0:
            return
        clave = (round(pitch, 3), round(rmax, 3))
        if self.pts_clave != clave:
            self.pts = A.espiral_arquimedes(A.pasos_para_radio(rmax, pitch), pitch)
            self.pts_clave = clave
        dwell = float(cmd['dwell_s'])
        p0 = A.p0_baliza_w(tm, tm, g['rango_km'] * 1e3)
        th_b = tm.theta_baliza_urad
        tx_err_base = _proj(tx_a - dirs['a'], bases['a'])      # donde esta B respecto del eje 'dir_a'
        app_b_err = _proj(app_b - dirs['b'], bases['b'])
        if self.det_b_hecho:
            return
        self.t_acum += dt
        while self.t_acum >= dwell - 1e-12:
            self.t_acum -= dwell
            pt = self.pts[self.k % len(self.pts)]
            self.off['a'] = pt.copy()
            e = tx_err_base - (self.card['a'] + self.off['a'] + self.pa['a'])
            r2 = float(np.dot(e, e))
            pd = float(A.prob_deteccion(p0 * math.exp(-2.0 * r2 / th_b ** 2), tm))
            self.k += 1
            if self.k % len(self.pts) == 0:
                self.barridos += 1
            if self.rng.random() < pd:
                self.det_b_hecho = True
                ev['det_b'] = True
                ev['cen_b'] = app_b_err - (self.card['b'] + self.off['b']) \
                    + self.rng.normal(0.0, tm.cam_centroide_urad, 2)
                break

    # -- apreton -----------------------------------------------------------
    def _apreton(self, dt, cmd, g, dirs, bases, tx_b, app_a, ev, out):
        tm = self.term
        t_prev = self.t_hs
        self.t_hs += dt
        if self.det_a_hecho:
            return
        tau = g['t_luz_s']
        dwell = tm.cam_dwell_s
        # cuadros de A que caen en (t_prev, t_hs] una vez que la luz de B llego (t >= tau)
        m_lo = max(1, int(math.floor((t_prev - tau) / dwell)) + 1)
        m1 = int(math.floor((self.t_hs - tau) / dwell))
        if m1 < m_lo:
            return
        p0 = A.p0_baliza_w(tm, tm, g['rango_km'] * 1e3)
        e_b = _proj(tx_b - dirs['b'], bases['b']) - (self.card['b'] + self.off['b'] + self.pa['b'])
        pd = float(A.prob_deteccion(p0 * math.exp(-2.0 * float(np.dot(e_b, e_b)) / tm.theta_baliza_urad ** 2), tm))
        app_a_err = _proj(app_a - dirs['a'], bases['a'])
        for _ in range(m_lo, m1 + 1):
            if self.rng.random() < pd:
                self.det_a_hecho = True
                ev['det_a'] = True
                ev['cen_a'] = app_a_err - (self.card['a'] + self.off['a']) \
                    + self.rng.normal(0.0, tm.cam_centroide_urad, 2)
                break

    # -- seguimiento y enlace ---------------------------------------------
    def _seguimiento(self, dt, cmd, g, dirs, bases, app_a, app_b, pa3_a, pa3_b, visible, out):
        tm = self.term
        lazo = tm.lazo
        n = int(round(dt * lazo.fs_hz))
        theta_h = self.theta_h_urad
        if not self.locked:
            # entrada: el QPD ve el error que dejo el centroide; los lazos arrancan de ahi
            self.canal = {}
            e0 = {'a': _proj(app_a - dirs['a'], bases['a']) - (self.card['a'] + self.off['a']),
                  'b': _proj(app_b - dirs['b'], bases['b']) - (self.card['b'] + self.off['b'])}
            for k in 'ab':
                for ax in (0, 1):
                    ch = S.Canal(lazo)
                    ch.d_cte = float(e0[k][ax])
                    self.canal[(k, ax)] = ch
                self.eps_cal[k] = self.rng.normal(0.0, tm.cal_point_ahead_urad, 2)
            self.locked = True
            self.t_lock = self.t
        p_nom = self._p_nom_w(g['rango_km']) if visible else 0.0
        e_trk, ymax = {}, 0.0
        for k in 'ab':
            nea = S.ruido_qpd_urad(p_nom, tm.lam_m, lazo.fs_hz, tm.qpd_w_urad, tm.qpd_fraccion,
                                   tm.qpd_nep_w_rthz, tm.qpd_eta)
            self.nea[k] = nea
            ex = np.zeros((n, 2))
            for ax in (0, 1):
                e, y, _ = self.canal[(k, ax)].bloque(n, self.rng, nea)
                ex[:, ax] = e
                ymax = max(ymax, float(np.max(np.abs(y))))
            e_trk[k] = ex
        # el adelanto mandado contra el verdadero (2D, en cada base) + calibracion
        eps = {'a': (np.asarray(cmd['pa_a']) - _proj(pa3_a, bases['a'])) + self.eps_cal['a'],
               'b': (np.asarray(cmd['pa_b']) - _proj(pa3_b, bases['b'])) + self.eps_cal['b']}
        e_tx = {k: e_trk[k] + eps[k][None, :] for k in 'ab'}
        r_a = np.hypot(e_trk['a'][:, 0], e_trk['a'][:, 1])
        r_b = np.hypot(e_trk['b'][:, 0], e_trk['b'][:, 1])
        ok = (visible and max(r_a.max(), r_b.max()) < tm.qpd_w_urad and ymax < lazo.fsm_carrera_urad
              and max(self.nea.values()) < theta_h / 3.0)
        # potencia por muestra en cada sentido: apuntamiento de transmision de un
        # extremo + de recepcion del otro (reciprocidad: mismo theta_h)
        x_ab = np.sum(e_tx['a'] ** 2, axis=1) + r_b ** 2
        x_ba = np.sum(e_tx['b'] ** 2, axis=1) + r_a ** 2
        l_ab = np.exp(-2.0 * x_ab / theta_h ** 2)
        l_ba = np.exp(-2.0 * x_ba / theta_h ** 2)
        p_ab, p_ba = p_nom * l_ab, p_nom * l_ba           # B recibe de A / A recibe de B
        rate = float(cmd['tasa_bps'])
        det = tm.detector
        if rate > 0 and visible:
            ber_ab = float(np.mean(det.ber(p_ab, rate, tm.lam_m)))
            ber_ba = float(np.mean(det.ber(p_ba, rate, tm.lam_m)))
        else:
            ber_ab = ber_ba = 0.5
        ber = max(ber_ab, ber_ba)
        cerrado = bool(ok and rate > 0 and ber <= self.cfg.ber_obj)
        out.update(qpd_ok=bool(ok), pr_a_dbm=float(E.dbm(np.mean(p_ba))) if visible else -200.0,
                   pr_b_dbm=float(E.dbm(np.mean(p_ab))) if visible else -200.0,
                   ber=ber, tasa_bps=rate if cerrado else 0.0, cerrado=cerrado,
                   perdida_db=float(-E.db(max(np.mean(0.5 * (l_ab + l_ba)), 1e-300))),
                   err_a_urad=r_a[::DECIMACION].copy(), err_b_urad=r_b[::DECIMACION].copy(),
                   err_a_ms=float(np.mean(r_a ** 2)), err_b_ms=float(np.mean(r_b ** 2)),
                   fsm_y_max_urad=ymax, nea_urad=max(self.nea.values()))


# ====================================================================== controlador

class ControladorIsl:
    """Software de vuelo: estimacion de efemerides y actitud, maquina de estados
    y eleccion de tasa. Solo ve el estado medido y su propia estimacion."""

    def __init__(self, cfg, const=None):
        self.cfg = cfg
        self.term = terminal_de(cfg)
        self.const = const or constelacion_de(cfg)
        rng = np.random.default_rng(cfg.semilla + 1000)
        self.dr = {k: rng.normal(0.0, cfg.sigma_pos_km, 3) for k in 'ab'}        # km
        self.datt = {k: rng.normal(0.0, cfg.sigma_actitud_urad * 1e-6, 3) for k in 'ab'}   # rad
        self.modo = MODO_ESPERA
        self.t_modo0 = 0.0
        self.t_mal = 0.0
        self.cen_a = np.zeros(2)
        self.cen_b = np.zeros(2)
        self.ultimo = None
        self.t_ultimo = -1.0
        self.t_busq0 = None
        self.rate = 0.0

    def _est(self, t):
        cfg = self.cfg
        r1, v1 = self.const.rv(cfg.sat_a, cfg.inicio_s + t)
        r2, v2 = self.const.rv(cfg.sat_b, cfg.inicio_s + t)
        return G.geometria_enlace(r1 + self.dr['a'], v1, r2 + self.dr['b'], v2, self.term.lam_m)

    def _dir(self, v, k):
        """Direccion con el error de conocimiento de actitud del terminal k."""
        d = _unit(v)
        return _unit(d + np.cross(self.datt[k], d))

    def _scan(self, rango_km):
        tm, cfg = self.term, self.cfg
        p0 = A.p0_baliza_w(tm, tm, rango_km * 1e3)
        r_hit = A.radio_acierto_urad(p0, tm, tm.theta_baliza_urad)
        sig = A.sigma_incertidumbre_urad(cfg.sigma_pos_km, rango_km, cfg.sigma_actitud_urad,
                                         tm.sigma_cardan_urad)
        if r_hit <= 0.0:
            return 0.0, 0.0, tm.cam_dwell_s, sig
        pitch = A.paso_espiral_urad(r_hit)
        rmax = 3.5 * sig
        dwell = tm.cam_dwell_s if rmax <= tm.lazo.fsm_carrera_urad else max(tm.cam_dwell_s,
                                                                         tm.t_asiento_cardan_s)
        return pitch, rmax, dwell, sig

    def paso(self, t, s):
        if self.ultimo is not None and t <= self.t_ultimo + 1e-12:
            return self.ultimo                                  # estado repetido (arranque)
        dt = t - self.t_ultimo if self.t_ultimo >= 0 else 0.0
        cfg, tm = self.cfg, self.term
        g = self._est(t)
        vis = g['visible']
        modo = self.modo
        if modo == MODO_ESPERA:
            if vis:
                modo = MODO_BUSQUEDA
        elif modo == MODO_BUSQUEDA:
            if not vis:
                modo = MODO_ESPERA
            elif s['det_b']:
                self.cen_b = np.asarray(s['cen_b'], float)
                modo = MODO_APRETON
        elif modo == MODO_APRETON:
            if s['det_a']:
                self.cen_a = np.asarray(s['cen_a'], float)
                modo = MODO_SEGUIMIENTO
                self.t_mal = 0.0
            elif t - self.t_modo0 > cfg.apreton_max_s or not vis:
                modo = MODO_BUSQUEDA if vis else MODO_ESPERA
        elif modo == MODO_SEGUIMIENTO:
            if not vis:
                modo = MODO_ESPERA
            else:
                self.t_mal = self.t_mal + dt if not s['qpd_ok'] else 0.0
                if self.t_mal >= cfg.perdida_t_s:
                    modo = MODO_BUSQUEDA
        if modo != self.modo:
            self.modo, self.t_modo0 = modo, t
            if modo == MODO_BUSQUEDA and self.t_busq0 is None:
                self.t_busq0 = t
        c = g['adelanto_rad']
        vp = g['v_perp']
        u = g['u']
        dir_a = self._dir(u - vp / G.C_LUZ_KM_S, 'a')           # eje de recepcion de A: direccion aparente de B
        dir_b = self._dir(-u + vp / G.C_LUZ_KM_S, 'b')
        pa3_a, pa3_b = 2.0 * vp / G.C_LUZ_KM_S, -2.0 * vp / G.C_LUZ_KM_S
        pa_a = _proj(pa3_a, G.base_perpendicular(dir_a))
        pa_b = _proj(pa3_b, G.base_perpendicular(dir_b))
        pitch, rmax, dwell, sig = self._scan(g['rango_km'])
        # tasa: de la potencia medida (la menor de los dos extremos) o, al entrar, de la estimada
        p_w = None
        if self.modo == MODO_SEGUIMIENTO and s['pr_a_dbm'] > -150.0 and s['pr_b_dbm'] > -150.0:
            p_w = float(E.w_de_dbm(min(s['pr_a_dbm'], s['pr_b_dbm'])))
        elif vis:
            p_w = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, g['rango_km'] * 1e3,
                                        tm.eta_tx, tm.eta_rx, tm.gamma)
        self.rate = E.tasa_maxima(tm.detector, p_w, tm.lam_m, cfg.ber_obj, cfg.margen_db) \
            if (p_w and self.modo == MODO_SEGUIMIENTO) else 0.0
        cmd = {'modo': self.modo, 'dir_a': dir_a, 'dir_b': dir_b,
               'steer_a': self.cen_a.copy(), 'steer_b': self.cen_b.copy(),
               'pa_a': pa_a, 'pa_b': pa_b, 'pitch_urad': pitch, 'rmax_urad': rmax,
               'dwell_s': dwell, 'tasa_bps': self.rate, 'sigma_u_urad': sig}
        self.ultimo, self.t_ultimo = cmd, t
        return cmd


# ====================================================================== medidor

class Medidor:
    """Acumula estado y mando de cada paso y resume la corrida (el RESUMEN)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.term = terminal_de(cfg)
        self.t, self.est, self.cmds = [], [], []

    def muestra(self, st, cmd):
        self.t.append(float(st['t']))
        self.est.append(st)
        self.cmds.append(cmd)

    def recortar(self, t_fin):
        n = sum(1 for t in self.t if t <= t_fin + 1e-9)
        self.t, self.est, self.cmds = self.t[:n], self.est[:n], self.cmds[:n]

    def numeros(self):
        cfg = self.cfg
        if len(self.t) < 2:
            return {}
        dt = cfg.paso_s
        t = np.array(self.t)
        modo = np.array([int(s['modo']) for s in self.est])
        vis = np.array([bool(s['visible']) for s in self.est])
        cerr = np.array([bool(s['cerrado']) for s in self.est])
        tasa = np.array([float(s['tasa_bps']) for s in self.est])
        out = {'duracion_s': float(t[-1] - t[0] + dt), 'n': len(t)}
        i_busq = next((i for i, m in enumerate(modo) if m == MODO_BUSQUEDA), None)
        i_lock = next((i for i, s in enumerate(self.est)
                       if int(s['modo']) == MODO_SEGUIMIENTO and s['qpd_ok']), None)
        out['t_busqueda_s'] = float(t[i_busq]) if i_busq is not None else None
        out['t_adq_s'] = float(t[i_lock] - t[i_busq]) if (i_busq is not None and i_lock is not None) else None
        out['t_lock_s'] = float(t[i_lock]) if i_lock is not None else None
        out['k_pasos'] = int(self.est[i_lock]['busq_k']) if i_lock is not None else None
        out['barridos'] = int(self.est[-1]['barridos'])
        out['visible_pct'] = float(100.0 * vis.mean())
        out['cerrado_pct_corrida'] = float(100.0 * cerr.mean())
        out['cerrado_pct_visible'] = float(100.0 * cerr[vis].mean()) if vis.any() else 0.0
        out['tasa_media_bps'] = float(tasa.mean())
        out['tasa_cerrado_bps'] = float(tasa[cerr].mean()) if cerr.any() else 0.0
        out['volumen_gb'] = float(np.sum(tasa) * dt / 8e9)
        # error fino: tras `regimen_s` del cierre, en SEGUIMIENTO
        if i_lock is not None:
            reg = [s for s in self.est[i_lock:] if int(s['modo']) == MODO_SEGUIMIENTO
                   and s['t'] >= t[i_lock] + cfg.regimen_s and len(s['err_a_urad'])]
        else:
            reg = []
        if reg:
            ea = np.concatenate([s['err_a_urad'] for s in reg])
            eb = np.concatenate([s['err_b_urad'] for s in reg])
            e = np.concatenate([ea, eb])
            ms = float(np.mean([0.5 * (s['err_a_ms'] + s['err_b_ms']) for s in reg]))
            out.update(err_rms_urad=math.sqrt(ms), err_p95_urad=float(np.percentile(e, 95)),
                       err_max_urad=float(e.max()), err_n=len(reg),
                       perdida_media_db=float(np.mean([s['perdida_db'] for s in reg])),
                       perdida_max_db=float(np.max([s['perdida_db'] for s in reg])),
                       fsm_y_max_urad=float(np.max([s['fsm_y_max_urad'] for s in reg])),
                       pr_min_dbm=float(np.min([min(s['pr_a_dbm'], s['pr_b_dbm']) for s in reg])),
                       pr_media_dbm=float(np.mean([0.5 * (s['pr_a_dbm'] + s['pr_b_dbm']) for s in reg])),
                       ber_max=float(np.max([s['ber'] for s in reg])))
        out['perdidas_seguimiento'] = int(sum(1 for a, b in zip(modo[:-1], modo[1:])
                                              if a == MODO_SEGUIMIENTO and b == MODO_BUSQUEDA))
        out['cortes_visibilidad'] = int(sum(1 for a, b in zip(modo[:-1], modo[1:])
                                            if a == MODO_SEGUIMIENTO and b == MODO_ESPERA))
        i = np.where(vis)[0]
        out['rango_km'] = (float(self.est[i[0]]['rango_km']), float(self.est[i[-1]]['rango_km'])) if len(i) else None
        out['adelanto_urad'] = float(np.mean([s['adelanto_urad'] for s in self.est if s['visible']] or [0.0]))
        out['theta_h_urad'] = float(self.est[-1]['theta_h_urad'])
        return out

    def resumen(self):
        """Lineas RESUMEN, deterministas (mismas cifras en ROS y fuera de ROS)."""
        n = self.numeros()
        if not n:
            return ['RESUMEN sin muestras']
        cfg, tm = self.cfg, self.term
        L = []
        L.append(f"RESUMEN enlace {tm.detector.nombre} {tm.D_m * 100:.0f} cm {tm.lam_m * 1e9:.0f} nm "
                 f"{tm.p_tx_w:.1f} W: rango {n['rango_km'][0]:.0f}-{n['rango_km'][1]:.0f} km, "
                 f"adelanto {n['adelanto_urad']:.1f} urad, semiangulo del haz {n['theta_h_urad']:.1f} urad, "
                 f"visible {n['visible_pct']:.0f} % de {n['duracion_s']:.0f} s")
        if n['t_adq_s'] is None:
            L.append('RESUMEN adquisicion: NO se adquirio el enlace')
        else:
            L.append(f"RESUMEN adquisicion: {n['t_adq_s']:.2f} s desde el inicio de la busqueda "
                     f"(t={n['t_busqueda_s']:.2f} s) hasta el seguimiento; {n['k_pasos']} pasos de "
                     f"barrido, {n['barridos']} barridos completos")
        if 'err_rms_urad' in n:
            L.append(f"RESUMEN seguimiento fino (tras {cfg.regimen_s:.0f} s): error radial rms "
                     f"{n['err_rms_urad']:.3f} urad, p95 {n['err_p95_urad']:.3f} urad, max "
                     f"{n['err_max_urad']:.3f} urad ({n['err_n']} pasos) | FSM |y| max "
                     f"{n['fsm_y_max_urad']:.1f} urad de {tm.lazo.fsm_carrera_urad:.0f}")
            L.append(f"RESUMEN enlace: perdida por apuntamiento media {n['perdida_media_db']:.3f} dB "
                     f"(max {n['perdida_max_db']:.3f}) | Pr media {n['pr_media_dbm']:.1f} dBm "
                     f"(min {n['pr_min_dbm']:.1f}) | BER max {n['ber_max']:.2e}")
        L.append(f"RESUMEN disponibilidad: enlace cerrado {n['cerrado_pct_visible']:.1f} % del tiempo "
                 f"visible ({n['cerrado_pct_corrida']:.1f} % de la corrida) | tasa "
                 f"{n['tasa_cerrado_bps'] / 1e6:.1f} Mb/s con el enlace cerrado, media "
                 f"{n['tasa_media_bps'] / 1e6:.1f} Mb/s | {n['volumen_gb']:.3f} GB | "
                 f"perdidas de seguimiento {n['perdidas_seguimiento']}, cortes por visibilidad {n['cortes_visibilidad']}")
        return L
