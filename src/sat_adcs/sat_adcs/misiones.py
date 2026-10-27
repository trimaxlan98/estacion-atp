"""Planta, controlador y medidor del ADCS, SIN ROS.

Los nodos `adcs_sim`, `adcs_controller` y `adcs_monitor` son envolturas finas
de las clases de aqui; `escenario.simular` las encadena en un solo proceso
(mismo orden de operaciones que el lockstep), asi que una corrida ROS y una
corrida `adcs_offline` dan el mismo RESUMEN.

Cadena de un paso (CONTRATO.md sec. 8), con t = k * paso:
    planta.medir()  -> sensores y verdad en t
    ctrl.paso(t, s) -> mando en t (modo, par de ruedas, dipolos)
    planta.avanzar(paso, mando) integra [t, t + paso] con el mando retenido
Modos: DETUMBLE (B-dot) -> NADIR -> TRACK_GS (antena a la estacion en un pase).
"""
import dataclasses
import math
from dataclasses import dataclass

import numpy as np

from atp_ros import model, orbit
from sat_adcs import fisica as F

MODO_DETUMBLE, MODO_NADIR, MODO_TRACK = 0, 1, 2
NOMBRE_MODO = {MODO_DETUMBLE: 'DETUMBLE', MODO_NADIR: 'NADIR', MODO_TRACK: 'TRACK_GS'}
BORESIGHT = np.array([0.0, 0.0, 1.0])       # eje +Z del cuerpo = antena


@dataclass
class Config:
    """Todos los parametros del escenario (cada nodo los declara como
    parametros ROS con estos mismos nombres y valores por omision)."""
    # --- escenario
    sat: str = 'FUNCUBE-1'
    estacion: str = 'cdmx'
    inicio_utc: str = '2026-10-02T00:00:00'
    pase_idx: int = 0
    el_max_min_deg: float = 20.0
    anticipo_min_s: float = 3600.0      # el pase empieza al menos asi despues del inicio
    paso_s: float = 1.0                 # periodo del lazo ADCS (lockstep)
    h_int_s: float = 0.5                # subpaso de RK4 de la planta
    semilla: int = 7
    w0_deg_s: float = 8.0               # tumbling inicial (|w|)
    # --- sensores (SUPUESTOS tipicos de COTS; ver README)
    mag_sigma_nt: float = 150.0
    giro_arw: float = 1.0e-4            # rad/s^0.5
    giro_rrw: float = 1.0e-6            # rad/s^1.5
    giro_sesgo0: float = 5.0e-4         # rad/s (1-sigma del sesgo inicial real)
    sol_sigma_deg: float = 0.5
    rueda_sigma_nms: float = 2.0e-7
    # --- estimador
    sigma_att0_deg: float = 6.0
    sigma_b0: float = 1.0e-3            # rad/s (lo que el filtro cree del sesgo inicial)
    # grado del campo magnetico de a bordo: 1 = dipolo inclinado (error ~32 %), 4 = 24
    # coeficientes IGRF (~4 %); 13 = la verdad. Por omision 4 (ver README: el dipolo no
    # basta para garantizar el apuntamiento, solo lo cumple por geometria favorable)
    modelo_bordo_nmax: int = 4
    # --- control
    bdot_tau_s: float = 50.0
    bdot_filtro_s: float = 3.0
    detumble_hold_s: float = 120.0
    wn: float = 0.1                     # rad/s
    zeta: float = 0.9
    desat_tau_s: float = 600.0
    previo_s: float = 150.0             # preapuntado antes del AOS
    cola_s: float = 30.0                # el escenario sigue tras el LOS
    # --- enlace (antena del satelite con su propio haz)
    banda: str = 'X'
    d_sat_m: float = 0.3                # apertura equivalente (reflectarray desplegable, SUPUESTO)
    eta_sat: float = 0.5
    p_tx_w: float = 5.0
    tasa_bps: float = 100e6
    perdida_max_db: float = 1.0         # tolerancia de perdida por apuntamiento
    d_gs_m: float = 3.0                 # plato de la estacion (model.DIAMETRO_PLATO_M)


def config_campos():
    """(nombre, tipo, defecto) de cada campo, para declarar parametros ROS."""
    return [(f.name, f.type, f.default) for f in dataclasses.fields(Config)]


# ====================================================================== geometria / efemeride

class Efemeride:
    """Orbita del satelite en TEME a t segundos desde `t0_utc`, por SGP4 (el TLE
    'subido' al satelite). Planta y controlador usan la MISMA, asi que el
    controlador conoce la orbita sin error: SUPUESTO que equivale a un GPS
    de ~10 m (irrelevante para una antena de varios grados); el error propio
    de SGP4 (~1 km) no afecta a la actitud."""

    def __init__(self, tle, t0_utc):
        self.tle = tle
        self.jd0, self.fr0 = orbit.a_jd(t0_utc)

    def jdfr(self, t):
        return self.jd0, self.fr0 + t / 86400.0

    def rv(self, t):
        jd, fr = self.jdfr(t)
        e, r, v = self.tle.satrec.sgp4(jd, fr)
        if e != 0:
            raise RuntimeError(f'SGP4 fallo (codigo {e}) en t={t}')
        return np.array(r), np.array(v)


class EfemerideCircular:
    """Orbita kepleriana circular (para pruebas), mismo contrato que `Efemeride`."""

    def __init__(self, a_km, inc_deg, t0_utc, raan_deg=0.0, u0_deg=0.0):
        self.a, self.inc = a_km, math.radians(inc_deg)
        self.raan, self.u0 = math.radians(raan_deg), math.radians(u0_deg)
        self.jd0, self.fr0 = orbit.a_jd(t0_utc)

    def jdfr(self, t):
        return self.jd0, self.fr0 + t / 86400.0

    def rv(self, t):
        return F.r_circular(self.a, self.inc, t, self.raan, self.u0)


def eci_a_ecef(x, jd, fr):
    th = float(orbit.gmst_rad(jd, fr))
    c, s = math.cos(th), math.sin(th)
    return np.array([c * x[0] + s * x[1], -s * x[0] + c * x[1], x[2]])


def ecef_a_eci(x, jd, fr):
    th = float(orbit.gmst_rad(jd, fr))
    c, s = math.cos(th), math.sin(th)
    return np.array([c * x[0] - s * x[1], s * x[0] + c * x[1], x[2]])


class Geometria:
    """Satelite visto desde la estacion terrena (cadena de `orbit.py`: GMST,
    ECEF, ENU de la latitud geodesica). Sin refraccion."""

    def __init__(self, efem, estacion):
        self.efem, self.est = efem, estacion
        self.r_gs_ecef = orbit.geodetica_a_ecef(estacion.lat_deg, estacion.lon_deg,
                                                estacion.alt_m)
        self.enu = orbit.matriz_enu(estacion.lat_deg, estacion.lon_deg)

    def en(self, t):
        """dict: r, v (TEME km, km/s), jd, fr, r_gs (TEME km), el_deg, rango_km."""
        r, v = self.efem.rv(t)
        jd, fr = self.efem.jdfr(t)
        rel = self.enu @ (eci_a_ecef(r, jd, fr) - self.r_gs_ecef)
        rho = float(np.linalg.norm(rel))
        return {'r': r, 'v': v, 'jd': jd, 'fr': fr,
                'r_gs': ecef_a_eci(self.r_gs_ecef, jd, fr),
                'el_deg': math.degrees(math.asin(rel[2] / rho)), 'rango_km': rho}

    def el(self, t):
        return self.en(t)['el_deg']


def q_ref_lvlh(r, v):
    """Actitud de referencia LVLH (cuerpo -> TEME): +Z = nadir, +Y = -normal de
    la orbita, +X = Y x Z (~ velocidad). Orbita casi circular."""
    z = -F.unitario(r)
    y = -F.unitario(np.cross(r, v))
    return F.dcm_a_q(np.column_stack([np.cross(y, z), y, z]))


def q_ref_gs(r, v, r_gs):
    """Referencia que apunta +Z a la estacion; el giro sobre el bore-sight se
    fija con +Y lo mas cerca posible de -normal de la orbita. Se degenera si
    la estacion cae sobre la normal de la orbita, imposible en un pase LEO
    (la estacion esta a <= ~66 deg del nadir; la normal, a 90)."""
    z = F.unitario(np.asarray(r_gs) - np.asarray(r))
    y0 = -F.unitario(np.cross(r, v))
    x = F.unitario(np.cross(y0, z))
    return F.dcm_a_q(np.column_stack([x, np.cross(z, x), z]))


def referencia_con_derivadas(fq, t, h=1.0):
    """(q_ref, w_ref, a_ref) por diferencias centrales de la funcion fq(t) ->
    cuaternion de referencia. w_ref y a_ref en el marco de la referencia (como
    pide `fisica.control_actitud`). Error O(h^2) de la orbita (n h)^2/6 ~ 1e-7."""
    qs = {k: fq(t + k * h) for k in (-2, -1, 0, 1, 2)}
    q0 = qs[0]
    for k in qs:
        if float(qs[k] @ q0) < 0.0:
            qs[k] = -qs[k]

    def w(qa, qc, qb):
        return 2.0 * F.q_mult(F.q_conj(qc), (qb - qa) / (2.0 * h))[1:]
    w0 = w(qs[-1], qs[0], qs[1])
    wp = w(qs[0], qs[1], qs[2])
    wm = w(qs[-2], qs[-1], qs[0])
    return q0, w0, (wp - wm) / (2.0 * h)


# ====================================================================== enlace

def umbral_apuntamiento_deg(cfg):
    """Error de apuntamiento de la antena del satelite que cuesta
    `perdida_max_db` en el enlace: de L = 12 (e/theta3)^2 (modelo.perdida_
    apuntamiento), e = theta3 sqrt(L/12). Es el umbral de aceptacion del
    modo TRACK_GS: lo fija el enlace, no el resultado."""
    f = model.BANDAS[cfg.banda]
    th3 = model.ancho_haz(cfg.d_sat_m, f)
    return th3 * math.sqrt(cfg.perdida_max_db / 12.0)


def enlace_sat(err_deg, d_km, cfg):
    """Presupuesto de enlace de bajada con la antena del SATELITE desapuntada
    `err_deg` (el de la estacion se supone perfecto: el segmento tierra mide
    ~0.001 deg). Mismo modelo que `model.presupuesto_enlace`: la perdida por
    apuntamiento del satelite sale de su propio ancho de haz theta3 = 70 lambda/D
    y se resta del EIRP. Devuelve dict de `presupuesto_enlace` mas
    `l_sat_db`, `theta3_sat_deg`. Valido dentro del lobulo principal; fuera, la
    perdida se acota con la ganancia de la antena (como el modelo de tierra)."""
    f = model.BANDAS[cfg.banda]
    th3 = model.ancho_haz(cfg.d_sat_m, f)
    g_sat = model.ganancia_plato(cfg.d_sat_m, f, cfg.eta_sat)
    l_sat = float(np.minimum(model.perdida_apuntamiento(err_deg, th3), g_sat))
    eirp = 10.0 * math.log10(cfg.p_tx_w) + g_sat - l_sat
    out = model.presupuesto_enlace(d_km, 0.0, f_hz=f, eirp_dbw=eirp,
                                   tasa_bps=cfg.tasa_bps, diametro_m=cfg.d_gs_m)
    out['l_sat_db'] = l_sat
    out['theta3_sat_deg'] = th3
    return out


def umbrales(sat):
    """(w_umbral, w_lim) en rad/s, derivados de las ruedas y la inercia:
    w_umbral = h_max / (4 J_max): fin del detumbling cuando el momento del cuerpo
    J w cabe 4 veces en las ruedas; w_lim = h_max / (2 J_max): tope de velocidad
    de la guia de apuntamiento (J w <= h_max/2 deja margen para la
    desaturacion y la dinamica)."""
    j = float(np.linalg.eigvalsh(sat.J)[-1])
    return sat.h_max / (4.0 * j), 0.5 * sat.h_max / j


# ====================================================================== planta

class PlantaAdcs:
    """Dinamica + ambiente + sensores. Integra con RK4 (`fisica.propagar`)
    con el ambiente evaluado en el PUNTO MEDIO de cada paso del lazo.

    Perturbaciones: gradiente de gravedad, aerodinamico (placas planas),
    dipolo residual y los magnetopares comandados; el campo es IGRF-13
    truncado a grado 2 (el controlador usa un dipolo: ese es su error de
    modelo). Los mandos se retienen (ZOH) durante el paso."""

    def __init__(self, cfg, sat, efem, rng=None):
        self.cfg, self.sat, self.efem = cfg, sat, efem
        self.rng = np.random.default_rng(cfg.semilla) if rng is None else rng
        q0 = F.q_aleatorio(self.rng)
        d = self.rng.normal(size=3)
        w0 = d / np.linalg.norm(d) * math.radians(cfg.w0_deg_s)
        self.y = F.estado_inicial(q0, w0, np.zeros(sat.n_ruedas))
        self.t = 0.0
        self.giro = F.Giroscopo(cfg.giro_arw, cfg.giro_rrw,
                                self.rng.normal(0.0, cfg.giro_sesgo0, 3), self.rng)
        self.u_aplicado = np.zeros(sat.n_ruedas)
        self.sens = self._sensar(cfg.paso_s)

    # -- ambiente --------------------------------------------------------
    def _campo(self, t, r):
        jd, fr = self.efem.jdfr(t)
        return F.campo_eci_T(r, jd, fr, nmax=F.NMAX_IGRF)

    def avanzar(self, dt, u_w, m_mtq):
        """Integra [t, t+dt] con ruedas y magnetopares retenidos."""
        sat = self.sat
        tm = self.t + 0.5 * dt
        r, v = self.efem.rv(tm)
        B = self._campo(tm, r)
        rho = F.densidad_atmosfera(float(np.linalg.norm(r)) - F.R_TIERRA_KM)
        m = np.clip(np.asarray(m_mtq, float), -sat.m_max, sat.m_max) + sat.m_res

        def tau(q, w):
            return (F.par_gradiente_gravedad(sat.J, q, r)
                    + F.par_aerodinamico(sat, q, r, v, rho)
                    + F.par_magnetico(m, q, B))
        self.y, self.u_aplicado = F.propagar(self.y, sat, u_w, tau, dt, self.cfg.h_int_s)
        self.t += dt
        self.sens = self._sensar(dt)

    # -- sensores y verdad ----------------------------------------------
    def _sensar(self, dt):
        cfg = self.cfg
        q, w, h = self.y[:4], self.y[4:7], self.y[7:]
        r, _ = self.efem.rv(self.t)
        jd, fr = self.efem.jdfr(self.t)
        R = F.q_a_dcm(q)
        b_body = R.T @ self._campo(self.t, r)
        s_eci = F.unitario(F.sol_eci_km(jd + fr))
        ecl = F.en_eclipse(r, F.sol_eci_km(jd + fr))
        mag = F.medir_magnetometro(b_body, cfg.mag_sigma_nt * 1e-9, self.rng)
        gyro = self.giro.medir(w, dt)
        sol = (np.zeros(3) if ecl
               else F.medir_sol(R.T @ s_eci, math.radians(cfg.sol_sigma_deg), self.rng))
        tach = h + self.rng.normal(0.0, cfg.rueda_sigma_nms, h.shape)
        return {'t': self.t, 'mag': mag, 'gyro': gyro, 'sol': sol, 'sol_valido': not ecl,
                'rueda_h': tach, 'q': q.copy(), 'w': w.copy(),
                'sesgo': self.giro.b.copy(), 'eclipse': ecl}

    def medir(self):
        return self.sens


# ====================================================================== controlador

class ControladorAdcs:
    """Software de vuelo: estimador (TRIAD + MEKF) y modos. Solo ve sensores y
    la efemeride; NUNCA la verdad. Responde con un dict:
    modo, u_w (par de motor), m (dipolos), est_valido, q_est, sesgo, sigma_att."""

    def __init__(self, cfg, sat, efem, estacion):
        self.cfg, self.sat, self.efem = cfg, sat, efem
        self.geo = Geometria(efem, estacion)
        self.mask = estacion.mascara_deg
        ev = np.linalg.eigvalsh(sat.J)
        r0, _ = efem.rv(0.0)
        jd0, fr0 = efem.jdfr(0.0)
        a_km = float(np.linalg.norm(r0))
        anio = F.anio_decimal(jd0 + fr0)
        self.bdot = F.Bdot(F.ganancia_bdot(ev[0], F.b_rms_dipolo_T(a_km, anio),
                                           cfg.bdot_tau_s),
                           sat.m_max, cfg.bdot_filtro_s)
        self.w_umbral, self.w_lim = umbrales(sat)
        # error del modelo de campo de a bordo (dipolo) respecto a la verdad:
        # se calcula en la orbita del TLE y se vuelve ruido de medida del filtro
        h0 = np.cross(*efem.rv(0.0))
        inc = math.degrees(math.acos(float(h0[2] / np.linalg.norm(h0))))
        em = F.error_modelo_campo(a_km, inc, cfg.modelo_bordo_nmax, 180, anio)
        self.sigma_modelo_rad = math.radians(em['rms_deg']) / math.sqrt(2.0)
        self.modelo = em
        self.modo = MODO_DETUMBLE
        self.mekf = None
        self.t_prev = None
        self.t_lento = None            # desde cuando |w| < umbral
        self.w_filt = None
        self.t_nadir = None
        self.ultimo = None
        self.sigma_mag_T = cfg.mag_sigma_nt * 1e-9

    # -- utilidades ------------------------------------------------------
    def _B_modelo(self, t, r):
        jd, fr = self.efem.jdfr(t)
        return F.campo_eci_T(r, jd, fr, nmax=self.cfg.modelo_bordo_nmax)

    def _sigma_mag(self, b_norm):
        return math.sqrt((self.sigma_mag_T / b_norm) ** 2 + self.sigma_modelo_rad ** 2)

    def _iniciar_estimador(self, t, s):
        r, _ = self.efem.rv(t)
        jd, fr = self.efem.jdfr(t)
        s_i = F.unitario(F.sol_eci_km(jd + fr))
        b_i = self._B_modelo(t, r)
        try:
            q0 = F.triad(s['sol'], s_i, s['mag'], b_i)
        except ValueError:
            return False
        sa = math.radians(self.cfg.sigma_att0_deg)
        self.mekf = F.Mekf(q0, np.zeros(3), sa, self.cfg.sigma_b0, self.cfg.giro_arw,
                           self.cfg.giro_rrw)
        return True

    def _estimar(self, t, dt, s):
        m = self.mekf
        m.propagar(s['gyro'], dt)
        r, _ = self.efem.rv(t)
        jd, fr = self.efem.jdfr(t)
        b_i = self._B_modelo(t, r)
        bn = float(np.linalg.norm(s['mag']))
        m.actualizar_vector(s['mag'], b_i, self._sigma_mag(bn))
        if s['sol_valido']:
            m.actualizar_vector(s['sol'], F.sol_eci_km(jd + fr),
                                math.radians(self.cfg.sol_sigma_deg))

    def _salida(self, u_w, m_mtq):
        mk = self.mekf
        return {'modo': self.modo, 'u_w': np.asarray(u_w, float),
                'm': np.asarray(m_mtq, float), 'est_valido': mk is not None,
                'q_est': mk.q.copy() if mk else np.array([1.0, 0, 0, 0]),
                'sesgo': mk.b.copy() if mk else np.zeros(3),
                'sigma_att': mk.sigma_actitud() if mk else np.zeros(3)}

    # -- lazo ------------------------------------------------------------
    def paso(self, t, s):
        cfg, sat = self.cfg, self.sat
        if self.t_prev is not None and t <= self.t_prev + 1e-9:
            return self.ultimo            # estado repetido (arranque): mismo mando
        dt = None if self.t_prev is None else t - self.t_prev
        self.t_prev = t

        if self.mekf is not None and dt is not None:
            self._estimar(t, dt, s)

        wn_ = float(np.linalg.norm(s['gyro']))
        if self.w_filt is None:
            self.w_filt = wn_
        else:
            a = (dt or 1.0) / (30.0 + (dt or 1.0))
            self.w_filt += a * (wn_ - self.w_filt)

        # ---- logica de modos
        if self.modo == MODO_DETUMBLE:
            if self.w_filt < self.w_umbral:
                if self.t_lento is None:
                    self.t_lento = t
            else:
                self.t_lento = None
            if (self.t_lento is not None and t - self.t_lento >= cfg.detumble_hold_s
                    and s['sol_valido'] and self._iniciar_estimador(t, s)):
                self.modo = MODO_NADIR
                self.t_nadir = t
        if self.modo != MODO_DETUMBLE:
            visible_ahora = self.geo.el(t) >= self.mask
            visible_pronto = self.geo.el(t + cfg.previo_s) >= self.mask
            self.modo = MODO_TRACK if (visible_ahora or visible_pronto) else MODO_NADIR

        if self.modo == MODO_DETUMBLE:
            m = self.bdot.paso(s['mag'], dt or cfg.paso_s)
            self.ultimo = self._salida(np.zeros(sat.n_ruedas), m)
            return self.ultimo

        # ---- NADIR / TRACK: PD en cuaterniones con ruedas + desaturacion
        mk = self.mekf
        if self.modo == MODO_TRACK:
            def fq(tt):
                g = self.geo.en(tt)
                return q_ref_gs(g['r'], g['v'], g['r_gs'])
        else:
            def fq(tt):
                r, v = self.efem.rv(tt)
                return q_ref_lvlh(r, v)
        q_ref, w_ref, a_ref = referencia_con_derivadas(fq, t)
        w_hat = np.asarray(s['gyro'], float) - mk.b
        h = np.asarray(s['rueda_h'], float)
        tau = F.control_actitud(mk.q, w_hat, h, q_ref, w_ref, a_ref, sat,
                                cfg.wn, cfg.zeta, self.w_lim)
        u = np.clip(F.asignar_ruedas(tau, sat), -sat.tau_max, sat.tau_max)
        m = F.desaturar(s['mag'], sat.A @ h, cfg.desat_tau_s, sat.m_max)
        self.ultimo = self._salida(u, m)
        return self.ultimo


# ====================================================================== medidor / RESUMEN

def _pct(x, p):
    return float(np.percentile(x, p)) if len(x) else float('nan')


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')


class Medidor:
    """Acumula las cifras del RESUMEN con la verdad de la planta (que el
    controlador no ve). Independiente: usa su propia efemeride. Lo usan el
    nodo monitor y la corrida offline."""

    def __init__(self, cfg, sat, efem, estacion):
        self.cfg, self.sat = cfg, sat
        self.geo = Geometria(efem, estacion)
        self.mask = estacion.mascara_deg
        self.w_umbral, self.w_lim = umbrales(sat)
        self.umbral = umbral_apuntamiento_deg(cfg)
        self.t = []
        self.modo = []
        self.w_deg = []                 # |w| verdadera, deg/s
        self.err_boresight = []         # vs objetivo del modo (nadir / estacion), deg
        self.conoc = []                 # error de conocimiento (est vs verdad), deg
        self.en_pase = []
        self.h = []
        self.link = []                  # (l_sat_db, margen_db, cerrado, rango)
        self.t_nadir = None
        self.t_track = None             # primer instante en TRACK_GS
        self.eclipse = []
        self.sigma = []

    def muestra(self, s, cmd):
        t = s['t']
        g = self.geo.en(t)
        modo = cmd['modo']
        if modo == MODO_TRACK:
            objetivo = g['r_gs'] - g['r']
        else:
            objetivo = -g['r']
        err = F.error_apuntamiento_deg(s['q'], BORESIGHT, objetivo) if modo != MODO_DETUMBLE else float('nan')
        pase = g['el_deg'] >= self.mask
        self.t.append(t)
        self.modo.append(modo)
        self.w_deg.append(math.degrees(float(np.linalg.norm(s['w']))))
        self.err_boresight.append(err)
        self.en_pase.append(bool(pase))
        self.eclipse.append(bool(s['eclipse']))
        self.conoc.append(math.degrees(F.angulo_entre(F.q_a_dcm(s['q']) @ BORESIGHT,
                                                      F.q_a_dcm(cmd['q_est']) @ BORESIGHT))
                          if cmd['est_valido'] else float('nan'))
        self.h.append(np.abs(s['rueda_h']).copy())
        self.sigma.append(math.degrees(float(np.max(cmd['sigma_att']))) if cmd['est_valido']
                          else float('nan'))
        if modo == MODO_NADIR and self.t_nadir is None:
            self.t_nadir = t
        if modo == MODO_TRACK and self.t_track is None:
            self.t_track = t
        if modo == MODO_TRACK and pase:
            e = enlace_sat(err, g['rango_km'], self.cfg)
            self.link.append((e['l_sat_db'], e['margen_db'], e['margen_db'] > 0.0, g['rango_km']))

    def recortar(self, t_fin):
        """Descarta las muestras posteriores a `t_fin` (el monitor ROS puede
        recibir unas cuantas mas antes de cerrar la cuenta)."""
        n = int(np.searchsorted(np.array(self.t), t_fin + 1e-9, side='right'))
        for k in ('t', 'modo', 'w_deg', 'err_boresight', 'conoc', 'en_pase', 'h',
                  'eclipse', 'sigma'):
            setattr(self, k, getattr(self, k)[:n])
        # el enlace se acumulo muestra a muestra solo en TRACK_GS en pase
        self.link = self.link[:int(sum(1 for m, p in zip(self.modo, self.en_pase)
                                       if m == MODO_TRACK and p))]

    # -- cifras ------------------------------------------------------------
    def cifras(self):
        t = np.array(self.t)
        modo = np.array(self.modo)
        err = np.array(self.err_boresight)
        pase = np.array(self.en_pase)
        c = {'n': len(t), 'duracion_s': float(t[-1]) if len(t) else 0.0}
        c['w0_deg_s'] = self.w_deg[0] if self.w_deg else float('nan')
        c['t_nadir_s'] = self.t_nadir
        # tiempo de detumbling con la VERDAD: primer instante desde el que |w| (real)
        # queda bajo el umbral durante `detumble_hold_s` seguidos
        hold = max(1, int(round(self.cfg.detumble_hold_s / self.cfg.paso_s)))
        bajo = np.array(self.w_deg) < math.degrees(self.w_umbral)
        c['t_detumble_s'] = None
        racha = 0
        for i, b in enumerate(bajo):
            racha = racha + 1 if b else 0
            if racha >= hold:
                c['t_detumble_s'] = float(t[i - hold + 1])
                break
        if self.t_nadir is not None:
            i = int(np.searchsorted(t, self.t_nadir))
            c['w_en_nadir_deg_s'] = self.w_deg[i]
        # NADIR en regimen: tras (pi / w_lim + 5 / (zeta wn)) desde que entra
        # (peor giro de 180 deg a velocidad limitada + 5 constantes de tiempo)
        t_asent = math.pi / self.w_lim + 5.0 / (self.cfg.zeta * self.cfg.wn)
        c['t_asentamiento_s'] = t_asent
        # NADIR solo ANTES del preapuntado (tras el LOS la antena aun mira a la estacion)
        reg = (modo == MODO_NADIR) & np.isfinite(err)
        if self.t_track is not None:
            reg &= t < self.t_track
        if self.t_nadir is not None:
            reg &= t >= self.t_nadir + t_asent
        c['nadir_n'], c['nadir_rms'], c['nadir_p95'] = int(reg.sum()), _rms(err[reg]), _pct(err[reg], 95)
        c['nadir_max'] = float(err[reg].max()) if reg.any() else float('nan')
        tr = (modo == MODO_TRACK) & pase
        c['track_n'] = int(tr.sum())
        c['track_rms'], c['track_p95'] = _rms(err[tr]), _pct(err[tr], 95)
        c['track_max'] = float(err[tr].max()) if tr.any() else float('nan')
        con = np.array(self.conoc)
        c['conoc_rms'] = _rms(con[np.isfinite(con) & ((modo == MODO_NADIR) | (modo == MODO_TRACK))
                                  & (t >= (self.t_nadir or 0.0) + t_asent)]) if self.t_nadir is not None else float('nan')
        sg = np.array(self.sigma)
        c['sigma_rms'] = _rms(sg[np.isfinite(sg) & ((modo == MODO_NADIR) | (modo == MODO_TRACK))
                                 & (t >= (self.t_nadir or 0.0) + t_asent)]) if self.t_nadir is not None else float('nan')
        h = np.array(self.h)
        c['h_max_nms'] = float(h.max()) if len(h) else 0.0
        c['h_max_frac'] = c['h_max_nms'] / self.sat.h_max
        c['eclipse_en_pase'] = bool(np.any(np.array(self.eclipse)[tr])) if tr.any() else False
        lk = np.array(self.link) if self.link else np.zeros((0, 4))
        c['l_sat_max_db'] = float(lk[:, 0].max()) if len(lk) else float('nan')
        c['l_sat_rms_db'] = _rms(lk[:, 0]) if len(lk) else float('nan')
        c['margen_min_db'] = float(lk[:, 1].min()) if len(lk) else float('nan')
        c['cerrado_pct'] = float(100.0 * lk[:, 2].mean()) if len(lk) else float('nan')
        c['umbral_deg'] = self.umbral
        c['theta3_sat_deg'] = model.ancho_haz(self.cfg.d_sat_m, model.BANDAS[self.cfg.banda])
        c['cumple'] = bool(tr.any() and c['track_p95'] <= self.umbral)
        return c

    def resumen(self):
        """Lineas del RESUMEN (texto determinista: formatos fijos)."""
        c = self.cifras()
        t_nad, t_det = c['t_nadir_s'], c['t_detumble_s']
        if t_det is None:
            l1 = f"RESUMEN detumble: |w| {c['w0_deg_s']:.2f} deg/s, no bajo del umbral {math.degrees(self.w_umbral):.3f} deg/s"
        else:
            espera = (t_nad is not None
                      and t_nad - (t_det + self.cfg.detumble_hold_s) > 5.0 * self.cfg.paso_s)
            l1 = (f"RESUMEN detumble: |w| {c['w0_deg_s']:.2f} -> < {math.degrees(self.w_umbral):.3f} deg/s "
                  f"en t={t_det:.0f} s (real); "
                  + (f"modo NADIR desde t={t_nad:.0f} s" if t_nad is not None else "sigue en DETUMBLE")
                  + (" (espera al Sol para el TRIAD)" if espera else ""))
        l2 = (f"RESUMEN NADIR (regimen, tras {c['t_asentamiento_s']:.0f} s): n={c['nadir_n']} "
              f"rms={c['nadir_rms']:.3f} p95={c['nadir_p95']:.3f} max={c['nadir_max']:.3f} deg")
        l3 = (f"RESUMEN TRACK_GS (en pase, el>={self.mask:g} deg): n={c['track_n']} "
              f"rms={c['track_rms']:.3f} p95={c['track_p95']:.3f} max={c['track_max']:.3f} deg "
              f"| umbral {c['umbral_deg']:.2f} deg (perdida {self.cfg.perdida_max_db:g} dB, "
              f"haz {c['theta3_sat_deg']:.1f} deg)")
        l4 = (f"RESUMEN conocimiento (rms est-verdad, bore-sight): {c['conoc_rms']:.3f} deg "
              f"(1-sigma del filtro, eje peor: {c['sigma_rms']:.3f} deg) "
              f"| ruedas: h max {c['h_max_nms']*1e3:.3f} mNms = {100*c['h_max_frac']:.1f}% de la capacidad")
        l5 = (f"RESUMEN enlace {self.cfg.banda} (antena del satelite {self.cfg.d_sat_m:g} m): "
              f"perdida por apuntamiento rms={c['l_sat_rms_db']:.3f} max={c['l_sat_max_db']:.3f} dB "
              f"| margen min={c['margen_min_db']:.2f} dB | cerrado {c['cerrado_pct']:.1f}%"
              f"{' | ECLIPSE en el pase' if c['eclipse_en_pase'] else ''}")
        l6 = f"RESUMEN veredicto: {'CUMPLE' if c['cumple'] else 'NO CUMPLE'} (p95 TRACK_GS <= umbral)"
        l7 = f"RESUMEN muestras: n={c['n']} hasta t={c['duracion_s']:.0f} s (paso {self.cfg.paso_s:g} s)"
        return [l1, l2, l3, l4, l5, l6, l7]
