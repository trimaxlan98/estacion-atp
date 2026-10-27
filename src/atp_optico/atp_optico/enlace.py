"""Enlace optico tierra-espacio: desvanecimiento (log-normal / gamma-gamma),
modems OOK (APD) y PPM (conteo de fotones) con la FEC de CCSDS, y presupuesto
de bajada y de subida con la atmosfera de `atmosfera.py`.

Solo numpy/scipy. Reutiliza de `sat_isl.enlace` el haz gaussiano (theta_h = sqrt(8/G)),
la perdida por apuntamiento y el detector `Apd` (OOK con APD + TIA).

PRESUPUESTO DE BAJADA (satelite -> telescopio de tierra), potencia MEDIA:

  P_r = P_t eta_t G_t L_t  (lam/4 pi R)^2  T_atm  G_r eta_r  L_turb(D, r0, sigma_rx)

  G_t = 8/theta_h^2 (haz gaussiano de semiangulo 1/e^2 theta_h), L_t = 1/(1 + 4 s_t^2/theta_h^2)
  (perdida MEDIA por la inestabilidad de apuntamiento s_t, rms por eje, del satelite),
  G_r = (pi D/lam)^2, y L_turb el acoplo del telescopio al detector: con un detector de
  campo (APD de area grande, `receptor='campo'`) la fraccion de la mancha de corta exposicion
  (radio 1/e^2 w = theta_d / sqrt(S_ST), S_ST = `atmosfera.strehl_corto`) que cae dentro del
  campo theta_f, ensanchada por el error residual del ATP: 1 - exp(-2 theta_f^2/(w^2 + 4 s^2));
  con fibra monomodo (`'smf'`): 0.81 S_ST / (1 + 4 s^2/theta_d^2).

PRESUPUESTO DE SUBIDA (tierra -> satelite): como arriba pero con el haz transmitido de la
estacion, cuyo semiangulo efectivo es theta_eff^2 = theta_up^2 + theta_d^2 (1/S_ST - 1) (la
turbulencia ensancha el haz en cuadratura) y cuya perdida por apuntamiento MEDIA usa la
varianza TOTAL: error residual del ATP + beam wander (`atmosfera.beam_wander`) no corregido
+ error de punteria (point-ahead) de calibracion.

DESVANECIMIENTO. La potencia instantanea es P_r * I con I >= 0 de media 1 y varianza sigma_I^2
(el indice de centelleo de `atmosfera.centelleo`): log-normal en regimen debil, gamma-gamma
(Al-Habash, Andrews y Phillips 2001) en moderado-fuerte; la gamma-gamma es el producto X*Y de dos
gammas independientes de media 1 (parametros alpha, beta), y su CDF sale de
P(I < t) = E_Y[ gammainc(alpha, alpha t / Y) ], sin funciones de Bessel de orden grande.
"""
import math
from dataclasses import dataclass, field, replace
from functools import lru_cache

import numpy as np
from scipy import special, stats
from scipy.optimize import brentq

from atp_optico import atmosfera as A
from sat_isl import enlace as E

C_LUZ = E.C_LUZ


# ---------------------------------------------------------------- desvanecimiento

class Desvanecimiento:
    """Intensidad normalizada I (media 1, var sigma_I^2). `modelo`: 'lognormal' | 'gammagamma' | 'ninguno'."""

    def __init__(self, sigma_I2, modelo='lognormal', alpha=None, beta=None, n_y=700):
        self.sI2 = float(sigma_I2)
        self.modelo = 'ninguno' if (self.sI2 < 1e-12 and modelo != 'gammagamma') else modelo
        self.alpha, self.beta = alpha, beta
        if self.modelo == 'lognormal':
            self.s2 = math.log1p(self.sI2)              # varianza de ln I; media de ln I = -s2/2
        if self.modelo == 'gammagamma':
            if alpha is None or beta is None:
                raise ValueError('gamma-gamma pide alpha y beta')
            # 1 + sI2 = (1 + 1/alpha)(1 + 1/beta)
            self.sI2 = (1.0 + 1.0 / alpha) * (1.0 + 1.0 / beta) - 1.0
            gy = stats.gamma(beta, scale=1.0 / beta)
            lo, hi = gy.ppf(1e-13), gy.ppf(1.0 - 1e-13)
            self._ly = np.linspace(math.log(lo), math.log(hi), n_y)
            y = np.exp(self._ly)
            self._wy = gy.pdf(y) * y
            self._wy = self._wy / np.trapezoid(self._wy, self._ly) if hasattr(np, 'trapezoid') \
                else self._wy / np.trapz(self._wy, self._ly)
            self._y = y

    @classmethod
    def de_centelleo(cls, c, forzar=None):
        """Desde el dict de `atmosfera.centelleo`: log-normal si el regimen es debil."""
        modelo = forzar or ('lognormal' if c['regimen'] == 'debil' else 'gammagamma')
        if modelo == 'gammagamma':
            return cls(c['sigma_I2'], 'gammagamma', c['alpha'], c['beta'])
        return cls(c['sigma_I2'], modelo)

    def cdf(self, t):
        """P(I <= t), t escalar o arreglo."""
        t = np.asarray(t, float)
        if self.modelo == 'ninguno':
            return (t >= 1.0).astype(float)
        if self.modelo == 'lognormal':
            z = (np.log(np.maximum(t, 1e-300)) + 0.5 * self.s2) / math.sqrt(self.s2)
            return 0.5 * special.erfc(-z / math.sqrt(2.0))
        tt = np.atleast_1d(t)
        out = np.empty(tt.shape)
        trap = getattr(np, 'trapezoid', None) or np.trapz
        for i, ti in enumerate(tt.ravel()):
            g = special.gammainc(self.alpha, np.minimum(self.alpha * ti / self._y, 1e300))
            out.flat[i] = trap(g * self._wy, self._ly)
        return out.reshape(np.shape(t)) if np.ndim(t) else float(out.ravel()[0])

    def outage(self, umbral):
        """P(I < umbral): probabilidad de que la potencia instantanea caiga bajo la requerida."""
        return float(self.cdf(umbral))

    def cuantil(self, p):
        """I con P(I <= I_p) = p (para el margen de desvanecimiento 1/I_p a una probabilidad p)."""
        if self.modelo == 'ninguno':
            return 1.0
        return math.exp(float(brentq(lambda x: float(self.cdf(math.exp(x))) - p, -12.0, 3.0, xtol=1e-9)))

    def esperanza(self, g, n=600):
        """E[g(I)] por Riemann-Stieltjes sobre la CDF (g vectorizada)."""
        if self.modelo == 'ninguno':
            return float(g(np.array([1.0]))[0])
        x = np.linspace(-12.0 if self.sI2 > 0.01 else -6.0, 3.5 if self.sI2 > 0.01 else 2.0, n)
        I = np.exp(x)
        F = np.atleast_1d(self.cdf(I))
        dF = np.diff(np.concatenate([[0.0], F, [1.0]]))
        Im = np.concatenate([[I[0]], 0.5 * (I[1:] + I[:-1]), [I[-1]]])
        return float(np.sum(g(Im) * dF))


def margen_desvanecimiento_db(des, p_outage):
    """Margen (dB) que hay que reservar para que la probabilidad de caer bajo el umbral sea p_outage."""
    return -10.0 * math.log10(des.cuantil(p_outage))


# ---------------------------------------------------------------- FEC: Reed-Solomon

def ber_salida_rs(p_bit, n=255, k=239, m=8):
    """BER a la salida de un RS(n, k) sobre simbolos de m bits con errores de bit independientes
    de probabilidad p_bit antes de decodificar (Sklar, Digital Communications): corrige t = (n-k)/2
    simbolos; P_simbolo_salida = sum_{i>t} (i/n) C(n,i) ps^i (1-ps)^(n-i) con ps = 1-(1-p)^m, y
    BER = P_simbolo_salida 2^(m-1)/(2^m - 1)."""
    t = (n - k) // 2
    ps = 1.0 - (1.0 - p_bit) ** m
    i = np.arange(t + 1, n + 1)
    pse = float(np.sum(i / n * stats.binom.pmf(i, n, ps)))
    return pse * 2.0 ** (m - 1) / (2.0 ** m - 1.0)


# ---------------------------------------------------------------- modems

@dataclass(frozen=True)
class ModemOOK:
    """OOK con APD (CCSDS 141.0-B: OOK-NRZ de baja complejidad; la FEC de CCSDS 142.0-B, tipo
    Reed-Solomon, baja un BER bruto <= `ber_bruto` al nivel util: ver `ber_salida_rs`).
    Escalera de tasas de `sat_isl.enlace.ESCALERA_TASAS` (10 Mb/s .. 10 Gb/s)."""
    apd: E.Apd = field(default_factory=E.Apd)
    ber_bruto: float = 1e-4
    nombre: str = 'OOK-APD'
    tipo: str = 'ook'

    def tasas(self):
        return E.ESCALERA_TASAS

    def ber(self, p_w, tasa, lam_m):
        return self.apd.ber(p_w, tasa, lam_m)

    def sensibilidad_w(self, tasa, lam_m):
        return self.apd.sensibilidad_w(tasa, lam_m, self.ber_bruto)

    def tasa_util(self, tasa, p_w=None, lam_m=None):
        return tasa * 239.0 / 255.0                       # RS(255,239): la FEC se lleva 6.3 %


def ser_ppm(ns, nb, M):
    """Tasa de error de simbolo (decision dura, desempate al azar) de M-PPM con conteo de Poisson:
    `ns` fotones de senal por PULSO, `nb` de fondo+oscuridad por RANURA. Exacta: la ranura de
    senal da K_s ~ Poisson(ns + nb), cada una de las M-1 vacias K_b ~ Poisson(nb), y
    P(acierto | K_s = k) = sum_j C(M-1,j) P(K_b = k)^j P(K_b < k)^(M-1-j) / (j+1)  (j rivales empatados).
    Cota inferior numerica 1e-15 (1 - suma en doble precision)."""
    lam_s = ns + nb
    kmax = int(lam_s + 14.0 * math.sqrt(lam_s + 1.0) + 40)
    k = np.arange(kmax + 1)
    ps = stats.poisson.pmf(k, lam_s)
    a = stats.poisson.pmf(k, nb) if nb > 0 else (k == 0).astype(float)
    b = stats.poisson.cdf(k - 1, nb) if nb > 0 else (k >= 1).astype(float)   # P(Kb < k)
    c = np.minimum(a + b, 1.0)
    # P(acierto | K_s = k) = sum_j C(M-1, j) a^j b^(M-1-j) / (j+1): todos los terminos positivos (estable)
    pc = np.zeros_like(a)
    for j in range(M):
        pc += special.comb(M - 1, j) * a ** j * b ** (M - 1 - j) / (j + 1.0)
    return float(max(1e-15, 1.0 - np.sum(ps * np.minimum(pc, 1.0))))


def ber_ppm(ns, nb, M):
    """BER = SER M / (2 (M - 1)) (simbolos ortogonales)."""
    return ser_ppm(ns, nb, M) * M / (2.0 * (M - 1.0))


def capacidad_ppm_dura(ns, nb, M):
    """Informacion (bit/simbolo) del canal M-ario simetrico que deja la decision dura:
    log2 M + (1-e) log2(1-e) + e log2(e/(M-1)), e = SER. Cota INFERIOR de la capacidad con decision
    blanda (la FEC de CCSDS 142.0-B, SCPPM, decodifica en blando y opera a ~1 dB de la capacidad:
    Moision y Hamkins 2005, de memoria, SIN verificar)."""
    e = min(max(ser_ppm(ns, nb, M), 1e-300), (M - 1.0) / M)
    return math.log2(M) + (1 - e) * math.log2(1 - e) + e * math.log2(e / (M - 1.0))


@dataclass(frozen=True)
class ModemPPM:
    """M-PPM con detector de conteo (SNSPD o APD en modo Geiger): CCSDS 141.0-B (PPM de alta eficiencia
    fotonica) con SCPPM de 142.0-B. `eta_det` eficiencia de deteccion, `fondo_cps` cuentas de
    fondo+oscuridad por segundo (SUPUESTO nocturno: 5e3), ranura = 1/tasa de ranuras."""
    M: int = 16
    eta_det: float = 0.7
    fondo_cps: float = 5e3
    nombre: str = 'PPM-16'
    tipo: str = 'ppm'
    ranuras: tuple = (1e7, 3e7, 1e8, 3e8, 1e9)         # ranuras/s

    def tasas(self):
        return self.ranuras

    def tasa_bruta(self, ranura_hz):
        return math.log2(self.M) / self.M * ranura_hz

    def fotones(self, p_w, ranura_hz, lam_m):
        """(ns por pulso, nb por ranura) detectados con potencia media p_w."""
        ns = self.eta_det * p_w * self.M / ranura_hz / E.energia_foton(lam_m)
        return ns, self.fondo_cps / ranura_hz

    def ber(self, p_w, ranura_hz, lam_m):
        p = np.atleast_1d(np.asarray(p_w, float))
        out = np.array([ber_ppm(*self.fotones(float(x), ranura_hz, lam_m), self.M) for x in p])
        return out if np.ndim(p_w) else float(out[0])

    def tasa_util(self, ranura_hz, p_w, lam_m):
        """Tasa de informacion con codigo ideal sobre el canal de decision dura (bit/s)."""
        ns, nb = self.fotones(p_w, ranura_hz, lam_m)
        return capacidad_ppm_dura(ns, nb, self.M) / self.M * ranura_hz

    def sensibilidad_w(self, ranura_hz, lam_m, ber_obj=1e-4):
        lo, hi = -100.0, 10.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.ber(float(E.w_de_dbm(mid)), ranura_hz, lam_m) > ber_obj:
                lo = mid
            else:
                hi = mid
        return float(E.w_de_dbm(0.5 * (lo + hi)))


def tasa_con_outage(modem, p_medio_w, lam_m, des, p_out=1e-2, ber_obj=None):
    """Mayor tasa de la escalera del modem tal que la probabilidad de que la potencia instantanea
    (p_medio * I) quede bajo la sensibilidad (BER bruto objetivo) sea <= p_out. Devuelve
    (indice, tasa_bruta_bps, outage_a_esa_tasa); indice -1 si ninguna sirve.
    La sensibilidad se evalua con el BER bruto de la FEC (`ber_bruto` del modem)."""
    ber_obj = ber_obj or getattr(modem, 'ber_bruto', 1e-4)
    mejor = (-1, 0.0, 1.0)
    for i, r in enumerate(modem.tasas()):
        if modem.tipo == 'ook':
            ps = modem.sensibilidad_w(r, lam_m)
        else:
            ps = modem.sensibilidad_w(r, lam_m, ber_obj)
        o = des.outage(ps / max(p_medio_w, 1e-30))
        if o <= p_out:
            tb = r if modem.tipo == 'ook' else modem.tasa_bruta(r)
            mejor = (i, tb, o)
    return mejor


# ---------------------------------------------------------------- terminales

@dataclass(frozen=True)
class TerminalSat:
    """Terminal optico del satelite (transmisor de bajada, receptor de subida)."""
    nombre: str = 'sat 1550 clase 6U (supuesto)'
    lam_m: float = A.LAM_1550
    p_tx_w: float = 1.0              # potencia optica media. SUPUESTO
    theta_h_urad: float = 100.0      # semiangulo 1/e^2 del haz (haz ancho, ~1.5 cm difractivo; 5 cm difractivo: 29.6 urad). SUPUESTO
    eta_tx: float = 0.5              # eficiencia optica de transmision (-3 dB). SUPUESTO
    sigma_apunt_urad: float = 3.0    # inestabilidad de apuntamiento rms por eje (ADCS + FSM propio). SUPUESTO
    D_rx_m: float = 0.05             # apertura de recepcion de la subida
    eta_rx: float = 0.4
    modem_sub: object = field(default_factory=ModemOOK)


@dataclass(frozen=True)
class EstacionOptica:
    """Telescopio de la estacion optica."""
    nombre: str = 'OGS 40 cm (supuesto)'
    D_m: float = 0.4
    eta_rx: float = 0.35             # obstruccion central, espejos, filtro y relevo al detector. SUPUESTO
    receptor: str = 'campo'          # 'campo' (APD de area grande) | 'smf' (fibra monomodo, coherente/EDFA)
    campo_urad: float = 12.0         # radio del campo del detector: APD de 50 um de diametro a f = 2 m (f/5 en 0.4 m)
    modem: object = field(default_factory=ModemOOK)
    # subida (laser de baliza/enlace)
    p_up_w: float = 20.0
    theta_up_urad: float = 12.0      # semiangulo del haz de subida: se ENSANCHA a proposito (beam wander)
    D_tx_m: float = 0.4              # apertura que transmite (fija W0 del beam wander)
    cal_pa_urad: float = 0.5         # error de calibracion del point-ahead (rms por eje). SUPUESTO
    eta_tx_up: float = 0.5
    corr_tilt_up: float = 0.0        # fraccion de la varianza del beam wander que el FSM pre-compensa
    sigma_apunt_urad: float = 1.0    # error residual del ATP (rms por eje); lo fija `atp.py`


def theta_difractivo_urad(D_m, lam_m):
    """Semiangulo equivalente 1/e^2 del haz de una apertura iluminada con gamma = 1.12 (0.956 lam/D)."""
    return 1e6 * E.semiangulo_haz_rad(D_m, lam_m)


# ---------------------------------------------------------------- presupuestos

def _db(x):
    return 10.0 * math.log10(max(x, 1e-300))


def acoplo_receptor(est, lam_m, atm, el_deg, sigma_rx_urad):
    """Fraccion de la potencia colectada que llega al detector (turbulencia + error residual del ATP)."""
    r0 = A.fried_r0(atm, lam_m, el_deg)
    S = A.strehl_corto(est.D_m, r0)
    th_d = 1e6 * E.semiangulo_haz_rad(est.D_m, lam_m)             # urad
    s2 = sigma_rx_urad ** 2
    if est.receptor == 'smf':
        return 0.81 * S / (1.0 + 4.0 * s2 / th_d ** 2)
    w2 = th_d ** 2 / S + 4.0 * s2
    return 1.0 - math.exp(-2.0 * est.campo_urad ** 2 / w2)


def presupuesto_bajada(sat, est, atm, el_deg, rango_km, sigma_rx_urad=None):
    """Potencia media recibida (W) y desglose en dB. `sigma_rx_urad` (rms por eje) por defecto el de `est`."""
    lam = sat.lam_m
    s_rx = est.sigma_apunt_urad if sigma_rx_urad is None else sigma_rx_urad
    R = rango_km * 1e3
    th = sat.theta_h_urad * 1e-6
    g_t = 8.0 / th ** 2
    l_t = 1.0 / (1.0 + 4.0 * (sat.sigma_apunt_urad * 1e-6) ** 2 / th ** 2)
    fspl = E.fspl_lineal(R, lam)
    t_atm = A.transmision(atm, lam, el_deg)
    g_r = E.ganancia_apertura(est.D_m, lam)
    l_acop = acoplo_receptor(est, lam, atm, el_deg, s_rx)
    p = sat.p_tx_w * sat.eta_tx * g_t * l_t * fspl * t_atm * g_r * est.eta_rx * l_acop
    return {'p_rx_w': p, 'p_rx_dbm': float(E.dbm(p)),
            'db': {'P_tx': _db(sat.p_tx_w), 'eta_tx': _db(sat.eta_tx), 'G_tx': _db(g_t), 'apunt_sat': _db(l_t),
                   'espacio_libre': _db(fspl), 'atmosfera': _db(t_atm), 'G_rx': _db(g_r), 'eta_rx': _db(est.eta_rx),
                   'acoplo_turbulencia_ATP': _db(l_acop)}}


def presupuesto_subida(sat, est, atm, el_deg, rango_km, sigma_rx_urad=None):
    """Potencia media (W) que llega al detector del satelite desde el laser de subida de la estacion."""
    lam = sat.lam_m
    s_rx = est.sigma_apunt_urad if sigma_rx_urad is None else sigma_rx_urad
    R = rango_km * 1e3
    r0 = A.fried_r0(atm, lam, el_deg)
    S = A.strehl_corto(est.D_tx_m, r0)
    th_d = 1e6 * E.semiangulo_haz_rad(est.D_tx_m, lam)
    th_eff = math.sqrt(est.theta_up_urad ** 2 + th_d ** 2 * (1.0 / S - 1.0))
    bw = A.beam_wander(atm, est.D_tx_m, el_deg, R)
    var_bw = bw['sigma_urad'] ** 2 * (1.0 - est.corr_tilt_up)
    var_tot = s_rx ** 2 + var_bw + est.cal_pa_urad ** 2
    l_pt = 1.0 / (1.0 + 4.0 * var_tot / th_eff ** 2)
    g_t = 8.0 / (th_eff * 1e-6) ** 2
    fspl = E.fspl_lineal(R, lam)
    t_atm = A.transmision(atm, lam, el_deg)
    g_r = E.ganancia_apertura(sat.D_rx_m, lam)
    p = est.p_up_w * est.eta_tx_up * g_t * l_pt * fspl * t_atm * g_r * sat.eta_rx
    return {'p_rx_w': p, 'p_rx_dbm': float(E.dbm(p)), 'theta_eff_urad': th_eff,
            'beam_wander_urad': bw['sigma_urad'], 'perdida_apunt_db': _db(l_pt),
            'db': {'P_tx': _db(est.p_up_w), 'eta_tx': _db(est.eta_tx_up), 'G_tx': _db(g_t), 'apunt_beam_wander': _db(l_pt),
                   'espacio_libre': _db(fspl), 'atmosfera': _db(t_atm), 'G_rx': _db(g_r), 'eta_rx': _db(sat.eta_rx)}}


@lru_cache(maxsize=4096)
def _desvanecimiento(lam_m, D_m, atm, el_deg):
    c = A.centelleo(atm, lam_m, el_deg, D_m)
    return Desvanecimiento.de_centelleo(c), c


def desvanecimiento_enlace(sat, est, atm, el_deg, subida=False):
    """`Desvanecimiento` del enlace: indice de centelleo de la apertura que RECIBE (bajada) o que TRANSMITE (subida,
    por reciprocidad: el satelite es un receptor puntual y la apertura de la estacion promedia)."""
    D = est.D_tx_m if subida else est.D_m
    return _desvanecimiento(sat.lam_m, D, atm, float(el_deg))


def evaluar_enlace(sat, est, atm, el_deg, rango_km, sigma_rx_urad=None, p_out=1e-2, subida=False, modem=None,
                   con_margen=True):
    """Una fila del estudio: potencia media, margen y tasa con outage <= p_out a esa elevacion."""
    pres = (presupuesto_subida if subida else presupuesto_bajada)(sat, est, atm, el_deg, rango_km, sigma_rx_urad)
    des, c = desvanecimiento_enlace(sat, est, atm, el_deg, subida)
    modem = modem or (sat.modem_sub if subida else est.modem)
    idx, tasa, out = tasa_con_outage(modem, pres['p_rx_w'], sat.lam_m, des, p_out)
    sens = (modem.sensibilidad_w(modem.tasas()[0], sat.lam_m) if modem.tipo == 'ook'
            else modem.sensibilidad_w(modem.tasas()[0], sat.lam_m))
    return {'el_deg': el_deg, 'rango_km': rango_km, 'p_rx_dbm': pres['p_rx_dbm'], 'sigma_I2': c['sigma_I2'],
            'regimen': c['regimen'], 'sigma_R2': c['sigma_R2'], 'tasa_bps': tasa, 'outage': out if idx >= 0 else 1.0,
            'idx_tasa': idx, 'margen_min_db': pres['p_rx_dbm'] - float(E.dbm(sens)),
            'margen_desv_db': margen_desvanecimiento_db(des, p_out) if con_margen else None, 'presupuesto': pres}


def ber_con_centelleo(modem, p_medio_w, tasa, lam_m, des):
    """BER BRUTO medio sobre el desvanecimiento: E_I[ BER(p_medio * I) ]."""
    return des.esperanza(lambda I: np.atleast_1d(modem.ber(p_medio_w * I, tasa, lam_m)))


# ---------------------------------------------------------------- misiones de referencia (solo contraste)

# OPALS (ISS -> OCTL de Table Mountain, 2014): 410 km, 1550 nm, 2.5 W, divergencia FWHM ~ 1 mrad,
# ganancia de transmision 70.4 dB, OOK de 50 Mb/s con Reed-Solomon, telescopio de 1 m y APD de espacio libre de
# 200 um; potencia recibida predicha de cientos de nW y medida de decenas de nW (OPALS, JPL; Giggenbach et al.
# 2023, IJSCN; resumenes consultados, no los articulos completos). FWHM = 2 sqrt(ln2/2) theta_h -> theta_h = 849 urad.
OPALS_SAT = TerminalSat(nombre='OPALS (ISS)', p_tx_w=2.5, theta_h_urad=1000.0 / (2.0 * math.sqrt(math.log(2.0) / 2.0)),
                        eta_tx=0.5, sigma_apunt_urad=0.0)
OPALS_EST = EstacionOptica(nombre='OCTL 1 m', D_m=1.0, eta_rx=0.35, campo_urad=100.0,
                           modem=ModemOOK(), sigma_apunt_urad=1.0)
