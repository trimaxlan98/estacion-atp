"""Adquisicion de un enlace optico: barrido en espiral del campo de
incertidumbre con una baliza ancha, deteccion en la camara del otro extremo y
probabilidad de adquisicion contra el tiempo (analitica y Monte Carlo).

ESQUEMA (el de los terminales de lasercom con baliza):
  1. El terminal A apunta con la efemeride (con su error) y BARRE con la baliza
     una espiral de Arquimedes alrededor de ese punto, a paso-y-mira (cada paso
     dura `cam_dwell_s`).
  2. El terminal B MIRA con una camara de campo ancho (mas que la incertidumbre
     de A: supuesto) y detecta la baliza cuando la potencia en un paso supera el
     umbral (falsa alarma `cam_pfa`).
  3. B mide el centroide y se apunta a A (otra vez con adelanto), emite SU
     baliza; A la detecta (sin barrer: B ya apunta con el error del centroide) y
     ambos pasan a los QPD (apreton de manos, ver `t_apreton_s`).

GEOMETRIA DEL BARRIDO. La baliza es gaussiana de semiangulo 1/e^2 theta_b y
potencia pico P0 sobre el eje en el receptor (ecuacion de enlace con ganancia
8/theta_b^2). La camara detecta con probabilidad >= 0.9 donde la potencia pasa
de P_det, o sea a distancia angular del eje del haz menor que

    r_hit = theta_b sqrt( (1/2) ln(P0 / P_det) )                       (1)

(si P0 < P_det no hay adquisicion posible). Para cubrir el disco con huellas de
radio r_hit se separan los puntos del barrido `paso = kappa r_hit` con
kappa = sqrt(2) (cubrimiento completo de una reticula cuadrada de discos). Cada
paso 'posee' un area paso^2, asi que tras k pasos el radio barrido es

    R(k) = paso sqrt(k/pi)     (area  pi R^2 = k paso^2).               (2)

Con la posicion verdadera del otro satelite gaussiana (error de efemerides y
de actitud) de sigma por eje, la probabilidad de haberlo cubierto es la
distribucion de Rayleigh, y

    P_acq(t) = Pd * ( 1 - exp(-R(t/t_d)^2 / (2 sigma^2)) ).             (3)

(3) es la 'analitica de cobertura' con la que se contrasta el Monte Carlo.
El mejor semiangulo de baliza (el que maximiza el area barrida por paso
paso^2 ~ theta_b^2 ln(A / (theta_b^2 P_det)), con P0 = A/theta_b^2) es

    theta_b* = sqrt( A / (e P_det) )  ->  P0 = e P_det.                 (4)
"""
import math
from functools import lru_cache

import numpy as np
from scipy.special import erfc, erfcinv

from sat_isl import enlace as E

KAPPA = math.sqrt(2.0)
PD_FIABLE = 0.9


def _q_inv(p):
    """Q^-1(p): umbral gaussiano para una cola de probabilidad p."""
    return math.sqrt(2.0) * float(erfcinv(2.0 * p))


def p0_baliza_w(term_tx, term_rx, rango_m, theta_b_urad=None):
    """Potencia PICO (W) de la baliza de `term_tx` en la camara de `term_rx`
    (sobre el eje del haz): P_b eta_b (8/theta_b^2) (lam/4 pi R)^2 G_r eta_cam."""
    th = (term_tx.theta_baliza_urad if theta_b_urad is None else theta_b_urad) * 1e-6
    g_b = 8.0 / th ** 2
    g_r = E.ganancia_apertura(term_rx.D_m, term_rx.lam_m)
    return (term_tx.p_baliza_w * term_tx.eta_baliza * g_b * E.fspl_lineal(rango_m, term_tx.lam_m)
            * g_r * term_rx.eta_rx_cam)


def estadistica_paso(term):
    """(h nu, ruido_e2): energia del foton y varianza del ruido de la camara por
    paso (e-^2): lectura^2 + (oscuridad + fondo) t_d."""
    nu = E.energia_foton(term.lam_m)
    var = term.cam_ruido_e ** 2 + (term.cam_oscuro_e_s + term.cam_fondo_e_s) * term.cam_dwell_s
    return nu, var


def prob_deteccion(p_w, term):
    """Probabilidad de que un paso de la camara declare la baliza con potencia
    `p_w`: electrones de senal N = eta P t_d / (h nu), umbral Th = Q^-1(Pfa) sigma,
    Pd = Q((Th - N) / sqrt(sigma^2 + N)). Gaussiana (valida con N >> 1)."""
    nu, var = estadistica_paso(term)
    n_s = term.cam_eta * np.asarray(p_w, float) * term.cam_dwell_s / nu
    th = _q_inv(term.cam_pfa) * math.sqrt(var)
    return 0.5 * erfc((th - n_s) / np.sqrt(var + n_s) / math.sqrt(2.0))


@lru_cache(maxsize=None)
def potencia_deteccion_w(term, pd=PD_FIABLE):
    """Potencia que da `pd` en un paso (biseccion en log)."""
    lo, hi = -120.0, 0.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if float(prob_deteccion(E.w_de_dbm(mid), term)) < pd:
            lo = mid
        else:
            hi = mid
    return float(E.w_de_dbm(0.5 * (lo + hi)))


def radio_acierto_urad(p0_w, term, theta_b_urad, pd=PD_FIABLE):
    """Ec. (1): radio (urad) donde la baliza se detecta con probabilidad >= pd."""
    p_det = potencia_deteccion_w(term, pd)
    if p0_w <= p_det:
        return 0.0
    return theta_b_urad * math.sqrt(0.5 * math.log(p0_w / p_det))


def theta_baliza_optimo_urad(term_tx, term_rx, rango_m, pd=PD_FIABLE):
    """Ec. (4): semiangulo de baliza que minimiza el tiempo de barrido a
    potencia de baliza y rango dados (P0 = e P_det en el optimo)."""
    a = p0_baliza_w(term_tx, term_rx, rango_m, theta_b_urad=1.0)      # A = P0 theta^2 con theta = 1 urad
    p_det = potencia_deteccion_w(term_rx, pd)
    return math.sqrt(a / (math.e * p_det))


def paso_espiral_urad(r_hit_urad, kappa=KAPPA):
    return kappa * r_hit_urad


def espiral_arquimedes(n, paso_urad):
    """Primeros `n` puntos (x, y) en urad de una espiral de Arquimedes
    r = b phi, b = paso / (2 pi), con el punto k a la longitud de arco EXACTA
    k * paso desde el centro (de modo que cada punto 'posee' ~ paso^2 de area,
    ec. 2). La longitud de arco es s(phi) = (b/2) (phi sqrt(1 + phi^2) + asinh phi);
    se invierte con Newton (vectorizado). El primero es el centro. (Una version que
    avanzara phi con el radio del paso anterior se saltaria la vuelta interior y
    dejaria un hueco alrededor del centro: lo vio el Monte Carlo del escenario.)"""
    b = paso_urad / (2.0 * math.pi)
    s = np.arange(n, dtype=float) * paso_urad
    phi = np.sqrt(2.0 * s / b)                              # arranque: s ~ b phi^2 / 2
    for _ in range(30):
        f = 0.5 * b * (phi * np.sqrt(1.0 + phi * phi) + np.arcsinh(phi)) - s
        df = b * np.sqrt(1.0 + phi * phi)                   # ds/dphi
        phi = np.maximum(phi - f / df, 0.0)
    r = b * phi
    return np.stack([r * np.cos(phi), r * np.sin(phi)], axis=1)


def pasos_para_radio(r_max_urad, paso_urad):
    """k tal que R(k) = r_max (ec. 2)."""
    return int(math.ceil(math.pi * (r_max_urad / paso_urad) ** 2)) + 1


def radio_barrido_urad(k, paso_urad):
    """Ec. (2)."""
    return paso_urad * math.sqrt(max(k, 0) / math.pi)


def p_adquisicion_analitica(t_s, paso_urad, t_dwell_s, sigma_urad, pd=1.0):
    """Ec. (3): probabilidad de haber barrido al otro satelite al tiempo t."""
    k = np.asarray(t_s, float) / t_dwell_s
    r2 = paso_urad ** 2 * k / math.pi
    return pd * (1.0 - np.exp(-r2 / (2.0 * sigma_urad ** 2)))


def sigma_incertidumbre_urad(sigma_pos_km, rango_km, sigma_actitud_urad, sigma_cardan_urad):
    """Incertidumbre del punto de apuntamiento (urad POR EJE): error de la
    posicion relativa transversal (dos satelites independientes: sqrt(2) sigma_pos
    por eje, sobre el rango) mas el de actitud y el del cardan, en cuadratura."""
    pos = math.sqrt(2.0) * sigma_pos_km / rango_km * 1e6
    return math.sqrt(pos ** 2 + sigma_actitud_urad ** 2 + sigma_cardan_urad ** 2)


def t_apreton_s(rango_km, term, n_cuadros=2):
    """Tiempo de apreton de manos tras la deteccion: ida y vuelta de la luz
    (la baliza de B llega a A y A confirma) mas `n_cuadros` pasos de camara por
    extremo y el cierre de los lazos (0.1 s, SUPUESTO)."""
    return 2.0 * rango_km * 1e3 / E.C_LUZ + 2 * n_cuadros * term.cam_dwell_s + 0.1


def monte_carlo(sigma_urad, term_tx, term_rx, rango_m, n=2000, r_max_sigmas=3.5,
                theta_b_urad=None, semilla=1, bloque=2048, t_max_s=None):
    """Monte Carlo del barrido: `n` posiciones del otro satelite ~ N(0, sigma^2)
    por eje, la espiral hasta R = r_max_sigmas sigma (o `t_max_s`), y en cada paso
    una deteccion Bernoulli con la probabilidad de la potencia que cae en el
    punto (baliza gaussiana, `prob_deteccion`). Devuelve (t_det_s, info):
    t_det_s[i] = tiempo de la primera deteccion (inf si no hubo en el barrido);
    info = {paso_urad, r_hit_urad, p0_dbm, pasos}."""
    th_b = term_tx.theta_baliza_urad if theta_b_urad is None else theta_b_urad
    p0 = p0_baliza_w(term_tx, term_rx, rango_m, th_b)
    r_hit = radio_acierto_urad(p0, term_rx, th_b)
    info = {'p0_dbm': float(E.dbm(p0)), 'r_hit_urad': r_hit, 'theta_b_urad': th_b}
    if r_hit <= 0.0:
        info.update(paso_urad=float('nan'), pasos=0)
        return np.full(n, np.inf), info
    paso = paso_espiral_urad(r_hit)
    k_max = pasos_para_radio(r_max_sigmas * sigma_urad, paso)
    if t_max_s is not None:
        k_max = min(k_max, int(t_max_s / term_tx.cam_dwell_s))
    info.update(paso_urad=paso, pasos=k_max)
    pts = espiral_arquimedes(k_max, paso)
    rng = np.random.default_rng(semilla)
    blanco = rng.normal(0.0, sigma_urad, size=(n, 2))
    t_det = np.full(n, np.inf)
    vivo = np.ones(n, bool)
    for k0 in range(0, k_max, bloque):
        k1 = min(k0 + bloque, k_max)
        idx = np.where(vivo)[0]
        if len(idx) == 0:
            break
        d2 = np.sum((blanco[idx, None, :] - pts[None, k0:k1, :]) ** 2, axis=2)     # (m, K)
        p = p0 * np.exp(-2.0 * d2 / th_b ** 2)
        pd = prob_deteccion(p, term_rx)
        hit = rng.random(pd.shape) < pd
        alguno = hit.any(axis=1)
        primero = np.argmax(hit, axis=1)
        sel = idx[alguno]
        t_det[sel] = (k0 + primero[alguno] + 1) * term_tx.cam_dwell_s
        vivo[sel] = False
    return t_det, info
