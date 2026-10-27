"""Consistencia estadistica del ESKF (P7): error normalizado al cuadrado (NEES) y banda chi-cuadrado, sin scipy.

NEES_k = e_k^T P_k^-1 e_k con e = estado verdadero - estimado (para la actitud, el vector de rotacion del error en el cuerpo,
q_verdadero = q_est (x) dq). Si el filtro es consistente, NEES_k ~ chi2(n) con n los grados de libertad del bloque; el promedio
de N corridas independientes en un mismo instante, ~ chi2(N n)/N. Es la prueba clasica de consistencia de filtros de
seguimiento (referencia de Bar-Shalom: NO VERIFICADA en esta sesion; la propiedad que se usa, que una suma de n cuadrados de
normales estandar independientes es chi2(n), es estadistica elemental y la banda se contrasta con valores tabulados en las pruebas).
"""
import json
import math

import numpy as np

from atp_dron.dron import DT, rot  # noqa: F401  (DT reexportado para los estudios)
from atp_dron.lockstep import Guiado, PlantaLock, cargar_cfg

BLOQUES = {'pos': slice(0, 3), 'vel': slice(3, 6), 'actitud': slice(6, 9), 'sesgo_a': slice(9, 12), 'sesgo_g': slice(12, 15)}


# ---------------------------------------------------------------- chi-cuadrado sin scipy
def _gamma_p(a, x):
    """Gamma incompleta regularizada inferior P(a, x): serie si x < a + 1, fraccion continua si no (Numerical Recipes, sec. 6.2)."""
    if x <= 0.0:
        return 0.0
    lga = math.lgamma(a)
    if x < a + 1.0:
        ap, suma, d = a, 1.0 / a, 1.0 / a
        for _ in range(2000):
            ap += 1.0
            d *= x / ap
            suma += d
            if abs(d) < abs(suma) * 1e-15:
                break
        return suma * math.exp(-x + a * math.log(x) - lga)
    b = x + 1.0 - a
    c, dd = 1e300, 1.0 / b
    h = dd
    for i in range(1, 2000):
        an = -i * (i - a)
        b += 2.0
        dd = an * dd + b
        dd = 1e-300 if abs(dd) < 1e-300 else dd
        c = b + an / c
        c = 1e-300 if abs(c) < 1e-300 else c
        dd = 1.0 / dd
        delta = dd * c
        h *= delta
        if abs(delta - 1.0) < 1e-15:
            break
    return 1.0 - math.exp(-x + a * math.log(x) - lga) * h


def chi2_cdf(x, k):
    return _gamma_p(0.5 * k, 0.5 * x)


def chi2_ppf(p, k):
    """Cuantil de chi2(k) por biseccion sobre la cdf."""
    lo, hi = 0.0, max(10.0, 10.0 * k)
    while chi2_cdf(hi, k) < p:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if chi2_cdf(mid, k) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def banda(n_gdl, n_muestras, nivel=0.95):
    """Banda de dos colas para el PROMEDIO de `n_muestras` NEES independientes de `n_gdl` grados de libertad."""
    a = (1.0 - nivel) / 2.0
    k = n_gdl * n_muestras
    return chi2_ppf(a, k) / n_muestras, chi2_ppf(1.0 - a, k) / n_muestras


# ---------------------------------------------------------------- error del estimador
def _rotvec(q_est, q_ver):
    dq = np.array([q_est[0], -q_est[1], -q_est[2], -q_est[3]])
    a, b = dq, q_ver
    r = np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])
    if r[0] < 0:
        r = -r
    n = np.linalg.norm(r[1:])
    return np.zeros(3) if n < 1e-14 else 2.0 * math.atan2(n, r[0]) * r[1:] / n


def error_vector(pl):
    """e (15,) = verdadero - estimado, en el orden del ESKF, para una PlantaLock con sensores."""
    d, e, s = pl.d, pl.est, pl.sens
    return np.r_[d.p - e.p, d.v - e.v, _rotvec(e.q, d.q), s.sesgo_a - e.ba, s.sesgo_g - e.bg]


def nees_bloques(pl):
    """Por bloque (pos, vel, actitud, sesgos) en este instante: NEES (`n`), traza de P (`trP_n`) y error al cuadrado (`e2_n`)."""
    e, P = error_vector(pl), pl.est.P
    out = {}
    for n, sl in BLOQUES.items():
        Pb = P[sl, sl]
        out[n] = float(e[sl] @ np.linalg.solve(Pb, e[sl]))
        out['trP_' + n] = float(np.trace(Pb))
        out['e2_' + n] = float(e[sl] @ e[sl])
    return out


def nees_corrida(cfg, t_muestreo, t_ini_s=5.0):
    """Corre la cadena completa (planta + guiado, como `lockstep.correr_cadena`) y devuelve la NEES de cada bloque cada
    `t_muestreo` segundos, desde `t_ini_s`. Devuelve (tiempos, {bloque: array}, planta)."""
    cfg = cargar_cfg(None, {**cfg, 'sensores': True})
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    cada = max(1, int(round(t_muestreo / (5 * DT))))
    ts, acc, k = [], {}, 0
    while not pl.fin:
        sp = gu.decidir(json.loads(pl.estado_txt()))
        pl.aplicar(json.loads(json.dumps(sp, separators=(',', ':'))))
        k += 1
        if k % cada == 0 and pl.d.t >= t_ini_s:
            ts.append(pl.d.t)
            for n, v in nees_bloques(pl).items():
                acc.setdefault(n, []).append(v)
    return np.array(ts), {n: np.array(v) for n, v in acc.items()}, pl


def resumen_nees(series, nivel=0.95):
    """series: lista (por semilla) de {bloque: array(tiempos)}. Para cada bloque: promedio de conjunto en cada instante
    (N semillas independientes), fraccion de instantes dentro de la banda por instante, promedio global y su banda.

    El promedio global usa `K_efectivo` = instantes espaciados al menos 'espaciado' como si fueran independientes; como las
    series estan autocorreladas esa banda es ESTRECHA (optimista): por eso se reporta tambien la fraccion por instante."""
    N = len(series)
    out = {}
    for n, sl in BLOQUES.items():
        gdl = sl.stop - sl.start
        M = np.array([s[n] for s in series])                       # [semillas, tiempos]
        prom_t = M.mean(axis=0)
        lo, hi = banda(gdl, N, nivel)
        glob_ = float(M.mean())
        K = M.shape[1]
        lo_g, hi_g = banda(gdl, N * K, nivel)
        trP, e2 = np.array([s['trP_' + n] for s in series]), np.array([s['e2_' + n] for s in series])
        out[n] = {'gdl': gdl, 'P_sobre_var_empirica': round(float(trP.mean() / e2.mean()), 3), 'nees_medio': round(glob_, 4), 'banda_instante': [round(lo, 4), round(hi, 4)],
                  'fraccion_instantes_en_banda': round(float(np.mean((prom_t >= lo) & (prom_t <= hi))), 4),
                  'banda_global_optimista': [round(lo_g, 4), round(hi_g, 4)],
                  'dentro_banda_global': bool(lo_g <= glob_ <= hi_g),
                  'mediana_instante': round(float(np.median(prom_t)), 4)}
    return out
