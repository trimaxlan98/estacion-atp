"""Diversidad de estaciones opticas y comparacion con un enlace de RF en Ka con lluvia.

La pregunta de red: un gateway optico (1-10 Gb/s por pase) rinde mas que un Ka (cientos de Mb/s) pero
una nube lo corta. Con varias estaciones de `data/estaciones.yaml`, que ven el satelite en momentos
distintos de su orbita, cada pase tiene su propia nube; este modulo reparte los pases de un satelite
SGP4 real entre las estaciones, les pone el estado de nubes de `nubes.py` (series con semilla) y mide

  * GB/dia entregados con 1, 2, ... N estaciones, y la fraccion de lo posible (sin nubes) que llega;
  * la probabilidad de que un dia no entregue NADA;
  * el RF Ka comparado: misma geometria, disponibilidad por lluvia (UIT-R P.618, margen de 4 dB -> 99.9 % y
    11 dB -> 99.99 % en clima templado: Vigo; 31 dB -> 99.9 % en clima ecuatorial: literatura leida en resumenes).

SUPUESTO del RF: tasa fija `tasa_rf_bps` (clase 0.5-1 Gb/s de un enlace Ka de microsatelite) y un margen de lluvia
elegido; no es un presupuesto de enlace de RF (el de `atp_ros.link_budget` es de la montura de 3 m).
"""
import datetime as dt
import math
from dataclasses import dataclass

import numpy as np

from atp_optico import atp as P
from atp_optico import escenario as E
from atp_optico import nubes as N
from atp_ros import orbit

# (margen de lluvia dB, indisponibilidad) por clima; la pendiente fuera de los puntos es 1 decada por 7 dB (la de los
# dos puntos templados). SUPUESTO la extrapolacion. Fuentes: busqueda web, Vigo (temperado) y un estudio ecuatorial.
KA_LLUVIA = {'templado': [(4.0, 1e-3), (11.0, 1e-4)], 'ecuatorial': [(31.0, 1e-3)]}
DECADA_DB = 7.0


def indisponibilidad_lluvia(margen_db, clima='templado'):
    """P(atenuacion por lluvia > margen) por interpolacion log-lineal de `KA_LLUVIA` (1 decada por 7 dB)."""
    pts = KA_LLUVIA[clima]
    m0, u0 = pts[0]
    return float(min(1.0, u0 * 10.0 ** (-(margen_db - m0) / DECADA_DB)))


def margen_para_disponibilidad(disp, clima='templado'):
    """Margen de lluvia (dB) necesario para una disponibilidad dada (inversa de la anterior)."""
    m0, u0 = KA_LLUVIA[clima][0]
    return m0 + DECADA_DB * math.log10(u0 / (1.0 - disp))


@dataclass(frozen=True)
class PaseUtil:
    estacion: str
    aos: object
    dur_s: float
    el_max: float
    gb: float


def pases_utiles(sat, est_ligas, estaciones, t0, dias, mascara_deg, atm_por_estacion, enlace, sigma_urad,
                 el_max_min=0.0):
    """Todos los pases de `sat` sobre las estaciones `est_ligas` en [t0, t0 + dias] con el volumen (GB) que
    entrega cada uno (enlace de bajada de `enlace` = (sat, est_optica), error residual sigma_urad)."""
    tle = orbit.buscar_tle(orbit.cargar_catalogo(), sat)
    sat_o, est_o = enlace
    out = []
    for k in est_ligas:
        e = estaciones[k]
        pases = orbit.predecir_pases(tle, e, t0, 24.0 * dias, mascara_deg, el_max_min_deg=el_max_min)
        atm = atm_por_estacion[k]
        for p in pases:
            perfil = orbit.muestrear_pase(tle, e, p, dt_s=2.0)
            v = E.volumen_pase(sat_o, est_o, atm, perfil, sigma_urad, dt_s=2.0)
            out.append(PaseUtil(k, p['aos'], p['duracion_s'], p['el_max_deg'], v['gb']))
    return sorted(out, key=lambda x: x.aos)


def entrega(pases, claro, claves, t0, subconjunto, paso_h=1.0, tasa_rf_gb=None, disp_rf=None, rng=None):
    """Datos entregados por un subconjunto de estaciones: suma de GB de los pases cuya estacion esta despejada a la hora
    del AOS+mitad. Devuelve (gb_entregado, gb_posible, gb_por_dia array). `claro`: matriz (estacion, hora) de `nubes.serie_nubes`."""
    idx = {k: i for i, k in enumerate(claves)}
    n_dias = int(claro.shape[1] * paso_h // 24)
    por_dia = np.zeros(n_dias)
    posible = 0.0
    for p in pases:
        if p.estacion not in subconjunto:
            continue
        t_h = ((p.aos - t0).total_seconds() + 0.5 * p.dur_s) / 3600.0
        h = int(t_h / paso_h)
        d = min(n_dias - 1, int(t_h // 24))
        posible += p.gb
        if h < claro.shape[1] and claro[idx[p.estacion], h]:
            por_dia[d] += p.gb
    return float(por_dia.sum()), posible, por_dia


def entrega_rf(pases, subconjunto, t0, dias, tasa_rf_bps, disp_rf, rng):
    """GB/dia de un enlace Ka de tasa fija sobre los MISMOS pases: tasa x duracion x disponibilidad (con sorteo por pase)."""
    por_dia = np.zeros(int(dias))
    for p in pases:
        if p.estacion not in subconjunto:
            continue
        d = min(int(dias) - 1, int(((p.aos - t0).total_seconds()) // 86400.0))
        if rng.random() < disp_rf:
            por_dia[d] += tasa_rf_bps * p.dur_s / 8e9
    return float(por_dia.sum()), por_dia
