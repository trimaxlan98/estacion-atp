"""Nubes y disponibilidad de una red de estaciones opticas.

Una nube opaca corta el enlace optico (a diferencia de la lluvia sobre el enlace de RF, que
solo lo debilita): la disponibilidad de UNA estacion es la probabilidad de linea de vista libre
de nubes (CFLOS), 30-65 % en los sitios de `data/clima_optico.yaml`. La salida es la
DIVERSIDAD de estaciones: con varias separadas cientos de km las nubes se decorrelacionan.

MODELO (sencillo, con semilla; cada parametro dice su fuente o SUPUESTO en el YAML):
  estado latente z_i(t) de cada estacion = proceso AR(1) gaussiano estacionario de media 0 y
  varianza 1, z_i(t) = phi z_i(t-1) + sqrt(1 - phi^2) eps_i(t), phi = exp(-paso/tau);
  las innovaciones eps(t) son gaussianas con correlacion espacial C_ij = exp(-d_ij / L)
  (d_ij distancia de circulo maximo entre estaciones);
  la estacion esta DESPEJADA si z_i(t) < Phi^-1(p_i)  (copula gaussiana).
Marginales exactas: P(despejado) = p_i. Para dos estaciones la probabilidad conjunta tiene forma
cerrada (normal bivariada con correlacion rho = C_ij: la autocorrelacion temporal no cambia la
marginal estacionaria): `p_ambas_claras`. Sin correlacion: P(alguna despejada) = 1 - prod(1 - p_i).
"""
import math
import os
from dataclasses import dataclass

import numpy as np
from scipy import signal, stats


@dataclass(frozen=True)
class Sitio:
    clave: str
    lat_deg: float
    lon_deg: float
    p_claro: float


def distancia_km(lat1, lon1, lat2, lon2):
    """Circulo maximo (haversine, R = 6371 km)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * 6371.0 * math.asin(min(1.0, math.sqrt(a)))


def cargar_clima(ruta=None, estaciones=None):
    """(sitios, modelo): `sitios` dict clave -> Sitio con las coordenadas de data/estaciones.yaml; `modelo` el
    dict de parametros del YAML (paso_h, tau_h, L_km)."""
    import yaml
    from atp_ros import orbit
    ruta = ruta or orbit.buscar_datos('clima_optico.yaml')
    with open(ruta) as f:
        y = yaml.safe_load(f)
    est, _ = orbit.cargar_estaciones(estaciones)
    sitios = {}
    for k, v in y['sitios'].items():
        e = est[k]
        sitios[k] = Sitio(k, e.lat_deg, e.lon_deg, float(v['p_claro']))
    return sitios, dict(y['modelo'])


def matriz_correlacion(sitios, L_km):
    n = len(sitios)
    C = np.eye(n)
    for i in range(n):
        for j in range(i + 1, n):
            d = distancia_km(sitios[i].lat_deg, sitios[i].lon_deg, sitios[j].lat_deg, sitios[j].lon_deg)
            C[i, j] = C[j, i] = math.exp(-d / L_km)
    return C


def serie_nubes(sitios, n_pasos, paso_h=1.0, tau_h=8.0, L_km=800.0, semilla=2026):
    """Matriz booleana (n_sitios, n_pasos): True = despejado. Reproducible por semilla."""
    rng = np.random.default_rng(semilla)
    n = len(sitios)
    C = matriz_correlacion(sitios, L_km)
    Lc = np.linalg.cholesky(C + 1e-12 * np.eye(n))
    phi = math.exp(-paso_h / tau_h)
    s = math.sqrt(1.0 - phi * phi)
    z = np.empty((n, n_pasos))
    z[:, 0] = Lc @ rng.standard_normal(n)
    eps = Lc @ rng.standard_normal((n, n_pasos))
    # z_k = phi z_(k-1) + s eps_k, k >= 1 (filtro de 1.er orden con el estado inicial z_0)
    z[:, 1:] = signal.lfilter([s], [1.0, -phi], eps[:, 1:], axis=1, zi=(phi * z[:, 0])[:, None])[0]
    umbral = np.array([stats.norm.ppf(sit.p_claro) for sit in sitios])
    return z < umbral[:, None]


def disponibilidad(claro, idx=None):
    """Fraccion de tiempo con al menos una estacion de `idx` despejada."""
    c = claro if idx is None else claro[list(idx)]
    return float(np.mean(np.any(c, axis=0)))


def disponibilidad_independiente(ps):
    """1 - prod(1 - p_i): cota superior de la diversidad (nubes sin correlacion alguna)."""
    return 1.0 - float(np.prod([1.0 - p for p in ps]))


def p_ambas_claras(p1, p2, rho):
    """P(ambas despejadas) de la copula gaussiana: Phi_2(Phi^-1(p1), Phi^-1(p2); rho)."""
    return float(stats.multivariate_normal(mean=[0.0, 0.0], cov=[[1.0, rho], [rho, 1.0]]).cdf(
        [stats.norm.ppf(p1), stats.norm.ppf(p2)]))


def disponibilidad_analitica_2(p1, p2, rho):
    """P(al menos una despejada) para dos estaciones, exacta de la copula."""
    return p1 + p2 - p_ambas_claras(p1, p2, rho)


def sitios_de(clave_lista, sitios):
    return [sitios[k] for k in clave_lista]
