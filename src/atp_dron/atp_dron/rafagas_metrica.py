"""Metrica de rafagas con corrida gemela (paquete P6). Sin ROS.

`error_transversal_rms_m` (metrics.json) mide la desviacion a la recta del tramo y NO crece con la intensidad de la rafaga: tiene un
sesgo geometrico (la parte del tramo 0 y del viento medio) que no depende de sigma. La metrica correcta contra la rafaga es la distancia
a la corrida GEMELA: la misma configuracion y semilla con `rafaga_sigma = 0`. Es 0.000 exacto con sigma = 0 (la gemela es la misma
corrida, determinista) y, como la rafaga de la semilla es `sigma * z(semilla)` con `z` la misma sucesion normal, crece con sigma.

`desv_vs_sin_rafagas_rms_m` = RMS de |p(t) - p_gemela(t)| sobre las filas de telemetria comunes (mismo indice = mismo instante; si una
termina antes por `parar_al_llegar`, se compara el prefijo comun).
"""
import numpy as np

from atp_dron.lazo import correr


def desv_vs_gemela_rms_m(filas, filas_gemela):
    n = min(len(filas), len(filas_gemela))
    if n == 0:
        return 0.0
    a = np.array([[r['x'], r['y'], r['z']] for r in filas[:n]])
    b = np.array([[r['x'], r['y'], r['z']] for r in filas_gemela[:n]])
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


def correr_con_metrica(cfg, filas_gemela=None):
    """Corre `cfg` y su gemela sin rafagas (o usa `filas_gemela`, ya calculada para ahorrar la segunda corrida).
    Devuelve (metricas con `desv_vs_sin_rafagas_rms_m`, filas, filas_gemela)."""
    met, filas = correr(cfg)
    if filas_gemela is None:
        _, filas_gemela = correr({**cfg, 'rafaga_sigma': 0.0})
    met = dict(met)
    met['desv_vs_sin_rafagas_rms_m'] = round(desv_vs_gemela_rms_m(filas, filas_gemela), 4)
    return met, filas, filas_gemela


def t_cuantil_975(gl):
    """Cuantil 0.975 de la t de Student con `gl` grados de libertad, por integracion numerica de la densidad y biseccion."""
    from math import gamma, pi, sqrt
    c = gamma((gl + 1) / 2) / (sqrt(gl * pi) * gamma(gl / 2))
    x = np.linspace(0.0, 60.0, 600001)
    f = c * (1 + x * x / gl) ** (-(gl + 1) / 2)
    cdf = np.concatenate([[0.0], np.cumsum((f[1:] + f[:-1]) * 0.5 * (x[1] - x[0]))])   # P(0 < T < x)
    return float(np.interp(0.475, cdf, x))


def ic95(valores):
    """(media, semiancho) del intervalo de confianza al 95 % de la media (t de Student)."""
    v = np.asarray(valores, float)
    n = len(v)
    if n < 2:
        return float(v.mean()), float('nan')
    return float(v.mean()), float(t_cuantil_975(n - 1) * v.std(ddof=1) / np.sqrt(n))
