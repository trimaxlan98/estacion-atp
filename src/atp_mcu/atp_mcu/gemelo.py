"""Gemelo digital del banco HIL: la montura simulada de `atp_ros` con el PID de
`pid_controller.py`, en lockstep a 20 Hz, alimentada con LAS MISMAS referencias
que recibio el microcontrolador. Logica pura (numpy, sin ROS), para compararla
con lo que midio el hardware (o el nodo de Linux) y sacar la *deriva*.

El gemelo reproduce, operacion por operacion, la cadena de CONTRATO.md sec. 8:
mount_sim integra [t_{k-1}, t_k] con el mando vigente a subpasos de 2 ms
(`PlantaMontura.paso`), publica el estado de t_k, y el PID calcula el mando de
t_k con el estado y la referencia del MISMO sello (`PidController.control`).

Lo que NO es igual al firmware (y por eso hay deriva): el PID del gemelo corre
a 20 Hz con retencion de orden cero; el del firmware a 1 kHz con la referencia
extrapolada, en float, con tope de velocidad de la referencia y filtro de
velocidad del encoder. La deriva mide justo esa diferencia mas el enlace.
"""
import math

import numpy as np

from atp_ros import model
from atp_ros.planta import PlantaMontura


def gemelo_pid(refs, dt=0.05, h=0.002, kp=25.0, ki=2.0, kd=8.0, tau=0.08,
               u_max=8.0, J=model.J_EJE, b=model.B_EJE):
    """refs: (N, 2) rad en los instantes t_0 + k dt (k = 0..N-1; la montura se
    engancha en refs[0], como `MountSim.on_ref`). Devuelve (meas, u): el estado
    publicado en cada instante y el mando calculado, ambos (N, 2)."""
    refs = np.asarray(refs, dtype=float)
    n = len(refs)
    planta = PlantaMontura(J, b, math.inf, h)
    planta.enganchar(refs[0])
    integral = np.zeros(2)
    e_prev = np.zeros(2)
    d_filt = np.zeros(2)
    u = np.zeros(2)
    meas = np.empty((n, 2))
    us = np.empty((n, 2))
    pasos = int(round(dt / h))
    resto = dt - pasos * h
    alfa = dt / (tau + dt)
    for k in range(n):
        if k > 0:
            for hh in [h] * pasos + ([resto] if resto > 1e-9 else []):
                planta.paso(u, 0.0, hh)
        m = planta.theta.copy()
        meas[k] = m
        e = refs[k] - m
        d_bruta = (e - e_prev) / dt
        d_filt = d_filt + alfa * (d_bruta - d_filt)
        u_raw = kp * e + ki * integral + kd * d_filt
        u = np.clip(u_raw, -u_max, u_max)
        sat = np.abs(u_raw) > u_max + 1e-12
        congelado = sat & (u_raw * e > 0.0)
        integral = integral + np.where(congelado, 0.0, e * dt)
        e_prev = e
        us[k] = u
    return meas, us


def interpolar(t_obj, t, y):
    """Interpolacion lineal de y(t) en t_obj; None si t_obj queda fuera de [t[0], t[-1]].
    t creciente; y de forma (N,) o (N, m)."""
    t = np.asarray(t, dtype=float)
    if len(t) < 2 or t_obj < t[0] or t_obj > t[-1]:
        return None
    y = np.asarray(y, dtype=float)
    i = int(np.searchsorted(t, t_obj))
    if t[i] == t_obj:
        return y[i]
    t0, t1 = t[i - 1], t[i]
    w = (t_obj - t0) / (t1 - t0)
    return y[i - 1] * (1.0 - w) + y[i] * w


def percentiles(x, qs=(50, 95, 99)):
    """{'n', 'min', 'p50', ..., 'max', 'media'} de una lista (vacia: n = 0)."""
    x = np.asarray(list(x), dtype=float)
    if x.size == 0:
        return {'n': 0}
    r = {'n': int(x.size), 'min': float(x.min()), 'media': float(x.mean()), 'max': float(x.max())}
    for q in qs:
        r[f'p{q}'] = float(np.percentile(x, q))
    return r


def deriva(meas_hil, meas_gemelo, refs, el_col=1, dt=0.05):
    """Compara el HIL con el gemelo (ambos (N, 2) rad, mismos instantes).
    Devuelve, en grados: diferencia rms y maxima por eje y en el cielo
    (sqrt((d_az cos el)^2 + d_el^2)), y el error de apuntamiento propio de
    cada uno contra la referencia."""
    meas_hil = np.asarray(meas_hil, dtype=float)
    meas_gemelo = np.asarray(meas_gemelo, dtype=float)
    refs = np.asarray(refs, dtype=float)
    d = np.degrees(meas_hil - meas_gemelo)
    el = meas_gemelo[:, el_col]

    def cielo(dd):
        return np.sqrt((dd[:, 0] * np.cos(el)) ** 2 + dd[:, 1] ** 2)

    e_hil = np.degrees(refs - meas_hil)
    e_gem = np.degrees(refs - meas_gemelo)
    rms = lambda v: float(np.sqrt(np.mean(np.square(v))))  # noqa: E731
    return {
        'n': int(len(d)),
        'dif_az_rms_deg': rms(d[:, 0]), 'dif_az_max_deg': float(np.max(np.abs(d[:, 0]))),
        'dif_el_rms_deg': rms(d[:, 1]), 'dif_el_max_deg': float(np.max(np.abs(d[:, 1]))),
        'dif_cielo_rms_deg': rms(cielo(d)), 'dif_cielo_max_deg': float(np.max(cielo(d))),
        # p50/p95/p99 resisten a los tirones del planificador de Linux que inflan el maximo
        'dif_cielo_p50_deg': float(np.percentile(cielo(d), 50)),
        'dif_cielo_p95_deg': float(np.percentile(cielo(d), 95)),
        'dif_cielo_p99_deg': float(np.percentile(cielo(d), 99)),
        'err_hil_cielo_rms_deg': rms(cielo(e_hil)), 'err_hil_cielo_max_deg': float(np.max(cielo(e_hil))),
        'err_gemelo_cielo_rms_deg': rms(cielo(e_gem)), 'err_gemelo_cielo_max_deg': float(np.max(cielo(e_gem))),
        # deriva = como crece la diferencia a lo largo del pase (pendiente de |d| por minuto)
        'deriva_cielo_deg_por_min': _pendiente(cielo(d)) * 60.0 / dt,
    }


def _pendiente(y):
    """Pendiente de minimos cuadrados de y contra el indice (unidades de y por muestra)."""
    y = np.asarray(y, dtype=float)
    if len(y) < 3:
        return 0.0
    x = np.arange(len(y), dtype=float)
    return float(np.polyfit(x, y, 1)[0])
