"""Apoyo de las pruebas: corridas del seguidor de portadora contra la verdad."""
import math

import numpy as np

from atp_sdr.receptor import SeguidorPortadora
from atp_sdr.senal import Beacon


def envolver(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def corrida_pll(cn0_dbhz, orden=3, bn_hz=20.0, t_int=1e-3, fs=10e3, dur_s=3.0,
                semilla=1, fd_ini=0.0, tasa_hz_s=0.0, jerk_hz_s2=0.0,
                f_nco_ini=None, asistencia='nunca', bn_fll_hz=10.0, bloque_s=0.05,
                evaluar=True, **kw_beacon):
    """Beacon (portadora, Doppler fd_ini + tasa*t + jerk*t^2/2) -> SeguidorPortadora.

    Devuelve dict con arreglos por epoca: 'err_fase' (verdad - NCO a mitad de
    epoca, rad, envuelto), 'err_f_hz' (NCO - verdad), 'f_hz', 'metrica',
    'enganchado' (por bloque) y los objetos 'seg' y 'beacon'. La verdad de
    fase es la media sobre la epoca de la fase generada: lo mismo que mide la
    integracion y descarga."""
    b = Beacon(fs, 'portadora', semilla=semilla, **kw_beacon)
    s = SeguidorPortadora(fs, t_int, orden, bn_hz, bn_fll_hz, asistencia=asistencia)
    s.iniciar(fd_ini if f_nco_ini is None else f_nco_ini)
    n = int(round(fs * bloque_s))
    f = fd_ini
    t = 0.0
    err, errf, fh, mets, eng = [], [], [], [], []
    for _ in range(int(round(dur_s / bloque_s))):
        a = tasa_hz_s + jerk_hz_s2 * t
        f1 = f + a * bloque_s
        x = b.bloque(n, f, f1, cn0_dbhz)
        r = s.procesar(x)
        err.append(envolver(b.fase_verdad.reshape(-1, s.n_ep).mean(axis=1)
                            - r['fase_mid']))
        tm = (np.arange(len(r['f_hz'])) * s.n_ep + 0.5 * (s.n_ep - 1)) / n
        errf.append(r['f_hz'] - (f + (f1 - f) * tm))
        fh.append(r['f_hz'])
        if evaluar:
            ev = s.evaluar(ruido_prompt=1.0 / s.n_ep)
            mets.append(ev['metrica'])
            eng.append(ev['enganchado'])
        f = f1
        t += bloque_s
    return {'err_fase': np.concatenate(err), 'err_f_hz': np.concatenate(errf),
            'f_hz': np.concatenate(fh), 'metrica': np.array(mets),
            'enganchado': np.array(eng), 'seg': s, 'beacon': b}
