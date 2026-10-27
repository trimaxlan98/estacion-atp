"""Adquisicion de frecuencia por FFT contra la cota de Cramer-Rao y la Pfa."""
import math

import numpy as np
import pytest

from atp_sdr.receptor import adquirir_fft
from atp_sdr.senal import Beacon

FS = 100e3


def _tono(rng, n, cn0, f, ruido=True):
    b = Beacon(FS, 'portadora', semilla=int(rng.integers(1 << 30)))
    b.fase = float(rng.uniform(0, 2 * math.pi))
    return b.bloque(n, f, f, cn0, ruido=ruido)


def test_error_de_frecuencia_contra_la_cota_de_cramer_rao():
    """Rife & Boorstyn 1974: var(f) >= 6 fs^2 / ((2 pi)^2 rho N (N^2 - 1)) con
    rho = A^2/sigma^2 por muestra. Con T = 10 ms (N = 1000) y C/N0 = 45 dB-Hz
    (rho = 0.316) la cota es 2.2 Hz. El estimador (FFT x4 con interpolacion
    parabolica) no puede bajar de ella y debe quedar a un factor 3."""
    rng = np.random.default_rng(5)
    n, cn0 = 1000, 45.0
    rho = 10 ** (cn0 / 10) / FS
    crb = math.sqrt(6.0 * FS ** 2 / ((2 * math.pi) ** 2 * rho * n * (n * n - 1)))
    errs = []
    for _ in range(300):
        f = float(rng.uniform(-40e3, 40e3))
        a = adquirir_fft(_tono(rng, n, cn0, f), FS, (-50e3, 50e3))
        assert a['detectado']
        errs.append(a['f_hz'] - f)
    rms = float(np.sqrt(np.mean(np.square(errs))))
    assert 0.9 * crb < rms < 3.0 * crb
    assert abs(np.mean(errs)) < 3 * rms / math.sqrt(300)         # sin sesgo


def test_probabilidad_de_falsa_alarma():
    """Solo ruido: la fraccion de falsas alarmas sigue a la Pfa de diseno
    (umbral gamma = ln(M_eff/Pfa) sobre el piso medido; M_eff cuenta los bins
    independientes de la ventana x2.8 por el relleno de ceros). 3000 pruebas,
    Pfa = 1 %: esperadas 30 +- 5."""
    rng = np.random.default_rng(6)
    n = 1000
    fa = 0
    for _ in range(3000):
        x = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * math.sqrt(0.5)
        fa += int(adquirir_fft(x, FS, (-50e3, 50e3), pfa=1e-2)['detectado'])
    assert 15 <= fa <= 50


def test_deteccion_segura_sobre_el_umbral_y_nula_sin_senal():
    rng = np.random.default_rng(7)
    n = 1000
    # umbral = ln(2.8 M/Pfa) = ln(2800/1e-3) = 14.8 -> pico/ruido de 11.7 dB;
    # pico/ruido ~ (C/N0) T = 10^(C/N0/10) 0.01: 38 dB-Hz queda en el umbral
    pd_alto = np.mean([adquirir_fft(_tono(rng, n, 45.0, 12e3), FS, (-50e3, 50e3))['detectado']
                       for _ in range(200)])
    pd_bajo = np.mean([adquirir_fft(_tono(rng, n, 27.0, 12e3), FS, (-50e3, 50e3))['detectado']
                       for _ in range(200)])
    assert pd_alto == 1.0
    assert pd_bajo < 0.05


def test_la_busqueda_respeta_la_ventana_de_doppler():
    """Un tono fuera de la ventana esperada no se acepta como Doppler."""
    rng = np.random.default_rng(8)
    x = _tono(rng, 1000, 55.0, 30e3)
    dentro = adquirir_fft(x, FS, (-20e3, 40e3))
    fuera = adquirir_fft(x, FS, (-40e3, 20e3))
    assert dentro['detectado'] and abs(dentro['f_hz'] - 30e3) < 5
    assert not fuera['detectado']


def test_tasa_doppler_que_reparte_el_pico():
    """Valido si tasa*T^2 < 1: con tasa = 0.8/T^2 el pico se mantiene y la
    estimacion cae al centro de la ventana (error < 1 bin); con tasa = 6/T^2
    el pico se reparte y pierde ~5 dB."""
    rng = np.random.default_rng(9)
    n, t = 1000, 0.01
    b = Beacon(FS, 'portadora', semilla=3)
    k = 0.8 / t ** 2
    x = b.bloque(n, 10e3, 10e3 + k * t, 60.0)
    a = adquirir_fft(x, FS, (-50e3, 50e3))
    assert a['detectado'] and abs(a['f_hz'] - (10e3 + k * t / 2)) < 1.0 / t
    b = Beacon(FS, 'portadora', semilla=3)
    x2 = b.bloque(n, 10e3, 10e3 + 6 * k / 0.8 * t, 60.0)
    a2 = adquirir_fft(x2, FS, (-50e3, 50e3))
    assert a['snr_db'] - a2['snr_db'] > 3.0          # medido 5.4 dB
