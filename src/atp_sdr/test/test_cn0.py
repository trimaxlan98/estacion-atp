"""Estimadores de C/N0: M2M4 y NWPR (bajo/medio), espectral (alto)."""
import numpy as np
import pytest

from atp_sdr.receptor import (SeguidorPortadora, cn0_espectral, cn0_m2m4, cn0_nwpr,
                              m2m4)
from atp_sdr.senal import Beacon


def _prompts(cn0, semilla, n_epocas=2000, fs=10e3, t=1e-3):
    """Muestras post-integracion de un PLL enganchado (3.er orden, B_L 20 Hz)."""
    b = Beacon(fs, 'portadora', semilla=semilla)
    s = SeguidorPortadora(fs, t, 3, 20.0, asistencia='nunca', n_ventana=n_epocas)
    s.iniciar(0.0)
    n = int(fs * t)
    for _ in range(n_epocas // 50 + 10):
        s.procesar(b.bloque(50 * n, 0.0, 0.0, cn0))
    return s.hist[-n_epocas:]


def test_m2m4_formulas_con_momentos_exactos():
    """Con senal de amplitud A en ruido complejo de varianza N: M2 = A^2 + N,
    M4 = A^4 + 4 A^2 N + 2 N^2 (Pauluzzi & Beaulieu 2000), y 2 M2^2 - M4 = A^4."""
    rng = np.random.default_rng(0)
    a, n = 2.0, 0.8
    p = a + np.sqrt(n / 2) * (rng.standard_normal(2_000_000)
                              + 1j * rng.standard_normal(2_000_000))
    s, nz = m2m4(p)
    assert s == pytest.approx(a * a, rel=0.01)
    assert nz == pytest.approx(n, rel=0.02)


@pytest.mark.parametrize('cn0', [34.0, 40.0, 50.0, 60.0])
def test_m2m4_y_nwpr_insesgados_en_su_rango(cn0):
    """T = 1 ms, 2000 muestras (2 s), PLL enganchado: sesgo de M2M4 < 0.3 dB y
    de NWPR < 0.5 dB entre 34 y 60 dB-Hz (medido: -0.20..-0.04 y -0.37..-0.12;
    el sesgo de NWPR es la fase del lazo que no es perfectamente constante,
    exp(-sigma^2) sobre la potencia coherente). Por debajo de ~28 dB-Hz M2M4
    se sesga > 0.7 dB."""
    m = [cn0_m2m4(_prompts(cn0, s), 1e-3) for s in range(6)]
    w = [cn0_nwpr(_prompts(cn0, s), 1e-3, 20) for s in range(6)]
    assert abs(np.mean(m) - cn0) < 0.3
    assert np.std(m) < 0.25
    assert abs(np.mean(w) - cn0) < 0.5


def test_m2m4_se_sesga_por_debajo_de_su_rango():
    sesgo = np.nanmean([cn0_m2m4(_prompts(25.0, s), 1e-3) for s in range(6)]) - 25.0
    assert abs(sesgo) > 0.5            # fuera de rango: no es fiable (medido -0.9 dB)


@pytest.mark.parametrize('cn0', [50.0, 60.0, 80.0, 90.0])
def test_estimador_espectral_a_c_n0_alto(cn0):
    """Potencia de banda ancha menos piso de ruido del espectro: insesgado
    (+0.02 dB medido) con 0.2 dB de desviacion por bloque de 50 ms a C/N0 alto
    (0.5 dB a 50 dB-Hz, 1 dB a 45). No necesita lazo enganchado ni amplitud
    constante, a diferencia de M2M4."""
    fs = 150e3
    b = Beacon(fs, 'portadora', semilla=3)
    est = [cn0_espectral(b.bloque(7500, 30e3, 30e3, cn0), fs, 30e3)[0] for _ in range(60)]
    assert abs(np.mean(est) - cn0) < (0.25 if cn0 == 50.0 else 0.1)
    assert np.std(est) < (0.7 if cn0 == 50.0 else 0.3)


@pytest.mark.parametrize('k', [100, 400])
def test_dispersion_de_m2m4_y_acuerdo_con_el_espectral(k):
    """M2M4 lee N de las fluctuaciones de |p|^2: su error relativo es el de una
    varianza muestral, sqrt(2/K), asi que sigma = 4.34 sqrt(2/K) dB (0.61 dB con
    K = 100, 0.31 con K = 400) y es IGUAL a 45 que a 95 dB-Hz (no hay techo de
    C/N0 mientras la amplitud sea constante; si la amplitud se modula, M2M4 lo
    lee como ruido y el espectral es mas robusto). Y el espectral y M2M4 miden
    lo mismo a 70 dB-Hz."""
    fs = 150e3
    est = []
    for cn0 in (45.0, 70.0, 95.0):
        v = []
        for sem in range(16):
            b = Beacon(fs, 'portadora', semilla=sem)
            s = SeguidorPortadora(fs, 1e-3, 3, 30.0, asistencia='nunca', n_ventana=k)
            s.iniciar(0.0)
            for _ in range(k // 50 + 8):
                s.procesar(b.bloque(7500, 0.0, 0.0, cn0))
            v.append(cn0_m2m4(s.hist[-k:], 1e-3))
        est.append(v)
        teo = 4.34 * np.sqrt(2.0 / k)
        assert np.std(v) == pytest.approx(teo, rel=0.45)
        assert abs(np.mean(v) - cn0) < 0.25
    sp = Beacon(fs, 'portadora', semilla=9)
    espectral = np.mean([cn0_espectral(sp.bloque(7500, 0.0, 0.0, 70.0), fs, 0.0)[0]
                         for _ in range(40)])
    assert abs(espectral - np.mean(est[1])) < 0.3
