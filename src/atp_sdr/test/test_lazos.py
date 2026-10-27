"""Seguimiento de portadora contra la teoria de lazos de fase (Kaplan & Hegarty,
Understanding GPS/GNSS, cap. 5 y 8; Gardner, Phaselock Techniques)."""
import math

import numpy as np
import pytest

from atp_sdr.receptor import SeguidorPortadora, ancho_banda_discreto, coef_lazo
from util_sdr import corrida_pll


def _bl_numerico(c0, c1, c2):
    """B_L = (1/2) integral |H(f)|^2 df de H = F/(s + F), F = c2 + c1/s + c0/s^2
    con wn = 1: la definicion, integrada con la regla del trapecio."""
    w = np.logspace(-4, 5, 600_001)
    s = 1j * w
    num = c2 * s ** 2 + c1 * s + c0 if c0 else c2 * s + c1
    den = s ** 3 + c2 * s ** 2 + c1 * s + c0 if c0 else s ** 2 + c2 * s + c1
    h2 = np.abs(num / den) ** 2
    integra = getattr(np, "trapezoid", None) or np.trapz
    return float(integra(h2, w) / (2 * np.pi))


@pytest.mark.parametrize('orden,esperado', [(2, 0.53), (3, 0.7845)])
def test_coeficientes_del_lazo(orden, esperado):
    """B_L = 0.53 wn (2.o orden, zeta 0.707) y 0.7845 wn (3.er orden, a3 = 1.1,
    b3 = 2.4): se recalcula la integral de |H|^2 con los coeficientes del codigo."""
    c0, c1, c2, wn = coef_lazo(orden, 10.0)
    assert wn == pytest.approx(10.0 / esperado, rel=2e-3)
    bl = _bl_numerico(c0 / wn ** 3, c1 / wn ** 2, c2 / wn)
    assert bl == pytest.approx(esperado, rel=2e-3)


def test_ancho_de_banda_discreto_cerca_del_nominal():
    """El lazo digital (retardo de computo, integracion a mitad de epoca) tiene
    un B_L un poco mayor que el continuo: +1 % a B_L T = 0.005, +4 % a 0.02."""
    assert ancho_banda_discreto(3, 5.0, 1e-3) == pytest.approx(5.0, rel=0.02)
    assert ancho_banda_discreto(3, 20.0, 1e-3) == pytest.approx(20.0, rel=0.06)
    assert ancho_banda_discreto(2, 20.0, 1e-3) > 20.0


@pytest.mark.parametrize('orden,bn,cn0', [(2, 20.0, 40.0), (3, 20.0, 40.0),
                                           (3, 30.0, 37.0), (2, 10.0, 34.0)])
def test_varianza_de_fase_del_pll(orden, bn, cn0):
    """sigma^2 = B_L/(C/N0) (1 + 1/(2 T C/N0)) rad^2 (Kaplan & Hegarty, PLL con
    atan2), con el B_L EFECTIVO del lazo digital. Valido con sigma < ~0.3 rad
    (discriminador lineal). 60 000 epocas, ~6 % de incertidumbre estadistica:
    se exige 12 %."""
    t = 1e-3
    r = corrida_pll(cn0, orden, bn, t, dur_s=60.0, semilla=orden * 100 + int(bn))
    e = r['err_fase'][500:]
    cl = 10 ** (cn0 / 10)
    teo = ancho_banda_discreto(orden, bn, t) / cl * (1 + 1 / (2 * t * cl))
    assert abs(np.mean(e)) < 4 * math.sqrt(teo / 1000)       # sin sesgo
    assert np.var(e) == pytest.approx(teo, rel=0.12)


def test_rampa_de_frecuencia_2o_contra_3er_orden():
    """Error de arrastre de una rampa de frecuencia de a Hz/s (aceleracion de
    fase 2 pi a rad/s^2): el 2.o orden deja un error de fase CONSTANTE
    theta = 2 pi a / wn^2 (= a/c1 en estado estacionario); el 3.o lo anula y solo
    sufre la sacudida (jerk) j: theta = 2 pi j / wn^3. La frecuencia del NCO sigue
    la rampa sin retardo en los dos (error de fase constante => frecuencia
    media igual a la verdadera)."""
    bn = 20.0
    a = 100.0
    _, _, _, wn2 = coef_lazo(2, bn)
    _, _, _, wn3 = coef_lazo(3, bn)
    r2 = corrida_pll(60.0, 2, bn, tasa_hz_s=a, dur_s=6.0)
    r3 = corrida_pll(60.0, 3, bn, tasa_hz_s=a, dur_s=6.0)
    th2 = 2 * math.pi * a / wn2 ** 2
    assert np.mean(r2['err_fase'][-1000:]) == pytest.approx(th2, rel=0.01)
    assert abs(np.mean(r3['err_fase'][-1000:])) < 1e-3
    assert abs(np.mean(r2['err_f_hz'][-1000:])) < 0.05
    assert abs(np.mean(r3['err_f_hz'][-1000:])) < 0.05
    j = 100.0
    rj = corrida_pll(60.0, 3, bn, jerk_hz_s2=j, dur_s=8.0)
    assert np.mean(rj['err_fase'][-1000:]) == pytest.approx(2 * math.pi * j / wn3 ** 3,
                                                           rel=0.02)
    # con la rampa de LEO en Ka (5.3 kHz/s) el 2.o orden de B_L = 100 Hz queda en
    # ~0.9 rad: pierde el enganche; el 3.o no
    assert 2 * math.pi * 5300.0 / coef_lazo(2, 100.0)[3] ** 2 > 0.9


def test_el_lazo_de_2o_orden_pierde_la_rampa_que_sigue_el_de_3o():
    """Rampa de 500 Hz/s con B_L = 20 Hz: theta_2o = 2 pi 500/wn^2 = 2.2 rad (127
    grados, mucho mas alla de los 45 de la regla): el detector lo declara sin
    enganche (cos 2 theta < 0); el 3.o sigue sin error."""
    r2 = corrida_pll(50.0, 2, 20.0, tasa_hz_s=500.0, dur_s=4.0)
    r3 = corrida_pll(50.0, 3, 20.0, tasa_hz_s=500.0, dur_s=4.0)
    assert np.mean(r2['err_fase'][-1000:]) == pytest.approx(
        2 * math.pi * 500.0 / coef_lazo(2, 20.0)[3] ** 2, rel=0.01)
    assert not r2['enganchado'][-1]
    assert r3['enganchado'][-1]
    assert np.sqrt(np.mean(r3['err_f_hz'][-1000:] ** 2)) < 1.5      # ruido del NCO a 50 dB-Hz


def test_detector_de_enganche_contra_exp_menos_2_sigma2():
    """Con fase gaussiana de desviacion sigma, E[cos 2 phi] = exp(-2 sigma^2):
    la metrica del detector (ruido compensado) debe seguir esa curva, sin
    sesgo por ruido, desde sigma = 3 hasta 14 grados."""
    bn, t = 20.0, 1e-3
    bl = ancho_banda_discreto(3, bn, t)
    for cn0 in (30.0, 33.0, 40.0):
        cl = 10 ** (cn0 / 10)
        sig2 = bl / cl * (1 + 1 / (2 * t * cl))
        medias = []
        for sem in range(8):
            r = corrida_pll(cn0, 3, bn, t, dur_s=3.0, semilla=sem)
            medias.append(np.nanmean(r['metrica'][3:]))
        assert np.mean(medias) == pytest.approx(math.exp(-2 * sig2), abs=0.02)


def test_detector_no_engancha_sin_senal_y_con_la_fase_girando():
    """NCO 200 Hz fuera y sin ayuda del FLL: la fase gira dentro de la epoca,
    la metrica cae a ~0 y el detector declara 'no enganchado'."""
    r = corrida_pll(50.0, 3, 20.0, fd_ini=0.0, f_nco_ini=400.0, dur_s=1.0,
                    asistencia='nunca')
    assert not r['enganchado'][-1]
    assert abs(r['metrica'][-1]) < 0.3
    s = SeguidorPortadora(10e3, 1e-3, 3, 20.0)
    s.iniciar(0.0)
    s.procesar(np.random.default_rng(1).standard_normal(10000)
               + 1j * np.random.default_rng(2).standard_normal(10000))
    assert not s.evaluar(ruido_prompt=1.0 / s.n_ep)['enganchado']


def test_umbral_de_perdida_de_enganche():
    """Perdida de enganche (deslizamientos de ciclo) contra la teoria. Regla de
    Kaplan & Hegarty: el lazo con atan2 es fiable mientras 3 sigma < 45 grados
    (sigma < 15 grados). Con B_L = 20 Hz, T = 1 ms eso es C/N0 ~ 27.4 dB-Hz.
    Medido: por encima de la regla no hay deslizamientos; la perdida (>= 1 ciclo)
    empieza con sigma teorico ~ 40 grados (~22 dB-Hz) y es mayoritaria con
    sigma ~ 64 grados (20 dB-Hz): la regla es conservadora ~5 dB."""
    bn, t = 20.0, 1e-3
    bl = ancho_banda_discreto(3, bn, t)

    def sigma_deg(cn0):
        cl = 10 ** (cn0 / 10)
        return math.degrees(math.sqrt(bl / cl * (1 + 1 / (2 * t * cl))))

    def deslizamientos(cn0, n=12):
        c = 0
        for sem in range(n):
            r = corrida_pll(cn0, 3, bn, t, dur_s=3.0, semilla=sem, evaluar=False)
            ue = np.unwrap(r['err_fase'])
            if np.max(np.abs(ue - ue[100])) > 2 * math.pi * 0.9:
                c += 1
        return c / n

    assert sigma_deg(28.0) < 15.0 < sigma_deg(26.0)          # el umbral de la regla
    assert deslizamientos(28.0) == 0.0
    assert deslizamientos(25.0) == 0.0
    perd_20 = deslizamientos(20.0)
    assert perd_20 >= 0.5
    assert deslizamientos(18.0) >= perd_20


def test_el_fll_de_ayuda_engancha_donde_el_pll_solo_no():
    """Error inicial de frecuencia de 220 Hz (>> la banda del lazo de 20 Hz, pero
    < 1/(4T) = 250 Hz del discriminador de frecuencia): con la ayuda del FLL el
    NCO llega a la frecuencia verdadera y se engancha en 1 s; el PLL solo, no."""
    con, sin = 0, 0
    for sem in range(4):
        a = corrida_pll(45.0, 3, 20.0, fd_ini=220.0, f_nco_ini=0.0, dur_s=1.0,
                        asistencia='adaptativa', semilla=sem)
        b = corrida_pll(45.0, 3, 20.0, fd_ini=220.0, f_nco_ini=0.0, dur_s=1.0,
                        asistencia='nunca', semilla=sem)
        con += int(a['enganchado'][-1] and abs(np.mean(a['err_f_hz'][-300:])) < 1.0)
        sin += int(b['enganchado'][-1])
    assert con == 4
    assert sin == 0


@pytest.mark.parametrize('orden,lw', [(3, 1.0), (2, 2.0)])
def test_ruido_de_fase_del_oscilador_a_traves_del_lazo(orden, lw):
    """Ruido de frecuencia blanco del oscilador (ancho de linea lw): el error de
    fase del lazo tiene var = 2 pi lw T sum g_k^2 (g = respuesta del lazo a un
    escalon de fase). A C/N0 alto domina el oscilador, no el ruido termico."""
    from atp_sdr.receptor import varianza_fase_por_oscilador
    bn, t = 20.0, 1e-3
    r = corrida_pll(80.0, orden, bn, t, dur_s=60.0, semilla=orden, lw_lo_hz=lw)
    e = r['err_fase'][500:]
    teo = varianza_fase_por_oscilador(orden, bn, t, lw)
    assert 0.002 < teo < 0.2                       # rango lineal
    assert np.var(e) == pytest.approx(teo, rel=0.15)
