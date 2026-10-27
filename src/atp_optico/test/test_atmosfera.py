"""Validacion de atp_optico.atmosfera contra cifras publicadas y formulas cerradas.

Referencias: HV 5/7 (Andrews y Phillips cap. 3): r0 = 5 cm y theta0 = 7 urad a 500 nm en el cenit; escalas
lam^(6/5) (r0) y sin(el)^(3/5); Cn2 uniforme: sigma_R^2 = 1.23 Cn2 k^(7/6) L^(11/6), beam wander
<r_c^2> = 2.42 Cn2 L^3 W0^(-1/3), r0 = (0.423 k^2 Cn2 L)^(-3/5); el tilt de llegada no depende de lam.
"""
import dataclasses
import math

import pytest

from atp_optico import atmosfera as A

HV = A.HV57


# ------------------------------------------------------------------ HV 5/7: el nombre

def test_hv57_r0_5cm_y_theta0_7urad_a_500nm():
    assert A.fried_r0(HV, A.LAM_500, 90.0) == pytest.approx(0.05, rel=0.03)         # 4.96 cm
    assert 1e6 * A.theta0(HV, A.LAM_500, 90.0) == pytest.approx(7.0, rel=0.03)      # 6.89 urad


def test_escala_con_la_longitud_de_onda_y_la_elevacion():
    k = (A.LAM_1550 / A.LAM_500) ** 1.2
    assert A.fried_r0(HV, A.LAM_1550) / A.fried_r0(HV, A.LAM_500) == pytest.approx(k, rel=1e-9)
    assert A.fried_r0(HV, A.LAM_1550, 30.0) / A.fried_r0(HV, A.LAM_1550, 90.0) == pytest.approx(math.sin(math.radians(30.0)) ** 0.6, rel=1e-9)
    # el isoplanatico escala sin(el)^(8/5)
    assert A.theta0(HV, A.LAM_1550, 30.0) / A.theta0(HV, A.LAM_1550, 90.0) == pytest.approx(math.sin(math.radians(30.0)) ** 1.6, rel=1e-9)


def test_r0_tipico_a_1550_nm():
    # 19.3 cm a nivel del mar y 21.1 cm a 2240 m (CDMX): la altitud ayuda poco porque manda la capa limite local
    assert 100 * A.fried_r0(HV, A.LAM_1550) == pytest.approx(19.3, abs=0.4)
    assert 100 * A.fried_r0(dataclasses.replace(HV, alt_estacion_m=2240.0), A.LAM_1550) == pytest.approx(21.1, abs=0.5)


def test_la_altitud_baja_la_turbulencia_y_un_cn2_mayor_la_sube():
    alto = dataclasses.replace(HV, alt_estacion_m=2240.0)
    assert A.rytov(alto, A.LAM_1550, 30.0) < A.rytov(HV, A.LAM_1550, 30.0)
    fuerte = dataclasses.replace(HV, cn2_suelo=1e-13)
    assert A.fried_r0(fuerte, A.LAM_1550, 45.0) < A.fried_r0(HV, A.LAM_1550, 45.0)
    # en modo 'agl' la altitud no cambia nada (todo se mide desde la estacion)
    agl = dataclasses.replace(HV, alt_estacion_m=2240.0, modo_altitud='agl')
    assert A.fried_r0(agl, A.LAM_1550) == pytest.approx(A.fried_r0(HV, A.LAM_1550), rel=1e-12)


# ------------------------------------------------------------------ formulas cerradas con Cn2 uniforme

CN2 = 2e-15
UNIF = A.Atmosfera(cn2_uniforme=CN2, h_max_m=10000.0)


def test_rytov_uniforme():
    k = A.numero_onda(A.LAM_1550)
    cerrada = 1.23 * CN2 * k ** (7 / 6) * 10000.0 ** (11 / 6)
    assert A.rytov(UNIF, A.LAM_1550, 90.0) == pytest.approx(cerrada, rel=0.005)     # 2.25*6/11 = 1.2273


def test_r0_uniforme():
    k = A.numero_onda(A.LAM_1550)
    assert A.fried_r0(UNIF, A.LAM_1550, 90.0) == pytest.approx((0.423 * k * k * CN2 * 10000.0) ** -0.6, rel=1e-4)


def test_beam_wander_uniforme():
    # <r_c^2> = 2.42 Cn2 L^3 W0^(-1/3): la integral de 7.25 W0^(-1/3) int (L-z)^2 dz
    D = 0.4
    W0 = D / (2 * 1.12)
    cerrada = 2.42 * CN2 * 10000.0 ** 3 * W0 ** (-1.0 / 3.0)
    r = A.beam_wander(UNIF, D, 90.0, 10000.0)
    assert r['rc_m'] ** 2 == pytest.approx(cerrada, rel=0.005)
    assert r['sigma_urad'] == pytest.approx(1e6 * math.sqrt(cerrada / 2) / 10000.0, rel=0.005)


def test_beam_wander_en_el_enlace_real():
    # 40 cm, cenit, 500 km: unos pocos urad por eje, comparable al haz de la apertura (3.7 urad): por eso la subida se ensancha
    w = A.beam_wander(HV, 0.4, 90.0, 500e3)
    assert 2.0 < w['sigma_urad'] < 6.0
    # a baja elevacion el haz atraviesa mas turbulencia y el brazo es mayor
    assert A.beam_wander(HV, 0.4, 20.0, 1400e3)['sigma_urad'] > w['sigma_urad']
    # una apertura mayor lo reduce (W0^(-1/6) en angulo)
    assert A.beam_wander(HV, 0.8, 90.0, 500e3)['sigma_urad'] < w['sigma_urad']


# ------------------------------------------------------------------ tilt

def test_tilt_independiente_de_lambda_y_formula_de_tyler():
    t1 = A.tilt_llegada_rad(HV, A.LAM_1550, 0.4, 45.0)
    t2 = A.tilt_llegada_rad(HV, A.LAM_1064, 0.4, 45.0)
    t3 = A.tilt_llegada_rad(HV, A.LAM_500, 0.4, 45.0)
    assert t1 == pytest.approx(t2, rel=1e-9) and t1 == pytest.approx(t3, rel=1e-9)
    r0 = A.fried_r0(HV, A.LAM_1550, 45.0)
    # sigma = 0.42 (lam/D)(D/r0)^(5/6) (la forma clasica)
    assert t1 == pytest.approx(0.427 * (A.LAM_1550 / 0.4) * (0.4 / r0) ** (5.0 / 6.0), rel=0.01)
    assert 1e6 * t1 == pytest.approx(3.6, abs=0.3)


def test_strehl_limites():
    assert A.strehl_corto(0.01, 0.2) == pytest.approx(1.0, abs=0.002)
    x = 0.134 * (0.05 / 0.2) ** (5.0 / 3.0)                  # D/r0 = 1/4: primer orden 1 - 0.134 (D/r0)^(5/3)
    assert A.strehl_corto(0.05, 0.2) == pytest.approx(1.0 - x, abs=0.004)
    # asintota (r0/D)^2 de la ley de Fried, a un factor
    s = A.strehl_corto(2.0, 0.1)
    assert 0.5 * (0.1 / 2.0) ** 2 < s < 20 * (0.1 / 2.0) ** 2


# ------------------------------------------------------------------ centelleo

def test_centelleo_debil_sin_apertura_es_rytov():
    debil = dataclasses.replace(HV, cn2_suelo=1e-16, viento_rms=1.0)
    c = A.centelleo(debil, A.LAM_1550, 90.0, 0.0)
    assert c['regimen'] == 'debil'
    assert c['sigma_I2'] == pytest.approx(c['sigma_R2'], rel=0.05)        # sigma_I^2 -> sigma_R^2 a sigma_R^2 << 1
    assert c['sigma_I2'] < 0.05


def test_promediado_por_apertura_baja_el_indice():
    previo = 1e9
    for D in (0.0, 0.1, 0.4, 1.0):
        s = A.centelleo(HV, A.LAM_1550, 30.0, D)['sigma_I2']
        assert s < previo
        previo = s
    # una apertura de 40 cm reduce el indice de un receptor puntual de >5x a 30 deg
    assert A.centelleo(HV, A.LAM_1550, 30.0, 0.4)['sigma_I2'] < 0.2 * A.centelleo(HV, A.LAM_1550, 30.0, 0.0)['sigma_I2']


def test_promediado_analitico_contra_churnside_por_capas():
    """Dos modelos independientes del mismo factor A(D) = sigma_I^2(D)/sigma_I^2(0): la formula de A&P con L efectiva y la
    integral por capas de Churnside (1991). Difieren hasta ~2x (A&P decae como d^(-5/3), Churnside como d^(-7/3)):
    se acepta el factor 2.5 y se prefiere la de A&P (la MAS pesimista)."""
    for el in (20.0, 45.0, 90.0):
        ap = A.centelleo(HV, A.LAM_1550, el, 0.4)['sigma_I2'] / A.centelleo(HV, A.LAM_1550, el, 0.0)['sigma_I2']
        ch = A.promediado_apertura_churnside(HV, A.LAM_1550, el, 0.4)
        assert ch / 2.5 < ap < ch * 2.5
        assert ap >= ch * 0.9                              # A&P es la conservadora


def test_el_centelleo_crece_hacia_el_horizonte():
    s = [A.centelleo(HV, A.LAM_1550, el, 0.4)['sigma_I2'] for el in (90, 60, 30, 15, 10)]
    assert all(a < b for a, b in zip(s, s[1:]))
    assert A.centelleo(HV, A.LAM_1550, 10.0, 0.4)['regimen'] == 'moderado-fuerte'
    assert A.centelleo(HV, A.LAM_1550, 90.0, 0.4)['regimen'] == 'debil'


def test_gamma_gamma_consistente():
    c = A.centelleo(HV, A.LAM_1550, 10.0, 0.4)
    # 1 + sigma_I^2 = (1 + 1/alpha)(1 + 1/beta)
    assert (1 + 1 / c['alpha']) * (1 + 1 / c['beta']) - 1 == pytest.approx(c['sigma_I2'], rel=1e-9)


# ------------------------------------------------------------------ viento y escala temporal

def test_tiempo_de_coherencia_y_greenwood():
    tau = A.tiempo_coherencia(HV, A.LAM_500, 90.0)
    assert 1e3 * tau == pytest.approx(1.9, abs=1.0)                     # unos ms a 500 nm
    assert A.frecuencia_greenwood(HV, A.LAM_500, 90.0) == pytest.approx(0.134 / tau, rel=1e-12)
    # el barrido de un satelite LEO (10 mrad/s) acorta mucho tau0
    assert A.tiempo_coherencia(HV, A.LAM_1550, 45.0, 0.010) < 0.3 * A.tiempo_coherencia(HV, A.LAM_1550, 45.0, 0.0)
    assert A.viento_tilt(HV, 45.0, 0.010) > A.viento_tilt(HV, 45.0, 0.0)


# ------------------------------------------------------------------ transmision

def test_transmision():
    t90 = A.transmision(HV, A.LAM_1550, 90.0)
    assert 0.90 < t90 < 0.97                                            # cielo despejado a nivel del mar: ~0.3-0.4 dB
    assert A.transmision(dataclasses.replace(HV, alt_estacion_m=2240.0), A.LAM_1550, 90.0) > t90
    assert A.transmision(HV, A.LAM_1550, 10.0) < A.transmision(HV, A.LAM_1550, 30.0) < t90
    # la niebla (visibilidad 2 km) la hunde
    assert A.transmision(dataclasses.replace(HV, visibilidad_km=2.0), A.LAM_1550, 90.0) < 0.6
    # masa de aire de Kasten y Young: 1 en el cenit, 5.6 a 10 deg
    assert float(A.masa_de_aire(90.0)) == pytest.approx(1.0, abs=2e-3)
    assert float(A.masa_de_aire(10.0)) == pytest.approx(5.6, abs=0.1)
