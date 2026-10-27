"""Validacion de sat_isl.adquisicion: la espiral, la probabilidad de adquisicion
contra la analitica de cobertura (Monte Carlo), el optimo de la baliza y la
relacion con el error de orbita (cifras de docs/OD.md).

Referencias: area barrida k paso^2 = pi R^2 (cobertura de una espiral de Arquimedes),
distribucion de Rayleigh del blanco gaussiano, y la derivada de theta^2 ln(A / theta^2 P)
para el semiangulo optimo de baliza.
"""
import math

import numpy as np
import pytest

from sat_isl import adquisicion as A
from sat_isl import enlace as E
from sat_isl import terminal as T

TM = T.ISL_COHERENTE
RHO = 5.5e6


# ------------------------------------------------------------------ la espiral

def test_espiral_cobertura_area_y_paso_de_arco():
    paso = 100.0
    n = 4000
    pts = A.espiral_arquimedes(n, paso)
    assert np.all(pts[0] == 0.0)
    r = np.hypot(pts[:, 0], pts[:, 1])
    assert np.all(np.diff(r) >= -1e-9)                           # crece monotona
    # radio tras k pasos: R(k) = paso sqrt(k/pi) (ec. 2) a mejor del 4 % para k >= 100
    for k in (100, 500, 1000, 4000):
        assert math.isclose(r[k - 1], A.radio_barrido_urad(k, paso), rel_tol=0.04)
    # el paso de arco consecutivo es ~ el paso (entre 0.8 y 1.1 salvo el arranque)
    d = np.hypot(*np.diff(pts, axis=0).T)
    assert 0.8 * paso < np.median(d[50:]) < 1.1 * paso
    # separacion entre vueltas: el radio crece `paso` por vuelta
    k_vuelta = int(2 * math.pi * r[2000] / paso)
    assert math.isclose(r[2000 + k_vuelta] - r[2000], paso, rel_tol=0.05)
    # inversa de la ec. (2)
    k = A.pasos_para_radio(3500.0, paso)
    assert A.radio_barrido_urad(k, paso) >= 3500.0 > A.radio_barrido_urad(k - 2, paso)


def test_la_espiral_no_deja_hueco_alrededor_del_centro():
    """Regresion (la vio el Monte Carlo del escenario): una espiral que se salta la vuelta interior
    deja sin cubrir el anillo entre r_hit y ~paso del centro. Con el arco exacto, todo punto a
    menos de 2.5 pasos del centro queda a r_hit o menos de algun punto del barrido."""
    r_hit = 100.0
    paso = A.paso_espiral_urad(r_hit)
    pts = A.espiral_arquimedes(200, paso)
    ang = np.linspace(0, 2 * math.pi, 361)[:-1]
    for r in np.linspace(0.0, 2.5 * paso, 80):
        q = r * np.stack([np.cos(ang), np.sin(ang)], axis=1)
        d = np.min(np.hypot(q[:, None, 0] - pts[None, :, 0], q[:, None, 1] - pts[None, :, 1]), axis=1)
        assert np.mean(d <= r_hit * 1.0001) > 0.985, (r, np.mean(d <= r_hit))
    # la longitud de arco entre puntos es el paso (exacta sobre la curva r = b phi)
    b = paso / (2 * math.pi)
    phi = np.hypot(pts[:, 0], pts[:, 1]) / b
    s = 0.5 * b * (phi * np.sqrt(1 + phi ** 2) + np.arcsinh(phi))
    assert np.allclose(s, np.arange(200) * paso, rtol=1e-9, atol=1e-6)


def test_con_kappa_raiz_de_2_los_discos_cubren_el_area():
    """Con paso = sqrt(2) r_hit, discos de radio r_hit centrados en los puntos cubren
    (casi) todo el disco de radio R(k): fraccion cubierta >= 99.5 % en puntos aleatorios."""
    r_hit = 100.0
    paso = A.paso_espiral_urad(r_hit)
    k = 600
    pts = A.espiral_arquimedes(k, paso)
    R = A.radio_barrido_urad(k, paso)
    rng = np.random.default_rng(2)
    n = 4000
    ang = rng.uniform(0, 2 * math.pi, n)
    q = np.stack([np.cos(ang), np.sin(ang)], axis=1) * (0.9 * R * np.sqrt(rng.uniform(0, 1, n)))[:, None]
    d = np.min(np.hypot(q[:, None, 0] - pts[None, :, 0], q[:, None, 1] - pts[None, :, 1]), axis=1)
    assert np.mean(d <= r_hit) > 0.995
    # y con paso = 2 r_hit (sin solape) quedan huecos: la cobertura cae
    pts2 = A.espiral_arquimedes(k, 2.0 * r_hit)
    d2 = np.min(np.hypot(q[:, None, 0] - pts2[None, :, 0], q[:, None, 1] - pts2[None, :, 1]), axis=1)
    assert np.mean(d2 <= r_hit) < 0.9


# ------------------------------------------------------------------ deteccion

def test_p0_de_la_baliza_y_radio_de_acierto():
    p0 = A.p0_baliza_w(TM, TM, RHO)
    assert -80.0 < float(E.dbm(p0)) < -65.0                       # ~ -71.6 dBm con la baliza de 300 urad
    # la potencia pico es la de la ecuacion de enlace con ganancia 8/theta^2 (cifra a mano)
    g_b = 8.0 / (TM.theta_baliza_urad * 1e-6) ** 2
    esperado = (TM.p_baliza_w * TM.eta_baliza * g_b * E.fspl_lineal(RHO, TM.lam_m)
                * E.ganancia_apertura(TM.D_m, TM.lam_m) * TM.eta_rx_cam)
    assert math.isclose(p0, esperado, rel_tol=1e-12)
    p_det = A.potencia_deteccion_w(TM)
    assert math.isclose(float(A.prob_deteccion(p_det, TM)), A.PD_FIABLE, abs_tol=1e-3)
    assert float(A.prob_deteccion(1e-20, TM)) < 2e-6                # sin senal: solo falsa alarma
    assert float(A.prob_deteccion(1e-9, TM)) > 0.999999
    r = A.radio_acierto_urad(p0, TM, TM.theta_baliza_urad)
    # (1): en r_hit la baliza vale exactamente p_det
    assert math.isclose(p0 * math.exp(-2 * r ** 2 / TM.theta_baliza_urad ** 2), p_det, rel_tol=1e-9)
    assert A.radio_acierto_urad(p_det * 0.5, TM, 100.0) == 0.0     # por debajo del umbral: no hay adquisicion


def test_falsa_alarma_por_paso():
    """Sin senal, la probabilidad de declarar la baliza es la Pfa pedida (cola gaussiana)."""
    pd0 = float(A.prob_deteccion(0.0, TM))
    assert math.isclose(pd0, TM.cam_pfa, rel_tol=0.02)


def test_sin_potencia_suficiente_no_se_adquiere_nunca():
    t, info = A.monte_carlo(500.0, TM, TM, 5e9, n=50)            # 5 millones de km
    assert info['r_hit_urad'] == 0.0
    assert np.all(np.isinf(t))


# ------------------------------------------------------------------ probabilidad de adquisicion

@pytest.mark.parametrize('sigma,instantes,tol', [(4000.0, (0.2, 0.5, 1.0, 2.0, 5.0), 0.03),
                                                   (8000.0, (0.5, 1.0, 2.0, 5.0, 10.0, 20.0), 0.015)])
def test_probabilidad_de_adquisicion_monte_carlo_contra_la_analitica(sigma, instantes, tol):
    """P_acq(t) del Monte Carlo (n = 4000) contra la ec. (3) (cobertura de Rayleigh del radio
    barrido): a mejor de 0.03 (sigma = 4.8 pasos) y 0.015 (9.6 pasos) absolutos; el error baja
    al crecer sigma/paso porque la analitica ignora el borde de la huella (con sigma = 2.4 pasos
    el Monte Carlo adelanta hasta 0.05 al principio)."""
    n = 4000
    t, info = A.monte_carlo(sigma, TM, TM, RHO, n=n, semilla=3)
    paso = info['paso_urad']
    assert paso < sigma / 4
    for tt in instantes:
        mc = float(np.mean(t <= tt))
        an = float(A.p_adquisicion_analitica(tt, paso, TM.cam_dwell_s, sigma, pd=1.0))
        assert abs(mc - an) < tol, (tt, mc, an)
    # la meseta es la fraccion que cae dentro del radio barrido: 1 - exp(-3.5^2/2) = 0.998
    assert abs(np.mean(np.isfinite(t)) - (1 - math.exp(-3.5 ** 2 / 2))) < 0.012


def test_tiempo_de_adquisicion_escala_con_el_cuadrado_de_la_incertidumbre():
    """Mediana del tiempo ~ sigma^2 (el area a barrer): duplicar sigma cuadruplica el tiempo."""
    med = {}
    for sig in (4000.0, 8000.0):
        t, _ = A.monte_carlo(sig, TM, TM, RHO, n=3000, semilla=5)
        med[sig] = float(np.median(t[np.isfinite(t)]))
    assert math.isclose(med[8000.0] / med[4000.0], 4.0, rel_tol=0.15)


def test_probabilidad_de_adquisicion_es_monotona_en_el_tiempo_y_decrece_con_sigma():
    t1, _ = A.monte_carlo(1000.0, TM, TM, RHO, n=2000, semilla=1)
    t2, _ = A.monte_carlo(4000.0, TM, TM, RHO, n=2000, semilla=1)
    for tt in (0.1, 0.5, 2.0):
        assert np.mean(t1 <= tt) > np.mean(t2 <= tt)
    grid = np.linspace(0.0, 10.0, 50)
    cdf = [np.mean(t2 <= x) for x in grid]
    assert all(b >= a for a, b in zip(cdf, cdf[1:]))


def test_monte_carlo_es_reproducible():
    a, _ = A.monte_carlo(3000.0, TM, TM, RHO, n=500, semilla=9)
    b, _ = A.monte_carlo(3000.0, TM, TM, RHO, n=500, semilla=9)
    c, _ = A.monte_carlo(3000.0, TM, TM, RHO, n=500, semilla=10)
    assert np.array_equal(a, b) and not np.array_equal(a, c)


# ------------------------------------------------------------------ baliza optima

def test_baliza_optima_P0_es_e_veces_el_umbral():
    th_opt = A.theta_baliza_optimo_urad(TM, TM, RHO)
    p_det = A.potencia_deteccion_w(TM)
    p0_opt = A.p0_baliza_w(TM, TM, RHO, th_opt)
    assert math.isclose(p0_opt / p_det, math.e, rel_tol=1e-9)         # ec. (4)
    # y es de verdad el maximo del area barrida por paso: (kappa r_hit)^2 con r_hit de (1)
    def area(th):
        return A.paso_espiral_urad(A.radio_acierto_urad(A.p0_baliza_w(TM, TM, RHO, th), TM, th)) ** 2
    for f in (0.5, 0.8, 1.25, 2.0):
        assert area(th_opt) > area(th_opt * f)


def test_una_baliza_mas_ancha_acelera_el_barrido():
    """El efecto practico: de 300 urad (por omision) al optimo el tiempo de barrido baja > 10x."""
    sig = 3000.0
    t_a, _ = A.monte_carlo(sig, TM, TM, RHO, n=1000, semilla=2)
    th = A.theta_baliza_optimo_urad(TM, TM, RHO)
    t_b, _ = A.monte_carlo(sig, TM, TM, RHO, n=1000, semilla=2, theta_b_urad=th)
    assert np.median(t_a[np.isfinite(t_a)]) > 10 * np.median(t_b[np.isfinite(t_b)])


# ------------------------------------------------------------------ relacion con el error de orbita (docs/OD.md)

def test_incertidumbre_viene_del_error_de_orbita_y_de_la_actitud():
    s = A.sigma_incertidumbre_urad(2.1, 5500.0, 145.0, 50.0)
    pos = math.sqrt(2) * 2.1 / 5500.0 * 1e6
    assert math.isclose(pos, 540.0, abs_tol=1.0)
    assert math.isclose(s, math.sqrt(pos ** 2 + 145.0 ** 2 + 50.0 ** 2), rel_tol=1e-12)
    # con la OD de la Estacion (docs/OD.md: sigma_pos formal 0.036 km) manda la actitud
    s_od = A.sigma_incertidumbre_urad(0.036, 5500.0, 145.0, 50.0)
    assert 150.0 < s_od < 160.0


def test_tiempo_de_adquisicion_contra_el_error_de_orbita_de_OD_md():
    """Cifras de docs/OD.md (error de posicion a 1, 3 dias de la epoca del TLE: 2.1 y 11 km; con
    la OD de la Estacion 0.036 km) con la actitud de un star tracker de nanosat (145 urad):
    mejorar la orbita de 11 km a 36 m reduce el tiempo de adquisicion (mediana con la baliza
    de 300 urad)."""
    tm = TM
    med = {}
    for nombre, sp in (('od', 0.036), ('tle1', 2.1), ('tle3', 11.0)):
        sig = A.sigma_incertidumbre_urad(sp, RHO / 1e3, 145.0, tm.sigma_cardan_urad)
        t, info = A.monte_carlo(sig, tm, tm, RHO, n=1500, semilla=4)
        med[nombre] = float(np.median(t[np.isfinite(t)]))
    assert med['od'] <= med['tle1'] < med['tle3']
    assert med['tle3'] > 5 * med['od']
