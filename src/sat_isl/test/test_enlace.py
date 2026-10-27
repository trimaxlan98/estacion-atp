"""Validacion de sat_isl.enlace: haz gaussiano truncado, perdida por apuntamiento,
ecuacion de enlace y detectores, contra formulas cerradas, integrales numericas
independientes y las cifras del curso 18 y de los terminales Tesat LCT.

Fuentes de las cifras de referencia:
  * curso 18 (isl.py): telescopio de 10 cm a 1550 nm -> 106.1 dBi; FSPL a 5000 km
    -> 272.2 dB; 1 W con 106 + 106 dBi -> -30 dBm.
  * Klein y Degnan, Appl. Opt. 13 (1974): relacion optima apertura/haz ~ 1.12.
  * limite cuantico del BPSK homodino: 9 fotones por bit para BER 1e-9.
  * Tesat LCT (TerraSAR-X/NFIRE 2008: 125 mm, 0.7 W, 1064 nm, 5.625 Gb/s, BER < 1e-9, ~5100 km;
    LCT 135: 135 mm, 1.8 Gb/s hasta 45 000 km LEO-GEO y "hasta 80 000 km"). Las perdidas
    internas y la penalizacion del receptor NO son publicas: son SUPUESTOS del preset.
"""
import math

import numpy as np
import pytest
from scipy.integrate import quad
from scipy.special import j0, erfc

from sat_isl import enlace as E
from sat_isl import terminal as T

LAM = 1550e-9


# ------------------------------------------------------------------ curso 18

def test_cifras_del_curso_18():
    g = E.ganancia_apertura(0.1, LAM)
    assert math.isclose(g, 4.108e10, rel_tol=1e-3)
    assert math.isclose(E.db(g), 106.1, abs_tol=0.05)
    fspl_db = -E.db(E.fspl_lineal(5e6, LAM))
    assert math.isclose(fspl_db, 272.2, abs_tol=0.05)
    pr_dbm = 30.0 + 2 * E.db(g) - fspl_db
    assert math.isclose(pr_dbm, -30.0, abs_tol=0.15)      # el curso redondea -29.9 a -30
    # y 1 W a 1550 nm: fotones por bit a 10 Gb/s con -30 dBm = 1 uW  -> ~ 780-800
    assert 700 < E.fotones_por_bit(1e-6, 1e10, LAM) < 850


def test_constantes_si_exactas():
    assert E.C_LUZ == 299792458.0
    assert math.isclose(E.energia_foton(LAM), 1.2813e-19, rel_tol=1e-3)


# ------------------------------------------------------------------ haz gaussiano truncado

def _ganancia_numerica(gamma, D=0.05, lam=LAM):
    """Ganancia sobre el eje de la apertura circular iluminada con un campo gaussiano
    truncado, por integracion numerica de |int E dA|^2 / int |E|^2 dA (independiente de
    la forma cerrada con tanh)."""
    a = D / 2
    w = a / gamma
    num = quad(lambda r: math.exp(-r * r / w ** 2) * 2 * math.pi * r, 0, a)[0] ** 2
    den = quad(lambda r: math.exp(-2 * r * r / w ** 2) * 2 * math.pi * r, 0, a)[0]
    return 4 * math.pi / lam ** 2 * num / den


@pytest.mark.parametrize('gamma', [0.3, 0.8, 1.12, 1.5, 2.5])
def test_ganancia_tx_forma_cerrada_contra_integral(gamma):
    assert math.isclose(E.ganancia_tx(0.05, LAM, gamma), _ganancia_numerica(gamma), rel_tol=1e-7)


def test_gamma_optimo_klein_degnan():
    g = E.gamma_optimo()
    assert math.isclose(g, 1.12, abs_tol=0.02)
    f = lambda x: E.fraccion_apertura(x) * E.eta_gauss(x)  # noqa: E731
    assert f(g) >= f(0.8) and f(g) >= f(1.5)
    assert math.isclose(f(g), 0.8145, abs_tol=2e-3)       # ~ -0.89 dB respecto de la apertura ideal
    # limites: apertura uniforme y haz muy angosto
    assert math.isclose(E.eta_gauss(1e-6), 1.0, rel_tol=1e-9)
    assert E.eta_gauss(5.0) < 0.2


def test_semiangulo_es_casi_lambda_sobre_d():
    """theta_h = sqrt(8/G): 0.956 lam/D en gamma = 1.12 (el 'theta ~ lam/D' del enunciado)."""
    th = E.semiangulo_haz_rad(0.05, LAM)
    assert math.isclose(th * 0.05 / LAM, 0.956, abs_tol=0.01)
    assert math.isclose(th * 1e6, 29.64, abs_tol=0.05)           # 5 cm a 1550 nm
    assert 0.8 < E.semiangulo_haz_rad(0.1, LAM) * 0.1 / LAM < 1.1


@pytest.mark.parametrize('frac,tol', [(0.25, 0.04), (0.5, 0.05), (0.75, 0.07), (1.0, 0.10)])
def test_gaussiana_equivalente_contra_el_campo_lejano_real(frac, tol):
    """La gaussiana de semiangulo sqrt(8/G) reproduce el patron REAL del haz truncado
    (campo lejano de Bessel) con error relativo < tol en intensidad, hasta theta_h."""
    gamma, D = E.GAMMA_KLEIN, 0.05
    a, w, k = D / 2, D / 2 / gamma, 2 * math.pi / LAM
    campo = lambda th: quad(lambda r: math.exp(-r * r / w ** 2) * j0(k * r * th) * r, 0, a, limit=200)[0]  # noqa: E731
    th_h = E.semiangulo_haz_rad(D, LAM, gamma)
    th = frac * th_h
    real = (campo(th) / campo(0.0)) ** 2
    modelo = float(E.perdida_apuntamiento(th, th_h))
    assert abs(real - modelo) / real < tol, (real, modelo)


def test_perdida_por_apuntamiento_puntos_conocidos():
    th = 30e-6
    assert float(E.perdida_apuntamiento(0.0, th)) == 1.0
    assert math.isclose(float(E.perdida_apuntamiento(th, th)), math.exp(-2), rel_tol=1e-12)
    # 3 dB cuando e = theta_h sqrt(ln2 / 2) = 0.589 theta_h
    assert math.isclose(float(E.db(E.perdida_apuntamiento(th * math.sqrt(math.log(2) / 2), th))),
                        -3.0103, abs_tol=1e-3)


def test_perdida_media_por_jitter_gaussiano_monte_carlo():
    """Media de exp(-2 e^2/theta^2) con e rayleigh de sigma por eje = 1/(1 + 4 sigma^2/theta^2)."""
    th = 30.0
    rng = np.random.default_rng(5)
    for sigma in (3.0, 8.0, 15.0):
        e = rng.normal(0.0, sigma, size=(400000, 2))
        mc = float(np.mean(E.perdida_apuntamiento(np.hypot(e[:, 0], e[:, 1]), th)))
        assert math.isclose(mc, E.perdida_apuntamiento_media(sigma, th), rel_tol=5e-3)


def test_ecuacion_de_enlace_cuadra_con_la_cascada_en_db():
    tm = T.ISL_COHERENTE
    rho = 5.0e6
    p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, rho, tm.eta_tx, tm.eta_rx, tm.gamma)
    g_t = 10 * math.log10(E.ganancia_tx(tm.D_m, tm.lam_m, tm.gamma))
    g_r = 10 * math.log10(E.ganancia_apertura(tm.D_m, tm.lam_m))
    casc = (30.0 + 10 * math.log10(tm.eta_tx * E.fraccion_apertura(tm.gamma)) + g_t
            - 20 * math.log10(4 * math.pi * rho / tm.lam_m) + g_r + 10 * math.log10(tm.eta_rx))
    assert math.isclose(float(E.dbm(p)), casc, abs_tol=1e-9)
    # 6 dB por cada duplicacion del diametro de una apertura (12 dB si son las dos), y 6 dB por duplicar el rango
    p2 = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, 2 * tm.D_m, 2 * tm.D_m, rho, tm.eta_tx, tm.eta_rx, tm.gamma)
    assert math.isclose(float(E.db(p2 / p)), 12.0412, abs_tol=0.001)
    p3 = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, 2 * rho, tm.eta_tx, tm.eta_rx, tm.gamma)
    assert math.isclose(float(E.db(p / p3)), 6.0206, abs_tol=1e-3)


# ------------------------------------------------------------------ detectores

def test_homodino_limite_cuantico_9_fotones_por_bit():
    d = E.Homodino(penalizacion_db=0.0)
    p = E.w_de_dbm(-40.0)
    s = d.sensibilidad_w(1e9, LAM, 1e-9)
    np_bit = E.fotones_por_bit(s, 1e9, LAM)
    assert 8.9 < np_bit < 9.1                       # 0.5 erfc(sqrt(2 N)) = 1e-9 <=> Q(z) = 1e-9, z = 5.998, N = z^2/4 = 8.99
    assert math.isclose(float(d.ber(8.993 * E.energia_foton(LAM) * 1e9, 1e9, LAM)), 1e-9, rel_tol=0.02)
    # BER solo depende de fotones por bit: igual a 1 Mb/s y a 10 Gb/s con la misma Np
    for r in (1e6, 1e10):
        assert math.isclose(float(d.ber(30.0 * E.energia_foton(LAM) * r, r, LAM)),
                            0.5 * erfc(math.sqrt(60.0)), rel_tol=1e-9)
    # la penalizacion de 5 dB cuesta 5 dB de sensibilidad
    d5 = E.Homodino(penalizacion_db=5.0)
    assert math.isclose(10 * math.log10(d5.sensibilidad_w(1e9, LAM, 1e-9) / s), 5.0, abs_tol=0.01)


def test_apd_ber_decrece_y_sensibilidad_en_el_orden_de_un_receptor_de_10g():
    d = E.Apd()
    ps = E.w_de_dbm(np.arange(-50.0, -20.0, 2.0))
    ber = d.ber(ps, 1e10, LAM)
    assert np.all(np.diff(ber) < 0)
    s10g = float(E.dbm(d.sensibilidad_w(1e10, LAM, 1e-9)))
    assert -40.0 < s10g < -30.0                      # hojas de datos de APD+TIA de 10G: -28 a -35 dBm
    assert math.isclose(float(d.ber(d.sensibilidad_w(1e10, LAM, 1e-9), 1e10, LAM)), 1e-9, rel_tol=0.01)


def test_apd_limitado_por_ruido_termico_escala_con_raiz_de_la_tasa():
    """Con ruido termico dominante la sensibilidad sube 10 log10(100)/2 = 10 dB por 100x de tasa
    (algo mas, ~12 dB, porque a tasas bajas el ruido de disparo de la senal pesa menos)."""
    d = E.Apd()
    s_lo = float(E.dbm(d.sensibilidad_w(1e8, LAM, 1e-9)))
    s_hi = float(E.dbm(d.sensibilidad_w(1e10, LAM, 1e-9)))
    assert 9.0 < s_hi - s_lo < 13.0


def test_coherente_gana_decenas_de_db_al_apd_a_igual_tasa():
    apd, coh = E.Apd(), E.Homodino(5.0)
    for r, minimo in ((1e7, 20.0), (1e10, 8.0)):
        dif = float(E.dbm(apd.sensibilidad_w(r, LAM, 1e-9)) - E.dbm(coh.sensibilidad_w(r, LAM, 1e-9)))
        assert dif > minimo


def test_tasa_maxima_escalera():
    coh = E.Homodino(5.0)
    p = E.w_de_dbm(-50.0)
    r = E.tasa_maxima(coh, p, LAM, 1e-6, 3.0)
    assert r > 0
    # monotona: mas potencia, tasa >= ; sin potencia, 0
    assert E.tasa_maxima(coh, p * 10, LAM, 1e-6, 3.0) >= r
    assert E.tasa_maxima(coh, 1e-15, LAM, 1e-6, 3.0) == 0.0
    # con el margen pedido el BER de esa tasa es al menos 3 dB mejor que el umbral
    assert float(coh.ber(p / 10 ** 0.3, r, LAM)) <= 1e-6 * 1.0001


# ------------------------------------------------------------------ caso publicado: Tesat LCT

def _margen_db(tm, rango_km, tasa, ber=1e-9):
    p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, rango_km * 1e3, tm.eta_tx, tm.eta_rx, tm.gamma)
    return float(E.db(p / tm.detector.sensibilidad_w(tasa, tm.lam_m, ber)))


def _rango_max_km(tm, tasa, ber=1e-9):
    lo, hi = 1e3, 5e5
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _margen_db(tm, mid, tasa, ber) > 0 else (lo, mid)
    return lo


def test_lct_terrasar_x_nfire_cierra_5100_km_con_5p6_gbps():
    """El enlace publicado (5.625 Gb/s, BER < 1e-9, ~5100 km, 125 mm, 0.7 W, 1064 nm) cierra
    con margen positivo en el modelo; el margen NO es un resultado de validacion fino porque
    las perdidas internas y la penalizacion del receptor son supuestos."""
    tm = T.LCT_TSX_NFIRE
    assert tm.D_m == 0.125 and tm.p_tx_w == 0.7 and tm.lam_m == 1064e-9
    m = _margen_db(tm, 5100.0, 5.625e9)
    assert 3.0 < m < 25.0, m
    p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, 5.1e6, tm.eta_tx, tm.eta_rx, tm.gamma)
    # unos 600 fotones por bit disponibles (cifra del modelo; el limite cuantico pide 9)
    assert 100 < E.fotones_por_bit(p, 5.625e9, tm.lam_m) < 2000
    assert _rango_max_km(tm, 5.625e9) > 5500.0       # mas que la distancia demostrada


def test_lct_135_edrs_cierra_45000_km_a_1p8_gbps_y_llega_al_orden_de_80000_km():
    tm = T.LCT_135
    assert _margen_db(tm, 45000.0, 1.8e9) > 0.0
    # Tesat publica 'hasta 80 000 km': el modelo (con 2.2 W SIN VERIFICAR y 5 dB de penalizacion
    # supuestos) da el mismo orden. Se exige factor 2, no mas.
    r = _rango_max_km(tm, 1.8e9)
    assert 40000.0 < r < 160000.0, r


def test_terminal_del_escenario_cierra_a_5500_km_con_margen():
    tm = T.ISL_COHERENTE
    p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, 5.5e6, tm.eta_tx, tm.eta_rx, tm.gamma)
    assert -55.0 < float(E.dbm(p)) < -45.0
    assert E.tasa_maxima(tm.detector, p, LAM, 1e-6, 3.0) >= 1e9
    apd = T.ISL_APD
    r_apd = E.tasa_maxima(apd.detector, p * 10, LAM, 1e-6, 3.0)     # con +10 dB el APD cierra ~250 Mb/s; sin ellos, nada
    assert 0 < r_apd <= 1e9
    assert E.tasa_maxima(apd.detector, p, LAM, 1e-6, 3.0) < E.tasa_maxima(tm.detector, p, LAM, 1e-6, 3.0) / 10
