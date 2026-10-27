"""Validacion de atp_optico.enlace: desvanecimiento, modems OOK/PPM, FEC y presupuestos (OPALS como contraste)."""
import dataclasses
import math

import numpy as np
import pytest
from scipy import stats

from atp_optico import atmosfera as A
from atp_optico import enlace as L
from sat_isl import enlace as E

HV = A.HV57


# ------------------------------------------------------------------ desvanecimiento

def test_lognormal_media_y_varianza():
    d = L.Desvanecimiento(0.1, 'lognormal')
    assert d.esperanza(lambda I: I) == pytest.approx(1.0, abs=2e-3)
    assert d.esperanza(lambda I: (I - 1.0) ** 2) == pytest.approx(0.1, rel=0.02)
    assert d.cdf(1e-9) < 1e-12 and d.cdf(1e9) == pytest.approx(1.0)


def test_gamma_gamma_media_varianza_y_cdf_contra_montecarlo():
    alpha, beta = 4.0, 3.0
    d = L.Desvanecimiento(0.0, 'gammagamma', alpha, beta)
    assert d.sI2 == pytest.approx((1 + 1 / alpha) * (1 + 1 / beta) - 1, rel=1e-12)
    assert d.esperanza(lambda I: I) == pytest.approx(1.0, abs=3e-3)
    assert d.esperanza(lambda I: (I - 1.0) ** 2) == pytest.approx(d.sI2, rel=0.02)
    rng = np.random.default_rng(1)
    n = 400000
    I = rng.gamma(alpha, 1 / alpha, n) * rng.gamma(beta, 1 / beta, n)
    for t in (0.1, 0.3, 0.7, 1.0, 1.6, 2.5):
        assert float(d.cdf(t)) == pytest.approx(float(np.mean(I <= t)), abs=3e-3)


def test_gamma_gamma_tiende_a_lognormal_con_alpha_beta_grandes():
    g = L.Desvanecimiento(0.0, 'gammagamma', 300.0, 200.0)
    ln = L.Desvanecimiento(g.sI2, 'lognormal')
    for t in (0.8, 0.9, 1.0, 1.1, 1.3):
        assert float(g.cdf(t)) == pytest.approx(float(ln.cdf(t)), abs=0.01)


def test_cuantil_y_margen_de_desvanecimiento():
    d = L.Desvanecimiento(0.2, 'lognormal')
    for p in (1e-1, 1e-2, 1e-3):
        assert float(d.cdf(d.cuantil(p))) == pytest.approx(p, rel=1e-4)
    # mas margen cuando se pide menos outage
    assert L.margen_desvanecimiento_db(d, 1e-3) > L.margen_desvanecimiento_db(d, 1e-2) > 0.0
    # sin centelleo no hay margen
    assert L.margen_desvanecimiento_db(L.Desvanecimiento(0.0), 1e-2) == pytest.approx(0.0, abs=1e-9)


def test_desvanecimiento_desde_el_regimen():
    d = L.Desvanecimiento.de_centelleo(A.centelleo(HV, A.LAM_1550, 90.0, 0.4))
    assert d.modelo == 'lognormal'
    assert L.Desvanecimiento.de_centelleo(A.centelleo(HV, A.LAM_1550, 10.0, 0.4)).modelo == 'gammagamma'


# ------------------------------------------------------------------ PPM

def test_ppm_sin_fondo_es_exponencial():
    # sin fondo solo hay error si no llega ningun fotoelectron: P_err = e^-ns (M-1)/M (empate entre M ranuras vacias)
    for M in (4, 16, 64):
        for ns in (1.0, 5.0, 12.0):
            assert L.ser_ppm(ns, 0.0, M) == pytest.approx(math.exp(-ns) * (M - 1) / M, rel=1e-6)


def test_ppm_contra_montecarlo():
    M, ns, nb = 8, 3.0, 0.6
    rng = np.random.default_rng(5)
    n = 200000
    ks = rng.poisson(ns + nb, n)
    kb = rng.poisson(nb, (n, M - 1))
    mayor = kb.max(axis=1)
    empate = (kb == ks[:, None]).sum(axis=1)
    # acierto: ks > max de las demas, o empate con j rivales: gana con prob 1/(j+1)
    ok = (ks > mayor) + ((ks == mayor) / (empate + 1.0))
    ser_mc = 1.0 - float(np.mean(ok))
    assert L.ser_ppm(ns, nb, M) == pytest.approx(ser_mc, abs=4e-3)


def test_ppm_monotonia_y_ber():
    s = [L.ser_ppm(ns, 0.1, 16) for ns in (1, 2, 4, 8, 16)]
    assert all(a > b for a, b in zip(s, s[1:]))
    assert L.ser_ppm(5.0, 1.0, 16) > L.ser_ppm(5.0, 0.01, 16)               # mas fondo, mas error
    assert L.ber_ppm(5.0, 0.1, 16) == pytest.approx(L.ser_ppm(5.0, 0.1, 16) * 16 / 30, rel=1e-12)


def test_capacidad_ppm_dura():
    M = 16
    # sin errores: log2 M bits por simbolo; con SER maximo (M-1)/M, cero informacion
    assert L.capacidad_ppm_dura(40.0, 0.0, M) == pytest.approx(4.0, abs=1e-6)
    assert L.capacidad_ppm_dura(0.001, 1.0, M) < 0.1
    c = [L.capacidad_ppm_dura(ns, 0.1, M) for ns in (1, 3, 6, 12)]
    assert all(a < b for a, b in zip(c, c[1:])) and c[-1] < 4.0 + 1e-9


def test_modem_ppm_sensibilidad_en_fotones():
    m = L.ModemPPM()
    s = m.sensibilidad_w(1e8, A.LAM_1550, 1e-4)
    ns, nb = m.fotones(s, 1e8, A.LAM_1550)
    # unos 8-10 fotones detectados por pulso para BER 1e-4 (PPM-16, fondo despreciable): ~2 fotones por bit
    assert 5.0 < ns < 14.0
    assert m.ber(s, 1e8, A.LAM_1550) == pytest.approx(1e-4, rel=0.05)
    # el PPM de pocos fotones es MUCHO mas sensible que el OOK con APD (el conteo de fotones gana ~15-20 dB)
    ook = L.ModemOOK().sensibilidad_w(1e8 * 4 / 16, A.LAM_1550)
    assert 10 * math.log10(ook / s) > 8.0


# ------------------------------------------------------------------ OOK y FEC

def test_ook_apd_monotonia_y_sensibilidad():
    m = L.ModemOOK()
    assert m.ber(1e-9, 1e9, A.LAM_1550) < m.ber(1e-10, 1e9, A.LAM_1550)
    # a mas tasa, hace falta mas potencia (ruido termico + disparo con B = 0.7 tasa)
    sens = [m.sensibilidad_w(r, A.LAM_1550) for r in m.tasas()]
    assert all(a < b for a, b in zip(sens, sens[1:]))
    assert m.tasa_util(1e9) == pytest.approx(1e9 * 239 / 255)


def test_rs_255_239():
    # RS(255,239): corrige 8 simbolos; entrada 1e-4 -> salida < 1e-12; entrada 1e-3 -> ~5e-6; monotona
    assert L.ber_salida_rs(1e-4) < 1e-12
    assert 1e-6 < L.ber_salida_rs(1e-3) < 1e-5
    b = [L.ber_salida_rs(p) for p in (1e-4, 1e-3, 3e-3, 1e-2)]
    assert all(x < y for x, y in zip(b, b[1:]))
    # por encima del umbral la FEC ya no ayuda: salida del orden de la entrada
    assert L.ber_salida_rs(5e-2) > 1e-2


# ------------------------------------------------------------------ presupuesto

def test_opals_ganancia_70_4_dB():
    # OPALS publica FWHM ~ 1 mrad y ganancia de transmision 70.4 dB: G = 8/theta_h^2 con theta_h = FWHM/1.177
    g = 10 * math.log10(8.0 / (L.OPALS_SAT.theta_h_urad * 1e-6) ** 2)
    assert g == pytest.approx(70.4, abs=0.1)


def test_opals_potencia_recibida_cientos_de_nanowatts():
    # OPALS en el OCTL (2200 m) a 30 deg sobre el horizonte (~820 km): la prediccion publicada fue de "cientos de nW";
    # la medida, decenas de nW por las perdidas atmosfericas no modeladas
    atm = dataclasses.replace(HV, alt_estacion_m=2200.0)
    p = L.presupuesto_bajada(L.OPALS_SAT, L.OPALS_EST, atm, 30.0, 820.0)['p_rx_w']
    assert 1e-7 < p < 1e-6, p


def test_cuadre_del_presupuesto_en_dB():
    sat, est = L.TerminalSat(), L.EstacionOptica()
    for el, R in ((20.0, 1300.0), (60.0, 620.0)):
        b = L.presupuesto_bajada(sat, est, HV, el, R)
        # P_rx(dBm) = suma de las partidas (dBW) + 30
        assert sum(b['db'].values()) + 30.0 == pytest.approx(b['p_rx_dbm'], abs=1e-6)
    s = L.presupuesto_subida(sat, est, HV, 45.0, 700.0)
    assert sum(s['db'].values()) + 30.0 == pytest.approx(s['p_rx_dbm'], abs=1e-6)


def test_la_potencia_cae_con_la_distancia_y_con_el_error_de_apuntamiento():
    sat, est = L.TerminalSat(), L.EstacionOptica()
    # espacio libre 20 log R
    p1 = L.presupuesto_bajada(sat, est, HV, 90.0, 500.0, 0.0)['p_rx_w']
    p2 = L.presupuesto_bajada(sat, est, HV, 90.0, 1000.0, 0.0)['p_rx_w']
    assert 10 * math.log10(p1 / p2) == pytest.approx(6.02, abs=0.5)         # + la pequena diferencia atmosferica a igual el: ninguna
    # el error residual del ATP degrada el acoplo: con un campo de 12 urad un error de 15 urad rms pierde casi todo
    bien = L.presupuesto_bajada(sat, est, HV, 45.0, 700.0, 0.5)['p_rx_w']
    mal = L.presupuesto_bajada(sat, est, HV, 45.0, 700.0, 15.0)['p_rx_w']
    assert mal < 0.35 * bien
    # receptor monomodo: perdida por apuntamiento 1/(1 + 4 s^2/theta^2) con theta = 0.956 lam/D = 3.7 urad
    smf = dataclasses.replace(est, receptor='smf')
    a = L.acoplo_receptor(smf, A.LAM_1550, HV, 45.0, 0.0)
    b = L.acoplo_receptor(smf, A.LAM_1550, HV, 45.0, 3.7)
    assert b / a == pytest.approx(1.0 / (1.0 + 4.0 * 3.7 ** 2 / (1e6 * E.semiangulo_haz_rad(0.4, A.LAM_1550)) ** 2), rel=0.02)


def test_subida_beam_wander_y_precompensacion():
    sat, est = L.TerminalSat(), L.EstacionOptica()
    base = L.presupuesto_subida(sat, est, HV, 45.0, 700.0)
    comp = L.presupuesto_subida(sat, dataclasses.replace(est, corr_tilt_up=0.8), HV, 45.0, 700.0)
    assert comp['p_rx_w'] > base['p_rx_w']
    assert base['beam_wander_urad'] > 2.0
    # un haz mas ancho tolera mejor el beam wander (menos perdida por apuntamiento) pero tiene menos ganancia
    ancho = L.presupuesto_subida(sat, dataclasses.replace(est, theta_up_urad=40.0), HV, 45.0, 700.0)
    assert ancho['perdida_apunt_db'] > base['perdida_apunt_db']


def test_la_tasa_baja_con_la_elevacion_y_el_outage_respeta_el_limite():
    sat, est = L.TerminalSat(), L.EstacionOptica()
    atm = dataclasses.replace(HV, alt_estacion_m=2240.0)
    el = (10, 20, 30, 60, 90)
    rangos = (1850.0, 1330.0, 1000.0, 620.0, 550.0)
    filas = [L.evaluar_enlace(sat, est, atm, e, r, 0.6, 1e-2) for e, r in zip(el, rangos)]
    tasas = [f['tasa_bps'] for f in filas]
    assert all(a <= b for a, b in zip(tasas, tasas[1:]))
    assert tasas[-1] > tasas[0]
    for f in filas:
        assert f['outage'] <= 1e-2
    # el margen de desvanecimiento es mayor a baja elevacion (mas centelleo)
    assert filas[0]['margen_desv_db'] > filas[-1]['margen_desv_db']


def test_ber_medio_con_centelleo_supera_al_de_potencia_media():
    # E[BER(P I)] >= BER(P) (convexidad de la cola Q): el desvanecimiento siempre cuesta
    m = L.ModemOOK()
    d = L.Desvanecimiento(0.2, 'lognormal')
    p = 10 ** (-33.0 / 10) * 1e-3
    assert L.ber_con_centelleo(m, p, 1e9, A.LAM_1550, d) > float(m.ber(p, 1e9, A.LAM_1550))
