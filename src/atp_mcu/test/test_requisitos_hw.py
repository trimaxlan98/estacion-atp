"""Pruebas de scripts/requisitos_hw.py (equipo S): cada cifra contra una fuente independiente."""
import math
import os
import sys

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(RAIZ, 'scripts'))
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_ros'))

rh = pytest.importorskip('requisitos_hw')
from atp_ros import model  # noqa: E402


def test_reparto_suma_uno():
    assert abs(sum(rh.REPARTO_RSS.values()) - 1.0) < 1e-12


def test_presupuesto_rss_recompone_el_total():
    p = rh.presupuesto(0.1)
    assert math.sqrt(sum(v * v for v in p.values())) == pytest.approx(0.1, rel=1e-12)


def test_cinematica_numerica_coincide_con_la_analitica():
    # model.tasa_acimut es la formula cerrada en la culminacion
    for h in rh.ALTURAS_KM:
        for el in (50.0, 80.0, 85.0):
            c = rh.cinematica_pase(h, el, n=40000)
            assert c['v_az'] == pytest.approx(c['v_az_analitica'], rel=0.01)


def test_cifras_conocidas_del_repositorio():
    # CONTROL.md: a 550 km, rotor de 6 deg/s -> keyhole en ~82.5 deg; CONTROL: ~4 % de pases lo superan
    assert model.elevacion_keyhole(550.0, 6.0) == pytest.approx(82.5, abs=0.1)
    assert rh.fraccion_pases_sobre(550.0, 82.5) == pytest.approx(0.04, abs=0.01)
    # el pase cenital N2 sigue a 9 deg/s a 85 deg
    assert rh.cinematica_pase(550.0, 85.0, n=40000)['v_az'] == pytest.approx(9.0, abs=0.1)


def test_aceleracion_crece_con_la_elevacion():
    a = [rh.cinematica_pase(550.0, el, n=30000)['a_az'] for el in (50.0, 70.0, 85.0, 88.0)]
    assert a == sorted(a) and a[-1] > 5.0


def test_elevacion_sobre_el_cenit_no_tiene_singularidad():
    r = rh.elevacion_hasta_180(550.0)
    assert r['v_el_pico'] == pytest.approx(r['v_el_analitica'], rel=0.02)   # v/h
    assert r['v_el_pico'] < 1.0


def test_clases_de_montura_monotonas():
    filas = rh.tabla_keyhole((550.0,))
    el = [f['el_max_alcanzable'] for f in filas]
    assert el == sorted(el)                                  # mas rapido, mas alto
    assert filas[0]['el_max_por_velocidad'] == pytest.approx(82.5, abs=0.1)


def test_ancho_de_haz_platos():
    assert rh.theta3_plato(3.0, 2.2e9) == pytest.approx(3.18, abs=0.01)    # model / PUESTA_EN_MARCHA
    assert rh.theta3_plato(3.0, 30e9) == pytest.approx(0.233, abs=0.001)
    # Yagi 12 dBi: Kraus ~ 51 deg; 70 lambda/D con boom de 3 m a 145.8 MHz da 48 deg
    assert rh.theta3_yagi(12.0) == pytest.approx(51.0, abs=0.5)
    assert model.ancho_haz(3.0, 145.8e6) == pytest.approx(48.0, abs=1.0)


def test_encoder_cuantizacion_regla_widrow():
    e = rh.encoder_necesario(0.0833)
    assert e['q_deg'] / math.sqrt(12.0) == pytest.approx(e['e_enc_rms_deg'])
    assert 360.0 / e['cuentas_vuelta_carga'] == pytest.approx(e['q_deg'])
    assert 2 ** e['bits_carga'] >= e['cuentas_vuelta_carga'] > 2 ** (e['bits_carga'] - 1)
    # CONTROL sec. 8: con 14 bits (0.022 deg) la cuantizacion ya consume ~un quinto de 0.1 deg
    assert (360.0 / 2 ** 14) / math.sqrt(12.0) / 0.1 == pytest.approx(0.0634, abs=0.001)


def test_ruido_de_velocidad_formula_es_cota_alta():
    f, m = rh.verificar_ruido_velocidad(n=60000)
    assert 0.0 < m <= f


def test_par_viento_del_curso():
    assert rh.par_viento(10.0, math.pi * 1.5 ** 2, 1.2, 0.15) == pytest.approx(78.0, abs=0.5)


def test_motor_coherente():
    e = rh.ESCENARIOS['C plato 1-3 m en S/X (la Estacion)']
    m = rh.dimensionar_motor(e, 30.0, 30.0)
    d = m['detalle_par']
    assert d['total'] == pytest.approx(d['inercial'] + d['viscoso'] + d['friccion'] + d['viento'] + d['desbalance'])
    assert d['inercial'] == pytest.approx(110.0 * math.radians(30.0))
    assert m['par_motor_nm'] * m['reduccion'] * 0.85 == pytest.approx(m['par_carga_nm'])


def test_slew_trapezoidal_cierra_el_tiempo():
    v = rh.v_slew_necesaria(180.0, 20.0, 30.0)
    assert 180.0 / v + v / 30.0 == pytest.approx(20.0)
    assert math.isnan(rh.v_slew_necesaria(180.0, 1.0, 10.0))


def test_latencia_con_y_sin_extrapolacion():
    l = rh.lazo_y_latencia(0.0833, 9.0, 1.0)
    assert l['T_max_sin_comp_ms'] == pytest.approx(1e3 * l['e_lat_deg'] / 9.0)
    assert l['T_max_con_extrapolacion_ms'] > l['T_max_sin_comp_ms']


def test_paso_a_paso_cumple_su_resolucion():
    r = rh.resolucion_paso_a_paso(0.1, 20.0)
    assert r['q_deg'] <= math.sqrt(12.0) * rh.presupuesto(0.1)['encoder (cuantizacion)']
    assert r['q_deg'] == pytest.approx(360.0 / (200 * 16 * r['reduccion']))


def test_calcular_rapido_completo():
    r = rh.calcular(rapido=True)
    assert set(r['escenarios']) == set(rh.ESCENARIOS)
    assert r['escenarios']['C plato 1-3 m en S/X (la Estacion)']['motor']['par_carga_nm'] > 100.0


def test_cenital_con_tope_mejora_con_mas_motor():
    a = rh.cenital_con_tope(550.0, 6.0, 2.0)
    b = rh.cenital_con_tope(550.0, 90.0, 400.0)
    assert b['err_cielo_max_deg'] < a['err_cielo_max_deg']


# ------------------------------------------------------------------ RF
def test_doppler_uhf_550km_contra_model():
    d = rh.doppler_pase(550.0, 437e6)
    # model.tasa_doppler_cenit: 141 Hz/s a 437 MHz y 550 km (hallazgo del equipo de RF, atp_sdr)
    assert d['tasa_cenit_analitica_hz_s'] == pytest.approx(140.5, abs=1.0)
    assert d['tasa_max_hz_s'] == pytest.approx(d['tasa_cenit_analitica_hz_s'], rel=0.03)
    # fd = vr/c f0 con vr ~ 7.4 km/s en AOS: ~ +-10 kHz
    assert d['fd_max_hz'] == pytest.approx(10.1e3, rel=0.03)


def test_doppler_escala_con_la_frecuencia():
    a = rh.doppler_pase(550.0, 145.8e6)['fd_max_hz']
    b = rh.doppler_pase(550.0, 437e6)['fd_max_hz']
    assert b / a == pytest.approx(437e6 / 145.8e6, rel=1e-6)


def test_resintonia():
    assert rh.intervalo_resintonia(140.5, 500.0) == pytest.approx(3.559, rel=1e-3)


def test_enlace_yagi_437_pase_tipico():
    # EIRP 0 dBW, Yagi 12 dBi, 300 K, 3 dB de perdidas; FSPL = 92.45 + 20 log d + 20 log f_GHz
    el5 = rh.c_n0_enlace(437e6, 0.0, 12.0, 300.0, rh.rango_pase_km(550.0, 5.0))
    el90 = rh.c_n0_enlace(437e6, 0.0, 12.0, 300.0, rh.rango_pase_km(550.0, 90.0))
    assert el90 - el5 == pytest.approx(20.0 * math.log10(rh.rango_pase_km(550.0, 5.0) / 550.0), abs=1e-6)
    assert 55.0 < el5 < 70.0                     # decenas de dB-Hz de sobra para telemetria de kbit/s


def test_banco_conducido_cierra_las_cuentas():
    b = rh.banco_conducido(45.0, 2.4e6, 5.0, -20.0)
    assert b['n_termico_rx_dbm'] == pytest.approx(-174.0 + 5.0 + 63.80, abs=0.01)
    assert b['p_rx_compuesta_dbm'] == pytest.approx(b['n_termico_rx_dbm'] + 20.0)
    assert b['atenuacion_total_db'] == pytest.approx(-20.0 - b['p_rx_compuesta_dbm'])
    # C/N0 = P_senal - N0 inyectada
    assert b['p_senal_dbm'] - b['n0_inyectada_dbm_hz'] == pytest.approx(45.0)
    assert b['snr_banda_db'] < 0.0               # la senal va bajo el ruido en el ancho de banda, como en el cielo


def test_aislamiento():
    assert rh.aislamiento_necesario(-20.0, -85.0) == pytest.approx(85.0)
