"""Pruebas de sensores y estimador (ronda D3): contra la fisica conocida, no solo que corra."""
import numpy as np
import pytest

from atp_dron import estudios as E
from atp_dron.dron import G0, Dron
from atp_dron.lazo import correr
from atp_dron.sensores import Sensores


def test_acelerometro_en_reposo_mide_g_hacia_arriba():
    d = Dron('tierra')
    s = Sensores(d, {'acel_ruido': 0.0, 'acel_sesgo0': 0.0, 'acel_sesgo_rw': 0.0}, 0)
    assert d.fuerza_especifica() == pytest.approx([0, 0, G0])
    _, a = s.imu()
    assert a == pytest.approx([0, 0, G0])


def test_acelerometro_en_hover_mide_g_y_en_caida_libre_cero():
    d = Dron('tierra', pos=(0, 0, 50.0))
    d.om[:] = d.empuje_a_om(np.full(4, d.m * d.g / 4))
    assert d.fuerza_especifica()[2] == pytest.approx(d.g, rel=1e-6)
    d.om[:] = 0.0
    assert np.linalg.norm(d.fuerza_especifica()) == pytest.approx(0.0, abs=1e-9)


def test_sensores_no_alteran_la_dinamica_ni_los_bytes_sin_sensores():
    c = {'ruta': [[5, 0, 4]], 'duracion_s': 8.0}
    assert correr(c) == correr({**c, 'sensores': False})
    m, _ = correr({**c, 'sensores': True})
    assert 'est_error_pos_rms_m' in m


def test_estimador_mejora_al_gps_crudo_y_estima_actitud():
    m, _ = correr({'ruta': [[30, 0, 10], [30, 30, 10], [0, 0, 10]], 'duracion_s': 90.0, 'viento': [2.0, 0, 0],
                   'rafaga_sigma': 1.0, 'semilla': 5, 'sensores': True})
    assert m['llego'] and not m['choque']
    assert m['est_error_pos_rms_m'] < 0.5          # GPS crudo: 0.8 m horizontal y 1.5 m vertical
    assert m['est_error_actitud_rms_deg'] < 2.0
    assert m['est_sesgo_gyro_error_rads'] < 0.02


def test_sensores_reproducibles_y_dependen_de_la_semilla():
    c = {'ruta': [[10, 0, 5]], 'duracion_s': 12.0, 'sensores': True}
    assert correr({**c, 'semilla': 3}) == correr({**c, 'semilla': 3})
    assert correr({**c, 'semilla': 3})[0]['est_error_pos_rms_m'] != correr({**c, 'semilla': 4})[0]['est_error_pos_rms_m']


def test_sin_gps_el_error_crece():
    f = E.perdida_gps(cortes_s=(None, 10.0), duracion_s=40.0)
    assert f[1]['est_error_pos_final_m'] > 2 * f[0]['est_error_pos_final_m']
    assert f[0]['est_error_pos_final_m'] < 1.0
