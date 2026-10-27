"""Determinismo (P7): misma semilla, mismos bytes; semilla distinta, resultado distinto; banderas apagadas, bytes de antes."""
import json

import pytest

from atp_dron.cascada_indi import correr_indi
from atp_dron.lazo import correr

CFG = {'ruta': [[10, 0, 5], [10, 10, 5]], 'duracion_s': 14.0, 'rafaga_sigma': 1.0, 'viento': [2.0, 0, 0], 'sensores': True}

APAGADAS = {'q_corregida': False, 'h_mag_exacto': False, 'gps_gm_tau_s': 0.0, 'gps_latencia_s': 0.0,
            'est_acel_sesgo_rw': None, 'est_gyro_sesgo_rw': None}


def _bytes(cfg, carpeta):
    correr(cfg, str(carpeta))
    return {n: (carpeta / n).read_bytes() for n in ('telemetry.jsonl', 'metrics.json')}


def test_misma_semilla_mismos_bytes_y_semilla_distinta_difiere(tmp_path):
    a = _bytes({**CFG, 'semilla': 4}, tmp_path / 'a')
    b = _bytes({**CFG, 'semilla': 4}, tmp_path / 'b')
    c = _bytes({**CFG, 'semilla': 5}, tmp_path / 'c')
    assert a == b
    assert a['telemetry.jsonl'] != c['telemetry.jsonl'] and a['metrics.json'] != c['metrics.json']


def test_banderas_de_p7_apagadas_no_cambian_un_byte(tmp_path):
    base = _bytes({**CFG, 'semilla': 1}, tmp_path / 'base')
    explicitas = _bytes({**CFG, 'semilla': 1, 'sensores_params': dict(APAGADAS)}, tmp_path / 'expl')
    assert base == explicitas


@pytest.mark.parametrize('flag', [{'q_corregida': True}, {'h_mag_exacto': True}, {'gps_gm_tau_s': 10.0}, {'gps_latencia_s': 0.2}])
def test_cada_bandera_encendida_cambia_el_resultado_y_sigue_siendo_reproducible(flag):
    c = {**CFG, 'semilla': 1, 'duracion_s': 8.0}
    ref = correr(c)[0]
    a, b = correr({**c, 'sensores_params': flag})[0], correr({**c, 'sensores_params': flag})[0]
    assert a == b and a != ref


def test_indi_con_ganancia_cero_es_la_cascada_de_siempre_y_con_ganancia_uno_difiere():
    c = {'ruta': [[5, 0, 4]], 'duracion_s': 8.0, 'rafaga_sigma': 1.0, 'semilla': 3}
    m0, f0 = correr(c)
    mi, fi, _ = correr_indi(c, 0.0)
    assert json.dumps([m0, f0], sort_keys=True) == json.dumps([mi, fi], sort_keys=True)
    m1, _, _ = correr_indi(c, 1.0)
    assert m1 != m0
    assert correr_indi(c, 1.0)[0] == m1
