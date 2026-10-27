"""Formato de lo que graba `adcs_recorder` (CONTRATO 2.1 y 3.1), sin lanzar ROS.

Se encadenan planta, controlador y medidor 30 pasos (lo que hace el lockstep) y se
pasa cada paso por `fila_de`; `metricas_de` se prueba aqui con ese Medidor corto y la
corrida completa se comprueba en la integracion del backend (~1 min).
"""
import json
import math

import pytest

from sat_adcs import misiones as M
from sat_adcs.escenario import crear, preparar
from sat_adcs.nodo_grabador import _limpio, fila_de, metricas_de, segmentos_de_modo

CLAVES = {'t', 'mode', 'w_deg_s', 'q', 'q_est', 'h_nms', 'eclipse', 'el_deg', 'range_km',
          'in_pass', 'u_w_nm', 'm_am2', 'err_deg', 'know_deg', 'sigma_deg'}


@pytest.fixture(scope='module')
def corta():
    cfg = M.Config(inicio_utc='2026-10-02T21:20:00', anticipo_min_s=3000.0, w0_deg_s=3.0)
    esc = preparar(cfg)
    planta, ctrl, med = crear(esc)
    filas = []
    for _ in range(30):
        s = planta.medir()
        cmd = ctrl.paso(s['t'], s)
        med.muestra(s, cmd)
        filas.append(fila_de(s['t'], cmd['modo'], s, cmd, med, med.geo.en(s['t'])))
        planta.avanzar(cfg.paso_s, cmd['u_w'], cmd['m'])
    return cfg, med, filas


def test_filas_json_y_claves(corta):
    _, _, filas = corta
    assert len(filas) == 30
    for f in filas:
        assert set(f) <= CLAVES and {'t', 'mode', 'w_deg_s', 'q', 'h_nms', 'u_w_nm'} <= set(f)
        assert len(f['q']) == 4 and len(f['h_nms']) == 3 and len(f['u_w_nm']) == 3
        json.dumps(f, allow_nan=False)                  # nada de NaN/inf: JSON valido
        assert all(v is not None for v in f.values())   # sin dato = campo omitido
    ts = [f['t'] for f in filas]
    assert ts == sorted(ts) and ts[0] == 0.0 and ts[1] - ts[0] == 1.0
    assert all(f['mode'] == M.MODO_DETUMBLE for f in filas)
    # en DETUMBLE no hay objetivo de apuntamiento: err_deg se omite
    assert all('err_deg' not in f for f in filas)
    # el cuaternion es unitario
    assert all(abs(math.sqrt(sum(x * x for x in f['q'])) - 1.0) < 1e-6 for f in filas)


def test_segmentos_de_modo():
    seg = segmentos_de_modo([0, 1, 2, 3, 4, 5], [0, 0, 1, 1, 2, 1])
    assert [(x['name'], x['t0'], x['t1']) for x in seg] == [
        ('DETUMBLE', 0, 1), ('NADIR', 2, 3), ('TRACK_GS', 4, 4), ('NADIR', 5, 5)]
    assert segmentos_de_modo([], []) == []


def test_limpio_quita_nan():
    x = _limpio({'a': float('nan'), 'b': [1.0, float('inf')], 'c': {'d': 2}})
    assert x == {'a': None, 'b': [1.0, None], 'c': {'d': 2}}
    json.dumps(x, allow_nan=False)


def test_metricas_con_corrida_corta(corta):
    cfg, med, _ = corta
    m = metricas_de(med, cfg)
    json.dumps(m, allow_nan=False)
    assert m['kind'] == 'adcs' and m['n'] == 30 and m['verdict'] == 'NO CUMPLE'
    assert [s['name'] for s in m['modes']] == ['DETUMBLE']
    assert m['track']['threshold_deg'] == pytest.approx(M.umbral_apuntamiento_deg(cfg))
    assert m['track']['p95'] is None                    # sin pase en 30 s: NaN -> null
    assert m['wheels']['capacity_nms'] == pytest.approx(1.77e-3)
