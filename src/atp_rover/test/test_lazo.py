"""Pruebas del lazo cerrado R4 (dinamica de R1 + navegador de R2, sin ROS). Corridas cortas para ahorrar tiempo."""
import json

import pytest

from atp_rover import cli, lazo
from atp_rover.navegacion import pendiente_deg, rugosidad
from atp_rover.planetas import planeta
from atp_rover.terreno import Terreno

CORTA = {'planeta': 'luna', 'terreno': 'llano', 'semilla': 1, 'ruta': [[4, 0]], 'duracion_s': 25, 'v_max': 0.5}


def test_terreno_de_r1_cumple_el_duck_type_de_r2():
    t = Terreno('pendiente', 0, planeta('luna')['suelo'], angulo_deg=12.0)
    assert pendiente_deg(t, 3.0, 1.0) == pytest.approx(12.0, abs=0.3)
    assert float(t.pendiente_deg(3.0, 1.0)) == pytest.approx(pendiente_deg(t, 3.0, 1.0), abs=1e-6)
    assert rugosidad(t, 3.0, 1.0) < 1e-3          # una rampa lisa no tiene rugosidad
    r = Terreno('rocoso', 3, planeta('luna')['suelo'])
    assert rugosidad(r, 20.0, 20.0) > 0.0


def test_defecto_basico_y_sin_claves_nuevas():
    assert cli.DEFECTOS['navegacion'] == 'basico'
    m, _ = cli.correr({**CORTA, 'duracion_s': 8})
    assert 'fallo' not in m and 'alarmas' not in m          # las metricas de R1 no cambian de forma
    m2, _ = lazo.correr({**CORTA, 'duracion_s': 8, 'navegacion': 'basico'})
    assert m == m2


def test_navegacion_desconocida():
    with pytest.raises(ValueError):
        lazo.correr({**CORTA, 'navegacion': 'magica'})


def test_completa_llega_y_es_determinista(tmp_path):
    cfg = {**CORTA, 'navegacion': 'completa'}
    m1, f1 = lazo.correr(cfg, str(tmp_path / 'a'))
    m2, f2 = lazo.correr(cfg, str(tmp_path / 'b'))
    assert m1['meta_alcanzada'] and m1['fallo'] is None and m1['vuelcos'] == 0
    assert m1['navegacion'] == 'completa'
    assert m1['energia_wh'] > 0.0                            # energia consumida, no regenerada
    for n in ('telemetry.jsonl', 'metrics.json'):
        assert (tmp_path / 'a' / n).read_bytes() == (tmp_path / 'b' / n).read_bytes()
    assert f1 == f2


def test_completa_meta_intransitable_da_sin_ruta():
    # R4 (completa_r4) rechaza una meta a 30 grados; R6 (completa) solo rechaza lo extremo (>= 35 grados) y a 30 intenta ir
    base = {**CORTA, 'terreno': 'pendiente', 'ruta': [[8, 0]], 'duracion_s': 10}
    m, f = lazo.correr({**base, 'terreno_params': {'angulo_deg': 30.0}, 'navegacion': 'completa_r4'})
    assert m['fallo'] == 'sin_ruta' and not m['meta_alcanzada']
    assert m['distancia_m'] < 0.5                            # no se mueve
    m, f = lazo.correr({**base, 'terreno_params': {'angulo_deg': 30.0}, 'navegacion': 'completa'})
    assert m['fallo'] != 'sin_ruta'
    m, f = lazo.correr({**base, 'terreno_params': {'angulo_deg': 40.0}, 'navegacion': 'completa'})
    assert m['fallo'] == 'sin_ruta' and m['distancia_m'] < 0.5


def test_cli_acepta_navegacion(tmp_path, capsys):
    rc = cli.main(['--planeta=luna', '--terreno=llano', '--semilla=1', '--ruta=[[3,0]]', '--duracion_s=20',
                   '--navegacion=completa', '--salida', str(tmp_path), '--id', 'x'])
    assert rc == 0
    met = json.loads((tmp_path / 'x' / 'metrics.json').read_text())
    assert met['navegacion'] == 'completa' and met['meta_alcanzada']


def _estudio():
    import importlib.util
    import os
    ruta = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts', 'estudio_rover.py')
    sp = importlib.util.spec_from_file_location('estudio_rover', ruta)
    m = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(m)
    return m


def test_estudio_configuraciones_y_agregado():
    est = _estudio()
    full, rap = est.configuraciones(False), est.configuraciones(True)
    assert len(full) == 5 * 5 * 3 * 2 + 5 * 6 and len(rap) == 2 * 3 * 2 + 2 * 2
    claves = {est._clave(c) for c in full}
    assert all(est._clave(c) in claves for c in rap)          # el rapido es un subconjunto: comparte cache
    met = {'distancia_m': 20.0, 'tiempo_s': 50.0, 'energia_wh': 0.5, 'deslizamiento_medio': 0.1, 'pendiente_max_deg': 3.0,
           'atascos': 0, 'vuelcos': 0, 'meta_alcanzada': True}
    filas = [{'cfg': c, 'met': dict(met)} for c in rap if c['estudio'] == 'matriz']
    agg = est.agregar(filas)
    assert len(agg) == 12 and all(a['meta_alcanzada'] == a['n'] == 1 for a in agg)
    assert agg[0]['wh_por_m'] == pytest.approx(0.025)
