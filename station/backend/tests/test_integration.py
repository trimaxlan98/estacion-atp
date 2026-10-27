"""Integracion REAL: lanza `ros2 launch` de verdad (solo dentro del contenedor,
con install/setup.bash cargado). Se omite si no hay ros2 en el PATH.

Solo se comprueba el FORMATO del contrato, no el veredicto: con el reloj
acelerado y la maquina cargada la fisica puede degradarse, y eso no es lo
que prueba este archivo.
"""
import json
import shutil

import pytest

from conftest import wait_status

pytestmark = [pytest.mark.integration,
              pytest.mark.skipif(shutil.which('ros2') is None, reason='sin ros2 en el PATH')]

TELEMETRY_KEYS = {'t', 'ref_az', 'ref_el', 'az', 'el', 'e_az', 'e_el', 'e_pt',
                  'range_km', 'range_rate_km_s', 'doppler_hz', 'doppler_rate_hz_s',
                  'l_point_db', 'cn0_dbhz', 'ebn0_db', 'margin_db', 'link_closed',
                  'u_az', 'u_el'}


@pytest.fixture
def real_client(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from station.app import create_app
    from station.config import Settings
    monkeypatch.setenv('STATION_RUNS_DIR', str(tmp_path / 'runs'))
    monkeypatch.setenv('STATION_DB_PATH', str(tmp_path / 'db' / 's.db'))
    monkeypatch.setenv('STATION_FRONTEND_DIST', str(tmp_path / 'sin-frontend'))
    monkeypatch.setenv('STATION_CUPO_PESADO', '2')      # la prueba de dominios necesita 2 a la vez
    s = Settings()
    # lanzador real (ros2 launch); la carga del host se simula para que la prueba no dependa de ella
    with TestClient(create_app(s, load_reader=lambda: 0.0)) as c:
        c.settings = s
        yield c


def check_telemetry(path):
    rows = [json.loads(ln) for ln in path.read_text().splitlines()]
    assert len(rows) > 50
    for r in rows:
        assert set(r) <= TELEMETRY_KEYS and 't' in r
        assert all(v is not None for v in r.values())    # sin dato = campo omitido
    ts = [r['t'] for r in rows]
    assert ts == sorted(ts) and ts[0] == 0.0
    # 10 Hz de tiempo simulado: ~0.1 s entre muestras
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    assert 0.09 < sorted(gaps)[len(gaps) // 2] < 0.35
    # dentro del pase (entre AOS y LOS, sin los extremos) toda fila trae la
    # medicion completa; en AOS y tras LOS no hay error ni enlace (CONTRATO 8)
    completas = {'ref_az', 'az', 'e_pt', 'range_km', 'doppler_hz', 'margin_db',
                 'link_closed', 'u_az'}
    en_pase = [r for r in rows if 0.0 < r['t'] < ts[-1]]
    assert en_pase and all(completas <= set(r) for r in en_pase)
    return rows


def check_metrics(m):
    assert m['n'] > 100 and m['duration_s'] > 0
    assert {'rms', 'p95', 'max', 'within_objective_pct', 'objective_deg'} <= set(m['pointing'])
    assert set(m['axes']) == {'az', 'el'} and set(m['axes']['az']) == {'rms', 'max'}
    assert {'band', 'margin_min_db', 'margin_max_db', 'closed_pct', 'doppler_max_hz',
            'doppler_min_hz', 'doppler_rate_max_hz_s', 'l_point_max_db'} <= set(m['link'])
    assert m['verdict'] in ('CUMPLE', 'NO CUMPLE')


def test_n2_real(real_client):
    c = real_client
    r = c.post('/api/runs', json={'experiment_id': 'n2-keyhole-cenital',
                                  'args': {'time_scale': 8}})
    assert r.status_code == 201
    rid = r.json()['run_id']
    det = wait_status(c, rid, timeout=180)
    log = c.get(f'/api/runs/{rid}/log').text
    assert det['status'] == 'done', log[-3000:]
    d = c.settings.runs_dir / rid
    rows = check_telemetry(d / 'telemetry.jsonl')
    m = json.loads((d / 'metrics.json').read_text())
    check_metrics(m)
    assert m['link']['band'] == 'S'
    assert abs(m['duration_s'] - 50.9) < 0.5      # duracion del pase a 85/70 deg
    assert rows[-1]['t'] <= m['duration_s'] + 0.2
    assert det['metrics'] == m                    # la API sirve lo mismo que el disco
    assert 100 <= det['ros_domain_id'] <= 199
    assert 'metrics.json escrito' in log
    assert len(c.get(f'/api/runs/{rid}/telemetry', params={'every': 5}).json()) > 10


def test_campana_real(real_client):
    c = real_client
    rid = c.post('/api/runs', json={'experiment_id': 'n4c-montecarlo-analitico',
                                    'args': {'n': 500}}).json()['run_id']
    det = wait_status(c, rid, timeout=120)
    assert det['status'] == 'done', c.get(f'/api/runs/{rid}/log').text[-3000:]
    cj = det['campaign']
    assert cj['mode'] == 'analitico' and cj['n'] == 500 and len(cj['rms_deg']) == 500
    assert {'p50', 'p95', 'p99', 'peor', 'pasa', 'objective_deg',
            'p95_uncertainty_pts', 'wall_time_s', 'el_max_deg'} <= set(cj)
    assert cj['p50'] <= cj['p95'] <= cj['p99'] <= cj['peor']


def test_dos_corridas_simultaneas_aisladas_por_dominio(real_client):
    """n1 (20 s de rampa) y n2 (50.9 s de pase) a la vez, en la misma red host:
    si compartieran dominio DDS se mezclarian /clock, /atp/reference, etc. y las
    duraciones medidas no serian las de cada experimento."""
    c = real_client
    a = c.post('/api/runs', json={'experiment_id': 'n1-rampa'}).json()['run_id']
    b = c.post('/api/runs', json={'experiment_id': 'n2-keyhole-cenital',
                                  'args': {'time_scale': 8}}).json()['run_id']
    da, db = wait_status(c, a, timeout=240), wait_status(c, b, timeout=240)
    assert da['status'] == db['status'] == 'done'
    assert da['ros_domain_id'] != db['ros_domain_id']
    assert abs(da['metrics']['duration_s'] - 20.0) < 0.5
    assert abs(db['metrics']['duration_s'] - 50.9) < 0.5


# ---------------------------------------------------------- kinds plan y adcs (reales)

ADCS_KEYS = {'t', 'mode', 'w_deg_s', 'q', 'q_est', 'h_nms', 'eclipse', 'el_deg', 'range_km',
             'in_pass', 'u_w_nm', 'm_am2', 'err_deg', 'know_deg', 'sigma_deg'}


def _chk_plan(d, quitada):
    """Invariantes de plan.json (CONTRATO 1) comprobados desde cero con las ventanas."""
    plan = json.loads((d / 'plan.json').read_text())
    ven = {v['id']: v for v in plan['ventanas']}
    assert len(ven) == plan['n_ventanas'] > 5 and sorted(ven) == list(range(len(ven)))
    for nombre, p in plan['planes'].items():
        assert set(p['ids']) <= set(ven), nombre
        sel = sorted((ven[i] for i in p['ids']), key=lambda v: v['aos'])
        # una antena por estacion: los contactos de una estacion no se solapan
        for est in {v['estacion'] for v in sel}:
            mias = [v for v in sel if v['estacion'] == est]
            assert all(a['los'] <= b['aos'] for a, b in zip(mias, mias[1:])), (nombre, est)
        # el resumen es la suma de las ventanas elegidas
        assert p['resumen']['contactos'] == len(sel)
        assert abs(p['resumen']['gb'] - sum(v['vol_gb'] for v in sel)) < 0.01 * max(1, len(sel))
    # el optimo no pierde contra ningun codicioso
    gb = {k: p['resumen']['gb'] for k, p in plan['planes'].items()}
    assert gb['optimo'] >= max(v for k, v in gb.items() if k != 'optimo') - 1e-6
    assert plan['brecha_codicioso_pct'] >= 0
    if quitada:
        ap = plan['aporte_estacion'][quitada]
        assert set(ap['ids_sin']) <= set(ven) and ap['gb_con'] >= ap['gb_sin'] - 1e-6
        assert all(ven[i]['estacion'] != quitada for i in ap['ids_sin'])     # ids de `ventanas`
        assert abs(ap['gb_sin'] - sum(ven[i]['vol_gb'] for i in ap['ids_sin'])) < 0.05
    return plan


def test_plan_real(real_client):
    """scripts/planificar.py de verdad (3 h de la red de 4 estaciones: unos segundos);
    dos corridas iguales dan el mismo plan.json byte a byte (fecha fija)."""
    c = real_client
    args = {'horas': 3, 'sin': 'tromso'}
    ids = [c.post('/api/runs', json={'experiment_id': 's6-plan-24h', 'args': args}).json()['run_id']
           for _ in range(2)]
    dets = [wait_status(c, rid, timeout=240) for rid in ids]
    for rid, det in zip(ids, dets):
        assert det['status'] == 'done', c.get(f'/api/runs/{rid}/log').text[-3000:]
    d = c.settings.runs_dir / ids[0]
    plan = _chk_plan(d, 'tromso')
    m = dets[0]['metrics']
    assert m['kind'] == 'plan' and m['plan'] == 'optimo'
    assert m['contactos'] == plan['planes']['optimo']['resumen']['contactos']
    assert m['estaciones'] == ['cdmx', 'tromso', 'ecuador', 'madrid'] and m['aporte_estacion']['tromso']
    assert dets[0]['plan'] == plan and dets[0]['kind'] == 'plan'
    assert (d / 'plan.json').read_bytes() == (c.settings.runs_dir / ids[1] / 'plan.json').read_bytes()
    assert 'Brecha del codicioso' in (d / 'resumen.txt').read_text()


def test_adcs_real_corta(real_client):
    """ros2 launch sat_adcs de verdad, escenario corto (pase en luz, tumbling de 3 deg/s,
    ~4000 pasos de 1 s: ~75 s). Solo el FORMATO (CONTRATO 2.1 y 3.1) y la coherencia de
    los modos; el veredicto es del equipo de ADCS."""
    c = real_client
    r = c.post('/api/runs', json={'experiment_id': 'e1-adcs-funcube', 'args': {
        'w0_deg_s': 3, 'lead_s': 3000, 'start_utc': '2026-10-02T21:20:00'}})
    assert r.status_code == 201
    rid = r.json()['run_id']
    det = wait_status(c, rid, timeout=600)
    log = c.get(f'/api/runs/{rid}/log').text
    assert det['status'] == 'done', log[-3000:]
    d = c.settings.runs_dir / rid
    rows = [json.loads(ln) for ln in (d / 'telemetry.jsonl').read_text().splitlines()]
    m = json.loads((d / 'metrics.json').read_text())
    assert det['metrics'] == m and det['kind'] == 'adcs' and 'RESUMEN veredicto' in log
    # una fila por paso de 1 s, t creciente desde 0 y sin filas tras el fin del escenario
    ts = [r['t'] for r in rows]
    assert ts[0] == 0.0 and all(b - a == 1.0 for a, b in zip(ts, ts[1:]))
    assert m['n'] == len(rows) and m['duration_s'] == ts[-1]
    for r in rows:
        assert set(r) <= ADCS_KEYS and {'t', 'mode', 'w_deg_s', 'q', 'h_nms', 'u_w_nm', 'm_am2'} <= set(r)
        assert all(v is not None for v in r.values())
        assert abs(sum(x * x for x in r['q']) - 1.0) < 1e-6 and r['mode'] in (0, 1, 2)
        assert ('err_deg' in r) == (r['mode'] != 0)               # DETUMBLE no tiene objetivo
    # modos: DETUMBLE -> NADIR -> TRACK_GS (-> NADIR), tramos contiguos que cubren el escenario
    seg = m['modes']
    assert [s['name'] for s in seg][:3] == ['DETUMBLE', 'NADIR', 'TRACK_GS']
    assert seg[0]['t0'] == 0.0 and seg[-1]['t1'] == ts[-1]
    assert all(b['t0'] - a['t1'] == 1.0 for a, b in zip(seg, seg[1:]))
    assert [r['mode'] for r in rows if r['t'] in (seg[1]['t0'], seg[2]['t0'])] == [1, 2]
    # el umbral sale del enlace (1 dB, haz de 8.3 deg) y TRACK_GS solo cuenta en pase
    assert abs(m['track']['threshold_deg'] - 2.404) < 0.01 and 0 < m['track']['n'] <= len(rows)
    assert m['wheels']['h_max_nms'] <= m['wheels']['capacity_nms'] + 1e-12
    assert m['verdict'] in ('CUMPLE', 'NO CUMPLE') and m['kind'] == 'adcs'
    assert 100 <= det['ros_domain_id'] <= 199
