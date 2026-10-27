"""API de la Estacion con un lanzador falso (sin ROS)."""
import os
import re
import time

import pytest

from conftest import parse_sse, wait_status

RUN_ID = re.compile(r'^\d{8}-\d{6}-[0-9a-f]{6}$')


def post_run(client, exp='n4a-pase-oblicuo-s', args=None):
    r = client.post('/api/runs', json={'experiment_id': exp, 'args': args})
    assert r.status_code == 201, r.text
    assert r.json()['status'] == 'queued'
    assert RUN_ID.match(r.json()['run_id'])
    return r.json()['run_id']


def test_health(client):
    j = client.get('/api/health').json()
    assert j['ok'] is True and j['active_runs'] == 0 and 'ros_distro' in j


def test_catalogo(client):
    lista = client.get('/api/experiments').json()
    ids = {e['id'] for e in lista}
    assert {'n1-rampa', 'n4c-montecarlo-lazo'} <= ids
    e = client.get('/api/experiments/n4a-pase-oblicuo-s').json()
    assert e['params_schema']['band']['options'] == ['UHF', 'S', 'X', 'Ka']
    assert client.get('/api/experiments/no-existe').status_code == 404


def test_corrida_completa(client):
    rid = post_run(client, args={'el_max': 40})
    r = wait_status(client, rid)
    assert r['status'] == 'done', r
    assert r['args']['el_max'] == 40 and r['args']['band'] == 'S'   # usuario + defaults
    assert r['metrics']['verdict'] == 'CUMPLE'
    assert r['campaign'] is None
    assert r['started_at'] and r['finished_at'] and r['exit_code'] == 0
    # dominio ROS propio del pool 100-199, registrado en params.json y en el log
    assert 100 <= r['ros_domain_id'] <= 199
    log = client.get(f'/api/runs/{rid}/log').text
    assert f'dominio {r["ros_domain_id"]}' in log
    # archivos del CONTRATO 1
    d = client.settings.runs_dir / rid
    assert {'params.json', 'launch.log', 'telemetry.jsonl', 'metrics.json'} <= \
        {p.name for p in d.iterdir()}
    import json
    p = json.loads((d / 'params.json').read_text())
    assert p['experiment_id'] == 'n4a-pase-oblicuo-s' and p['ros_domain_id'] == r['ros_domain_id']
    assert set(p) == {'experiment_id', 'launch', 'args', 'ros_domain_id', 'created_at'}


def test_corrida_campana(client):
    rid = post_run(client, 'n4c-montecarlo-analitico', {'n': 50})
    r = wait_status(client, rid)
    assert r['status'] == 'done'
    assert r['campaign']['p95'] == 0.05 and r['metrics'] is None


def test_listado_y_filtro(client):
    a = post_run(client, 'n1-rampa')
    b = post_run(client, 'n4a-pase-oblicuo-s')
    wait_status(client, a), wait_status(client, b)
    todas = client.get('/api/runs').json()
    assert {a, b} <= {r['run_id'] for r in todas}
    assert set(todas[0]) == {'run_id', 'experiment_id', 'title', 'status', 'created_at',
                             'finished_at', 'args', 'metrics'}
    solo = client.get('/api/runs', params={'experiment_id': 'n1-rampa', 'limit': 5}).json()
    assert [r['run_id'] for r in solo] == [a]
    assert solo[0]['title'] and solo[0]['metrics']['n'] == 8
    assert client.get('/api/runs', params={'experiment_id': '../x'}).json() == []
    assert client.get('/api/runs', params={'limit': 0}).status_code == 422


@pytest.mark.parametrize('body,code', [
    ({'experiment_id': 'no-existe'}, 404),
    ({'experiment_id': '../../etc'}, 404),
    ({'experiment_id': 'n4a-pase-oblicuo-s', 'args': {'mask': 50}}, 422),      # rango
    ({'experiment_id': 'n4a-pase-oblicuo-s', 'args': {'record_dir': '/tmp'}}, 422),  # clave ajena
    ({'experiment_id': 'n4a-pase-oblicuo-s', 'args': {'band': 'Q'}}, 422),     # enum
    ({'experiment_id': 'n4a-pase-oblicuo-s', 'args': {'el_max': 'alto'}}, 422),  # tipo
    ({'experiment_id': 'n4a-pase-oblicuo-s', 'args': [1]}, 422),
    ({'args': {}}, 422),
])
def test_post_invalido(client, body, code):
    assert client.post('/api/runs', json=body).status_code == code
    assert client.get('/api/runs').json() == []       # nada quedo encolado


@pytest.mark.parametrize('rid', ['..', '../../etc/passwd', 'x', '20200101-000000-ZZZZZZ',
                                 '20200101-000000-abcdef',      # formato valido, no existe
                                 '..%2f..%2fetc%2fpasswd', '%2e%2e'])
@pytest.mark.parametrize('ruta', ['', '/telemetry', '/log', '/stream', '/export.csv'])
def test_ids_estrictos(client, rid, ruta):
    assert client.get(f'/api/runs/{rid}{ruta}').status_code == 404
    assert client.post(f'/api/runs/{rid}/cancel').status_code == 404
    assert client.delete(f'/api/runs/{rid}').status_code == 404


def test_telemetria_log_csv(client):
    rid = post_run(client)
    wait_status(client, rid)
    t = client.get(f'/api/runs/{rid}/telemetry').json()
    assert len(t) == 8 and t[0]['t'] == 0.0 and 'az' in t[0]
    assert len(client.get(f'/api/runs/{rid}/telemetry', params={'every': 3}).json()) == 3
    assert client.get(f'/api/runs/{rid}/telemetry', params={'every': 0}).status_code == 422
    assert 'muestra 7' in client.get(f'/api/runs/{rid}/log').text
    csv_r = client.get(f'/api/runs/{rid}/export.csv')
    assert csv_r.headers['content-type'].startswith('text/csv')
    lines = csv_r.text.strip().splitlines()
    assert lines[0].startswith('t,ref_az,ref_el,az,el,e_az,e_el,e_pt') and len(lines) == 9
    assert lines[1].split(',')[0] == '0.0' and lines[1].split(',')[3] == '10.0'


def test_sse(client, behavior):
    rid = post_run(client)
    with client.stream('GET', f'/api/runs/{rid}/stream') as r:
        assert r.status_code == 200
        assert r.headers['content-type'].startswith('text/event-stream')
        body = ''.join(r.iter_text())
    ev = parse_sse(body)
    kinds = [k for k, _ in ev]
    assert kinds[0] == 'status' and ev[0][1]['status'] in ('queued', 'running')
    assert kinds[-1] == 'done' and kinds[-2] == 'status' and ev[-2][1]['status'] == 'done'
    assert ev[-1][1]['metrics']['verdict'] == 'CUMPLE'
    samples = [d for k, d in ev if k == 'sample']
    assert [s['t'] for s in samples] == [i * 0.1 for i in range(8)]
    lines = [d['line'] for k, d in ev if k == 'log']
    assert 'muestra 7' in lines
    # una corrida ya terminada se repite entera
    with client.stream('GET', f'/api/runs/{rid}/stream') as r:
        ev2 = parse_sse(''.join(r.iter_text()))
    assert [k for k, _ in ev2].count('sample') == 8 and ev2[-1][0] == 'done'


@pytest.mark.parametrize('behavior', [{'mode': 'fail', 'delay': '0.05'}])
def test_error_si_no_hay_resultado(client):
    r = wait_status(client, post_run(client))
    assert r['status'] == 'error' and r['exit_code'] == 3
    assert 'sin producir' in r['error']


@pytest.mark.parametrize('behavior', [{'mode': 'nometrics', 'delay': '0.01'}])
def test_error_sin_metricas(client):
    assert wait_status(client, post_run(client))['status'] == 'error'


@pytest.mark.parametrize('behavior', [{'mode': 'sleep', 'delay': '0.05'}])
def test_cancelar_mata_el_grupo(client):
    rid = post_run(client)
    wait_status(client, rid, ('running',))
    # esperar al pid que imprime el proceso falso
    pid = None
    for _ in range(50):
        m = re.search(r'pid (\d+)', client.get(f'/api/runs/{rid}/log').text)
        if m:
            pid = int(m.group(1))
            break
        time.sleep(0.1)
    assert pid
    assert client.delete(f'/api/runs/{rid}').status_code == 409      # activa
    r = client.post(f'/api/runs/{rid}/cancel')
    assert r.status_code == 200 and r.json()['status'] == 'cancelled'
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
    assert wait_status(client, rid)['status'] == 'cancelled'
    assert client.post(f'/api/runs/{rid}/cancel').status_code == 409  # ya termino
    assert client.get('/api/health').json()['active_runs'] == 0


@pytest.mark.parametrize('behavior', [{'mode': 'sleep', 'delay': '0.05'}])
def test_timeout_duro(client, monkeypatch):
    client.app.state.settings.default_timeout = 1
    client.app.state.settings.kill_grace_s = 3
    rid = post_run(client)
    r = wait_status(client, rid, timeout=20)
    assert r['status'] == 'timeout' and 'limite' in r['error']
    assert r['timeout_s'] == 1


@pytest.mark.parametrize('behavior', [{'mode': 'sleep', 'delay': '0.05'}])
def test_maximo_dos_simultaneas_y_dominios_distintos(client):
    ids = [post_run(client) for _ in range(3)]
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15:
        estados = [client.get(f'/api/runs/{i}').json() for i in ids]
        if sum(e['status'] == 'running' for e in estados) == 2:
            break
        time.sleep(0.1)
    assert sorted(e['status'] for e in estados) == ['queued', 'running', 'running']
    dominios = [e['ros_domain_id'] for e in estados if e['status'] == 'running']
    assert len(set(dominios)) == 2 and all(100 <= d <= 199 for d in dominios)
    assert client.get('/api/health').json()['active_runs'] == 3
    # cancelar la que esta en cola no la ejecuta nunca
    cola = next(e['run_id'] for e in estados if e['status'] == 'queued')
    assert client.post(f'/api/runs/{cola}/cancel').json()['status'] == 'cancelled'
    for e in estados:
        client.post(f'/api/runs/{e["run_id"]}/cancel')
    for i in ids:
        assert wait_status(client, i)['status'] == 'cancelled'
    assert client.get(f'/api/runs/{cola}').json()['started_at'] is None


def test_borrar(client):
    rid = post_run(client)
    wait_status(client, rid)
    d = client.settings.runs_dir / rid
    assert d.is_dir()
    assert client.delete(f'/api/runs/{rid}').status_code == 200
    assert not d.exists()
    assert client.get(f'/api/runs/{rid}').status_code == 404


def test_reinicio_marca_interrumpidas(client):
    db = client.app.state.db
    db.create_run('20200101-000000-abcdef', 'n1-rampa', 'x', {}, 'atp_loop.launch.py', 60)
    db.update_run('20200101-000000-abcdef', status='running')
    db.mark_interrupted()
    r = db.get_run('20200101-000000-abcdef')
    assert r['status'] == 'error' and 'reinicio' in r['error']


def test_frontend_estatico(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from station.app import create_app
    from station.config import Settings
    dist = tmp_path / 'dist'
    dist.mkdir()
    (dist / 'index.html').write_text('<h1>estacion</h1>')
    monkeypatch.setenv('STATION_FRONTEND_DIST', str(dist))
    monkeypatch.setenv('STATION_RUNS_DIR', str(tmp_path / 'runs'))
    monkeypatch.setenv('STATION_DB_PATH', str(tmp_path / 'db' / 's.db'))
    with TestClient(create_app(Settings())) as c:
        assert '<h1>estacion</h1>' in c.get('/').text
        assert c.get('/api/health').json()['ok'] is True


# ------------------------------------------------------------- kinds plan y adcs

def test_catalogo_trae_plan_y_adcs(client):
    por_id = {e['id']: e for e in client.get('/api/experiments').json()}
    assert por_id['s6-plan-24h']['kind'] == 'plan' and por_id['s6-plan-24h']['launch'] == 'planificar.py'
    assert por_id['e1-adcs-funcube']['kind'] == 'adcs'
    assert por_id['s6-plan-24h']['params_schema']['sin']['labels'] == {'': 'ninguna'}


def test_corrida_de_plan(client):
    rid = post_run(client, 's6-plan-24h', {'horas': 6, 'sin': ''})
    r = wait_status(client, rid)
    assert r['status'] == 'done', r
    assert r['kind'] == 'plan' and r['campaign'] is None
    # metrics derivadas de plan.json: el bloque del optimo + la brecha del codicioso
    m = r['metrics']
    assert m['kind'] == 'plan' and m['plan'] == 'optimo' and m['optimo_probado'] is True
    assert (m['contactos'], m['gb'], m['brecha_codicioso_pct']) == (3, 4.5, 33.3)
    assert m['codicioso']['gb'] == 3.0 and m['ventanas'] == 6 and m['estaciones'] == ['cdmx', 'tromso']
    assert m['aporte_estacion']['tromso']['delta_gb'] == 1.5
    # plan.json completo en el detalle; metrics.json escrito en la carpeta de la corrida
    assert len(r['plan']['ventanas']) == 6 and r['plan']['planes']['optimo']['ids'] == [0, 1, 2]
    d = client.settings.runs_dir / rid
    assert {'params.json', 'launch.log', 'plan.json', 'metrics.json'} <= {p.name for p in d.iterdir()}
    # el resumen de la lista trae metrics pero no el plan (que pesa ~100 kB)
    fila = next(x for x in client.get('/api/runs').json() if x['run_id'] == rid)
    assert fila['metrics']['kind'] == 'plan' and 'plan' not in fila
    # sin telemetria: lista vacia, no error
    assert client.get(f'/api/runs/{rid}/telemetry').json() == []
    # el comando real: script de la raiz, `--k=v`, id y carpeta de la corrida, sin ros2
    cmd = client.comandos[-1]
    assert cmd[1].endswith('scripts/planificar.py') and 'ros2' not in cmd
    assert '--horas=6' in cmd and '--banda=X' in cmd and f'--id={rid}' in cmd
    assert f'--salida={client.settings.runs_dir}' in cmd
    assert not any(a.startswith('--sin') for a in cmd)          # '' = sin estacion que quitar
    assert '--sats=ISS,NOAA 15,NOAA 18,NOAA 19,NOAA 20,NOAA 21,METEOR-M2 3,METEOR-M2 4' in cmd
    assert '--inicio=2026-10-02T12:00:00' in cmd and '--margen-acm=1.0' in cmd


@pytest.mark.parametrize('behavior', [{'mode': 'fail', 'delay': '0.05'}])
def test_plan_que_falla_es_error(client):
    r = wait_status(client, post_run(client, 's6-plan-24h'))
    assert r['status'] == 'error' and 'planificar.py termino' in r['error']
    assert r['plan'] is None and r['metrics'] is None


@pytest.mark.parametrize('behavior', [{'mode': 'nometrics', 'delay': '0.01'}])
def test_plan_sin_plan_json_es_error(client):
    assert wait_status(client, post_run(client, 's6-plan-24h'))['status'] == 'error'


def test_corrida_adcs(client):
    rid = post_run(client, 'e1-adcs-funcube', {'w0_deg_s': 3, 'lead_s': 3000,
                                               'start_utc': '2026-10-02T21:20:00'})
    r = wait_status(client, rid)
    assert r['status'] == 'done' and r['kind'] == 'adcs' and r['plan'] is None
    assert r['args']['w0_deg_s'] == 3 and r['args']['model_degree'] == 4        # usuario + defaults
    assert r['metrics']['verdict'] == 'CUMPLE'
    cmd = client.comandos[-1]
    assert cmd[:4] == ['ros2', 'launch', 'sat_adcs', 'adcs.launch.py']          # paquete sat_adcs
    assert 'w0_deg_s:=3' in cmd and 'time_scale:=0' in cmd and f'record_dir:={client.settings.runs_dir / rid}' in cmd
    assert len(client.get(f'/api/runs/{rid}/telemetry').json()) == 8


@pytest.mark.parametrize('exp,args', [
    ('s6-plan-24h', {'sats': 'ISS'}),                  # clave fija: no editable
    ('s6-plan-24h', {'horas': 0}),
    ('s6-plan-24h', {'horas': 100}),
    ('s6-plan-24h', {'sin': 'atlantida'}),
    ('s6-plan-24h', {'keyhole': 'ignorar; rm -rf /'}),
    ('s6-plan-24h', {'inicio': '2026-01-01T00:00:00'}),   # la fecha es fija
    ('e1-adcs-funcube', {'seed': 1.5}),
    ('e1-adcs-funcube', {'model_degree': 14}),
    ('e1-adcs-funcube', {'start_utc': '2026-10-03T00:00:00'}),
    ('e1-adcs-funcube', {'record_dir': '/tmp'}),
    ('e1-adcs-funcube', {'time_scale': 1}),            # fijo en 0 (lockstep a maxima velocidad)
])
def test_post_invalido_de_los_kinds_nuevos(client, exp, args):
    assert client.post('/api/runs', json={'experiment_id': exp, 'args': args}).status_code == 422
    assert client.get('/api/runs').json() == []


# ------------------------------------------------------------- kind mision

def test_catalogo_trae_la_mision(client):
    e = {e['id']: e for e in client.get('/api/experiments').json()}['m1-dia-operaciones']
    assert (e['kind'], e['launch']) == ('mision', 'mision.py')
    assert e['args']['inicio'] == '2026-10-02T12:00:00' and 'inicio' not in e['params_schema']


def test_corrida_de_mision(client):
    rid = post_run(client, 'm1-dia-operaciones', {'horas': 3, 'n_od': 2, 'sdr': 'ninguno'})
    r = wait_status(client, rid)
    assert r['status'] == 'done', r
    assert r['kind'] == 'mision' and r['plan'] is None and r['campaign'] is None
    assert r['metrics']['kind'] == 'mision' and r['metrics']['gb'] == 1.5
    assert r['mision']['totales']['contactos_ejecutados'] == 1
    cmd = client.comandos[-1]
    assert cmd[1].endswith('scripts/mision.py') and 'ros2' not in cmd
    assert '--horas=3' in cmd and '--n-od=2' in cmd and '--sdr=ninguno' in cmd
    assert f'--id={rid}' in cmd and f'--salida={client.settings.runs_dir}' in cmd
    # cada contacto es una corrida dentro de la mision: telemetria, metricas y log
    assert client.get(f'/api/runs/{rid}/contactos/c001').json()['metrics'] == {'n': 5}
    assert len(client.get(f'/api/runs/{rid}/contactos/c001/telemetry?every=2').json()) == 3
    assert 'ENGANCHE' in client.get(f'/api/runs/{rid}/contactos/c001/log').text
    # ids de contacto estrictos y solo para misiones
    for malo in ('c1', 'c001_x', 'c999', '%2e%2e', 'c001%2f..'):
        assert client.get(f'/api/runs/{rid}/contactos/{malo}').status_code == 404
    otro = wait_status(client, post_run(client, 's6-plan-24h', {'horas': 6, 'sin': ''}))['run_id']
    assert client.get(f'/api/runs/{otro}/contactos/c001').status_code == 404


@pytest.mark.parametrize('behavior', [{'mode': 'fail', 'delay': '0.05'}])
def test_mision_que_falla_es_error(client):
    r = wait_status(client, post_run(client, 'm1-dia-operaciones'))
    assert r['status'] == 'error' and 'mision.py termino' in r['error'] and r['mision'] is None


@pytest.mark.parametrize('exp,args', [
    ('m1-dia-operaciones', {'sats': 'ISS'}),                   # clave fija
    ('m1-dia-operaciones', {'inicio': '2026-01-01T00:00:00'}),  # la fecha es fija
    ('m1-dia-operaciones', {'horas': 0}),
    ('m1-dia-operaciones', {'horas': 100}),
    ('m1-dia-operaciones', {'concurrencia': 50}),
    ('m1-dia-operaciones', {'sdr': 'ignorar; rm -rf /'}),
])
def test_mision_args_invalidos(client, exp, args):
    assert client.post('/api/runs', json={'experiment_id': exp, 'args': args}).status_code == 422
