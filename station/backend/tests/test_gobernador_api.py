"""Las corridas pasan por el gobernador: espera por carga, reintento tras senal externa,
cancelacion en cada fase y GET /api/sistema. Lanzador falso: no se ejecuta ROS."""
import time

import pytest

from conftest import parse_sse, wait_status, wait_until

FINALES = ('done', 'error', 'cancelled', 'timeout', 'interrumpida')


def lanzar(client, exp='n4a-pase-oblicuo-s', args=None):
    r = client.post('/api/runs', json={'experiment_id': exp, 'args': args})
    assert r.status_code == 201, r.text
    return r.json()['run_id']


def get(client, rid):
    return client.get(f'/api/runs/{rid}').json()


def wait_final(client, rid, timeout=30.0):
    """Estado final: `interrumpida` solo cuenta con finished_at (antes espera el reintento)."""
    return wait_until(lambda: (lambda r: r if r['status'] in FINALES and (
        r['status'] != 'interrumpida' or r['finished_at']) else None)(get(client, rid)), timeout)


# ------------------------------------------------------------------ carga
def test_corrida_espera_por_la_carga_y_arranca_al_bajar(client):
    client.carga.valor = 9.0                              # el umbral de `pesado` es 6
    rid = lanzar(client)
    r = wait_until(lambda: (lambda x: x if x['status'] == 'esperando_carga' else None)(get(client, rid)))
    assert r['started_at'] is None and r['ros_domain_id'] is None
    assert client.get('/api/health').json()['active_runs'] == 1
    sis = client.get('/api/sistema').json()
    assert sis['colas']['pesado'] == {'en_cola': 0, 'esperando_carga': 1, 'corriendo': 0}
    assert sis['cupos']['pesado']['admite_ahora'] is False
    time.sleep(0.3)
    assert get(client, rid)['status'] == 'esperando_carga'    # sigue esperando mientras la carga siga alta
    client.carga.valor = 2.0
    r = wait_status(client, rid)
    assert r['status'] == 'done' and r['started_at'] and r['reintentos'] == 0


def test_plan_es_ligero_y_no_espera_con_carga_media(client):
    client.carga.valor = 9.0                              # supera pesado (6) pero no ligero (10)
    r = wait_status(client, lanzar(client, 's6-plan-24h', {'horas': 6, 'sin': ''}))
    assert r['status'] == 'done' and r['clase'] == 'ligero'


def test_clase_por_defecto_de_los_kinds(client):
    ids = {'n4a-pase-oblicuo-s': 'pesado', 'n4c-montecarlo-analitico': 'pesado',
           'e1-adcs-funcube': 'pesado', 'm1-dia-operaciones': 'pesado', 's6-plan-24h': 'ligero'}
    client.carga.valor = 99.0                              # nadie arranca: solo se mira la clase
    for exp, clase in ids.items():
        assert get(client, lanzar(client, exp))['clase'] == clase


def test_cancelar_en_esperando_carga(client):
    client.carga.valor = 9.0
    rid = lanzar(client)
    wait_until(lambda: get(client, rid)['status'] == 'esperando_carga')
    assert client.post(f'/api/runs/{rid}/cancel').json()['status'] == 'cancelled'
    client.carga.valor = 0.0
    time.sleep(0.3)
    r = get(client, rid)
    assert r['status'] == 'cancelled' and r['started_at'] is None       # nunca llego a ejecutarse
    assert client.get('/api/sistema').json()['gobernador']['esperando'] == []


@pytest.mark.parametrize('extra_env', [{'STATION_CUPO_PESADO': '1'}])
@pytest.mark.parametrize('behavior', [{'mode': 'sleep', 'delay': '0.05'}])
def test_cupo_pesado_1_serializa_las_corridas(client):
    a, b = lanzar(client), lanzar(client)
    wait_until(lambda: get(client, a)['status'] == 'running')
    time.sleep(0.3)
    assert get(client, b)['status'] == 'queued'               # cupo lleno, no es la carga: sigue en cola
    sis = client.get('/api/sistema').json()
    assert sis['colas']['pesado'] == {'en_cola': 1, 'esperando_carga': 0, 'corriendo': 1}
    client.post(f'/api/runs/{a}/cancel')
    wait_until(lambda: get(client, b)['status'] == 'running')   # al liberarse, entra la siguiente
    client.post(f'/api/runs/{b}/cancel')
    assert wait_status(client, b)['status'] == 'cancelled'


# --------------------------------------------------------- senal externa y reintentos
@pytest.mark.parametrize('behavior', [{'mode': 'senal_una_vez', 'delay': '0.01'}])
def test_muerte_por_senal_reintenta_y_termina(client):
    rid = lanzar(client)
    r = wait_final(client, rid)
    assert r['status'] == 'done', r
    assert r['reintentos'] == 1 and r['error'] is None and r['metrics']['verdict'] == 'CUMPLE'
    assert len(client.contador.read_text()) == 2             # dos ejecuciones
    log = client.get(f'/api/runs/{rid}/log').text
    assert 'interrumpida por senal externa (codigo -9); reintento 1/2' in log
    assert 'reintento 1: de nuevo a la cola' in log
    # el reintento arranca de cero: la telemetria es de la segunda ejecucion, no se duplica
    assert len(client.get(f'/api/runs/{rid}/telemetry').json()) == 8


@pytest.mark.parametrize('extra_env', [{'STATION_REINTENTO_ESPERA': '1.5'}])
@pytest.mark.parametrize('behavior', [{'mode': 'senal_una_vez', 'delay': '0.01'}])
def test_interrumpida_es_transitoria_mientras_espera_el_reintento(client):
    rid = lanzar(client)
    r = wait_until(lambda: (lambda x: x if x['status'] == 'interrumpida' else None)(get(client, rid)))
    assert r['finished_at'] is None and r['reintentos'] == 1 and 'reintento 1/2 en 1.5 s' in r['error']
    assert client.get('/api/health').json()['active_runs'] == 1          # sigue activa
    assert client.delete(f'/api/runs/{rid}').status_code == 409            # no se borra a medias
    assert wait_final(client, rid)['status'] == 'done'


@pytest.mark.parametrize('behavior', [{'mode': 'senal', 'delay': '0.01'}])
def test_senal_en_cada_intento_agota_los_reintentos(client):
    rid = lanzar(client)
    r = wait_final(client, rid)
    assert r['status'] == 'interrumpida' and r['finished_at'] and r['reintentos'] == 2
    assert 'agoto los 2 reintentos' in r['error']
    assert len(client.contador.read_text()) == 3             # 1 intento + 2 reintentos
    # es final: el SSE cierra con `done`, se puede borrar
    ev = parse_sse(client.get(f'/api/runs/{rid}/stream').text)
    assert ev[-1][0] == 'done' and ev[-2] == ('status', {'status': 'interrumpida'})
    assert client.delete(f'/api/runs/{rid}').status_code == 200


@pytest.mark.parametrize('extra_env', [{'STATION_REINTENTOS': '0'}])
@pytest.mark.parametrize('behavior', [{'mode': 'senal', 'delay': '0.01'}])
def test_sin_reintentos_la_senal_es_interrumpida_directa(client):
    r = wait_final(client, lanzar(client))
    assert r['status'] == 'interrumpida' and r['reintentos'] == 0 and len(client.contador.read_text()) == 1


@pytest.mark.parametrize('behavior', [{'mode': 'fail', 'delay': '0.01'}])
def test_un_error_propio_no_se_reintenta(client):
    r = wait_status(client, lanzar(client))
    assert r['status'] == 'error' and r['exit_code'] == 3 and r['reintentos'] == 0
    assert client.get(f'/api/runs/{r["run_id"]}/log').text.count('algo salio mal') == 1


@pytest.mark.parametrize('extra_env', [{'STATION_REINTENTO_ESPERA': '30'}])
@pytest.mark.parametrize('behavior', [{'mode': 'senal', 'delay': '0.01'}])
def test_cancelar_durante_la_espera_del_reintento(client):
    rid = lanzar(client)
    wait_until(lambda: get(client, rid)['status'] == 'interrumpida')
    t0 = time.monotonic()
    assert client.post(f'/api/runs/{rid}/cancel').json()['status'] == 'cancelled'
    assert time.monotonic() - t0 < 5                           # no espera los 30 s
    r = get(client, rid)
    assert r['status'] == 'cancelled' and r['finished_at']
    assert len(client.contador.read_text()) == 1               # no hubo reintento


def test_espera_creciente_entre_reintentos(client):
    from station.executor import ESPERA_MAX_S
    s = client.settings
    esperas = [min(s.reintento_espera_s * 2 ** (i - 1), ESPERA_MAX_S) for i in (1, 2, 3)]
    assert esperas == [0.05, 0.1, 0.2]                         # 30, 60, 120 s con la espera por defecto


# ------------------------------------------------------------------ /api/sistema
def test_sistema(client):
    client.journal['v'] = {'lineas': [{'t': '2026-10-05T12:00:00-0600', 'texto': 'SOBRECARGA load=13'}],
                           'ultima_sobrecarga': {'t': '2026-10-05T12:00:00-0600', 'texto': 'SOBRECARGA load=13'}}
    sis = client.get('/api/sistema').json()
    assert set(sis) >= {'carga', 'memoria', 'gobernador', 'colas', 'cupos', 'pantallas', 'activas', 'vigilante'}
    assert set(sis['carga']) >= {'l1', 'l5', 'l15', 'hilos'} and sis['carga']['hilos'] >= 1
    assert sis['memoria']['total_mb'] > 0 and 0 <= sis['memoria']['usada_pct'] <= 100
    assert set(sis['colas']) == {'ligero', 'pesado', 'gazebo', 'manim'}
    assert sis['cupos']['gazebo']['umbral_carga'] == 4.0 and sis['cupos']['ligero']['cupo'] == 2
    assert sis['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    assert sis['activas'] == {'corridas': 0, 'renders': 0}
    assert sis['vigilante']['ultima_sobrecarga']['texto'].startswith('SOBRECARGA')


def test_sistema_sin_diario_es_null(client):
    assert client.get('/api/sistema').json()['vigilante'] is None


def test_sistema_con_una_corrida_en_marcha(client):
    client.carga.valor = 1.0
    rid = lanzar(client, 'n1-rampa')
    sis = wait_until(lambda: (lambda s: s if s['gobernador']['corriendo'] else None)(client.get('/api/sistema').json()))
    assert sis['gobernador']['corriendo'][0]['tarea'] == f'corrida:{rid}'
    assert sis['colas']['pesado']['corriendo'] == 1 and sis['activas']['corridas'] == 1
    wait_status(client, rid)
    assert client.get('/api/sistema').json()['gobernador']['corriendo'] == []
