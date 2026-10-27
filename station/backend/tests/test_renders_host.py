"""Renders `ejecutor: host` (docs/APP.md 4.1) con un trabajador simulado: reclamar, latidos, fin, latido
perdido y reintento, cancelacion y admision por el gobernador. Ningun trabajo lo lanza el backend."""
import copy
import hashlib
import json
import time
from pathlib import Path

import pytest

from conftest import parse_sse, wait_status, wait_until
from station import renders

VIDEO = b'video-del-host' * 50


def run_hecho(c):
    rid = c.post('/api/runs', json={'experiment_id': 'prueba-proceso'}).json()['run_id']
    assert wait_status(c, rid)['status'] == 'done'
    return rid


def encolar(c, rid, render_id='render-host', args=None):
    r = c.post(f'/api/runs/{rid}/renders', json={'render_id': render_id, 'args': args})
    assert r.status_code == 201, r.text
    return r.json()['job_id']


def reclamar(c, clases=None, trabajador='host-1'):
    return c.post('/api/renders/reclamar', json={'trabajador': trabajador, 'clases': clases})


def job(c, jid):
    return c.get(f'/api/renders/{jid}').json()


def latido(c, jid, **cuerpo):
    r = c.post(f'/api/renders/{jid}/latido', json=cuerpo)
    assert r.status_code == 200, r.text
    return r.json()


def fin(c, jid, rc=0, senal=None):
    r = c.post(f'/api/renders/{jid}/fin', json={'exit_code': rc, 'senal': senal})
    assert r.status_code == 200, r.text
    return r.json()


def escribir_media(c, trabajo, miniatura=True, sha=None):
    """Lo que dejaria el script del render en `salida` (como el trabajador lo ve: relativa a la raiz)."""
    d = Path(trabajo['salida'])
    d = d if d.is_absolute() else c.settings.root / d
    (d / 'visor.mp4').write_bytes(VIDEO)
    archivos = [{'ruta': 'visor.mp4', 'tipo': 'video', 'ancho': 1920, 'alto': 1080, 'duracion_s': 10.0,
                 'sha256': sha or hashlib.sha256(VIDEO).hexdigest(), 'principal': True}]
    if miniatura:
        (d / 'miniatura.jpg').write_bytes(b'\xff\xd8\xff\xe0' + b'\0' * 100)
        archivos.append({'ruta': 'miniatura.jpg', 'tipo': 'imagen', 'ancho': 640, 'alto': 360})
    (d / 'media.json').write_text(json.dumps({
        'version': 1, 'job_id': trabajo['job_id'], 'render': trabajo['render'], 'run_id': trabajo['run_id'],
        'creado_utc': '2026-10-06T12:00:00Z', 'archivos': archivos,
        'procedencia': {'commit': 'abc', 'sucio': False, 'comando': trabajo['comando'], 'args': {},
                        'corrida': {'run_id': trabajo['run_id']}}}))
    return d


@pytest.fixture
def host(client, tmp_path):
    """Cliente con la raiz en tmp_path: las rutas del trabajador salen relativas, como en el despliegue."""
    client.settings.root = tmp_path
    client.rid = run_hecho(client)
    return client


# ------------------------------------------------------------------ parser
def test_parser_ejecutor_por_defecto_y_invalido():
    base = dict(id='r-ok', title='T', requiere={'cualquiera': True}, comando=['x'], recursos='ligero')
    d = copy.deepcopy(base)
    renders._check_render(d, 'r-ok')
    assert d['ejecutor'] == 'backend'
    d = dict(copy.deepcopy(base), ejecutor='host')
    renders._check_render(d, 'r-ok')
    assert renders.render_spec(d)['ejecutor'] == 'host'
    for malo in ('nube', 'HOST', '', None, True):
        with pytest.raises(ValueError, match='ejecutor'):
            renders._check_render(dict(copy.deepcopy(base), ejecutor=malo), 'r-ok')


# ------------------------------------------------------------------ reclamar
def test_el_backend_no_lanza_los_trabajos_del_host(host):
    jid = encolar(host, host.rid)
    time.sleep(0.4)
    j = job(host, jid)
    assert j['estado'] == 'en_cola' and j['ejecutor'] == 'host' and j['started_at'] is None
    assert not (host.settings.runs_dir / host.rid / 'media' / jid / 'render.log').exists()
    assert host.get('/api/sistema').json()['gobernador']['corriendo'] == []
    assert job(host, encolar(host, host.rid, 'render-falso'))['ejecutor'] == 'backend'


def test_reclamar_sin_nada_pendiente_es_204(host):
    r = reclamar(host)
    assert r.status_code == 204 and r.content == b''


def test_reclamar_devuelve_el_trabajo_resuelto(host):
    jid = encolar(host, host.rid, args={'acelerar': 60})
    r = reclamar(host)
    assert r.status_code == 200
    t = r.json()
    assert t['job_id'] == jid and t['render'] == 'render-host' and t['run_id'] == host.rid and t['clase'] == 'gazebo'
    assert t['pantalla'] == ':80' and t['cwd'] == '.'
    assert t['salida'] == f'runs/{host.rid}/media/{jid}' and t['run_dir'] == f'runs/{host.rid}'
    # comando resuelto con rutas RELATIVAS a la raiz del repo (la misma en el host y en el contenedor)
    assert t['comando'] == ['python3', './station/backend/tests/fixtures/render_falso.py', f'runs/{host.rid}',
                            t['salida'], ':80', jid, '--modo', 'ok', '--acelerar', '60']
    env = t['entorno']
    assert env['DISPLAY'] == ':80' and env['LP_NUM_THREADS'] == '3' and env['RENDER_JOB_ID'] == jid
    assert env['RENDER_ID'] == 'render-host' and 100 <= int(env['ROS_DOMAIN_ID']) <= 199
    assert t['timeout_s'] > 0 and t['latido_cada_s'] > 0
    j = job(host, jid)
    assert j['estado'] == 'corriendo' and j['trabajador'] == 'host-1' and j['pantalla'] == ':80' and j['started_at']
    # el gobernador contabiliza el turno y la pantalla
    sis = host.get('/api/sistema').json()
    assert sis['gobernador']['corriendo'][0]['tarea'] == f'render:{jid}'
    assert sis['colas']['gazebo']['corriendo'] == 1 and sis['pantallas']['libres'] == [':81', ':82', ':83']
    assert host.get('/api/health').json()['active_renders'] == 1
    # un segundo reclamo: el cupo de gazebo (1) esta lleno
    encolar(host, host.rid)
    assert reclamar(host).status_code == 204


def test_reclamar_respeta_la_antiguedad_y_el_filtro_de_clases(host):
    a = encolar(host, host.rid)                                  # gazebo
    b = encolar(host, host.rid, 'render-host-corto')             # ligero
    assert reclamar(host, ['manim']).status_code == 204
    assert reclamar(host, ['ligero']).json()['job_id'] == b      # filtra por clase
    assert reclamar(host, ['gazebo', 'ligero']).json()['job_id'] == a
    assert reclamar(host).status_code == 204


def test_reclamar_valida_la_entrada(host):
    for cuerpo in ({}, {'trabajador': ''}, {'trabajador': 'a b'}, {'trabajador': '../x'}, {'trabajador': 'x' * 65},
                   {'trabajador': 'h', 'clases': []}, {'trabajador': 'h', 'clases': ['enorme']},
                   {'trabajador': 'h', 'clases': 'gazebo'}):
        assert host.post('/api/renders/reclamar', json=cuerpo).status_code == 422, cuerpo


def test_ningun_trabajo_cuando_la_carga_no_lo_admite(host):
    jid = encolar(host, host.rid)                                # gazebo: umbral 4
    host.carga.valor = 5.0
    assert reclamar(host).status_code == 204
    assert job(host, jid)['estado'] == 'esperando_carga' and job(host, jid)['started_at'] is None
    assert host.get('/api/sistema').json()['colas']['gazebo']['esperando_carga'] == 0   # el gobernador no lo retiene: lo reclama el host
    host.carga.valor = 9.0
    lig = encolar(host, host.rid, 'render-host-corto')           # ligero: umbral 10
    assert reclamar(host, ['ligero']).json()['job_id'] == lig    # otra clase si pasa
    host.carga.valor = 3.9
    assert reclamar(host).json()['job_id'] == jid                # al bajar la carga, se entrega
    assert job(host, jid)['estado'] == 'corriendo'


# ------------------------------------------------------------------ latidos
def test_latidos_escriben_log_y_progreso(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    r = latido(host, jid, progreso=0.25, log=['hola', 'segunda linea'])
    assert r == {'cancelar': False, 'estado': 'corriendo'}
    latido(host, jid, progreso=0.5)
    latido(host, jid)                                            # un latido vacio solo mantiene vivo el trabajo
    log = (host.settings.runs_dir / host.rid / 'media' / jid / 'render.log').read_text()
    assert log == 'hola\nsegunda linea\nPROGRESO 0.250\nPROGRESO 0.500\n'
    assert job(host, jid)['progreso'] == 0.5
    assert host.get(f'/api/renders?run_id={host.rid}').json()[0]['progreso'] == 0.5


def test_latido_con_datos_invalidos(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    for cuerpo in ({'progreso': 1.5}, {'progreso': -0.1}, {'progreso': 'x'}, {'log': 'texto'},
                   {'log': ['x'] * 2001}, {'log': ['x' * 8001]}):
        assert host.post(f'/api/renders/{jid}/latido', json=cuerpo).status_code == 422, cuerpo
    r = latido(host, jid, log=['a\nb\r', 'c'])
    assert r['cancelar'] is False
    assert (host.settings.runs_dir / host.rid / 'media' / jid / 'render.log').read_text() == 'a b\nc\n'


def test_sse_de_un_trabajo_del_host(host):
    jid = encolar(host, host.rid)
    t = reclamar(host).json()
    latido(host, jid, progreso=0.4, log=['trabajando'])
    escribir_media(host, t)
    fin(host, jid, 0)
    ev = parse_sse(host.get(f'/api/renders/{jid}/stream').text)
    assert [e for e, _ in ev][0] == 'estado' and ev[-1][0] == 'hecho'
    assert [d['progreso'] for e, d in ev if e == 'progreso'] == [0.4]
    assert any(d['line'] == 'trabajando' for e, d in ev if e == 'log')


def test_latidos_de_trabajos_ajenos_o_inexistentes(host):
    assert host.post('/api/renders/rmalo/latido', json={}).status_code == 404
    assert host.post('/api/renders/r20200101-000000-abcdef/latido', json={}).status_code == 404
    assert host.post('/api/renders/r20200101-000000-abcdef/fin', json={'exit_code': 0}).status_code == 404
    propio = encolar(host, host.rid, 'render-falso')               # ejecutor backend
    assert host.post(f'/api/renders/{propio}/latido', json={}).status_code == 409
    assert host.post(f'/api/renders/{propio}/fin', json={'exit_code': 0}).status_code == 409
    assert host.post(f'/api/renders/{encolar(host, host.rid)}/fin', json={}).status_code == 422


def test_latido_de_un_trabajo_no_reclamado_dice_cancelar(host):
    jid = encolar(host, host.rid)                                  # sigue en cola: nadie lo reclamo
    assert latido(host, jid)['cancelar'] is True


# ------------------------------------------------------------------ fin
def test_fin_correcto_valida_media_y_libera_el_turno(host):
    jid = encolar(host, host.rid, args={'acelerar': 45})
    t = reclamar(host).json()
    latido(host, jid, progreso=0.9)
    escribir_media(host, t)
    r = fin(host, jid, 0)
    assert r == {'job_id': jid, 'estado': 'hecho', 'error': None, 'reintentos': 0}
    j = job(host, jid)
    assert j['estado'] == 'hecho' and j['exit_code'] == 0 and j['finished_at'] and j['progreso'] == 1.0
    assert j['media']['job_id'] == jid and j['miniatura'] == f'/api/medios/{jid}/miniatura.jpg'
    # el medio entra a la galeria y se sirve con Range
    g = host.get('/api/medios').json()
    assert [m['job_id'] for m in g] == [jid] and g[0]['ancho'] == 1920
    r = host.get(f'/api/medios/{jid}/visor.mp4', headers={'Range': 'bytes=0-9'})
    assert r.status_code == 206 and r.content == VIDEO[:10]
    # el turno, la pantalla y el dominio se devolvieron
    sis = host.get('/api/sistema').json()
    assert sis['gobernador']['corriendo'] == [] and sis['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    assert sis['gobernador']['dominios_ros_en_uso'] == []
    assert host.get('/api/health').json()['active_renders'] == 0


@pytest.mark.parametrize('prepara,fragmento', [
    (lambda c, t: None, 'sin escribir media.json'),
    (lambda c, t: escribir_media(c, t, miniatura=False), 'miniatura'),
    (lambda c, t: escribir_media(c, t, sha='0' * 64), 'sha256 no coincide'),
])
def test_fin_sin_media_json_valido_es_error(host, prepara, fragmento):
    jid = encolar(host, host.rid)
    t = reclamar(host).json()
    prepara(host, t)
    r = fin(host, jid, 0)
    assert r['estado'] == 'error' and fragmento in r['error']
    j = job(host, jid)
    assert j['media'] is None and j['finished_at'] and host.get('/api/medios').json() == []
    assert host.get('/api/sistema').json()['gobernador']['corriendo'] == []      # libero el turno igual


def test_fin_con_error_propio_no_se_reintenta(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    r = fin(host, jid, 4)
    assert r['estado'] == 'error' and 'codigo 4' in r['error'] and r['reintentos'] == 0
    time.sleep(0.3)
    assert job(host, jid)['estado'] == 'error' and reclamar(host).status_code == 204


def test_fin_por_senal_externa_reintenta_y_se_vuelve_a_reclamar(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    (host.settings.runs_dir / host.rid / 'media' / jid / 'basura.tmp').write_text('parcial')
    r = fin(host, jid, -9, senal='SIGKILL')
    assert r['estado'] == 'interrumpida' and r['reintentos'] == 1
    # tras la espera vuelve a la cola, limpio, y el mismo trabajador (u otro) lo reclama de nuevo
    t = wait_until(lambda: (lambda x: x if x.status_code == 200 else None)(reclamar(host, trabajador='host-2')))
    assert t.json()['job_id'] == jid and job(host, jid)['trabajador'] == 'host-2'
    d = host.settings.runs_dir / host.rid / 'media' / jid
    assert not (d / 'basura.tmp').exists()
    log = (d / 'render.log').read_text()
    assert 'interrumpido (senal externa, codigo -9); reintento 1/2' in log and 'reintento 1: de nuevo a la cola' in log
    escribir_media(host, t.json())
    assert fin(host, jid, 0)['estado'] == 'hecho' and job(host, jid)['reintentos'] == 1


@pytest.mark.parametrize('rc,senal', [(-9, None), (137, None), (1, True)])
def test_que_cuenta_como_senal_externa(host, rc, senal):
    jid = encolar(host, host.rid)
    reclamar(host)
    assert fin(host, jid, rc, senal)['estado'] == 'interrumpida'


def test_senal_en_cada_intento_agota_los_reintentos(host):
    jid = encolar(host, host.rid)
    for n in range(3):
        t = wait_until(lambda: (lambda x: x if x.status_code == 200 else None)(reclamar(host)))
        assert t.json()['job_id'] == jid
        r = fin(host, jid, -9)
    assert r['estado'] == 'interrumpida' and r['reintentos'] == 2 and 'agoto los 2 reintentos' in r['error']
    j = job(host, jid)
    assert j['finished_at'] and reclamar(host).status_code == 204
    assert host.delete(f'/api/renders/{jid}').status_code == 200            # es final


def test_senal_con_resultado_ya_escrito_es_hecho(host):
    """Como en las corridas: si el resultado ya existe, una senal tardia no lo invalida (rc 0 solo)."""
    jid = encolar(host, host.rid)
    t = reclamar(host).json()
    escribir_media(host, t)
    assert fin(host, jid, 0, senal=True)['estado'] == 'hecho'


# ---------------------------------------------------------- latido perdido
@pytest.mark.parametrize('extra_env', [{'STATION_LATIDO_TIMEOUT': '0.4'}])
def test_latido_perdido_interrumpe_y_reintenta(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    # el trabajador late un rato y luego muere sin avisar
    for _ in range(3):
        assert latido(host, jid, log=['vivo'])['cancelar'] is False
        time.sleep(0.15)
    j = wait_until(lambda: (lambda x: x if x['estado'] != 'corriendo' else None)(job(host, jid)))
    assert j['estado'] in ('interrumpida', 'en_cola') and j['reintentos'] == 1
    assert 'sin latido en 0.4 s' in (host.settings.runs_dir / host.rid / 'media' / jid / 'render.log').read_text()
    assert host.get('/api/sistema').json()['gobernador']['corriendo'] == []     # el turno se libero
    # si el trabajador zombi vuelve a latir, se le dice que cancele; su `fin` tardio no cambia nada
    assert latido(host, jid)['cancelar'] is True
    wait_until(lambda: job(host, jid)['estado'] == 'en_cola')
    assert fin(host, jid, 0)['estado'] == 'en_cola' and job(host, jid)['reintentos'] == 1
    # y otro trabajador lo toma
    assert reclamar(host, trabajador='host-2').json()['job_id'] == jid


@pytest.mark.parametrize('extra_env', [{'STATION_LATIDO_TIMEOUT': '0.3'}])
def test_latido_perdido_en_todos_los_intentos_deja_interrumpida(host):
    jid = encolar(host, host.rid)
    for _ in range(3):
        t = wait_until(lambda: (lambda x: x if x.status_code == 200 else None)(reclamar(host)))
        assert t.json()['job_id'] == jid
        wait_until(lambda: job(host, jid)['estado'] != 'corriendo')
    j = wait_until(lambda: (lambda x: x if x['finished_at'] else None)(job(host, jid)))
    assert j['estado'] == 'interrumpida' and j['reintentos'] == 2 and 'agoto los 2 reintentos' in j['error']
    assert host.get('/api/sistema').json()['pantallas']['libres'] == [':80', ':81', ':82', ':83']


@pytest.mark.parametrize('extra_env', [{'STATION_LATIDO_TIMEOUT': '5'}])
def test_trabajo_que_pasa_de_su_timeout_se_cancela(host):
    jid = encolar(host, host.rid, 'render-host-corto')               # timeout_s: 1
    reclamar(host)
    fin_t = time.monotonic() + 8
    while time.monotonic() < fin_t:
        if latido(host, jid)['cancelar']:
            break
        time.sleep(0.2)
    j = job(host, jid)
    assert j['estado'] == 'error' and 'excedio el limite de 1 s' in j['error'] and j['finished_at']
    assert fin(host, jid, -15)['estado'] == 'error'                  # el fin tardio no cambia nada
    assert host.get('/api/sistema').json()['gobernador']['corriendo'] == []


# ------------------------------------------------------------------ cancelacion
def test_cancelar_un_trabajo_pendiente(host):
    jid = encolar(host, host.rid)
    assert host.post(f'/api/renders/{jid}/cancel').json()['estado'] == 'cancelado'
    assert reclamar(host).status_code == 204
    assert host.post(f'/api/renders/{jid}/cancel').status_code == 409


def test_cancelar_un_trabajo_en_curso_avisa_en_el_siguiente_latido(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    assert latido(host, jid)['cancelar'] is False
    r = host.post(f'/api/renders/{jid}/cancel')
    assert r.json() == {'job_id': jid, 'estado': 'cancelado'}
    j = job(host, jid)
    assert j['estado'] == 'cancelado' and j['finished_at']
    # el turno y la pantalla siguen retenidos mientras el trabajador mata el grupo (no se reasignan a medias)
    sis = host.get('/api/sistema').json()
    assert sis['colas']['gazebo']['corriendo'] == 1 and sis['pantallas']['libres'] == [':81', ':82', ':83']
    encolar(host, host.rid)
    assert reclamar(host).status_code == 204
    assert latido(host, jid) == {'cancelar': True, 'estado': 'cancelado'}
    assert fin(host, jid, -15)['estado'] == 'cancelado'                  # confirma: se libera el turno
    sis = host.get('/api/sistema').json()
    assert sis['colas']['gazebo']['corriendo'] == 0 and sis['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    assert reclamar(host).status_code == 200                              # ahora si entra el siguiente


@pytest.mark.parametrize('extra_env', [{'STATION_LATIDO_TIMEOUT': '0.4'}])
def test_cancelado_cuyo_trabajador_no_confirma_libera_por_timeout(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    host.post(f'/api/renders/{jid}/cancel')
    wait_until(lambda: host.get('/api/sistema').json()['colas']['gazebo']['corriendo'] == 0)
    assert job(host, jid)['estado'] == 'cancelado'


@pytest.mark.parametrize('extra_env', [{'STATION_REINTENTO_ESPERA': '30'}])
def test_cancelar_durante_la_espera_del_reintento(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    assert fin(host, jid, -9)['estado'] == 'interrumpida'
    assert job(host, jid)['finished_at'] is None
    t0 = time.monotonic()
    assert host.post(f'/api/renders/{jid}/cancel').json()['estado'] == 'cancelado'
    assert time.monotonic() - t0 < 3
    assert reclamar(host).status_code == 204 and job(host, jid)['estado'] == 'cancelado'


def test_borrar_la_corrida_con_un_trabajo_del_host_activo_es_409(host):
    encolar(host, host.rid)
    assert host.delete(f'/api/runs/{host.rid}').status_code == 409


def test_un_reinicio_deja_interrumpido_lo_que_tenia_el_host(host):
    jid = encolar(host, host.rid)
    reclamar(host)
    host.app.state.db.mark_interrupted()
    j = job(host, jid)
    assert j['estado'] == 'interrumpida' and j['finished_at'] and 'reinicio' in j['error']
