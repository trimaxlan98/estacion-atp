"""Renders (docs/APP.md 4): catalogo, trabajos, contrato de `media.json`, SSE, cancelacion, reintentos
y galeria. Los scripts son de tests/fixtures (sin Gazebo ni ffmpeg)."""
import copy
import hashlib
import json

import pytest
import yaml

from conftest import parse_sse, wait_job, wait_status, wait_until
from station import renders
from station.renders import MediaError, validar_media

JOB, RUN = 'r20261005-120000-abc123', '20261005-120000-def456'


def run_hecho(c, exp='prueba-proceso', args=None):
    if args is None:
        args = {'dur_s': 4} if exp == 'prueba-proceso' else {}
    rid = c.post('/api/runs', json={'experiment_id': exp, 'args': args}).json()['run_id']
    assert wait_status(c, rid)['status'] == 'done'
    return rid


def render(c, rid, render_id='render-falso', args=None, esperar=True):
    r = c.post(f'/api/runs/{rid}/renders', json={'render_id': render_id, 'args': args})
    assert r.status_code == 201, r.text
    j = r.json()
    assert j['estado'] == 'en_cola' and renders_id_ok(j['job_id'])
    return wait_job(c, j['job_id']) if esperar else j


def renders_id_ok(job_id):
    import re
    return re.match(r'^r\d{8}-\d{6}-[0-9a-f]{6}$', job_id)


# ------------------------------------------------------------------ catalogo
RENDER_BASE = dict(id='r-ok', title='T', requiere={'contrato': 'red'},
                   comando=['bash', 'scripts/render/x.sh', '{run_dir}', '{salida}', '{pantalla}'],
                   estilo_args='cli', recursos='gazebo', pantalla=True, duracion_estimada_s=300,
                   args={'acelerar': 30}, params_schema={'acelerar': {'type': 'number', 'min': 1, 'max': 120}})


def render_con(**cambios):
    d = copy.deepcopy(RENDER_BASE)
    d.update(cambios)
    for k in [k for k, v in d.items() if v is None]:
        d.pop(k)
    return d


def test_render_ejemplo_de_la_app_valida():
    renders._check_render(copy.deepcopy(RENDER_BASE), 'r-ok')


@pytest.mark.parametrize('malo,por_que', [
    (render_con(requiere={'contrato': 'red', 'kind': 'loop'}), 'requiere'),      # una sola clave
    (render_con(requiere={'tonteria': 1}), 'requiere'),
    (render_con(requiere='red'), 'requiere'),
    (render_con(requiere={'kind': 'inventado'}), 'kind'),
    (render_con(requiere={'kind': ['loop', 'inventado']}), 'kind'),
    (render_con(requiere={'kind': []}), 'kind'),
    (render_con(requiere={'archivo': '../x'}), 'ruta'),
    (render_con(requiere={'cualquiera': False}), 'cualquiera'),
    (render_con(requiere={'contrato': 'Mal Contrato'}), 'contrato'),
    (render_con(comando='bash x'), 'comando'),
    (render_con(comando=['bash', '{inventado}']), 'marcador'),
    (render_con(pantalla='si'), 'pantalla'),
    (render_con(recursos='enorme'), 'recursos'),
    (render_con(recursos=None), 'recursos'),
    (render_con(duracion_estimada_s=-5), 'duracion'),
    (render_con(estilo_args='raro'), 'estilo_args'),
    (render_con(args={'Mal-Arg': 1}), 'clave'),
    (render_con(id='otro'), 'id'),
])
def test_render_invalido(malo, por_que):
    with pytest.raises(ValueError) as ex:
        renders._check_render(malo, 'r-ok')
    assert por_que in str(ex.value)


def test_catalogo_de_renders_omite_los_invalidos(tmp_path):
    (tmp_path / 'r-ok.yaml').write_text(yaml.safe_dump(RENDER_BASE))
    (tmp_path / 'r-mal.yaml').write_text(yaml.safe_dump(dict(RENDER_BASE, id='r-mal', comando='x')))
    (tmp_path / 'roto.yaml').write_text('a: [1, 2')
    assert list(renders.load_render_catalog(tmp_path)) == ['r-ok']
    assert renders.load_render_catalog(tmp_path / 'no-existe') == {}


def test_endpoint_catalogo(client_real):
    cat = {r['id']: r for r in client_real.get('/api/renders/catalogo').json()}
    assert set(cat) == {'render-falso', 'render-pantalla', 'render-solo-loop', 'render-host', 'render-host-corto'}
    assert cat['render-host']['ejecutor'] == 'host' and cat['render-falso']['ejecutor'] == 'backend'
    assert cat['render-pantalla']['pantalla'] is True and cat['render-pantalla']['requiere'] == {'contrato': 'red'}
    assert cat['render-falso']['params_schema']['pasos']['max'] == 20


# ------------------------------------------------------------------ flujo completo
def test_render_de_punta_a_punta(client_real):
    c = client_real
    rid = run_hecho(c)
    j = render(c, rid, 'render-pantalla', {'acelerar': 45})
    assert j['estado'] == 'hecho' and j['error'] is None and j['exit_code'] == 0
    assert j['clase'] == 'gazebo' and j['pantalla'] == ':80' and j['progreso'] == 1.0
    assert j['render_id'] == 'render-pantalla' and j['args']['acelerar'] == 45 and j['reintentos'] == 0
    assert 100 <= j['ros_domain_id'] <= 199
    m = j['media']
    assert m['version'] == 1 and m['job_id'] == j['job_id'] and m['run_id'] == rid
    assert m['procedencia']['corrida']['run_id'] == rid
    assert j['miniatura'] == f'/api/medios/{j["job_id"]}/miniatura.jpg'
    # el trabajo recibio la pantalla, DISPLAY y los hilos de llvmpipe
    d = c.settings.runs_dir / rid / 'media' / j['job_id']
    log = (d / 'render.log').read_text()
    assert "pantalla ':80' DISPLAY :80 LP 3" in log and 'PROGRESO 1.000' in log
    assert {'media.json', 'visor.mp4', 'miniatura.jpg', 'render.log'} == {p.name for p in d.iterdir()}
    # la pantalla se devolvio al pool
    assert c.get('/api/sistema').json()['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    # listados
    assert [x['job_id'] for x in c.get(f'/api/renders?run_id={rid}').json()] == [j['job_id']]
    assert c.get('/api/renders?estado=hecho').json()[0]['job_id'] == j['job_id']
    assert c.get('/api/renders?estado=error').json() == []
    assert c.get('/api/renders?run_id=malo').json() == []
    assert c.get('/api/health').json()['active_renders'] == 0


def test_requiere_y_estado_de_la_corrida(client_real):
    c = client_real
    rid = run_hecho(c)
    liger = run_hecho(c, 'prueba-ligera', {})
    # el render pide contrato red: la corrida ligera no lo cumple
    r = c.post(f'/api/runs/{liger}/renders', json={'render_id': 'render-pantalla'})
    assert r.status_code == 409 and 'contrato red' in r.json()['detail']
    # {kind: loop}: una corrida `proceso` no sirve
    r = c.post(f'/api/runs/{rid}/renders', json={'render_id': 'render-solo-loop'})
    assert r.status_code == 409 and 'kind loop' in r.json()['detail']
    # {cualquiera: true}: sirve cualquiera terminada; {pantalla} vale '' si el render no pide pantalla
    j = c.post(f'/api/runs/{liger}/renders', json={'render_id': 'render-falso'})
    assert j.status_code == 201
    j = wait_job(c, j.json()['job_id'])
    log = (c.settings.runs_dir / liger / 'media' / j['job_id'] / 'render.log').read_text()
    assert j['pantalla'] is None and log.startswith("pantalla '' DISPLAY")
    # una corrida que no esta `done` no se renderiza
    c.carga.valor = 99.0
    vivo = c.post('/api/runs', json={'experiment_id': 'prueba-proceso'}).json()['run_id']
    r = c.post(f'/api/runs/{vivo}/renders', json={'render_id': 'render-falso'})
    assert r.status_code == 409 and 'no esta terminada' in r.json()['detail']
    # inexistentes e invalidos
    assert c.post(f'/api/runs/{rid}/renders', json={'render_id': 'no-existe'}).status_code == 404
    assert c.post(f'/api/runs/{rid}/renders', json={'render_id': '../x'}).status_code == 404
    assert c.post('/api/runs/20200101-000000-abcdef/renders', json={'render_id': 'render-falso'}).status_code == 404
    assert c.post('/api/runs/malo/renders', json={'render_id': 'render-falso'}).status_code == 404
    for args in ({'modo': 'borrar'}, {'pasos': 0}, {'pasos': 99}, {'comando': 'x'}, {'pasos': 'x'}):
        assert c.post(f'/api/runs/{rid}/renders', json={'render_id': 'render-falso', 'args': args}).status_code == 422
    assert c.post(f'/api/runs/{rid}/renders', json={}).status_code == 422


def test_requiere_archivo_y_contrato_inferido(tmp_path):
    d = tmp_path
    run = {'run_id': 'x', 'experiment_id': 'sin-yaml', 'kind': 'loop', 'launch': 'atp_loop.launch.py', 'spec': None}
    assert renders.cumple_requiere({'archivo': 'telemetry.jsonl'}, run, d)[0] is False
    (d / 'telemetry.jsonl').write_text('{}\n')
    assert renders.cumple_requiere({'archivo': 'telemetry.jsonl'}, run, d)[0] is True
    assert renders.cumple_requiere({'kind': 'loop'}, run, d)[0] is True
    assert renders.cumple_requiere({'kind': ['loop', 'campaign']}, run, d)[0] is True    # lista de kinds
    ok, motivo = renders.cumple_requiere({'kind': ['campaign', 'adcs']}, run, d)
    assert ok is False and 'campaign o adcs' in motivo
    assert renders.cumple_requiere({'contrato': 'red'}, run, d)[0] is False
    (d / 'escena.json').write_text('{}')
    (d / 'red.jsonl').write_text('{}\n')                      # sin YAML que lo declare: se infiere por archivos
    assert renders.cumple_requiere({'contrato': 'red'}, run, d)[0] is True
    assert renders.cumple_requiere({'contrato': 'topologia'}, run, d)[0] is False


# ------------------------------------------------------- fallas: el trabajo dice por que
@pytest.mark.parametrize('modo,fragmentos', [
    ('sinmedia', ['sin escribir media.json']),
    ('sha_malo', ['sha256 no coincide']),
    ('sinminiatura', ['miniatura']),
    ('falla', ['codigo 4']),
])
def test_render_con_error(client_real, modo, fragmentos):
    c = client_real
    j = render(c, run_hecho(c), 'render-falso', {'modo': modo})
    assert j['estado'] == 'error' and j['media'] is None and j['reintentos'] == 0
    assert all(f in j['error'] for f in fragmentos)
    assert j['progreso'] is None
    assert c.get('/api/medios').json() == []                  # un trabajo fallido no entra a la galeria
    assert c.get('/api/sistema').json()['gobernador']['corriendo'] == []


def test_cancelar_un_render_en_curso(client_real):
    c = client_real
    r = c.post(f'/api/runs/{run_hecho(c)}/renders', json={'render_id': 'render-falso', 'args': {'modo': 'dormir'}})
    job = r.json()['job_id']
    wait_until(lambda: c.get(f'/api/renders/{job}').json()['estado'] == 'corriendo')
    assert c.delete(f'/api/renders/{job}').status_code == 409
    assert c.post(f'/api/renders/{job}/cancel').json()['estado'] == 'cancelado'
    j = c.get(f'/api/renders/{job}').json()
    assert j['estado'] == 'cancelado' and j['finished_at']
    assert c.post(f'/api/renders/{job}/cancel').status_code == 409          # ya termino
    assert c.get('/api/sistema').json()['pantallas']['libres']
    assert c.delete(f'/api/renders/{job}').status_code == 200
    assert c.get(f'/api/renders/{job}').status_code == 404


def test_render_espera_por_la_carga_del_host(client_real):
    c = client_real
    rid = run_hecho(c)
    c.carga.valor = 5.0                                       # gazebo pide < 4, ligero < 10
    pant = render(c, rid, 'render-pantalla', esperar=False)['job_id']
    lig = render(c, rid, 'render-falso')                      # el ligero pasa de largo
    assert lig['estado'] == 'hecho'
    j = wait_until(lambda: (lambda x: x if x['estado'] == 'esperando_carga' else None)(c.get(f'/api/renders/{pant}').json()))
    assert j['started_at'] is None and j['pantalla'] is None
    assert c.get('/api/sistema').json()['colas']['gazebo']['esperando_carga'] == 1
    c.carga.valor = 1.0
    assert wait_job(c, pant)['estado'] == 'hecho'


def test_cancelar_un_render_que_espera_la_carga(client_real):
    c = client_real
    rid = run_hecho(c)
    c.carga.valor = 9.0
    job = render(c, rid, 'render-pantalla', esperar=False)['job_id']
    wait_until(lambda: c.get(f'/api/renders/{job}').json()['estado'] == 'esperando_carga')
    assert c.post(f'/api/renders/{job}/cancel').json()['estado'] == 'cancelado'
    c.carga.valor = 0.0
    assert c.get(f'/api/renders/{job}').json()['started_at'] is None
    assert c.get('/api/sistema').json()['gobernador']['esperando'] == []


def test_senal_externa_reintenta_el_render(client_real):
    c = client_real
    rid = run_hecho(c)
    j = render(c, rid, 'render-falso', {'modo': 'senal_una_vez'})
    assert j['estado'] == 'hecho' and j['reintentos'] == 1 and j['media'] is not None
    log = (c.settings.runs_dir / rid / 'media' / j['job_id'] / 'render.log').read_text()
    assert 'interrumpido por senal externa (codigo -9); reintento 1/2' in log
    assert 'reintento 1: de nuevo a la cola' in log


def test_senal_en_cada_intento_deja_el_render_interrumpido(client_real):
    c = client_real
    rid = run_hecho(c)
    j = render(c, rid, 'render-falso', {'modo': 'senal'})
    assert j['estado'] == 'interrumpida' and j['finished_at'] and j['reintentos'] == 2
    assert 'agoto los 2 reintentos' in j['error'] and j['media'] is None
    assert len((c.settings.runs_dir / rid / f'contador_{j["job_id"]}').read_text()) == 3
    assert c.delete(f'/api/renders/{j["job_id"]}').status_code == 200        # es final: se puede borrar


def test_borrar_la_corrida_borra_sus_renders(client_real):
    c = client_real
    rid = run_hecho(c)
    j = render(c, rid)
    assert c.delete(f'/api/runs/{rid}').status_code == 200
    assert c.get(f'/api/renders/{j["job_id"]}').status_code == 404
    assert c.get('/api/medios').json() == []


def test_no_se_borra_una_corrida_con_renders_activos(client_real):
    c = client_real
    rid = run_hecho(c)
    job = render(c, rid, 'render-falso', {'modo': 'dormir'}, esperar=False)['job_id']
    wait_until(lambda: c.get(f'/api/renders/{job}').json()['estado'] == 'corriendo')
    r = c.delete(f'/api/runs/{rid}')
    assert r.status_code == 409 and 'renders activos' in r.json()['detail']
    c.post(f'/api/renders/{job}/cancel')
    assert c.delete(f'/api/runs/{rid}').status_code == 200


def test_reinicio_marca_los_renders_activos_como_interrumpidos(client_real):
    db = client_real.app.state.db
    db.create_job(JOB, RUN, 'render-falso', 'T', {}, {'comando': ['x'], 'estilo_args': 'cli'}, 'ligero', 60)
    db.update_job(JOB, estado='corriendo')
    db.mark_interrupted()
    j = db.get_job(JOB)
    assert j['estado'] == 'interrumpida' and j['finished_at'] and 'reinicio' in j['error']


# ------------------------------------------------------------------ SSE
def test_sse_del_render(client_real):
    c = client_real
    rid = run_hecho(c)
    j = render(c, rid, 'render-falso', {'pasos': 4})
    ev = parse_sse(c.get(f'/api/renders/{j["job_id"]}/stream').text)
    nombres = [e for e, _ in ev]
    assert nombres[0] == 'estado' and nombres[-1] == 'hecho'
    assert ev[0][1]['estado'] == 'hecho'                          # una conexion tardia ve el estado final
    prog = [d['progreso'] for e, d in ev if e == 'progreso']
    assert prog == [0.25, 0.5, 0.75, 1.0]
    assert any('PROGRESO 0.250' in d['line'] for e, d in ev if e == 'log')
    assert ev[-1][1]['estado'] == 'hecho' and ev[-1][1]['media']['render'] == 'render-falso'
    assert c.get('/api/renders/rmalo/stream').status_code == 404


def test_sse_de_un_render_en_vivo(client_real):
    c = client_real
    job = render(c, run_hecho(c), 'render-falso', {'modo': 'falla'}, esperar=False)['job_id']
    ev = parse_sse(c.get(f'/api/renders/{job}/stream').text)       # espera a que termine
    assert ev[-1][0] == 'hecho' and ev[-1][1]['estado'] == 'error' and 'codigo 4' in ev[-1][1]['error']
    assert [e for e, _ in ev].count('estado') >= 1


# ------------------------------------------------------------------ galeria y escenas
def test_galeria_de_medios(client_real):
    c = client_real
    rid = run_hecho(c)
    a = render(c, rid, 'render-falso')
    b = render(c, rid, 'render-pantalla')
    g = c.get('/api/medios').json()
    assert [x['job_id'] for x in g] == [b['job_id'], a['job_id']]            # mas nuevos primero
    x = g[0]
    assert x['tipo'] == 'video' and x['ruta'] == 'visor.mp4' and (x['ancho'], x['alto']) == (1920, 1080)
    assert x['url'] == f'/api/medios/{b["job_id"]}/visor.mp4' and x['miniatura'].endswith('/miniatura.jpg')
    assert x['escena'] == 'prueba-proceso' and x['render'] == 'render-pantalla' and x['run_id'] == rid
    assert x['procedencia']['commit'] == 'deadbeef' and x['sha256'] and x['creado_utc']
    assert {f['ruta'] for f in x['archivos']} == {'visor.mp4', 'miniatura.jpg'}
    # filtros
    assert len(c.get('/api/medios?tipo=video').json()) == 2 and c.get('/api/medios?tipo=imagen').json() == []
    assert len(c.get('/api/medios?escena=prueba-proceso').json()) == 2
    assert c.get('/api/medios?escena=otra-escena').json() == [] and c.get('/api/medios?escena=../x').json() == []
    assert len(c.get('/api/medios?limit=1').json()) == 1
    # la miniatura del ultimo medio de la escena sale en /api/escenas y se sirve
    esc = {e['id']: e for e in c.get('/api/escenas').json()}
    mini = esc['prueba-proceso']['miniatura']
    assert mini == f'/api/medios/{b["job_id"]}/miniatura.jpg' and esc['n1-rampa']['miniatura'] is None
    r = c.get(mini)
    assert r.status_code == 200 and r.headers['content-type'] == 'image/jpeg' and r.content.startswith(b'\xff\xd8')


# ------------------------------------------------------------------ contrato de media.json
def media_ok(tmp_path, **cambios):
    video = b'video-falso'
    (tmp_path / 'visor.mp4').write_bytes(video)
    (tmp_path / 'miniatura.jpg').write_bytes(b'jpg')
    m = {'version': 1, 'job_id': JOB, 'render': 'r-ok', 'run_id': RUN, 'creado_utc': '2026-10-05T12:00:00Z',
         'duracion_pared_s': 3.5, 'notas': 'x',
         'archivos': [{'ruta': 'visor.mp4', 'tipo': 'video', 'ancho': 1920, 'alto': 1080, 'duracion_s': 80.0,
                       'sha256': hashlib.sha256(video).hexdigest(), 'principal': True},
                      {'ruta': 'miniatura.jpg', 'tipo': 'imagen', 'ancho': 640, 'alto': 360}],
         'procedencia': {'commit': 'abc', 'sucio': False, 'comando': ['bash', 'x'], 'args': {'a': 1},
                         'corrida': {'run_id': RUN, 'experiment_id': 'e', 'args': {}}}}
    m.update(cambios)
    return m


def valida(tmp_path, m):
    return validar_media(m, JOB, 'r-ok', RUN, tmp_path)


def test_media_valido(tmp_path):
    m = media_ok(tmp_path)
    assert valida(tmp_path, m) is m
    assert renders.archivo_principal(m)['ruta'] == 'visor.mp4'
    assert renders.miniatura_de(JOB, m) == f'/api/medios/{JOB}/miniatura.jpg'
    # sin `principal`, el principal es el primero que no es miniatura; solo imagen: la imagen
    m2 = media_ok(tmp_path, archivos=[{'ruta': 'miniatura.jpg', 'tipo': 'imagen'}, {'ruta': 'visor.mp4', 'tipo': 'video'}])
    assert renders.archivo_principal(m2)['ruta'] == 'visor.mp4'


def mutar(m, f):
    m = copy.deepcopy(m)
    f(m)
    return m


@pytest.mark.parametrize('cambio,fragmento', [
    (lambda m: m.update(version=2), 'version'),
    (lambda m: m.update(job_id='otro'), 'job_id'),
    (lambda m: m.update(render='otro'), 'render'),
    (lambda m: m.update(run_id='otro'), 'run_id'),
    (lambda m: m.update(creado_utc='ayer'), 'creado_utc'),
    (lambda m: m.update(creado_utc=5), 'creado_utc'),
    (lambda m: m.update(duracion_pared_s='mucho'), 'duracion_pared_s'),
    (lambda m: m.update(archivos=[]), 'archivos'),
    (lambda m: m.update(archivos='visor.mp4'), 'archivos'),
    (lambda m: m['archivos'].append(dict(m['archivos'][0])), 'repetido'),
    (lambda m: m['archivos'][0].update(ruta='../fuera.mp4'), 'ruta'),
    (lambda m: m['archivos'][0].update(ruta='/etc/passwd'), 'ruta'),
    (lambda m: m['archivos'][0].update(ruta='no-existe.mp4'), 'no existe'),
    (lambda m: m['archivos'][0].update(tipo='holograma'), 'tipo'),
    (lambda m: m['archivos'][0].update(ancho=1920.5), 'ancho'),
    (lambda m: m['archivos'][0].update(alto=0), 'alto'),
    (lambda m: m['archivos'][0].update(duracion_s=-1), 'duracion_s'),
    (lambda m: m['archivos'][0].update(principal='si'), 'principal'),
    (lambda m: m['archivos'][0].update(sha256='abc'), 'sha256'),
    (lambda m: m['archivos'][0].update(sha256='0' * 64), 'no coincide'),
    (lambda m: m['archivos'].pop(), 'miniatura'),                      # video sin miniatura.jpg
    (lambda m: m['archivos'][1].update(tipo='video'), 'imagen'),
    (lambda m: m.pop('procedencia'), 'procedencia'),
    (lambda m: m['procedencia'].pop('commit'), 'commit'),
    (lambda m: m['procedencia'].update(sucio='no'), 'sucio'),
    (lambda m: m['procedencia'].update(comando='bash x'), 'comando'),
    (lambda m: m['procedencia'].update(args=[]), 'args'),
    (lambda m: m['procedencia']['corrida'].update(run_id='otra'), 'corrida'),
    (lambda m: m.update(notas=5), 'notas'),
])
def test_media_invalido(tmp_path, cambio, fragmento):
    m = mutar(media_ok(tmp_path), cambio)
    with pytest.raises(MediaError) as ex:
        valida(tmp_path, m)
    assert fragmento in str(ex.value) and str(ex.value).startswith('media.json:')


def test_media_no_es_un_objeto(tmp_path):
    for malo in ([], 'x', None, 3):
        with pytest.raises(MediaError):
            valida(tmp_path, malo)


def test_media_con_enlace_que_sale_de_la_carpeta(tmp_path):
    fuera = tmp_path / 'fuera.mp4'
    fuera.write_bytes(b'secreto')
    salida = tmp_path / 'salida'
    salida.mkdir()
    (salida / 'miniatura.jpg').write_bytes(b'jpg')
    (salida / 'visor.mp4').symlink_to(fuera)
    m = {'version': 1, 'job_id': JOB, 'render': 'r-ok', 'run_id': RUN, 'creado_utc': '2026-10-05T12:00:00Z',
         'archivos': [{'ruta': 'visor.mp4', 'tipo': 'video'}, {'ruta': 'miniatura.jpg', 'tipo': 'imagen'}],
         'procedencia': {'commit': 'a', 'sucio': False, 'comando': [], 'args': {}, 'corrida': {'run_id': RUN}}}
    with pytest.raises(MediaError, match='no existe'):
        validar_media(m, JOB, 'r-ok', RUN, salida)


def test_ultimo_progreso(tmp_path):
    p = tmp_path / 'render.log'
    assert renders.ultimo_progreso(p) is None
    p.write_text('hola\nPROGRESO 0.1\nruido\nPROGRESO 0.45\nPROGRESO x\n')
    assert renders.ultimo_progreso(p) == 0.45
    p.write_text('PROGRESO 7\n')
    assert renders.ultimo_progreso(p) == 1.0                         # se acota a [0, 1]
