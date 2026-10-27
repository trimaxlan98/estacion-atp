"""`kind: proceso` (docs/APP.md 2): validacion estricta del YAML, construccion del comando, fin por
`hecho_cuando`, metricas y artefactos. La parte E2E ejecuta scripts de tests/fixtures (sin ROS)."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from conftest import REPO, wait_status, wait_until
from station import experiments
from station.experiments import (ArgError, _check_experiment, proceso_args, proceso_command,
                                 proceso_spec, sustituir_marcadores, validate_args)
from station.runner import default_command

BASE = dict(
    id='red-prueba', title='Red de prueba', level='R1', kind='proceso', area='red',
    description='Que se mide.', comando=['python3', '-m', 'atp_demo.cli', '--salida', '{runs_dir}', '--id', '{run_id}'],
    estilo_args='cli', hecho_cuando='metricas.json', metricas='metricas.json',
    artefactos=[{'ruta': 'red.jsonl', 'contrato': 'red'}, {'ruta': 'eventos.jsonl', 'contrato': 'red_eventos'}],
    recursos='pesado', timeout_s=1800,
    args={'n_sats': 72, 'modo': 'mbb'},
    params_schema={'n_sats': {'type': 'integer', 'min': 1, 'max': 100},
                   'modo': {'type': 'enum', 'options': ['mbb', 'bbm']}})


def con(**cambios):
    d = copy.deepcopy(BASE)
    for k, v in cambios.items():
        if v is KeyError:
            d.pop(k)
        else:
            d[k] = v
    return d


def test_el_yaml_de_ejemplo_valida():
    _check_experiment(copy.deepcopy(BASE), 'red-prueba')
    minimo = {k: BASE[k] for k in ('id', 'title', 'level', 'kind', 'description', 'comando', 'hecho_cuando',
                                   'area', 'args', 'params_schema')}
    d = copy.deepcopy(minimo)
    _check_experiment(d, 'red-prueba')
    assert d['launch'] == 'proceso'


@pytest.mark.parametrize('malo,por_que', [
    (con(comando='python3 -m x'), 'comando'),                       # un texto, no una lista (shell)
    (con(comando=[]), 'comando'),
    (con(comando=['python3', 5]), 'comando'),
    (con(comando=['python3', '']), 'comando'),
    (con(comando=KeyError), 'comando'),
    (con(comando=['python3', '{desconocido}']), 'marcador'),
    (con(comando=['python3', '{pantalla}']), 'marcador'),           # {pantalla} es de los renders
    (con(estilo_args='shell'), 'estilo_args'),
    (con(hecho_cuando=KeyError), 'hecho_cuando'),
    (con(hecho_cuando='../fuera.json'), 'ruta'),
    (con(hecho_cuando='/etc/passwd'), 'ruta'),
    (con(hecho_cuando='a//b.json'), 'ruta'),
    (con(hecho_cuando=''), 'ruta'),
    (con(metricas='../x'), 'ruta'),
    (con(area='cocina'), 'area'),
    (con(area=KeyError), 'area'),
    (con(recursos='enorme'), 'recursos'),
    (con(timeout_s=0), 'timeout'),
    (con(timeout_s='mucho'), 'timeout'),
    (con(timeout_s=True), 'timeout'),
    (con(artefactos='red.jsonl'), 'artefactos'),
    (con(artefactos=[{'ruta': 'x', 'extra': 1}]), 'artefactos'),
    (con(artefactos=[{'contrato': 'red'}]), 'artefactos'),
    (con(artefactos=[{'ruta': '/abs'}]), 'ruta'),
    (con(artefactos=[{'ruta': 'ok', 'contrato': 'Red'}]), 'contrato'),
    (con(launch='atp_loop.launch.py'), 'launch'),
    (con(id='otro-id'), 'id'),
    (con(args={'Mala-Clave': 1}), 'clave'),
    (con(params_schema={'n': {'type': 'complejo'}}), 'type'),
    (con(params_schema={'n': {'type': 'enum'}}), 'enum'),
    (con(kind='inventado'), 'kind'),
])
def test_yaml_proceso_invalido(malo, por_que):
    with pytest.raises(ValueError) as ex:
        _check_experiment(malo, 'red-prueba')
    assert por_que in str(ex.value)


def test_load_catalog_omite_el_invalido_y_conserva_los_demas(tmp_path):
    (tmp_path / 'red-prueba.yaml').write_text(yaml.safe_dump(BASE))
    malo = dict(BASE, id='roto', comando='shell -c algo')
    (tmp_path / 'roto.yaml').write_text(yaml.safe_dump(malo))
    cat = experiments.load_catalog(tmp_path)
    assert list(cat) == ['red-prueba']


def test_kinds_anteriores_siguen_validos_con_metadatos_opcionales():
    d = yaml.safe_load((REPO / 'experiments' / 'n4a-pase-oblicuo-s.yaml').read_text())
    _check_experiment(copy.deepcopy(d), 'n4a-pase-oblicuo-s')
    _check_experiment(dict(copy.deepcopy(d), area='control', recursos='ligero'),
                      'n4a-pase-oblicuo-s')
    with pytest.raises(ValueError):
        _check_experiment(dict(copy.deepcopy(d), area='cocina'), 'n4a-pase-oblicuo-s')
    with pytest.raises(ValueError):
        _check_experiment(dict(copy.deepcopy(d), recursos='enorme'), 'n4a-pase-oblicuo-s')


def test_los_yaml_reales_del_repo_validan():
    """Si alguien (AC) agrega escenas o renders reales con un error, esta prueba dice cual."""
    for carpeta, check in (('experiments', _check_experiment),):
        for p in sorted((REPO / carpeta).glob('*.yaml')):
            check(yaml.safe_load(p.read_text(encoding='utf-8')), p.stem)
    render = REPO / 'render'
    if render.is_dir():
        from station.renders import _check_render
        for p in sorted(render.glob('*.yaml')):
            _check_render(yaml.safe_load(p.read_text(encoding='utf-8')), p.stem)


# ------------------------------------------------------------------ comando
CTX = {'run_id': '20261005-120000-abc123', 'run_dir': Path('/r/20261005-120000-abc123'),
       'runs_dir': Path('/r'), 'domain': 117, 'root': Path('/ws')}


def test_marcadores():
    out = sustituir_marcadores(['{root}/x', 'a{run_id}b', '{run_dir}', '{runs_dir}', '{domain}', 'sin'], CTX)
    assert out == ['/ws/x', 'a20261005-120000-abc123b', '/r/20261005-120000-abc123', '/r', '117', 'sin']


def test_args_estilo_cli_respeta_guiones_bajos_y_omite_vacios():
    out = proceso_args({'n_sats': 72, 'dur_s': 2400.5, 'activo': True, 'otro': False, 'nombre': '',
                        'x': -1, 'm': 'mbb', 'raro': '-guion'}, 'cli')
    assert out == ['--n_sats', '72', '--dur_s', '2400.5', '--activo', 'true', '--otro', 'false',
                   '--x=-1', '--m', 'mbb', '--raro=-guion']


def test_args_estilo_ros():
    assert proceso_args({'n_sats': 72, 'activo': True, 'vacio': ''}, 'ros') == ['n_sats:=72', 'activo:=true']


def test_args_dict_y_lista_van_como_json_valido():
    import json
    out = proceso_args({'ruta': [[12, 0], [12, 10]], 'terreno_params': {'altura': 0.18, 'x_inicio': 3}, 'nombres': ['a', 'b']}, 'cli')
    assert out == ['--ruta', '[[12, 0], [12, 10]]', '--terreno_params', '{"altura": 0.18, "x_inicio": 3}', '--nombres', '["a", "b"]']
    assert json.loads(out[3]) == {'altura': 0.18, 'x_inicio': 3}
    # con str() un dict salia `{'altura': 0.18}` (comillas simples): json.loads fallaba y rover.py recibia un texto
    assert str({'altura': 0.18}) != json.dumps({'altura': 0.18})
    # una lista numerica sale igual que antes (las escenas con `ruta` ya funcionaban)
    assert json.dumps([[12, 0], [12, 10]]) == str([[12, 0], [12, 10]])


def test_comando_completo():
    spec = proceso_spec(BASE)
    cmd = proceso_command(spec, {'n_sats': 8, 'modo': 'bbm'}, CTX)
    assert cmd == ['python3', '-m', 'atp_demo.cli', '--salida', '/r', '--id', '20261005-120000-abc123',
                   '--n_sats', '8', '--modo', 'bbm']
    # los valores de usuario nunca se interpretan: van como UN elemento del argv, sin shell
    cmd = proceso_command(dict(spec, estilo_args='cli'), {'modo': '$(rm -rf /); `x` | y'}, CTX)
    assert cmd[-2:] == ['--modo', '$(rm -rf /); `x` | y']


def test_default_command_usa_el_spec_guardado():
    run = {'kind': 'proceso', 'launch': 'proceso', 'spec': proceso_spec(BASE), 'args': {'n_sats': 5},
           'ros_domain_id': 133}
    rd = Path('/runs/20261005-120000-abc123')
    cmd = default_command(run, rd)
    assert cmd[:3] == ['python3', '-m', 'atp_demo.cli'] and cmd[3:7] == ['--salida', '/runs', '--id', rd.name]
    assert cmd[-2:] == ['--n_sats', '5']


def test_validate_args_en_proceso():
    exp = copy.deepcopy(BASE)
    assert validate_args(exp, {'n_sats': 8}) == {'n_sats': 8, 'modo': 'mbb'}
    for malo in ({'n_sats': 0}, {'n_sats': 101}, {'n_sats': 1.5}, {'modo': 'x'}, {'salida': '/tmp'}):
        with pytest.raises(ArgError):
            validate_args(exp, malo)


# ------------------------------------------------------------------ E2E con scripts de prueba
def lanzar(c, exp, args=None):
    r = c.post('/api/runs', json={'experiment_id': exp, 'args': args})
    assert r.status_code == 201, r.text
    return r.json()['run_id']


def test_catalogo_trae_los_proceso_de_prueba(client_real):
    por_id = {e['id']: e for e in client_real.get('/api/experiments').json()}
    assert por_id['prueba-proceso']['kind'] == 'proceso' and por_id['prueba-proceso']['launch'] == 'proceso'


def test_proceso_de_punta_a_punta(client_real):
    c = client_real
    rid = lanzar(c, 'prueba-proceso', {'n_sats': 5, 'dur_s': 6})
    r = wait_status(c, rid)
    assert r['status'] == 'done', r
    assert r['kind'] == 'proceso' and r['clase'] == 'pesado' and r['exit_code'] == 0
    assert r['metrics'] == {'n_sats': 5, 'handovers': 1, 'veredicto': 'OK'}      # de `metricas`
    d = c.settings.runs_dir / rid
    # el proceso recibio los marcadores y su ROS_DOMAIN_ID propio
    assert (d / 'dominio.txt').read_text() == str(r['ros_domain_id']) and 100 <= r['ros_domain_id'] <= 199
    assert 'n_sats 5' in c.get(f'/api/runs/{rid}/log').text
    assert len((d / 'red.jsonl').read_text().splitlines()) == 7
    # params.json igual que en las demas corridas (CONTRATO 1)
    p = json.loads((d / 'params.json').read_text())
    assert set(p) == {'experiment_id', 'launch', 'args', 'ros_domain_id', 'created_at'}
    assert p['launch'] == 'proceso' and p['args']['n_sats'] == 5 and p['args']['modo'] == 'ok'
    # artefactos que la app sabe mostrar, con su contrato y si existen
    arts = {a['ruta']: a for a in r['artefactos']}
    assert arts['red.jsonl'] == {'ruta': 'red.jsonl', 'contrato': 'red', 'existe': True}
    assert arts['eventos.jsonl']['contrato'] == 'red_eventos' and arts['eventos.jsonl']['existe']
    # la lista no trae el detalle, pero si el resumen con las metricas
    fila = next(x for x in c.get('/api/runs').json() if x['run_id'] == rid)
    assert fila['metrics']['veredicto'] == 'OK' and 'artefactos' not in fila
    # la telemetria de una corrida sin telemetry.jsonl es una lista vacia, no un error
    assert c.get(f'/api/runs/{rid}/telemetry').json() == []


def test_artefactos_declarados_se_sirven(client_real):
    """C4a: GET /runs/{id}/artefactos/{ruta} solo sirve lo que la escena declara y existe."""
    c = client_real
    rid = lanzar(c, 'prueba-proceso', {'n_sats': 5, 'dur_s': 6})
    assert wait_status(c, rid)['status'] == 'done'
    r = c.get(f'/api/runs/{rid}/artefactos/red.jsonl')
    assert r.status_code == 200 and r.headers['content-type'].startswith('application/x-ndjson')
    assert len(r.text.splitlines()) == 7
    assert c.get(f'/api/runs/{rid}/artefactos/red.jsonl', headers={'Range': 'bytes=0-4'}).status_code == 206
    assert c.head(f'/api/runs/{rid}/artefactos/red.jsonl').status_code == 200
    # no declarados (existen en la carpeta) o inexistentes o que escapan: 404
    for ruta in ('params.json', 'dominio.txt', 'no-existe.json', '..%2Fparams.json'):
        assert c.get(f'/api/runs/{rid}/artefactos/{ruta}').status_code == 404, ruta
    assert c.get('/api/runs/r00000000-000000-xxxx/artefactos/red.jsonl').status_code == 404


def test_proceso_estilo_ros_y_sin_metricas(client_real):
    rid = lanzar(client_real, 'prueba-ligera', {'alfa': 2.5})
    r = wait_status(client_real, rid)
    assert r['status'] == 'done' and r['clase'] == 'ligero' and r['metrics'] is None
    assert (client_real.settings.runs_dir / rid / 'fin.txt').read_text() == 'alfa:=2.5 activo:=true'


@pytest.mark.parametrize('modo,fragmento', [('falla', 'codigo 2'), ('sinhecho', 'metricas.json no existe')])
def test_proceso_que_no_cumple_hecho_cuando_es_error(client_real, modo, fragmento):
    r = wait_status(client_real, lanzar(client_real, 'prueba-proceso', {'modo': modo, 'dur_s': 3}))
    assert r['status'] == 'error' and fragmento in r['error'] and r['metrics'] is None
    assert r['reintentos'] == 0                       # un error propio no se reintenta


def test_proceso_en_curso_se_cancela(client_real):
    rid = lanzar(client_real, 'prueba-proceso', {'modo': 'dormir'})
    wait_until(lambda: client_real.get(f'/api/runs/{rid}').json()['status'] == 'running')
    assert client_real.post(f'/api/runs/{rid}/cancel').json()['status'] == 'cancelled'


def test_args_hostiles_en_proceso_son_422(client_real):
    for args in ({'modo': 'ok; rm -rf /'}, {'salida': '/tmp'}, {'n_sats': 0}, {'n_sats': 'x'},
                 {'dur_s': 1}, {'id': 'x'}):
        r = client_real.post('/api/runs', json={'experiment_id': 'prueba-proceso', 'args': args})
        assert r.status_code == 422, args
    assert client_real.get('/api/runs').json() == []


def test_corrida_en_cola_conserva_su_spec_aunque_cambie_el_yaml(client_real, tmp_path):
    c = client_real
    c.carga.valor = 99.0                                     # queda esperando la carga
    rid = lanzar(c, 'prueba-proceso')
    yml = Path(c.settings.experiments_dir) / 'prueba-proceso.yaml'
    d = yaml.safe_load(yml.read_text())
    d['comando'] = ['python3', '-c', 'import sys; sys.exit(9)']
    yml.write_text(yaml.safe_dump(d))
    c.carga.valor = 0.0
    assert wait_status(c, rid)['status'] == 'done'           # corrio lo que se pidio, no lo nuevo


def test_escenas_enriquece_el_catalogo(client_real):
    esc = {e['id']: e for e in client_real.get('/api/escenas').json()}
    p = esc['prueba-proceso']
    assert (p['area'], p['recursos']) == ('red', 'pesado')
    assert p['artefactos'][0] == {'ruta': 'red.jsonl', 'contrato': 'red'}
    assert p['miniatura'] is None and p['kind'] == 'proceso'
    # las escenas de las rondas anteriores salen igual, con valores por defecto
    q = esc['n4a-pase-oblicuo-s']
    assert (q['area'], q['recursos'], q['artefactos']) == (None, 'pesado', [])
    assert esc['s6-plan-24h']['recursos'] == 'ligero'
    viejo = next(e for e in client_real.get('/api/experiments').json() if e['id'] == 'n4a-pase-oblicuo-s')
    assert all(q[k] == v for k, v in viejo.items())                       # /escenas es un superconjunto
