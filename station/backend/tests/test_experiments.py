"""Catalogo real (experiments/*.yaml) y validacion de argumentos."""
from pathlib import Path

import pytest

from station import experiments
from station.config import Settings

REPO = Path(__file__).resolve().parents[3]

REQUIRED = {'n1-rampa', 'n2-keyhole-cenital', 'n4a-pase-oblicuo-s',
            'n4b-ka-rafaga', 'n4c-montecarlo-analitico', 'n4c-montecarlo-lazo',
            's6-plan-24h', 'e1-adcs-funcube'}


@pytest.fixture(scope='module')
def catalog():
    return experiments.load_catalog(REPO / 'experiments')


def test_catalogo_completo(catalog):
    assert REQUIRED <= set(catalog)


def test_cada_experimento_es_coherente(catalog):
    for e in catalog.values():
        assert e['kind'] in ('loop', 'campaign', 'plan', 'adcs', 'mision', 'proceso')
        if e['kind'] == 'proceso':
            # escenas genericas (docs/APP.md 2): su validacion estricta vive en experiments.py y en
            # test_los_yaml_reales_del_repo_validan; aqui solo lo comun a todo el catalogo
            assert e['comando'] and e['hecho_cuando'] and e['area']
            assert len(e['description']) > 200
            for k, spec in e['params_schema'].items():
                assert k in e['args'], f"{e['id']}: params_schema.{k} sin valor por defecto en args"
                experiments._check_value(k, spec, e['args'][k])
            continue
        # todo lanzamiento de lazo acepta record_dir (el grabador) y cierra solo
        assert e['launch'] == {'campaign': 'campaign_run.launch.py', 'plan': 'planificar.py',
                               'adcs': 'adcs.launch.py'}.get(e['kind'], e['launch'])
        if e['kind'] == 'loop':
            assert e['launch'] in ('atp_loop.launch.py', 'atp_autotrack.launch.py')
        assert len(e['description']) > 200
        # los valores por defecto de los parametros editables respetan su propio esquema
        # y TODO parametro editable trae valor por defecto en `args` (CONTRATO 5): el
        # formulario del frontend se inicializa con args[k]; sin el, el campo queda vacio
        # y el boton Lanzar inhabilitado (hallazgo del E2E: n2 vel_max, n4a eirp_dbw)
        for k, spec in e['params_schema'].items():
            assert k in e['args'], f"{e['id']}: params_schema.{k} sin valor por defecto en args"
            experiments._check_value(k, spec, e['args'][k])


def test_args_de_validar_sh(catalog):
    n4b = catalog['n4b-ka-rafaga']['args']
    # time_scale 0: lockstep a maxima velocidad (resultado identico al de x1)
    assert (n4b['el_max'], n4b['mask'], n4b['time_scale'], n4b['band']) == (85, 20, 0, 'Ka')
    assert (n4b['gust_nm'], n4b['gust_start']) == (3.0, 60)
    assert catalog['n4a-pase-oblicuo-s']['args']['time_scale'] == 0
    assert catalog['n4c-montecarlo-lazo']['args']['mode'] == 'lazo'
    assert catalog['n4c-montecarlo-analitico']['args']['n'] == 2000


def test_validacion_ok(catalog):
    e = catalog['n4a-pase-oblicuo-s']
    a = experiments.validate_args(e, {'el_max': 45, 'band': 'Ka'})
    assert a['el_max'] == 45 and a['band'] == 'Ka' and a['mask'] == 5


@pytest.mark.parametrize('args', [
    {'profile': 'ramp'},               # clave no declarada (fija en el YAML)
    {'el_max': 5},                     # bajo el minimo
    {'el_max': 95},                    # sobre el maximo
    {'el_max': '45'},                  # tipo
    {'el_max': True},                  # bool no es numero
    {'el_max': float('nan')},
    {'band': 'Q'},                     # fuera del enum
    {'band': ['S']},
    {'el_max; rm -rf /': 1},           # clave hostil
])
def test_validacion_rechaza(catalog, args):
    with pytest.raises(experiments.ArgError):
        experiments.validate_args(catalog['n4a-pase-oblicuo-s'], args)


def test_integer_y_modo_fijo(catalog):
    e = catalog['n4c-montecarlo-lazo']
    assert experiments.validate_args(e, {'n': 100.0})['n'] == 100
    for bad in ({'n': 1.5}, {'n': 5}, {'n': 99999}, {'mode': 'analitico'}):
        with pytest.raises(experiments.ArgError):
            experiments.validate_args(e, bad)


def test_cli_args():
    out = experiments.cli_args({'a': 1, 'b': 0.5, 'c': True, 'band': 'Ka'})
    assert out == ['a:=1', 'b:=0.5', 'c:=true', 'band:=Ka']


def test_yaml_roto_se_omite(tmp_path):
    (tmp_path / 'malo.yaml').write_text('id: malo\n', encoding='utf-8')
    (tmp_path / 'otro.yaml').write_text(': : :', encoding='utf-8')
    assert experiments.load_catalog(tmp_path) == {}


def test_settings_por_defecto():
    s = Settings()
    assert s.max_concurrent == 2 and (s.domain_min, s.domain_max) == (100, 199)


def test_kinds_plan_y_adcs(catalog):
    p, a = catalog['s6-plan-24h'], catalog['e1-adcs-funcube']
    assert (p['kind'], p['launch']) == ('plan', 'planificar.py')
    assert (a['kind'], a['launch']) == ('adcs', 'adcs.launch.py')
    assert experiments.kind_of(p['launch']) == 'plan' and experiments.kind_of(a['launch']) == 'adcs'
    assert experiments.kind_of('campaign_run.launch.py') == 'campaign'
    assert experiments.kind_of('atp_loop.launch.py') == 'loop'
    assert [experiments.result_file(x) for x in ('planificar.py', 'campaign_run.launch.py',
                                                 'adcs.launch.py', 'atp_loop.launch.py')] == [
        'plan.json', 'campaign.json', 'metrics.json', 'metrics.json']
    # s6: red de 4 estaciones, los NOAA/METEOR/ISS y fecha fija (repetible byte a byte)
    assert p['args']['estaciones'] == 'cdmx,tromso,ecuador,madrid'
    assert {'ISS', 'NOAA 19', 'METEOR-M2 4'} <= set(p['args']['sats'].split(','))
    assert p['args']['inicio'] == '2026-10-02T12:00:00' and 'inicio' not in p['params_schema']
    assert a['args']['time_scale'] == 0 and 'time_scale' not in a['params_schema']


def test_validacion_del_plan_y_adcs(catalog):
    a = experiments.validate_args(catalog['s6-plan-24h'], {'horas': 2, 'sin': ''})
    assert a['horas'] == 2 and a['sin'] == '' and a['banda'] == 'X' and a['keyhole'] == 'marcar'
    b = experiments.validate_args(catalog['e1-adcs-funcube'], {'start_utc': '2026-10-02T21:20:00', 'seed': 3.0})
    assert b['seed'] == 3 and isinstance(b['seed'], int) and b['model_degree'] == 4


def test_script_args():
    out = experiments.script_args({'sats': 'ISS,NOAA 19', 'margen_acm': 1.0, 'sin': '', 'x': True})
    assert out == ['--sats=ISS,NOAA 19', '--margen-acm=1.0', '--x=true']       # '' se omite
    assert experiments.script_args({'horas': 0}) == ['--horas=0']              # 0 no es vacio


@pytest.mark.parametrize('d,msg', [
    ({'kind': 'plan', 'launch': 'atp_loop.launch.py'}, 'kind plan'),
    ({'kind': 'plan', 'launch': '../../bin/sh'}, 'kind plan'),
    ({'kind': 'adcs', 'launch': 'atp_loop.launch.py'}, 'adcs'),
    ({'kind': 'loop', 'launch': 'adcs.launch.py'}, 'adcs'),
    ({'kind': 'loop', 'launch': 'planificar.py'}, 'launch'),
    ({'kind': 'loop', 'launch': 'mision.py'}, 'launch'),
    ({'kind': 'mision', 'launch': 'atp_loop.launch.py'}, 'kind mision'),
    ({'kind': 'mision', 'launch': '../../bin/sh'}, 'kind mision'),
    ({'kind': 'otro', 'launch': 'atp_loop.launch.py'}, 'kind'),
])
def test_kind_y_launch_deben_coincidir(d, msg):
    base = {'id': 'x', 'title': 't', 'level': 'N1', 'description': 'd', 'args': {}, 'params_schema': {}}
    with pytest.raises(ValueError, match=msg):
        experiments._check_experiment({**base, **d}, 'x')


def test_comando_real_segun_el_kind(tmp_path):
    from station.runner import default_command
    d = tmp_path / 'runs' / '20261002-120000-abc123'
    plan = default_command({'launch': 'planificar.py', 'args': {'horas': 3, 'sin': ''}}, d)
    assert 'python' in plan[0] and plan[1].endswith('scripts/planificar.py')
    assert plan[2:] == ['--horas=3', '--id=20261002-120000-abc123', f'--salida={d.parent}']
    adcs = default_command({'launch': 'adcs.launch.py', 'args': {'seed': 7}}, d)
    assert adcs == ['ros2', 'launch', 'sat_adcs', 'adcs.launch.py', 'seed:=7', f'record_dir:={d}']
    loop = default_command({'launch': 'atp_loop.launch.py', 'args': {'band': 'S'}}, d)
    assert loop == ['ros2', 'launch', 'atp_ros', 'atp_loop.launch.py', 'band:=S', f'record_dir:={d}']


def test_csv_de_adcs_aplana_los_arreglos():
    from station import stream
    filas = [{'t': 0.0, 'mode': 0, 'w_deg_s': 3.0, 'q': [1, 0, 0, 0], 'h_nms': [0.1, 0.2, 0.3],
              'eclipse': False, 'el_deg': -60.0, 'range_km': 11000.0, 'in_pass': False,
              'u_w_nm': [0, 0, 0], 'm_am2': [0.1, 0, 0]},
             {'t': 1.0, 'mode': 1, 'w_deg_s': 2.0, 'q': [0, 1, 0, 0], 'q_est': [0, 1, 0, 0], 'err_deg': 1.5,
              'h_nms': [0, 0, 0], 'eclipse': True, 'el_deg': 1.0, 'range_km': 2.0, 'in_pass': True,
              'u_w_nm': [0, 0, 0], 'm_am2': [0, 0, 0]}]
    lineas = stream.to_csv(filas).strip().splitlines()
    cab = lineas[0].split(',')
    assert cab[:7] == ['t', 'mode', 'w_deg_s', 'q_0', 'q_1', 'q_2', 'q_3'] and 'h_nms_2' in cab and 'err_deg' in cab
    fila0 = dict(zip(cab, lineas[1].split(',')))
    assert fila0['q_0'] == '1' and fila0['h_nms_1'] == '0.2' and fila0['err_deg'] == ''   # omitido -> vacio
    assert dict(zip(cab, lineas[2].split(',')))['q_est_1'] == '1'
    # la de un lazo sigue con las columnas del CONTRATO 2
    assert stream.to_csv([{'t': 0.0, 'az': 1.0}]).splitlines()[0].startswith('t,ref_az')


def test_kind_mision(catalog):
    m = catalog['m1-dia-operaciones']
    assert (m['kind'], m['launch']) == ('mision', 'mision.py')
    assert experiments.kind_of('mision.py') == 'mision' and experiments.result_file('mision.py') == 'informe.json'
    assert {'ISS', 'NOAA 20', 'METEOR-M2 4'} <= set(m['args']['sats'].split(','))
    assert m['args']['inicio'] == '2026-10-02T12:00:00' and 'inicio' not in m['params_schema']
    a = experiments.validate_args(m, {'horas': 3, 'n_od': 2.0})
    assert a['horas'] == 3 and a['n_od'] == 2 and a['gemelos'] == 'programa'
    assert experiments.script_args(a)[:2] == ['--sats=ISS,NOAA 15,NOAA 18,NOAA 19,NOAA 20,NOAA 21,METEOR-M2 3,METEOR-M2 4',
                                              '--estaciones=cdmx,tromso,ecuador,madrid']
