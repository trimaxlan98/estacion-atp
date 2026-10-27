"""Pruebas del equipo R1 (CONTRATO sec. 18): planetas, terreno, dinamica del rover y determinismo. Sin ROS."""
import json
import math

import numpy as np
import pytest

from atp_rover import cli
from atp_rover.planetas import PLANETAS, planeta
from atp_rover.rover import DT, Rover
from atp_rover.terreno import TIPOS, Terreno


def _rover(pl='luna', tipo='llano', suelo=None, rho0=False, rumbo=0.0, **kw):
    P = planeta(pl)
    if rho0:
        P['densidad_atm'] = 0.0
    if suelo:
        P['suelo'].update(suelo)
    T = Terreno(tipo, kw.pop('semilla', 0), P['suelo'], **kw)
    return Rover(P, T, rumbo_deg=rumbo)


# ---- planetas ---------------------------------------------------------------------------------------------------
def test_planetas_campos_y_g():
    assert set(PLANETAS) == {'tierra', 'luna', 'marte', 'titan', 'ceres'}
    for k, p in PLANETAS.items():
        assert {'g', 'suelo', 'densidad_atm', 'temp_k', 'retardo_tierra_s', 'nombre_es'} <= set(p), k
        assert {'kc', 'kphi', 'n', 'c', 'phi_deg', 'k_deslizamiento'} <= set(p['suelo']), k
    assert PLANETAS['tierra']['g'] == pytest.approx(9.80665)
    assert PLANETAS['luna']['g'] == pytest.approx(1.62)
    assert PLANETAS['marte']['g'] == pytest.approx(3.71)
    assert PLANETAS['titan']['g'] == pytest.approx(1.352)
    assert PLANETAS['ceres']['g'] < 0.3
    planeta('luna')['g'] = 0.0                      # la copia no altera el original
    assert PLANETAS['luna']['g'] == pytest.approx(1.62)


# ---- terreno ----------------------------------------------------------------------------------------------------
def test_terreno_determinista_y_semilla():
    for tipo in TIPOS:
        a, b = Terreno(tipo, 5, extension=40.0), Terreno(tipo, 5, extension=40.0)
        assert a.bytes_huella() == b.bytes_huella(), tipo
    for tipo in ('rocoso', 'dunas'):
        assert Terreno(tipo, 5, extension=40.0).bytes_huella() != Terreno(tipo, 6, extension=40.0).bytes_huella()


def test_terreno_altura_normal_suelo():
    P = planeta('marte')
    t = Terreno('pendiente', 0, P['suelo'], angulo_deg=20.0, extension=40.0)
    assert t.altura(10.0, 3.0) == pytest.approx(10.0 * math.tan(math.radians(20)), abs=1e-6)
    n = t.normal(np.array([1.0, 5.0]), np.array([0.0, 2.0]))
    assert np.allclose(np.linalg.norm(n, axis=-1), 1.0)
    assert t.pendiente_deg(3.0, 3.0) == pytest.approx(20.0, abs=0.01)
    ll = Terreno('llano', 0, P['suelo'], extension=40.0)
    assert ll.pendiente_deg(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert t.suelo(0.0, 0.0)['phi_deg'] == pytest.approx(P['suelo']['phi_deg'])
    d = Terreno('dunas', 1, P['suelo'], extension=60.0)           # arena suelta: menos friccion que el suelo base
    assert d.suelo(0.0, 0.0)['phi_deg'] < P['suelo']['phi_deg']
    r = Terreno('rocoso', 2, P['suelo'], extension=60.0)
    assert abs(r.altura(0.0, 0.0)) < 1e-9                         # arranque llano
    assert np.ptp(r.z) > 0.1
    c = Terreno('crater', 0, P['suelo'], extension=80.0)
    assert c.altura(25.0, 0.0) < -2.5                             # fondo del cuenco


# ---- caida libre ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('pl', ['tierra', 'luna', 'marte', 'titan', 'ceres'])
def test_caida_libre_g_del_planeta(pl):
    r = _rover(pl, rho0=True, extension=40.0)
    r.s[2] += 30.0
    z0 = r.s[2]
    e0 = r.energia_mecanica()
    for _ in range(100):                                  # 1 s
        r.paso(0.0, 0.0)
    g = PLANETAS[pl]['g']
    assert r.s[9] == pytest.approx(-g * 1.0, rel=1e-6)
    assert r.s[2] - z0 == pytest.approx(-0.5 * g, abs=1e-3)
    assert abs(r.energia_mecanica() - e0) / abs(e0) < 1e-9     # energia conservada


def test_caida_libre_con_atmosfera_frena_algo():
    """Con atmosfera (Tierra, 1 s) la velocidad es menor que g*t pero cerca (Cd*A = 0.8 m2)."""
    r = _rover('tierra', extension=40.0)
    r.s[2] += 30.0
    for _ in range(100):
        r.paso(0.0, 0.0)
    g = PLANETAS['tierra']['g']
    assert 0.9 * g < -r.s[9] < g


# ---- llano sin deslizamiento --------------------------------------------------------------------------------------
def test_llano_sin_deslizamiento_ni_deriva():
    r = _rover('luna', extension=60.0)
    r.asentar(1.0)
    for _ in range(1200):                                 # 12 s a 0.5 m/s
        r.paso(0.5, 0.0)
    e = r.estado()
    m = r.metricas()
    assert m['deslizamiento_medio'] < 0.1
    assert abs(e['y']) < 0.02 and abs(e['rumbo_deg']) < 0.2
    assert abs(e['alabeo_deg']) < 0.1
    assert e['v'] == pytest.approx(0.5, rel=0.1)
    assert m['atascos'] == 0 and m['vuelcos'] == 0


# ---- limite de pendiente por phi del suelo ------------------------------------------------------------------------
def _sube(phi, ang, t=8.0):
    r = _rover('luna', 'pendiente', suelo={'phi_deg': phi, 'c': 100.0}, angulo_deg=ang, extension=60.0)
    r.asentar(1.0)
    for _ in range(int(t / DT)):
        r.paso(0.5, 0.0)
    return float(r.s[0])


def test_limite_de_pendiente_por_phi():
    phi = 25.0
    avance = {a: _sube(phi, a) for a in (4, 14, 20, 32)}
    assert avance[4] > 2.0                                 # sube con holgura (el resto, deslizamiento)
    assert avance[4] > avance[14] > avance[20]  # cuanto mas empinada, menos avanza
    assert avance[14] > 0.0                                # aun sube con 14 grados
    # el limite de traccion queda POR DEBAJO de phi: T(1) = (1 - K/L ...) tan(phi) < tan(phi), mas la compactacion
    assert avance[20] < 0.0 and avance[32] < 0.0           # con 20 y 32 grados (< y > phi=25) retrocede
    # un suelo con mas friccion sube lo que el otro no
    assert _sube(40.0, 20) > 1.0 > avance[20]


# ---- vuelco -------------------------------------------------------------------------------------------------------
def _ladea(ang):
    r = _rover('tierra', 'pendiente', suelo={'phi_deg': 60.0, 'c': 3000.0}, angulo_deg=ang, rumbo=90.0,
               extension=60.0)
    r.asentar(1.0)
    for _ in range(800):
        r.paso(0.0, 0.0)
    return r


def test_vuelco_sobre_el_angulo_estatico():
    """Estatico: atan(semivia / altura del CM) = atan(0.40 / 0.40) = 45 grados (la suspension lo baja un poco)."""
    estable = _ladea(35.0)
    assert not estable.vuelco and abs(estable.estado()['alabeo_deg']) < 36.0
    volcado = _ladea(52.0)
    assert volcado.vuelco and volcado.metricas()['vuelcos'] == 1


# ---- mismo orden, distinto planeta ---------------------------------------------------------------------------------
def test_misma_orden_distinto_resultado_por_planeta():
    res = {}
    for pl in ('luna', 'marte', 'tierra'):
        r = _rover(pl, extension=60.0)
        r.asentar(1.0)
        for _ in range(1000):
            r.paso(1.0, 0.0)
        m = r.metricas()
        res[pl] = (m['distancia_m'], m['deslizamiento_medio'], m['energia_wh'])
    for a, b in (('luna', 'marte'), ('luna', 'tierra'), ('marte', 'tierra')):
        for i in range(3):
            assert abs(res[a][i] - res[b][i]) > 0.01 * max(abs(res[a][i]), abs(res[b][i])), (a, b, i, res)
    assert res['marte'][1] == min(v[1] for v in res.values())   # Marte (suelo mas cohesivo) es el que menos resbala


# ---- contrato y determinismo --------------------------------------------------------------------------------------
CAMPOS = {'t', 'x', 'y', 'z', 'rumbo_deg', 'cabeceo_deg', 'alabeo_deg', 'v', 'w', 'pendiente_deg', 'deslizamiento',
          'par_nm', 'potencia_w', 'energia_wh', 'atascado'}
METRICAS = {'distancia_m', 'tiempo_s', 'energia_wh', 'deslizamiento_medio', 'pendiente_max_deg', 'atascos', 'vuelcos',
            'meta_alcanzada', 'planeta', 'terreno', 'semilla'}


def _escena(semilla):
    return {'planeta': 'marte', 'terreno': 'rocoso', 'semilla': semilla, 'ruta': [[4, 0]], 'duracion_s': 8,
            'terreno_params': {'extension': 40.0}}


def test_cli_determinista_y_contrato(tmp_path):
    a, b, c = (str(tmp_path / n) for n in 'abc')
    cli.correr(_escena(4), a)
    cli.correr(_escena(4), b)
    cli.correr(_escena(5), c)
    for f in ('telemetry.jsonl', 'metrics.json'):
        assert open(f'{a}/{f}', 'rb').read() == open(f'{b}/{f}', 'rb').read(), f
    assert open(f'{a}/telemetry.jsonl', 'rb').read() != open(f'{c}/telemetry.jsonl', 'rb').read()
    filas = [json.loads(x) for x in open(f'{a}/telemetry.jsonl')]
    assert filas[1]['t'] - filas[0]['t'] == pytest.approx(0.1)           # 10 Hz de tiempo simulado
    assert set(filas[0]) == CAMPOS
    assert len(filas[0]['deslizamiento']) == 6 and len(filas[0]['par_nm']) == 6
    met = json.load(open(f'{a}/metrics.json'))
    assert set(met) == METRICAS and met['planeta'] == 'marte' and met['semilla'] == 4


def test_cli_argumentos_pisan_la_escena(tmp_path, capsys):
    assert cli.main(['--salida', str(tmp_path), '--id', 'x', '--planeta=luna', '--duracion_s=2',
                     '--terreno_params={"extension": 30.0}']) == 0
    met = json.load(open(tmp_path / 'x' / 'metrics.json'))
    assert met['planeta'] == 'luna' and met['tiempo_s'] == pytest.approx(2.0)
