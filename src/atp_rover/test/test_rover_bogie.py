"""Pruebas de R6 sobre el rover: suspension rocker-bogie real, contacto por envolvente, rocas y escalones, margen de vuelco y regresion
de bytes de la suspension `simple` (el defecto). Cada prueba larga dura unos segundos; las de bytes son cortas (8 a 14 s simulados)."""
import hashlib
import json
import math
import re

import numpy as np
import pytest

from atp_rover import cli
from atp_rover.planetas import planeta
from atp_rover.rover import DT, ParametrosRover, Rover
from atp_rover.terreno import Terreno, distribucion_rocas

RB = {'suspension': 'rocker_bogie'}


def _rover(pl_nombre='marte', tipo='llano', susp=None, x=0.0, rumbo=0.0, suelo=None, **tp):
    pl = planeta(pl_nombre)
    if suelo:
        pl['suelo'] = {**pl['suelo'], **suelo}
    ter = Terreno(tipo, 1, pl['suelo'], **tp)
    return Rover(pl, ter, susp or {}, x=x, rumbo_deg=rumbo), ter


# ---- regresion: la suspension `simple` no cambia ni un byte -----------------------------------------------------------------------
_CERO_NEGATIVO = re.compile(rb'(?<![\d.])-0\.0(?![\d.eE])')


def _huella(ruta):
    """sha256 (16 hex) del archivo con el cero negativo escrito como `0.0`. Segun la ruta SIMD de NumPy (con o sin AVX-512) un resultado
    exactamente cero sale con uno u otro signo, y ese es el UNICO byte que cambia entre CPUs (los valores son identicos): sin esta
    normalizacion las huellas dependian de la CPU en que se calcularon."""
    return hashlib.sha256(_CERO_NEGATIVO.sub(b'0.0', ruta.read_bytes())).hexdigest()[:16]


# Huellas del paquete de R1/R4 (commit 5a342f0), antes de tocar rover.py, terreno.py y lazo.py. Recalculadas el 2026-10-07 con `_huella`
# (las originales eran del texto crudo en otra CPU) ejecutando el codigo de R4 en un Xeon W-2235; el codigo actual da lo mismo, con
# y sin AVX-512. Los metrics.json no cambian: no llevan ceros negativos.
HUELLAS = {
    'A': ({'planeta': 'marte', 'terreno': 'dunas', 'semilla': 3, 'ruta': [[3, 0]], 'duracion_s': 10, 'v_max': 0.5},
          '6f6ad7211400346b', '8def3544dff17ae0'),
    'B': ({'planeta': 'luna', 'terreno': 'rocoso', 'semilla': 7, 'ruta': [[3, 0], [3, 3]], 'duracion_s': 14, 'v_max': 0.5},
          'b4d704892c7db1e5', '6d4c6fec3b087c8b'),
}


@pytest.mark.parametrize('caso', sorted(HUELLAS))
def test_simple_conserva_los_bytes_de_r1(caso, tmp_path):
    cfg, h_tel, h_met = HUELLAS[caso]
    HIST = {'rodadura_en_motor': False}      # las huellas son de la fisica historica; el defecto actual es la corregida
    for extra in ({}, {'suspension': 'simple'}, {'rover': {'suspension': 'simple', 'contacto': 'punto'}}):
        cli.correr({**cfg, 'rover': HIST, **{k: v for k, v in extra.items() if k != 'rover'}}, str(tmp_path / 'x')) if 'rover' not in extra \
            else cli.correr({**cfg, 'rover': {**HIST, **extra['rover']}}, str(tmp_path / 'x'))
        h = lambda n: _huella(tmp_path / 'x' / n)
        assert (h('telemetry.jsonl'), h('metrics.json')) == (h_tel, h_met), extra
    # sin campos opcionales en el estado ni en las metricas
    fila = json.loads((tmp_path / 'x' / 'telemetry.jsonl').read_text().splitlines()[0])
    met = json.loads((tmp_path / 'x' / 'metrics.json').read_text())
    for k in ('balancin_deg', 'bogie_deg', 'carga_n', 'margen_vuelco_deg'):
        assert k not in fila
    assert 'suspension' not in met and 'margen_vuelco_min_deg' not in met


def test_completa_r4_reproduce_la_navegacion_de_r4(tmp_path):
    # huella de la telemetria de la navegacion `completa` de R4 (commit 5a342f0) en esta corrida corta
    cfg = {'planeta': 'marte', 'terreno': 'pendiente', 'semilla': 1, 'terreno_params': {'angulo_deg': 10}, 'ruta': [[5, 0]],
           'duracion_s': 14, 'v_max': 0.5, 'navegacion': 'completa_r4', 'rover': {'rodadura_en_motor': False}}
    cli.correr(cfg, str(tmp_path / 'x'))
    assert _huella(tmp_path / 'x' / 'telemetry.jsonl') == '8a8b7254a83d9277'                  # recalculada con `_huella`, ver arriba


def test_suspension_y_contacto_desconocidos():
    with pytest.raises(ValueError):
        ParametrosRover.desde_dict({'suspension': 'magica'})
    with pytest.raises(ValueError):
        ParametrosRover.desde_dict({'contacto': 'magico'})


# ---- cinematica -------------------------------------------------------------------------------------------------------------------
def test_cinematica_reposo_igual_a_los_cubos_simples_y_jacobiano():
    r, _ = _rover(susp=RB)
    hubs, J = r._hubs_rb(np.zeros(3))
    assert np.allclose(hubs, r.hubs, atol=1e-12)
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = rng.uniform(-0.3, 0.3, 3)
        _, J = r._hubs_rb(q)
        for k in range(3):
            e = np.zeros(3)
            e[k] = 1e-6
            num = (r._hubs_rb(q + e)[0] - r._hubs_rb(q - e)[0]) / 2e-6
            assert np.allclose(J[:, :, k], num, atol=1e-7)


def test_barra_diferencial_balancines_opuestos():
    r, _ = _rover(susp=RB)
    h, _ = r._hubs_rb(np.array([0.2, 0.0, 0.0]))
    # delta > 0: la rueda delantera izquierda baja y la derecha sube lo mismo (los balancines se mueven al reves)
    assert h[0, 2] - r.hubs[0, 2] == pytest.approx(-(h[3, 2] - r.hubs[3, 2]), rel=0.12)      # casi simetrico (el brazo tiene componente vertical)
    assert h[0, 2] < r.hubs[0, 2] and h[3, 2] > r.hubs[3, 2]
    # la rueda media esta bajo el pivote del balancin: casi no se mueve (efecto de segundo orden) y la trasera hace lo contrario de la delantera
    assert abs(h[1, 2] - r.hubs[1, 2]) < 0.1 * abs(h[0, 2] - r.hubs[0, 2])
    assert h[2, 2] > r.hubs[2, 2] and h[5, 2] < r.hubs[5, 2]


# ---- reparto de carga -------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('pl', ['luna', 'marte', 'tierra'])
def test_reparto_de_carga_en_llano_es_un_sexto_por_rueda(pl):
    r, _ = _rover(pl, susp=RB)
    r.asentar(3.0)
    e = r.estado()
    esperado = r.p.masa * r.g / 6.0
    assert np.allclose(e['carga_n'], esperado, rtol=0.02)
    assert np.allclose(e['balancin_deg'], 0.0, atol=0.05) and np.allclose(e['bogie_deg'], 0.0, atol=0.05)


def test_suma_de_cargas_es_el_peso_normal_en_pendiente():
    r, _ = _rover('marte', 'pendiente', RB, angulo_deg=15.0)
    r.asentar(3.0)
    e = r.estado()
    assert sum(e['carga_n']) == pytest.approx(r.p.masa * r.g * math.cos(math.radians(15.0)), rel=0.03)
    # el bogie y el balancin se acomodan: los dos lados cargan igual y la rueda trasera (cuesta abajo) lleva mas que la delantera
    assert e['carga_n'][0] == pytest.approx(e['carga_n'][3], rel=0.02)
    assert e['carga_n'][2] > e['carga_n'][0]


def test_rueda_sobre_una_roca_el_balancin_se_mueve_y_las_demas_siguen_cargadas():
    # un escalon de 0.12 m bajo las ruedas delanteras: el balancin las sube sin que ninguna rueda deje de apoyar
    r, _ = _rover('marte', 'escalones', RB, altura=0.12, x_inicio=0.0 + 0.8)
    r.asentar(2.0)
    for _ in range(int(2.0 / DT)):
        r.paso(0.3, 0.0)
    e = r.estado()
    assert min(e['carga_n']) > 0.0
    assert max(abs(e['balancin_deg'][0]), abs(e['bogie_deg'][0]), abs(e['bogie_deg'][1])) > 1.0      # las articulaciones trabajaron
    assert e['z'] > 0.3


# ---- escalones y rocas ------------------------------------------------------------------------------------------------------------
def _sube(pl, h, susp, dur=45.0):
    r, _ = _rover(pl, 'escalones', susp, altura=h, x_inicio=3.0)
    r.asentar(2.0)
    for _ in range(int(dur / DT)):
        r.paso(0.5, 0.0)
        if r.s[0] > 5.5 or r.vuelco:
            break
    return r.s[0] > 5.5 and not r.vuelco


def test_escalon_bajo_se_supera_y_uno_mucho_mas_alto_que_la_rueda_no():
    assert _sube('marte', 0.10, RB)
    assert not _sube('marte', 0.40, RB, dur=30.0)       # mas alto que el radio de la rueda (0.25 m) y que su diametro menos...


def test_altura_maxima_de_escalon_baja_con_la_gravedad():
    # medido (runs: ver docs/ROVER.md «Resultados R6»): con 0.5 m/s mandado la Luna supera 0.20 m y la Tierra no supera 0.15 m
    assert _sube('luna', 0.20, RB)
    assert not _sube('tierra', 0.15, RB, dur=30.0)


def test_terreno_escalones_y_pendiente_lateral():
    t = Terreno('escalones', 0, planeta('luna')['suelo'], altura=0.2, x_inicio=3.0, n=2, separacion=4.0)
    assert t.altura(2.0, 0.0) == 0.0 and t.altura(3.5, 0.0) == pytest.approx(0.2) and t.altura(8.0, 0.0) == pytest.approx(0.4)
    p = Terreno('pendiente', 0, planeta('luna')['suelo'], angulo_deg=20.0, direccion_deg=90.0)
    assert float(p.pendiente_deg(1.0, 1.0)) == pytest.approx(20.0, abs=0.1)
    assert abs(p.normal(1.0, 1.0)[0]) < 1e-9                       # la pendiente sube hacia +y, no hacia +x
    q = Terreno('pendiente', 0, planeta('luna')['suelo'], angulo_deg=20.0)
    assert q.bytes_huella() == Terreno('pendiente', 0, planeta('luna')['suelo'], angulo_deg=20.0, direccion_deg=0.0).bytes_huella()


def test_rocas_deterministas_con_distribucion_de_golombek():
    s = planeta('marte')['suelo']
    a = Terreno('rocas', 4, s, abundancia=0.10, radio_rocas=20.0, extension=60.0)
    b = Terreno('rocas', 4, s, abundancia=0.10, radio_rocas=20.0, extension=60.0)
    c = Terreno('rocas', 5, s, abundancia=0.10, radio_rocas=20.0, extension=60.0)
    assert a.bytes_huella() == b.bytes_huella() and a.bytes_huella() != c.bytes_huella()
    assert a.n_rocas > 500 and 0.10 < a.altura_roca_max < 0.6
    # zona de arranque libre de rocas
    X = np.linspace(-2.0, 2.0, 21)
    assert np.abs(a.altura(X[:, None], X[None, :])).max() < 0.05
    # la densidad numerica integrada (rocas por m2) crece al bajar la abundancia total k menos que proporcional... y sigue la ley F(D)=k exp(-qD)
    D, dens, tot = distribucion_rocas(0.10, 0.15, 0.80)
    q = 1.79 + 0.152 / 0.10
    area_cubierta = np.sum(0.5 * (dens[1:] * D[1:] ** 2 + dens[:-1] * D[:-1] ** 2) * np.diff(D)) * math.pi / 4.0
    assert area_cubierta == pytest.approx(0.10 * (math.exp(-q * 0.15) - math.exp(-q * 0.80)), rel=0.01)
    # la fraccion de area con rocas medida en el terreno es del orden de F(d_min) (menos: solapes y formas de semi-elipse)
    dentro = (np.abs(a._xs) <= 20.0)
    cub = (a.zona[np.ix_(dentro, dentro)] == 2).mean()
    assert 0.2 * 0.10 * math.exp(-q * 0.15) < cub < 1.2 * 0.10 * math.exp(-q * 0.15)


# ---- margen de vuelco y vuelco real -----------------------------------------------------------------------------------------------
def test_margen_de_vuelco_llano_y_pendiente_lateral():
    r, _ = _rover('tierra', susp=RB)
    r.asentar(2.0)
    assert r.estado()['margen_vuelco_deg'] == pytest.approx(45.0, abs=0.5)      # atan(0.40 / 0.40): el angulo estatico del modelo
    r2, _ = _rover('tierra', 'pendiente', RB, angulo_deg=20.0, direccion_deg=90.0, suelo=dict(phi_deg=60.0, c=3000.0))
    r2.asentar(2.0)
    e = r2.estado()
    assert e['margen_vuelco_deg'] == pytest.approx(45.0 - 20.0, abs=1.5)
    assert abs(e['alabeo_deg']) == pytest.approx(20.0, abs=1.5)
    assert 'margen_vuelco_min_deg' in r2.metricas()


def test_vuelco_real_en_pendiente_lateral_sobre_el_angulo_estatico():
    # suelo de mucho agarre para que no se deslice antes de volcar (como la prueba de R1)
    res = {}
    for ang in (30.0, 55.0):
        r, _ = _rover('tierra', 'pendiente', RB, angulo_deg=ang, direccion_deg=90.0, suelo=dict(phi_deg=70.0, c=5000.0))
        r.asentar(0.5)
        for _ in range(int(5.0 / DT)):
            r.paso(0.0, 0.0)
        res[ang] = (r.vuelco, r.estado()['margen_vuelco_deg'])
    assert not res[30.0][0] and res[30.0][1] > 5.0
    assert res[55.0][0]


# ---- rodadura: el motor vence la resistencia por compactacion (hallazgo de R5) ---------------------------------------------------------
def _energia_traccion(pl, flag):
    m, _ = cli.correr({'planeta': pl, 'terreno': 'llano', 'semilla': 1, 'ruta': [[8, 0]], 'duracion_s': 25, 'v_max': 0.4,
                       'rover': {'rodadura_en_motor': flag}})
    return m['energia_wh'] - 20.0 * m['tiempo_s'] / 3600.0, m['distancia_m']


def test_rodadura_en_motor_sube_la_energia_de_traccion_y_crece_con_g():
    e0, d0 = _energia_traccion('tierra', False)
    e1, d1 = _energia_traccion('tierra', True)
    el, dl = _energia_traccion('luna', True)
    assert e1 > 10 * e0                     # antes la resistencia por compactacion ayudaba a girar la rueda
    assert e1 / d1 > 3 * el / dl            # y ahora el costo de rodar sigue la gravedad
    assert ParametrosRover().rodadura_en_motor is True        # el defecto es la fisica corregida (ronda 8); False reproduce R1/R4
