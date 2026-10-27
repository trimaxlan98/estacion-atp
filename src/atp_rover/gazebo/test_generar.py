"""Pruebas del generador de mundos de Gazebo del rover (R3). Sin Gazebo ni ROS: solo geometria y XML.
    python3 -m pytest src/atp_rover/gazebo/test_generar.py
"""
import filecmp
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
import estado_gz  # noqa: E402
import generar  # noqa: E402


def test_terreno_determinista_y_origen_a_cero():
    for tipo in generar.TERRENOS:
        a = generar.altura(tipo, 5, n=65)
        b = generar.altura(tipo, 5, n=65)
        assert np.array_equal(a, b), tipo
        assert a[32, 32] == 0.0, tipo
        # llano alrededor del origen (el rover nace ahi)
        assert np.abs(a[30:35, 30:35]).max() < 0.05, tipo
    assert not np.array_equal(generar.altura('rocoso', 1, 65), generar.altura('rocoso', 2, 65))


def test_tipos_con_el_relieve_que_dicen():
    rango = lambda t: np.ptp(generar.altura(t, 1))   # noqa: E731
    assert rango('llano') < 0.1
    assert rango('rocoso') > 1.2 and rango('dunas') > 0.8
    cr = generar.altura('crater', 1)
    # centro del crater en (13, 4) m: hundido
    j, i = int(round((13 + 30) / 60 * 256)), int(round((4 + 30) / 60 * 256))
    assert cr[i, j] < -1.5
    pe = generar.altura('pendiente', 1)
    gx = np.gradient(pe, 60 / 256, axis=1)
    assert abs(math.degrees(math.atan(gx[128, 200])) - 14) < 1.5
    with pytest.raises(ValueError):
        generar.altura('hielo', 1)


@pytest.mark.parametrize('planeta,g', [('luna', 1.62), ('marte', 3.71), ('titan', 1.35), ('ceres', 0.28), ('tierra', 9.81)])
def test_mundo_por_planeta(tmp_path, planeta, g):
    r = generar.escribir(str(tmp_path), planeta, 'rocoso', 1, 'diffdrive', 'file:///ws/x')
    raiz = ET.parse(r['mundo']).getroot()
    mundo = raiz.find('world')
    assert mundo.get('name') == f'rover_{planeta}'
    assert [float(v) for v in mundo.find('gravity').text.split()] == [0, 0, -g]
    assert mundo.find('scene/background').text.strip() == generar.lineal(generar.PLANETAS[planeta]['cielo'])
    for f in ('terreno.png', 'color.png', 'normal.png', 'rover.sdf', 'gui.config', 'origen.json'):
        assert os.path.exists(tmp_path / f), f
    # el heightmap de la colision y el de la vista apuntan a la misma imagen; cuadrado 2^n + 1
    assert generar.N == 257
    uris = {e.text for e in mundo.iter('uri')}
    assert 'file:///ws/x/terreno.png' in uris


def test_cielos_y_suelos_distintos():
    assert len({p['cielo'] for p in generar.PLANETAS.values()}) == 5
    assert len({p['suelo'] for p in generar.PLANETAS.values()}) == 5


def test_misma_semilla_mismos_bytes(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    generar.escribir(str(a), 'luna', 'dunas', 3, uri='file:///ws/x')
    generar.escribir(str(b), 'luna', 'dunas', 3, uri='file:///ws/x')
    for f in ('terreno.png', 'color.png', 'mundo.sdf', 'rover.sdf'):
        assert filecmp.cmp(a / f, b / f, shallow=False), f


def test_rover_seis_ruedas_rocker_bogie():
    m = ET.fromstring(generar.sdf_rover('diffdrive'))
    enlaces = {e.get('name') for e in m.findall('link')}
    assert len(enlaces) == 1 + 2 + 2 + 6
    juntas = {j.get('name'): j for j in m.findall('joint')}
    ruedas = [n for n in juntas if n.startswith('rueda_')]
    assert len(ruedas) == 6
    for lado in 'id':
        assert juntas[f'rocker_{lado}_j'].find('parent').text == 'chasis'
        assert juntas[f'bogie_{lado}_j'].find('parent').text == f'rocker_{lado}'
        assert juntas[f'rueda_{lado}d_j'].find('parent').text == f'rocker_{lado}'   # delantera en el balancin
        assert juntas[f'rueda_{lado}m_j'].find('parent').text == f'bogie_{lado}'
        assert juntas[f'rueda_{lado}t_j'].find('parent').text == f'bogie_{lado}'
    for n, j in juntas.items():
        assert j.get('type') == 'revolute', n          # DART no soporta `continuous`
    plug = m.find("plugin[@name='gz::sim::systems::DiffDrive']")
    assert len(plug.findall('left_joint')) == 3 and len(plug.findall('right_joint')) == 3
    assert plug.find('topic').text == '/rover/cmd_vel'
    # simetria izquierda/derecha
    lp = {e.get('name'): e.find('pose').text for e in m.findall('link') if e.find('pose') is not None}
    for k in ('d', 'm', 't'):
        a, b = lp[f'rueda_i{k}'].split(), lp[f'rueda_d{k}'].split()
        assert float(a[1]) == -float(b[1]) and a[0] == b[0] and a[2] == b[2]


def test_modo_estado_es_estatico_y_sin_plugins_de_conduccion():
    m = ET.fromstring(generar.sdf_rover('estado'))
    assert m.find('static').text == 'true'
    assert m.find('plugin') is None


def test_estado_a_pose():
    p, q = estado_gz.pose_de_estado({'x': 1, 'y': 2, 'z': 0.7, 'rumbo_deg': 90}, z0=0.5)
    assert p == (1, 2, pytest.approx(1.2))
    assert q == pytest.approx((math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)))
    assert estado_gz.pose_de_estado({'t': 0}) is None
    # morro arriba (cabeceo > 0): el eje x del rover apunta hacia +z
    _, (w, x, y, z) = estado_gz.pose_de_estado({'x': 0, 'y': 0, 'cabeceo_deg': 30})
    ex_z = 2 * (x * z - w * y)           # componente z de R * (1,0,0)
    assert ex_z == pytest.approx(math.sin(math.radians(30)))
    # alabeo > 0: lado derecho (-y) abajo -> el eje y del rover sube
    _, (w, x, y, z) = estado_gz.pose_de_estado({'x': 0, 'y': 0, 'alabeo_deg': 20})
    ey_z = 2 * (y * z + w * x)
    assert ey_z == pytest.approx(math.sin(math.radians(20)))
