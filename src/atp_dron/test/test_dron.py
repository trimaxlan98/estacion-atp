"""Pruebas del cuadrirrotor: fisica contra formulas cerradas, control y determinismo."""
import json

import numpy as np
import pytest

from atp_dron.control import Cascada
from atp_dron.dron import DT, G0, Dron, euler, rot
from atp_dron.lazo import correr


def test_caida_libre_con_g_del_cuerpo():
    d = Dron('marte', pos=(0, 0, 100.0))
    for _ in range(int(round(2.0 / DT))):
        d.paso(np.zeros(4))
    # sin empuje y con poco arrastre a baja velocidad en Marte: v = -g t (arrastre < 1 %)
    assert d.v[2] == pytest.approx(-3.71 * 2.0, rel=0.01)


def test_hover_en_equilibrio_empuje_igual_peso():
    d = Dron('tierra', pos=(0, 0, 10.0))
    d.om[:] = d.empuje_a_om(np.full(4, d.m * G0 / 4))
    T = np.full(4, d.m * G0 / 4)
    for _ in range(int(round(5.0 / DT))):
        d.paso(T)
    assert np.linalg.norm(d.v) < 1e-6 and d.p[2] == pytest.approx(10.0, abs=1e-6)
    assert np.allclose(d.w, 0.0, atol=1e-9)


def test_guinada_por_par_de_reaccion():
    d = Dron('tierra', pos=(0, 0, 10.0))
    d.om[:] = d.empuje_a_om(np.full(4, d.m * G0 / 4))
    T = np.full(4, d.m * G0 / 4) + np.array([0.3, -0.3, 0.3, -0.3])    # rotores de giro +1 mas fuertes
    for _ in range(50):
        d.paso(T)
    assert d.w[2] < -0.01 and abs(d.w[0]) < 1e-6 and abs(d.w[1]) < 1e-6


def test_energia_hover_orden_de_magnitud():
    # 1.5 kg, 4 helices de 5", FoM 0.6, eta 0.8: P = 4 T^1.5/(sqrt(2 rho A) FoM eta) + 15 W ~ 210 W (a mano)
    m, _ = correr({'ruta': [[0, 0, 5]], 'duracion_s': 30.0, 'parar_al_llegar': False})
    T = 1.5 * G0 / 4
    A = np.pi * 0.127 ** 2
    p = 4 * T ** 1.5 / (np.sqrt(2 * 1.225 * A) * 0.6 * 0.8) + 15
    assert m['energia_Wh'] == pytest.approx(p * 28.0 / 3600, rel=0.25)   # ~2 s de ascenso a mas potencia


def test_llega_a_la_meta_y_se_queda():
    m, f = correr({'ruta': [[10, 5, 8]], 'duracion_s': 30.0, 'parar_al_llegar': False})
    assert m['llego'] and not m['choque']
    assert f[-1]['x'] == pytest.approx(10, abs=0.2) and f[-1]['z'] == pytest.approx(8, abs=0.2)


def test_viento_constante_se_rechaza_con_integral():
    m, f = correr({'ruta': [[0, 0, 5]], 'duracion_s': 40.0, 'parar_al_llegar': False, 'viento': [4.0, 0, 0]})
    assert abs(f[-1]['x']) < 0.2 and m['inclinacion_max_deg'] < 20


def test_rafagas_reproducibles_y_dependen_de_la_semilla():
    kw = dict(ruta=[[20, 0, 6]], duracion_s=25.0, rafaga_sigma=1.5)
    a, _ = correr({**kw, 'semilla': 3})
    b, _ = correr({**kw, 'semilla': 3})
    c, _ = correr({**kw, 'semilla': 4})
    assert a == b and a != c


def test_recupera_de_una_inclinacion_grande():
    d = Dron('tierra', pos=(0, 0, 20.0))
    d.q = np.array([np.cos(0.4), np.sin(0.4), 0.0, 0.0])    # alabeo de ~46 grados
    ctl = Cascada(d)
    for _ in range(int(round(6.0 / DT))):
        d.paso(ctl.mando(ctl.velocidad_deseada([0, 0, 20.0])))
    assert abs(euler(d.q)[0]) < np.radians(1.0) and d.p[2] > 10


def test_marte_no_vuela_con_rotores_terrestres_y_si_con_rotores_grandes():
    chico, _ = correr({'cuerpo': 'marte', 'ruta': [[0, 0, 5]], 'duracion_s': 10.0})
    assert not chico['vuelo_posible'] and not chico['llego']
    grande, _ = correr({'cuerpo': 'marte', 'ruta': [[0, 0, 5]], 'duracion_s': 15.0, 'dron': {'rotor_escala': 25.0}})
    assert grande['vuelo_posible'] and grande['llego']


def test_choque_por_impacto():
    d = Dron('tierra', pos=(0, 0, 30.0))
    for _ in range(int(round(4.0 / DT))):
        d.paso(np.zeros(4))
    assert d.choque


def test_cli_determinista_y_contrato(tmp_path):
    kw = dict(ruta=[[10, 0, 5], [10, 10, 5]], duracion_s=30.0, rafaga_sigma=0.8, semilla=2)
    correr(kw, str(tmp_path / 'a'))
    correr(kw, str(tmp_path / 'b'))
    for n in ('telemetry.jsonl', 'metrics.json'):
        assert (tmp_path / 'a' / n).read_bytes() == (tmp_path / 'b' / n).read_bytes()
    met = json.loads((tmp_path / 'a' / 'metrics.json').read_text())
    assert met['llego'] and 0 < met['energia_Wh'] < 5
    assert rot(np.array([1.0, 0, 0, 0])).tolist() == np.eye(3).tolist()
