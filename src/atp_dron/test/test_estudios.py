"""Pruebas de la ronda D2: efecto suelo, bateria y banco de experimentos contra formulas cerradas."""
import numpy as np
import pytest

from atp_dron import estudios as E
from atp_dron.dron import G0, Dron
from atp_dron.lazo import correr


def test_efecto_suelo_empuje_cheeseman_bennett():
    R = 0.127
    d = Dron('tierra', params={'efecto_suelo': True}, pos=(0, 0, 0.3))
    libre = Dron('tierra', pos=(0, 0, 0.3))
    for x in (d, libre):
        x.om[:] = 500.0
        x.paso(np.full(4, 2.0))
    a_ge, a_libre = d.v[2], libre.v[2]
    k = 1.0 / (1.0 - (R / (4 * 0.3)) ** 2)
    # la aceleracion neta sube con el empuje (arrastre y g iguales): (k T - W) > (T - W)
    assert a_ge > a_libre
    assert (a_ge * d.m + d.m * d.g) / (a_libre * libre.m + libre.m * libre.g) == pytest.approx(k, rel=0.02)


def test_efecto_suelo_apagado_no_cambia_bytes():
    c = {'ruta': [[5, 0, 4]], 'duracion_s': 8.0}
    m1, f1 = correr(c)
    m2, f2 = correr({**c, 'dron': {'efecto_suelo': False}})
    assert m1 == m2 and f1 == f2


def test_bateria_se_agota_y_corta_la_corrida():
    m, _ = correr({'ruta': [[0, 0, 5]], 'duracion_s': 60.0, 'parar_al_llegar': False, 'dron': {'bateria_wh': 0.5}})
    assert m['bateria_agotada'] and m['soc_final'] == 0.0 and m['tiempo_s'] < 20.0
    m2, _ = correr({'ruta': [[0, 0, 5]], 'duracion_s': 10.0, 'parar_al_llegar': False, 'dron': {'bateria_wh': 100.0}})
    assert not m2['bateria_agotada'] and 0.9 < m2['soc_final'] < 1.0


def test_empuje_peso_analitico_y_rotor_minimo():
    assert E.empuje_peso('tierra') == pytest.approx(2.2)
    assert E.empuje_peso('marte') == pytest.approx(2.2 * (0.020 / 1.225) * (G0 / 3.71))
    esc = E.rotor_escala_minima('marte', 1.3)
    assert E.empuje_peso('marte', rotor_escala=esc) == pytest.approx(1.3)
    assert E.empuje_peso('tierra', masa_kg=3.0) == pytest.approx(1.1)


def test_potencia_hover_coincide_con_la_simulada():
    m, _ = correr({'ruta': [[0, 0, 5]], 'duracion_s': 60.0, 'parar_al_llegar': False})
    p = E.potencia_hover_w('tierra')
    assert 170 < p < 195
    # energia media de la corrida ~ hover + el ascenso inicial: dentro de 15 %
    assert m['energia_Wh'] * 3600 / 60.0 == pytest.approx(p, rel=0.15)


def test_limite_de_viento_por_inclinacion_maxima():
    # a la inclinacion maxima, m g tan(35) = 1/2 rho CdA v^2  ->  v_max ~ 23.7 m/s (1.5 kg, CdA 0.03)
    v_max = np.sqrt(2 * 1.5 * G0 * np.tan(np.radians(35.0)) / (1.225 * 0.03))
    assert 23.0 < v_max < 24.5
    f = E.barrido_viento(velocidades=(v_max - 5, v_max + 3), duracion_s=30.0)
    assert f[0]['desviacion_max_m'] < 1.0 and f[0]['inclinacion_max_deg'] < 35.5
    assert f[1]['desviacion_max_m'] > 5.0          # no puede sostener la posicion: deriva


def test_barrido_carga_monotono():
    f = E.barrido_carga()
    p = [r['p_hover_w'] for r in f]
    a = [r['autonomia_min'] for r in f]
    assert p == sorted(p) and a == sorted(a, reverse=True)
    assert f[0]['empuje_peso'] == pytest.approx(2.2)


def test_montecarlo_reproducible():
    a = E.montecarlo_rafagas(n=2, ruta=((10, 0, 5),), semilla0=3)
    b = E.montecarlo_rafagas(n=2, ruta=((10, 0, 5),), semilla0=3)
    assert a == b and a['llegaron'] == 2
