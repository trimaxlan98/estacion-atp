"""Pruebas rapidas del equipo R5 (validacion cientifica del rover): invariancias, convergencia en dt y la maquinaria de
sensibilidad de `scripts/validacion_rover.py`. Sin ROS. ~1 min."""
import math
import os
import sys

import numpy as np
import pytest

_RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_RAIZ, 'scripts'))
import validacion_rover as V  # noqa: E402


def test_carga_estatica_es_m_g_en_los_cinco_cuerpos():
    for pl, d in V.inv_g_estatica().items():
        assert abs(d['err_rel']) < 1e-9, pl


def test_simetria_izquierda_derecha_exacta():
    s = V.inv_simetria(T=3.0)
    assert s['llano_recto']['max_abs_y_m'] < 1e-9
    for k, v in s.items():
        if k.startswith('espejo'):
            assert v['max_err_y_m'] < 1e-9 and v['max_err_rumbo_deg'] < 1e-9 and v['max_err_alabeo_deg'] < 1e-9, k
            assert v['recorrido_x_m'] > 0.5                                  # la trayectoria no es trivial


def test_energia_sin_traccion_se_conserva_y_con_perdidas_no_sube():
    e = V.inv_energia(T=2.0)
    assert e['sin_traccion_sin_amortiguador']['deriva_rel_a_PE_liberada'] < 1e-6
    assert e['sin_traccion_amortiguador_real']['mayor_subida_por_paso_J'] < 1e-6
    assert e['inercia_suelo_real_motores_apagados']['subida_total_max_J'] < 1e-6      # el suelo es pasivo


def test_similitud_dinamica_de_g():
    d = V.inv_similitud(4.0, T0=4.0)
    assert d['err_pos_max_rel'] < 5e-3
    assert d['err_rumbo_max_deg'] < 0.05
    assert d['err_desl'] < 1e-3


def test_convergencia_dt_en_marte_llano():
    base = dict(planeta='marte', T=3.0, settle=0.6)
    ref = V.simular(dict(base, dt=0.005))
    for dt, tol in ((0.01, V.TOL_REL), (0.02, 3 * V.TOL_REL)):
        r = V.simular(dict(base, dt=dt))
        assert abs(r['dist_m'] - ref['dist_m']) / ref['dist_m'] < tol, dt
        assert abs(r['wh_m'] - ref['wh_m']) / ref['wh_m'] < tol, dt
        assert abs(r['desl'] - ref['desl']) < V.TOL_DESL, dt


def test_muestreo_sobol_determinista_y_en_rango():
    a, b = V.disenio_saltelli(6, 8), V.disenio_saltelli(6, 8)
    assert all(np.array_equal(x, y) for x, y in zip(a, b)) and len(a) == 8
    assert all(0.0 <= m.min() and m.max() <= 1.0 for m in a)
    suelo, g = V._mapear(np.full(7, 0.5), 'marte', con_g=True)
    assert suelo['phi_deg'] == pytest.approx(33.0) and suelo['c'] == pytest.approx(800.0)
    assert V.G_RANGO[0] < g < V.G_RANGO[1]
    lo, _ = V._mapear(np.zeros(6), 'titan')
    hi, _ = V._mapear(np.ones(6), 'titan')
    assert lo['phi_deg'] == pytest.approx(26.0 - 8.0) and hi['phi_deg'] == pytest.approx(26.0 + 8.0)


def test_indices_de_sobol_funcion_de_ishigami():
    """Ishigami (a = 7, b = 0.1): S1 = (0.3139, 0.4424, 0), ST = (0.5576, 0.4424, 0.2437)."""
    k, N = 3, 4096
    mats = V.disenio_saltelli(k, N)

    def f(u):
        x = (u * 2 - 1) * math.pi
        return np.sin(x[:, 0]) + 7 * np.sin(x[:, 1]) ** 2 + 0.1 * x[:, 2] ** 4 * np.sin(x[:, 0])
    ind = V.indices_sobol(f(mats[0]), f(mats[1]), [f(m) for m in mats[2:]], n_boot=20)
    assert ind['S1'] == pytest.approx([0.3139, 0.4424, 0.0], abs=0.05)
    assert ind['ST'] == pytest.approx([0.5576, 0.4424, 0.2437], abs=0.05)


@pytest.mark.xfail(reason='DEFECTO REPORTADO (R5) y corregido como opcion en R6: con el defecto de R1/R4 (rodadura_en_motor=False) el motor no paga la '
                          'compactacion; es la fisica historica (rodadura_en_motor=False), ya no el defecto.', strict=True)
def test_potencia_de_traccion_defecto_historico():
    d = V.inv_potencia_compactacion(('marte', 'tierra'), rodadura_en_motor=False)
    for pl in ('marte', 'tierra'):
        assert d[pl]['razon'] > 0.5, pl


def test_potencia_de_traccion_cubre_la_compactacion():
    d = V.inv_potencia_compactacion(('marte', 'tierra'), rodadura_en_motor=True)
    for pl in ('marte', 'tierra'):
        assert d[pl]['razon'] > 0.5, pl
