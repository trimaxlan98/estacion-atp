"""Consistencia estadistica del ESKF (P7): NEES contra la banda chi-cuadrado de 95 % con 30 semillas.

Escenario: vuelo estacionario con rafagas (sigma 1 m/s) y viento de 2 m/s, 20 s, NEES cada 1 s desde t = 5 s. Con N = 30 corridas
independientes el promedio de conjunto de la NEES de un bloque de n gdl cae en [chi2(0.025; 30n)/30, chi2(0.975; 30n)/30] si el filtro
es consistente. La referencia clasica de la prueba (Bar-Shalom) NO esta verificada en esta sesion; la banda es estadistica elemental
y sus cuantiles se contrastan aqui con valores tabulados.
"""
import multiprocessing as mp

import numpy as np
import pytest

from atp_dron import consistencia as C

CFG = {'ruta': [[0, 0, 8]], 'duracion_s': 20.0, 'rafaga_sigma': 1.0, 'viento': [2.0, 0, 0], 'parar_al_llegar': False}
SEMILLAS = list(range(30))


def test_cuantiles_chi_cuadrado_coinciden_con_las_tablas():
    # valores de tabla estandar de chi-cuadrado
    assert C.chi2_ppf(0.975, 3) == pytest.approx(9.348, abs=1e-3)
    assert C.chi2_ppf(0.025, 3) == pytest.approx(0.216, abs=1e-3)
    assert C.chi2_ppf(0.975, 1) == pytest.approx(5.024, abs=1e-3)
    assert C.chi2_ppf(0.95, 10) == pytest.approx(18.307, abs=1e-3)
    assert C.chi2_cdf(3.0, 3) == pytest.approx(0.6084, abs=1e-3)
    lo, hi = C.banda(3, 30)
    assert lo < 3.0 < hi and hi - lo < 2.0


def _uno(a):
    sp, s = a
    _, acc, _ = C.nees_corrida({**CFG, 'semilla': s, 'sensores_params': sp}, 1.0)
    return acc


def _conjunto(sp):
    with mp.get_context('fork').Pool(min(4, mp.cpu_count())) as p:
        return C.resumen_nees(p.map(_uno, [(sp, s) for s in SEMILLAS]))


@pytest.fixture(scope='module')
def corregido():
    return _conjunto({'q_corregida': True, 'h_mag_exacto': True})


@pytest.fixture(scope='module')
def defecto():
    return _conjunto({})


def test_con_q_corregida_la_velocidad_es_consistente_y_la_actitud_no_es_optimista(corregido):
    r = corregido['vel']
    lo, hi = r['banda_instante']
    assert lo <= r['mediana_instante'] <= hi and r['fraccion_instantes_en_banda'] >= 0.9, r      # medido: 3.04, fraccion 1.0
    assert 0.8 < r['P_sobre_var_empirica'] < 1.25, r                                              # medido 0.97
    a = corregido['actitud']                      # medido 2.08 < 2.19: algo conservadora (P/var 1.4), nunca optimista
    assert a['nees_medio'] < a['banda_instante'][1] and a['P_sobre_var_empirica'] > 1.0, a
    assert corregido['pos']['nees_medio'] < corregido['pos']['banda_instante'][1] + 0.5          # medido 3.64


def test_q_por_defecto_es_conservadora_y_la_corregida_la_acerca(defecto, corregido):
    # control negativo: con Q x250 (la de siempre) la velocidad esta por DEBAJO de la banda (P sobredimensionada)
    lo, _ = defecto['vel']['banda_instante']
    assert defecto['vel']['nees_medio'] < lo and defecto['vel']['fraccion_instantes_en_banda'] == 0.0     # medido 1.67
    assert defecto['vel']['P_sobre_var_empirica'] > 1.5                                                    # medido 1.73
    assert corregido['vel']['nees_medio'] > 1.5 * defecto['vel']['nees_medio']
