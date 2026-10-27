"""Escenario completo sin ROS: montura de atp_ros + FSM + atmosfera + enlace; determinismo; diversidad."""
import dataclasses
import datetime as dt
import json
import math

import numpy as np
import pytest

from atp_optico import atmosfera as A
from atp_optico import diversidad as D
from atp_optico import enlace as L
from atp_optico import escenario as E
from atp_optico import montura as M
from atp_optico import nubes as N
from atp_ros import model, orbit

CFG = E.ConfigOptico()


@pytest.fixture(scope='module')
def perfil_corto():
    """Pase circular sintetico de 550 km que culmina a 55 deg (cielo sin TLE): los primeros 30 s."""
    p = model.perfil_pase(550.0, 55.0, 10.0, 140.0, n=2400)
    k = int(np.searchsorted(p['t'], 30.0))
    return {c: (v[:k] if isinstance(v, np.ndarray) else v) for c, v in p.items()} | {'duracion': float(p['t'][k - 1])}


# ------------------------------------------------------------------ la montura

def test_montura_sigue_un_pase_con_error_pequeno(perfil_corto):
    m = M.simular_montura(perfil_corto)
    assert len(m["t"]) > 500
    rms = float(np.sqrt(np.mean(m['e_pt_deg'] ** 2)))
    assert 1e-5 < rms < 0.05                       # grados: la montura de atp_ros sigue un pase, ni exacta ni perdida
    # una montura con un perfil mas rapido (menos elevacion maxima) tiene mas error
    assert m['e_pt_deg'].max() < 0.2


def test_montura_determinista(perfil_corto):
    a = M.simular_montura(perfil_corto)
    b = M.simular_montura(perfil_corto)
    for k in a:
        assert np.array_equal(a[k], b[k])


def test_montura_viento_aumenta_el_error(perfil_corto):
    base = M.simular_montura(perfil_corto)
    viento = M.simular_montura(perfil_corto, M.ConfigMontura(wind_sigma_nm=0.5))
    assert np.sqrt(np.mean(viento['e_pt_deg'] ** 2)) > np.sqrt(np.mean(base['e_pt_deg'] ** 2))


# ------------------------------------------------------------------ el escenario

def test_escenario_corto_con_fsm_cierra_y_sin_fsm_no(perfil_corto):
    med, filas, meta = E.simular(CFG, perfil=perfil_corto)
    r = med.resumen_dict()
    assert r['enganchado_pct'] == 100.0
    assert r['rms_con_fsm_urad'] < 2.0                            # por eje; el campo es de 12 urad
    assert r['rms_sin_fsm_urad'] > 20.0 * r['rms_con_fsm_urad']
    assert r['con_fsm']['gb'] > 3.0 * r['sin_fsm']['gb']
    assert r['rms_sobre_campo'] < 0.2
    assert len(med.resumen()) == 3


def test_escenario_determinista_byte_a_byte(perfil_corto, tmp_path):
    for i in (1, 2):
        med, filas, _ = E.simular(dataclasses.replace(CFG), perfil=perfil_corto)
        E.escribir_salida(str(tmp_path / str(i)), med, filas)
    for f in ('optico.json', 'optico_serie.jsonl'):
        assert (tmp_path / '1' / f).read_bytes() == (tmp_path / '2' / f).read_bytes()
    assert json.loads((tmp_path / '1' / 'optico.json').read_text())['n_pasos'] == len(filas)
    # otra semilla cambia el tilt (y con el la serie)
    med3, filas3, _ = E.simular(dataclasses.replace(CFG, semilla=99), perfil=perfil_corto)
    E.escribir_salida(str(tmp_path / '3'), med3, filas3)
    assert (tmp_path / '3' / 'optico_serie.jsonl').read_bytes() != (tmp_path / '1' / 'optico_serie.jsonl').read_bytes()


def test_un_campo_mas_estrecho_necesita_mas_precision(perfil_corto):
    ancho = E.simular(CFG, perfil=perfil_corto)[0].resumen_dict()
    estrecho = E.simular(dataclasses.replace(CFG, campo_urad=3.0), perfil=perfil_corto)[0].resumen_dict()
    assert estrecho['rms_sobre_campo'] > ancho['rms_sobre_campo']
    assert estrecho['con_fsm']['gb'] <= ancho['con_fsm']['gb'] + 1e-9


# ------------------------------------------------------------------ analisis casi estatico

def test_curva_elevacion_y_subida():
    sat, est, atm, _ = E.construir(CFG)
    f = E.curva_elevacion(sat, est, atm, 550.0, (10, 30, 60, 90), 0.6)
    assert [x['tasa_bps'] for x in f] == sorted(x['tasa_bps'] for x in f)
    g = E.curva_elevacion(sat, est, atm, 550.0, (10, 30, 90), 0.6, subida=True)
    assert all(x['p_rx_dbm'] < 0.0 for x in g)
    assert g[0]['presupuesto']['beam_wander_urad'] > g[-1]['presupuesto']['beam_wander_urad']


def test_volumen_de_un_pase_crece_con_la_elevacion_maxima():
    sat, est, atm, _ = E.construir(CFG)
    bajo = E.volumen_pase(sat, est, atm, model.perfil_pase(550.0, 20.0, 10.0, 140.0, n=600), 0.6)
    alto = E.volumen_pase(sat, est, atm, model.perfil_pase(550.0, 80.0, 10.0, 140.0, n=600), 0.6)
    assert alto['gb'] > bajo['gb'] > 0.0


# ------------------------------------------------------------------ diversidad y RF

def test_lluvia_ka_cifras_citadas():
    assert D.indisponibilidad_lluvia(4.0) == pytest.approx(1e-3)
    assert D.indisponibilidad_lluvia(11.0) == pytest.approx(1e-4)
    assert D.margen_para_disponibilidad(0.999) == pytest.approx(4.0, abs=1e-9)
    assert D.margen_para_disponibilidad(0.9999) == pytest.approx(11.0, abs=1e-9)
    assert D.indisponibilidad_lluvia(31.0, 'ecuatorial') == pytest.approx(1e-3)
    assert D.indisponibilidad_lluvia(10.0, 'ecuatorial') > D.indisponibilidad_lluvia(31.0, 'ecuatorial')


def test_diversidad_con_pases_reales():
    estaciones, _ = orbit.cargar_estaciones()
    sitios, mod = N.cargar_clima()
    claves = ['madrid', 'cdmx', 'tromso']
    sat, est, atm0, _ = E.construir(CFG)
    atms = {k: dataclasses.replace(atm0, alt_estacion_m=float(estaciones[k].alt_m)) for k in claves}
    t0 = dt.datetime(2026, 10, 2, 0, 0, tzinfo=dt.timezone.utc)
    pases = D.pases_utiles('NOAA 20', claves, estaciones, t0, 3, 10.0, atms, (sat, est), 0.6)
    assert len(pases) > 15 and all(p.gb > 0 for p in pases)
    claro = N.serie_nubes([sitios[k] for k in claves], 3 * 24, mod['paso_h'], mod['tau_h'], mod['L_km'], semilla=11)
    g1 = D.entrega(pases, claro, claves, t0, {'madrid'})
    g3 = D.entrega(pases, claro, claves, t0, set(claves))
    assert g3[1] > g1[1]                           # mas estaciones, mas pases posibles
    assert 0.0 <= g1[0] / g1[1] <= 1.0
    assert g3[2].shape == (3,)
