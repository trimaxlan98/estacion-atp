"""Pruebas P8: fisica del rotor. Contrastes analiticos y controles negativos; todo en simulacion."""
import hashlib

import numpy as np
import pytest

from atp_dron import rotor_fisica as RF
from atp_dron.cuerpos import cuerpo, viscosidad
from atp_dron.dron import Dron, PARAMS, rot

R = PARAMS['radio_rotor_m']


def _hover_potencia(c, params):
    d = Dron(c, params=params, pos=(0, 0, 1000.0))
    Th = d.m * d.g / 4
    for _ in range(500):
        d.paso(np.full(4, Th))
    e0 = d.energia_j
    d.paso(np.full(4, Th))
    return (d.energia_j - e0) / 0.004, d


def _hash(c, params, om0=None):
    d = Dron(c, params=params, pos=(0, 0, 0.3), viento=(2, 1, 0), rafaga_sigma=1.0, semilla=3, yaw_deg=10)
    if om0 is not None:
        d.om[:] = om0
    h = hashlib.sha256()
    rng = np.random.default_rng(1)
    for k in range(400):
        d.paso(d.m * d.g / 4 * (1 + 0.1 * np.sin(k * 0.01) + 0.05 * rng.standard_normal(4)))
        h.update(np.concatenate([d.p, d.v, d.q, d.w, d.om, d.fuerza_especifica(), [d.energia_j]]).tobytes())
    return h.hexdigest()


def test_legacy_incoherente_y_coherente_cierra():
    for c in ('tierra', 'titan'):
        _, d = _hover_potencia(c, {})
        assert abs(RF.fom_implicita(d.empuje[0], d.om[0], d.P['k_guinada_m'], d.rho, R) - 0.6) > 0.09   # 0.5046 vs 0.6
        p, d = _hover_potencia(c, {'modo_rotor': 'coherente'})
        assert abs(RF.fom_implicita(d.empuje[0], d.om[0], d.P['k_guinada_m'], d.rho, R) - 0.6) < 1e-6
        cu = cuerpo(c)
        esperado = 4 * (1.5 * cu['g'] / 4) ** 1.5 / (np.sqrt(2 * cu['rho'] * np.pi * R ** 2) * 0.6 * 0.8) + 15.0
        assert abs(p - esperado) / esperado < 1e-9


def test_fom_implicita_coherente_vale_a_cualquier_empuje():
    d = Dron('tierra', params={'modo_rotor': 'coherente'})
    for f in (0.4, 1.0, 1.9):
        T = f * d.m * d.g / 4
        assert abs(RF.fom_implicita(T, np.sqrt(T / d.kT), d.P['k_guinada_m'], d.rho, R) - 0.6) < 1e-12


def test_energia_par_sube_la_potencia_y_control_negativo_legacy_no():
    p_leg, _ = _hover_potencia('tierra', {})
    p_par, d = _hover_potencia('tierra', {'modo_rotor': 'energia_par'})
    assert p_par > 1.15 * p_leg                         # FoM efectiva 0.5046 < 0.6
    assert d.P['k_guinada_m'] == 0.016


def test_momentum_ramas_cerradas():
    vh = 5.0
    for vc in (0.0, 0.3, 2.0, 9.0):
        vi = RF.induccion_axial(vc, vh)
        assert abs(vi * (vi + vc) - vh ** 2) / vh ** 2 < 1e-12
        assert abs(vi - (-vc / 2 + np.sqrt((vc / 2) ** 2 + vh ** 2))) / vi < 1e-3
    for vc in (-10.0, -15.0, -40.0):                   # freno de molino: vi (vi+vc) = -vh^2
        vi = RF.induccion_axial(vc, vh)
        assert abs(vi * (vi + vc) + vh ** 2) / vh ** 2 < 1e-12
    assert abs(RF.induccion_axial(-10.0, vh) - vh) < 1e-12      # continuidad con la rama de ascenso en -2 vh


def test_vrs_no_implementado_lanza():
    with pytest.raises(NotImplementedError):
        RF.induccion_axial(-3.0, 5.0)
    with pytest.raises(ValueError):
        Dron('tierra', params={'vrs': True})


def test_glauert_iterativo_igual_a_cerrado_y_mu_cero():
    vh = 5.44
    for V in (0.0, 1.0, 5.0, 20.0, 80.0):
        assert abs(RF.glauert_iterativa(V, vh)[0] - RF.glauert_cerrada(V, vh)) / vh < 1e-12
    Th = 1.5 * 9.80665 / 4
    p0, _ = RF.potencia_avance(Th, 0.0, 1.225, R, 0.6, 674.0, 4.65, 0.03)
    assert abs(p0 - RF.potencia_ideal(Th, 1.225, R) / 0.6) / p0 < 1e-9
    assert abs(RF.glauert_cerrada(500.0, vh) / (vh ** 2 / 500.0) - 1) < 1e-6          # alta velocidad: vh^2/V


def test_curva_en_u_y_control_negativo_sin_glauert():
    Th, om = 3.68, 674.0
    Vs = np.linspace(0, 30, 61)
    ind = [RF.potencia_avance(Th, V, 1.225, R, 0.6, om, 4.65, 0.03)[1]['inducida'] for V in Vs]
    tot = np.array([RF.potencia_avance(Th, V, 1.225, R, 0.6, om, 4.65, 0.03)[0] for V in Vs])
    assert np.all(np.diff(ind) < 0)
    assert 0 < int(np.argmin(tot)) < len(Vs) - 1
    V = 30.0
    par = RF.potencia_avance(Th, V, 1.225, R, 0.6, om, 4.65, 0.03)[1]['parasita'] * 4
    assert abs(par - 0.5 * 0.03 * 1.225 * V ** 3) / par < 1e-12
    # control negativo: con induccion de hover constante la potencia solo crece
    plano = [RF.potencia_ideal(Th, 1.225, R) / 0.6 * 1.0 + 0.5 * 0.03 * 1.225 * V ** 3 / 4 for V in Vs]
    assert np.all(np.diff(plano) > 0)


def test_energia_avance_en_simulador():
    d0 = Dron('tierra', params={'avance': True})
    Th = d0.m * d0.g / 4
    om = np.sqrt(Th / d0.kT)
    d0.om[:] = om
    d0.empuje = d0.kT * d0.om ** 2
    d0._energia(1.0)
    base = 4 * RF.potencia_ideal(Th, 1.225, R) / (0.6 * 0.8) + 15.0
    assert abs(d0.energia_j - base) / base < 1e-9


def test_reynolds_tierra_factor_uno_y_bytes():
    om_h = float(np.sqrt(1.5 * 9.80665 / 4 / Dron('tierra').kT))
    assert RF.f_reynolds(RF.reynolds(1.225, viscosidad('tierra'), om_h, R, 0.02)) == 1.0
    assert _hash('tierra', {}, om0=om_h) == _hash('tierra', {'f_reynolds': True}, om0=om_h)
    # control negativo: desde reposo el rotor pasa por Re bajo y f < 1 cambia los bytes
    assert _hash('tierra', {}) != _hash('tierra', {'f_reynolds': True})


def test_reynolds_marte_reduce_empuje_y_potencia_sube():
    mu = viscosidad('marte')
    d = Dron('marte', params={'f_reynolds': True, 'rotor_escala': 11.0})
    d.om[:] = d.P['om_max']
    assert d._empuje_efectivo(d.om, 5.0)[0] < 0.8 * d.t_max
    cu = cuerpo('marte')
    s0 = RF.escala_minima_hover(cu['rho'], cu['g'])
    assert abs(s0 - 10.5326) < 1e-3
    assert RF.escala_minima_hover(cu['rho'], cu['g'], mu) > s0
    assert RF.escala_minima_hover(cu['rho'], cu['g'], mu, expo_kT=0.0) == pytest.approx(s0, rel=1e-9)   # sin Re: solo densidad
    sA = RF.escala_minima_hover(cu['rho'], cu['g'], mu)
    pA = RF.hover_con_re(cu['rho'], cu['g'], mu, sA * 1.001)['potencia_w']
    p0 = 4 * RF.potencia_ideal(1.5 * cu['g'] / 4, cu['rho'], R) / (0.6 * 0.8) + 15
    assert pA > p0
    assert viscosidad('titan') is None


def test_factor_suelo_unico_y_control_negativo():
    rng = np.random.default_rng(3)
    for z in (0.03, 0.1, 0.5, 2.0):
        d = Dron('tierra', params={'efecto_suelo': True}, pos=(0, 0, z))
        d.q = rng.standard_normal(4); d.q /= np.linalg.norm(d.q)
        d.v = rng.standard_normal(3); d.om = 600 + 100 * rng.random(4); d.en_suelo = False
        wind = np.array([1.0, -2.0, 0.5])
        d.viento_medio = wind
        s = np.concatenate([d.p, d.v, d.q, d.w, d.om])
        a_body = rot(d.q).T @ (d._deriv(s, d.om, wind)[3:6] + np.array([0, 0, d.g]))
        assert np.abs(a_body - d.fuerza_especifica()).max() < 1e-12
        assert np.array_equal(d._empuje_efectivo(d.om, z), RF.aplicar_suelo(d.kT * d.om * d.om, z, R))

    class Roto(Dron):                                  # una sola ruta alterada: la prueba se rompe
        def fuerza_especifica(self):
            T = RF.aplicar_suelo(self.kT * self.om * self.om, self.p[2], R * 1.01)
            return np.array([0.0, 0.0, T.sum()]) / self.m
    d = Roto('tierra', params={'efecto_suelo': True}, pos=(0, 0, 0.1))
    d.om[:] = 650.0; d.en_suelo = False
    s = np.concatenate([d.p, d.v, d.q, d.w, d.om])
    assert np.abs(rot(d.q).T @ (d._deriv(s, d.om, d.viento)[3:6] + np.array([0, 0, d.g])) - d.fuerza_especifica()).max() > 1e-3


def test_cruz_manual_y_legacy_identicos():
    from atp_dron.dron import _cruz
    rng = np.random.default_rng(0)
    for _ in range(100):
        a, b = rng.standard_normal(3), rng.standard_normal(3)
        assert np.array_equal(_cruz(a, b), np.cross(a, b))
    # hashes de referencia del comportamiento por defecto (calculados antes de tocar dron.py)
    ref = {('tierra', ()): '2d06688e3766bf35', ('marte', ()): '354041364708a064', ('titan', ()): 'b06092527d1b5809'}
    from importlib import import_module  # noqa: F401
    for (c, _), h in ref.items():
        d = Dron(c, pos=(0, 0, 0.3), viento=(2, 1, 0), rafaga_sigma=1.0, semilla=3, yaw_deg=10)
        hh = hashlib.sha256()
        rng = np.random.default_rng(1)
        for k in range(1500):
            d.paso(d.m * d.g / 4 * (1 + 0.1 * np.sin(k * 0.01) + 0.05 * rng.standard_normal(4)))
            hh.update(np.concatenate([d.p, d.v, d.q, d.w, d.om, d.fuerza_especifica(), [d.energia_j]]).tobytes())
        assert hh.hexdigest()[:16] == h
