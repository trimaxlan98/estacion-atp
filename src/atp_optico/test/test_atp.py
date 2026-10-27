"""Validacion de atp_optico.atp: el lazo del FSM, el banco del tilt, el motor de un paso (reproducible por semilla)
y el efecto del FSM sobre el error residual y el enlace."""
import dataclasses
import math

import numpy as np
import pytest
from scipy import signal

from atp_optico import atmosfera as A
from atp_optico import atp as P
from atp_optico import enlace as L

HV2240 = dataclasses.replace(A.HV57, alt_estacion_m=2240.0)
_trapz = getattr(np, 'trapezoid', None) or np.trapz


# ------------------------------------------------------------------ el lazo del FSM

def test_lazo_fino_estable_con_margenes():
    lin = P.lazo_fino(P.LAZO_FSM)
    assert lin.estable()
    m = lin.margenes()
    assert m['f_cruce_hz'] == pytest.approx(P.LAZO_FSM.f_cruce_hz, rel=0.01)
    assert m['pm_deg'] > 45.0 and m['gm_db'] > 6.0 and m['ms'] < 2.0, m


def test_rechazo_de_baja_frecuencia_tipo_1():
    lin = P.lazo_fino(P.LAZO_FSM)
    # |S| = f/f_cruce a baja frecuencia (integrador puro); |T| ~ 1
    for f in (0.5, 5.0, 20.0):
        assert abs(lin.S(f)[0]) == pytest.approx(f / P.LAZO_FSM.f_cruce_hz, rel=0.05)
    assert abs(lin.T(1.0)[0]) == pytest.approx(1.0, abs=0.01)
    # S + T = 1
    f = np.array([1.0, 50.0, 400.0, 1500.0])
    assert np.allclose(lin.S(f) + lin.T(f), 1.0, atol=1e-9)


def test_respuesta_temporal_a_un_tono_contra_S():
    lin = P.lazo_fino(P.LAZO_FSM)
    fs, f0, A0 = P.LAZO_FSM.fs_hz, 40.0, 100.0
    n = 40000
    d = A0 * np.sin(2 * math.pi * f0 * np.arange(n) / fs)
    e = signal.lfilter(lin.bS, lin.aS, d)
    amp = np.max(np.abs(e[n // 2:]))
    assert amp == pytest.approx(A0 * abs(lin.S(f0)[0]), rel=0.02)


# ------------------------------------------------------------------ el banco del tilt

def test_banco_de_tilt_ajusta_la_forma_y_normaliza():
    b = P.banco_tilt(P.LAZO_FSM.fs_hz)
    f_c, f_L = 30.0, 1.0
    w = b.pesos(f_c, f_L)
    assert w.sum() == pytest.approx(1.0, abs=1e-12) and np.all(w >= 0.0)
    f = np.logspace(-1, 3.2, 200)
    psd = b.psd(f, w)
    # varianza total = 1 (PSD unilateral integrada)
    ff = np.logspace(-3, math.log10(0.499 * P.LAZO_FSM.fs_hz), 40000)
    assert _trapz(b.psd(ff, w), ff) == pytest.approx(1.0, rel=0.03)
    # forma: contra la PSD objetivo normalizada, dentro de ~30 % entre 2 f_L y 1.2 f_c (donde esta la varianza) y la
    # pendiente -2/3 ... -17/3 aproximada
    obj = P.espectro_tilt(ff, f_L, f_c)
    obj = obj / _trapz(obj, ff)
    sel = (f >= 2.0 * f_L) & (f <= 1.2 * f_c)
    ratio = psd[sel] / np.interp(f[sel], ff, obj)
    assert np.all((ratio > 0.7) & (ratio < 1.4)), (ratio.min(), ratio.max())
    # pendiente alta: cae >= f^-4 sobre 3 f_c (la forma objetivo es -17/3)
    hi = b.psd(np.array([3 * f_c, 30 * f_c]), w)
    assert math.log10(hi[0] / hi[1]) > 3.5


def test_rms_del_tilt_simulado_y_analitico_contra_el_lazo():
    """El rms del error residual del FSM sobre una perturbacion SOLO de tilt: integral de |S|^2 PSD (analitico)
    contra la simulacion temporal con lfilter (el mismo camino que usa el motor)."""
    fs = P.LAZO_FSM.fs_hz
    lin = P.lazo_fino(P.LAZO_FSM)
    b = P.banco_tilt(fs)
    f_c, f_L = 40.0, 1.5
    w = b.pesos(f_c, f_L)
    sigma = 5.0
    ff = np.logspace(-2, math.log10(0.499 * fs), 60000)
    var_e = sigma ** 2 * _trapz(b.psd(ff, w) * np.abs(lin.S(ff)) ** 2, ff)
    rng = np.random.default_rng(3)
    can = P.CanalTilt(b)
    n = 600000
    d = sigma * np.concatenate([can.bloque(10000, rng, w) for _ in range(n // 10000)])
    e = signal.lfilter(lin.bS, lin.aS, d)
    assert float(np.std(d)) == pytest.approx(sigma, rel=0.12)           # la varianza total (la cola de baja frecuencia converge lento)
    assert float(np.sqrt(np.mean(e[20000:] ** 2))) == pytest.approx(math.sqrt(var_e), rel=0.10)


# ------------------------------------------------------------------ motor de un paso

def _motor(semilla=7, **kw):
    sat, est = L.TerminalSat(), L.EstacionOptica()
    cfg = P.ConfigAtp(semilla=semilla, burn_s=2.0, **kw)
    return P.AtpOptico(cfg, sat, est, HV2240), cfg


def _correr(motor, n, err_fun=lambda k: np.zeros(2)):
    filas = []
    for k in range(n):
        el = 30.0 + 0.1 * k
        az = 100.0 + 0.5 * k
        filas.append(motor.paso(0.05 * (k + 1), az, el, 1000.0 - 2.0 * k, err_fun(k)))
    return filas


def test_el_motor_es_reproducible_por_semilla():
    a, _ = _motor(7)
    b, _ = _motor(7)
    c, _ = _motor(8)
    fa = _correr(a, 30, lambda k: np.array([50.0, -20.0]))
    fb = _correr(b, 30, lambda k: np.array([50.0, -20.0]))
    fc = _correr(c, 30, lambda k: np.array([50.0, -20.0]))
    assert fa == fb                                                     # identico campo a campo (float a float)
    assert fa != fc


def test_el_fsm_reduce_el_error_de_la_montura():
    m, _ = _motor()
    # error de montura lento de 200 urad rms (como un seguimiento deficiente): el FSM lo absorbe
    filas = _correr(m, 200, lambda k: 200.0 * np.array([math.sin(0.05 * k), math.cos(0.03 * k)]))
    con = np.sqrt(np.mean([f['sigma_fsm_urad'] ** 2 for f in filas[20:]]))
    sin = np.sqrt(np.mean([f['sigma_sin_urad'] ** 2 for f in filas[20:]]))
    assert sin > 100.0
    assert con < sin / 30.0
    assert all(f['enganchado'] for f in filas)


def test_saturacion_del_fsm_pierde_el_enganche_y_el_enlace_cae():
    m, _ = _motor(modelo_apunt_urad=0.0)
    filas = _correr(m, 60, lambda k: np.array([2500.0, 0.0]))          # 2.5 mrad > la carrera de 1 mrad
    assert not any(f['enganchado'] for f in filas[5:])
    # sin enganche el enlace ve el error de la montura: tasa mas baja que con el FSM sano
    sano, _ = _motor(modelo_apunt_urad=0.0)
    ok = _correr(sano, 60, lambda k: np.array([300.0, 0.0]))
    assert filas[-1]['con_fsm']['tasa_bps'] < ok[-1]['con_fsm']['tasa_bps']


def test_ruido_del_sensor_crece_al_bajar_la_potencia():
    cerca, _ = _motor()
    lejos, _ = _motor()
    a = cerca.paso(0.05, 100.0, 60.0, 600.0, np.zeros(2))
    b = lejos.paso(0.05, 100.0, 12.0, 1800.0, np.zeros(2))
    assert b['nea_urad'] > a['nea_urad']


def test_tilt_rms_coincide_con_la_formula():
    m, _ = _motor(modelo_apunt_urad=0.0, vibracion_urad=0.0)
    filas = _correr(m, 300)
    el = np.array([f['el'] for f in filas])
    sig = 1e6 * A.tilt_llegada_rad(HV2240, 1550e-9, 0.4, float(np.mean(el)))
    rms = np.sqrt(np.mean([f['rms_tilt_urad'] ** 2 for f in filas]))
    assert rms == pytest.approx(sig, rel=0.5)                           # 15 s de pase: la cola lenta no converge


def test_medidor_resumen_formato():
    m, cfg = _motor()
    med = P.Medidor(cfg, L.EstacionOptica())
    for f in _correr(m, 40, lambda k: np.array([30.0, 10.0])):
        med.muestra(f)
    r = med.resumen_dict()
    assert r['rms_con_fsm_urad'] < r['rms_sin_fsm_urad']
    assert 0 <= r['con_fsm']['cerrado_pct'] <= 100
    assert len(med.resumen()) == 3 and med.resumen()[0].startswith('RESUMEN OPTICO')


# ------------------------------------------------------------------ lineas base analiticas

def test_montura_cerrada_sobre_el_sensor_no_sigue_el_tilt_y_el_fsm_si():
    sig = 4.0
    f_c, f_L = 0.3 * 60.0 / 0.4, 60.0 / 25.0
    montura = P.residuo_montura_cerrada_urad(sig, f_c, f_L)
    fsm = P.residuo_fsm_analitico_urad(sig, f_c, f_L)
    # la montura (~0.5 Hz) deja pasar casi todo el tilt, que vive sobre f_L; el FSM de 400 Hz lo rechaza >10x
    assert montura > 0.7 * sig
    assert fsm < sig / 10.0
    # |S| de la montura: ~1 sobre 3 Hz, << 1 bajo 0.1 Hz
    s = np.abs(P.sensibilidad_montura(np.array([0.01, 0.1, 3.0, 30.0])))
    assert s[0] < 0.05 and s[2] > 0.85 and s[3] == pytest.approx(1.0, abs=0.02)


def test_residuo_analitico_crece_con_el_ruido_del_sensor():
    a = P.residuo_fsm_analitico_urad(4.0, 150.0, 2.4, nea_urad=0.0)
    b = P.residuo_fsm_analitico_urad(4.0, 150.0, 2.4, nea_urad=2.0)
    assert b > a
