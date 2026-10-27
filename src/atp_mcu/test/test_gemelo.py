"""Pruebas del gemelo digital del banco HIL (logica pura, sin ROS)."""
import math
import os

import numpy as np
import pytest

from atp_mcu import gemelo
from atp_ros import model

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def test_interpolar_lineal_y_bordes():
    t = [0.0, 1.0, 2.0]
    y = np.array([[0.0, 10.0], [1.0, 20.0], [4.0, 30.0]])
    assert gemelo.interpolar(0.5, t, y) == pytest.approx([0.5, 15.0])
    assert gemelo.interpolar(1.0, t, y) == pytest.approx([1.0, 20.0])
    assert gemelo.interpolar(1.5, t, y) == pytest.approx([2.5, 25.0])
    assert gemelo.interpolar(-0.1, t, y) is None
    assert gemelo.interpolar(2.1, t, y) is None


def test_percentiles():
    r = gemelo.percentiles([1.0, 2.0, 3.0, 4.0, 100.0])
    assert r['n'] == 5 and r['min'] == 1.0 and r['max'] == 100.0 and r['p50'] == 3.0
    assert gemelo.percentiles([])['n'] == 0


def test_gemelo_reposo_se_queda_quieto():
    refs = np.tile([1.0, 0.5], (40, 1))
    meas, u = gemelo.gemelo_pid(refs)
    assert np.allclose(meas, refs) and np.allclose(u, 0.0)


def test_gemelo_sigue_la_rampa_n2():
    # rampa de acimut a 9 deg/s (el pase cenital N2): el PID (tipo 2) sigue la rampa; CONTROL.md,
    # tabla (a): PID rms 0.0167 deg en el pase completo (incluye transitorio de enganche)
    dt = 0.05
    t = np.arange(0, 20.0, dt)
    refs = np.stack([math.radians(10.0) + math.radians(9.0) * t, np.full_like(t, 0.5)], axis=1)
    meas, _ = gemelo.gemelo_pid(refs, dt=dt)
    err = np.degrees(refs[:, 0] - meas[:, 0])
    assert np.sqrt(np.mean(err[100:] ** 2)) < 0.1          # regimen, tras el enganche
    assert abs(err[-1]) < 0.05


def test_gemelo_identico_a_mount_sim_mas_pid_en_un_lazo_independiente():
    # Reimplementacion minima y directa del mismo lazo (Euler semi-implicito J w' = u - b w): la
    # trayectoria del gemelo debe coincidir con ella a precision de maquina
    dt, h = 0.05, 0.002
    refs = np.array([[0.1 * math.sin(0.3 * k * dt), 0.0] for k in range(60)])
    meas, _ = gemelo.gemelo_pid(refs, dt=dt, h=h)
    th, w, integ, e_prev, d = refs[0, 0], 0.0, 0.0, 0.0, 0.0
    u = 0.0
    a = dt / (0.08 + dt)
    for k in range(60):
        if k:
            for _ in range(25):
                w += (u - model.B_EJE * w) / model.J_EJE * h
                th += w * h
        e = refs[k, 0] - th
        d += a * ((e - e_prev) / dt - d)
        raw = 25.0 * e + 2.0 * integ + 8.0 * d
        u = max(-8.0, min(8.0, raw))
        if not (abs(raw) > 8.0 and raw * e > 0):
            integ += e * dt
        e_prev = e
        assert meas[k, 0] == pytest.approx(th, abs=1e-12)


def test_deriva_cero_si_son_iguales_y_crece_con_diferencia():
    refs = np.tile([1.0, 0.5], (100, 1))
    m = np.tile([1.0, 0.5], (100, 1))
    d0 = gemelo.deriva(m, m, refs)
    assert d0['dif_cielo_max_deg'] == 0.0 and d0['deriva_cielo_deg_por_min'] == 0.0
    m2 = m + np.outer(np.arange(100) * 1e-4, [1.0, 0.0])           # una diferencia que crece
    d1 = gemelo.deriva(m2, m, refs)
    assert d1['dif_az_max_deg'] == pytest.approx(math.degrees(99e-4))
    assert d1['deriva_cielo_deg_por_min'] > 0.0
    # en el cielo el acimut pesa cos(el): 0.5 rad -> 0.8776
    assert d1['dif_cielo_max_deg'] == pytest.approx(math.degrees(99e-4) * math.cos(0.5), rel=1e-6)


def test_el_nodo_y_su_lanzador_estan_registrados():
    with open(os.path.join(RAIZ, 'src', 'atp_mcu', 'setup.py')) as f:
        assert 'mcu_lockstep = atp_mcu.mcu_lockstep:main' in f.read()
    assert os.path.exists(os.path.join(RAIZ, 'src', 'atp_mcu', 'launch', 'mcu_hil_lockstep.launch.py'))
