"""Las cifras del curso 30 (Sistemas ATP) que la Estacion debe reproducir."""
import numpy as np
import pytest

from atp_ros import model as m


def test_geometria_pase():
    assert float(m.rango_oblicuo(550, 90)) == pytest.approx(550.0, abs=0.5)
    assert float(m.rango_oblicuo(550, 5)) == pytest.approx(2205, abs=5)
    assert m.tasa_acimut(550, 85) == pytest.approx(9.05, abs=0.05)
    assert m.tasa_acimut(550, 30) == pytest.approx(0.51, abs=0.02)
    assert m.duracion_pase(550, 90, 5) / 60 == pytest.approx(9.8, abs=0.05)


def test_perfil_coherente():
    p = m.perfil_pase(550, 85, 5)
    assert p['duracion'] == pytest.approx(m.duracion_pase(550, 85, 5), rel=1e-3)
    assert np.max(p['el']) == pytest.approx(85, abs=0.1)
    assert p['vr'][0] < 0 < p['vr'][-1]          # se acerca y luego se aleja


def test_enlace():
    assert m.ancho_haz(3, 2.2e9) == pytest.approx(3.18, abs=0.01)
    assert m.ancho_haz(3, 30e9) == pytest.approx(0.233, abs=0.001)
    assert m.ganancia_plato() == pytest.approx(34.6, abs=0.1)
    assert m.g_sobre_t() == pytest.approx(12.8, abs=0.1)
    th3 = m.ancho_haz()
    assert float(m.perdida_apuntamiento(th3 / 2, th3)) == pytest.approx(3.0)
    lb = m.presupuesto_enlace(1815, 0.1)
    assert float(lb['cn0_dbhz']) == pytest.approx(82.9, abs=0.1)
    assert float(lb['ebn0_db']) == pytest.approx(22.9, abs=0.1)
    # fuera del lobulo principal la perdida se acota a la ganancia del plato
    ka = m.presupuesto_enlace(600, 10.0, f_hz=30e9)
    assert float(ka['l_point_db']) == pytest.approx(m.ganancia_plato(f_hz=30e9))


def test_doppler():
    assert float(m.doppler_hz(1.0, 437e6)) < 0            # se aleja: baja
    # con la curvatura (R/(R+h)); la formula plana v^2/h daba 153
    assert m.tasa_doppler_cenit(550, 437e6) == pytest.approx(140.8, abs=1)
    # y coincide con la derivada numerica del Doppler sobre un pase cenital
    p = m.perfil_pase(550, 89.9, 5.0, n=20001)
    fd = m.doppler_hz(p['vr'], 437e6)
    tasa = np.max(np.abs(np.gradient(fd, p['t'])))
    assert tasa == pytest.approx(m.tasa_doppler_cenit(550, 437e6), rel=0.01)


def test_campanas_deterministas():
    a1 = m.campana_analitica(n=300, semilla=1)['rms']
    a2 = m.campana_analitica(n=300, semilla=1)['rms']
    assert np.array_equal(a1, a2)
    l1 = m.campana_lazo(n=20, semilla=3)
    l2 = m.campana_lazo(n=20, semilla=3)
    assert np.array_equal(l1['rms'], l2['rms'])


def test_lazo_pierde_en_keyhole():
    """Con el rotor a 6 deg/s, los pases que culminan por encima de la
    elevacion de keyhole (~82.5 deg) van mucho peor que los bajos."""
    assert m.elevacion_keyhole(550, 6.0) == pytest.approx(82.5, abs=0.1)
    out = m.campana_lazo(n=400, semilla=11, vel_max_deg_s=6.0)
    altos = out['rms'][out['el_max'] > m.elevacion_keyhole(550, 6.0)]
    bajos = out['rms'][out['el_max'] < 40]
    assert len(altos) and len(bajos)
    assert np.median(altos) > 3 * np.median(bajos)
