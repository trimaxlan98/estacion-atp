"""Compensacion Doppler del receptor: la
precompensacion con el Doppler predicho reduce lo que el receptor debe buscar
y seguir, y la tasa cenital medida coincide con la formula del README."""
import math

import numpy as np
import pytest

from atp_ros import keyhole, model
from atp_sdr.cadena import CadenaRF
from atp_sdr.senal import doppler_de_vr

F_UHF = 437.8e6


def test_tasa_doppler_cenital_contra_la_formula_del_readme():
    """(v^2/h)(R/(R+h)) f0/c: medida sobre el perfil de un pase cenital (UHF y S, 550 y 420 km)."""
    for h in (420.0, 550.0):
        for f0 in (F_UHF, 2.2e9):
            n = int(model.duracion_pase(h, 89.9, 5.0) / 0.05)
            p = model.perfil_pase(h, 89.9, 5.0, 140.0, n=n)
            tasa = np.max(np.abs(np.gradient(doppler_de_vr(p['vr'], f0), p['t'])))
            v = model.velocidad_circular(h) * 1000.0
            formula = v * v / (h * 1000.0) * model.R_TIERRA_KM / (model.R_TIERRA_KM + h) \
                * f0 / model.C_LUZ_M_S
            assert tasa == pytest.approx(formula, rel=0.01)
            # la formula "plana" (v^2/h) sobrestima ~ (R+h)/R
            assert formula * (model.R_TIERRA_KM + h) / model.R_TIERRA_KM > 1.05 * formula


def _uhf(pred, ventana=None, fs=None, delta=-25.0, bn=None):
    """UHF (Yagi de 15 dBi, 1 W en el satelite) sobre 40 s del tramo cenital de un pase de 60 grados."""
    perfil = model.perfil_pase(550.0, 60.0, 5.0)
    tc = perfil['duracion'] / 2
    cad = CadenaRF('UHF', 'portadora', 3, f0_hz=F_UHF, delta_cn0_db=delta, eirp_dbw=0.0,
                   dish_m=keyhole.diametro_equivalente_yagi(F_UHF), tsys_k=300.0,
                   ventana_hz=ventana, fs_hz=fs, bn_hz=bn)
    t_adq, locked = float('nan'), []
    n = int(40.0 / cad.dt)
    for k in range(n):
        t = tc - 20.0 + k * cad.dt
        d = float(np.interp(t, perfil['t'], perfil['d']))
        vr = float(np.interp(t, perfil['t'], perfil['vr']))
        fd_pred = None
        if pred is not None:
            fd_pred = float(doppler_de_vr(vr, F_UHF)) + pred(t - tc)      # prediccion = verdad + error
        m = cad.paso(t, d, vr, 0.0, fd_pred_hz=fd_pred)
        locked.append(m['locked'])
        if m['locked'] and math.isnan(t_adq):
            t_adq = k * cad.dt
    return cad, t_adq, np.array(locked)


def test_sin_precompensar_el_lazo_estrecho_de_uhf_no_sigue_la_tasa_cenital():
    """Hallazgo: con el B_L por defecto de UHF (10 Hz) y SIN precompensacion, el receptor
    no sostiene el enganche en el tramo cenital (141-190 Hz/s): el detector cae y readquiere
    una y otra vez. Con B_L = 30 Hz si lo sostiene. Ni siquiera con C/N0 alto (74 dB-Hz)."""
    estrecho, _, l10 = _uhf(None, delta=0.0, bn=10.0)
    ancho, _, l30 = _uhf(None, delta=0.0, bn=30.0)
    assert l10.mean() < 0.5 and estrecho.resumen()['perdidas'] > 10
    assert l30.mean() > 0.95 and ancho.resumen()['perdidas'] == 0


def test_precompensar_reduce_la_ventana_y_el_error_de_frecuencia():
    """Con la prediccion (error 40 Hz + deriva 5 Hz/s) el lazo estrecho (10 Hz) engancha y
    sostiene, con una ventana de busqueda 22 veces mas angosta (+-500 Hz contra +-11.9 kHz)
    y con ~3 veces menos error de frecuencia (rms) que el lazo ancho (30 Hz) que necesita
    el receptor sin precompensar."""
    ca, ta, la = _uhf(None, bn=30.0)                                       # sin precompensar, lazo ancho
    cb, tb, lb = _uhf(lambda s: 40.0 + 5.0 * s, ventana=500.0, fs=50e3, bn=10.0)
    ra, rb = ca.resumen(), cb.resumen()
    assert la.mean() > 0.9 and lb.mean() > 0.9
    assert rb['perdidas'] == 0 and ra['perdidas'] == 0
    assert cb.ventana < ca.ventana / 20.0
    assert rb['doppler_rms_hz'] < 0.6 * ra['doppler_rms_hz']
    assert rb['doppler_rms_hz'] < 1.0


def test_el_error_de_la_prediccion_cabe_en_la_ventana_o_no_engancha_a_la_primera():
    """Si la prediccion se equivoca mas que la ventana (1 kHz de error contra +-500 Hz),
    la busqueda no encuentra el pico en la ventana: sin enganche en los primeros segundos."""
    cb, tb, lb = _uhf(lambda s: 1000.0, ventana=500.0, fs=50e3, bn=10.0)
    assert math.isnan(tb) or tb > 5.0
