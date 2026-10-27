"""P6: guiado con autoridad, antiwindup, mezclador con prioridad de par, cruz manual y metrica de rafagas. Sin ROS."""
import hashlib
import json

import numpy as np

from atp_dron import control, guiado_autoridad as GA, rafagas_metrica as RM
from atp_dron.dron import Dron
from atp_dron.lazo import correr
from atp_dron.lockstep import Guiado, PlantaLock, cargar_cfg

TITAN = {'cuerpo': 'titan', 'semilla': 2, 'ruta': [[20, 0, 10], [40, 0, 10]], 'duracion_s': 45.0, 'rafaga_sigma': 0.5,
         'viento': [1.0, 0.0, 0.0], 'parar_al_llegar': False}


def _huella(cfg):
    m, f = correr(cfg)
    return hashlib.sha256((json.dumps(m, sort_keys=True) + ''.join(json.dumps(r, sort_keys=True) for r in f)).encode()).hexdigest()


def _sobreimpulso(cfg, post=None):
    cfg = cargar_cfg(None, cfg)
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    if post:
        post(gu)
    xs = []
    while not pl.fin:
        pl.aplicar(json.loads(json.dumps(gu.decidir(json.loads(pl.estado_txt())))))
        xs.append(pl.d.p[0])
    return max(xs) - cfg['ruta'][-1][0], pl.cerrar()


# ---------------------------------------------------------------------------------------------- formulas
def test_formulas_cerradas():
    assert abs(GA.a_horizontal(1.352, 35.0) - 0.9467) < 1e-3                      # g tan 35 en Titan
    assert abs(GA.v_terminal(1.5, 1.352, 5.4, 0.03, 35.0) - 4.187) < 0.01
    assert abs(GA.v_terminal(1.5, 9.80665, 1.225, 0.03, 35.0) - 23.68) < 0.05      # el 23.7 m/s de DRON.md
    assert GA.v_terminal(1.5, 1.62, 1e-14, 0.03, 35.0) > 1e5 or np.isfinite(GA.v_terminal(1.5, 1.62, 0.0, 0.03, 35.0)) is False


def test_limites_dejan_la_tierra_y_bajan_titan():
    a, v, _ = GA.limites_autoridad('tierra')
    assert (a, v) == (2.0, 5.0)
    a, v, info = GA.limites_autoridad('titan')
    assert abs(a - 0.6 * 0.9467) < 1e-3 and abs(v - 0.9 * 4.187) < 0.01 and info['az_acel_ms2'] <= 0.5 * 1.352 + 1e-12


# ---------------------------------------------------------------------------------------------- bytes por defecto
def test_banderas_apagadas_son_los_bytes_de_siempre():
    cfg = {**TITAN, 'duracion_s': 8.0}
    apagadas = {**cfg, 'autoridad_guiado': False, 'autoridad_acel': False, 'antiwindup': False, 'mezcla_par': False,
                'seg0_corregido': False}
    assert _huella(cfg) == _huella(apagadas)


def test_cruz_manual_es_np_cross_bit_a_bit():
    rng = np.random.default_rng(0)
    for _ in range(200):
        a, b = rng.normal(size=3) * 10.0 ** rng.integers(-3, 3), rng.normal(size=3)
        assert control._cruz(a, b).tobytes() == np.cross(a, b).tobytes()


def test_cruz_manual_no_cambia_la_telemetria():
    cfg = {**TITAN, 'duracion_s': 10.0}
    nueva = control._cruz
    try:
        control._cruz = lambda a, b: np.cross(a, b)
        h_vieja = _huella(cfg)
    finally:
        control._cruz = nueva
    assert _huella(cfg) == h_vieja


# ---------------------------------------------------------------------------------------------- Titan
def test_titan_baseline_se_pasa_de_la_meta_y_con_autoridad_no():
    base, _ = _sobreimpulso(TITAN)
    nuevo, met = _sobreimpulso({**TITAN, 'autoridad_guiado': True, 'autoridad_acel': 'rampa', 'antiwindup': True})
    assert base > 3.0
    assert nuevo < 0.5 and met['llego'] and met['inclinacion_max_deg'] < 40.0


def test_control_negativo_a_brake_forzado_reaparece_el_sobreimpulso():
    cfg = {**TITAN, 'autoridad_guiado': True, 'autoridad_acel': 'rampa', 'antiwindup': True}

    def forzar(gu):
        gu.a_brake = 2.0
    ok, _ = _sobreimpulso(cfg)
    mal, _ = _sobreimpulso(cfg, forzar)
    assert ok < 0.5 and mal > 10 * max(ok, 0.05)


# ---------------------------------------------------------------------------------------------- mezclador
def test_mezclador_sin_tope_es_la_mezcla_de_siempre_y_con_tope_respeta_el_par():
    d = Dron('titan')
    tau = np.array([0.2, -0.3, 0.01])
    T = d.m * d.g
    base = d.A_inv @ np.array([T, *tau])
    assert base.min() < 0.0                                       # el caso satura: un rotor pide menos de 0 N
    assert np.array_equal(GA.mezclar_prioridad_par(d.A_inv, T, tau, d.t_max, 0.0), base)
    out = GA.mezclar_prioridad_par(d.A_inv, T, tau, d.t_max, 1e9)
    assert out.min() >= -1e-12 and out.max() <= d.t_max
    assert np.allclose((d.A @ out)[1:], tau)                      # el par se conserva
    assert (d.A @ out)[0] > T                                     # a costa de empuje colectivo extra


# ---------------------------------------------------------------------------------------------- antiwindup
def test_antiwindup_congela_la_integral_con_la_inclinacion_saturada():
    d = Dron('tierra', pos=(0, 0, 10))
    ints = {}
    for aw in (False, True):
        c = control.Cascada(d, antiwindup=aw)
        for _ in range(200):
            c._lazo_externo(np.array([20.0, 0.0, 0.0]), 0.02)     # error enorme: la inclinacion se satura en seguida
        ints[aw] = c.int_v.copy()
    assert abs(ints[False][0]) > 2.9 and abs(ints[True][0]) < 0.5


def test_viento_sostenido_con_antiwindup_no_deriva():
    _, f = correr({'ruta': [[0, 0, 10]], 'duracion_s': 30.0, 'parar_al_llegar': False, 'viento': [12.0, 0, 0], 'antiwindup': True})
    assert max(np.hypot(r['x'], r['y']) for r in f if r['t'] > 15.0) < 0.2


# ---------------------------------------------------------------------------------------------- lockstep aditivo
def test_ruta_vacia_no_lanza_y_se_queda():
    m, f = correr({'ruta': [], 'duracion_s': 1.0})
    assert m['metas_total'] == 0 and f[-1]['z'] == 0.0 and all(r['meta'] == 0 for r in f)


def test_seg0_corregido_baja_el_error_transversal_sin_viento():
    cfg = {'ruta': [[20, 10, 10]], 'duracion_s': 40.0}
    a, _ = correr(cfg)
    b, _ = correr({**cfg, 'seg0_corregido': True})
    assert b['error_transversal_rms_m'] < 0.5 * a['error_transversal_rms_m']


# ---------------------------------------------------------------------------------------------- metrica de rafagas
def test_desv_vs_sin_rafagas_cero_exacto_y_creciente():
    cfg = {'ruta': [[12, 0, 6], [12, 10, 6]], 'duracion_s': 14.0, 'viento': [1.0, 0, 0], 'semilla': 4, 'parar_al_llegar': False}
    _, _, gem = RM.correr_con_metrica({**cfg, 'rafaga_sigma': 0.0})
    v = []
    for sg in (0.0, 0.5, 1.0, 2.0):
        m, _, _ = RM.correr_con_metrica({**cfg, 'rafaga_sigma': sg}, gem)
        v.append(m['desv_vs_sin_rafagas_rms_m'])
    assert v[0] == 0.0 and v[0] < v[1] < v[2] < v[3]


def test_t_student_y_ic95():
    assert abs(RM.t_cuantil_975(29) - 2.0452) < 2e-3
    m, h = RM.ic95([1.0, 2.0, 3.0])
    assert m == 2.0 and abs(h - 4.3027 / np.sqrt(3)) < 2e-3
