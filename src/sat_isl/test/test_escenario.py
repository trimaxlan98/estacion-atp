"""Pruebas del escenario completo (planta + controlador + medidor, sin ROS):
adquisicion -> seguimiento -> enlace, determinismo del RESUMEN, que el controlador no
use la verdad, perdida de enlace por visibilidad y por potencia, y los dos detectores.

La corrida ROS (scripts/validar_isl.sh) debe dar el MISMO RESUMEN que `simular`.
"""
import dataclasses
import math

import numpy as np
import pytest

from sat_isl.escenario import simular, tabla_vecinos
from sat_isl.misiones import (MODO_ESPERA, MODO_SEGUIMIENTO, Config, ControladorIsl,
                              constelacion_de)

CFG = Config(dur_s=15.0)


@pytest.fixture(scope='module')
def const():
    return constelacion_de(CFG)


@pytest.fixture(scope='module')
def base(const):
    return simular(CFG, const)


def con(**kw):
    return dataclasses.replace(CFG, **kw)


def test_adquisicion_seguimiento_enlace(base):
    n = base.numeros()
    assert n['t_adq_s'] is not None and n['t_adq_s'] < 1.0       # ~0.3 s con efemerides de 1 dia
    assert n['barridos'] == 0                                    # a la primera pasada
    assert n['visible_pct'] == 100.0
    # seguimiento fino: el error radial es del orden de 1 urad, mucho menor que el haz (29.6 urad)
    assert n['err_rms_urad'] < 1.5 and n['err_p95_urad'] < 2.5
    assert n['err_p95_urad'] < 0.1 * n['theta_h_urad']
    assert n['perdida_max_db'] < 0.1                             # perdida por apuntamiento despreciable
    # enlace: se cierra todo el tiempo salvo la adquisicion
    assert n['cerrado_pct_visible'] > 95.0
    assert n['tasa_cerrado_bps'] >= 1e9
    assert n['ber_max'] < 1e-6
    assert -55.0 < n['pr_media_dbm'] < -45.0
    # el volumen es tasa x tiempo cerrado
    assert math.isclose(n['volumen_gb'], n['tasa_media_bps'] * n['duracion_s'] / 8e9, rel_tol=0.1)
    # geometria del par del mismo plano: 5.49 Mm, adelanto 38 urad
    assert math.isclose(n['rango_km'][0], 5492.0, abs_tol=20.0)
    assert math.isclose(n['adelanto_urad'], 38.0, abs_tol=1.0)


def test_el_resumen_es_determinista(const, base):
    otra = simular(CFG, const)
    assert otra.resumen() == base.resumen()
    # y depende de la semilla (lazos y sorteos distintos)
    distinta = simular(con(semilla=8), const)
    assert distinta.resumen() != base.resumen()


def test_resumen_tiene_las_cifras_pedidas(base):
    txt = '\n'.join(base.resumen())
    for clave in ('adquisicion', 'seguimiento fino', 'p95', 'rms', 'urad', 'cerrado', 'Mb/s', 'GB'):
        assert clave in txt


def test_el_controlador_no_usa_la_verdad(const):
    """Con un estado que solo trae los sensores el controlador responde igual: no necesita
    rango, visibilidad, error de apuntamiento ni nada de la verdad."""
    sensores = ('det_a', 'det_b', 'cen_a', 'cen_b', 'qpd_ok', 'pr_a_dbm', 'pr_b_dbm')
    from sat_isl.misiones import PlantaIsl
    p = PlantaIsl(CFG, const)
    c1, c2 = ControladorIsl(CFG, const), ControladorIsl(CFG, const)
    st = p.medir()
    solo = {k: st[k] for k in sensores}
    for t in (0.0, 0.05, 0.1):
        a = c1.paso(t, st)
        b = c2.paso(t, solo)
        assert a['modo'] == b['modo']
        assert np.array_equal(a['dir_a'], b['dir_a']) and a['tasa_bps'] == b['tasa_bps']


def test_el_error_de_efemerides_y_actitud_es_del_controlador_no_de_la_planta(const):
    """La direccion mandada difiere de la verdad por el error de posicion (2.1 km) y de actitud
    (145 urad): del orden de 500-600 urad."""
    from sat_isl.misiones import PlantaIsl, _geom_directa
    p = PlantaIsl(CFG, const)
    c = ControladorIsl(CFG, const)
    cmd = c.paso(0.0, p.medir())
    app_a = _geom_directa(p._geom())[0]
    err = np.linalg.norm(np.asarray(cmd['dir_a']) - app_a) * 1e6
    assert 100.0 < err < 2500.0
    # con efemerides exactas y actitud perfecta el error desaparece
    exacto = ControladorIsl(con(sigma_pos_km=1e-9, sigma_actitud_urad=1e-9), const)
    cmd2 = exacto.paso(0.0, p.medir())
    assert np.linalg.norm(np.asarray(cmd2['dir_a']) - app_a) * 1e6 < 1e-3


def test_incertidumbre_mayor_tarda_mas_en_adquirir(const):
    peor = simular(con(sigma_actitud_urad=3000.0, dur_s=40.0), const)
    n0, n1 = simular(con(dur_s=5.0), const).numeros(), peor.numeros()
    assert n1['t_adq_s'] is not None
    assert n1['t_adq_s'] > 10 * n0['t_adq_s']                    # barrido de ~3 mrad: decenas de s con el cardan
    assert n1['k_pasos'] > 30 * n0['k_pasos']


def test_conjunto_de_semillas_adquiere_siempre_con_1700_urad_de_actitud(const):
    """Regresion: con 1730 urad de incertidumbre de actitud, las 8 semillas adquieren en < 12 s (la
    espiral antigua dejaba un hueco central y 2 de 8 semillas nunca adquirian)."""
    ts = []
    for sem in range(1, 9):
        n = simular(con(semilla=sem, sigma_actitud_urad=1728.0, dur_s=14.0), const).numeros()
        assert n['t_adq_s'] is not None, sem
        ts.append(n['t_adq_s'])
    assert max(ts) < 12.0 and np.median(ts) > 0.5       # la mediana es del orden de segundos (MC: 0.75-2.5 s)


def test_tiempo_de_adquisicion_de_la_planta_contra_el_monte_carlo_de_la_espiral(const):
    """El tiempo hasta el apreton de la planta COMPLETA (efemerides con error, actitud, cardan,
    camara con ruido, un paso de 50 s por punto) sobre 150 semillas reproduce los cuartiles del
    Monte Carlo puro de `adquisicion.monte_carlo` (blanco gaussiano de sigma = incertidumbre del
    controlador) a ~30 %: la diferencia es el retardo de un paso del lockstep (0.05 s) y que el
    blanco del escenario sale de rotaciones y posiciones, no de una normal ideal. (Con 400 semillas:
    mediana 1.05 s contra 0.95 s, p90 3.40 contra 3.25.)"""
    from sat_isl import adquisicion as A
    from sat_isl.misiones import PlantaIsl
    cfg0 = con(sigma_actitud_urad=1728.0)
    ts = []
    for sem in range(1, 151):
        cfg = dataclasses.replace(cfg0, semilla=sem)
        pl, ct = PlantaIsl(cfg, const), ControladorIsl(cfg, const)
        t_adq = np.inf
        for _ in range(700):
            st = pl.medir()
            cmd = ct.paso(st['t'], st)
            if cmd['modo'] == 2:
                t_adq = st['t']
                break
            pl.avanzar(cfg.paso_s, cmd)
        ts.append(t_adq)
    ts = np.array(ts)
    tm = pl.term
    mc, info = A.monte_carlo(float(cmd['sigma_u_urad']), tm, tm, 5.49e6, n=4000, semilla=1)
    mc = mc * (tm.t_asiento_cardan_s / tm.cam_dwell_s)          # el barrido sale del FSM: 50 ms por punto
    for q in (25, 50, 75):
        a, b = np.percentile(ts[np.isfinite(ts)], q), np.percentile(mc[np.isfinite(mc)], q)
        assert abs(a - b) < 0.3 * b + 0.15, (q, a, b)
    assert np.mean(np.isinf(ts)) < 0.03


def test_detector_apd_no_cierra_donde_el_coherente_si(const):
    """Misma geometria y misma potencia (-50 dBm): el coherente cierra 1.25 Gb/s, el APD (con 3 dB
    de margen) no cierra ni 10 Mb/s: la diferencia de sensibilidad a esa tasa es > 20 dB."""
    apd = simular(con(detector='apd', dur_s=10.0), const).numeros()
    assert apd['t_adq_s'] is not None                            # el seguimiento (QPD) es el mismo
    assert apd['cerrado_pct_visible'] == 0.0 and apd['tasa_cerrado_bps'] == 0.0
    # con mas potencia el APD si cierra, a una tasa mucho menor
    apd2 = simular(con(detector='apd', p_tx_w=20.0, dur_s=10.0), const).numeros()
    assert apd2['cerrado_pct_visible'] > 80.0
    assert 0.0 < apd2['tasa_cerrado_bps'] < 1e9


def test_enlace_debil_pierde_el_seguimiento_y_reintenta(const):
    """Con 0.1 mW el QPD no tiene senal (NEA > theta/3): el seguimiento se declara perdido y el
    controlador vuelve a buscar una y otra vez; nunca hay enlace."""
    n = simular(con(p_tx_w=1e-4, dur_s=8.0), const).numeros()
    assert n['cerrado_pct_visible'] == 0.0
    assert n['perdidas_seguimiento'] >= 3
    assert n['t_adq_s'] is None


def test_par_de_planos_distintos_pierde_la_visibilidad_y_el_enlace(const):
    """P0S0 - P2S2 deja de verse (la Tierra + 100 km tapa el rayo) hacia t = 1050 s: el enlace se
    cierra solo mientras es visible y el controlador vuelve a ESPERA."""
    cfg = con(sat_b=18, inicio_s=1020.0, dur_s=60.0)
    med = simular(cfg, constelacion_de(cfg))
    vis = np.array([s['visible'] for s in med.est])
    cer = np.array([s['cerrado'] for s in med.est])
    modo = np.array([s['modo'] for s in med.est])
    assert vis[0] and not vis[-1]
    assert not np.any(cer & ~vis)                                # nunca cerrado sin ver
    t_fin = int(np.argmax(~vis))
    assert 1050.0 < cfg.inicio_s + t_fin * cfg.paso_s < 1080.0   # la ventana termina entre 1050 y 1080 s (rejilla de 30 s)
    assert np.all(modo[t_fin + 2:] == MODO_ESPERA)
    # el controlador apaga el enlace con SU estimacion de visibilidad (error de efemerides 2.1 km),
    # unos segundos antes que la verdad: 10 s antes aun hay seguimiento
    assert modo[t_fin - 200] == MODO_SEGUIMIENTO
    n = med.numeros()
    assert n['cerrado_pct_corrida'] < n['visible_pct'] < 100.0
    assert n['cortes_visibilidad'] == 1 and n['perdidas_seguimiento'] == 0
    assert n['rango_km'][0] > 5000.0 and n['adelanto_urad'] > 20.0


def test_tabla_de_vecinos():
    cfg = Config(inicio_s=0.0, dur_s=3000.0)
    cons = constelacion_de(cfg)
    filas = tabla_vecinos(cons, 0, dur_s=3000.0, paso_s=60.0)
    por = {f['a']: f for f in filas}
    assert len(filas) == 4
    for nombre in ('P0S1', 'P0S7'):                              # del plano: siempre visibles, ~5.5 Mm
        f = por[nombre]
        assert f['visible_pct'] == 100.0 and 5400 < f['rango_min_km'] < f['rango_max_km'] < 5600
        assert math.isclose(f['tasa_los_max_mrad_s'], 1.04, abs_tol=0.03)       # la LOS gira a la tasa orbital
    otros = [f for f in filas if f['a'] not in ('P0S1', 'P0S7')]
    assert all(f['visible_pct'] < 60.0 for f in otros)
    assert all(f['tasa_los_max_mrad_s'] > 3.0 for f in otros)    # 4x mas rapido: el seguimiento grueso es mas duro
