"""Pruebas del enlace del rover (R7): presupuesto, orbita, horizonte, retardo de luz, ventanas, DTN y filtro. Sin ROS."""
import json
import math
import os

import numpy as np
import pytest

from atp_rover import comunicacion as C
from atp_rover.planetas import PLANETAS
from atp_rover.terreno import Terreno


def test_fspl_coincide_con_la_formula_de_atp_ros():
    # 92.45 + 20 log d_km + 20 log f_GHz; 1000 km a 1 GHz = 152.45 dB
    assert C.fspl_db(1000.0, 1e9) == pytest.approx(152.45)
    try:
        from atp_ros.model import fspl_db
    except Exception:
        return
    assert C.fspl_db(1234.5, 4.0e8) == pytest.approx(fspl_db(1234.5, 4.0e8))


def test_perdida_de_apuntamiento_y_ganancias():
    assert C.perdida_apuntamiento_db(0.0, 10.0) == 0.0
    assert C.perdida_apuntamiento_db(5.0, 10.0) == pytest.approx(3.0)       # 12 (0.5)^2
    # helice: -3 dBi en el horizonte, 3 dBi en el cenit; monopolo: nulo cerca del cenit
    assert C.ganancia_rover_uhf_dbi('omni_helice', 0.0) == pytest.approx(-3.0)
    assert C.ganancia_rover_uhf_dbi('omni_helice', 90.0) == pytest.approx(3.0)
    assert C.ganancia_rover_uhf_dbi('omni_lineal', 90.0) < C.ganancia_rover_uhf_dbi('omni_lineal', 30.0)
    assert C.ganancia_rover_uhf_dbi('direccional', 20.0, error_apunt_deg=0.0) == pytest.approx(10.0)
    with pytest.raises(ValueError):
        C.ganancia_rover_uhf_dbi('parabola', 10.0)


def test_tasa_adaptativa_monotona_y_con_tope():
    prx = np.arange(-140.0, -90.0, 0.5)
    t, m = C.tasa_uhf(prx, 2.0)
    assert np.all(np.diff(t) >= 0)
    assert t[0] == 0.0 and t[-1] == 2048e3
    assert np.all(m[t > 0] >= 2.0)
    t2, _ = C.tasa_uhf(prx, 2.0, max_bps=256e3)
    assert t2.max() == 256e3
    # con un dB mas de margen exigido nunca sube la tasa
    t3, _ = C.tasa_uhf(prx, 3.0)
    assert np.all(t3 <= t)
    assert C.prob_perdida_trama(0.0) == pytest.approx(0.5)
    assert C.prob_perdida_trama(10.0) < 1e-6


def test_orbita_kepleriana_periodo_y_radio():
    for nombre in ('mro', 'odyssey'):
        el = C.RELES['marte'][nombre]
        gm = C.CUERPOS['marte']['gm']
        T = 2 * math.pi * math.sqrt(el['a_km'] ** 3 / gm)
        r0 = C.posicion_orbita(gm, el, [0.0])[:, 0]
        r1 = C.posicion_orbita(gm, el, [T])[:, 0]
        assert np.linalg.norm(r1 - r0) < 1e-3
        r = np.linalg.norm(C.posicion_orbita(gm, el, np.linspace(0, T, 50)), axis=0)
        assert r.min() >= el['a_km'] * (1 - el['e']) - 1e-6 and r.max() <= el['a_km'] * (1 + el['e']) + 1e-6
    # periodos de la guia JPL: MRO 1 h 52 min, Odyssey 1 h 58 min (a +-1 min)
    gm = C.CUERPOS['marte']['gm']
    for nombre, minutos in (('mro', 112.0), ('odyssey', 118.0)):
        a = C.RELES['marte'][nombre]['a_km']
        assert 2 * math.pi * math.sqrt(a ** 3 / gm) / 60.0 == pytest.approx(minutos, abs=1.0)


def test_horizonte_llano_y_crater():
    assert np.all(C.mascara_horizonte(None) == 0.0)
    assert C.mascara_horizonte(Terreno('llano', 0)).max() <= 0.0
    cr = Terreno('crater', 1, cx=0.0, cy=0.0, radio=15.0, profundidad=3.0)
    m = C.mascara_horizonte(cr)
    # el cuenco de 15 m x 3 m eleva el horizonte a unos 10-18 deg en todos los azimuts
    assert 8.0 < m.min() and m.max() < 20.0


def test_cuerpos_planetas_y_errores():
    assert set(C.CUERPOS) <= set(PLANETAS)
    with pytest.raises(KeyError):
        C.Enlace('titan')
    with pytest.raises(KeyError):
        C.Enlace('marte', 'noexiste')
    with pytest.raises(KeyError):
        C.Enlace('zzz')


def test_retardo_de_luz_marte():
    # oposicion de 2020-10-13 (~0.42 UA reales) y conjuncion de 2021-10-08: el modelo circular da fases correctas (+-)
    o = C.owlt_s('marte', C.jd_de_iso('2020-10-13')) / 60.0
    c = C.owlt_s('marte', C.jd_de_iso('2021-10-08')) / 60.0
    assert 3.5 < o < 5.5 and 19.0 < c < 22.5
    # rango anual completo del modelo circular: 0.5237 a 2.5237 UA
    jd = C.jd_de_iso('2026-01-01') + np.arange(0, 800)
    d = C.distancia_tierra_km('marte', jd) / C.UA_KM
    assert d.min() == pytest.approx(0.5237, abs=0.01) and d.max() == pytest.approx(2.5237, abs=0.01)
    assert C.jd_de_iso('2000-01-01T12:00:00') == pytest.approx(2451545.0)
    # luna: 384400 km -> 1.282 s; ida y vuelta coherente con PLANETAS
    assert C.owlt_s('luna', 2460000.0) == pytest.approx(1.2822, abs=1e-3)
    assert 2 * C.owlt_s('luna', 2460000.0) == pytest.approx(PLANETAS['luna']['retardo_tierra_s'], abs=0.01)
    # el retardo tipico de PLANETAS['marte'] (1500 s ida y vuelta) cae dentro del rango del modelo
    assert 2 * 0.5237 * C.UA_KM / C.C_KM_S < PLANETAS['marte']['retardo_tierra_s'] < 2 * 2.5237 * C.UA_KM / C.C_KM_S


def test_pases_marte_mro_ordenes_de_magnitud():
    e = C.Enlace('marte', 'mro', -4.6, 137.4)
    vol, npas = e.volumen_por_dia(6)
    assert 1.5 <= npas.mean() <= 3.5                # guia JPL: 2.3 sesiones por dia (Curiosity) y 1-3 cerca del ecuador
    assert 100e6 < vol.mean() < 1500e6              # cientos de Mbit por dia (498 Mbit/dia medidos con MRO, primer ano)
    ps = e.pases(0, 3 * 88775.0)
    assert all(3 * 60 < p['dur_s'] < 15 * 60 for p in ps)    # ~10 min por encima de 10 deg segun JPL
    assert all(p['el_max_deg'] >= e.el_min for p in ps)
    # Odyssey topa en 256 kbps: no puede pasar de eso
    o = C.Enlace('marte', 'odyssey', -4.6, 137.4)
    assert o.pases(0, 88775.0)[0]['tasa_pico_bps'] <= 256e3


def test_terreno_reduce_la_visibilidad():
    llano = C.Enlace('marte', 'mro', 0.0, 0.0)
    cr = C.Enlace('marte', 'mro', 0.0, 0.0, Terreno('crater', 2, cx=0.0, cy=0.0, radio=15.0, profundidad=3.0))
    vl, nl = llano.volumen_por_dia(6)
    vc, nc = cr.volumen_por_dia(6)
    assert vc.sum() < vl.sum()
    # con un relieve que eleva el horizonte a ~14 deg, un pase con maximo < 14 deg desaparece
    assert nc.sum() <= nl.sum()


def test_mas_ganancia_mas_datos_y_determinismo():
    a = C.Enlace('marte', 'mro', antena='omni_helice')
    b = C.Enlace('marte', 'mro', antena='omni_helice', ganancia_extra_db=6.0)
    c = C.Enlace('marte', 'mro', antena='omni_helice', ganancia_extra_db=-6.0)
    va, vb, vc = (x.volumen_por_dia(4)[0].sum() for x in (a, b, c))
    assert vc < va < vb
    assert a.volumen_por_dia(4)[0].tolist() == C.Enlace('marte', 'mro', antena='omni_helice').volumen_por_dia(4)[0].tolist()


def test_luna_reles_y_tierra_fija_en_el_cielo():
    near = C.Enlace('luna', 'polar_100km', 0.0, 0.0)
    far = C.Enlace('luna', 'polar_100km', 0.0, 180.0)
    t = np.arange(0, 28 * 86400.0, 600.0)
    mn, mf = near.muestras(t), far.muestras(t)
    assert mn['tierra_visible'].all() and not mf['tierra_visible'].any()      # cara visible vs cara oculta
    nrho = C.Enlace('luna', 'nrho_kepler', -89.5, 0.0)
    assert nrho.rele['e'] > 0.85 and 33000 < nrho.rele['a_km'] < 35500
    ps = nrho.pases(0, 14 * 86400.0, 120.0)
    assert len(ps) >= 1


def test_entrega_de_comandos_incluye_retardo_y_espera():
    e = C.Enlace('marte', 'mro', -4.6, 137.4, jd0=C.jd_de_iso('2026-10-07'))
    hor = 6 * 88775.0
    ow = e._owlt(1000.0)
    t_ent, via = e.entrega_comando(1000.0, hor, modo='retardo')
    assert t_ent == pytest.approx(1000.0 + ow) and via == 'retardo'
    t_v, via2 = e.entrega_comando(1000.0, hor)
    assert t_v is not None and via2 in ('rele', 'directo')
    assert t_v >= 1000.0 + ow
    # la latencia con ventanas nunca es menor que el retardo de luz y no pasa de ~2 soles de espera
    lat = [e.entrega_comando(t, hor)[0] - t for t in np.arange(0, 3 * 88775.0, 1800.0)]
    assert min(lat) >= ow * 0.9 and max(lat) < ow + 2 * 88775.0


def test_cola_dtn_retransmite_y_descarta():
    rng = np.random.default_rng(0)
    q = C.ColaDTN(capacidad_bits=3e6)
    for i in range(5):
        q.agregar(1e6, float(i))
    assert q.descartados == 2 and len(q.q) == 3                       # solo caben 3 bundles de 1 Mbit
    ent = q.transmitir(1.5e6, 10.0, rng, 0.0)
    assert [e[0] for e in ent] == [3] and len(q.q) == 2               # el 1.5 Mbit da para 1 y medio
    # con perdida certera nada se entrega y se cuentan las retransmisiones
    q2 = C.ColaDTN(1e9)
    q2.agregar(1e6, 0.0)
    assert q2.transmitir(3.5e6, 1.0, rng, 1.0) == [] and q2.retransmisiones >= 1


def test_simular_dtn_determinista_y_conserva_bits():
    e = C.Enlace('marte', 'mro', -4.6, 137.4)
    a = C.simular_dtn(e, 6 * 88775.0, 200e6, semilla=3)
    b = C.simular_dtn(e, 6 * 88775.0, 200e6, semilla=3)
    assert a == b
    assert a['bits_entregados_por_dia'] <= a['bits_generados_por_dia'] * 1.001
    assert a['latencia_mediana_h'] is not None and a['latencia_mediana_h'] > 0
    # la latencia incluye al menos el retardo de luz
    assert a['latencia_mediana_h'] * 3600 > e._owlt(0.0)


def test_filtro_retrasa_y_pierde_de_forma_determinista():
    e = C.Enlace('luna', 'polar_100km', -89.5, 0.0)
    f = C.FiltroEnlace(e, 'retardo', semilla=1)
    f.cmd(0.0, (0.5, 0.0))
    c, _ = f.listos(0.5)
    assert c == []                                            # 1.28 s de retardo de luz
    c, _ = f.listos(1.3)
    assert c == [(pytest.approx(1.2822, abs=1e-3), (0.5, 0.0))]
    ideal = C.FiltroEnlace(e, 'ideal')
    ideal.cmd(2.0, (1.0, 0.1))
    assert ideal.listos(2.0)[0] == [(2.0, (1.0, 0.1))]

    def corrida(sem):
        g = C.FiltroEnlace(e, 'retardo', p_perdida_cmd=0.3, semilla=sem)
        for k in range(200):
            g.cmd(float(k), (float(k), 0.0))
        return g.stats['cmd_perdidos'], [v for _, v in g.listos(1e4)[0]]
    n1, v1 = corrida(5)
    assert (n1, v1) == corrida(5)
    assert 30 < n1 < 90 and len(v1) == 200 - n1
    assert v1 == sorted(v1)                                   # se entregan en orden


def test_filtro_ventanas_guarda_telemetria_hasta_el_contacto():
    e = C.Enlace('marte', 'mro', -4.6, 137.4)
    f = C.FiltroEnlace(e, 'ventanas', horizonte_s=4 * 88775.0)
    est = json.dumps({'t': 0.0, 'x': 1.0})
    t_ent = f.estado(0.0, est)
    assert t_ent is not None and t_ent > e._owlt(0.0)
    assert f.listos(10.0)[1] == [] and f.pendientes()[1] == 1
    assert len(f.listos(t_ent + 1.0)[1]) == 1
    # fuera del horizonte: se cuenta y se descarta
    assert f.estado(4 * 88775.0 + 1.0, est) is None and f.stats['est_no_llegan'] == 1
    with pytest.raises(ValueError):
        C.FiltroEnlace(e, 'otro')


def test_cli_enlace_determinista(tmp_path):
    cfg = dict(planeta='marte', rele='mro', dias=2)
    a = C.correr_enlace(cfg, str(tmp_path / 'a'))
    b = C.correr_enlace(cfg, str(tmp_path / 'b'))
    assert a == b
    for f in ('enlace.json', 'metrics.json'):
        assert open(tmp_path / 'a' / f, 'rb').read() == open(tmp_path / 'b' / f, 'rb').read()
    assert a['pases_por_dia'] > 0 and os.path.exists(tmp_path / 'a' / 'metrics.json')


def test_main_acepta_argumentos_con_espacio_y_igual(tmp_path):
    # el backend de la app pasa `--clave valor` (estilo cli) y valores negativos como `--clave=-4.6`
    assert C.main(['--salida', str(tmp_path), '--id', 'x', '--planeta', 'luna', '--rele', 'polar_100km', '--lat_deg=-89.5',
                   '--lon_deg', '0', '--dias', '1', '--fecha', '2026-10-07']) == 0
    m = json.load(open(tmp_path / 'x' / 'metrics.json'))
    assert m['planeta'] == 'luna' and m['pases_por_dia'] > 5


def test_escenas_enlace_validas_para_el_catalogo():
    yaml = pytest.importorskip('yaml')
    raiz = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    for n in ('rover-enlace-marte', 'rover-enlace-luna'):
        d = yaml.safe_load(open(os.path.join(raiz, 'experiments', n + '.yaml')))
        assert d['kind'] == 'proceso' and d['area'] == 'rf' and d['hecho_cuando'] == 'metrics.json'
        assert set(d['args']) <= set(C.DEFECTOS_ENLACE)


def test_estudio_funciones_basicas():
    import importlib.util
    ruta = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'scripts',
                        'estudio_enlace_rover.py')
    if not os.path.exists(ruta):
        pytest.skip('sin scripts/')
    spec = importlib.util.spec_from_file_location('estudio_enlace_rover', ruta)
    est = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(est)
    r = est.e1_base(3)
    assert r['mro']['sesiones_dia'] > 0 and r['odyssey']['publicado']['mbit_dia'] == 173.0
    x = est.e7_directo_x(4)
    assert x['filas'][0]['hga_kbps'] >= x['filas'][0]['lga_kbps']
