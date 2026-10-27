"""Validacion cientifica de planificador.py (equipo F, Red y planificacion).

Los TLE de estas pruebas estan EMBEBIDOS (epoca 2026-10-01): el resultado no
depende de que data/tle/ se refresque. Instancias sinteticas para la parte
combinatoria (con optimo conocido), instancias reales SGP4 para las ventanas,
el enlace y el volumen.
"""
import datetime as dt
import itertools
import json
import math
import random

import numpy as np
import pytest

from atp_ros import model
from atp_ros import orbit as o
from atp_ros import planificador as P

UTC = dt.timezone.utc

TLES = {
    'ISS (ZARYA)': (
        '1 25544U 98067A   26274.82022115  .00003852  00000+0  78825-4 0  9993',
        '2 25544  51.6316 132.3693 0006937 211.2109 148.8469 15.48706258588289'),
    'NOAA 19': (
        '1 33591U 09005A   26274.93802951 -.00000008  00000+0  19766-4 0  9994',
        '2 33591  98.9421 345.8364 0014804  88.9251 271.3617 14.13487659909563'),
    'METEOR-M2 3': (
        '1 57166U 23091A   26274.96711575 -.00000033  00000+0  41975-5 0  9999',
        '2 57166  98.5968 327.6972 0003394 204.5400 155.5618 14.24053366169711'),
}
T0 = dt.datetime(2026, 10, 2, 12, 0, 0, tzinfo=UTC)
ESTACIONES = {
    'cdmx': o.Estacion('Ciudad de Mexico', 19.4326, -99.1332, 2240.0, 5.0),
    'madrid': o.Estacion('Madrid', 40.4168, -3.7038, 650.0, 5.0),
    'tromso': o.Estacion('Tromso', 69.6496, 18.9560, 20.0, 5.0),
}


def _tles(nombres=None):
    return [o.tle_desde_lineas(n, *TLES[n]) for n in (nombres or TLES)]


@pytest.fixture(scope='module')
def red():
    """3 satelites x 3 estaciones, 12 h: ~30 ventanas reales (SGP4)."""
    tles = _tles()
    rotor = P.Rotor()
    vs = P.construir_ventanas(tles, ESTACIONES, T0, 12.0, P.PerfilEnlace(), rotor)
    return tles, rotor, vs


# ---------------------------------------------------------------- MODCOD y ACM

def test_modcod_tabla_etsi():
    """28 MODCOD de la Tabla 13 de ETSI EN 302 307 V1.2.1; valores puntuales
    copiados del texto del PDF oficial."""
    assert len(P.MODCOD_DVBS2) == 28
    tab = {n: (e, eta) for n, e, eta in P.MODCOD_DVBS2}
    assert tab['QPSK 1/4'] == (-2.35, 0.490243)
    assert tab['QPSK 1/2'] == (1.00, 0.988858)
    assert tab['8PSK 3/4'] == (7.91, 2.228124)
    assert tab['16APSK 2/3'] == (8.97, 2.637201)
    assert tab['32APSK 9/10'] == (16.05, 4.453027)


def test_modcod_bajo_shannon():
    """Ningun MODCOD puede superar la capacidad de Shannon a su propia Es/N0
    (log2(1 + Es/N0) bit/simbolo): es la comprobacion de que la tabla y la
    cota estan en las mismas unidades. La brecha tipica de DVB-S2 es ~0.7 dB."""
    for nombre, req, eta in P.MODCOD_DVBS2:
        cap = math.log2(1.0 + 10.0 ** (req / 10.0))
        assert eta < cap, nombre
        # y no demasiado lejos: a Es/N0 req + 1.5 dB Shannon ya lo supera
        assert eta > 0.0
        assert math.log2(1.0 + 10.0 ** ((req + 1.5) / 10.0)) > eta, nombre


def test_elegir_modcod():
    idx, eta = P.elegir_modcod(1.0)                       # justo QPSK 1/2
    assert P.MODCOD_DVBS2[idx][0] == 'QPSK 1/2' and eta == pytest.approx(0.988858)
    idx, eta = P.elegir_modcod(0.99)                      # una pizca menos: QPSK 2/5
    assert P.MODCOD_DVBS2[idx][0] == 'QPSK 2/5'
    assert P.elegir_modcod(-3.0) == (-1, 0.0)             # no cierra ni QPSK 1/4
    idx, eta = P.elegir_modcod(25.0)
    assert P.MODCOD_DVBS2[idx][0] == '32APSK 9/10'
    # con margen: 16.05 + 1 = 17.05 dB para el ultimo
    assert P.MODCOD_DVBS2[P.elegir_modcod(17.0, 1.0)[0]][0] == '32APSK 8/9'
    assert P.MODCOD_DVBS2[P.elegir_modcod(17.06, 1.0)[0]][0] == '32APSK 9/10'
    # vectorizado = escalar; la eficiencia nunca baja al subir Es/N0
    x = np.linspace(-4, 20, 241)
    ix, ex = P.elegir_modcod(x)
    assert all(P.elegir_modcod(float(v))[1] == e for v, e in zip(x, ex))
    assert np.all(np.diff(ex) >= 0)
    # 16APSK 2/3 (8.97 dB) domina a 8PSK 5/6 (9.35 dB): a 9.0 dB no se elige 8PSK
    assert P.MODCOD_DVBS2[P.elegir_modcod(9.0)[0]][0] == '16APSK 2/3'


def test_enlace_a_mano():
    """C/N0 contra el calculo a mano con las formulas de definicion (FSPL =
    20 log(4 pi d f / c), G = eta (pi D f / c)^2, k = 1.380649e-23)."""
    pf = P.PerfilEnlace()
    d_km, f = 1000.0, 8.4e9
    fspl = 20 * math.log10(4 * math.pi * d_km * 1e3 * f / model.C_LUZ_M_S)
    g = 10 * math.log10(0.6 * (math.pi * 3.0 * f / model.C_LUZ_M_S) ** 2)
    cn0 = (12.0 - fspl - 3.0 + g - 10 * math.log10(150.0)
           - 10 * math.log10(1.380649e-23))
    assert float(P.cn0_enlace(d_km, pf)) == pytest.approx(cn0, abs=0.01)
    esn0 = cn0 - 70.0
    assert float(P.tasas_enlace(d_km, pf)['esn0_db'][0]) == pytest.approx(esn0, abs=0.01)
    assert esn0 > 16.05 + 1.0                              # ~21 dB: el MODCOD tope
    t = P.tasas_enlace([1000.0, 2500.0], pf)
    assert t['tasa_bps'][0] == pytest.approx(10e6 * 4.453027)
    assert t['tasa_bps'][1] < t['tasa_bps'][0]             # mas lejos, menos tasa
    assert np.all(t['tasa_bps'] <= t['shannon_bps'])


# ---------------------------------------------------------------- ventanas reales

def test_ventanas_coinciden_con_predecir_pases(red):
    tles, _, vs = red
    for tle in tles:
        for clave, est in ESTACIONES.items():
            ref = o.predecir_pases(tle, est, T0, 12.0, est.mascara_deg)
            mias = [v for v in vs if v.sat == tle.nombre and v.estacion == clave]
            assert len(mias) == len(ref), (tle.nombre, clave)
            for v, p in zip(mias, ref):
                assert v.aos == p['aos'] and v.los == p['los']
                assert v.el_max_deg == pytest.approx(p['el_max_deg'])
                assert v.duracion_s == pytest.approx(p['duracion_s'])
                assert v.az_aos_deg == pytest.approx(p['az_aos_deg'], abs=0.01)
                assert v.az_los_deg == pytest.approx(p['az_los_deg'], abs=0.01)
    assert len(vs) > 20
    assert [v.id for v in vs] == list(range(len(vs)))
    keys = [(v.aos, v.sat, v.estacion) for v in vs]
    assert keys == sorted(keys)                            # orden por AOS


def test_subred_equivale_a_construir_directo(red):
    tles, rotor, vs = red
    sub = P.subred(vs, estaciones={'cdmx', 'madrid'})
    directo = P.construir_ventanas(
        tles, {k: ESTACIONES[k] for k in ('cdmx', 'madrid')}, T0, 12.0,
        P.PerfilEnlace(), rotor)
    assert [v.id for v in sub] == list(range(len(sub)))
    assert [(v.sat, v.estacion, v.aos, v.los, v.vol_bits) for v in sub] == \
        [(v.sat, v.estacion, v.aos, v.los, v.vol_bits) for v in directo]
    assert len(P.subred(vs)) == len(vs)
    solo = P.subred(vs, satelites={'NOAA 19'})
    assert solo and {v.sat for v in solo} == {'NOAA 19'}
    # no toca las ventanas originales
    assert [v.id for v in vs] == list(range(len(vs)))


def test_tromso_no_ve_la_iss(red):
    _, _, vs = red
    assert not [v for v in vs if v.sat == 'ISS (ZARYA)' and v.estacion == 'tromso']
    assert [v for v in vs if v.sat == 'NOAA 19' and v.estacion == 'tromso']


def test_elevacion_minima_util():
    tle = _tles(['NOAA 19'])
    est = {'madrid': ESTACIONES['madrid']}
    v0 = P.construir_ventanas(tle, est, T0, 12.0)
    v20 = P.construir_ventanas(tle, est, T0, 12.0, el_min_deg=20.0)
    assert len(v20) < len(v0)
    assert all(v.el_max_deg >= 20.0 for v in v20)
    # una ventana a mascara 20 esta contenida en la de mascara 5 y es mas corta
    for v in v20:
        w = next(x for x in v0 if x.aos <= v.aos and v.los <= x.los)
        assert v.duracion_s < w.duracion_s


# ---------------------------------------------------------------- volumen

def _volumen_independiente(tle, est, v, dt_fino=0.25):
    """Integral de la tasa ACM con codigo propio de la prueba: rango de
    `orbit.observar`, formulas de definicion y la tabla recorrida en bucle
    Python; regla del punto medio en una malla 4 veces mas fina."""
    f, rs = 8.4e9, 10e6
    dur = (v.los - v.aos).total_seconds()
    n = int(round(dur / dt_fino))
    seg = (np.arange(n) + 0.5) * (dur / n)
    t0 = o.a_jd(v.aos)
    jd = np.full(n, t0[0])
    fr = t0[1] + seg / 86400.0
    d = o.observar(tle, est, jd, fr)['rango_km']
    total, shannon = 0.0, 0.0
    for dk in d:
        fspl = 20 * math.log10(4 * math.pi * dk * 1e3 * f / model.C_LUZ_M_S)
        g = 10 * math.log10(0.6 * (math.pi * 3.0 * f / model.C_LUZ_M_S) ** 2)
        cn0 = (12.0 - fspl - 3.0 + g - 10 * math.log10(150.0)
               - 10 * math.log10(1.380649e-23))
        esn0 = cn0 - 10 * math.log10(rs)
        eta = 0.0
        for _, req, e in P.MODCOD_DVBS2:
            if esn0 >= req + 1.0 and e > eta:
                eta = e
        total += rs * eta * (dur / n)
        shannon += rs * math.log2(1 + 10 ** (esn0 / 10)) * (dur / n)
    return total, shannon


def test_volumen_coincide_con_integracion_independiente(red):
    tles, _, vs = red
    por_nombre = {t.nombre: t for t in tles}
    # un pase alto, uno bajo y uno medio, de satelites distintos
    elegidas = [max(vs, key=lambda v: v.el_max_deg),
                min((v for v in vs if v.duracion_s > 200), key=lambda v: v.el_max_deg),
                next(v for v in vs if v.sat == 'ISS (ZARYA)')]
    for v in elegidas:
        vol, shan = _volumen_independiente(por_nombre[v.sat], ESTACIONES[v.estacion], v)
        assert v.vol_bits == pytest.approx(vol, rel=0.005), (v.sat, v.estacion)
        assert v.vol_shannon_bits == pytest.approx(shan, rel=0.005)
        assert v.vol_bits < v.vol_shannon_bits


def test_volumen_mismo_muestreo_exacto(red):
    """Con las MISMAS muestras, una reimplementacion escalar da el mismo
    numero (descarta errores de unidades y de indexacion)."""
    tles, _, vs = red
    v = vs[len(vs) // 2]
    tle = next(t for t in tles if t.nombre == v.sat)
    est = ESTACIONES[v.estacion]
    p = o.predecir_pases(tle, est, T0, 12.0, est.mascara_deg)
    pase = min(p, key=lambda x: abs((x['aos'] - v.aos).total_seconds()))
    s = o.muestrear_pase(tle, est, pase, 1.0)
    tasa = []
    for dk in s['d']:
        r = model.presupuesto_enlace(float(dk), 0.0, f_hz=8.4e9, eirp_dbw=12.0,
                                     perdidas_db=3.0, tasa_bps=1.0)
        esn0 = r['cn0_dbhz'] - 70.0
        tasa.append(10e6 * max([e for _, req, e in P.MODCOD_DVBS2
                                if esn0 >= req + 1.0], default=0.0))
    vol = sum(0.5 * (tasa[i] + tasa[i + 1]) * (s['t'][i + 1] - s['t'][i])
              for i in range(len(tasa) - 1))
    assert v.vol_bits == pytest.approx(vol, rel=1e-9)


def test_volumen_nunca_supera_shannon(red):
    _, _, vs = red
    assert all(0.0 < v.vol_bits < v.vol_shannon_bits for v in vs)
    # sin margen y con un enlace pobre tambien, en los dos extremos de EIRP
    for eirp in (-10.0, 12.0, 30.0):
        r = P.tasas_enlace(np.linspace(500, 3000, 50),
                           P.PerfilEnlace(eirp_dbw=eirp, margen_acm_db=0.0))
        assert np.all(r['tasa_bps'] <= r['shannon_bps'] + 1e-6)


def test_volumen_escala_con_eirp_y_simbolos(red):
    tle = _tles(['NOAA 19'])
    est = {'madrid': ESTACIONES['madrid']}
    base = P.construir_ventanas(tle, est, T0, 12.0)[0]
    # sin enlace (EIRP muy bajo): volumen 0
    nada = P.construir_ventanas(tle, est, T0, 12.0,
                                P.PerfilEnlace(eirp_dbw=-60.0))[0]
    assert nada.vol_bits == 0.0 and nada.modcod_max == 'sin enlace'
    # el doble de simbolos y EIRP saturado (mismo MODCOD tope): el doble de volumen
    a = P.construir_ventanas(tle, est, T0, 12.0,
                             P.PerfilEnlace(eirp_dbw=60.0, rs_baud=1e6))[0]
    b = P.construir_ventanas(tle, est, T0, 12.0,
                             P.PerfilEnlace(eirp_dbw=60.0, rs_baud=2e6))[0]
    assert b.vol_bits == pytest.approx(2.0 * a.vol_bits, rel=1e-9)
    assert a.vol_bits == pytest.approx(1e6 * 4.453027 * a.duracion_s, rel=1e-6)
    assert base.vol_bits > 0


def test_adquisicion_resta_volumen():
    tle = _tles(['NOAA 19'])
    est = {'madrid': ESTACIONES['madrid']}
    a = P.construir_ventanas(tle, est, T0, 12.0,
                             P.PerfilEnlace(eirp_dbw=60.0))[0]
    b = P.construir_ventanas(tle, est, T0, 12.0,
                             P.PerfilEnlace(eirp_dbw=60.0, t_adq_s=30.0))[0]
    assert b.vol_bits == pytest.approx(a.vol_bits * (a.duracion_s - 30.0)
                                       / a.duracion_s, rel=2e-3)


# ---------------------------------------------------------------- keyhole

def test_keyhole_marca_recorta_y_descarta():
    iss = _tles(['ISS (ZARYA)'])[0]
    tt = T0 + dt.timedelta(hours=3)
    s = o.observar_en(iss, o.Estacion('x', 0, 0, 0, 5), tt)
    lat, lon, _ = o.ecef_a_geodetica(s['r_ecef_km'])
    pf = P.PerfilEnlace()

    def pase(dlon, modo):
        est = {'x': o.Estacion('x', lat, lon + dlon, 0.0, 5.0)}
        vs = P.construir_ventanas([iss], est, T0, 6.0, pf, P.Rotor(keyhole=modo))
        return [v for v in vs if abs((v.aos - tt).total_seconds()) < 900]

    cenital = pase(0.0, 'marcar')[0]
    assert cenital.el_max_deg > 89.0 and cenital.keyhole
    assert cenital.az_pto_max_deg_s > 6.0 and cenital.t_perdido_s > 0.0
    # el keyhole cerrado de model.elevacion_keyhole a la altitud de la ISS
    assert cenital.el_keyhole_deg == pytest.approx(
        model.elevacion_keyhole(417.0, 6.0), abs=0.2)
    recortado = pase(0.0, 'recortar')[0]
    assert recortado.vol_bits < cenital.vol_bits          # se pierde el tramo
    assert recortado.vol_bits == pytest.approx(
        cenital.vol_bits - cenital.t_perdido_s * 10e6 * 4.453027, rel=0.02)
    assert pase(0.0, 'descartar') == []
    # un pase bajo: sin marca ni perdida
    bajo = pase(4.0, 'marcar')[0]
    assert not bajo.keyhole and bajo.t_perdido_s == 0.0 and bajo.az_pto_max_deg_s < 6.0


def test_keyhole_cerrado_coincide_con_la_demanda_real():
    """La marca de `model.elevacion_keyhole` (orbita circular) debe coincidir
    con la demanda de acimut de la trayectoria SGP4, salvo en +-1.5 grados de
    elevacion alrededor del umbral (la orbita real no es circular)."""
    iss = _tles(['ISS (ZARYA)'])[0]
    tt = T0 + dt.timedelta(hours=3)
    s = o.observar_en(iss, o.Estacion('x', 0, 0, 0, 5), tt)
    lat, lon, _ = o.ecef_a_geodetica(s['r_ecef_km'])
    n = 0
    for dlon in (0.0, 0.1, 0.3, 0.6, 1.0, 1.5, 2.5, 4.0):
        est = {'x': o.Estacion('x', lat, lon + dlon, 0.0, 5.0)}
        for v in P.construir_ventanas([iss], est, T0, 6.0):
            if abs((v.aos - tt).total_seconds()) > 900:
                continue
            n += 1
            if abs(v.el_max_deg - v.el_keyhole_deg) < 1.5:
                continue
            assert v.keyhole == (v.az_pto_max_deg_s > 6.0), (dlon, v.el_max_deg)
    assert n >= 6


# ---------------------------------------------------------------- restricciones

def _slew(a, b, rotor):
    """Re-apuntamiento escrito aparte, en la prueba."""
    daz = abs(a.az_los_deg - b.az_aos_deg)
    daz = min(daz, 360.0 - daz)
    d_el = abs(a.el_los_deg - b.el_aos_deg)
    return max(daz / rotor.vel_az_deg_s, d_el / rotor.vel_el_deg_s) + rotor.t_asent_s


def _revisar_a_mano(ids, vs, rotor):
    sel = [vs[i] for i in ids]
    for clave in {v.estacion for v in sel}:
        lst = sorted((v for v in sel if v.estacion == clave), key=lambda v: v.t_aos_s)
        for a, b in zip(lst, lst[1:]):
            assert b.t_aos_s >= a.t_los_s, 'solape en la antena'
            assert b.t_aos_s - a.t_los_s >= _slew(a, b, rotor) - 1e-6, 're-apuntamiento'
    for sat in {v.sat for v in sel}:
        lst = [v for v in sel if v.sat == sat]
        for a, b in itertools.combinations(lst, 2):
            assert a.t_los_s <= b.t_aos_s or b.t_los_s <= a.t_aos_s, 'satelite doble'


def test_planes_reales_no_violan_restricciones(red):
    _, rotor, vs = red
    planes = [P.planificar_codicioso(vs, rotor, c) for c in P.CRITERIOS]
    planes.append(P.planificar_optimo(vs, rotor, metodo='bb', limite_nodos=2000))
    for p in planes:
        assert p.ids == sorted(p.ids) and len(p.ids) > 5
        _revisar_a_mano(p.ids, vs, rotor)
        assert P.verificar_plan(p.ids, vs, rotor) == []
        assert p.valor == pytest.approx(sum(vs[i].peso for i in p.ids))


def _mk(i, sat, est, a, b, az_a=0.0, az_b=0.0, w=1.0, el_a=5.0, el_b=5.0):
    """Ventana sintetica (tiempos en s)."""
    t = T0 + dt.timedelta(seconds=a)
    return P.Ventana(
        id=i, sat=sat, norad=i, estacion=est, aos=t,
        los=T0 + dt.timedelta(seconds=b), t_aos_s=float(a), t_los_s=float(b),
        az_aos_deg=az_a, az_los_deg=az_b, el_aos_deg=el_a, el_los_deg=el_b,
        el_max_deg=45.0, rango_min_km=800.0, aos_truncado=False,
        los_truncado=False, keyhole=False, el_keyhole_deg=80.0, t_perdido_s=0.0,
        az_pto_max_deg_s=1.0, vol_bits=w * P.BITS_POR_GB,
        vol_shannon_bits=2 * w * P.BITS_POR_GB, modcod_max='QPSK 1/2',
        esn0_max_db=3.0, prioridad=1.0, peso=float(w))


def test_verificar_plan_detecta_violaciones(red):
    _, rotor, vs = red
    sint = [_mk(0, 'A', 'e1', 0, 600), _mk(1, 'B', 'e1', 300, 900),   # misma antena
            _mk(2, 'C', 'e2', 0, 600), _mk(3, 'C', 'e3', 100, 700),   # mismo satelite
            _mk(4, 'D', 'e4', 0, 100, az_b=180.0),
            _mk(5, 'E', 'e4', 120, 300, az_a=0.0)]                    # 20 s < 35 s
    assert any('solape' in m for m in P.verificar_plan([0, 1], sint, rotor))
    assert any('simultaneos' in m for m in P.verificar_plan([2, 3], sint, rotor))
    assert any('re-apuntamiento' in m for m in P.verificar_plan([4, 5], sint, rotor))
    assert P.verificar_plan([0], sint, rotor) == []
    assert 'ids repetidos' in P.verificar_plan([0, 0], sint, rotor)
    # un plan valido para un rotor rapido deja de serlo con uno lentisimo
    plan = P.planificar_codicioso(vs, rotor, 'aos')
    lento = P.Rotor(vel_az_deg_s=0.01, vel_el_deg_s=0.01)
    assert P.verificar_plan(plan.ids, vs, lento) != []


def test_reapuntamiento_camino_corto_y_tiempos():
    rotor = P.Rotor(vel_az_deg_s=6.0, vel_el_deg_s=3.0, t_asent_s=5.0)
    a = _mk(0, 'A', 'e', 0, 100, az_b=180.0)
    assert P.tiempo_reapuntamiento(a, _mk(1, 'B', 'e', 0, 1, az_a=0.0), rotor) \
        == pytest.approx(180 / 6 + 5)                      # 35 s
    # 350 -> 10 grados: 20 grados por el camino corto, no 340
    b = _mk(0, 'A', 'e', 0, 100, az_b=350.0)
    assert P.tiempo_reapuntamiento(b, _mk(1, 'B', 'e', 0, 1, az_a=10.0), rotor) \
        == pytest.approx(20 / 6 + 5)
    # el eje de elevacion (3 deg/s) manda si el salto es grande
    c = _mk(0, 'A', 'e', 0, 100, el_b=5.0)
    d = _mk(1, 'B', 'e', 0, 1, el_a=65.0)
    assert P.tiempo_reapuntamiento(c, d, rotor) == pytest.approx(60 / 3 + 5)
    # frontera: 34 s de hueco no basta (35 s), 36 s si
    x = _mk(0, 'A', 'e', 0, 100, az_b=180.0)
    assert not P.compatibles_estacion(x, _mk(1, 'B', 'e', 134, 200, az_a=0.0), rotor)
    assert P.compatibles_estacion(x, _mk(1, 'B', 'e', 136, 200, az_a=0.0), rotor)
    assert P.compatibles_estacion(x, _mk(1, 'B', 'e', 135, 200, az_a=0.0), rotor)
    # el mismo satelite en dos estaciones: choque solo si se solapan
    assert P.solapan_satelite(_mk(0, 'A', 'e', 0, 100), _mk(1, 'A', 'f', 50, 150))
    assert not P.solapan_satelite(_mk(0, 'A', 'e', 0, 100), _mk(1, 'A', 'f', 100, 150))
    assert P.solapan_satelite(_mk(0, 'A', 'e', 0, 100), _mk(1, 'A', 'f', 100, 150), 5.0)


def test_codicioso_cae_en_la_trampa_y_el_optimo_no():
    """Una ventana larga y pesada (w=10) tapa a dos que juntas valen mas
    (6 + 6): el codicioso por peso se queda con 10, el optimo con 12."""
    rotor = P.Rotor()
    vs = [_mk(0, 'A', 'e', 0, 1000, w=10.0),
          _mk(1, 'B', 'e', 0, 400, w=6.0),
          _mk(2, 'C', 'e', 500, 900, w=6.0)]
    g = P.planificar_codicioso(vs, rotor, 'peso')
    assert g.ids == [0] and g.valor == 10.0
    for metodo in ('bb', 'auto'):
        op = P.planificar_optimo(vs, rotor, metodo=metodo)
        assert op.ids == [1, 2] and op.valor == 12.0 and op.optimo_probado
    assert P.optimo_fuerza_bruta(vs, rotor).valor == 12.0


def test_choque_de_satelite_cambia_el_optimo():
    """Dos estaciones ven al mismo satelite a la vez: solo una se lo lleva."""
    rotor = P.Rotor()
    vs = [_mk(0, 'A', 'e1', 0, 600, w=5.0), _mk(1, 'A', 'e2', 10, 610, w=4.0)]
    op = P.planificar_optimo(vs, rotor, metodo='bb')
    assert op.ids == [0] and op.valor == 5.0 and op.optimo_probado


def _instancia_aleatoria(semilla, n=12, n_est=3, n_sat=4):
    rng = random.Random(semilla)
    vs = []
    for i in range(n):
        a = rng.uniform(0, 3000)
        d = rng.uniform(150, 900)
        vs.append(_mk(0, f'S{rng.randrange(n_sat)}', f'E{rng.randrange(n_est)}',
                      a, a + d, az_a=rng.uniform(0, 360), az_b=rng.uniform(0, 360),
                      w=rng.uniform(0.5, 10.0), el_a=rng.uniform(5, 30),
                      el_b=rng.uniform(5, 30)))
    vs.sort(key=lambda v: (v.t_aos_s, v.sat, v.estacion))
    for i, v in enumerate(vs):
        v.id = i
    return vs


def test_optimo_coincide_con_fuerza_bruta_y_supera_al_codicioso():
    rotor = P.Rotor()
    n_estrictos = 0
    for semilla in range(60):
        vs = _instancia_aleatoria(semilla, n=11 + semilla % 4)
        bf = P.optimo_fuerza_bruta(vs, rotor)
        bb = P.planificar_optimo(vs, rotor, metodo='bb')
        assert bb.optimo_probado
        assert bb.valor == pytest.approx(bf.valor, rel=1e-12), semilla
        assert P.verificar_plan(bb.ids, vs, rotor) == []
        for c in P.CRITERIOS:
            g = P.planificar_codicioso(vs, rotor, c)
            assert g.valor <= bb.valor + 1e-9
            assert P.verificar_plan(g.ids, vs, rotor) == []
            n_estrictos += g.valor < bb.valor - 1e-9
    assert n_estrictos > 5      # el codicioso falla de verdad en algunas instancias


def test_optimo_coincide_con_milp_si_hay_scipy():
    pytest.importorskip('scipy.optimize')
    rotor = P.Rotor()
    for semilla in range(30):
        vs = _instancia_aleatoria(100 + semilla, n=40, n_est=3, n_sat=5)
        bb = P.planificar_optimo(vs, rotor, metodo='bb', limite_nodos=200000)
        mi = P.planificar_optimo(vs, rotor, metodo='milp')
        assert bb.optimo_probado and mi.optimo_probado
        assert bb.valor == pytest.approx(mi.valor, rel=1e-9), semilla


def test_optimo_real_pequeno_contra_fuerza_bruta(red):
    """Instancia REAL (ventanas SGP4): las 15 primeras, 3 estaciones."""
    _, rotor, vs = red
    sub = vs[:15]
    for i, v in enumerate(sub):
        v.id = i
    try:
        bf = P.optimo_fuerza_bruta(sub, rotor)
        bb = P.planificar_optimo(sub, rotor, metodo='bb')
        assert bb.optimo_probado
        assert bb.valor == pytest.approx(bf.valor, rel=1e-12)
        assert bb.valor >= P.planificar_codicioso(sub, rotor).valor
    finally:
        for i, v in enumerate(vs):
            v.id = i


def test_optimo_real_completo_domina_al_codicioso(red):
    """En las ~50 ventanas reales: optimo (bb) >= mejor codicioso, y si hay
    scipy el MILP da el mismo valor."""
    _, rotor, vs = red
    bb = P.planificar_optimo(vs, rotor, metodo='bb', limite_nodos=50000)
    mejor = max(P.planificar_codicioso(vs, rotor, c).valor for c in P.CRITERIOS)
    assert bb.valor >= mejor - 1e-9
    assert bb.cota_sup >= bb.valor - 1e-9
    try:
        from scipy.optimize import milp          # noqa: F401
    except ImportError:
        return
    mi = P.planificar_optimo(vs, rotor, metodo='milp')
    assert mi.valor >= bb.valor - 1e-6
    if bb.optimo_probado:
        assert mi.valor == pytest.approx(bb.valor, rel=1e-9)


def test_prioridades_cambian_el_plan(red):
    tles, rotor, _ = red
    est = {'madrid': ESTACIONES['madrid']}
    a = P.construir_ventanas(tles, est, T0, 12.0)
    b = P.construir_ventanas(tles, est, T0, 12.0, prioridades={'ISS (ZARYA)': 100.0})
    assert [v.vol_bits for v in a] == [v.vol_bits for v in b]    # prioridad no toca el volumen
    pa = P.planificar_optimo(a, rotor, metodo='bb')
    pb = P.planificar_optimo(b, rotor, metodo='bb')
    assert sum(b[i].sat == 'ISS (ZARYA)' for i in pb.ids) >= \
        sum(a[i].sat == 'ISS (ZARYA)' for i in pa.ids)


def test_sin_ventanas():
    vs = P.construir_ventanas(_tles(['ISS (ZARYA)']), {'tromso': ESTACIONES['tromso']},
                              T0, 12.0)
    assert vs == []
    rotor = P.Rotor()
    assert P.planificar_codicioso(vs, rotor).ids == []
    op = P.planificar_optimo(vs, rotor, metodo='bb')
    assert op.ids == [] and op.valor == 0.0 and op.optimo_probado


# ---------------------------------------------------------------- determinismo y JSON

def test_determinismo_con_fecha_fija():
    tles = _tles(['ISS (ZARYA)', 'NOAA 19'])
    est = {k: ESTACIONES[k] for k in ('cdmx', 'madrid')}
    out = [json.dumps(P.calcular_plan(tles, est, T0, 8.0, metodo='bb'),
                      sort_keys=True) for _ in range(2)]
    assert out[0] == out[1]
    d = json.loads(out[0])
    assert d['n_ventanas'] == len(d['ventanas']) > 3
    assert d['planes']['optimo']['valor'] >= max(
        d['planes'][f'codicioso_{c}']['valor'] for c in P.CRITERIOS) - 1e-6
    assert d['planes']['optimo']['optimo_probado'] is True
    assert 0.0 <= d['brecha_codicioso_pct'] <= 100.0


def test_resumen_consistente(red):
    _, rotor, vs = red
    p = P.planificar_codicioso(vs, rotor)
    r = P.resumen(p.ids, vs)
    assert r['contactos'] == len(p.ids)
    assert r['gb'] == pytest.approx(sum(vs[i].vol_gb for i in p.ids), abs=0.002)
    assert sum(x['gb'] for x in r['por_satelite'].values()) == pytest.approx(r['gb'], abs=0.01)
    assert sum(x['gb'] for x in r['por_estacion'].values()) == pytest.approx(r['gb'], abs=0.01)
    assert sum(x['contactos'] for x in r['por_estacion'].values()) == r['contactos']
    assert r['gb'] <= r['gb_shannon']
    assert r['minutos'] == pytest.approx(
        sum(vs[i].duracion_s for i in p.ids) / 60.0, abs=0.01)


def test_estaciones_y_catalogo_del_repo():
    """Camino completo con data/ del repositorio (si esta disponible)."""
    try:
        est, _ = o.cargar_estaciones()
        cat = o.cargar_catalogo()
    except FileNotFoundError:
        pytest.skip('data/ no disponible')
    assert {'cdmx', 'tromso', 'ecuador', 'madrid'} <= set(est)
    tle = o.buscar_tle(cat, 'NOAA 19')
    vs = P.construir_ventanas([tle], {'ecuador': est['ecuador']}, T0, 6.0)
    assert vs and all(v.el_max_deg >= est['ecuador'].mascara_deg for v in vs)
