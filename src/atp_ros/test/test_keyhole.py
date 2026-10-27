"""Estrategias de keyhole (atp_ros.keyhole,: cada cifra contra
su contraste analitico o su definicion, y el comportamiento por defecto intacto.
Sin ROS salvo la prueba del generador (importorskip)."""
import math

import numpy as np
import pytest

from atp_ros import keyhole as K
from atp_ros import model

MASK_CORTA = 60.0     # pases de ~100 s: el keyhole ocurre en la culminacion y la prueba es rapida


def _perfil(el_max, mask=MASK_CORTA, n=3000):
    return model.perfil_pase(550.0, el_max, mask, 140.0, n=n)


# ---------------------------------------------------------------- haces y geometria

def test_ancho_de_haz_por_banda():
    # plato de 3 m: 70 lambda/D (model.ancho_haz); Yagi de 15 dBi: D = 2.31 lambda -> 30.3 deg
    assert K.ancho_haz_banda('S') == pytest.approx(model.ancho_haz(3.0, 2.2e9))
    assert K.ancho_haz_banda('S') == pytest.approx(3.18, abs=0.01)
    assert K.ancho_haz_banda('X') == pytest.approx(0.833, abs=0.005)
    d = K.diametro_equivalente_yagi(437e6)
    assert d / (model.C_LUZ_M_S / 437e6) == pytest.approx(2.31, abs=0.01)
    assert K.ancho_haz_banda('UHF') == pytest.approx(30.3, abs=0.2)
    # una Yagi de 15 dBi tiene la misma apertura en lambdas en VHF y UHF
    assert K.ancho_haz_banda('VHF') == pytest.approx(K.ancho_haz_banda('UHF'), rel=1e-6)
    assert K.semihaz_3db('S') == pytest.approx(K.ancho_haz_banda('S') / 2)
    # en el semihaz el haz pierde 3 dB (model.perdida_apuntamiento)
    assert model.perdida_apuntamiento(K.semihaz_3db('X'), K.ancho_haz_banda('X')) == pytest.approx(3.0)


def test_a_cielo_y_separacion():
    # la misma direccion con las dos ramas de la montura
    az, el = K.a_cielo(30.0 + 180.0, 180.0 - 70.0)
    assert (az, el) == (pytest.approx(30.0 + 360.0), pytest.approx(70.0))
    assert K.separacion_deg(30.0, 70.0, 30.0 + 360.0, 70.0) == pytest.approx(0.0, abs=1e-9)
    assert K.error_apuntamiento(30.0, 70.0, 210.0, 110.0) == pytest.approx(0.0, abs=1e-9)
    # sin flip es la identidad
    assert K.error_apuntamiento(30.0, 70.0, 30.0, 70.0) == pytest.approx(0.0, abs=1e-9)
    # dos puntos a 90 grados de acimut sobre el horizonte, y el cenit
    assert K.separacion_deg(0.0, 0.0, 90.0, 0.0) == pytest.approx(90.0)
    assert K.separacion_deg(0.0, 0.0, 123.0, 90.0) == pytest.approx(90.0)
    # cerca del cenit un error de acimut PEQUENO vale ~ D cos(el); uno grande es la cuerda
    assert K.separacion_deg(0.0, 89.0, 10.0, 89.0) == pytest.approx(10.0 * math.cos(math.radians(89.0)), rel=0.01)
    assert K.separacion_deg(0.0, 89.0, 60.0, 89.0) == pytest.approx(1.0, abs=1e-3)      # triangulo equilatero


# ---------------------------------------------------------------- comportamiento por defecto

def test_ingenua_es_la_referencia_sin_cambios():
    p = _perfil(85.0)
    plan = K.planificar(p['t'], p['az_continuo'], p['el'], 'ingenua')
    assert plan['aplica'] and plan['modo'] == 'directo' and plan['eps_deg'] == 0.0
    assert np.array_equal(plan['az_cmd'], p['az_continuo'])
    assert np.array_equal(plan['el_cmd'], p['el'])


def test_el_perfil_sintetico_por_defecto_no_cambio():
    """`pass_n` (nuevo parametro del generador) vale 1200, el defecto de siempre."""
    a = model.perfil_pase(550.0, 85.0, 70.0, 140.0)
    b = model.perfil_pase(550.0, 85.0, 70.0, 140.0, n=1200)
    for k in ('t', 'az_continuo', 'el', 'd', 'vr'):
        assert np.array_equal(a[k], b[k])


def test_generador_por_defecto_publica_la_direccion_del_satelite():
    rclpy = pytest.importorskip('rclpy')
    import rclpy as r
    from atp_ros.reference_generator import ReferenceGenerator
    r.init()
    try:
        n = ReferenceGenerator()
        assert n.estrategia == 'ingenua' and n.cmd_az is None and n.truth_pub is None
        assert float(n.get_parameter('tle_dt_s').value) == 0.5
        assert int(n.get_parameter('pass_n').value) == 1200
        assert n.get_parameter('el_montura_max_deg').value == 90.0
        n.destroy_node()
    finally:
        r.shutdown()
    del rclpy


def test_estrategia_desconocida_y_montura_limitada():
    p = _perfil(88.0)
    with pytest.raises(ValueError):
        K.planificar(p['t'], p['az_continuo'], p['el'], 'inventada')
    # sobre_cenit con una montura de 90 grados no aplica (y lo dice)
    plan = K.planificar(p['t'], p['az_continuo'], p['el'], 'sobre_cenit', el_montura_max_deg=90.0)
    assert not plan['aplica'] and 'no aplica' in plan['motivo']


# ---------------------------------------------------------------- el programa lineal

def test_plan_respeta_topes_y_es_determinista():
    p = _perfil(88.0)
    kw = dict(vel_max_deg_s=6.0, acel_max_deg_s2=20.0, dt_plan_s=0.25, frac_vel=1.0)
    a = K.planificar(p['t'], p['az_continuo'], p['el'], 'anticipada', **kw)
    b = K.planificar(p['t'], p['az_continuo'], p['el'], 'anticipada', **kw)
    assert np.array_equal(a['az_cmd'], b['az_cmd']) and a['eps_deg'] == b['eps_deg']
    tp = np.arange(p['t'][0], p['t'][-1] + 1e-9, 0.25)
    x = np.interp(tp, p['t'], a['az_cmd'])
    v = np.diff(x) / 0.25
    ac = np.diff(x, 2) / 0.25 ** 2
    assert np.max(np.abs(v)) <= 6.0 + 1e-6
    assert np.max(np.abs(ac)) <= 20.0 + 1e-5
    # arranca apuntando al AOS
    assert a['az_cmd'][0] == pytest.approx(p['az_continuo'][0], abs=1e-6)


def test_lp_contra_el_algoritmo_de_alcanzabilidad_independiente():
    """El error minimo del LP (sin tope de aceleracion, velocidad completa) debe
    coincidir con la biseccion + alcanzabilidad por intervalos sobre el pase
    PLANAR: dos metodos distintos para el mismo optimo (cada uno discretiza a su manera)."""
    for el_max in (86.0, 88.0, 89.5):
        pl = K.pase_planar(el_max, 550.0, dt=0.25)
        plan = K.planificar(pl['t'], pl['az'], pl['el'], 'anticipada', 6.0, 1e5, dt_plan_s=0.25,
                            frac_vel=1.0)
        eps_tubo = K.eps_tubo_planar(pl['t'], pl['az'], pl['el'], 6.0)
        assert plan['eps_deg'] == pytest.approx(eps_tubo, rel=0.06, abs=0.02), el_max


# ---------------------------------------------------------------- contraste analitico

def test_contraste_analitico_del_cenit_y_del_keyhole():
    c = K.contraste_analitico(89.9)
    assert c['el_keyhole_deg'] == pytest.approx(model.elevacion_keyhole(550.0, 6.0))
    assert c['el_keyhole_deg'] == pytest.approx(82.49, abs=0.01)
    assert c['satura'] and c['demanda_az_deg_s'] == pytest.approx(model.tasa_acimut(550.0, 89.9))
    assert c['eps_cenit_deg'] == pytest.approx(0.1)
    assert not K.contraste_analitico(80.0)['satura']
    # ordenes: cenit < anticipada < ingenua
    for el in (85.0, 88.0, 89.5):
        c = K.contraste_analitico(el)
        assert c['eps_anticipada_deg'] < c['eps_ingenua_deg']


def test_modelo_planar_coincide_con_el_esferico_cerca_del_cenit():
    """El pase planar (sin curvatura de la orbita) reproduce el error de la
    persecucion ideal del pase esferico a ~10 % en la zona del keyhole."""
    esf = model.perfil_pase(550.0, 88.0, 70.0, 140.0, n=6000)
    x = K.seguidor_limitado(esf['t'], esf['az_continuo'], 6.0)
    e_esf = float(np.max(K.error_apuntamiento(esf['az_continuo'], esf['el'], x, esf['el'])))
    c = K.contraste_analitico(88.0)
    assert e_esf == pytest.approx(c['eps_ingenua_deg'], rel=0.10)


# ---------------------------------------------------------------- el lazo completo

def _lazo(p, estrategia, el_montura=90.0, **kw):
    return K.evaluar_pase(p, estrategia, 6.0, 20.0, el_montura, ('UHF', 'S', 'X'), **kw)


def test_ingenua_pierde_el_satelite_por_encima_del_keyhole_y_no_por_debajo():
    bajo = _lazo(_perfil(80.0), 'ingenua')
    alto = _lazo(_perfil(88.0), 'ingenua')
    assert bajo['bandas']['S']['fuera_s'] == 0.0 and bajo['max_deg'] < 0.2       # transitorio de arranque
    assert alto['bandas']['S']['fuera_s'] > 5.0 and alto['max_deg'] > 3.0
    # y el error medido coincide con el de la persecucion ideal del contraste analitico
    assert alto['max_deg'] == pytest.approx(K.contraste_analitico(88.0)['eps_ingenua_deg'], rel=0.08)


def test_anticipada_mejora_a_la_ingenua():
    p = _perfil(88.0)
    ing, ant = _lazo(p, 'ingenua'), _lazo(p, 'anticipada')
    assert ant['max_deg'] < 0.7 * ing['max_deg']
    assert ant['rms_deg'] < ing['rms_deg']
    assert ant['bandas']['X']['fuera_s'] < ing['bandas']['X']['fuera_s']
    # el error planificado es cota inferior de lo logrado (hasta el margen de velocidad)
    assert ant['eps_plan_deg'] > 0.0


def test_sobre_cenit_logra_el_error_de_la_distancia_al_cenit():
    """Cruzar por el cenit: el satelite siempre esta a >= 90 - el_max del cenit, asi que el
    error minimo es ESA distancia (contraste analitico); el lazo real se queda cerca."""
    p = _perfil(89.9, n=6000)
    sc = _lazo(p, 'sobre_cenit', 180.0)
    ing = _lazo(p, 'ingenua')
    assert sc['modo'] == 'sobre_cenit'
    assert sc['max_deg'] < 0.35 and sc['max_deg'] >= 0.1 - 0.02    # >= m = 0.1 (hasta el planificador)
    assert sc['max_deg'] < 0.05 * ing['max_deg']
    assert sc['bandas']['S']['fuera_s'] == 0.0 and sc['datos_perdidos_pct'] == 0.0
    assert ing['bandas']['S']['fuera_s'] > 5.0


def test_sobre_cenit_no_se_usa_si_seguir_directo_es_mejor():
    """A 85 grados el cruce costaria 5 grados de error (la distancia al cenit): se sigue directo."""
    p = _perfil(85.0)
    plan = K.planificar(p['t'], p['az_continuo'], p['el'], 'sobre_cenit', el_montura_max_deg=180.0)
    assert plan['aplica'] and plan['modo'] == 'directo'


def test_metricas_cuentan_el_tiempo_fuera_del_haz():
    t = np.arange(0.0, 100.0, 0.1)
    e = np.zeros_like(t)
    e[(t >= 20) & (t < 30)] = 2.0           # 10 s a 2 grados: fuera en X (0.42) y S (1.59), no en UHF
    m = K.metricas(t, e, ('UHF', 'S', 'X'))
    assert m['bandas']['X']['fuera_s'] == pytest.approx(10.0, abs=0.2)
    assert m['bandas']['S']['fuera_s'] == pytest.approx(10.0, abs=0.2)
    assert m['bandas']['UHF']['fuera_s'] == 0.0
    assert m['max_deg'] == 2.0
    assert m['bandas']['S']['fuera_pct'] == pytest.approx(10.0, abs=0.3)


def test_montura_xy_mueve_el_keyhole_al_horizonte():
    """Referencia cinematica: en un pase casi cenital (89.9) el acimut pide cientos de grados por
    segundo y la X-Y (eje primario al norte) menos de 1; en un pase bajo la X-Y es la que pide mas
    (singularidad en el horizonte norte), pero nunca pasa de unos 2 deg/s con mascara de 5 grados."""
    alto = model.perfil_pase(550.0, 89.9, 5.0, 40.0, n=6000)
    azel = K.tasa_max_articulaciones(alto['t'], alto['az_continuo'], alto['el'], 'azel')
    xy = K.tasa_max_articulaciones(alto['t'], alto['az_continuo'], alto['el'], 'xy')
    assert azel[0] > 300.0 and max(xy) < 1.0
    peor = 0.0
    for a0 in range(0, 360, 20):
        b = model.perfil_pase(550.0, 8.0, 5.0, float(a0), n=3000)
        peor = max(peor, max(K.tasa_max_articulaciones(b['t'], b['az_continuo'], b['el'], 'xy')))
    assert 1.0 < peor < 3.0
    # la proyeccion es una rotacion: conserva la direccion (y = asin(norte))
    x, y = K.a_montura_xy(np.array([0.0, 90.0]), np.array([30.0, 30.0]))
    assert y[0] == pytest.approx(math.degrees(math.asin(math.cos(math.radians(30.0)))))
    assert y[1] == pytest.approx(0.0, abs=1e-9)
    with pytest.raises(ValueError):
        K.tasa_max_articulaciones(alto['t'], alto['az_continuo'], alto['el'], 'z')
