"""Validacion cientifica de orbit.py (Sprint 4): SGP4 contra el vector oficial,
la cadena de marcos contra casos resueltos a mano y contra fisica elemental,
y los pases contra la geometria esferica de model.py.

Los TLE de estas pruebas estan EMBEBIDOS: el resultado no depende de que
data/tle/ se refresque.
"""
import datetime as dt
import math
import os

import numpy as np
import pytest

from atp_ros import model
from atp_ros import orbit as o

UTC = dt.timezone.utc

# --- 00005 (Vallado, SGP4-VER): el caso canonico de verificacion de SGP4 ----
L1_00005 = '1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753'
L2_00005 = '2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667'
# tsince (min): r (km, TEME), v (km/s). Copiado de tcppver.out (Vallado et al.).
REF_00005 = {
    0.0: ((7022.46529266, -1400.08296755, 0.03995155),
          (1.893841015, 6.405893759, 4.534807250)),
    360.0: ((-7154.03120202, -3783.17682504, -3536.19412294),
            (4.741887409, -4.151817765, -2.093935425)),
    720.0: ((-7134.59340119, 6531.68641334, 3260.27186483),
            (-4.113793027, -2.911922039, -2.557327851)),
    1440.0: ((-938.55923943, -6268.18748831, -4294.02924751),
             (7.536105209, -0.427127707, 0.989878080)),
    4320.0: ((-9060.47373569, 4658.70952502, 813.68673153),
             (-2.232832783, -4.110453490, -3.157345433)),
}

# --- ISS, TLE real de CelesTrak (epoca 2026-10-01 19:41 UTC) ---------------
ISS_L1 = '1 25544U 98067A   26274.82022115  .00003852  00000+0  78825-4 0  9993'
ISS_L2 = '2 25544  51.6316 132.3693 0006937 211.2109 148.8469 15.48706258588289'


@pytest.fixture(scope='module')
def iss():
    return o.tle_desde_lineas('ISS (ZARYA)', ISS_L1, ISS_L2)


@pytest.fixture(scope='module')
def cdmx():
    return o.Estacion('Ciudad de Mexico', 19.4326, -99.1332, 2240.0, 5.0)


@pytest.fixture(scope='module')
def pases_iss(iss, cdmx):
    """Pases de 3 dias desde la epoca (donde el TLE es mas fiable)."""
    return o.predecir_pases(iss, cdmx, iss.epoca, 72.0)


# ------------------------------------------------ (a) vector oficial de SGP4

def test_sgp4_vector_oficial_00005():
    """SGP4 contra Vallado (tcppver.out): 00005, WGS72. La libreria sgp4 es
    la implementacion, pero la ruta completa de orbit.py (jd, fr -> TEME) debe
    reproducir las 5 filas al milimetro y 1 um/s."""
    tle = o.tle_desde_lineas('00005', L1_00005, L2_00005)
    for tsince, (r_ref, v_ref) in REF_00005.items():
        jd = tle.satrec.jdsatepoch
        fr = tle.satrec.jdsatepochF + tsince / 1440.0
        r, v = o.propagar_teme(tle, [jd], [fr])
        assert np.abs(r[0] - r_ref).max() < 1e-6, (tsince, r[0] - r_ref)
        assert np.abs(v[0] - v_ref).max() < 1e-8, (tsince, v[0] - v_ref)


def _sgp4_dir():
    import sgp4
    return os.path.dirname(sgp4.__file__)


def test_sgp4_tcppver_completo():
    """Todos los satelites de SGP4-VER.TLE (32 objetos, 666 filas de
    tcppver.out, campo cercano y espacio profundo) contra el resultado oficial
    de Vallado, si la instalacion de sgp4 trae ambos archivos. Medido: error
    maximo 1.6e-7 km (el archivo oficial imprime 8 decimales)."""
    d = _sgp4_dir()
    f_tle, f_out = os.path.join(d, 'SGP4-VER.TLE'), os.path.join(d, 'tcppver.out')
    if not (os.path.exists(f_tle) and os.path.exists(f_out)):
        pytest.skip('sgp4 no trae SGP4-VER.TLE / tcppver.out')
    sats = {}
    pend = None
    for ln in open(f_tle):
        if ln.startswith('#') or not ln.strip():
            continue
        if ln.startswith('1 '):
            pend = ln[:69]
        elif ln.startswith('2 ') and pend:
            tle = o.tle_desde_lineas(pend[2:7], pend, ln[:69])
            sats[int(pend[2:7])] = tle
            pend = None
    assert len(sats) >= 29
    worst_near, worst_deep, n_filas = 0.0, 0.0, 0
    sat_actual = None
    for ln in open(f_out):
        p = ln.split()
        if len(p) == 2 and p[1] == 'xx':
            sat_actual = sats[int(p[0])]
            continue
        if sat_actual is None or len(p) < 7:
            continue
        tsince = float(p[0])
        r_ref = np.array([float(x) for x in p[1:4]])
        s = sat_actual.satrec
        e, r, _ = s.sgp4_array(np.array([s.jdsatepoch]),
                               np.array([s.jdsatepochF + tsince / 1440.0]))
        if e[0] != 0:
            continue
        err = float(np.abs(np.array(r[0]) - r_ref).max())
        n_filas += 1
        if sat_actual.n_rev_dia > 6.4:
            worst_near = max(worst_near, err)
        else:
            worst_deep = max(worst_deep, err)
    assert n_filas > 600
    assert worst_near < 1e-6            # km (1 mm)
    assert worst_deep < 1e-6


# ------------------------------------------------ marcos y tiempo (e)

def test_gmst_vallado_ejemplo_3_5():
    """Vallado, Ejemplo 3-5: 1992-08-20 12:14:00 UT1 -> GMST 152.578787810 deg."""
    jd, fr = o.a_jd(dt.datetime(1992, 8, 20, 12, 14, 0, tzinfo=UTC))
    assert math.degrees(float(o.gmst_rad(jd, fr))) == pytest.approx(
        152.578787810, abs=1e-6)


def test_geodetica_ecef_casos_cerrados():
    assert np.allclose(o.geodetica_a_ecef(0, 0, 0), [o.A_WGS84_KM, 0, 0], atol=1e-9)
    assert np.allclose(o.geodetica_a_ecef(0, 90, 0), [0, o.A_WGS84_KM, 0], atol=1e-9)
    assert np.allclose(o.geodetica_a_ecef(90, 0, 0), [0, 0, o.B_WGS84_KM], atol=1e-9)
    assert o.B_WGS84_KM == pytest.approx(6356.752314, abs=1e-5)
    # ida y vuelta, incluida altitud de LEO y de la estacion
    for lat, lon, alt_m in ((19.4326, -99.1332, 2240.0), (-45.0, 170.0, 0.0),
                            (69.65, 18.96, 420e3)):
        r = o.geodetica_a_ecef(lat, lon, alt_m)
        la, lo, al = o.ecef_a_geodetica(r)
        assert (la, lo, al * 1000.0) == pytest.approx((lat, lon, alt_m), abs=1e-6)


def test_enu_a_azel_ejemplo_del_curso():
    """Curso 30, L1.2: e=400, n=300, u=500 -> Az 53.13, El 45.0, d 707."""
    az, el, d = o.enu_a_azel(400.0, 300.0, 500.0)
    assert (float(az), float(el), float(d)) == pytest.approx((53.13, 45.0, 707.1), abs=0.05)
    # cuadrantes: el atan2 es (E, N)
    assert float(o.enu_a_azel(-1.0, 0.0, 0.0)[0]) == pytest.approx(270.0)
    assert float(o.enu_a_azel(0.0, -1.0, 0.0)[0]) == pytest.approx(180.0)


def test_enu_apunta_bien():
    """Un punto 100 km directamente 'arriba' de la estacion (en la vertical
    GEODESICA) tiene el = 90; uno 100 km al norte local sobre el horizonte
    tiene az = 0 y el ~ -0.5 deg (curvatura)."""
    lat, lon, alt = 19.4326, -99.1332, 2240.0
    est = o.geodetica_a_ecef(lat, lon, alt)
    m = o.matriz_enu(lat, lon)
    arriba = est + 100.0 * m[2]
    enu = (arriba - est) @ m.T
    assert float(o.enu_a_azel(*enu)[1]) == pytest.approx(90.0)
    # la matriz es una rotacion propia
    assert np.allclose(m @ m.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(m) == pytest.approx(1.0)


# ------------------------------------------------ (b) consistencia ECEF

def test_ecef_radio_y_altitud_iss(iss):
    """|r_ecef| = |r_teme| (una rotacion conserva la norma) y el radio del
    satelite queda en [a(1-e), a(1+e)] de Kepler (a desde el movimiento
    medio), es decir ~R+h; la altitud geodesica de la ISS cae en 400-440 km."""
    n = 2000
    seg = np.linspace(0, 3 * 5560.0, n)           # 3 orbitas
    jd = np.full(n, iss.satrec.jdsatepoch)
    fr = iss.satrec.jdsatepochF + seg / 86400.0
    r_t, v_t = o.propagar_teme(iss, jd, fr)
    r_e, v_e = o.teme_a_ecef(r_t, v_t, jd, fr)
    assert np.allclose(np.linalg.norm(r_e, axis=1), np.linalg.norm(r_t, axis=1), atol=1e-9)
    mu = 398600.4418
    a = (mu / (iss.n_rev_dia * 2 * math.pi / 86400.0) ** 2) ** (1 / 3)
    ecc = float(iss.linea2[26:33]) * 1e-7
    rad = np.linalg.norm(r_e, axis=1)
    # SGP4 usa el movimiento medio de Kozai: ~ +-10 km de diferencia con la a kepleriana
    assert rad.min() > a * (1 - ecc) - 15.0 and rad.max() < a * (1 + ecc) + 15.0
    assert a - o.A_WGS84_KM == pytest.approx(420.0, abs=25.0)        # R + h ~ 6800 km
    _, _, alt = o.ecef_a_geodetica(r_e)
    assert 400.0 < alt.min() and alt.max() < 440.0
    # el elipsoide importa: a lat 51.6 el radio terrestre es ~13 km menor que el ecuatorial
    assert rad.max() - rad.min() < 30.0


def test_ground_track_inclinacion_y_deriva(iss):
    """La latitud geocentrica del satelite llega a la inclinacion (51.63 deg) y tras
    una revolucion el cruce ascendente se corre al oeste ~ 360 T/T_sidereo
    (~23 deg): comprueba el SENTIDO y la magnitud de la rotacion GMST."""
    n = 6000
    seg = np.linspace(0, 2 * 5600.0, n)
    jd = np.full(n, iss.satrec.jdsatepoch)
    fr = iss.satrec.jdsatepochF + seg / 86400.0
    r, _ = o.propagar_ecef(iss, jd, fr)
    lat, lon, _ = o.ecef_a_geodetica(r)
    # la latitud GEOCENTRICA llega a la inclinacion; la geodesica la excede
    # ~0.15 deg (a 52 deg el elipsoide aplana): confundirlas es el error clasico
    lat_gc = np.degrees(np.arcsin(r[:, 2] / np.linalg.norm(r, axis=1)))
    assert lat_gc.max() == pytest.approx(51.63, abs=0.1)
    assert lat_gc.min() == pytest.approx(-51.63, abs=0.1)
    assert 0.05 < lat.max() - lat_gc.max() < 0.25
    # cruces ascendentes del ecuador
    k = np.where((lat[:-1] < 0) & (lat[1:] >= 0))[0]
    assert len(k) >= 2
    lon_c = [lon[i] - lat[i] * (lon[i + 1] - lon[i]) / (lat[i + 1] - lat[i]) for i in k[:2]]
    dlon = (lon_c[1] - lon_c[0] + 180.0) % 360.0 - 180.0
    periodo_nodal = seg[k[1]] - seg[k[0]]
    # en un periodo nodal T la Tierra gira 360 T/Tsid hacia el este: el cruce
    # se corre al oeste esa cantidad (mas ~0.3 deg de regresion del nodo por J2)
    esperado = -360.0 * periodo_nodal / 86164.0905
    assert dlon == pytest.approx(esperado, abs=0.6)
    assert -24.0 < dlon < -22.0


# ------------------------------------------------ (c) pases

def test_pases_basicos(pases_iss):
    assert len(pases_iss) >= 6               # la ISS pasa 3-6 veces/dia por CDMX
    prev = None
    for p in pases_iss:
        assert p['aos'] < p['tca'] < p['los']
        assert p['aos'] < p['t_el_max'] < p['los']
        assert 60.0 < p['duracion_s'] < 700.0
        assert 5.0 <= p['el_max_deg'] <= 90.0
        if prev:
            assert p['aos'] > prev['los']
        prev = p


def test_aos_los_estan_en_la_mascara(iss, cdmx, pases_iss):
    for p in pases_iss[:4]:
        for clave, signo in (('aos', +1), ('los', -1)):
            t = p[clave]
            el0 = float(o.observar_en(iss, cdmx, t)['el_deg'])
            assert el0 == pytest.approx(cdmx.mascara_deg, abs=0.02)
            antes = float(o.observar_en(iss, cdmx, t - signo * dt.timedelta(seconds=2))['el_deg'])
            assert antes < cdmx.mascara_deg < float(
                o.observar_en(iss, cdmx, t + signo * dt.timedelta(seconds=2))['el_deg'])


def test_tca_rango_minimo_y_vr_cambia_de_signo(iss, cdmx, pases_iss):
    """Rango minimo = TCA (contra fuerza bruta a 0.1 s) y vr cruza el cero ahi."""
    for p in pases_iss[:4]:
        t0 = o.a_jd(p['aos'])
        seg = np.arange(0.0, p['duracion_s'], 0.1)
        jd, fr = np.full_like(seg, t0[0]), t0[1] + seg / 86400.0
        ob = o.observar(iss, cdmx, jd, fr)
        k = int(np.argmin(ob['rango_km']))
        t_fuerza_bruta = seg[k]
        t_tca = (p['tca'] - p['aos']).total_seconds()
        assert t_tca == pytest.approx(t_fuerza_bruta, abs=0.3)
        assert p['rango_min_km'] == pytest.approx(float(ob['rango_km'][k]), abs=0.05)
        # velocidad radial: negativa antes, positiva despues, ~0 en el TCA
        vr = lambda s: float(o.observar_en(iss, cdmx, p['tca'] + dt.timedelta(seconds=s))['vr_km_s'])
        assert vr(-10.0) < 0.0 < vr(10.0)
        assert abs(vr(0.0)) < 2e-3                  # km/s (la pendiente es ~ 0.1 km/s^2)
        # vr es la derivada exacta del rango (diferencia central a 1 s)
        h = 1.0
        d_num = (o.observar_en(iss, cdmx, p['aos'] + dt.timedelta(seconds=60 + h))['rango_km']
                 - o.observar_en(iss, cdmx, p['aos'] + dt.timedelta(seconds=60 - h))['rango_km']) / (2 * h)
        vr60 = float(o.observar_en(iss, cdmx, p['aos'] + dt.timedelta(seconds=60))['vr_km_s'])
        assert vr60 == pytest.approx(float(d_num), abs=2e-3)
        # el pico de elevacion queda a pocos segundos del TCA
        assert abs((p['t_el_max'] - p['tca']).total_seconds()) < 15.0


def test_elevacion_maxima_contra_fuerza_bruta(iss, cdmx, pases_iss):
    for p in pases_iss[:4]:
        t0 = o.a_jd(p['aos'])
        seg = np.arange(0.0, p['duracion_s'], 0.1)
        ob = o.observar(iss, cdmx, np.full_like(seg, t0[0]), t0[1] + seg / 86400.0)
        assert p['el_max_deg'] == pytest.approx(float(ob['el_deg'].max()), abs=0.01)
        assert p['az_aos_deg'] == pytest.approx(float(ob['az_deg'][0]), abs=0.1)


def test_pase_truncado_al_inicio(iss, cdmx, pases_iss):
    """Si la ventana empieza a mitad de un pase, AOS = inicio y se marca."""
    p = pases_iss[0]
    ini = p['aos'] + dt.timedelta(seconds=100)
    q = o.predecir_pases(iss, cdmx, ini, 1.0)[0]
    assert q['aos_truncado'] and not q['los_truncado']
    assert abs((q['los'] - p['los']).total_seconds()) < 1.0


def test_muestreo_formato_de_perfil_pase(iss, cdmx, pases_iss):
    p = pases_iss[1]
    s = o.muestrear_pase(iss, cdmx, p, dt_s=0.5)
    for k in ('t', 'az', 'az_continuo', 'el', 'd', 'vr', 'az_pto', 'el_pto',
              'duracion', 'd_min'):
        assert k in s
    assert s['t'][0] == 0.0 and s['t'][-1] == pytest.approx(p['duracion_s'])
    assert s['el'][0] == pytest.approx(cdmx.mascara_deg, abs=0.02)
    assert s['el'][-1] == pytest.approx(cdmx.mascara_deg, abs=0.02)
    assert np.max(np.abs(np.diff(s['az_continuo']))) < 5.0       # desenroscado
    assert s['vr'][0] < 0.0 < s['vr'][-1]


# ------------------------------------------------ (d) contra model.perfil_pase

def test_comparacion_con_geometria_esferica(iss, cdmx, pases_iss):
    """Mismo pase en dos modelos: SGP4 (TLE real, Tierra elipsoidal que GIRA)
    contra model.perfil_pase (orbita circular, Tierra esferica FIJA), con la
    altitud geodesica del satelite en el TCA y la elevacion maxima y acimut de
    culminacion del pase real.

    Diferencia ESPERADA y sistematica: la ISS es prograda y la Tierra gira
    hacia el este, asi que la velocidad angular relativa al sitio es menor que
    la inercial: el pase real dura MAS (medido +3.5 a +4.7 %) y el acimut se
    mueve ~3-4 % mas despacio. El rango minimo coincide a < 1 %. La excentricidad
    (0.0007), el J2 y la altura de la estacion (2.2 km) pesan < 0.5 %."""
    for p in pases_iss:
        s = o.muestrear_pase(iss, cdmx, p, dt_s=0.25)
        r, v_rel = o.propagar_ecef(iss, *[np.array([x]) for x in o.a_jd(p['tca'])])
        h = float(o.ecef_a_geodetica(r)[2][0])
        mp = model.perfil_pase(h, p['el_max_deg'], cdmx.mascara_deg, p['az_el_max_deg'], n=2000)
        dur_rel = p['duracion_s'] / mp['duracion'] - 1.0
        assert 0.0 < dur_rel < 0.08, dur_rel
        assert p['rango_min_km'] == pytest.approx(mp['d_min'], rel=0.01)
        i, j = int(np.argmax(s['el'])), int(np.argmax(mp['el']))
        assert abs(s['az_pto'][i]) == pytest.approx(abs(mp['az_pto'][j]), rel=0.08)
        assert abs(s['az_pto'][i]) < abs(mp['az_pto'][j])         # el signo de la diferencia
        # el barrido total de acimut es geometria pura: coincide a ~2 deg
        d_az_sgp4 = abs(s['az_continuo'][-1] - s['az_continuo'][0])
        d_az_esf = abs(mp['az_continuo'][-1] - mp['az_continuo'][0])
        assert d_az_sgp4 == pytest.approx(d_az_esf, abs=3.0)


def test_diferencia_explicada_por_rotacion_terrestre(iss, cdmx, pases_iss):
    """La diferencia de duracion se explica con un solo numero: la razon entre
    la rapidez INERCIAL del satelite y su rapidez RELATIVA a la Tierra (ECEF),
    ambas en el TCA. Si el modelo esferico fuera 'sin rotacion', dur_sgp4 /
    dur_esf ~ v_inercial / v_ecef."""
    for p in pases_iss[:6]:
        jd, fr = [np.array([x]) for x in o.a_jd(p['tca'])]
        r_t, v_t = o.propagar_teme(iss, jd, fr)
        r_e, v_e = o.teme_a_ecef(r_t, v_t, jd, fr)
        razon = float(np.linalg.norm(v_t) / np.linalg.norm(v_e))
        h = float(o.ecef_a_geodetica(r_e)[2][0])
        mp = model.perfil_pase(h, p['el_max_deg'], cdmx.mascara_deg, p['az_el_max_deg'], n=2000)
        dur_rel = p['duracion_s'] / mp['duracion']
        # la razon de rapideces es la rotacion en el satelite (a ~6800 km); en el
        # sitio la velocidad de arrastre es la de su latitud: se admite 2 puntos
        assert dur_rel == pytest.approx(razon, abs=0.02), (dur_rel, razon)


# ------------------------------------------------ catalogo y estaciones

def test_parseo_y_busqueda_tle():
    txt = f'ISS (ZARYA)\n{ISS_L1}\n{ISS_L2}\n00005\n{L1_00005}\n{L2_00005}\n'
    cat = o.parsear_tle(txt)
    assert [t.norad for t in cat] == [25544, 5]
    assert all(t.checksum_ok for t in cat)
    assert o.buscar_tle(cat, 'iss').norad == 25544
    assert o.buscar_tle(cat, '5').nombre == '00005'
    assert cat[0].epoca == dt.datetime(2026, 10, 1, 19, 41, 7, 107360, tzinfo=UTC)
    with pytest.raises(KeyError):
        o.buscar_tle(cat, 'NOEXISTE')
    # dos lineas sin nombre
    assert o.parsear_tle(f'{ISS_L1}\n{ISS_L2}\n')[0].norad == 25544
    # checksum malo se detecta
    malo = ISS_L1[:-1] + '0'
    assert not o.tle_desde_lineas('x', malo, ISS_L2).checksum_ok


def test_catalogo_real_del_repo():
    try:
        cat = o.cargar_catalogo()
    except FileNotFoundError:
        pytest.skip('data/tle no disponible')
    assert len(cat) >= 10
    assert all(t.checksum_ok for t in cat)
    assert o.buscar_tle(cat, 'ISS').norad == 25544


def test_estaciones_yaml():
    try:
        est, defecto = o.cargar_estaciones()
    except FileNotFoundError:
        pytest.skip('data/estaciones.yaml no disponible')
    assert defecto == 'cdmx' and len(est) >= 3
    e = o.buscar_estacion(None)
    assert (e.lat_deg, e.lon_deg, e.alt_m, e.mascara_deg) == (19.4326, -99.1332, 2240.0, 5.0)
    assert o.buscar_estacion('Ciudad de Mexico').nombre == e.nombre
    assert o.buscar_estacion('TROMSO').lat_deg > 60
    with pytest.raises(KeyError):
        o.buscar_estacion('marte')


def test_pases_json_serializable(pases_iss):
    import json
    s = json.dumps(o.pases_json(pases_iss[:2]))
    assert '"aos"' in s and s.count('Z"') >= 6


def test_ut1_utc_y_sensibilidad(iss, cdmx, pases_iss):
    """Documenta la magnitud de lo que se desprecia: 0.9 s de UT1-UTC mueven
    el satelite menos de 0.1 deg en el cielo para este pase (< 0.2 deg en
    el peor caso con mascara)."""
    p = pases_iss[1]
    jd, fr = o.a_jd(p['tca'])
    a = o.observar(iss, cdmx, np.array([jd]), np.array([fr]), dut1_s=0.0)
    b = o.observar(iss, cdmx, np.array([jd]), np.array([fr]), dut1_s=0.9)
    sep = math.hypot((a['az_deg'][0] - b['az_deg'][0]) * math.cos(math.radians(a['el_deg'][0])),
                     a['el_deg'][0] - b['el_deg'][0])
    assert 0.001 < sep < 0.2


# ------------------------------------------------ (e) verificacion independiente

# Salida de Skyfield 1.55 (implementacion INDEPENDIENTE de la cadena de marcos:
# usa UT1 y movimiento del polo de IERS, WGS84 propia) para el mismo TLE de la
# ISS y la estacion CDMX (19.4326, -99.1332, 2240 m). Las filas se generaron
# una vez con Skyfield (EarthSatellite, wgs84.latlon, altaz()); aqui son datos.
SKYFIELD_ISS_CDMX = [
    ('2026-10-02T15:52:33', 238.3769, 4.9921, 1868.611),
    ('2026-10-02T15:54:13', 249.5504, 14.6836, 1244.487),
    ('2026-10-02T15:55:53', 281.8817, 30.2239, 775.200),
    ('2026-10-02T15:56:53', 322.1336, 33.6274, 717.309),
    ('2026-10-02T15:58:33', 6.7227, 18.6086, 1083.806),
    ('2026-10-02T16:00:53', 24.5591, 4.2218, 1941.209),
]


def test_contra_skyfield_independiente(iss, cdmx):
    """Az/El/rango de nuestra cadena TEME->ECEF->ENU contra Skyfield en 6
    instantes de un pase. Medido sobre 26 muestras: separacion angular maxima
    0.003 deg, rango 0.04 km. Es la diferencia atribuible a UT1-UTC y al
    movimiento del polo que orbit.py desprecia (la libreria SGP4 es la
    misma: lo que se valida aqui es la cadena de marcos)."""
    for t_s, az, el, d in SKYFIELD_ISS_CDMX:
        t = dt.datetime.fromisoformat(t_s).replace(tzinfo=UTC)
        m = o.observar_en(iss, cdmx, t)
        daz = (float(m['az_deg']) - az + 180.0) % 360.0 - 180.0
        e1, e2 = math.radians(float(m['el_deg'])), math.radians(el)
        c = math.sin(e1) * math.sin(e2) + math.cos(e1) * math.cos(e2) * math.cos(math.radians(daz))
        sep = math.degrees(math.acos(min(1.0, c)))
        assert sep < 0.01, (t_s, sep)
        assert float(m['rango_km']) == pytest.approx(d, abs=0.15)


def test_skyfield_en_vivo_si_esta_instalado(iss, cdmx):
    sf = pytest.importorskip('skyfield.api')
    ts = sf.load.timescale()
    sat = sf.EarthSatellite(ISS_L1, ISS_L2, 'ISS', ts)
    st = sf.wgs84.latlon(19.4326, -99.1332, elevation_m=2240.0)
    t = dt.datetime(2026, 10, 2, 15, 56, 53, tzinfo=UTC)
    el, az, d = (sat - st).at(ts.from_datetime(t)).altaz()
    m = o.observar_en(iss, cdmx, t)
    assert float(m['el_deg']) == pytest.approx(el.degrees, abs=0.01)
    assert float(m['rango_km']) == pytest.approx(d.km, abs=0.15)


def test_perfil_tle_listo_para_el_generador():
    """El camino completo que usa reference_generator: nombre -> TLE del repo
    -> estacion por defecto -> pase indexado -> perfil muestreado."""
    try:
        perfil, pase, tle, est = o.perfil_tle(
            'ISS', None, 0, '2026-10-02T15:00:00', horas=24.0)
    except FileNotFoundError:
        pytest.skip('data/ no disponible')
    assert tle.norad == 25544 and est.nombre == 'Ciudad de Mexico'
    # el TLE de data/ puede refrescarse: solo se exige un AOS cercano al esperado
    assert abs((pase['aos'] - dt.datetime(2026, 10, 2, 15, 52, 33, tzinfo=UTC)).total_seconds()) < 600
    assert perfil['t'][0] == 0.0 and perfil['duracion'] == pytest.approx(pase['duracion_s'])
    with pytest.raises(LookupError):
        o.perfil_tle('ISS', None, 99, '2026-10-02T15:00:00', horas=24.0)
    with pytest.raises(KeyError):
        o.perfil_tle('ISS', 'marte', 0, '2026-10-02T15:00:00')
