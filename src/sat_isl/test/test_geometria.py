"""Validacion de sat_isl.geometria: Walker, adelanto (point-ahead), visibilidad y
refraccion, contra formulas cerradas y contra calculos independientes.

Referencias: Walker (1984) para el patron; curso 18 (isl.py): 7.6 km/s -> 50.7 urad;
visibilidad de dos satelites coplanares a la misma altura = geometria de la
cuerda (Pitagoras); refraccion contra la integral de Abel numerica del rayo en
una atmosfera exponencial; point-ahead contra tiempos de luz retardados.
"""
import math

import numpy as np
import pytest
from scipy.integrate import quad

from sat_isl import geometria as G
from atp_ros import propagador as P

MU = P.WGS84.mu
C = G.C_LUZ_KM_S


@pytest.fixture(scope='module')
def cons():
    return G.Constelacion(T=24, Pl=3, Fase=1, h_km=800.0, inc_deg=55.0, t_fin_s=3600.0)


def par_coplanar(h_km, phi_rad):
    """Dos satelites circulares a `h` en el mismo plano ecuatorial, separados phi:
    (r1, v1, r2, v2) en +-phi/2 (Kepler exacto)."""
    a = G.R_TIERRA_KM + h_km
    v = math.sqrt(MU / a)
    out = []
    for s in (-0.5, 0.5):
        u = s * phi_rad
        out += [a * np.array([math.cos(u), math.sin(u), 0.0]),
                v * np.array([-math.sin(u), math.cos(u), 0.0])]
    return out


# ------------------------------------------------------------------ Walker

def test_walker_patron(cons):
    el = cons.elementos
    assert len(el) == 24
    # P planos igualmente espaciados en RAAN, S por plano igualmente espaciados en u
    raans = sorted({round(e.raan_rad, 9) for e in el})
    assert np.allclose(np.diff(raans), 2 * math.pi / 3)
    p0 = sorted(e.u0_rad for e in el if e.plano == 0)
    assert np.allclose(np.diff(p0), 2 * math.pi / 8)
    # desfase entre planos contiguos: 360 F / T = 15 deg
    u00 = [e.u0_rad for e in el if e.plano == 0 and e.indice == 0][0]
    u10 = [e.u0_rad for e in el if e.plano == 1 and e.indice == 0][0]
    assert math.isclose(math.degrees(u10 - u00), 360.0 * 1 / 24, abs_tol=1e-9)
    with pytest.raises(ValueError):
        G.walker_delta(25, 3, 1, 800, 55)


def test_estado_circular_kepler():
    el = G.walker_delta(24, 3, 1, 800.0, 55.0)[5]
    r, v = G.estado_circular(el)
    a = G.R_TIERRA_KM + 800.0
    assert math.isclose(np.linalg.norm(r), a, rel_tol=1e-12)
    assert math.isclose(np.linalg.norm(v), math.sqrt(MU / a), rel_tol=1e-12)
    assert abs(np.dot(r, v)) < 1e-9
    h = np.cross(r, v)
    assert math.isclose(math.degrees(math.acos(h[2] / np.linalg.norm(h))), 55.0, abs_tol=1e-9)


def test_propagacion_con_j2_conserva_la_geometria(cons):
    """Con J2 la orbita circular osculante oscila unos km y el patron se conserva:
    la separacion dentro del plano cambia < 1 % en una orbita; el radio < 15 km."""
    i, j = 0, 1
    rr = [np.linalg.norm(cons.rv(i, t)[0]) for t in np.arange(0, 3600, 60.0)]
    assert max(rr) - min(rr) < 15.0
    d = [np.linalg.norm(cons.rv(i, t)[0] - cons.rv(j, t)[0]) for t in np.arange(0, 3600, 60.0)]
    assert (max(d) - min(d)) / np.mean(d) < 0.01
    # cuerda esperada: 2 a sin(22.5 deg) = 5494 km
    a = G.R_TIERRA_KM + 800.0
    assert math.isclose(np.mean(d), 2 * a * math.sin(math.pi / 8), rel_tol=5e-3)


def test_vecinos_son_dos_del_plano_y_uno_por_plano_contiguo(cons):
    v = cons.vecinos(0)
    assert len(v) == 4
    planos = sorted(cons.elementos[k].plano for k in v)
    assert planos == [0, 0, 1, 2]
    assert {1, 7} <= set(v)


# ------------------------------------------------------------------ adelanto (point-ahead)

def test_adelanto_curso18_7p6_km_s():
    """2 v / c con v = 7.6 km/s: 50.7 urad (las cifras de isl.py del curso 18)."""
    assert math.isclose(2 * 7.6 / C * 1e6, 50.7, abs_tol=0.05)


@pytest.mark.parametrize('phi_deg', [20.0, 45.0, 60.0, 90.0])
def test_adelanto_coplanar_forma_cerrada(phi_deg):
    """Dos satelites circulares coplanares: v_rel = 2 v sin(phi/2), perpendicular
    a la linea de vista, asi que adelanto = 4 v sin(phi/2) / c."""
    phi = math.radians(phi_deg)
    r1, v1, r2, v2 = par_coplanar(800.0, phi)
    g = G.geometria_enlace(r1, v1, r2, v2)
    v = math.sqrt(MU / (G.R_TIERRA_KM + 800.0))
    assert math.isclose(g['adelanto_rad'], 4 * v * math.sin(phi / 2) / C, rel_tol=1e-12)
    assert abs(g['v_rad_km_s']) < 1e-9 * v                    # radial nula
    assert math.isclose(g['tasa_los_rad_s'], v / (G.R_TIERRA_KM + 800.0), rel_tol=1e-9)  # la LOS gira a n
    assert math.isclose(g['t_luz_s'], g['rango_km'] / C, rel_tol=1e-15)


def test_doppler_optico_curso18():
    """v / lam: 7.6 km/s a 1550 nm -> 4.9 GHz."""
    r1, v1, r2, v2 = par_coplanar(800.0, 0.5)
    r2 = r1 + np.array([1000.0, 0, 0.0])
    v2 = v1 + np.array([7.6, 0, 0.0])
    g = G.geometria_enlace(r1, v1, r2, v2, 1550e-9)
    assert math.isclose(g['doppler_hz'], 7.6e3 / 1550e-9, rel_tol=1e-12)
    assert math.isclose(g['doppler_hz'] / 1e9, 4.9, abs_tol=0.01)


@pytest.mark.parametrize('j,t', [(1, 100.0), (7, 2000.0), (13, 2000.0), (18, 500.0)])
def test_adelanto_contra_tiempos_de_luz_retardados(cons, j, t):
    """La formula 2 v_perp/c coincide con el point-ahead de la fuerza bruta (luz
    retardada + aberracion del terminal) a mejor de 1e-3 relativo, en pares del
    mismo plano y de planos distintos propagados con J2."""
    g = cons.enlace(0, j, t)
    ex = G.adelanto_exacto_rad(lambda s: cons.rv(0, s), lambda s: cons.rv(j, s), t)
    assert ex > 5e-6
    assert math.isclose(g['adelanto_rad'], ex, rel_tol=1e-3)


# ------------------------------------------------------------------ visibilidad

@pytest.mark.parametrize('phi_deg', [10.0, 30.0, 50.0, 51.0, 52.0, 90.0, 150.0])
def test_altura_minima_rayo_coplanar(phi_deg):
    """Cuerda de dos satelites coplanares: la altura del punto medio es
    (R+h) cos(phi/2) - R."""
    h = 800.0
    phi = math.radians(phi_deg)
    r1, v1, r2, v2 = par_coplanar(h, phi)
    esperado = (G.R_TIERRA_KM + h) * math.cos(phi / 2) - G.R_TIERRA_KM
    assert math.isclose(G.altura_minima_rayo_km(r1, r2), esperado, abs_tol=1e-6)


def test_visibilidad_cambia_justo_en_la_separacion_maxima():
    h = 800.0
    phi_max = G.separacion_angular_max_deg(h)
    assert math.isclose(phi_max, 51.0275, abs_tol=1e-3)         # 2 acos(6478.137/7178.137)
    for dphi, vis in ((-0.05, True), (+0.05, False)):
        r1, v1, r2, v2 = par_coplanar(h, math.radians(phi_max + dphi))
        assert G.geometria_enlace(r1, v1, r2, v2)['visible'] is vis
    # y el rango maximo es la cuerda tangente a R + 100 km
    r1, v1, r2, v2 = par_coplanar(h, math.radians(phi_max))
    g = G.geometria_enlace(r1, v1, r2, v2)
    assert math.isclose(g['rango_km'], G.rango_maximo_visible_km(h), rel_tol=1e-9)
    assert math.isclose(g['h_rayo_km'], G.H_RAYO_MIN_KM, abs_tol=1e-6)


def test_visibilidad_casos_extremos():
    a = G.R_TIERRA_KM + 800.0
    r1, r2 = np.array([a, 0, 0.0]), np.array([-a, 0, 0.0])
    assert math.isclose(G.altura_minima_rayo_km(r1, r2), -G.R_TIERRA_KM, abs_tol=1e-9)   # pasa por el centro
    # el punto mas cercano al centro cae FUERA del segmento: manda el extremo mas cercano
    r1, r2 = np.array([a, 0, 0.0]), np.array([a + 3000.0, 0, 0.0])
    assert math.isclose(G.altura_minima_rayo_km(r1, r2), 800.0, abs_tol=1e-9)
    # contra el muestreo bruto del segmento (pares aleatorios a alturas distintas)
    rng = np.random.default_rng(3)
    for _ in range(20):
        d1, d2 = rng.normal(size=3), rng.normal(size=3)
        p1 = d1 / np.linalg.norm(d1) * (G.R_TIERRA_KM + rng.uniform(500, 1500))
        p2 = d2 / np.linalg.norm(d2) * (G.R_TIERRA_KM + rng.uniform(500, 1500))
        s_ = np.linspace(0.0, 1.0, 200001)[:, None]
        bruto = float(np.min(np.linalg.norm(p1 + s_ * (p2 - p1), axis=1))) - G.R_TIERRA_KM
        assert math.isclose(G.altura_minima_rayo_km(p1, p2), bruto, abs_tol=2e-3)


def test_planos_distintos_son_visibles_solo_parte_de_la_orbita(cons):
    ts = np.arange(0, 3600.0, 30.0)
    vis = np.array([cons.enlace(0, 18, t)['visible'] for t in ts])
    assert 0.2 < vis.mean() < 0.7
    rr = [cons.enlace(0, 18, t)['rango_km'] for t in ts]
    # donde es visible, el rango no pasa de la cuerda maxima (dentro de la oscilacion de J2)
    assert max(r for r, v in zip(rr, vis) if v) < G.rango_maximo_visible_km(800.0) * 1.02


def test_base_perpendicular_es_ortonormal():
    for u in ([1.0, 0, 0], [0, 0, 1.0], [0.3, -0.5, 0.8]):
        u = np.array(u) / np.linalg.norm(u)
        e1, e2 = G.base_perpendicular(u)
        assert abs(np.dot(e1, u)) < 1e-12 and abs(np.dot(e2, u)) < 1e-12
        assert math.isclose(np.dot(e1, e2), 0.0, abs_tol=1e-12)
        assert math.isclose(np.linalg.norm(e1), 1.0) and math.isclose(np.linalg.norm(e2), 1.0)
        assert np.allclose(np.cross(e1, e2), u)


# ------------------------------------------------------------------ refraccion (justifica 100 km)

def _bending_numerico_urad(h_t_km, lam_n0=G.N_MENOS_1_SUELO):
    """Desviacion TOTAL del rayo por la integral de Abel con el indice n(r) = 1 +
    N0 rho(r)/rho0 de la misma atmosfera (independiente de la formula cerrada):
    alfa = -2 n_t r_t int_{r_t}^inf (dn/dr) / (n sqrt(n^2 r^2 - n_t^2 r_t^2)) dr."""
    atm = P.ATMOSFERA
    R = G.R_TIERRA_KM
    rt = R + h_t_km

    def n(r):
        return 1.0 + lam_n0 * float(atm.densidad(r - R)) / float(atm.rho0[0])

    def dn(r, e=1e-3):
        return (n(r + e) - n(r - e)) / (2 * e)
    nt = n(rt)
    # variable s = r - rt; el integrando tiene singularidad integrable 1/sqrt(s) en 0
    f = lambda s: -2 * nt * rt * dn(rt + s) / (n(rt + s) * math.sqrt(max((n(rt + s) * (rt + s)) ** 2 - (nt * rt) ** 2, 1e-300)))  # noqa: E731
    val = 0.0
    for a, b in ((0, 5), (5, 50), (50, 300), (300, 1500)):
        val += quad(f, a, b, limit=400)[0]
    return val * 1e6


@pytest.mark.parametrize('h', [40.0, 60.0, 80.0, 100.0])
def test_refraccion_formula_cerrada_contra_la_integral_de_abel(h):
    cerrada = G.refraccion_urad(h)
    numerica = _bending_numerico_urad(h)
    assert math.isclose(cerrada, numerica, rel_tol=0.03), (cerrada, numerica)


def test_refraccion_justifica_los_100_km():
    r = {h: G.refraccion_urad(h) for h in (30, 50, 70, 100, 150)}
    assert all(r[a] > r[b] for a, b in ((30, 50), (50, 70), (70, 100), (100, 150)))
    assert r[100] < 0.05                  # urad: despreciable frente a un haz de ~30 urad
    assert 5.0 < r[50] < 50.0             # a 50 km se desvia ~ un haz entero
    assert r[30] > 100.0
    h1 = G.altura_para_refraccion_km(1.0)
    assert 60.0 < h1 < 80.0               # 1 urad ~ 70 km: 100 km deja margen de ~30 km
    assert G.H_RAYO_MIN_KM > h1
