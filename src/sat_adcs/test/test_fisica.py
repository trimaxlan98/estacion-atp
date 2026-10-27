"""Validacion de `sat_adcs.fisica`: cada prueba dice contra que se contrasta
(solucion analitica, ley de conservacion, otro codigo independiente o una
cifra publicada) y bajo que condicion vale.
"""
import datetime as dt
import math

import numpy as np
import pytest

from atp_ros import orbit
from sat_adcs import fisica as F

UTC = dt.timezone.utc


# ------------------------------------------------------------- cuaterniones

def test_algebra_de_cuaterniones():
    """R(q) es ortogonal propia; R(a (x) b) = R(a) R(b) (composicion); el
    cuaternion de 90 deg sobre z manda x en y (regla de la mano derecha);
    dcm -> q -> dcm es la identidad incluso en giros de 180 deg (donde
    w = 0 y un metodo ingenuo divide por cero)."""
    rng = np.random.default_rng(1)
    for _ in range(200):
        a, b = F.q_aleatorio(rng), F.q_aleatorio(rng)
        R = F.q_a_dcm(a)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)
        assert np.allclose(F.q_a_dcm(F.q_mult(a, b)), R @ F.q_a_dcm(b), atol=1e-12)
        assert np.allclose(F.q_a_dcm(F.dcm_a_q(R)), R, atol=1e-12)
        th = rng.normal(size=3)
        assert np.allclose(F.q_a_rotvec(F.q_de_rotvec(th * 0.4)), th * 0.4, atol=1e-12)
    q90 = F.q_de_eje_angulo([0, 0, 1], math.pi / 2)
    assert np.allclose(F.q_rotar(q90, [1, 0, 0]), [0, 1, 0], atol=1e-12)
    for eje in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]):
        q = F.q_de_eje_angulo(eje, math.pi)
        assert np.allclose(F.q_a_dcm(F.dcm_a_q(F.q_a_dcm(q))), F.q_a_dcm(q), atol=1e-12)


def test_cinematica_del_cuaternion():
    """Convencion dq/dt = 1/2 q (x) [0, w_B]: con w_B constante y la integral
    exacta q(t) = q0 (x) exp(w t), la derivada numerica coincide con la
    ecuacion diferencial (consistencia de convenciones entre q_de_rotvec,
    q_mult y f_rigido)."""
    rng = np.random.default_rng(2)
    q0 = F.q_aleatorio(rng)
    w = np.array([0.03, -0.02, 0.05])
    h = 1e-5
    q1 = F.q_mult(q0, F.q_de_rotvec(w * h))
    q_1 = F.q_mult(q0, F.q_de_rotvec(-w * h))
    dq_num = (q1 - q_1) / (2 * h)
    assert np.allclose(dq_num, 0.5 * F.q_mult(q0, [0, *w]), atol=1e-9)


# ------------------------------------------------------------- dinamica

def _tumble():
    sat = F.cubesat_3u()
    y0 = F.estado_inicial(F.q_aleatorio(np.random.default_rng(3)),
                          [0.35, -0.25, 0.4], [0.4e-3, -0.9e-3, 0.3e-3])
    return sat, y0


def _H_inercial(y, sat):
    return F.q_a_dcm(y[:4]) @ F.momento_total_cuerpo(y, sat)


def test_conservacion_momento_y_energia_sin_pares():
    """Sin pares externos y con las ruedas a momento constante (u = 0) se
    conservan: el momento angular TOTAL en el inercial H_I = R (J w + A h) y la
    energia rotacional T = 1/2 w.J w (d(T)/dt = w.(-w x H) = 0). RK4 con
    h = 0.05 s durante 600 s de un tumbling rapido (20 deg/s, 12 vueltas de
    precesion). Cota del error relativo: 1e-7 (el error global de RK4 es
    ~ C (w h)^4 por vuelta; con w h = 0.0175 rad medimos ~5e-9, y el orden
    h^4 se verifica aparte en test_rk4_orden_4). Los dos deben conservarse a
    la vez: con un signo mal en la dinamica de Euler, T derivaria."""
    sat, y0 = _tumble()
    y, _ = F.propagar(y0, sat, np.zeros(3), np.zeros(3), 600.0, h_int=0.05)
    dH = np.linalg.norm(_H_inercial(y, sat) - _H_inercial(y0, sat)) / np.linalg.norm(_H_inercial(y0, sat))
    dT = abs(F.energia_rotacional(y, sat) - F.energia_rotacional(y0, sat)) / F.energia_rotacional(y0, sat)
    assert dH < 1e-7, dH
    assert dT < 1e-7, dT
    assert np.linalg.norm(y[:4]) == pytest.approx(1.0, abs=1e-12)
    # el cuerpo realmente se movio (no es una prueba trivial)
    assert np.linalg.norm(y[4:7] - y0[4:7]) > 0.05


def test_rk4_orden_4():
    """El error global de RK4 escala como h^4: al duplicar el paso crece x16.
    Se mide contra la solucion ANALITICA de Euler (cuerpo axisimetrico,
    w_x = w_t cos(lambda t), w_y = w_t sin(lambda t)) con un tumbling a
    0.5 rad/s durante 60 s y pasos 0.5, 0.25, 0.125 y 0.0625 s, lejos del piso
    de redondeo. Pendiente aceptada: entre 3.7 y 4.4. El error de energia
    (un invariante) converge aun mas rapido (medimos ~h^5): se exige >= h^3.7."""
    base = F.cubesat_3u()
    It, Ia = 0.042, 0.0067
    sat = F.Satelite(J=np.diag([It, It, Ia]), A=np.eye(3), h_max=1.0, tau_max=1.0,
                     m_max=0.2, masa=4.0, dims=base.dims, cg=np.zeros(3), m_res=np.zeros(3))
    wt, wz, T = 0.5, 0.6, 60.0
    lam = (Ia - It) / It * wz
    y0 = F.estado_inicial(F.q_aleatorio(np.random.default_rng(5)), [wt, 0.0, wz], np.zeros(3))
    T0 = F.energia_rotacional(y0, sat)
    e_w, e_T = [], []
    for h in (0.5, 0.25, 0.125, 0.0625):
        y, _ = F.propagar(y0, sat, np.zeros(3), np.zeros(3), T, h_int=h)
        e_w.append(math.hypot(y[4] - wt * math.cos(lam * T), y[5] - wt * math.sin(lam * T)))
        e_T.append(abs(F.energia_rotacional(y, sat) - T0) / T0)
    p_w = [math.log2(e_w[i] / e_w[i + 1]) for i in range(3)]
    p_T = [math.log2(e_T[i] / e_T[i + 1]) for i in range(3)]
    assert all(3.7 < p < 4.4 for p in p_w), (e_w, p_w)
    assert all(p > 3.7 for p in p_T), (e_T, p_T)
    assert e_w[-1] > 1e-12                       # no se mide el piso de redondeo


def test_precesion_libre_axisimetrica_contra_la_solucion_analitica():
    """Cuerpo axisimetrico (Ix = Iy = It, Iz = Ia) sin pares: el eje de simetria
    gira en el cuerpo con lambda = (Ia - It)/It * w_z (w_z constante) y
        w_x = w_t cos(lambda t),  w_y = w_t sin(lambda t)   (Euler, solucion exacta).
    En el INERCIAL, el eje z del cuerpo recorre un cono de semiangulo constante
    theta = atan(It w_t / (Ia w_z)) alrededor de H, con velocidad de
    precesion phi' = H/It. Se compara la integracion RK4 con las tres
    cantidades analiticas, durante 5 periodos."""
    base = F.cubesat_3u()
    It, Ia = 0.042, 0.0067
    sat = F.Satelite(J=np.diag([It, It, Ia]), A=np.eye(3), h_max=1.0, tau_max=1.0,
                     m_max=0.2, masa=4.0, dims=base.dims, cg=np.zeros(3), m_res=np.zeros(3))
    wt, wz = 0.04, 0.2
    q0 = F.q_aleatorio(np.random.default_rng(5))
    y0 = F.estado_inicial(q0, [wt, 0.0, wz], np.zeros(3))
    lam = (Ia - It) / It * wz
    T = 2 * math.pi / abs(lam)
    t_fin = 5 * T
    y, _ = F.propagar(y0, sat, np.zeros(3), np.zeros(3), t_fin, h_int=0.05)
    assert y[6] == pytest.approx(wz, abs=1e-10)                     # w_z constante
    assert y[4] == pytest.approx(wt * math.cos(lam * t_fin), abs=1e-8)
    assert y[5] == pytest.approx(wt * math.sin(lam * t_fin), abs=1e-8)
    # cono inercial
    H = _H_inercial(y0, sat)
    Hn = np.linalg.norm(H)
    theta = math.atan2(It * wt, Ia * wz)
    assert F.angulo_entre(F.q_a_dcm(y0[:4]) @ [0, 0, 1], H) == pytest.approx(theta, abs=1e-12)
    assert F.angulo_entre(F.q_a_dcm(y[:4]) @ [0, 0, 1], H) == pytest.approx(theta, abs=1e-8)
    # velocidad de precesion: azimut del eje z alrededor de H
    e1 = F.unitario(np.cross(H, [0.3, 0.2, 0.9]))
    e2 = np.cross(H / Hn, e1)

    def azimut(yy):
        z = F.q_a_dcm(yy[:4]) @ [0, 0, 1]
        return math.atan2(z @ e2, z @ e1)
    n = 40
    ts = np.linspace(0, 40.0, n)
    az = []
    yy = y0
    for i in range(n):
        az.append(azimut(yy))
        yy, _ = F.propagar(yy, sat, np.zeros(3), np.zeros(3), ts[1] - ts[0], h_int=0.05)
    rate = np.polyfit(ts, np.unwrap(az), 1)[0]
    assert abs(rate) == pytest.approx(Hn / It, rel=1e-6)


# ------------------------------------------------------------- ruedas

def test_saturacion_de_ruedas():
    """Con un mando de par constante por encima del maximo, la rueda recibe
    solo tau_max, y al llegar a h_max el momento se detiene EXACTAMENTE ahi
    (|h| <= h_max en todo instante) y el par sobre el cuerpo se anula: los
    limites del actuador se respetan aunque el controlador los pida
    violar."""
    sat = F.cubesat_3u()
    y = F.estado_inicial([1, 0, 0, 0], np.zeros(3), np.zeros(3))
    hs = []
    for k in range(200):
        y, u = F.propagar(y, sat, np.array([5e-3, 0, 0]), np.zeros(3), 1.0, 0.25)
        assert abs(u[0]) <= sat.tau_max + 1e-15
        hs.append(y[7])
    hs = np.array(hs)
    assert hs.max() <= sat.h_max + 1e-12
    assert hs.max() == pytest.approx(sat.h_max, rel=1e-9)         # llego y se quedo
    # rampa de tau_max hasta saturar: tarda h_max/tau_max segundos
    t_sat = int(np.argmax(hs >= sat.h_max - 1e-12)) + 1
    assert t_sat == pytest.approx(sat.h_max / sat.tau_max, abs=1.0)
    # saturada, el cuerpo ya no recibe par: el momento angular se conserva (J w = -h)
    # y la velocidad angular queda constante
    assert np.allclose(sat.J @ y[4:7] + y[7:], 0.0, atol=1e-12)
    y2, _ = F.propagar(y, sat, np.array([5e-3, 0, 0]), np.zeros(3), 10.0, 0.25)
    assert np.allclose(y2[4:7], y[4:7], atol=1e-12)


# ------------------------------------------------------------- perturbaciones

def test_gradiente_de_gravedad_formula_y_orden_de_magnitud():
    """Para un cuerpo con ejes principales alineados con LVLH rotado un angulo
    theta de cabeceo, tau_pitch = (3/2) n^2 (Ix - Iz) sin 2 theta (Wie). Se
    compara con el producto vectorial 3 mu/r^3 c x J c (otra forma). Orden
    de magnitud: 3 n^2 dI ~ 1e-7 N m a 6928 km con dI = 0.035."""
    a = 6928.0
    n = math.sqrt(F.MU_M3_S2 * 1e-9 / a ** 3)
    J = np.diag([0.042, 0.050, 0.0067])
    from sat_adcs.misiones import q_ref_lvlh
    r = np.array([a, 0, 0])
    q_lvlh = q_ref_lvlh(r, np.array([0.0, 7.6, 0.0]))     # z = nadir, y = -normal
    th = math.radians(10)
    qp = F.q_mult(q_lvlh, F.q_de_eje_angulo([0, 1, 0], th))
    tau = F.par_gradiente_gravedad(J, qp, r)
    esperado = 1.5 * n ** 2 * (J[0, 0] - J[2, 2]) * math.sin(2 * th)
    assert abs(tau[1]) == pytest.approx(esperado, rel=1e-9)
    assert abs(tau[0]) < 1e-15 and abs(tau[2]) < 1e-15
    assert 3 * n ** 2 * 0.035 == pytest.approx(1.2e-7, rel=0.2)


def test_par_aerodinamico_placa_plana():
    """Con el viento solo sobre la cara +x (v = -x_body... el satelite se mueve
    hacia +x) la fuerza es F = -1/2 rho Cd A v^2 en x, aplicada a (dx/2 - cg_x)
    en x y (0 - cg_y, 0 - cg_z) en el plano: tau = brazo x F. Se compara
    componente a componente (analitico) con rho fijado."""
    sat = F.cubesat_3u()
    r = np.array([6928.0, 0, 0])
    # velocidad inercial tal que la velocidad RELATIVA al aire (v - w x r) sea
    # exactamente 7.5 km/s hacia +x del cuerpo (cuerpo = inercial)
    w_e = np.cross([0, 0, F.OMEGA_TIERRA], r)
    v = np.array([7.5, 0, 0]) + w_e
    rho = 1e-12
    tau = F.par_aerodinamico(sat, np.array([1.0, 0, 0, 0]), r, v, rho)
    dx, dy, dz = sat.dims
    Fx = -0.5 * rho * sat.cd * (dy * dz) * (7500.0 ** 2)
    arm = np.array([dx / 2, 0, 0]) - sat.cg
    esperado = np.cross(arm, [Fx, 0, 0])
    assert np.allclose(tau, esperado, rtol=1e-9, atol=1e-18)
    # orden de magnitud a 550 km con el CG a ~2 cm: 1e-8 N m
    assert 1e-9 < np.linalg.norm(F.par_aerodinamico(sat, np.array([1.0, 0, 0, 0]), r, v, 7e-13)) < 1e-6


def test_atmosfera_monotona_y_continua():
    """La densidad decrece con la altura y las capas se empalman con saltos
    < 25 % (la tabla exponencial por capas no es continua por construccion)."""
    h = np.arange(300, 900, 5.0)
    rho = np.array([F.densidad_atmosfera(x) for x in h])
    assert np.all(np.diff(rho) < 0)
    assert np.max(np.abs(np.diff(np.log(rho)))) < 0.25
    # empalme de capas (continuidad de la tabla): rho0 de cada capa = rho al final de la anterior
    for (h0, r0, H), (h1, r1, _) in zip(F.CAPAS_ATMOSFERA, F.CAPAS_ATMOSFERA[1:]):
        assert r1 == pytest.approx(r0 * math.exp(-(h1 - h0) / H), rel=0.01), h1


# ------------------------------------------------------------- campo geomagnetico

def _rad(lat, lon, r=F.A_GEOMAG_KM):
    la, lo = math.radians(lat), math.radians(lon)
    return r * np.array([math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)])


def test_dipolo_cifras_conocidas():
    """El dipolo del IGRF-13 2020 tiene B0 = sqrt(g10^2 + g11^2 + h11^2) = 29 8xx nT
    (la cifra '30 000 nT' del campo ecuatorial); en el polo magnetico vale 2 B0 y
    en el ecuador magnetico B0, ambos por (a/r)^3 (dipolo centrado). Se
    comprueba con puntos construidos a partir del eje del dipolo, que sale de
    los mismos coeficientes."""
    g10, g11, h11 = -29404.8, -1450.9, 4652.5
    B0 = math.sqrt(g10 ** 2 + g11 ** 2 + h11 ** 2)
    assert 29_000 < B0 < 30_500
    eje = F.unitario([g11, h11, g10])               # eje del dipolo (hacia el que apunta -m)
    anio = 2020.0
    r = 7000.0
    polo = r * eje
    b = np.linalg.norm(F.campo_geomagnetico(polo, anio, 1))
    assert b == pytest.approx(2 * B0 * (F.A_GEOMAG_KM / r) ** 3, rel=1e-9)
    perp = F.unitario(np.cross(eje, [0.3, 0.1, 0.9]))
    for ang in (0.0, 1.0, 2.5):
        p = r * (math.cos(ang) * perp + math.sin(ang) * np.cross(eje, perp))
        assert np.linalg.norm(F.campo_geomagnetico(p, anio, 1)) == pytest.approx(
            B0 * (F.A_GEOMAG_KM / r) ** 3, rel=1e-9)


def test_campo_es_el_gradiente_del_potencial():
    """B = -grad V (diferencias finitas centrales de V, h = 1 m): valida las
    recurrencias de Legendre y sus derivadas de TODOS los grados (1..13) con
    el codigo del potencial, que no usa las derivadas. Tolerancia: 2e-6 nT
    relativo al campo (error de la diferencia finita ~ h^2 V''')."""
    anio = 2026.75
    for (lat, lon, alt) in ((20.0, -99.0, 600.0), (-60.0, 40.0, 550.0), (85.0, 10.0, 700.0),
                            (-26.0, -52.0, 500.0)):
        r = _rad(lat, lon, F.A_GEOMAG_KM + alt)
        h = 1e-3          # km
        g = np.zeros(3)
        for i in range(3):
            d = np.zeros(3)
            d[i] = h
            vp = F.potencial_geomagnetico(r + d, anio)
            vm = F.potencial_geomagnetico(r - d, anio)
            # B = -grad V con V en nT km: dV/dx en nT km / km = nT
            g[i] = -(vp - vm) / (2 * h)
        b = F.campo_geomagnetico(r, anio)
        # B en ECEF cartesiano = -grad V cartesiano
        assert np.allclose(b, g, atol=2e-6 * np.linalg.norm(b) + 1e-4), (b, g)


def test_legendre_contra_scipy():
    """P_n^m de Schmidt semi-normalizado = (-1)^m sqrt(2 (n-m)!/(n+m)!) lpmv(m, n, x)
    para m > 0 (y lpmv para m = 0), con scipy.special como referencia
    independiente (lpmv incluye la fase de Condon-Shortley, que IGRF no usa)."""
    sp = pytest.importorskip('scipy.special')
    th = 1.1
    c, s = math.cos(th), math.sin(th)
    P, dP, Q = F._legendre(13, c, s)
    for n in range(1, 14):
        for m in range(0, n + 1):
            k = 1.0 if m == 0 else math.sqrt(2.0 * math.factorial(n - m) / math.factorial(n + m))
            ref = (-1) ** m * k * sp.lpmv(m, n, c)
            assert P[n, m] == pytest.approx(ref, rel=1e-9, abs=1e-12), (n, m)
            if m > 0:
                assert Q[n, m] == pytest.approx(ref / s, rel=1e-9, abs=1e-12), (n, m)
            # derivada por diferencias finitas de scipy
            e = 1e-6
            refd = ((-1) ** m * k * (sp.lpmv(m, n, math.cos(th + e)) - sp.lpmv(m, n, math.cos(th - e))) / (2 * e))
            assert dP[n, m] == pytest.approx(refd, rel=1e-5, abs=1e-8), (n, m)


def test_campo_anclas_publicadas():
    """Valores de anclaje del campo principal (IGRF/WMM, intensidad total F en
    la superficie, epoca 2020): Anomalia del Atlantico Sur (lat -26, lon -52)
    ~ 22 500-23 500 nT (el minimo mundial de F, ~22 800 nT); ecuador peruano
    (lat -12, lon -77, Jicamarca) ~ 24 000-27 500 nT (orden de magnitud); zona del polo norte
    magnetico (lat 80, lon -72) ~ 54 000-60 000 nT. Rangos tomados de las cartas
    mundiales (orden de magnitud, no una referencia numerica exacta: la
    verificacion rigurosa son las pruebas de Legendre y del gradiente). Con r = a_geomag (esfera): la
    diferencia geocentrica/geodesica es < 1 % en F."""
    for (lat, lon, lo, hi) in ((-26, -52, 22_000, 24_000), (-12, -77, 24_000, 27_500),
                               (80, -72, 54_000, 60_500)):
        f = np.linalg.norm(F.campo_geomagnetico(_rad(lat, lon), 2020.0))
        assert lo < f < hi, (lat, lon, f)


def test_campo_en_orbita_leo():
    """A 600 km de altura el campo total oscila entre ~20 y ~48 uT a lo largo
    de una orbita polar (min en la SAA, max cerca de los polos magneticos)."""
    r = 6978.0
    f = []
    for u in np.linspace(0, 2 * math.pi, 361):
        p = np.array([r * math.cos(u), 0, r * math.sin(u)])      # pasa por los polos
        for lon in (0.0, 1.0, 2.0):
            R3 = np.array([[math.cos(lon), -math.sin(lon), 0], [math.sin(lon), math.cos(lon), 0], [0, 0, 1]])
            f.append(np.linalg.norm(F.campo_geomagnetico(R3 @ p, 2026.75)))
    f = np.array(f)
    assert 17_000 < f.min() < 30_000
    assert 40_000 < f.max() < 56_000


def test_error_del_modelo_de_a_bordo_baja_con_el_grado():
    """El error del campo de a bordo respecto a IGRF-13 completo baja
    monotonamente con el grado y se anula en el grado 13. El dipolo
    (grado 1) deja ~32 % del campo y ~13 deg de direccion (rms en una orbita
    polar de 600 km, medido), el grado 4 ~4 % y el grado 8 ~0.1 %: es el limite
    que el modelo de campo impone a la actitud (ver README)."""
    e = [F.error_modelo_campo(6978.0, 97.5, k, 120) for k in (1, 2, 4, 8, 13)]
    rms = [x['rms_rel'] for x in e]
    assert all(a > b for a, b in zip(rms, rms[1:])), rms
    assert rms[-1] < 1e-12
    assert 0.10 < rms[0] < 0.45
    assert 4.0 < e[0]['rms_deg'] < 20.0
    assert rms[2] < 0.06 and rms[3] < 0.005


# ------------------------------------------------------------- Sol

def test_sol_posicion():
    """Algoritmo 29 de Vallado: (a) ejemplo 5-1 del libro (2006-04-02 0h UT1,
    JD 2453827.5): r = [146 186 212, 28 788 976, 12 481 064] km; (b) en el
    equinoccio de marzo la declinacion es ~0 y en el solsticio de junio
    ~ +23.44 deg (fisica, el resultado no depende del libro); (c) la distancia
    es 0.983 UA en el perihelio (inicio de enero) y 1.017 UA en el afelio
    (inicio de julio)."""
    r = F.sol_eci_km(2453827.5)
    ref = np.array([146186212.0, 28788976.0, 12481064.0])
    assert math.degrees(F.angulo_entre(r, ref)) < 0.01
    assert np.linalg.norm(r) == pytest.approx(np.linalg.norm(ref), rel=2e-4)

    def dec(fecha):
        jd, fr = orbit.a_jd(fecha)
        s = F.sol_eci_km(jd + fr)
        return math.degrees(math.asin(s[2] / np.linalg.norm(s))), np.linalg.norm(s) / F.UA_KM
    d_eq, _ = dec(dt.datetime(2026, 3, 20, 14, 46, tzinfo=UTC))
    d_sol, _ = dec(dt.datetime(2026, 6, 21, 8, 24, tzinfo=UTC))
    _, d_per = dec(dt.datetime(2026, 1, 3, 17, 0, tzinfo=UTC))
    _, d_afe = dec(dt.datetime(2026, 7, 6, 17, 0, tzinfo=UTC))
    assert abs(d_eq) < 0.05
    assert d_sol == pytest.approx(23.44, abs=0.02)
    assert d_per == pytest.approx(0.9833, abs=0.0005)
    assert d_afe == pytest.approx(1.0167, abs=0.0005)


def test_eclipse_cilindrico():
    """Detras de la Tierra a 7000 km del centro, sobre el eje: eclipse; a un
    lado a mas de R_T del eje: no; del lado del Sol: no. La fraccion de
    una orbita circular de 600 km con el Sol en el plano de la orbita es
    asin(R_T/a)/pi (derivacion geometrica): ~ 0.37 de la orbita."""
    s = np.array([1.0, 0, 0]) * F.UA_KM
    assert F.en_eclipse([-7000.0, 0, 0], s)
    assert not F.en_eclipse([-7000.0, 6500.0, 0], s)
    assert not F.en_eclipse([7000.0, 0, 0], s)
    a = F.R_TIERRA_KM + 600.0
    n = 3600
    frac = np.mean([F.en_eclipse(a * np.array([math.cos(u), math.sin(u), 0]), s)
                    for u in np.linspace(0, 2 * math.pi, n, endpoint=False)])
    assert frac == pytest.approx(math.asin(F.R_TIERRA_KM / a) / math.pi, abs=2e-3)


# ------------------------------------------------------------- estimacion

def test_triad_exacto_y_con_ruido():
    """TRIAD sin ruido recupera la actitud exactamente; con ruido, el error
    del eje del vector primario es ~ sigma_1 y el giro alrededor de el sigue
    a sigma_2/sin(angulo entre vectores) (Shuster y Oh 1981): se verifica el
    orden en 500 realizaciones (sigma_1 = 0.5 deg, sigma_2 = 3 deg)."""
    rng = np.random.default_rng(7)
    r1, r2 = F.unitario([1, 0.2, 0.1]), F.unitario([0.2, 1, -0.4])
    q = F.q_aleatorio(rng)
    R = F.q_a_dcm(q)
    qe = F.triad(R.T @ r1, r1, R.T @ r2, r2)
    assert np.allclose(F.q_a_dcm(qe), R, atol=1e-12)
    with pytest.raises(ValueError):
        F.triad(R.T @ r1, r1, R.T @ r1, r1)           # paralelos
    s1, s2 = math.radians(0.5), math.radians(3.0)
    e1, e2 = [], []
    for _ in range(500):
        q = F.q_aleatorio(rng)
        R = F.q_a_dcm(q)
        qe = F.triad(F._perturbar_unitario(R.T @ r1, s1, rng), r1,
                     F._perturbar_unitario(R.T @ r2, s2, rng), r2)
        # error del eje del vector 1 (cuerpo) y giro alrededor del mismo
        e1.append(F.angulo_entre(F.q_a_dcm(qe) @ (R.T @ r1), r1))
        d = F.q_a_rotvec(F.q_error(q, qe))
        e2.append(abs(d @ (R.T @ r1)))
    sin_ang = np.linalg.norm(np.cross(r1, r2))
    assert np.sqrt(np.mean(np.square(e1))) == pytest.approx(math.sqrt(2) * s1, rel=0.15)
    assert np.sqrt(np.mean(np.square(e2))) == pytest.approx(s2 / sin_ang, rel=0.2)


def _mc_mekf(n_runs, t_obs, dt_s=1.0, seed=11, sigma_vec=math.radians(0.5), vectores=2):
    """Monte Carlo de MEKF CONSISTENTE: el modelo de la verdad es exactamente el
    del filtro (giroscopo de Farrenkopf, vectores unitarios con error
    angular isotropo) y el estado inicial se sortea de la P0 que el
    filtro cree. Devuelve NEES (n_runs, len(t_obs)) y errores."""
    rI = [F.unitario([1, 0.3, 0.2]), F.unitario([0.1, -1, 0.5])][:vectores]
    sv, su = 1.0e-4, 1.0e-6
    sa0, sb0 = math.radians(3.0), 1.0e-3
    rng = np.random.default_rng(seed)
    nees = np.zeros((n_runs, len(t_obs)))
    ang = np.zeros((n_runs, len(t_obs)))
    for i in range(n_runs):
        q = F.q_aleatorio(rng)
        b = rng.normal(0.0, sb0, 3)
        w = np.array([0.004, -0.002, 0.003])
        gy = F.Giroscopo(sv, su, b, rng)
        qe = F.q_mult(q, F.q_de_rotvec(-rng.normal(0.0, sa0, 3)))   # q = qe (x) dq
        mk = F.Mekf(qe, np.zeros(3), sa0, sb0, sv, su)     # b_est = 0: error de sesgo = b ~ N(0, sb0^2)
        j = 0
        for k in range(1, int(max(t_obs) / dt_s) + 1):
            q = F.q_norm(F.q_mult(q, F.q_de_rotvec(w * dt_s)))
            mk.propagar(gy.medir(w, dt_s), dt_s)
            R = F.q_a_dcm(q)
            for r in rI:
                mk.actualizar_vector(F._perturbar_unitario(R.T @ r, sigma_vec, rng), r, sigma_vec)
            if j < len(t_obs) and k * dt_s >= t_obs[j]:
                nees[i, j] = F.nees(mk.q, mk.b, mk.P, q, gy.b)
                ang[i, j] = math.degrees(np.linalg.norm(F.q_a_rotvec(F.q_error(mk.q, q))))
                j += 1
    return nees, ang


def test_chi2_wilson_hilferty_contra_scipy():
    """La aproximacion de Wilson-Hilferty del cuantil chi-cuadrado (la que usa
    la prueba de consistencia, para no depender de scipy) coincide con
    scipy.stats.chi2.ppf a < 0.3 % para k >= 100."""
    st = pytest.importorskip('scipy.stats')
    for k in (100, 240, 600):
        for p in (0.005, 0.025, 0.975, 0.995):
            assert F.chi2_cuantil(p, k) == pytest.approx(st.chi2.ppf(p, k), rel=3e-3)


def test_mekf_converge_y_es_consistente_nees():
    """MEKF de 6 estados: (a) el error de actitud converge a ~0.1 deg con un
    sol/campo a 0.5 deg (fraccion de grado, no el error inicial de ~5 deg);
    (b) el filtro es CONSISTENTE: el NEES promedio sobre N corridas
    independientes de un escenario con la misma estadistica que el modelo
    del filtro debe caer en la banda de confianza de dos colas al 99 % de
    chi-cuadrado con 6N grados de libertad (promedio de N variables chi2_6):
    [chi2_.005(6N)/N, chi2_.995(6N)/N] (Bar-Shalom, Li y Kirubarajan, sec.
    5.4). Se verifica en 3 instantes separados (t = 60, 200, 500 s). Es un
    contraste de las ECUACIONES del filtro (Phi, Q, H, reset) con una verdad
    generada aparte; con modelo de campo erroneo el filtro NO es consistente
    salvo que se infle R (se hace en el escenario, ver README)."""
    n = 60
    t_obs = [60.0, 200.0, 500.0]
    nees, ang = _mc_mekf(n, t_obs)
    lo, hi = F.chi2_cuantil(0.005, 6 * n) / n, F.chi2_cuantil(0.995, 6 * n) / n
    media = nees.mean(axis=0)
    assert np.all((media > lo) & (media < hi)), (media, lo, hi)
    assert np.sqrt(np.mean(ang[:, -1] ** 2)) < 0.15      # converge: ~0.1 deg rms


# ------------------------------------------------------------- control

def test_control_pd_apunta_y_respeta_los_limites():
    """Giro de 120 deg sin pares externos con la ley PD con limite de
    velocidad: (a) converge a < 0.05 deg y a velocidad angular ~0 en 600 s;
    (b) el momento de las ruedas nunca excede h_max (el limite de velocidad
    w_lim = h_max/(2 J) lo garantiza en un giro desde el reposo); (c) se
    conserva el momento angular total (sin pares externos: lo que ganan las
    ruedas lo pierde el cuerpo); (d) la rapidez del cuerpo no pasa de w_lim
    mas un 25 % de sobreimpulso."""
    sat = F.cubesat_3u()
    ev = np.linalg.eigvalsh(sat.J)
    w_lim = 0.5 * sat.h_max / ev[-1]
    q_ref = np.array([1.0, 0, 0, 0])
    q0 = F.q_de_eje_angulo([0.3, 0.5, 0.8], math.radians(120))
    y = F.estado_inicial(q0, np.zeros(3), np.zeros(3))
    H0 = _H_inercial(y, sat)
    h_max_vista, w_max_vista = 0.0, 0.0
    for k in range(600):
        tau = F.control_actitud(y[:4], y[4:7], y[7:], q_ref, np.zeros(3), np.zeros(3),
                                sat, wn=0.1, zeta=0.9, w_lim=w_lim)
        u = F.asignar_ruedas(tau, sat)
        y, _ = F.propagar(y, sat, u, np.zeros(3), 1.0, 0.25)
        h_max_vista = max(h_max_vista, np.max(np.abs(y[7:])))
        w_max_vista = max(w_max_vista, np.linalg.norm(y[4:7]))
    assert math.degrees(F.angulo_entre(F.q_a_dcm(y[:4]) @ [0, 0, 1], [0, 0, 1])) < 0.05
    assert math.degrees(np.linalg.norm(F.q_a_rotvec(F.q_error(q_ref, y[:4])))) < 0.05
    assert np.linalg.norm(y[4:7]) < 1e-4
    assert h_max_vista <= sat.h_max + 1e-12
    assert w_max_vista < 1.25 * w_lim
    assert np.linalg.norm(_H_inercial(y, sat) - H0) < 1e-9


def test_control_pd_sigue_una_referencia_en_movimiento():
    """Con la prealimentacion de w_ref y a_ref el error de seguimiento de una
    referencia que gira a 0.3 deg/s (0.005 rad/s) con aceleracion cero es
    ~0 en regimen (el termino de Euler w x J w ya esta cancelado), < 0.01 deg
    tras 600 s."""
    sat = F.cubesat_3u()
    wr = np.array([0.001, -0.002, 0.004])
    q_ref = F.q_aleatorio(np.random.default_rng(9))
    y = F.estado_inicial(F.q_mult(q_ref, F.q_de_rotvec([0.05, -0.04, 0.03])), wr, np.zeros(3))
    for k in range(900):
        tau = F.control_actitud(y[:4], y[4:7], y[7:], q_ref, wr, np.zeros(3), sat,
                                0.1, 0.9, 0.0176)
        y, _ = F.propagar(y, sat, F.asignar_ruedas(tau, sat), np.zeros(3), 1.0, 0.25)
        q_ref = F.q_norm(F.q_mult(q_ref, F.q_de_rotvec(wr * 1.0)))
    assert math.degrees(np.linalg.norm(F.q_a_rotvec(F.q_error(q_ref, y[:4])))) < 0.01


def test_desaturacion_magnetica_derivacion():
    """El dipolo de desaturacion produce tau_m = m x B = -(1/tau) h_perp: la
    parte de h perpendicular a B decae con constante tau y la paralela no
    cambia (no se puede descargar a lo largo de B). Con dipolo sin saturar
    es una igualdad exacta."""
    rng = np.random.default_rng(4)
    B = rng.normal(size=3) * 3e-5
    h = np.array([1e-3, -5e-4, 8e-4])
    tau_d = 600.0
    m = F.desaturar(B, h, tau_d, 1e9)
    tm = np.cross(m, B)
    bh = B / np.linalg.norm(B)
    h_perp = h - (h @ bh) * bh
    assert np.allclose(tm, -h_perp / tau_d, atol=1e-18, rtol=1e-9)
    assert abs(tm @ bh) < 1e-18
    m_sat = F.desaturar(B, h, tau_d, 0.01)
    assert np.max(np.abs(m_sat)) <= 0.01 + 1e-15


def test_ganancia_bdot_derivacion():
    """Con w >> 2n, dB/dt = -w x B; entonces tau = m x B = -k B^2 w_perp: la
    componente paralela a B no se amortigua y la perpendicular decae con
    tasa k B^2 / J. La ganancia ganancia_bdot(J, B, tau) da exactamente esa
    tasa 1/tau sobre el eje de inercia J (igualdad algebraica)."""
    J, Brms, tau = 0.0067, 4e-5, 50.0
    k = F.ganancia_bdot(J, Brms, tau)
    assert k * Brms ** 2 / J == pytest.approx(1.0 / tau, rel=1e-12)
    B = np.array([3e-5, -1e-5, 2e-5])
    w = np.array([0.1, 0.05, -0.08])
    bdot = -np.cross(w, B)
    tm = np.cross(-k * bdot, B)
    bh = B / np.linalg.norm(B)
    w_perp = w - (w @ bh) * bh
    assert np.allclose(tm, -k * (B @ B) * w_perp, rtol=1e-9, atol=1e-18)


# ------------------------------------------------------------- gradiente de gravedad

def _orbita_gg(J, q0_extra, n_orbitas, dt_s=5.0, inc_deg=0.0):
    """Integra solo con gradiente de gravedad sobre una orbita circular
    (a = 6928 km) a partir de LVLH girado `q0_extra` (cuaternion de
    perturbacion en el cuerpo). Devuelve t y los angulos (roll, pitch, yaw
    pequenos = 2 x parte vectorial de q_ref^-1 q) en el cuerpo."""
    from sat_adcs.misiones import q_ref_lvlh
    a = 6928.0
    n = math.sqrt(F.MU_M3_S2 * 1e-9 / a ** 3)
    sat = F.Satelite(J=J, A=np.eye(3), h_max=1.0, tau_max=1.0, m_max=0.0, masa=4.0,
                     dims=(0.1, 0.1, 0.34), cg=np.zeros(3), m_res=np.zeros(3))
    r, v = F.r_circular(a, math.radians(inc_deg), 0.0)
    qr = q_ref_lvlh(r, v)
    q0 = F.q_mult(qr, q0_extra)
    w0 = np.array([0.0, -n, 0.0])               # velocidad orbital: -n a lo largo de +Y (Y = -normal)
    # w en el cuerpo perturbado: el mismo vector inercial n*h^
    w0 = F.q_a_dcm(q0).T @ (F.unitario(np.cross(r, v)) * n)
    y = F.estado_inicial(q0, w0, np.zeros(3))
    T = 2 * math.pi / n
    ts, ang = [], []
    t = 0.0
    while t < n_orbitas * T:
        tt = t + 0.5 * dt_s

        rr, _ = F.r_circular(a, math.radians(inc_deg), tt)

        def tau(q, w, rr=rr):
            return F.par_gradiente_gravedad(J, q, rr)
        y, _ = F.propagar(y, sat, np.zeros(3), tau, dt_s, dt_s)
        t += dt_s
        r, v = F.r_circular(a, math.radians(inc_deg), t)
        ang.append(F.q_a_rotvec(F.q_error(q_ref_lvlh(r, v), y[:4])))
        ts.append(t)
    return n, np.array(ts), np.array(ang)


def test_gradiente_de_gravedad_equilibrio_estable():
    """Orientacion de equilibrio del gradiente de gravedad: con el eje de MENOR
    inercia en la vertical local y el de MAYOR en la normal de la orbita
    (Iy > Ix > Iz), LVLH es un equilibrio ESTABLE (Wie, 'Space Vehicle
    Dynamics and Control', cap. 6: Iy > Ix > Iz). Con una perturbacion de 1 deg
    en cada eje: (a) el angulo se mantiene acotado (< 3 x la perturbacion)
    durante 3 orbitas; (b) el cabeceo oscila con la frecuencia analitica
    w_p = n sqrt(3 (Ix - Iz)/Iy) (se mide el periodo por cruces por cero y se
    compara a < 1 %); (c) sin perturbacion, LVLH se mantiene a < 1e-6 rad."""
    J = np.diag([0.040, 0.050, 0.0067])
    th = math.radians(1.0)
    n, t, ang = _orbita_gg(J, F.q_de_rotvec([th, th, th]), 3)
    assert np.max(np.abs(ang)) < 3 * th * 1.001
    # cabeceo = componente y; periodo medido
    p = ang[:, 1]
    cruces = [t[i] for i in range(1, len(p)) if p[i - 1] < 0.0 <= p[i]]
    periodo = np.mean(np.diff(cruces))
    w_p = n * math.sqrt(3 * (J[0, 0] - J[2, 2]) / J[1, 1])
    assert periodo == pytest.approx(2 * math.pi / w_p, rel=0.01)
    # paso de 1 s: el ambiente (posicion) se congela por paso, error ~ (n dt)^2
    _, _, ang0 = _orbita_gg(J, np.array([1.0, 0, 0, 0]), 1, dt_s=1.0)
    assert np.max(np.abs(ang0)) < 1e-6


def test_gradiente_de_gravedad_equilibrio_inestable():
    """Con el eje de MAYOR inercia en la vertical (Iz > Ix, Iy pequeno) el
    termino de cabeceo es 3 n^2 (Ix - Iz) < 0: inestable, con crecimiento
    exponencial de tasa n sqrt(3 (Iz - Ix)/Iy). Una perturbacion de 1 deg
    debe crecer > 10 veces en 1.5 orbitas (la tasa da e^(1.5*2pi*sqrt(3*.../Iy))
    mucho mayor): confirma que la estabilidad de la prueba anterior viene del
    reparto de inercias y no de un artefacto del integrador."""
    J = np.diag([0.040, 0.0067, 0.050])
    th = math.radians(1.0)
    n, t, ang = _orbita_gg(J, F.q_de_rotvec([0, th, 0]), 1.5)
    assert np.max(np.abs(ang[:, 1])) > 10 * th
