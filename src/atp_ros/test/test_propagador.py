"""Validacion de propagador.py (equipo N, determinacion de orbita): gravedad,
zonales, atmosfera, arrastre, integradores, STM y comparacion con SGP4.

Referencias independientes: formulas analiticas de Vallado (tasas seculares
de J2, forma explicita de la aceleracion J2), conservacion de energia,
solucion circular de Kepler, diferencias finitas del potencial, y SGP4
(libreria sgp4) para la comparacion de dias. El TLE de la ISS esta EMBEBIDO
(epoca 2026-10-01): el resultado no depende de que data/tle se refresque.
"""
import math

import numpy as np
import pytest

from atp_ros import od
from atp_ros import orbit as o
from atp_ros import propagador as P

ISS_L1 = '1 25544U 98067A   26274.82022115  .00003852  00000+0  78825-4 0  9993'
ISS_L2 = '2 25544  51.6316 132.3693 0006937 211.2109 148.8469 15.48706258588289'
# Coeficiente balistico EFECTIVO de la ISS: Cd A/m fisico 0.0105 m2/kg por el
# factor 0.406 que minimiza la diferencia con SGP4 en 24 h (rms 0.17 km). Ver
# docs/OD.md: el arrastre de SGP4 usa su propia atmosfera y el B* del TLE.
P_ISS = 0.406
BC_ISS = 0.0105


@pytest.fixture(scope='module')
def iss():
    return o.tle_desde_lineas('ISS (ZARYA)', ISS_L1, ISS_L2)


@pytest.fixture(scope='module')
def x_iss(iss):
    x, reloj = od.estado_tle(iss)
    return x


# ------------------------------------------------ constantes y gravedad

def test_constantes_gravitatorias_son_las_de_sgp4():
    from sgp4.earth_gravity import wgs72, wgs84
    for mio, ref in ((P.WGS72, wgs72), (P.WGS84, wgs84)):
        assert mio.mu == ref.mu and mio.re == ref.radiusearthkm
        assert (mio.j2, mio.j3, mio.j4) == (ref.j2, ref.j3, ref.j4)


def test_gravedad_central_pura():
    r = np.array([[7000.0, 100.0, -300.0]])
    a = P.aceleracion_gravedad(r, P.WGS84, 0)[0]
    rn = np.linalg.norm(r[0])
    assert np.allclose(a, -P.WGS84.mu * r[0] / rn ** 3, rtol=1e-14, atol=0)


def test_j2_contra_forma_explicita_de_vallado():
    """Vallado 2013, Eq. 8-31: la forma clasica de la aceleracion por J2."""
    g = P.WGS84
    for r in ([6800.0, 500.0, 2500.0], [-3000.0, 5000.0, 4000.0], [0.0, 6700.0, 100.0]):
        x, y, z = r
        rn = math.sqrt(x * x + y * y + z * z)
        k = -1.5 * g.j2 * g.mu * g.re ** 2 / rn ** 4
        a_j2 = k * np.array([x / rn * (1 - 5 * z * z / rn ** 2),
                             y / rn * (1 - 5 * z * z / rn ** 2),
                             z / rn * (3 - 5 * z * z / rn ** 2)])
        total = P.aceleracion_gravedad(np.array([r]), g, 2)[0]
        central = P.aceleracion_gravedad(np.array([r]), g, 0)[0]
        assert np.allclose(total - central, a_j2, rtol=1e-12, atol=1e-18)


@pytest.mark.parametrize('n', [2, 3, 4])
def test_zonales_son_el_gradiente_del_potencial(n):
    """J2, J3 y J4 contra diferencias finitas del potencial zonal
    U_n = -mu J_n Re^n P_n(z/r) / r^(n+1) (independiente de la derivacion a mano)."""
    g = P.WGS84
    jn = {2: g.j2, 3: g.j3, 4: g.j4}[n]
    leg = {2: lambda s: (3 * s * s - 1) / 2, 3: lambda s: (5 * s ** 3 - 3 * s) / 2,
           4: lambda s: (35 * s ** 4 - 30 * s * s + 3) / 8}[n]

    def U(r):
        rn = np.linalg.norm(r)
        return -g.mu * jn * g.re ** n * leg(r[2] / rn) / rn ** (n + 1)
    r0 = np.array([4500.0, -3200.0, 3900.0])
    h = 1e-2
    fd = np.array([(U(r0 + h * e) - U(r0 - h * e)) / (2 * h) for e in np.eye(3)])
    a = (P.aceleracion_gravedad(np.array([r0]), g, n)[0]
         - P.aceleracion_gravedad(np.array([r0]), g, n - 1)[0])
    assert np.allclose(a, fd, rtol=1e-7)


def test_aceleracion_escalar_igual_a_la_vectorizada():
    rng = np.random.default_rng(3)
    for grado in (0, 2, 3, 4):
        m = P.Modelo(grado=grado, rho_factor=1.7)
        g = P.aceleracion_escalar(m)
        for _ in range(10):
            r = rng.normal(size=3)
            r = r / np.linalg.norm(r) * rng.uniform(6650, 7400)
            v = rng.normal(size=3)
            v = v / np.linalg.norm(v) * 7.5
            p = rng.uniform(0.3, 2.0)
            a1 = P.aceleracion(r[None], v[None], np.array([p]), m)[0]
            a2 = np.array(g(*r, *v, p))
            assert np.abs(a1 - a2).max() < 1e-15 * np.abs(a1).max() * 10


def test_jacobiano_analitico_igual_a_paso_complejo_y_a_diferencias_finitas():
    rng = np.random.default_rng(4)
    for grado in (0, 2, 4):
        m = P.Modelo(grado=grado)
        jac = P.jacobiano_escalar(m)
        r = np.array([4200.0, -3800.0, 3900.0])
        v = np.array([-4.1, 3.2, 4.6])
        p = 1.3
        a, dr, dv, dp = jac(*r, *v, p)
        a2, A = P.jacobiano_aceleracion(m, r, v, p, True)
        assert np.abs(a - a2).max() < 1e-15
        assert np.abs(dr - A[3:6, 0:3]).max() < 1e-12 * np.abs(dr).max()
        assert np.abs(dv - A[3:6, 3:6]).max() < 1e-12 * np.abs(dv).max()
        assert np.abs(dp - A[3:6, 6]).max() < 1e-12 * np.abs(dp).max()
        # diferencias finitas (central) de la aceleracion, independientes
        for j in range(3):
            e = np.zeros(3)
            e[j] = 1e-3
            fd = (P.acel1(r + e, v, m, p) - P.acel1(r - e, v, m, p)) / 2e-3
            assert np.abs(fd - dr[:, j]).max() < 1e-6 * np.abs(dr).max()
            fd = (P.acel1(r, v + e, m, p) - P.acel1(r, v - e, m, p)) / 2e-3
            assert np.abs(fd - dv[:, j]).max() < 1e-4 * np.abs(dv).max()
        fd = (P.acel1(r, v, m, p + 0.1) - P.acel1(r, v, m, p - 0.1)) / 0.2   # lineal en p
        assert np.abs(fd - dp).max() < 1e-6 * np.abs(dp).max()


# ------------------------------------------------ atmosfera y arrastre

def test_atmosfera_continua_entre_capas():
    """La tabla se transcribio: una cifra mal copiada rompe la continuidad
    (rho0_i exp(-dh/H_i) debe empatar con rho0_{i+1}); con capas de hasta
    100 km la mayor discontinuidad de la propia tabla es ~14 %."""
    t = P.ATMOSFERA_VALLADO
    for (h0, r0, H), (h1, r1, _) in zip(t[:-1], t[1:]):
        assert abs(r0 * math.exp(-(h1 - h0) / H) / r1 - 1.0) < 0.15, (h0, h1)


def test_atmosfera_contra_us_standard_1976():
    """Valores de la US Standard Atmosphere 1976 (atmosfera media) que la
    exponencial de Vallado debe reproducir a mejor de 40 % (la diferencia
    entre modelos de atmosfera es de ese orden o mayor)."""
    us76 = {200.0: 2.541e-10, 300.0: 1.916e-11, 400.0: 2.803e-12,
            500.0: 5.215e-13, 600.0: 1.137e-13}
    for h, rho in us76.items():
        r = float(P.ATMOSFERA.densidad(np.array([h]))[0])
        assert abs(r / rho - 1.0) < 0.40, (h, r, rho)
    h = np.linspace(100.0, 900.0, 200)
    assert np.all(np.diff(P.ATMOSFERA.densidad(h)) < 0.0)


def test_arrastre_decaimiento_circular_polar_contra_formula():
    """da/dt = -B rho sqrt(mu a) (circular, Tierra sin rotacion: polar). Con
    la densidad PROMEDIADA sobre la orbita (la altitud varia 21 km por el
    aplanamiento) el decaimiento numerico debe coincidir a 3 %. Se usa
    gravedad central (grado 0) para aislar el arrastre: con J2 el radio
    oscila varios km a lo largo de la orbita y la densidad media cambia ~9 %."""
    mu = P.WGS84.mu
    B = 0.01
    m = P.Modelo(grado=0, arrastre=True, cd_a_m=B)
    a0 = P.WGS84.re + 400.0
    x0 = np.array([a0, 0.0, 0.0, 0.0, 0.0, math.sqrt(mu / a0)])
    ts = np.arange(0.0, 86400.0, 30.0)
    Y = P.propagar(P.derivadas(m), x0, 0.0, ts, metodo='dp45', rtol=1e-10, atol=1e-10)
    a, *_ = P.elementos_clasicos(Y[:, :3], Y[:, 3:6], mu)
    n = int(round(2 * math.pi * math.sqrt(a0 ** 3 / mu) / 30.0))
    am = np.array([a[k:k + n].mean() for k in range(0, len(a) - n, n)])
    pend = np.polyfit(np.arange(len(am)) * n * 30.0, am, 1)[0] * 1000.0     # m/s
    lat = np.linspace(0, 2 * np.pi, 720, endpoint=False)
    h = a0 - P.WGS84.re * (1 - P.F_APLANAMIENTO * np.sin(lat) ** 2)
    rho = float(np.mean(P.ATMOSFERA.densidad(h)))
    teor = -B * rho * math.sqrt(mu * 1e9 * a0 * 1e3)                          # m/s
    assert pend == pytest.approx(teor, rel=0.03)


def test_arrastre_rotacion_de_la_atmosfera():
    """Una atmosfera que co-rota frena MENOS a una orbita progrado ecuatorial
    que a una retrograda (velocidad relativa menor): el cociente de las
    aceleraciones es ~ ((1 - w/n)/(1 + w/n))^2 ~ 0.77 en LEO."""
    m = P.Modelo(grado=0, arrastre=True, cd_a_m=0.01)
    r = np.array([[6778.0, 0.0, 0.0]])
    v = math.sqrt(P.WGS84.mu / 6778.0)
    a_pro = P.aceleracion_arrastre(r, np.array([[0.0, v, 0.0]]), np.array([1.0]), m)[0]
    a_retro = P.aceleracion_arrastre(r, np.array([[0.0, -v, 0.0]]), np.array([1.0]), m)[0]
    w_n = P.OMEGA_TIERRA * 6778.0 / v
    esperado = ((1 - w_n) / (1 + w_n)) ** 2
    assert np.linalg.norm(a_pro) / np.linalg.norm(a_retro) == pytest.approx(esperado, rel=1e-9)
    # el arrastre se opone a la velocidad relativa
    assert a_pro[1] < 0.0 < a_retro[1]


# ------------------------------------------------ integradores

def test_kepler_circular_exacto():
    """Gravedad central, orbita circular: la solucion exacta es r(t) = R rot(n t)."""
    mu = P.WGS84.mu
    R = 7000.0
    n = math.sqrt(mu / R ** 3)
    x0 = np.array([R, 0.0, 0.0, 0.0, R * n, 0.0])
    t = 12345.678
    m = P.Modelo(grado=0, arrastre=False)
    for kw, tol in ((dict(metodo='dp45', rtol=1e-12, atol=1e-12), 1e-6),
                    (dict(metodo='rk4', h=5.0), 1e-4)):
        y = P.propagar(P.derivadas(m), x0, 0.0, [t], **kw)[0]
        ex = np.array([R * math.cos(n * t), R * math.sin(n * t)])
        assert np.abs(y[:2] - ex).max() < tol, (kw, np.abs(y[:2] - ex).max())


def test_conservacion_de_energia_gravedad_central(x_iss):
    """Solo gravedad central: la energia especifica es constante. Deriva
    relativa en 1 dia (ISS): RK4 h=10 s < 1e-9; Dormand-Prince rtol 1e-11 < 3e-10."""
    m = P.Modelo(grado=0, arrastre=False)
    ts = np.arange(0.0, 86400.1, 120.0)
    for kw, tope in ((dict(metodo='rk4', h=10.0), 1e-9),
                     (dict(metodo='dp45', rtol=1e-11, atol=1e-11), 3e-10)):
        Y = P.propagar(P.derivadas(m), x_iss, 0.0, ts, **kw)
        E = P.energia(P.WGS84, Y[:, :3], Y[:, 3:6])
        assert np.ptp(E) / abs(E[0]) < tope, (kw, np.ptp(E) / abs(E[0]))


def test_orden_de_convergencia_de_rk4(x_iss):
    """Con h/2 el error global de RK4 baja ~16 veces (orden 4)."""
    m = P.Modelo(grado=2, arrastre=False)
    f = P.derivadas(m)
    T = 3000.0
    ref = P.propagar(f, x_iss, 0.0, [T], metodo='rk4', h=0.5)[0]
    e = [np.linalg.norm(P.propagar(f, x_iss, 0.0, [T], metodo='rk4', h=h)[0][:3] - ref[:3])
         for h in (40.0, 20.0, 10.0)]
    assert 12.0 < e[0] / e[1] < 20.0 and 12.0 < e[1] / e[2] < 20.0, e


def test_dormand_prince_acuerdo_con_rk4(x_iss):
    """En 6 h con J2+arrastre, el adaptativo (rtol 1e-9, el valor por defecto)
    coincide con RK4 h=2 s a < 1 cm y apretar la tolerancia converge al
    mismo valor: el error global del adaptativo es del orden del mm por hora."""
    m = P.Modelo(grado=2, arrastre=True, cd_a_m=0.0042)
    f = P.derivadas(m)
    T = 21600.0
    rk = P.propagar(f, x_iss, 0.0, [T], metodo='rk4', h=2.0)[0]
    d9 = P.propagar(f, x_iss, 0.0, [T], metodo='dp45')[0]
    d11 = P.propagar(f, x_iss, 0.0, [T], metodo='dp45', rtol=1e-11, atol=1e-11)[0]
    assert np.linalg.norm(d9[:3] - rk[:3]) < 1e-2
    assert np.linalg.norm(d11[:3] - rk[:3]) < 2e-3
    assert np.linalg.norm(d11[:3] - d9[:3]) < 1e-2


def test_propagacion_hacia_atras_vuelve(x_iss):
    m = P.Modelo(grado=2, arrastre=False)
    f = P.derivadas(m)
    y1 = P.propagar(f, x_iss, 0.0, [5000.0], metodo='dp45', rtol=1e-11, atol=1e-11)[0]
    y0 = P.propagar(f, y1, 5000.0, [0.0], metodo='dp45', rtol=1e-11, atol=1e-11)[0]
    assert np.linalg.norm(y0[:3] - x_iss[:3]) < 1e-5


# ------------------------------------------------ J2 secular (Vallado)

def _estado_kepler(a, e, i, raan, w, nu, mu):
    p = a * (1 - e * e)
    r_pf = np.array([p * math.cos(nu), p * math.sin(nu), 0.0]) / (1 + e * math.cos(nu))
    v_pf = math.sqrt(mu / p) * np.array([-math.sin(nu), e + math.cos(nu), 0.0])

    def R3(t):
        return np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])

    def R1(t):
        return np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    Q = R3(raan) @ R1(i) @ R3(w)
    return np.r_[Q @ r_pf, Q @ v_pf]


def test_elementos_clasicos_ida_y_vuelta():
    mu = P.WGS84.mu
    x = _estado_kepler(8000.0, 0.1, math.radians(40), math.radians(30), math.radians(60),
                       math.radians(25), mu)
    a, e, i, raan, w, nu = (float(v[0]) for v in P.elementos_clasicos(x[:3], x[3:], mu))
    assert (a, e) == pytest.approx((8000.0, 0.1), rel=1e-12)
    assert (math.degrees(i), math.degrees(raan), math.degrees(w), math.degrees(nu)) == \
        pytest.approx((40.0, 30.0, 60.0, 25.0), abs=1e-9)


def test_j2_deriva_secular_del_nodo_y_del_perigeo():
    """Con SOLO J2, la deriva del nodo y del perigeo en 2 dias coincide con
    las formulas analiticas de Vallado (Eq. 9-41/9-42, a primer orden en J2)
    a mejor de 0.5 % (medido: 0.12 % y 0.18 %)."""
    mu = P.WGS84.mu
    x0 = _estado_kepler(8000.0, 0.1, math.radians(40), math.radians(30), math.radians(60), 0.0, mu)
    m = P.Modelo(grado=2, arrastre=False)
    ts = np.arange(0.0, 2 * 86400.0, 30.0)
    Y = P.propagar(P.derivadas(m), x0, 0.0, ts, metodo='dp45', rtol=1e-10, atol=1e-10)
    a, e, i, raan, w, nu = P.elementos_clasicos(Y[:, :3], Y[:, 3:6], mu)
    s_raan = np.polyfit(ts, np.unwrap(raan), 1)[0]
    s_w = np.polyfit(ts, np.unwrap(w), 1)[0]
    t_raan, t_w = P.tasas_seculares_j2(a.mean(), e.mean(), i.mean())
    assert s_raan == pytest.approx(t_raan, rel=0.005)
    assert s_w == pytest.approx(t_w, rel=0.005)
    # el signo y la magnitud tienen sentido fisico (orbita progrado: nodo hacia el oeste)
    assert s_raan < 0.0 < s_w


def test_j2_inclinacion_heliosincrona():
    """A 800 km la inclinacion heliosincrona (~98.6 deg) da una precesion del
    nodo de +0.9856 deg/dia (una vuelta al anio): contra la formula de Vallado."""
    mu = P.WGS84.mu
    a = P.WGS84.re + 800.0
    i = math.radians(98.6)
    raan_dot, _ = P.tasas_seculares_j2(a, 0.0, i)
    assert math.degrees(raan_dot) * 86400.0 == pytest.approx(0.9856, abs=0.02)


# ------------------------------------------------ STM

def test_stm_contra_diferencias_finitas(x_iss):
    """La matriz de transicion (7x7 con el factor de arrastre) integrada por
    las ecuaciones variacionales coincide con el jacobiano por diferencias
    finitas del flujo, a 1e-6 relativo en 100 min."""
    m = P.Modelo(grado=4, cd_a_m=0.0042)
    y0 = np.r_[x_iss, 1.0]
    T = 6000.0
    kw = dict(metodo='rk4', h=20.0)
    Y = P.propagar(P.derivadas_stm(m, True), P.y0_stm(y0), 0.0, [T], **kw)[0]
    Phi = Y[7:].reshape(7, 7)
    f = P.derivadas(m)
    sc = np.array([1e-3] * 3 + [1e-6] * 3 + [1e-3])
    for j in range(7):
        d = np.zeros(7)
        d[j] = sc[j]
        yp = P.propagar(f, y0 + d, 0.0, [T], **kw)[0]
        ym = P.propagar(f, y0 - d, 0.0, [T], **kw)[0]
        fd = (yp - ym) / (2 * sc[j])
        assert np.abs(Phi[:, j] - fd).max() < 1e-6 * np.abs(fd).max(), j
    # el estado dentro de la integracion es el mismo que sin STM
    y = P.propagar(f, y0, 0.0, [T], **kw)[0]
    assert np.abs(Y[:7] - y).max() < 1e-9


def test_stm_propiedad_de_semigrupo(x_iss):
    """Phi(t2, t0) = Phi(t2, t1) Phi(t1, t0) (sin arrastre: dinamica autonoma
    y Hamiltoniana; ademas Phi es simpletica con J2: det = 1)."""
    m = P.Modelo(grado=2, arrastre=False)
    f = P.derivadas_stm(m, False)
    kw = dict(metodo='rk4', h=20.0)
    Y1 = P.propagar(f, P.y0_stm(x_iss), 0.0, [2000.0], **kw)[0]
    Y2 = P.propagar(f, P.y0_stm(x_iss), 0.0, [5000.0], **kw)[0]
    Phi01 = Y1[6:].reshape(6, 6)
    Phi02 = Y2[6:].reshape(6, 6)
    Y12 = P.propagar(f, P.y0_stm(Y1[:6]), 2000.0, [5000.0], **kw)[0]
    Phi12 = Y12[6:].reshape(6, 6)
    assert np.abs(Phi12 @ Phi01 - Phi02).max() < 1e-9 * np.abs(Phi02).max()
    assert np.linalg.det(Phi02) == pytest.approx(1.0, abs=1e-6)


# ------------------------------------------------ efemeride

def test_efemeride_hermite_quintica(x_iss):
    """La interpolacion de Hermite sobre rejilla de 30 s reproduce la
    integracion directa en instantes intermedios a < 1 cm en posicion (el
    error de RK4 h=15 s es del mismo orden) y < 1e-8 km/s en velocidad."""
    m = P.Modelo(grado=2, arrastre=True, cd_a_m=0.0042)
    ef = od.efemeride(m, x_iss, 0.0, 0.0, 3600.0, 30.0)
    tq = np.array([7.3, 100.0, 1234.5, 3001.7])
    r, v = ef.estado(tq)
    Y = P.propagar(P.derivadas(m), x_iss, 0.0, tq, **od.INT_VERDAD)
    assert np.abs(r - Y[:, :3]).max() < 1e-5
    assert np.abs(v - Y[:, 3:6]).max() < 1e-8


# ------------------------------------------------ contra SGP4

def _rtn(r, v, dr):
    rh = r / np.linalg.norm(r, axis=1)[:, None]
    h = np.cross(r, v)
    hh = h / np.linalg.norm(h, axis=1)[:, None]
    th = np.cross(hh, rh)
    return np.stack([np.sum(dr * rh, 1), np.sum(dr * th, 1), np.sum(dr * hh, 1)], 1)


def test_comparacion_con_sgp4_iss_tres_dias(iss, x_iss):
    """Propagador numerico (J4 + arrastre con el BC efectivo calibrado) contra
    SGP4 a lo largo de 3 dias, partiendo del estado SGP4 de la epoca.

    Esperado: ~0.17 km rms en 24 h y UNOS 8 km en 72 h, casi todo A LO LARGO
    DE LA PISTA. Por que: SGP4 propaga elementos MEDIOS (Brouwer) con su propio
    modelo de arrastre (B* del TLE y atmosfera de ley de potencias), y el
    estado que entrega en la epoca es osculante con periodicos de J2; integrar
    ese estado osculante con otro arrastre cambia ligeramente el semieje medio
    (unos metros), y un error de semieje da un error de fase que crece con el
    CUADRADO del tiempo (1.5 n da t). Sin calibrar el BC (0.0105 fisico) el
    error es 4.6 km a 24 h: el arrastre domina."""
    from atp_ros import medidas as M
    reloj = M.Reloj(iss.epoca)
    sg = od.EfemerideSGP4(iss, reloj)
    ts = np.arange(0.0, 3 * 86400.0 + 1, 600.0)
    rs, vs = sg.estado(ts)
    m = P.Modelo(grado=4, cd_a_m=BC_ISS * P_ISS)
    Y = P.propagar(P.derivadas(m), x_iss, 0.0, ts, **od.INT_VERDAD)
    d = Y[:, :3] - rs
    n = np.linalg.norm(d, axis=1)
    assert np.sqrt(np.mean(n[:145] ** 2)) < 0.3          # 24 h
    assert n[432] < 15.0 and n[432] > 2.0                # 72 h: km, no metros
    rtn = _rtn(rs, vs, d)
    assert abs(rtn[432, 1]) > 0.9 * n[432]               # a lo largo de la pista
    # sin calibrar el arrastre, el error a 24 h es 20 veces mayor
    m0 = P.Modelo(grado=4, cd_a_m=BC_ISS)
    Y0 = P.propagar(P.derivadas(m0), x_iss, 0.0, ts[:145], **od.INT_VERDAD)
    assert np.linalg.norm(Y0[144, :3] - rs[144]) > 3.0


def test_sgp4_sin_arrastre_solo_j2_tiene_el_mismo_orden(iss, x_iss):
    """Solo J2 (sin arrastre), 24 h: la diferencia con SGP4 es de ~3 km (la
    parte que NO es arrastre: periodicos y J3/J4); con arrastre calibrado
    baja a < 0.7 km. Confirma que el modelo de fuerzas del propagador captura
    lo esencial de SGP4."""
    from atp_ros import medidas as M
    sg = od.EfemerideSGP4(iss, M.Reloj(iss.epoca))
    ts = np.arange(0.0, 86400.1, 600.0)
    rs, _ = sg.estado(ts)
    m = P.Modelo(grado=2, arrastre=False)
    Y = P.propagar(P.derivadas(m), x_iss, 0.0, ts, **od.INT_VERDAD)
    assert np.linalg.norm(Y[-1, :3] - rs[-1]) < 5.0
    m2 = P.Modelo(grado=2, cd_a_m=BC_ISS * P_ISS)
    Y2 = P.propagar(P.derivadas(m2), x_iss, 0.0, ts, **od.INT_VERDAD)
    assert np.linalg.norm(Y2[-1, :3] - rs[-1]) < 0.7
