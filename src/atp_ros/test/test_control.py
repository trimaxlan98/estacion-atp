"""Control avanzado: Riccati, LQR integral, margenes, Kalman (NEES), planta
realista y su equivalencia EXACTA con la planta ideal por defecto.

Las cifras del curso 30 (leccion 3.1) se fijan aqui como numeros y, si el
repo del curso esta a la vista, se contrastan ademas con atp.py en vivo."""
import math
import os
import re

import numpy as np
import pytest

from atp_ros import control as c
from atp_ros import model as m
from atp_ros import planta as pl

ATP_PY = os.path.expanduser('~/Documentos/code-projects/codeaerospace_contenido/studio/'
                            'content/manim_extensions/atp.py')
DI_A = np.array([[0.0, 1.0], [0.0, 0.0]])
DI_B = np.array([[0.0], [1.0]])


def _atp():
    """Funciones puras de atp.py (sin manim) o None si el curso no esta."""
    if not os.path.exists(ATP_PY):
        return None
    src = open(ATP_PY).read()
    g = {}
    exec(compile(src[:src.index('from manim import')], 'atp', 'exec'), g)
    return g


# ---------------------------------------------------------------- algebra

def test_discretizacion_zoh_forma_cerrada():
    """Eje J=2, b=0.5: Ad = [[1, (1-e^-aT)/a],[0, e^-aT]], a = b/J."""
    A, B = c.matrices_eje(2.0, 0.5)
    Ts, a = 0.05, 0.25
    Ad, Bd = c.discretizar_zoh(A, B, Ts)
    e = math.exp(-a * Ts)
    assert Ad == pytest.approx(np.array([[1.0, (1 - e) / a], [0.0, e]]), abs=1e-14)
    # Bd = int_0^T e^{A s} ds B / J: segunda fila (1-e)/(a J), primera (T - (1-e)/a)/(a J)
    assert Bd[:, 0] == pytest.approx(
        [(Ts - (1 - e) / a) / (a * 2.0), (1 - e) / (a * 2.0)], abs=1e-14)


def test_expm_contra_scipy():
    sl = pytest.importorskip('scipy.linalg')
    rng = np.random.default_rng(1)
    M = rng.normal(size=(5, 5))
    assert c.expm(M) == pytest.approx(sl.expm(M), rel=1e-10, abs=1e-10)


def test_controlabilidad_curso():
    A, B = c.matrices_eje()
    r = c.controlabilidad(A, B)
    assert r['rango'] == 2 and r['det'] == pytest.approx(-0.25)


# ---------------------------------------------------------------- Riccati

@pytest.mark.parametrize('q,r', [(100, 1), (100, 100), (1, 1), (1e4, 0.5)])
def test_riccati_continua_forma_cerrada(q, r):
    """Doble integrador, Q = diag(q,0): k1 = sqrt(q/r), k2 = sqrt(2 k1)."""
    K, P = c.lqr(DI_A, DI_B, np.diag([q, 0.0]), r)
    f = c.lqr_doble_integrador(q, r)
    assert K[0] == pytest.approx([f['k1'], f['k2']], rel=1e-9)
    assert f['zeta'] == pytest.approx(1 / math.sqrt(2))     # para CUALQUIER q, r
    # la solucion cumple la ecuacion de Riccati
    res = DI_A.T @ P + P @ DI_A - P @ DI_B @ DI_B.T @ P / r + np.diag([q, 0.0])
    assert np.max(np.abs(res)) < 1e-8 * max(1.0, np.max(np.abs(P)))


def test_cifras_leccion_3_1():
    """q=100, r=1 -> k1=10, k2=4.47, wn=3.16, t_est=1.75 s; r=100 -> wn=1,
    3.16x mas lento (la raiz cuarta de 100)."""
    f1 = c.lqr_doble_integrador(100, 1)
    f2 = c.lqr_doble_integrador(100, 100)
    assert (f1['k1'], f1['k2']) == pytest.approx((10.0, 4.4721), abs=1e-4)
    assert f1['wn'] == pytest.approx(3.1623, abs=1e-4)
    assert f1['t_est'] == pytest.approx(1.7495, abs=1e-3)
    assert f2['wn'] == pytest.approx(1.0)
    assert f1['wn'] / f2['wn'] == pytest.approx(100 ** 0.25)
    A, B = c.matrices_eje(2.0, 0.5)
    K, _ = c.lqr(A, B, np.diag([100.0, 0.0]), 1.0)
    assert K[0] == pytest.approx([10.0, 5.8443], abs=1e-4)   # eje J=2, b=0.5
    g = _atp()
    if g is not None:     # contraste en vivo con la fuente
        Kg, Pg = g['lqr'](A, B, np.diag([100.0, 0.0]), 1.0)
        assert K == pytest.approx(Kg, abs=1e-9)
        assert g['lqr_doble_integrador'](100, 1)['t_est'] == pytest.approx(f1['t_est'])


def test_dare_escalar_forma_cerrada():
    """x+ = a x + b u: g p^2 + (1 - a^2 - q g) p - q = 0, g = b^2/r."""
    a, b, q, r = 1.1, 0.7, 2.0, 0.5
    g = b * b / r
    k = 1 - a * a - q * g
    p = (-k + math.sqrt(k * k + 4 * g * q)) / (2 * g)
    X = c.dare(np.array([[a]]), np.array([[b]]), np.array([[q]]), np.array([[r]]))
    assert X[0, 0] == pytest.approx(p, rel=1e-12)
    K, _ = c.lqr_discreto(np.array([[a]]), np.array([[b]]), np.array([[q]]), np.array([[r]]))
    assert K[0, 0] == pytest.approx(a * b * p / (r + b * b * p), rel=1e-12)
    assert abs(a - b * K[0, 0]) < 1.0           # estabiliza una planta inestable


def test_dare_contra_scipy_y_residuo():
    A, B = c.matrices_eje(2.0, 0.5)
    Ad, Bd = c.discretizar_zoh(A, B, 0.05)
    Q, R = np.diag([1e3, 1.0]), np.array([[2.0]])
    X = c.dare(Ad, Bd, Q, R)
    G = Bd @ np.linalg.solve(R, Bd.T)
    res = Ad.T @ X @ np.linalg.solve(np.eye(2) + G @ X, Ad) + Q - X
    assert np.max(np.abs(res)) < 1e-9 * np.max(np.abs(X))
    sl = pytest.importorskip('scipy.linalg')
    assert X == pytest.approx(sl.solve_discrete_are(Ad, Bd, Q, R), rel=1e-8)


def test_lqr_discreto_tiende_al_continuo():
    """Con Ts -> 0 y pesos q Ts, r Ts el LQR discreto del doble integrador
    converge al continuo (k1=10, k2=4.47)."""
    Ts = 1e-3
    Ad, Bd = c.discretizar_zoh(DI_A, DI_B, Ts)
    K, _ = c.lqr_discreto(Ad, Bd, np.diag([100.0 * Ts, 0.0]), np.array([[1.0 * Ts]]))
    assert K[0] == pytest.approx([10.0, 4.4721], rel=0.02)


# ---------------------------------------------------------------- lazo cerrado

@pytest.mark.parametrize('q', [dict(), dict(q_theta=1e3), dict(q_theta=1e5, q_int=1e4),
                               dict(q_omega=10.0, r=5.0)])
def test_lqr_integral_lazo_cerrado_estable(q):
    d = c.lqr_integral(2.0, 0.5, 0.05, **q)
    assert np.all(np.abs(d['polos']) < 1.0)
    assert d['K'].shape == (1, 3) and np.all(d['K'] > 0)
    # los polos son los autovalores de Aa - Ba K
    assert np.sort(np.abs(np.linalg.eigvals(d['Aa'] - d['Ba'] @ d['K']))) \
        == pytest.approx(np.sort(np.abs(d['polos'])))


def test_lqr_integral_cero_error_con_par_constante():
    """Accion integral: un par de perturbacion constante (gravedad,
    friccion) se absorbe sin error estacionario (el polo lento del integrador,
    |z| = 0.984, lo lleva a < 1e-6 rad en 100 s)."""
    J, b, Ts = 2.0, 0.5, 0.05
    d = c.lqr_integral(J, b, Ts)
    Ad, Bd, K = d['Ad'], d['Bd'][:, 0], d['K'][0]
    x, z = np.zeros(2), 0.0
    for _ in range(2000):                      # 100 s
        u = -(K[0] * x[0] + K[1] * x[1] + K[2] * z)
        z += Ts * x[0]
        x = Ad @ x + Bd * (u + 0.3)            # par constante de 0.3 N m
    assert abs(x[0]) < 1e-6 and abs(x[1]) < 1e-6


def test_margenes_curso_lqr_continuo():
    """El LQR continuo con R diagonal da MG infinito y MF >= 60 grados: se
    comprueba (65.53 grados en el doble integrador q=100, r=1) y que un
    retardo se come el margen (leccion 3.1): 0.05 s -> 51.45, 0.2 s -> 9.2."""
    K, _ = c.lqr(DI_A, DI_B, np.diag([100.0, 0.0]), 1.0)
    mg = c.margenes(DI_A, DI_B, K)
    assert mg['margen_fase_deg'] == pytest.approx(65.53, abs=0.01)
    assert mg['wc'] == pytest.approx(4.9135, abs=1e-3)
    assert mg['margen_ganancia_db'] == float('inf')
    assert mg['margen_fase_deg'] >= 60.0
    assert c.margen_fase_con_retardo(DI_A, DI_B, K, 0.05) == pytest.approx(51.454, abs=0.01)
    assert c.margen_fase_con_retardo(DI_A, DI_B, K, 0.2) == pytest.approx(9.226, abs=0.01)
    A, B = c.matrices_eje(2.0, 0.5)
    Ke, _ = c.lqr(A, B, np.diag([100.0, 0.0]), 1.0)
    assert c.margenes(A, B, Ke)['margen_fase_deg'] == pytest.approx(66.84, abs=0.01)
    g = _atp()
    if g is not None:
        assert g['margenes'](DI_A, DI_B, K)['margen_fase_deg'] == pytest.approx(
            mg['margen_fase_deg'], abs=0.01)
        assert g['margen_fase_con_retardo'](DI_A, DI_B, K, 0.1) == pytest.approx(
            c.margen_fase_con_retardo(DI_A, DI_B, K, 0.1), abs=0.01)


def test_margenes_discreto_tiende_al_continuo():
    """El lazo digital del LQR del doble integrador, con Ts muy chico, tiene
    el margen de fase del continuo (65.5 grados)."""
    Ts = 1e-3
    Ad, Bd = c.discretizar_zoh(DI_A, DI_B, Ts)
    K, _ = c.lqr_discreto(Ad, Bd, np.diag([100.0 * Ts, 0.0]), np.array([[Ts]]))
    mg = c.margenes_discreto(Ad, Bd, K, Ts)
    assert mg['margen_fase_deg'] == pytest.approx(65.5, abs=2.0)
    assert mg['margen_ganancia_db'] == float('inf')


def test_margenes_lazo_digital_montura():
    """El LQR integral por defecto a 20 Hz: MF > 45 grados, margen de
    retardo mayor que un periodo de muestreo y sensibilidad Ms < 2."""
    d = c.lqr_integral(2.0, 0.5, 0.05)
    mg = c.margenes_discreto(d['Aa'], d['Ba'], d['K'], 0.05)
    assert mg['margen_fase_deg'] > 45.0
    assert mg['margen_retardo_s'] > 0.05
    assert mg['Ms'] < 2.0


def test_diferenciador_referencia():
    """v y a por diferencias hacia atras de 2.o orden: exactas en un
    polinomio de grado 2."""
    Ts = 0.05
    dif = c.DiferenciadorRef(Ts)
    for k in range(10):
        t = k * Ts
        v, a = dif.paso(np.array([3.0 + 2.0 * t + 0.5 * t * t]))
        if k >= 2:
            assert v[0] == pytest.approx(2.0 + t, abs=1e-9)
            assert a[0] == pytest.approx(1.0, abs=1e-9)


def test_lazo_lqr_sigue_rampa_con_preali():
    """Lazo digital completo (planta exacta + Kalman + LQR): sigue una
    referencia con aceleracion; la pre-alimentacion reduce el error."""
    J, b, Ts = 2.0, 0.5, 0.05
    A, B = c.matrices_eje(J, b)
    Ad, Bd = c.discretizar_zoh(A, B, Ts)
    d = c.lqr_integral(J, b, Ts, q_theta=1e4)

    def corre(ff):
        lazo = c.LazoLQR(d, J, b, 8.0, forma=(1,), usar_ff=ff, usar_dhat=True)
        dif = c.DiferenciadorRef(Ts)
        x = np.zeros(2)
        est = None
        errs = []
        for k in range(600):
            t = k * Ts
            r = np.radians(np.array([2.0 * t + 0.4 * t * t]))   # rampa + aceleracion
            if k == 0:
                x = np.array([r[0], np.radians(2.0)])           # engancha
                est = c.crear_estimador(True, J, b, Ts, np.array([x[0]]), 1e-4, 0, (1,))
                est.x[1] = x[1]
                xe = est.x
            else:
                xe = est.paso(np.array([x[0]]), u)
            rv, ra = dif.paso(r)
            u = lazo.calcular(xe, r, rv, ra)
            errs.append(abs(r[0] - x[0]))        # x(t_k) contra r(t_k), antes de integrar
            x = Ad @ x + Bd[:, 0] * u[0]
        return np.degrees(np.max(errs[200:]))

    con, sin = corre(True), corre(False)
    assert con < 0.02            # grados, tras el transitorio
    assert con < sin


# ---------------------------------------------------------------- Kalman

def test_kalman_covarianza_estacionaria_es_dare_dual():
    dis = c.diseno_kalman(2.0, 0.5, 0.05, math.radians(0.02))
    kf = c.FiltroKalman(dis, np.zeros(1), (1,))
    for _ in range(3000):
        kf.paso(np.zeros(1), 0.0)
    # covarianza A PRIORI estacionaria = DARE del sistema dual
    Pp = c.dare(dis['Ad'].T, dis['H'].T, dis['Qd'], np.array([[dis['R']]]))
    P_post = kf.P
    Ppred = dis['Ad'] @ P_post @ dis['Ad'].T + dis['Qd']
    assert Ppred == pytest.approx(Pp, rel=1e-6, abs=1e-14)


def test_kalman_consistente_nees():
    """NEES medio ~ dimension (3): el filtro no es ni optimista ni
    pesimista cuando la verdad cumple el modelo (Bar-Shalom, Li y
    Kirubarajan 2001, cap. 5). 400 corridas independientes."""
    Ts, N, T = 0.05, 400, 80
    dis = c.diseno_kalman(2.0, 0.5, Ts, math.radians(0.02), q_acc=1e-2, q_par=1e-2)
    Ad, Bd, Qd, R = dis['Ad'], dis['Bd'][:, 0], dis['Qd'], dis['R']
    L = np.linalg.cholesky(Qd)
    rng = np.random.default_rng(42)
    x = np.zeros((3, N))
    x[0] = rng.normal(0, 2e-3, N)
    x[1] = rng.normal(0, 0.05, N)
    x[2] = rng.normal(0, 0.1, N)
    kf = c.FiltroKalman(dis, x[0] + rng.normal(0, math.sqrt(R), N), (N,))
    kf.x[1] = 0.0
    kf.x[2] = 0.0
    nees = []
    for k in range(T):
        u = 0.2 * math.sin(0.3 * k * Ts)
        x = Ad @ x + np.outer(Bd, np.full(N, u)) + L @ rng.normal(size=(3, N))
        y = x[0] + rng.normal(0.0, math.sqrt(R), N)
        kf.paso(y, u)
        nees.append(np.mean(kf.nees(x)))
    medio = float(np.mean(nees[10:]))
    assert 3.0 - 0.4 < medio < 3.0 + 0.4
    # y el error de posicion estimado es menor que el de la medida cruda
    assert np.sqrt(np.mean((kf.x[0] - x[0]) ** 2)) < math.sqrt(R)


def test_kalman_estima_par_de_sesgo():
    """Con un par constante desconocido el filtro lo converge (d_hat -> d)."""
    J, b, Ts = 2.0, 0.5, 0.05
    dis = c.diseno_kalman(J, b, Ts, math.radians(0.005), q_acc=1e-2, q_par=1e-2)
    kf = c.FiltroKalman(dis, np.zeros(1), (1,))
    x = np.zeros(2)
    A, B = c.matrices_eje(J, b)
    Ad, Bd = c.discretizar_zoh(A, B, Ts)
    u = 0.0
    for k in range(400):
        x = Ad @ x + Bd[:, 0] * (u + 0.25)
        xe = kf.paso(np.array([x[0]]), u)
        u = -2.0 * (xe[0] - 1.0)           # un lazo cualquiera que excita la planta
    assert xe[2, 0] == pytest.approx(0.25, abs=0.03)


def test_kalman_cuantizacion_aumenta_r():
    d0 = c.diseno_kalman(2.0, 0.5, 0.05, math.radians(0.01), bits=0)
    d14 = c.diseno_kalman(2.0, 0.5, 0.05, math.radians(0.01), bits=14)
    q = 2 * math.pi / 2 ** 14
    assert d14['R'] - d0['R'] == pytest.approx(q * q / 12.0)


# ---------------------------------------------------------------- planta

def _lazo_viejo(n_pasos, seed=3):
    """El integrador original de mount_sim, copiado tal cual."""
    rng = np.random.default_rng(seed)
    J, b, h, w_max = 2.0, 0.5, 0.002, math.inf
    theta, omega = np.array([1.0, 0.5]), np.zeros(2)
    u = np.zeros(2)
    out = []
    for k in range(n_pasos):
        if k % 25 == 0:
            u = np.clip(rng.normal(0, 3, 2), -8.0, 8.0)
        par_ext = 3.0 if 200 <= k < 700 else 0.0
        alpha = (u + par_ext - b * omega) / J
        omega = np.clip(omega + alpha * h, -w_max, w_max)
        theta += omega * h
        out.append((theta.copy(), omega.copy()))
    return out


def test_planta_por_defecto_identica_bit_a_bit():
    """Con TODO apagado el paso es operacion por operacion el de siempre."""
    ref = _lazo_viejo(2000)
    rng = np.random.default_rng(3)
    P = pl.PlantaMontura(2.0, 0.5, math.inf, 0.002)
    P.enganchar(np.array([1.0, 0.5]))
    u = np.zeros(2)
    for k in range(2000):
        if k % 25 == 0:
            u = np.clip(rng.normal(0, 3, 2), -8.0, 8.0)
        P.paso(u, 3.0 if 200 <= k < 700 else 0.0, 0.002)
        assert np.array_equal(P.theta, ref[k][0]) and np.array_equal(P.omega, ref[k][1])
    assert not P.avanzada


def test_campana_pid_identica_a_model_campana_lazo():
    a = m.campana_lazo(n=10, semilla=3, mascara_deg=30.0)
    b = c.campana_lazo_ctrl('pid', n=10, semilla=3, mascara_deg=30.0)
    assert np.array_equal(a['rms'], b['rms']) and np.array_equal(a['el_max'], b['el_max'])


def test_friccion_adherencia_y_arranque():
    """Con par neto por debajo de Fs = Fc + Fe la montura NO se mueve;
    por encima, arranca y desliza con friccion de Coulomb."""
    P = pl.PlantaMontura(2.0, 0.5, coulomb_nm=0.05, stribeck_nm=0.02)
    P.enganchar(np.zeros(2))
    for _ in range(3000):
        P.paso(np.array([0.06, -0.06]), 0.0, 0.002)
    assert np.all(P.omega == 0.0) and np.all(P.theta == 0.0)
    for _ in range(3000):
        P.paso(np.array([0.2, 0.0]), 0.0, 0.002)
    assert P.theta[0] > 0.1 and P.theta[1] == 0.0
    # desliza: aceleracion ~ (u - Fc - b w)/J (Stribeck ya decayo)
    w0 = P.omega[0]
    P.paso(np.array([0.2, 0.0]), 0.0, 0.002)
    assert (P.omega[0] - w0) / 0.002 == pytest.approx((0.2 - 0.05 - 0.5 * w0) / 2.0, rel=0.02)


def test_friccion_stribeck_curva():
    f0 = float(pl.friccion_stribeck(1e-9, 0.05, 0.02, 0.01))
    finf = float(pl.friccion_stribeck(1.0, 0.05, 0.02, 0.01))
    assert f0 == pytest.approx(0.07, abs=1e-6) and finf == pytest.approx(0.05, abs=1e-9)
    assert float(pl.friccion_stribeck(-1.0, 0.05, 0.02, 0.01)) == pytest.approx(-0.05)


def test_backlash_holgura_acotada_y_zona_muerta():
    """La holgura motor-carga nunca excede delta/2; se recorren ambos
    contactos; y al invertir el par el motor cruza la holgura ENTERA antes
    de volver a arrastrar la carga (zona muerta)."""
    delta = math.radians(0.5)
    P = pl.PlantaMontura(2.0, 0.5, backlash_rad=delta, h=0.002)
    P.enganchar(np.zeros(2))
    lados, dmax = set(), 0.0
    for k in range(15000):
        t = k * 0.002
        u = np.array([0.5 * math.sin(2 * math.pi * 0.2 * t), 0.0])
        P.paso(u, 0.0, 0.002)
        dmax = max(dmax, abs(float(P.separacion()[0])))
        lados.add(float(P.lado[0]))
    assert dmax <= delta / 2 + 1e-12
    assert {-1.0, 0.0, 1.0} <= lados
    # zona muerta: carga en contacto +, se invierte el par: separacion pasa
    # de +delta/2 a -delta/2 (recorre delta) con la carga casi quieta
    P = pl.PlantaMontura(2.0, 0.5, backlash_rad=delta, h=0.002)
    P.enganchar(np.zeros(2))
    for _ in range(1000):
        P.paso(np.array([0.3, 0.0]), 0.0, 0.002)
    assert float(P.lado[0]) == 1.0
    d_ini = float(P.separacion()[0])
    th_ini = float(P.theta[0])
    P.paso(np.array([-0.3, 0.0]), 0.0, 0.002)
    for _ in range(3000):
        P.paso(np.array([-0.3, 0.0]), 0.0, 0.002)
        if float(P.lado[0]) == -1.0:
            break
    assert d_ini == pytest.approx(delta / 2, rel=1e-6)
    assert float(P.separacion()[0]) == pytest.approx(-delta / 2, rel=1e-6)


def test_backlash_diminuto_equivale_a_cuerpo_rigido():
    """Con holgura ~0 los dos cuerpos se comportan como el cuerpo unico."""
    sol = []
    for bl in (0.0, 1e-9):
        P = pl.PlantaMontura(2.0, 0.5, backlash_rad=bl, h=0.002)
        P.enganchar(np.zeros(2))
        for k in range(5000):
            t = k * 0.002
            P.paso(np.array([0.4 * math.sin(t), 0.2]), 0.0, 0.002)
        sol.append(P.theta.copy())
    assert sol[0] == pytest.approx(sol[1], abs=2e-5)


def test_desbalance_gravitatorio_solo_elevacion():
    """Con u = m g d cos(el) la elevacion queda en equilibrio; el acimut no
    siente la gravedad."""
    el = math.radians(60.0)
    mgd = 0.4
    P = pl.PlantaMontura(2.0, 0.5, mgd_nm=mgd)
    P.enganchar(np.array([0.3, el]))
    for _ in range(500):
        P.paso(np.array([0.0, mgd * math.cos(el)]), 0.0, 0.002)
    assert P.omega == pytest.approx(np.zeros(2), abs=1e-9)
    P.enganchar(np.array([0.3, el]))
    for _ in range(100):
        P.paso(np.zeros(2), 0.0, 0.002)
    assert P.omega[1] < -1e-3 and P.omega[0] == 0.0


def test_encoder_cuantizado():
    q = 2 * math.pi / 2 ** 14
    assert math.degrees(q) == pytest.approx(0.02197, abs=1e-5)       # = resolucion_encoder(14)
    th = np.random.default_rng(0).uniform(-3, 3, 1000)
    qz = pl.cuantizar(th, 14)
    assert np.max(np.abs(qz - th)) <= q / 2 + 1e-15
    assert np.allclose(qz / q, np.round(qz / q), atol=1e-6)


def test_viento_dryden_estadistica():
    v = pl.VientoTurbulento(0.1, 'dryden', tau_s=5.0, forma=(1,), semilla=3)
    x = np.array([v.par(k * 0.05, 0.05)[0] for k in range(60000)])
    assert np.std(x) == pytest.approx(0.1, rel=0.1)
    ac = np.mean(x[:-100] * x[100:]) / np.var(x)                       # retardo 5 s = tau
    assert ac == pytest.approx(math.exp(-1.0), abs=0.08)


def test_viento_determinista_y_semilla():
    a = [pl.VientoTurbulento(0.1, 'dryden', semilla=5).par(1.0, 0.05) for _ in range(2)]
    assert np.array_equal(a[0], a[1])
    b = pl.VientoTurbulento(0.1, 'dryden', semilla=6).par(1.0, 0.05)
    assert not np.array_equal(a[0], b)
    d1 = pl.VientoTurbulento(0.1, 'davenport', semilla=5).par(10.0, 0.05)
    d2 = pl.VientoTurbulento(0.1, 'davenport', semilla=5).par(10.0, 0.05)
    assert np.array_equal(d1, d2)


def test_viento_davenport_varianza_y_rms():
    """La varianza del espectro de Davenport integra a 6 kappa V10^2 (fuente)
    y la serie sintetizada tiene el RMS pedido."""
    f = np.logspace(-6, 3, 400000)
    s = pl.davenport_psd(f, 10.0, 0.005)
    var = np.trapezoid(s, f) if hasattr(np, 'trapezoid') else np.trapz(s, f)
    assert var == pytest.approx(6 * 0.005 * 100.0, rel=0.02)
    v = pl.VientoTurbulento(0.1, 'davenport', v10_m_s=10.0, forma=(1,), semilla=1)
    x = np.array([v.par(t, 0.5)[0] for t in np.arange(0.0, 3000.0, 0.5)])
    assert np.std(x) == pytest.approx(0.1, rel=0.3)


def test_dryden_psd_integra_a_varianza():
    f = np.logspace(-6, 3, 400000)
    var = (np.trapezoid if hasattr(np, 'trapezoid') else np.trapz)(
        pl.dryden_psd(f, 0.3, 5.0), f)
    assert var == pytest.approx(0.09, rel=0.01)


# ---------------------------------------------------------------- campana

def test_campana_controladores_deterministas_y_finitos():
    kw = dict(n=6, semilla=9, mascara_deg=40.0)
    for ctl in c.CONTROLADORES:
        r1 = c.campana_lazo_ctrl(ctl, **kw)['rms']
        r2 = c.campana_lazo_ctrl(ctl, **kw)['rms']
        assert np.array_equal(r1, r2) and np.all(np.isfinite(r1))


def test_campana_kalman_gana_con_encoder_cuantizado():
    """Con encoder de 14 bits el LQR que deriva la medida pierde frente al
    LQR + Kalman (la cuantizacion q/Ts se amplifica en omega)."""
    kw = dict(n=8, semilla=4, mascara_deg=40.0, encoder_bits=14,
              lqr=dict(q_theta=1e4), sigma_ruido_deg=0.005)
    sin = c.campana_lazo_ctrl('lqr', **kw)['rms']
    con = c.campana_lazo_ctrl('lqr_kf', **kw)['rms']
    assert np.median(con) < np.median(sin)


def test_lanzamiento_defectos_coinciden_con_el_nodo():
    """Los valores por defecto del launch (q_theta...) son los de control.py."""
    ruta = os.path.join(os.path.dirname(__file__), '..', 'launch', 'atp_loop.launch.py')
    txt = open(ruta).read()
    for arg, clave in (('q_theta', 'q_theta'), ('q_omega', 'q_omega'),
                       ('q_int', 'q_int'), ('r_u', 'r')):
        val = float(re.search(r"\('%s', '([^']*)'\)" % arg, txt).group(1))
        assert val == pytest.approx(c.LQR_DEFECTO[clave])
    for arg, clave in (('kf_q_acc', 'q_acc'), ('kf_q_par', 'q_par')):
        val = float(re.search(r"\('%s', '([^']*)'\)" % arg, txt).group(1))
        assert val == pytest.approx(c.KALMAN_DEFECTO[clave])
