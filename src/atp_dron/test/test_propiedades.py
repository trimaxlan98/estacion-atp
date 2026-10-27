"""Propiedades fisicas y matematicas del dron (P7): invariantes que un cambio de signo, de exponente o de orden rompe.

Cada prueba compara contra una cantidad conservada, una razon analitica o una simetria, no contra un numero copiado de una corrida.
Las tolerancias son holgadas respecto de lo medido (se indica en cada una) pero mucho menores que el efecto de un mutante.
"""
import numpy as np
import pytest

from atp_dron.dron import DT, G0, Dron, euler, q_de_euler, rot
from atp_dron.estimador import ESKF
from atp_dron.lazo import correr
from atp_dron.sensores import SENSORES

INERCIA_ASIM = [0.01, 0.02, 0.035]          # kg m2: tres momentos distintos (sin simetria, el movimiento libre es no trivial)


def _libre(dt, T, w0=(3.0, 0.4, 0.8), v0=(1.0, 2.0, 0.5)):
    d = Dron('luna', {'inercia': INERCIA_ASIM}, pos=(0, 0, 1e5))     # Luna: sin atmosfera, sin arrastre
    d.w, d.v = np.array(w0, float), np.array(v0, float)
    for _ in range(int(round(T / dt))):
        d.paso(np.zeros(4), dt)
    return d


def test_energia_y_momento_angular_se_conservan_sin_par_en_la_luna():
    # medido a 100 s: dE/E ~ 1e-13 y dL/L ~ 7e-11; aqui 30 s y tolerancia 1e-8
    d = Dron('luna', {'inercia': INERCIA_ASIM}, pos=(0, 0, 1e5))
    d.w, d.v = np.array([3.0, 0.4, 0.8]), np.array([1.0, 2.0, 0.5])
    E = lambda: 0.5 * d.w @ d.I @ d.w + 0.5 * d.m * d.v @ d.v + d.m * d.g * d.p[2]
    L = lambda: rot(d.q) @ (d.I @ d.w)                       # momento angular en el mundo
    E0, L0 = E(), L()
    for _ in range(int(round(30.0 / DT))):
        d.paso(np.zeros(4))
    assert abs(E() - E0) / E0 < 1e-8
    assert np.linalg.norm(L() - L0) / np.linalg.norm(L0) < 1e-8
    assert d.w @ d.I @ d.w != pytest.approx(E0 * 0)           # el cuerpo giro de verdad (no es un caso trivial)


def test_rk4_es_de_cuarto_orden():
    ref = np.r_[_libre(0.001, 2.0).w, _libre(0.001, 2.0).q]
    err = lambda dt: np.linalg.norm(np.r_[_libre(dt, 2.0).w, _libre(dt, 2.0).q] - ref)
    razon = err(0.1) / err(0.05)
    assert 12.0 < razon < 20.0                               # medido 16.2 (2^4); un Euler daria ~2 y un RK2 ~4


def test_invariancia_por_rotacion_de_90_grados_en_el_mundo():
    c = {'ruta': [[12, 5, 6], [12, 15, 6]], 'duracion_s': 14.0, 'viento': [3.0, -1.0, 0.0], 'parar_al_llegar': False,
         'hz_telemetria': 50.0}
    rz = lambda v: [-v[1], v[0], v[2]]                          # giro de +90 grados alrededor de z
    _, a = correr(c)
    _, b = correr({**c, 'ruta': [rz(m) for m in c['ruta']], 'viento': rz(c['viento']), 'yaw_deg': 90.0})
    pa = np.array([[r['x'], r['y'], r['z']] for r in a])
    pb = np.array([[r['x'], r['y'], r['z']] for r in b])
    assert np.max(np.abs(pb - np.array([rz(p) for p in pa]))) < 1e-5     # medido ~1e-12 (error de redondeo)
    assert np.max(np.abs(pb - pa)) > 1.0                                  # y no es que las trayectorias sean la misma


def test_la_norma_del_cuaternion_se_mantiene_en_un_vuelo_agresivo():
    d = Dron('tierra', pos=(0, 0, 20.0))
    from atp_dron.control import Cascada
    ctl = Cascada(d)
    d.w = np.array([2.0, -1.0, 1.5])
    peor = 0.0
    for k in range(int(round(10.0 / DT))):
        v = [4.0 * np.sign(np.sin(0.8 * k * DT)), 3.0 * np.cos(k * DT), 0.0]
        d.paso(ctl.mando(v))
        peor = max(peor, abs(np.linalg.norm(d.q) - 1.0))
    assert peor < 1e-12


def test_ornstein_uhlenbeck_tiene_la_varianza_y_la_autocorrelacion_pedidas():
    sigma, tau, dt = 2.0, 2.0, 0.25
    d = Dron('luna', pos=(0, 0, 1e7), rafaga_sigma=sigma, rafaga_tau_s=tau, semilla=1)
    x = []
    for _ in range(20000):
        d.paso(np.zeros(4), dt)
        x.append(d.rafaga.copy())
    x = np.array(x)
    assert x.std(axis=0) == pytest.approx([sigma] * 3, rel=0.05)                    # medido 1.98-1.99
    for lag, k in ((1, 1), (8, 8)):
        rho = [np.corrcoef(x[:-k, i], x[k:, i])[0, 1] for i in range(3)]
        assert rho == pytest.approx([np.exp(-lag * dt / tau)] * 3, abs=0.03)      # exp(-lag dt/tau): 0.882 y 0.368


def test_potencia_inducida_escala_con_empuje_a_la_1_5_y_con_la_densidad_a_la_menos_media():
    def potencia(cuerpo, T):
        d = Dron(cuerpo, {'p_avionica_w': 0.0})
        d.empuje = np.full(4, float(T))
        d._energia(1.0)
        return d.energia_j
    assert potencia('tierra', 8.0) / potencia('tierra', 4.0) == pytest.approx(2.0 ** 1.5, rel=1e-9)
    assert potencia('titan', 4.0) / potencia('tierra', 4.0) == pytest.approx((1.225 / 5.4) ** 0.5, rel=1e-9)


def test_apoyado_en_el_suelo_no_desliza_ni_gira_y_se_endereza():
    d = Dron('tierra', pos=(0, 0, 0.0), viento=(8.0, 0.0, 0.0))
    d.q = q_de_euler(0.2, -0.1, 0.5)
    d.v, d.w = np.array([1.0, 2.0, 0.0]), np.array([1.0, 1.0, 1.0])
    for _ in range(50):
        d.paso(np.zeros(4))
    assert np.allclose(d.v, 0.0) and np.allclose(d.w, 0.0) and d.p[2] == 0.0
    r, p, y = euler(d.q)
    assert abs(r) < 1e-12 and abs(p) < 1e-12 and y == pytest.approx(0.5, abs=0.02)   # conserva el rumbo (en el primer paso aun gira un poco)


def test_el_barometro_ancla_la_altura_sin_gps():
    c = {'ruta': [[0, 0, 10]], 'duracion_s': 40.0, 'parar_al_llegar': False, 'sensores': True, 'semilla': 2,
         'sensores_params': {'gps_corte_s': 0.0}}
    _, f = correr(c)                                          # el GPS no entrega nada: solo IMU, barometro y magnetometro
    ez = np.array([r['ez'] - r['z'] for r in f if 'ez' in r and r['t'] > 5.0])
    assert np.sqrt(np.mean(ez ** 2)) < 0.6                    # ancla: orden de baro_sigma (0.3 m)
    c['sensores_params'] = {'gps_corte_s': 0.0, 'baro_hz': 1e-6}          # control negativo: casi sin barometro
    _, g = correr(c)
    ez2 = np.array([r['ez'] - r['z'] for r in g if 'ez' in r and r['t'] > 5.0])
    assert np.sqrt(np.mean(ez2 ** 2)) > 5 * np.sqrt(np.mean(ez ** 2))


def test_h_del_magnetometro_exacto_coincide_con_las_diferencias_finitas_y_la_actual_falla_con_cabeceo():
    from atp_dron.estimador import _qmul
    q = q_de_euler(np.radians(20), np.radians(35), np.radians(40))

    def H_de(flag):
        e = ESKF(G0, [0, 0, 0], q, {**SENSORES, 'h_mag_exacto': flag})
        cap = []
        e._corregir = lambda y, H, Rm: cap.append(H.copy())
        e.mag(euler(q)[2])
        return cap[0][0, 6:9]
    fd = np.zeros(3)
    for i in range(3):
        dth = np.zeros(3)
        dth[i] = 1e-6
        qp = _qmul(q, np.r_[1.0, 0.5 * dth])
        qp /= np.linalg.norm(qp)
        fd[i] = (euler(qp)[2] - euler(q)[2]) / 1e-6
    assert np.max(np.abs(H_de(True) - fd)) < 1e-4
    assert np.max(np.abs(H_de(False) - fd)) > 0.3             # el H original: error = sin(cabeceo) = 0.57 en el eje x


def test_q_corregida_es_dt_veces_menor_en_v_y_theta_y_igual_en_los_sesgos():
    a = ESKF(G0, [0, 0, 0], [1, 0, 0, 0], {**SENSORES})
    b = ESKF(G0, [0, 0, 0], [1, 0, 0, 0], {**SENSORES, 'q_corregida': True})
    assert a.Q[3, 3] / b.Q[3, 3] == pytest.approx(1.0 / DT, rel=1e-12)
    assert a.Q[6, 6] / b.Q[6, 6] == pytest.approx(1.0 / DT, rel=1e-12)
    assert np.array_equal(a.Q[9:, 9:], b.Q[9:, 9:])


def test_gps_gauss_markov_tiene_la_varianza_y_la_correlacion_pedidas_y_la_latencia_retrasa():
    from atp_dron.sensores import Sensores
    d = Dron('tierra', pos=(0, 0, 50.0))
    tau, lat = 10.0, 0.5
    s = Sensores(d, {'gps_gm_tau_s': tau, 'gps_pos_h': 0.8}, 0)
    e = []
    for _ in range(int(round(2000.0 / DT))):
        o = s.otros()
        if 'gps' in o:
            e.append(o['gps'][0] - d.p)
        d.t += DT
    e = np.array(e)
    assert e[:, 0].std() == pytest.approx(0.8, rel=0.25)
    paso = 1.0 / SENSORES['gps_hz']
    rho = np.corrcoef(e[:-1, 0], e[1:, 0])[0, 1]
    assert rho == pytest.approx(np.exp(-paso / tau), abs=0.03)
    # latencia: la medida entregada en t es la posicion de t - latencia
    d2 = Dron('tierra', pos=(0, 0, 50.0))
    s2 = Sensores(d2, {'gps_latencia_s': lat, 'gps_pos_h': 0.0, 'gps_pos_v': 0.0, 'gps_vel': 0.0}, 0)
    d2.v = np.array([10.0, 0.0, 0.0])
    llegadas = []
    for _ in range(int(round(3.0 / DT))):
        o = s2.otros()
        if 'gps' in o:
            llegadas.append((d2.t, o['gps'][0][0]))
        d2.p = d2.p + d2.v * DT
        d2.t += DT
    t_ll, x_ll = llegadas[-1]
    assert x_ll == pytest.approx(10.0 * (t_ll - lat), abs=10.0 * 2 * DT)


def test_efecto_suelo_en_la_dinamica_sigue_cheeseman_bennett_exacto():
    # la aceleracion vertical inicial con rotores girando a hover sale de T/(1-(R/4z)^2); un exponente o un factor mal puesto la mueve
    z = 0.4
    d = Dron('tierra', {'efecto_suelo': True}, pos=(0, 0, z))
    d.om[:] = d.empuje_a_om(np.full(4, d.m * d.g / 4))
    s = np.concatenate([d.p, d.v, d.q, d.w, d.om])
    az = d._deriv(s, d.om, d.viento)[5]
    r = d.P['radio_rotor_m']
    esperado = (d.m * d.g / (1.0 - (r / (4.0 * z)) ** 2)) / d.m - d.g
    assert az == pytest.approx(esperado, rel=1e-9) and az > 0.05


def test_el_acelerometro_mide_la_fuerza_especifica_mas_el_sesgo_sin_ruido():
    from atp_dron.sensores import Sensores
    d = Dron('tierra', pos=(0, 0, 50.0))
    d.om[:] = d.empuje_a_om(np.full(4, d.m * d.g / 4))
    s = Sensores(d, {'acel_ruido': 0.0, 'acel_sesgo_rw': 0.0, 'gyro_ruido': 0.0, 'gyro_sesgo_rw': 0.0}, 3)
    g, a = s.imu()
    assert a == pytest.approx(d.fuerza_especifica() + s.sesgo_a, abs=1e-12) and np.linalg.norm(s.sesgo_a) > 0
    assert g == pytest.approx(d.w + s.sesgo_g, abs=1e-12)


def test_el_mando_proyecta_el_empuje_en_el_eje_real_del_cuerpo():
    from atp_dron.control import Cascada
    d = Dron('tierra', pos=(0, 0, 20.0))
    d.q = q_de_euler(0.7, 0.0, 0.0)                           # alabeo de 40 grados
    ctl = Cascada(d)
    T = ctl.mando(np.zeros(3))
    zb = ctl.z_b_des
    assert T.sum() == pytest.approx(ctl.T_des * max(0.0, float(zb @ rot(d.q)[:, 2])), rel=1e-9)
    assert T.sum() < 0.8 * ctl.T_des


def test_el_controlador_a_bordo_usa_la_tasa_del_giroscopio_sin_su_sesgo_estimado():
    import json

    from atp_dron.lockstep import Guiado, PlantaLock, cargar_cfg
    cfg = cargar_cfg(None, {'ruta': [[0, 0, 5]], 'duracion_s': 15.0, 'parar_al_llegar': False, 'sensores': True, 'semilla': 1,
                            'sensores_params': {'gyro_sesgo0': 0.1}})
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    while not pl.fin:
        pl.aplicar(json.loads(json.dumps(gu.decidir(json.loads(pl.estado_txt())))))
    assert np.linalg.norm(pl.sens.sesgo_g) > 0.05            # el sesgo es grande a proposito
    assert np.linalg.norm(pl.vista.w - pl.d.w) < 0.03         # y la tasa que ve el control esta corregida
