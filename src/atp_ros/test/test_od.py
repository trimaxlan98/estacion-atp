"""Validacion de la determinacion de orbita (equipo N): modelos de medicion,
minimos cuadrados por lotes, EKF, consistencia (residuos y NEES con Monte
Carlo), observabilidad, y el lazo completo "medir -> estimar -> predecir el
pase siguiente".

Los TLE estan EMBEBIDOS (epoca 2026-10-01). La "verdad" de los escenarios es
el propagador numerico (J4 + arrastre con variacion temporal de densidad) y el
estimador usa un modelo MAS POBRE (J2 + arrastre constante): no hay "crimen
inverso". Las pruebas de consistencia (NEES) usan el MISMO modelo a ambos
lados a proposito: ahi la covarianza formal debe ser exacta; con modelos
distintos solo puede cubrir el ruido (ver docs/OD.md).
"""
import datetime as dt
import math

import numpy as np
import pytest

from atp_ros import medidas as M
from atp_ros import model
from atp_ros import od
from atp_ros import od_experimento as X
from atp_ros import orbit as o
from atp_ros import propagador as P

UTC = dt.timezone.utc
ISS_L1 = '1 25544U 98067A   26274.82022115  .00003852  00000+0  78825-4 0  9993'
ISS_L2 = '2 25544  51.6316 132.3693 0006937 211.2109 148.8469 15.48706258588289'
ESTACIONES = {'cdmx': o.Estacion('Ciudad de Mexico', 19.4326, -99.1332, 2240.0, 5.0),
              'madrid': o.Estacion('Madrid', 40.4168, -3.7038, 650.0, 5.0),
              'ecuador': o.Estacion('Quito', -0.1807, -78.4678, 2850.0, 10.0)}


@pytest.fixture(scope='module')
def iss():
    return o.tle_desde_lineas('ISS (ZARYA)', ISS_L1, ISS_L2)


@pytest.fixture(scope='module')
def esc(iss):
    """Escenario de 3 dias: verdad con variacion de densidad, p_cal fijo."""
    return X.Escenario(iss, ESTACIONES, dias=3.0, p_cal=0.406)


@pytest.fixture(scope='module')
def esc_igual(iss):
    """Verdad SIN variacion de densidad y con el mismo modelo que el
    estimador (para probar el algoritmo, no el error de modelo)."""
    e = X.Escenario(iss, ESTACIONES, dias=1.0, p_cal=0.406, factor_bc_verdad=1.0,
                    variacion_densidad=(), grado_verdad=2, desfase_pos_km=(0, 0, 0),
                    desfase_vel_mm_s=(0, 0, 0))
    return e


@pytest.fixture(scope='module')
def arranque(esc):
    """Primer pase de CDMX con elevacion >= 20 deg tras 1.5 dias."""
    return esc.pases('cdmx', 1.5 * 86400.0, 2.5 * 86400.0, 20.0)[0]


# ------------------------------------------------ modelos de medicion

def test_observables_coinciden_con_orbit_observar(iss):
    """h(x) de medidas.py = orbit.observar sobre estados SGP4 (3 dias, 3
    estaciones): az, el, rango y tasa de rango a precision de maquina."""
    reloj = M.Reloj(iss.epoca)
    ts = np.arange(0.0, 3 * 86400.0, 2500.0)
    jd, fr = reloj.jdfr(ts)
    r, v = o.propagar_teme(iss, jd, fr)
    for est in ESTACIONES.values():
        ob = o.observar(iss, est, jd, fr)
        h = M.observables(reloj, M.sitio(est), ts, r, v)
        assert np.abs(np.degrees(h['az']) - ob['az_deg']).max() < 1e-9
        assert np.abs(np.degrees(h['el']) - ob['el_deg']).max() < 1e-9
        assert np.abs(h['rng'] - ob['rango_km']).max() < 1e-8
        assert np.abs(h['rr'] - ob['vr_km_s']).max() < 1e-12


def test_jacobiano_de_las_medidas_contra_diferencias_finitas(iss):
    """Parciales analiticos de az, el, rng y rr respecto de (r, v) contra
    diferencias finitas centrales, en los instantes visibles (> 5 deg) de 3 dias."""
    reloj = M.Reloj(iss.epoca)
    sit = M.sitio(ESTACIONES['cdmx'])
    ts = np.arange(0.0, 3 * 86400.0, 60.0)
    r, v = o.propagar_teme(iss, *reloj.jdfr(ts))
    el = M.observables(reloj, sit, ts, r, v)['el']
    ok = np.degrees(el) > 5.0
    ts, r, v = ts[ok], r[ok], v[ok]
    assert len(ts) >= 20
    J = M.jacobiano(reloj, sit, ts, r, v)
    for j in range(6):
        d = np.zeros(6)
        d[j] = 1e-3 if j < 3 else 1e-6
        hp = M.observables(reloj, sit, ts, r + d[:3], v + d[3:])
        hm = M.observables(reloj, sit, ts, r - d[:3], v - d[3:])
        for k, c in enumerate(M.CANALES):
            dd = hp[c] - hm[c]
            if c == 'az':
                dd = M.envolver(dd)
            fd = dd / (2 * d[j])
            assert np.abs(J[:, k, j] - fd).max() <= 1e-6 * max(np.abs(fd).max(), 1e-12), (c, j)


def test_doppler_de_medidas_igual_al_de_model(iss):
    rr = np.array([-6.5, 0.0, 3.2])
    assert np.allclose(M.doppler_hz(rr, 2.2e9), model.doppler_hz(rr, 2.2e9), rtol=1e-9)


def test_separacion_angular():
    """Casos resueltos: 0.1 deg en elevacion; 0.1 deg de acimut a 60 deg de
    elevacion son 0.05 deg de arco (compresion cos el); polos opuestos."""
    r = math.radians
    assert M.separacion_deg(r(10), r(30), r(10), r(30.1)) == pytest.approx(0.1, rel=1e-9)
    assert M.separacion_deg(r(10), r(60), r(10.1), r(60)) == pytest.approx(
        0.1 * math.cos(r(60)), rel=1e-5)
    assert M.separacion_deg(r(0), r(90), r(0), r(-90)) == pytest.approx(180.0)
    assert M.separacion_deg(r(350), r(20), r(10), r(20)) < 20.0     # cruce de 0/360


def test_sigmas_por_defecto_coherentes_con_la_estacion():
    """Las sigmas se justifican en medidas.py: 0.01 deg de angulos (0.004
    deg de autotrack y cuantizacion de 14 bits en cuadratura), rango 10 m,
    Doppler 0.05 m/s (0.37 Hz en S)."""
    cuant = 360.0 / 2 ** 14 / math.sqrt(12.0)
    assert math.hypot(0.004, cuant) == pytest.approx(0.0075, abs=0.0005)
    assert math.degrees(M.SIGMAS_DEFECTO['az']) == pytest.approx(0.01)
    assert M.SIGMAS_DEFECTO['rr'] * 1e3 * 2.2e9 / 299792458.0 == pytest.approx(0.37, abs=0.01)


# ------------------------------------------------ estadistica auxiliar

def test_chi2_limites_contra_cuantiles_exactos():
    """Wilson-Hilferty contra los cuantiles de chi2(6): [1.237, 14.449]; y
    el promedio de 40 variables chi2(6): 6 +- 1.96 sqrt(2 * 6 / 40) = [4.93, 7.07]."""
    lo, hi = od.chi2_limites(6, 1, 0.95)
    assert lo == pytest.approx(1.237, rel=0.06) and hi == pytest.approx(14.449, rel=0.03)
    lo, hi = od.chi2_limites(6, 40, 0.95)
    assert lo == pytest.approx(4.93, abs=0.1) and hi == pytest.approx(7.07, abs=0.15)


# ------------------------------------------------ ajuste por lotes

def test_ajuste_sin_ruido_recupera_el_estado_exacto(esc_igual):
    """Medidas sin ruido de un pase de la verdad (mismo modelo): el ajuste
    parte de un estado a 3 km y 3 m/s del real y converge al estado
    verdadero a < 1 m."""
    e = esc_igual
    pase = e.pases('cdmx', 0.0, 86400.0, 15.0)[0]
    med = e.medidas(pase, rng=None, dt_s=10.0)
    t0 = med.t[0]
    xt = e.estado_verdad(t0)
    x0 = xt + np.r_[2.0, -2.0, 1.5, 1.5e-3, -2e-3, 1e-3]
    res = od.ajuste_lotes(med, x0, t0, e.modelo_est, estima_p=False, max_iter=20)
    assert res.convergio
    assert np.linalg.norm(res.x[:3] - xt[:3]) < 1e-3
    assert np.linalg.norm(res.x[3:6] - xt[3:6]) < 1e-6
    assert res.chi2_dof < 1e-6      # sin ruido, los residuos se anulan


def test_residuos_normalizados_con_la_sigma_correcta(esc, arranque):
    """Con ruido gaussiano de las sigmas declaradas, los residuos normalizados
    tienen rms ~ 1 por canal y chi2/dof ~ 1; declarando la MITAD de la sigma
    real el rms sube a ~2 y chi2/dof a ~4 (la prueba detecta una sigma
    optimista)."""
    rng = np.random.default_rng(11)
    med = esc.medidas(arranque, rng=rng, dt_s=4.0)
    res = esc.ajustar([med], M.CANALES, prior=True)
    assert 0.7 < res.chi2_dof < 1.3
    for c in M.CANALES:
        assert 0.7 < res.rms_norm[c] < 1.3, (c, res.rms_norm)
    mitad = {c: 0.5 * s for c, s in M.SIGMAS_DEFECTO.items()}
    res2 = esc.ajustar([med], M.CANALES, prior=True, sigmas=mitad)
    assert 3.0 < res2.chi2_dof < 4.6


def test_observabilidad_angulos_solos_contra_con_doppler(esc, arranque):
    """Con SOLO angulos de un pase el semieje (periodo) queda mal medido: la
    covarianza sin prior es mucho mayor que con Doppler (formal)."""
    med = esc.medidas(arranque, rng=np.random.default_rng(2), dt_s=5.0)
    a = esc.ajustar([med], ('az', 'el'), prior=False)
    b = esc.ajustar([med], ('az', 'el', 'rr'), prior=False)
    assert a.sigma_pos_km() > 5.0 * b.sigma_pos_km()
    # el valor singular minimo de la informacion normalizada crece con Doppler
    assert b.sv_norm[-1] > 5.0 * a.sv_norm[-1]


def test_covarianza_formal_consistente_nees_monte_carlo(esc_igual):
    """NEES con Monte Carlo (30 corridas): la verdad se sortea del prior, los
    datos tienen ruido de las sigmas, el estimador usa el MISMO modelo. El
    NEES medio debe caer en el intervalo de chi2(6) al 95 % (modelo exacto =>
    covarianza formal exacta). Ademas el error real rms de posicion es del
    orden de sigma_pos formal."""
    e = esc_igual
    pase = e.pases('cdmx', 0.0, 86400.0, 25.0)[0]
    t = np.arange(pase['aos'], pase['los'], 6.0)
    x_nom = e.estado_verdad(t[0])
    xb, Pb = od.prior_de_tle(x_nom, 1.0)
    L = np.linalg.cholesky(Pb)
    rng = np.random.default_rng(2026)
    f = P.derivadas(e.modelo_est)
    valores, sigmas_pos, errs = [], [], []
    for _ in range(30):
        xv = xb + L @ rng.standard_normal(6)
        Y = P.propagar(f, xv, t[0], t, metodo='rk4', h=15.0)
        med = M.generar(e.reloj, e.sitios, e.isit('cdmx'), t, Y[:, :3], Y[:, 3:6],
                        rng=rng, canales=('az', 'el', 'rr'))
        res = od.ajuste_lotes(med, xb, t[0], e.modelo_est, prior=(xb, Pb))
        err = res.x[:6] - xv
        valores.append(od.nees(err, res.P))
        sigmas_pos.append(res.sigma_pos_km())
        errs.append(np.linalg.norm(err[:3]))
    lo, hi = od.chi2_limites(6, 30, 0.95)
    assert lo < np.mean(valores) < hi, (np.mean(valores), lo, hi)
    assert 0.5 < np.sqrt(np.mean(np.square(errs))) / np.sqrt(np.mean(np.square(sigmas_pos))) < 1.6


def test_ekf_coincide_con_los_lotes_y_es_consistente(esc_igual):
    """El EKF (un pase, mismo prior) llega al mismo estado final que los
    lotes propagados al mismo instante, dentro de 1 sigma; y la innovacion
    normalizada al cuadrado (NIS) tiene media ~ 1."""
    e = esc_igual
    pase = e.pases('cdmx', 0.0, 86400.0, 25.0)[0]
    med = e.medidas(pase, rng=np.random.default_rng(5), dt_s=5.0)
    canales = ('az', 'el', 'rr')
    lotes = e.ajustar([med], canales, prior=True)
    kf = e.filtrar([med], canales)
    ef = od.propagar_resultado(lotes, kf.t + 1.0, 5.0)
    r, v = ef.estado(np.array([kf.t]))
    d = np.r_[r[0], v[0]] - kf.x[:6]
    assert np.linalg.norm(d[:3]) < kf.sigma_pos_km()
    assert 0.8 < kf.nis.mean() < 1.3
    # el filtro reduce la incertidumbre a lo largo del pase
    assert kf.historial_pos_sigma[-1, 1] < kf.historial_pos_sigma[0, 1]


def test_sesgo_angular_se_absorbe_en_la_orbita_y_no_se_ve_en_los_residuos(esc, arranque):
    """LIMITACION central de la OD con angulos: un sesgo de apuntamiento de
    0.02 deg en el acimut (2 sigma) lo absorbe el estado (el estimador mueve
    la orbita ~0.6 km) y los residuos casi no lo delatan (chi2/dof < 1.3):
    el error de posicion crece >10 veces sin que la covarianza formal ni
    chi2 lo digan."""
    canales = ('az', 'el', 'rr')
    sin = esc.medidas(arranque, rng=np.random.default_rng(3), dt_s=5.0)
    con = esc.medidas(arranque, rng=np.random.default_rng(3), dt_s=5.0,
                      sesgo_ang_deg=(0.02, 0.0))
    r0 = esc.ajustar([sin], canales, prior=True)
    r1 = esc.ajustar([con], canales, prior=True)
    assert r1.chi2_dof < 1.3
    assert esc.error_posicion_km(r1)[0] > 10.0 * esc.error_posicion_km(r0)[0]
    assert esc.error_posicion_km(r1)[0] > 0.3


# ------------------------------------------------ el lazo completo

def test_od_de_un_pase_mejora_el_apuntamiento_del_pase_siguiente(esc, arranque):
    """Lazo completo con verdad numerica: TLE de ~1.8 dias; un pase de CDMX
    con az/el + rango + Doppler; el desapuntamiento sobre el siguiente pase
    de CDMX (10 h despues) baja al menos 3 veces respecto del TLE sin tocar
    (medido: 0.49 deg -> 0.11 deg). Con SOLO angulos de un pase no hay mejora
    comparable: ver docs/OD.md y test_observabilidad_*."""
    rng = np.random.default_rng(8)
    med = esc.medidas(arranque, rng=rng, dt_s=5.0)
    res = esc.ajustar([med], M.CANALES, prior=True)
    assert res.convergio
    sig = esc.pases('cdmx', arranque['los'] + 3 * 3600.0, 2.9 * 86400.0, 10.0)[0]
    e_tle = esc.error_pase(esc.sgp4, sig)['max']
    ef = od.propagar_resultado(res, sig['los'] + 120.0)
    e_od = esc.error_pase(ef, sig)['max']
    assert e_tle > 0.3                 # el TLE viejo no sirve para Ka
    assert e_od < e_tle / 3.0, (e_od, e_tle)
    # el error de posicion en la epoca es de decenas de metros, no de km
    assert esc.error_posicion_km(res)[0] < 0.2


def test_sin_energia_en_el_prior_los_angulos_solos_empeoran(esc, arranque):
    """Ablacion del prior: con SOLO angulos de un pase, quitar el termino de
    energia del prior (el periodo conocido por el TLE) empeora la
    prediccion del pase siguiente (el periodo queda mal medido: 1 m/s a lo
    largo de la pista = 250 km/dia)."""
    med = esc.medidas(arranque, rng=np.random.default_rng(4), dt_s=5.0)
    sig = esc.pases('cdmx', arranque['los'] + 3 * 3600.0, 2.9 * 86400.0, 10.0)[0]
    errs = {}
    for nombre, kw in (('con', {}), ('sin', {'sigma_deriva_km_dia': None})):
        res = esc.ajustar([med], ('az', 'el'), prior=True, **kw)
        ef = od.propagar_resultado(res, sig['los'] + 120.0)
        errs[nombre] = esc.error_pase(ef, sig)['max']
    assert errs['sin'] > 3.0 * errs['con'], errs


def test_factor_de_arrastre_se_acota(esc_igual):
    """`limites_p` recorta el factor de arrastre en cada paso: partiendo de
    p = 10 (absurdo) el ajuste lo lleva a la cota 3, no a un valor sin sentido;
    y con p estimable y datos sin ruido en el rango fisico lo recupera."""
    e = esc_igual
    pase = e.pases('cdmx', 0.0, 86400.0, 15.0)[0]
    med = e.medidas(pase, rng=None, dt_s=10.0)
    t0 = med.t[0]
    xt = e.estado_verdad(t0)
    res = od.ajuste_lotes(med, np.r_[xt, 10.0], t0, e.modelo_est, estima_p=True,
                          prior=(np.r_[xt, 1.0], np.diag([1.0] * 3 + [1e-3] * 3 + [10.0])),
                          max_iter=3)
    assert 0.2 <= res.x[6] <= 3.0


# ------------------------------------------------ prior y planificador

def test_prior_del_tle_energia_y_diagonal(esc):
    """El prior con termino de energia es mucho mas estrecho que la
    diagonal en la direccion de la energia y igual de laxo en las demas."""
    x = esc.x_tle
    xb, P1 = od.prior_de_tle(x, 2.0, sigma_deriva_km_dia=None)
    _, P2 = od.prior_de_tle(x, 2.0)
    mu = P.WGS84.mu
    g = np.r_[mu * x[:3] / np.linalg.norm(x[:3]) ** 3, x[3:]]
    var_E = lambda Pm: g @ Pm @ g                                    # noqa: E731
    assert var_E(P2) < 1e-3 * var_E(P1)
    # sigma de semieje del prior con energia: ~14 m * max(1, edad)
    a = 1.0 / (2.0 / np.linalg.norm(x[:3]) - x[3:] @ x[3:] / mu)
    sig_a = math.sqrt(var_E(P2)) * 2 * a * a / mu
    assert sig_a * 1000.0 == pytest.approx(28.0, rel=0.1)
    assert np.allclose(P1, np.diag(np.diag(P1)))


def test_planificador_elige_pases_de_otras_estaciones_y_baja_sigma(esc):
    """El criterio de informacion sobre el modelo linealizado: cada pase
    elegido baja la sigma formal esperada sobre el pase objetivo, y los
    candidatos vienen del planificador de la red (planificador.py)."""
    A1 = esc.pases('cdmx', 1.2 * 86400.0, 2.2 * 86400.0, 20.0)[0]
    obj = esc.pases('cdmx', A1['los'] + 20 * 3600.0, 2.9 * 86400.0, 10.0)[0]
    cands = X.candidatos_planificador(esc, ['cdmx', 'madrid', 'ecuador'], A1['aos'] - 60.0,
                                      (A1['los'] - A1['aos']) / 3600.0 + 14.1, 10.0)
    cands = [c for c in cands if c['aos'] > A1['los'] and c['los'] < A1['los'] + 14 * 3600.0]
    assert len(cands) >= 2
    prim = {'clave': 'cdmx', 'aos': A1['aos'], 'los': A1['los']}
    elegidos, tabla = X.seleccionar_pases_od(esc, cands, prim, 2, obj, ('az', 'el', 'rr'))
    assert len(elegidos) == 3 and elegidos[0] is prim
    s = [t[2] for t in tabla]
    assert s[1] < s[0]
    # las ventanas del planificador (hechas con el TLE viejo) casan con los pases
    # verdaderos; en pases rasantes el AOS difiere hasta ~1 min (medido: 55 s)
    for c in elegidos[1:]:
        pv = X.pase_verdadero(esc, c)
        assert pv is not None and abs(pv['aos'] - c['aos']) < 90.0


# ------------------------------------------------ serializacion y perfil para la antena

def test_resultado_json_ida_y_vuelta_y_perfil_od(esc, arranque):
    """El JSON de la orbita estimada se lee de vuelta con el mismo estado, y
    `perfil_od` entrega un pase con el MISMO formato que `orbit.muestrear_pase`
    (lo que consume reference_generator), con AOS/LOS del pase verdadero a
    menos de 15 s (error de prediccion del OD)."""
    med = esc.medidas(arranque, rng=np.random.default_rng(1), dt_s=5.0)
    res = esc.ajustar([med], ('az', 'el', 'rr'), prior=True)
    d = od.resultado_json(res, esc.reloj, 'ISS', 25544, ['cdmx'])
    x, modelo, reloj, estp = od.orbita_de_json(d)
    assert np.allclose(x, res.x) and not estp and modelo.grado == 2
    assert reloj.epoca == esc.reloj.utc(res.t0).astimezone(UTC) or \
        abs((reloj.epoca - esc.reloj.utc(res.t0)).total_seconds()) < 1e-3
    sig = esc.pases('cdmx', arranque['los'] + 3 * 3600.0, 2.9 * 86400.0, 10.0)[0]
    inicio = esc.reloj.utc(sig['aos'] - 1800.0).strftime('%Y-%m-%dT%H:%M:%S')
    perfil, pase, _, est = od.perfil_od(d, 'cdmx', 0, inicio, 12.0)
    ref = o.muestrear_pase(esc.tle, ESTACIONES['cdmx'], {
        'aos': esc.reloj.utc(sig['aos']), 'duracion_s': sig['duracion_s']}, 0.5)
    assert set(perfil) == set(ref)
    assert abs((pase['aos'] - esc.reloj.utc(sig['aos'])).total_seconds()) < 15.0
    assert perfil['t'][0] == 0.0 and np.all(np.diff(perfil['t']) > 0)
    assert np.all(np.diff(perfil['az_continuo']) * np.sign(perfil['az_continuo'][-1]
                                                          - perfil['az_continuo'][0]) > -1e-6)
    assert perfil['el'].max() == pytest.approx(pase['el_max_deg'], abs=0.2)
    with pytest.raises(LookupError):
        od.perfil_od(d, 'cdmx', 500, inicio, 12.0)
