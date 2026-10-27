"""Validacion del escenario ADCS (sin ROS): geometria, detumbling con B-dot,
apuntamiento en NADIR y a la estacion durante un pase real con SGP4, y el
acople con el presupuesto de enlace.

Las corridas completas tardan ~2 min cada una (7000 pasos de 1 s); se comparten
entre pruebas con fixtures de modulo.
"""
import math

import numpy as np
import pytest

from atp_ros import model, orbit
from sat_adcs import fisica as F
from sat_adcs import misiones as M
from sat_adcs.escenario import cargar_base, crear, preparar, simular

CFG_DEFECTO = dict(inicio_utc='2026-10-02T20:30:00')       # pase de 79 deg en luz
CFG_ECLIPSE = dict(inicio_utc='2026-10-02T08:30:00')       # pase de 36 deg dentro del eclipse


@pytest.fixture(scope='module')
def corrida_defecto():
    esc = preparar(M.Config(**CFG_DEFECTO))
    med, planta, ctrl = simular(esc)
    return esc, med, med.cifras()


@pytest.fixture(scope='module')
def corrida_dipolo():
    esc = preparar(M.Config(**CFG_DEFECTO, modelo_bordo_nmax=1))
    med, planta, ctrl = simular(esc)
    return esc, med, med.cifras()


@pytest.fixture(scope='module')
def corrida_eclipse_grado4():
    esc = preparar(M.Config(**CFG_ECLIPSE, modelo_bordo_nmax=4))
    med, planta, ctrl = simular(esc)
    return esc, med, med.cifras()


# ------------------------------------------------------------- geometria y referencias

def test_geometria_coincide_con_orbit_en_el_pase():
    """La elevacion del satelite vista desde la estacion (cadena propia:
    GMST, ECEF, ENU de la latitud geodesica) vale la mascara en el AOS y en el
    LOS de `orbit.predecir_pases` (a 0.05 deg: la biseccion de AOS/LOS tiene
    tolerancia de 0.1 s y la elevacion cambia ~0.2 deg/s a baja elevacion)
    y el maximo coincide con `el_max_deg` (0.1 deg)."""
    esc = preparar(M.Config(**CFG_DEFECTO))
    geo = M.Geometria(esc.efem, esc.estacion)
    p = esc.pase
    assert geo.el(esc.aos_s) == pytest.approx(esc.estacion.mascara_deg, abs=0.05)
    assert geo.el(esc.los_s) == pytest.approx(esc.estacion.mascara_deg, abs=0.05)
    t = np.arange(esc.aos_s, esc.los_s, 1.0)
    assert max(geo.el(x) for x in t) == pytest.approx(p['el_max_deg'], abs=0.1)


def test_referencias_lvlh_y_gs_apuntan_bien():
    """q_ref_lvlh pone +Z en el nadir, +Y en -(normal de la orbita) y es un giro
    propio; q_ref_gs pone +Z exactamente hacia la estacion (error < 1e-9 rad)
    y +Y lo mas cerca posible de -normal."""
    esc = preparar(M.Config(**CFG_DEFECTO))
    geo = M.Geometria(esc.efem, esc.estacion)
    g = geo.en(esc.aos_s + 200.0)
    q = M.q_ref_lvlh(g['r'], g['v'])
    R = F.q_a_dcm(q)
    assert np.allclose(R[:, 2], -g['r'] / np.linalg.norm(g['r']), atol=1e-12)
    n = np.cross(g['r'], g['v'])
    assert np.allclose(R[:, 1], -n / np.linalg.norm(n), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)
    qg = M.q_ref_gs(g['r'], g['v'], g['r_gs'])
    z = F.q_a_dcm(qg)[:, 2]
    assert F.angulo_entre(z, g['r_gs'] - g['r']) < 1e-9
    # el el de la estacion visto desde el satelite: el cosenos entre +Z_ref y el nadir
    # crece con el angulo fuera del nadir (el satelite mira al lado)
    assert F.angulo_entre(z, -g['r']) > math.radians(5)


def test_velocidad_y_aceleracion_de_referencia_orbita_circular():
    """En una orbita circular el LVLH gira con la velocidad orbital n alrededor
    de su eje -Y... (w_ref = [0, -n, 0] en el marco de la referencia) y a_ref =
    0, analiticamente. Las diferencias centrales (h = 1 s) lo reproducen a
    1e-6 relativo."""
    t0 = orbit.a_utc('2026-10-02T00:00:00')
    efem = M.EfemerideCircular(6928.0, 51.6, t0, 30.0, 10.0)
    n = math.sqrt(F.MU_M3_S2 * 1e-9 / 6928.0 ** 3)

    def fq(t):
        r, v = efem.rv(t)
        return M.q_ref_lvlh(r, v)
    q, w, a = M.referencia_con_derivadas(fq, 500.0)
    assert np.allclose(w, [0.0, -n, 0.0], rtol=1e-6, atol=1e-10)
    assert np.linalg.norm(a) < 1e-9


# ------------------------------------------------------------- enlace

def test_umbral_de_apuntamiento_cuesta_exactamente_la_perdida_elegida():
    """Por construccion: con e = umbral, la perdida por apuntamiento de la antena
    del satelite es `perdida_max_db` (L = 12 (e/theta3)^2). En e = theta3/2
    son 3 dB (el punto de media potencia, la propiedad definitoria del
    ancho de haz). Y el margen baja exactamente lo que sube la perdida."""
    cfg = M.Config()
    th3 = model.ancho_haz(cfg.d_sat_m, model.BANDAS[cfg.banda])
    u = M.umbral_apuntamiento_deg(cfg)
    assert M.enlace_sat(u, 1500.0, cfg)['l_sat_db'] == pytest.approx(cfg.perdida_max_db, rel=1e-12)
    assert M.enlace_sat(th3 / 2, 1500.0, cfg)['l_sat_db'] == pytest.approx(3.0, rel=1e-12)
    m0 = M.enlace_sat(0.0, 1500.0, cfg)
    m1 = M.enlace_sat(u, 1500.0, cfg)
    assert m0['margen_db'] - m1['margen_db'] == pytest.approx(cfg.perdida_max_db, rel=1e-9)
    # con la antena de tierra del modelo y 1500 km el enlace cierra con margen
    assert m0['margen_db'] > 0
    # fuera del lobulo la perdida se acota con la ganancia de la antena
    g = model.ganancia_plato(cfg.d_sat_m, model.BANDAS[cfg.banda], cfg.eta_sat)
    assert M.enlace_sat(80.0, 1500.0, cfg)['l_sat_db'] == pytest.approx(g)


# ------------------------------------------------------------- B-dot

def _detumble(seed, k_signo=1.0, t_max=6000.0, w0=8.0):
    cfg = M.Config(**CFG_DEFECTO, semilla=seed, w0_deg_s=w0)
    esc = preparar(cfg)
    planta, ctrl, med = crear(esc)
    ctrl.bdot.k *= k_signo
    ts, ws = [], []
    t = 0.0
    while t <= t_max:
        s = planta.medir()
        c = ctrl.paso(s['t'], s)
        ts.append(t)
        ws.append(math.degrees(np.linalg.norm(s['w'])))
        if c['modo'] != M.MODO_DETUMBLE:
            break
        planta.avanzar(cfg.paso_s, c['u_w'], c['m'])
        t += cfg.paso_s
    return np.array(ts), np.array(ws), ctrl, c


@pytest.mark.parametrize('semilla', [7, 11, 23])
def test_bdot_la_velocidad_angular_decae_y_converge(semilla):
    """B-dot con magnetopares (solo magnetometro ruidoso): desde un tumbling de
    8 deg/s en direccion aleatoria, |w| (a) decae: el promedio de cada bloque
    de 15 min es menor que el del anterior; (b) converge por debajo del
    umbral h_max/(4 J_max) (el momento del cuerpo cabe 4 veces en las
    ruedas, 0.59 deg/s) y el controlador pasa a NADIR; (c) lo hace en un
    tiempo finito (< 2 h) con el campo IGRF-13 completo como verdad. La
    cota inferior de B-dot es ~2 n = 0.12 deg/s (el campo gira respecto al
    inercial 2 veces por orbita), de modo que el umbral es alcanzable."""
    ts, ws, ctrl, c = _detumble(semilla)
    assert c['modo'] == M.MODO_NADIR, f'no termino el detumbling (|w| final {ws[-1]:.3f} deg/s)'
    assert ts[-1] < 7200.0
    bloques = [ws[i:i + 900].mean() for i in range(0, len(ws) - 450, 900)]
    assert all(a > b for a, b in zip(bloques, bloques[1:])), bloques
    assert ws[-1] < math.degrees(ctrl.w_umbral) * 1.05
    assert ws[0] == pytest.approx(8.0, abs=1e-6)


def test_bdot_con_signo_invertido_empeora():
    """Control negativo: con k < 0 (m = +k dB/dt) el par acelera el cuerpo en
    vez de frenarlo. Si el B-dot de arriba 'convergiera' por cualquier otra
    razon (p. ej. disipacion numerica), esta prueba lo delataria: en 15 min
    |w| debe CRECER (tope: saturacion del dipolo)."""
    ts, ws, ctrl, c = _detumble(7, k_signo=-1.0, t_max=900.0)
    assert ws[-1] > ws[0]


# ------------------------------------------------------------- escenario completo

def test_escenario_detumble_a_nadir(corrida_defecto):
    esc, med, c = corrida_defecto
    assert c['t_nadir_s'] is not None and c['t_detumble_s'] is not None
    assert c['t_detumble_s'] + esc.cfg.detumble_hold_s <= c['t_nadir_s'] + 1e-9
    # entra a NADIR con |w| bajo el umbral y con margen de tiempo antes del pase
    assert c['w_en_nadir_deg_s'] < math.degrees(med.w_umbral) * 1.05
    assert c['t_nadir_s'] < esc.aos_s - esc.cfg.previo_s - c['t_asentamiento_s']


def test_escenario_nadir_error_acotado_por_el_modelo_magnetico(corrida_defecto):
    """En NADIR (antes del preapuntado) el error lo fija el conocimiento de
    actitud, y este el modelo de campo de a bordo: con el IGRF de grado 4
    (error ~4 % del campo, ~1.7 deg de direccion, medido por
    `error_modelo_campo`) el p95 del apuntamiento a nadir debe quedar
    bajo 2 x ese error de direccion (el filtro, que suaviza con el giroscopio y
    ve el Sol en luz, debe ganarle al magnetometro desnudo) y el error de
    conocimiento del bore-sight bajo 1 deg rms."""
    esc, med, c = corrida_defecto
    em = F.error_modelo_campo(6978.0, 97.5, esc.cfg.modelo_bordo_nmax, 120)
    assert c['nadir_n'] > 500
    assert c['nadir_p95'] < 2.0 * em['rms_deg']
    assert c['conoc_rms'] < 1.0


def test_el_dipolo_a_bordo_limita_la_actitud_y_el_filtro_no_lo_sabe(corrida_defecto, corrida_dipolo):
    """LIMITACION medida: con el dipolo inclinado a bordo (error ~32 % del
    campo, ~13 deg de direccion) el error de conocimiento es de varios grados
    (> 3 deg rms y > 3 x el del grado 4) y el filtro esta SOBRECONFIADO: su
    1-sigma reportado (< 1 deg) es mucho menor que su error real. Es lo que se
    espera: el error del modelo magnetico esta correlado en el tiempo (no es
    ruido blanco) y el MEKF lo trata como blanco. Que el pase se cumpla o no
    con el dipolo depende de la geometria (ver README), no de la calidad
    del estimador."""
    _, _, c4 = corrida_defecto
    _, _, c1 = corrida_dipolo
    assert c1['conoc_rms'] > 3.0 and c1['conoc_rms'] > 3.0 * c4['conoc_rms']
    assert c1['sigma_rms'] < 1.0
    assert c1['conoc_rms'] > 5.0 * c1['sigma_rms']             # sobreconfianza
    assert c1['nadir_p95'] > 3.0 * c4['nadir_p95']


def test_escenario_track_gs_bajo_el_umbral_del_enlace(corrida_defecto):
    """Apuntamiento de la antena del satelite a la estacion durante un pase REAL
    (FUNCUBE-1 sobre Ciudad de Mexico, 79 deg de elevacion maxima, en luz,
    campo de a bordo de grado 4). Umbral: el error que cuesta 1 dB de enlace,
    e = theta3 sqrt(1/12) = 2.4 deg para el haz de 8.3 deg (reflectarray
    de 0.3 m en banda X; derivado del enlace, no ajustado al resultado). Se
    exige p95 <= umbral, que la perdida por apuntamiento no pase de 1 dB y
    que el enlace nunca se cierre con menos de 10 dB de margen (el margen sin
    desapuntamiento ya es de ~14 dB)."""
    esc, med, c = corrida_defecto
    assert esc.pase['el_max_deg'] > 70.0 and not c['eclipse_en_pase']
    assert c['track_n'] > 500
    assert c['track_p95'] <= c['umbral_deg']
    assert c['l_sat_max_db'] <= esc.cfg.perdida_max_db
    assert c['margen_min_db'] > 10.0
    assert c['cerrado_pct'] == 100.0
    assert c['cumple']


def test_escenario_ruedas_dentro_de_su_capacidad(corrida_defecto):
    """Ninguna rueda excede su momento maximo (h_max, de la hoja de datos) ni
    llega a saturar (< 80 % de h_max) durante todo el escenario: la
    desaturacion con magnetopares funciona."""
    esc, med, c = corrida_defecto
    assert c['h_max_nms'] <= esc.sat.h_max + 1e-9
    assert c['h_max_frac'] < 0.8


def test_escenario_eclipse_con_campo_de_grado_4(corrida_eclipse_grado4):
    """Pase de 36 deg ENTERAMENTE en eclipse (sin sensor solar) con un campo
    de a bordo de grado 4 (error ~ 4 % del campo): solo magnetometro y
    giroscopio dan un apuntamiento por debajo del umbral del enlace. Con el
    dipolo, el mismo pase queda en el limite del umbral (2.46 deg contra
    2.40, medido, ver README): el modelo de campo es el factor limitante."""
    esc, med, c = corrida_eclipse_grado4
    assert c['eclipse_en_pase']
    assert c['track_n'] > 400
    assert c['track_p95'] <= c['umbral_deg']


# ------------------------------------------------------------- determinismo

def test_corrida_offline_determinista_y_sensible_a_la_semilla():
    """La misma configuracion y semilla dan EXACTAMENTE las mismas muestras
    (verdad, sensores y mandos); otra semilla da otras. Es la version sin ROS
    del `det` de scripts/validar.sh (el de ROS es validar_adcs.sh)."""
    def corre(semilla):
        esc = preparar(M.Config(**CFG_DEFECTO, semilla=semilla))
        med, planta, ctrl = simular(esc, t_fin_s=300.0)
        return np.array(med.w_deg), planta.y.copy()
    a1, y1 = corre(7)
    a2, y2 = corre(7)
    b1, y3 = corre(8)
    assert np.array_equal(a1, a2) and np.array_equal(y1, y2)
    assert not np.array_equal(a1, b1)
