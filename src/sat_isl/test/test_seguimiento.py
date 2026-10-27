"""Validacion de sat_isl.seguimiento: estabilidad y ancho de banda del lazo FSM +
montura, rechazo de perturbaciones (analitico contra simulacion temporal),
seguimiento de rampa (tipo 2), QPD y normalizacion de las PSD.

Referencias independientes: asintotas cerradas del lazo (S ~ s^2 / (Ki (s + w_d)) a baja
frecuencia), respuesta a un tono contra |S(f)|, integral espectral contra la
simulacion con `lfilter`, y la integral numerica de la mancha gaussiana en el QPD.
"""
import dataclasses
import math

import numpy as np
import pytest
from scipy import signal
from scipy.integrate import dblquad

from sat_isl import seguimiento as S
from sat_isl import terminal as T

LAZO = T.LAZO_CARDAN


@pytest.fixture(scope='module')
def lin():
    return S.lazo_lineal(LAZO)


# ------------------------------------------------------------------ estabilidad y margenes

def test_lazo_estable_con_margenes_de_un_servo_fino(lin):
    assert lin.estable()
    m = lin.margenes()
    assert math.isclose(m['f_cruce_hz'], LAZO.f_cruce_hz, rel_tol=0.01)       # el cruce pedido
    assert m['pm_deg'] > 45.0, m                                              # margen de fase > 45 deg
    assert m['gm_db'] > 6.0, m                                                # margen de ganancia > 6 dB
    assert m['ms'] < 2.0, m                                                   # pico de sensibilidad < 6 dB


def test_ancho_de_banda_del_lazo(lin):
    m = lin.margenes()
    # |T| cae -3 dB entre el cruce y ~4 veces el cruce (lo da el FSM, no el controlador)
    assert LAZO.f_cruce_hz < m['f_bw_hz'] < 5 * LAZO.f_cruce_hz
    # a baja frecuencia T ~ 1 (el haz sigue la perturbacion) y S ~ 0
    assert abs(lin.T(np.array([1.0]))[0]) > 0.999
    assert abs(lin.S(np.array([1.0]))[0]) < 3e-3


def test_sensibilidad_a_baja_frecuencia_es_la_asintota_de_tipo_2(lin):
    """S ~ s^2 / (Ki (s + w_d)) para w << w_c : la montura (integrador) y el integrador del
    FSM dan tipo 2. A 0.1 y 0.5 Hz coincide con la asintota a mejor del 1-2 %."""
    wd = 2 * math.pi * LAZO.f_descarga_hz
    for f, tol in ((0.1, 0.01), (0.5, 0.02)):
        w = 2 * math.pi * f
        asint = abs(-w * w / (lin.Ki * (1j * w + wd)))
        assert math.isclose(abs(lin.S(np.array([f]))[0]), asint, rel_tol=tol), f
    # y pendiente de -40 dB/dec bajo w_d: S(0.5 Hz)/S(0.1 Hz) = 25 |wd + j w1| / |wd + j w5| ~ 24.8
    r = abs(lin.S(np.array([0.5]))[0]) / abs(lin.S(np.array([0.1]))[0])
    assert math.isclose(r, 25.0 * abs(wd + 0.628j) / abs(wd + 3.14j), rel_tol=0.02)


def test_respuesta_a_un_tono_es_la_sensibilidad(lin):
    """Un tono de 20 Hz y 10 urad a traves del lazo simulado sale con la amplitud |S(20 Hz)| x 10."""
    fs = LAZO.fs_hz
    t = np.arange(int(4 * fs)) / fs
    d = 10.0 * np.sin(2 * math.pi * 20.0 * t)
    e = signal.lfilter(lin.bS, lin.aS, d)
    amp = np.max(np.abs(e[int(2 * fs):]))
    assert math.isclose(amp, 10.0 * abs(lin.S(np.array([20.0]))[0]), rel_tol=0.02)


def test_rampa_error_nulo_en_regimen_permanente(lin):
    """La linea de vista gira ~mrad/s: con el lazo de tipo 2 un rampa de 4 mrad/s se sigue con
    error -> 0 (el integrador de la montura absorbe la rampa)."""
    fs = LAZO.fs_hz
    t = np.arange(int(3 * fs)) / fs
    d = 4000.0 * t                                    # urad (4 mrad/s)
    e = signal.lfilter(lin.bS, lin.aS, d)
    assert abs(e[-1]) < 0.05                          # urad tras 3 s
    assert np.max(np.abs(e)) < 5.0                    # el transitorio no pasa de unos urad (|e| < theta_h/5)


def test_escalon_de_arranque_decae_sin_sobreimpulso_excesivo(lin):
    fs = LAZO.fs_hz
    d = np.full(int(0.5 * fs), 5.0)                   # 5 urad de error de centroide al cerrar
    e = signal.lfilter(lin.bS, lin.aS, d)
    assert abs(e[0]) <= 5.0 * 1.0001
    assert np.max(np.abs(e)) < 5.0 * 1.4              # sobreimpulso < 40 %
    assert abs(e[-1]) < 1e-3 and np.max(np.abs(e[int(0.01 * fs):])) < 1.0   # asentado en ~10 ms (cruce a 400 Hz)


def test_lazo_con_menos_margen_se_detecta_inestable():
    """Control negativo: con 6 muestras de retardo el mismo diseno pierde el margen de fase
    (el analisis no da siempre 'estable')."""
    malo = dataclasses.replace(LAZO, retardo_muestras=6.0, f_cruce_hz=900.0)
    lm = S.LazoLineal(malo)
    m = lm.margenes()
    assert (not lm.estable()) or m['pm_deg'] < 20.0 or m['gm_db'] < 3.0


def test_montura_lenta_pide_mas_carrera_al_fsm():
    """El ADCS (0.005 Hz) como montura gruesa deja al FSM la deriva lenta: necesita 20x mas carrera."""
    y_cardan = S.carrera_baja_frecuencia_urad(LAZO)
    y_adcs = S.carrera_baja_frecuencia_urad(T.LAZO_ADCS)
    assert y_cardan < 50.0
    assert y_adcs > 10 * y_cardan
    assert 4 * y_cardan < LAZO.fsm_carrera_urad < 4 * y_adcs      # el cardan cabe en el FSM, el ADCS no


def test_el_lazo_discreto_rechaza_montura_demasiado_lenta():
    with pytest.raises(ValueError):
        S.Canal(T.LAZO_ADCS)


# ------------------------------------------------------------------ perturbaciones y rms

@pytest.mark.parametrize('comp', S.PLATAFORMA_DEFECTO, ids=lambda c: c.nombre)
def test_psd_de_cada_componente_integra_a_su_rms(comp):
    f = np.logspace(-4, math.log10(0.5 * LAZO.fs_hz), 400001)
    fm = S.FormadorRuido(comp, LAZO.fs_hz)
    var = np.trapz(fm.psd_unilateral(f), f) if hasattr(np, 'trapz') else np.trapezoid(fm.psd_unilateral(f), f)
    assert math.isclose(math.sqrt(var), comp.rms_urad, rel_tol=0.01)


def test_jitter_de_rueda_es_ud_sobre_inercia():
    from sat_adcs.fisica import cubesat_3u
    ixx = float(cubesat_3u().J[0, 0])
    assert math.isclose(S.jitter_rueda_urad(3e-8, ixx), 3e-8 / ixx * 1e6, rel_tol=1e-12)
    assert 0.5 < S.jitter_rueda_urad(3e-8, ixx) < 1.0           # ~0.7 urad en un 3U


def test_rms_analitico_contra_simulacion_temporal():
    """El error residual rms (integral de |S|^2 por la PSD + ruido del QPD por |T|^2) coincide
    con 60 s simulados con `Canal` a mejor del 5 %; la perturbacion cruda es ~400 urad y
    queda < 1 urad."""
    nea = 0.3
    a = S.rms_analitico(LAZO, nea_urad=nea)
    rng = np.random.default_rng(11)
    ch = S.Canal(LAZO)
    E, Y = [], []
    for _ in range(1200):                              # 60 s en bloques de 50 ms
        e, y, _ = ch.bloque(500, rng, nea)
        E.append(e)
        Y.append(y)
    E, Y = np.concatenate(E)[100000:], np.concatenate(Y)[100000:]    # descarta 10 s de arranque
    assert math.isclose(E.std(), a['e_rms'], rel_tol=0.05), (E.std(), a['e_rms'])
    # la posicion del FSM la domina la deriva de 0.05 Hz (~3 ciclos en 60 s: poca estadistica);
    # sin ella la comparacion es fina
    a2 = S.rms_analitico(LAZO, comps=S.PLATAFORMA_DEFECTO[1:], nea_urad=nea)
    ch2, rng2, Y2 = S.Canal(LAZO, comps=S.PLATAFORMA_DEFECTO[1:]), np.random.default_rng(12), []
    for _ in range(1200):
        Y2.append(ch2.bloque(500, rng2, nea)[1])
    assert math.isclose(np.concatenate(Y2)[100000:].std(), a2['y_rms'], rel_tol=0.06)
    assert Y.std() > 5.0 and a['y_rms'] < 40.0           # y la total es decenas de urad: cabe en +-1000
    assert a['d_rms'] > 300.0 and a['e_rms'] < 1.0
    assert a['d_rms'] / a['e_rms'] > 300.0              # rechazo de la perturbacion > 50 dB


def test_el_ruido_del_qpd_se_filtra_con_t_no_con_s():
    """Sin perturbacion el error residual es -T n: con ruido blanco de 1 urad/muestra su rms es
    sqrt(integral de |T|^2 df / (fs/2))."""
    sin_pert = ()
    a = S.rms_analitico(LAZO, comps=sin_pert, nea_urad=1.0)
    f = np.linspace(0.0, 0.5 * LAZO.fs_hz, 1 << 18)[1:]
    t2 = np.abs(S.lazo_lineal(LAZO).T(f)) ** 2
    esperado = math.sqrt(np.mean(t2))
    assert math.isclose(a['e_rms_ruido'], esperado, rel_tol=5e-3)
    assert 0.3 < a['e_rms_ruido'] < 0.7          # el lazo deja pasar el ruido hasta su ancho de banda (~1.4 kHz de 5)


def test_canal_es_reproducible_con_la_misma_semilla():
    def corre(semilla):
        ch = S.Canal(LAZO)
        rng = np.random.default_rng(semilla)
        return np.concatenate([ch.bloque(500, rng, 0.2)[0] for _ in range(10)])
    assert np.array_equal(corre(3), corre(3))
    assert not np.array_equal(corre(3), corre(4))


# ------------------------------------------------------------------ QPD

def test_qpd_pendiente_y_saturacion():
    w = 100.0
    # pendiente en el origen = K_QPD / w = 1.596 / w
    h = 1e-3
    pend = (S.error_cuadrante(h, w) - S.error_cuadrante(-h, w)) / (2 * h)
    assert math.isclose(pend, S.K_QPD / w, rel_tol=1e-6)
    assert math.isclose(S.K_QPD, 2 * math.sqrt(2 / math.pi), rel_tol=1e-12)
    assert abs(S.error_cuadrante(5 * w, w)) > 0.999            # satura
    assert S.error_cuadrante(0.0, w) == 0.0


def test_qpd_contra_la_integral_numerica_de_la_mancha():
    """(A+B-C-D)/suma con A,B a la derecha: integral de la gaussiana I ~ exp(-2 r^2/w^2)
    centrada en x0 sobre x > 0 menos x < 0, normalizada."""
    w, x0 = 100.0, 37.0
    inten = lambda y, x: math.exp(-2 * ((x - x0) ** 2 + y ** 2) / w ** 2)  # noqa: E731
    lim = 6 * w
    der = dblquad(inten, 0, lim, -lim, lim)[0]
    izq = dblquad(inten, -lim, 0, -lim, lim)[0]
    assert math.isclose((der - izq) / (der + izq), float(S.error_cuadrante(x0, w)), rel_tol=1e-5)


def test_ruido_del_qpd_crece_al_bajar_la_potencia():
    kw = dict(lam_m=1550e-9, fs_hz=10000.0, w_urad=100.0, fraccion=0.1, nep_w_rthz=1.1e-13)
    n1 = S.ruido_qpd_urad(1e-8, **kw)
    n2 = S.ruido_qpd_urad(1e-9, **kw)
    n3 = S.ruido_qpd_urad(1e-10, **kw)
    assert n1 < n2 < n3
    # dominado por el ruido termico (NEP): escala 1/P
    assert math.isclose(n3 / n2, 10.0, rel_tol=0.1)
    # a mucha potencia manda el ruido de disparo: escala 1/sqrt(P)
    a = S.ruido_qpd_urad(1e-3, **kw)
    b = S.ruido_qpd_urad(1e-2, **kw)
    assert math.isclose(a / b, math.sqrt(10.0), rel_tol=0.05)
