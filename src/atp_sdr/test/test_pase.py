"""La cadena completa sobre pases REALES del modelo (esferico y SGP4): Doppler
estimado contra el verdadero, en S y en Ka, y los defectos del oscilador."""
import math

import numpy as np
import pytest

from atp_ros import model
from atp_sdr.cadena import ENGANCHADO, CadenaRF, simular_pase


def _segmento(banda, dur_s, modo='portadora', el_max=60.0, semilla=5, err_deg=0.02,
              t_centro=None, perfil=None, **kw):
    """Cadena sobre `dur_s` segundos centrados en el cenit del pase (donde la
    tasa Doppler es maxima) o en `t_centro`."""
    perfil = perfil or model.perfil_pase(550.0, el_max, 5.0)
    tc = perfil['duracion'] / 2 if t_centro is None else t_centro
    cad = CadenaRF(banda, modo, semilla, **kw)
    out = simular_pase(cad, perfil, tc - dur_s / 2, tc + dur_s / 2, err_deg)
    return cad, out


@pytest.mark.parametrize('banda,dur,rms_max', [('S', 40.0, 0.5), ('Ka', 15.0, 0.3)])
def test_doppler_estimado_contra_verdadero_en_un_pase(banda, dur, rms_max):
    """Pase de 60 grados de elevacion maxima (550 km), 20 s a cada lado... el
    tramo central, donde la tasa Doppler es maxima (S: ~390 Hz/s, Ka: ~5.3
    kHz/s). Error rms del Doppler (NCO del receptor contra la verdad a mitad de
    cada epoca, con el receptor enganchado) en Hz: la cota teorica del ruido a
    ~92 dB-Hz con B_L = 30 Hz (S) o 100 Hz (Ka) es de decimas de Hz."""
    cad, out = _segmento(banda, dur)
    r = cad.resumen()
    assert r['huecos'] == 0 and r['perdidas'] == 0 and r['adquisiciones'] == 1
    assert r['enganchado_pct'] > 97.0
    assert r['doppler_rms_hz'] < rms_max
    assert abs(r['doppler_sesgo_hz']) < 0.05
    # C/N0 medido contra el del presupuesto (mismo error de apuntamiento)
    assert abs(r['cn0_sesgo_db']) < 0.1
    assert r['cn0_rms_db'] < 0.35
    # tasa Doppler vista en el tramo
    f = np.array([m['doppler_true_hz'] for m in out])
    tasa = np.max(np.abs(np.diff(f))) / cad.dt
    assert tasa > (300.0 if banda == 'S' else 4000.0)


def test_el_2o_orden_pierde_el_pase_en_ka_y_el_3o_no():
    """Tasa de 5.3 kHz/s en Ka: el 2.o orden con B_L = 100 Hz deja un error de fase
    constante de 2 pi 5300 / (B_L/0.53)^2 = 0.9 rad (> 45 grados: el detector lo
    declara sin enganche); el 3.o lo anula."""
    c3, _ = _segmento('Ka', 6.0, orden=3)
    c2, _ = _segmento('Ka', 6.0, orden=2)
    assert c3.resumen()['enganchado_pct'] > 90.0
    assert c2.resumen()['enganchado_pct'] < 40.0


def test_offset_del_oscilador_se_cuenta_como_doppler():
    """Un TCXO de 1 ppm a 2.2 GHz (2.2 kHz) desplaza todo: el receptor adquiere
    y sigue, pero el 'Doppler' medido tiene un sesgo igual al offset (sin
    calibrar). Con la ventana ampliada se engancha igual."""
    cad, _ = _segmento('S', 12.0, offset_lo_hz=2200.0)
    r = cad.resumen()
    assert r['enganchado_pct'] > 95.0
    assert r['doppler_sesgo_hz'] == pytest.approx(2200.0, abs=0.2)
    assert r['doppler_rms_hz'] - 2200.0 < 1.0 or math.sqrt(
        r['doppler_rms_hz'] ** 2 - r['doppler_sesgo_hz'] ** 2) < 0.5


def test_ruido_de_fase_del_oscilador_no_rompe_el_seguimiento():
    """Ancho de linea de 0.5 Hz (un TCXO): la varianza de fase del lazo de
    B_L = 30 Hz es 2 pi lw T sum g^2 = 0.029 rad^2 (10 grados); con 5 Hz seria
    0.29 rad^2 (31 grados) y el detector ya no lo daria por enganchado. El NCO
    sigue ademas el ruido de FRECUENCIA del oscilador (blanco, densidad
    lw/(2 pi) Hz^2/Hz): lo cuenta como Doppler, con varianza lw B_L / pi."""
    from atp_sdr.receptor import ancho_banda_discreto
    cad, _ = _segmento('S', 12.0, lw_lo_hz=0.5)
    r = cad.resumen()
    assert r['enganchado_pct'] > 90.0 and r['perdidas'] == 0
    bl = ancho_banda_discreto(3, 30.0, 1e-3)
    assert r['doppler_rms_hz'] ** 2 == pytest.approx(0.5 * bl / math.pi, rel=0.25)


def test_bpsk_sobre_un_pase_en_s():
    """BPSK a 12.5 ksps sobre el mismo Doppler: sin errores de bit a ~92 dB-Hz
    (Eb/N0 = 51 dB) y el Doppler se sigue con la etapa de cuadrado + FLL."""
    cad, _ = _segmento('S', 10.0, modo='bpsk')
    r = cad.resumen()
    assert r['enganchado_pct'] > 90.0
    assert r['ber_n'] > 80_000 and r['ber'] == 0.0
    assert r['doppler_rms_hz'] < 3.0
    assert abs(r['cn0_sesgo_db']) < 0.5


def test_perdida_y_readquisicion():
    """El C/N0 cae 70 dB durante 1 s (la senal queda bajo el ruido): el lazo
    pierde el enganche, la cadena vuelve a BUSQUEDA y readquiere con la FFT
    cuando la senal vuelve."""
    perfil = model.perfil_pase(550.0, 60.0, 5.0)
    cad = CadenaRF('S', 'portadora', 3)
    estados = []
    tc = perfil['duracion'] / 2
    for k in range(int(10.0 / cad.dt)):
        t = tc + k * cad.dt
        cad.delta_cn0 = -70.0 if 3.0 <= k * cad.dt < 4.0 else 0.0
        d = float(np.interp(t, perfil['t'], perfil['d']))
        vr = float(np.interp(t, perfil['t'], perfil['vr']))
        estados.append(cad.paso(t, d, vr, 0.02)['estado'])
    assert cad.resumen()['perdidas'] >= 1 or cad.resumen()['adquisiciones'] >= 2
    assert cad.resumen()['adquisiciones'] >= 2
    assert estados[-1] == ENGANCHADO
    assert estados[10] == ENGANCHADO


def test_determinismo_de_la_cadena():
    """Misma semilla, mismo resumen BIT A BIT (salvo los tiempos de pared);
    otra semilla, otro (ruido distinto)."""
    quita = ('muestras_por_s_rx', 'muestras_por_s_total', 'ms_por_bloque_gen',
             'ms_por_bloque_rx')

    def resumen(sem):
        cad, _ = _segmento('S', 4.0, semilla=sem)
        return {k: v for k, v in cad.resumen().items() if k not in quita}
    a, b, c = resumen(3), resumen(3), resumen(4)
    assert a == b
    assert a != c


def test_pase_real_sgp4_de_la_iss_en_s():
    """El pase REAL de la ISS sobre CDMX con SGP4 y el TLE de data/tle (el mismo
    del experimento S4): el Doppler de +-48 kHz de la banda S se sigue con error
    rms < 0.5 Hz, y el C/N0 medido coincide con el del presupuesto."""
    orbit = pytest.importorskip('atp_ros.orbit')
    try:
        perfil, pase, tle, est = orbit.perfil_tle('ISS', 'cdmx', 0, '2026-10-02T15:00:00')
    except (LookupError, FileNotFoundError, OSError) as e:      # sin TLE en el arbol
        pytest.skip(f'sin TLE: {e}')
    cad, _ = _segmento('S', 40.0, perfil=perfil, h_km=420.0)
    r = cad.resumen()
    f0 = model.BANDAS['S']
    fd = -np.asarray(perfil['vr']) * 1000.0 / model.C_LUZ_M_S * f0
    assert 40e3 < np.max(np.abs(fd)) < 60e3
    assert r['enganchado_pct'] > 95.0 and r['perdidas'] == 0
    assert r['doppler_rms_hz'] < 0.5
    assert abs(r['cn0_sesgo_db']) < 0.15
