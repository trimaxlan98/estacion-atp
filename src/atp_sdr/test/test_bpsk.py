"""BPSK/QPSK: BER contra Q(sqrt(2 Eb/N0)), reloj de simbolo y Costas."""
import math

import numpy as np
import pytest

from atp_sdr.bpsk import (LazoCostas, ReceptorBPSK, SincronizadorGardner, alinear_ber,
                          pendiente_gardner)
from atp_sdr.receptor import adquirir_fft, ancho_banda_discreto
from atp_sdr.senal import Beacon, ber_bpsk_teorica, intervalo_wilson, rrc

Z999 = 3.29            # 99.9 % bilateral


def _ber_ideal(modo, ebn0_db, n_sim, semilla, sps=4, bloque_sim=40000):
    """Receptor con la fase y el reloj exactos (genie): filtro adaptado
    h/sps y muestreo en los instantes de simbolo. Valida la normalizacion
    C/N0 <-> sigma^2 del generador y la formula de BER. Devuelve (errores, n)."""
    rs = 10e3
    fs = rs * sps
    bits_por_simbolo = 1 if modo == 'bpsk' else 2
    cn0 = ebn0_db + 10 * math.log10(rs * bits_por_simbolo)       # Eb = C Ts / bits
    b = Beacon(fs, modo, tasa_simbolos=rs, semilla=semilla)
    h = rrc(0.35, 8, sps)[::-1] / sps
    cola = np.zeros(len(h) - 1, dtype=complex)
    ys = []
    for _ in range(n_sim // bloque_sim):
        x = b.bloque(bloque_sim * sps, 0.0, 0.0, cn0)
        ent = np.concatenate([cola, x])
        ys.append(np.convolve(ent, h, 'valid')[::sps])
        cola = ent[-(len(h) - 1):]
    z = np.concatenate(ys)[8:]            # el retardo de los dos RRC: span simbolos
    bits = b.bits_tx()
    if modo == 'bpsk':
        rx = (z.real < 0).astype(np.int8)
    else:
        rx = np.empty(2 * len(z), dtype=np.int8)
        rx[0::2], rx[1::2] = (z.real < 0), (z.imag < 0)
    n = min(len(rx), len(bits))
    return int(np.sum(rx[:n] != bits[:n])), n


@pytest.mark.parametrize('modo', ['bpsk', 'qpsk'])
def test_ber_ideal_contra_la_curva_teorica(modo):
    """BER = Q(sqrt(2 Eb/N0)) (Proakis & Salehi, cap. 4.3): la curva medida debe
    contener a la teoria en su intervalo de confianza del 99.9 % de Wilson en
    cada punto, de 0 a 8 dB. El QPSK con codificacion Gray tiene la MISMA BER
    por bit que el BPSK a igual Eb/N0."""
    puntos = {0.0: 200_000, 2.0: 200_000, 4.0: 400_000, 6.0: 1_000_000, 8.0: 2_000_000}
    for k, (ebn0, n_sim) in enumerate(puntos.items()):
        err, n = _ber_ideal(modo, ebn0, n_sim, semilla=40 + k)
        lo, hi = intervalo_wilson(err, n, Z999)
        teo = ber_bpsk_teorica(ebn0)
        assert lo <= teo <= hi, (modo, ebn0, err / n, (lo, hi), teo)


def test_pendiente_del_detector_de_gardner():
    """Pendiente de la curva S normalizada: 1.00 +- 0.02 por simbolo (independiente
    de sps), positiva para muestreo tardio: es el Kd con que se dimensionan las
    ganancias del lazo de reloj (Rice 2009, cap. 8)."""
    for sps in (8, 16):
        assert pendiente_gardner(sps=sps) == pytest.approx(1.0, abs=0.03)
    assert SincronizadorGardner.KD == pytest.approx(pendiente_gardner(), abs=0.02)


def test_el_reloj_de_simbolo_engancha_desde_cualquier_desfase():
    """El lazo de Gardner lleva el instante de muestreo al centro del ojo desde
    cualquier desfase inicial: la dispersion de los simbolos (EVM) baja de
    ~40 % a < 8 % a 20 dB de Es/N0."""
    rs, sps = 10e3, 16
    fs = rs * sps
    h = rrc(0.35, 8, sps)[::-1] / sps
    for sem, desfase in enumerate((0.1, 0.3, 0.5, 0.8)):
        b = Beacon(fs, 'bpsk', tasa_simbolos=rs, semilla=sem)
        x = b.bloque(30000 * sps, 0.0, 0.0, 20.0 + 10 * math.log10(rs))
        x = x[int(desfase * sps):]
        y = np.convolve(x, h, 'valid')
        g = SincronizadorGardner(sps, 0.005)
        s = g.procesar(y)
        # convergencia: EVM = distancia al +-A mas cercano / A, ultimos 5000
        a = np.mean(np.abs(s.real[-5000:]))
        evm = np.sqrt(np.mean((np.abs(s.real[-5000:]) - a) ** 2 + s.imag[-5000:] ** 2)) / a
        assert evm < 0.12, (desfase, evm)


@pytest.mark.parametrize('theta0', [0.4, 1.2, 2.0, 2.9])
def test_costas_engancha_y_tiene_ambiguedad_de_180_grados(theta0):
    """El Costas engancha a theta0 o a theta0 - pi: el lazo no sabe cual es cual
    (Proakis cap. 6): con |theta0| < 90 grados los bits salen derechos y con mas
    de 90 salen invertidos. Se resuelve con sincronia o codificacion
    diferencial."""
    rng = np.random.default_rng(11)
    bits = rng.integers(0, 2, 4000)
    s = (1.0 - 2.0 * bits) * np.exp(1j * theta0) + 0.05 * (
        rng.standard_normal(4000) + 1j * rng.standard_normal(4000))
    lazo = LazoCostas(10e3, 0.02, 2)
    z, _ = lazo.procesar(s)
    assert np.max(np.abs(z.imag[-500:])) < 0.3          # fase quitada
    derecho = np.mean((z.real[-500:] < 0) == (bits[-500:] == 1))
    assert (derecho > 0.99) if theta0 < math.pi / 2 else (derecho < 0.01)


def test_varianza_de_fase_del_costas():
    """sigma^2 = B_L/(C/N0) (1 + 1/(2 Es/N0)) rad^2 (la misma ley que el PLL: el
    discriminador atan(Q/I) tiene la perdida del cuadrado, Kaplan & Hegarty cap.
    8) con el B_L efectivo del lazo digital a la tasa de simbolo; Es/N0 = 10 dB
    y B_L = 1 % de Rs."""
    rng = np.random.default_rng(12)
    rs, bn_rel, esn0 = 10e3, 0.01, 10.0
    n = 300_000
    snr = 10 ** (esn0 / 10)
    bits = rng.integers(0, 2, n)
    theta = 0.7
    s = (1.0 - 2.0 * bits) * np.exp(1j * theta) + math.sqrt(0.5 / snr) * (
        rng.standard_normal(n) + 1j * rng.standard_normal(n))
    lazo = LazoCostas(rs, bn_rel, 2)
    z, fase = lazo.procesar(s)
    err = (theta - fase + math.pi / 2) % math.pi - math.pi / 2
    err = err[5000:]
    bl = ancho_banda_discreto(2, bn_rel * rs, 1.0 / rs)
    teo = bl / (snr * rs) * (1 + 1 / (2 * snr))
    assert np.var(err) == pytest.approx(teo, rel=0.12)


@pytest.mark.parametrize('ebn0,perdida_max_db', [(4.0, 0.3), (6.0, 0.3)])
def test_ber_de_la_cadena_completa_con_doppler(ebn0, perdida_max_db):
    """Cadena completa a nivel IQ: adquisicion FFT sobre x^2, FLL de 2.o orden que
    quita un Doppler de 40 kHz con rampa de 400 Hz/s, filtro adaptado, Gardner y
    Costas. 250 000 simbolos de 25 ksps: la BER medida debe ser consistente
    (IC 99.9 %) con la teorica degradada <= 0.3 dB (perdida de implementacion) y
    no mejor que la teorica ideal."""
    rs, fs = 25e3, 500e3
    cn0 = ebn0 + 10 * math.log10(rs)
    b = Beacon(fs, 'bpsk', tasa_simbolos=rs, semilla=11)
    rx = ReceptorBPSK(fs, rs)
    n = int(fs * 0.05)
    fd0, tasa = 40e3, 400.0
    err_f = []
    for i in range(200):
        fa = fd0 + tasa * i * 0.05
        fb = fa + tasa * 0.05
        x = b.bloque(n, fa, fb, cn0)
        if i == 0:
            a = adquirir_fft(x[:int(0.03 * fs)], fs, (-60e3, 60e3), cuadrado=True)
            assert a['detectado']
            rx.iniciar(a['f_hz'])
        r = rx.procesar(x)
        if i >= 20:
            tm = (np.arange(len(r['f_hz'])) * rx.etapa1.n_ep + 0.5 * (rx.etapa1.n_ep - 1)) / n
            err_f.append(r['f_hz'] - (fa + (fb - fa) * tm))
    res = alinear_ber(b.bits_tx(), rx.bits_rx(), descartar=2000)
    lo, hi = intervalo_wilson(res['errores'], res['n'], Z999)
    assert res['n'] > 240_000
    assert hi >= ber_bpsk_teorica(ebn0)                      # no mejor que la teoria
    assert lo <= ber_bpsk_teorica(ebn0 - perdida_max_db)     # <= 0.3 dB de perdida
    # el FLL deja un Doppler residual de pocos Hz, bien dentro del Costas
    assert np.sqrt(np.mean(np.concatenate(err_f) ** 2)) < 12.0
    m = rx.evaluar()
    assert m['metrica'] > 0.9
    # C/N0 medido con M2M4 sobre los simbolos
    assert m['cn0_dbhz'] == pytest.approx(cn0, abs=1.0)
