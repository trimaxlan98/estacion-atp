"""El generador de IQ: pulso RRC, potencia/C/N0, Doppler, oscilador y semilla."""
import math

import numpy as np
import pytest

from atp_ros import model
from atp_sdr import senal
from atp_sdr.receptor import cn0_espectral


def test_rrc_normalizacion_y_nyquist():
    """sum(h^2) = sps y el par RRC*RRC es un coseno alzado: ISI nula en los
    instantes de simbolo (Proakis cap. 9.2; el truncamiento a 8 simbolos deja
    < 1 % de residuo)."""
    sps = 16
    h = senal.rrc(0.35, 8, sps)
    assert np.sum(h * h) == pytest.approx(sps, rel=1e-12)
    rc = np.convolve(h, h) / sps
    c = len(rc) // 2
    assert rc[c] == pytest.approx(1.0, abs=1e-12)
    isi = rc[c + sps * np.arange(1, 8)]
    assert np.max(np.abs(isi)) < 0.015      # truncado a 8 simbolos: ~1 %


@pytest.mark.parametrize('modo', ['portadora', 'bpsk', 'qpsk'])
def test_potencia_y_c_n0(modo):
    """Piso de ruido fijo (varianza 1) y amplitud por C/N0: A^2 = C/N0 / fs. La
    potencia de la senal limpia es C = A^2 (envolvente de potencia media 1) y el
    C/N0 que lee el estimador espectral (independiente del generador: usa solo
    el espectro y la potencia total) coincide con el pedido."""
    fs, cn0 = 300e3, 70.0
    b = senal.Beacon(fs, modo, tasa_simbolos=12.5e3, semilla=4)
    c = 10 ** (cn0 / 10)
    pot = [np.mean(np.abs(b.bloque(15000, 20e3, 20e3, cn0, ruido=False)) ** 2)
           for _ in range(40)]
    assert np.mean(pot) == pytest.approx(c / fs, rel=2e-3)
    est = [cn0_espectral(b.bloque(15000, 20e3, 20e3, cn0), fs, 20e3)[0]
           for _ in range(40)]
    assert np.mean(est) == pytest.approx(cn0, abs=0.1)
    # solo ruido: varianza 1 por muestra compleja
    ruido = b.bloque(60000, 0.0, 0.0, -200.0)
    assert np.var(ruido) == pytest.approx(1.0, rel=0.02)


def test_doppler_instantaneo_y_continuidad_de_fase():
    """La frecuencia instantanea sigue la recta fd_ini -> fd_fin de cada bloque
    y la fase es continua entre bloques (la integral del Doppler)."""
    fs = 100e3
    b = senal.Beacon(fs, 'portadora', semilla=1)
    n = 5000
    x1 = b.bloque(n, 10e3, 12e3, 80.0, ruido=False)
    x2 = b.bloque(n, 12e3, 11e3, 80.0, ruido=False)
    x = np.concatenate([x1, x2])
    f_inst = np.diff(np.unwrap(np.angle(x))) * fs / (2 * np.pi)
    esperada = np.concatenate([np.linspace(10e3, 12e3, n), np.linspace(12e3, 11e3, n)])[:-1]
    # la fase entre muestras k y k+1 corresponde a la frecuencia a mitad de paso
    assert np.max(np.abs(f_inst - esperada)) < 0.6        # < 1 resolucion de 2 kHz/5000
    # fase acumulada: integral del Doppler (trapecio) = 2 pi * (11000*0.05 + 11500*0.05 ...)
    dur = n / fs
    fase_fin = 2 * np.pi * ((10e3 + 12e3) / 2 * dur + (12e3 + 11e3) / 2 * dur)
    # fase de la muestra n_total (virtual) = fase de x[-1] + un paso
    paso = 2 * np.pi * (11e3 - 1e3 / n / 2) / fs
    assert (np.unwrap(np.angle(x))[-1] - np.unwrap(np.angle(x))[0]
            + paso) == pytest.approx(fase_fin, abs=2e-3)


def test_ruido_de_fase_lorentziano():
    """Ruido de fase de frecuencia blanca (caminata aleatoria): var(dphi) =
    2 pi lw dt. Rubiola 2009, cap. 1: un oscilador de ancho de linea lw da
    una linea lorentziana de ancho lw."""
    fs, lw = 100e3, 50.0
    b = senal.Beacon(fs, 'portadora', lw_lo_hz=lw, semilla=2)
    x = np.concatenate([b.bloque(50000, 0.0, 0.0, 80.0, ruido=False) for _ in range(10)])
    dphi = np.diff(np.unwrap(np.angle(x)))
    assert np.var(dphi) == pytest.approx(2 * np.pi * lw / fs, rel=0.03)
    # la varianza de la fase crece linealmente con el tiempo
    fase = np.unwrap(np.angle(x))
    for lag in (1000, 10000):
        v = np.var(fase[lag:] - fase[:-lag])
        assert v == pytest.approx(2 * np.pi * lw * lag / fs, rel=0.10)


def test_offset_y_deriva_del_oscilador():
    fs = 100e3
    b = senal.Beacon(fs, 'portadora', offset_lo_hz=2200.0, deriva_lo_hz_s=100.0, semilla=1)
    x = np.concatenate([b.bloque(10000, 5e3, 5e3, 80.0, ruido=False) for _ in range(5)])
    f_inst = np.diff(np.unwrap(np.angle(x))) * fs / (2 * np.pi)
    t = (np.arange(len(f_inst)) + 0.5) / fs
    esperada = 5e3 + 2200.0 + 100.0 * t
    assert np.max(np.abs(f_inst - esperada)) < 0.1


def test_determinismo_por_semilla():
    def corrida(sem, modo):
        b = senal.Beacon(300e3, modo, tasa_simbolos=12.5e3, lw_lo_hz=10.0, semilla=sem)
        return np.concatenate([b.bloque(15000, 1e3, 1.2e3, 60.0) for _ in range(3)]), b.bits_tx()
    for modo in ('portadora', 'bpsk'):
        a, ba = corrida(7, modo)
        a2, ba2 = corrida(7, modo)
        c, _ = corrida(8, modo)
        assert np.array_equal(a, a2) and np.array_equal(ba, ba2)
        assert not np.allclose(a, c)


def test_doppler_de_un_pase_en_s_y_ka():
    """Doppler y tasa de un pase del modelo. En el cenit la aceleracion de rango
    es v^2/h - v^2/(R+h) = (v^2/h) R/(R+h) (el termino de curvatura resta la
    aceleracion centripeta de la orbita). Este equipo detecto que
    `model.tasa_doppler_cenit` usaba solo v^2/h (Tierra plana, un 8 % de mas a
    550 km); el integrador la corrigio y ahora debe coincidir con el perfil.
    El Doppler escala con f0 (Ka/S = 30/2.2)."""
    pase = model.perfil_pase(550.0, 89.5, 5.0)
    t, fd_s, tasa_s = senal.doppler_de_perfil(pase, model.BANDAS['S'])
    _, fd_k, tasa_k = senal.doppler_de_perfil(pase, model.BANDAS['Ka'])
    assert np.max(np.abs(fd_k)) / np.max(np.abs(fd_s)) == pytest.approx(30e9 / 2.2e9, rel=1e-9)
    esfera = model.tasa_doppler_cenit(550.0, model.BANDAS['S'])
    plana = esfera * (model.R_TIERRA_KM + 550.0) / model.R_TIERRA_KM
    assert np.max(np.abs(tasa_s)) == pytest.approx(esfera, rel=0.01)
    assert np.max(np.abs(tasa_s)) < plana
    # el Doppler cruza cero en el cenit y se aleja (negativo) al final
    assert fd_s[0] > 0 > fd_s[-1]
    assert np.max(np.abs(fd_s)) < senal.ventana_doppler_hz(model.BANDAS['S'], 550.0)
    assert senal.ventana_doppler_hz(2.2e9, 550.0) == pytest.approx(58.4e3, rel=0.01)


def test_intervalo_de_wilson_y_q():
    assert senal.q_func(0.0) == pytest.approx(0.5)
    assert senal.q_func(1.0) == pytest.approx(0.158655, abs=1e-6)
    assert senal.ber_bpsk_teorica(9.6) == pytest.approx(1.0e-5, rel=0.1)   # tabla clasica
    lo, hi = senal.intervalo_wilson(0, 1000)
    assert lo == 0.0 and 0.003 < hi < 0.005
    lo, hi = senal.intervalo_wilson(50, 1000)
    assert lo < 0.05 < hi
