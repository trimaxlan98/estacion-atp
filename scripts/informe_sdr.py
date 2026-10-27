#!/usr/bin/env python3
"""Informe de la cadena SDR: cada cifra medida junto a su teoria.

Uso (dentro del contenedor, desde /ws):
    python3 scripts/informe_sdr.py [lazos umbral cn0 adq ber pase rend]

Reutiliza las corridas de src/atp_sdr/test y las imprime como tablas (las
pruebas solo dicen pasa/falla; esto deja los numeros para el README). Tarda
unos minutos: la mayor parte es el Monte Carlo del umbral y la BER.
"""
import math
import os
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(AQUI), 'src')
for p in ('atp_sdr', 'atp_ros', 'atp_sdr/test'):
    sys.path.insert(0, os.path.join(SRC, p))

import numpy as np  # noqa: E402

from atp_ros import model  # noqa: E402
from atp_sdr.bpsk import ReceptorBPSK, alinear_ber, pendiente_gardner  # noqa: E402
from atp_sdr.cadena import CadenaRF, simular_pase  # noqa: E402
from atp_sdr.receptor import (adquirir_fft, ancho_banda_discreto, cn0_espectral,  # noqa: E402
                              cn0_m2m4, cn0_nwpr, coef_lazo,
                              varianza_fase_por_oscilador)
from atp_sdr.senal import (Beacon, ber_bpsk_teorica, intervalo_wilson)  # noqa: E402
from util_sdr import corrida_pll  # noqa: E402
from test_bpsk import _ber_ideal  # noqa: E402
from test_cn0 import _prompts  # noqa: E402


def lazos():
    print('\n== PLL: varianza de fase contra B_L/(C/N0) (1 + 1/(2 T C/N0)), T = 1 ms, 60 s')
    print('orden  B_L nominal  B_L efectivo  C/N0    sigma medida  sigma teorica  cociente')
    for orden, bn, cn0 in ((2, 20.0, 40.0), (3, 20.0, 40.0), (3, 30.0, 37.0), (2, 10.0, 34.0)):
        r = corrida_pll(cn0, orden, bn, 1e-3, dur_s=60.0, semilla=orden * 100 + int(bn))
        e = r['err_fase'][500:]
        bl = ancho_banda_discreto(orden, bn, 1e-3)
        cl = 10 ** (cn0 / 10)
        teo = bl / cl * (1 + 1 / (2e-3 * cl))
        print(f'{orden:>5} {bn:>10.0f} Hz {bl:>10.2f} Hz {cn0:>5.0f}  '
              f'{math.degrees(np.std(e)):>9.2f} deg {math.degrees(math.sqrt(teo)):>10.2f} deg '
              f'{np.var(e) / teo:>9.3f}')
    print('\n== Rampa de frecuencia (B_L = 20 Hz): error de fase estacionario')
    for orden in (2, 3):
        wn = coef_lazo(orden, 20.0)[3]
        for a in (100.0, 500.0):
            r = corrida_pll(60.0, orden, 20.0, tasa_hz_s=a, dur_s=6.0)
            teo = 2 * math.pi * a / wn ** 2 if orden == 2 else 0.0
            print(f'  orden {orden} rampa {a:>5.0f} Hz/s: medido {np.mean(r["err_fase"][-1000:]):+.4f} rad, '
                  f'teorico {teo:.4f} rad, enganchado={bool(r["enganchado"][-1])}')
    wn3 = coef_lazo(3, 20.0)[3]
    for j in (100.0, 400.0):
        r = corrida_pll(60.0, 3, 20.0, jerk_hz_s2=j, dur_s=8.0)
        print(f'  orden 3 jerk {j:>4.0f} Hz/s2: medido {np.mean(r["err_fase"][-1000:]):.4f} rad, '
              f'teorico 2 pi j/wn^3 = {2 * math.pi * j / wn3 ** 3:.4f} rad')
    print('\n== Ruido de fase del oscilador (B_L = 20 Hz, 80 dB-Hz): var = 2 pi lw T sum g^2')
    for orden, lw in ((3, 1.0), (2, 2.0)):
        r = corrida_pll(80.0, orden, 20.0, 1e-3, dur_s=60.0, semilla=orden, lw_lo_hz=lw)
        teo = varianza_fase_por_oscilador(orden, 20.0, 1e-3, lw)
        print(f'  orden {orden} lw {lw} Hz: medida {np.var(r["err_fase"][500:]):.4f} rad^2, '
              f'teorica {teo:.4f} rad^2')


def umbral():
    print('\n== Umbral de perdida de enganche (3.er orden, B_L = 20 Hz, T = 1 ms, 3 s por corrida)')
    bl = ancho_banda_discreto(3, 20.0, 1e-3)
    print('C/N0   sigma teorica   metrica media  exp(-2 s^2)  deslizamientos  detector sin enganche al final')
    for cn0 in (20, 22, 24, 26, 28, 30, 33, 40):
        cl = 10 ** (cn0 / 10)
        sig2 = bl / cl * (1 + 1 / (2e-3 * cl))
        n, desl, unl, met = 20, 0, 0, []
        for sem in range(n):
            r = corrida_pll(float(cn0), 3, 20.0, 1e-3, dur_s=3.0, semilla=sem)
            ue = np.unwrap(r['err_fase'])
            desl += int(np.max(np.abs(ue - ue[100])) > 2 * math.pi * 0.9)
            unl += int(not r['enganchado'][-1])
            met.append(np.nanmean(r['metrica'][3:]))
        print(f'{cn0:>4}  {math.degrees(math.sqrt(sig2)):>9.1f} deg  {np.mean(met):>10.3f}  '
              f'{math.exp(-2 * sig2):>10.3f}  {desl:>9}/{n}  {unl:>14}/{n}')
    cl = 10 ** (np.arange(200, 400) / 100)
    print('(regla de Kaplan: sigma < 15 grados <=> C/N0 >= 27.4 dB-Hz con este lazo)')


def cn0():
    print('\n== Estimadores de C/N0 (T = 1 ms, PLL enganchado): sesgo +- desviacion en dB')
    print('C/N0 real   M2M4 (K=2000)      NWPR (K=2000)')
    for c in (28, 34, 40, 50, 60, 80):
        m = [cn0_m2m4(_prompts(float(c), s), 1e-3) for s in range(8)]
        w = [cn0_nwpr(_prompts(float(c), s), 1e-3, 20) for s in range(8)]
        print(f'{c:>8}   {np.nanmean(m) - c:+.2f} +- {np.nanstd(m):.2f}      {np.nanmean(w) - c:+.2f} +- {np.nanstd(w):.2f}')
    print('Estimador espectral (bloque de 50 ms, fs = 150 kHz):')
    fs = 150e3
    for c in (45, 50, 60, 80, 95):
        b = Beacon(fs, 'portadora', semilla=3)
        e = [cn0_espectral(b.bloque(7500, 30e3, 30e3, float(c)), fs, 30e3)[0] for _ in range(80)]
        print(f'  {c:>3} dB-Hz: {np.mean(e) - c:+.3f} +- {np.std(e):.3f} dB por bloque')


def adq():
    print('\n== Adquisicion FFT (fs 100 kHz, T 10 ms, C/N0 45 dB-Hz)')
    rng = np.random.default_rng(5)
    n, c = 1000, 45.0
    rho = 10 ** (c / 10) / 100e3
    crb = math.sqrt(6.0 * 100e3 ** 2 / ((2 * math.pi) ** 2 * rho * n * (n * n - 1)))
    errs = []
    for _ in range(400):
        f = float(rng.uniform(-40e3, 40e3))
        b = Beacon(100e3, 'portadora', semilla=int(rng.integers(1 << 30)))
        b.fase = float(rng.uniform(0, 2 * math.pi))
        a = adquirir_fft(b.bloque(n, f, f, c), 100e3, (-50e3, 50e3))
        errs.append(a['f_hz'] - f)
    print(f'  error rms {np.sqrt(np.mean(np.square(errs))):.2f} Hz, cota de Cramer-Rao (Rife-Boorstyn) {crb:.2f} Hz')
    fa = 0
    for _ in range(10000):
        x = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * math.sqrt(0.5)
        fa += int(adquirir_fft(x, 100e3, (-50e3, 50e3), pfa=1e-2)['detectado'])
    print(f'  falsas alarmas: {fa / 100:.2f} % (Pfa de diseno 1 %, 10 000 pruebas)')


def ber():
    print('\n== BER del receptor ideal (fase y reloj exactos) contra Q(sqrt(2 Eb/N0))')
    print('modo   Eb/N0   BER medida   IC 99.9 %                 teorica     simbolos')
    puntos = {0.0: 200_000, 2.0: 200_000, 4.0: 400_000, 6.0: 1_000_000, 8.0: 2_000_000}
    for modo in ('bpsk', 'qpsk'):
        for k, (e, ns) in enumerate(puntos.items()):
            err, n = _ber_ideal(modo, e, ns, semilla=40 + k)
            lo, hi = intervalo_wilson(err, n, 3.29)
            print(f'{modo:>5} {e:>6.1f}   {err / n:.3e}   ({lo:.3e}, {hi:.3e})   {ber_bpsk_teorica(e):.3e}   {n}')
    print('\n== Cadena completa (adquisicion x^2, FLL, filtro adaptado, Gardner, Costas), Doppler 40 kHz + 400 Hz/s')
    print('Eb/N0   BER medida   IC 99.9 %                 teorica     Eb/N0 efectivo   perdida')
    rs, fs = 25e3, 500e3
    for ebn0 in (2.0, 4.0, 6.0, 8.0):
        nb = 200 if ebn0 < 8 else 600
        cn0 = ebn0 + 10 * math.log10(rs)
        b = Beacon(fs, 'bpsk', tasa_simbolos=rs, semilla=11)
        rx = ReceptorBPSK(fs, rs)
        n = int(fs * 0.05)
        for i in range(nb):
            fa = 40e3 + 400.0 * i * 0.05
            x = b.bloque(n, fa, fa + 20.0, cn0)
            if i == 0:
                rx.iniciar(adquirir_fft(x[:int(0.03 * fs)], fs, (-60e3, 60e3), cuadrado=True)['f_hz'])
            rx.procesar(x)
        res = alinear_ber(b.bits_tx(), rx.bits_rx(), descartar=2000)
        lo, hi = intervalo_wilson(res['errores'], res['n'], 3.29)
        # Eb/N0 efectivo: el que da esa BER en la curva teorica
        ef = min(np.arange(-2, 14, 0.01), key=lambda v: abs(ber_bpsk_teorica(v) - res['ber']))
        print(f'{ebn0:>5.1f}   {res["ber"]:.3e}   ({lo:.3e}, {hi:.3e})   {ber_bpsk_teorica(ebn0):.3e}   '
              f'{ef:>8.2f} dB      {ebn0 - ef:+.2f} dB')
    print(f'\nPendiente del detector de Gardner: {pendiente_gardner():.3f} por simbolo (teoria de diseno 1.0)')


def pase():
    print('\n== Doppler estimado contra verdadero en un pase (60 grados de elevacion maxima, 550 km)')
    perfil = model.perfil_pase(550.0, 60.0, 5.0)
    tc = perfil['duracion'] / 2
    print('banda modo       orden B_L   tramo  Doppler rms  sesgo     max      C/N0 sesgo  C/N0 rms  enganchado')
    casos = [('S', 'portadora', {}, 40.0), ('Ka', 'portadora', {}, 15.0),
             ('Ka', 'portadora', {'orden': 2}, 6.0), ('S', 'bpsk', {}, 10.0),
             ('S', 'portadora', {'offset_lo_hz': 2200.0}, 12.0),
             ('S', 'portadora', {'lw_lo_hz': 0.5}, 12.0)]
    for banda, modo, kw, dur in casos:
        cad = CadenaRF(banda, modo, 5, **kw)
        simular_pase(cad, perfil, tc - dur / 2, tc + dur / 2, 0.02)
        r = cad.resumen()
        extra = ' '.join(f'{k}={v}' for k, v in kw.items())
        print(f'{banda:>4} {modo:>10} {cad.seg.orden:>5} {cad.seg.bn:>4.0f} {dur:>5.0f} s '
              f'{r["doppler_rms_hz"]:>9.3f} Hz {r["doppler_sesgo_hz"]:>+8.3f} {r["doppler_max_hz"]:>7.2f} '
              f'{r["cn0_sesgo_db"]:>+9.3f} dB {r["cn0_rms_db"]:>7.3f} dB {r["enganchado_pct"]:>8.1f} %  {extra}')


def rend():
    print('\n== Rendimiento (CPU de proceso, un solo hilo; la maquina de medida estaba cargada)')
    perfil = model.perfil_pase(550.0, 60.0, 5.0)
    tc = perfil['duracion'] / 2
    print('banda modo        fs        generador   receptor     MS/s receptor  MS/s total  margen sobre tiempo real')
    for banda, modo in (('UHF', 'portadora'), ('S', 'portadora'), ('X', 'portadora'),
                        ('Ka', 'portadora'), ('S', 'bpsk')):
        cad = CadenaRF(banda, modo, 5)
        t0 = time.process_time()
        simular_pase(cad, perfil, tc - 5, tc + 5, 0.02)
        cpu = time.process_time() - t0
        r = cad.resumen()
        n = r['bloques']
        print(f'{banda:>4} {modo:>10} {r["fs_hz"] / 1e3:>7.0f} kHz {r["ms_por_bloque_gen"]:>8.2f} ms '
              f'{r["ms_por_bloque_rx"]:>8.2f} ms {r["muestras_por_s_rx"] / 1e6:>10.2f}  '
              f'{r["muestras_por_s_total"] / 1e6:>10.2f}  x{r["muestras_por_s_total"] / r["fs_hz"]:.1f} '
              f'(CPU total {cpu / n * 1e3:.2f} ms/bloque de 50)')


SECCIONES = {'lazos': lazos, 'umbral': umbral, 'cn0': cn0, 'adq': adq, 'ber': ber,
             'pase': pase, 'rend': rend}

if __name__ == '__main__':
    for s in (sys.argv[1:] or list(SECCIONES)):
        SECCIONES[s]()
