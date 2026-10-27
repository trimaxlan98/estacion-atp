"""Senal de RF a nivel de muestras IQ: el beacon de un satelite LEO en banda base.

Solo numpy, sin ROS. Convenciones (todas las cifras del paquete salen de aqui):

* Banda base compleja a `fs_hz` muestras/s. El oscilador local del receptor
  esta sintonizado a la frecuencia NOMINAL f0 del beacon: lo que se ve en
  banda base es el Doppler (+ el desplazamiento del oscilador, si lo hay).
* Piso de ruido FIJO: ruido blanco gaussiano complejo de varianza 1 por muestra
  (N0 * fs = 1, "AGC apagado"). La amplitud de la portadora sale del C/N0:
  A^2 = (C/N0) / fs  ->  C/N0 = A^2 fs / sigma^2. Asi la amplitud IQ medida es
  una medida de la potencia recibida (la usa el autotrack por amplitud).
* Para BPSK/QPSK con raiz de coseno alzado (RRC), h se normaliza a sum(h^2)=sps:
  potencia media de la senal = A^2 y, tras el filtro adaptado h/sps, el simbolo
  sale con amplitud A y ruido de varianza sigma^2/sps por muestra compleja:
  Es/N0 = A^2 sps / sigma^2 = (C/N0)/Rs.

Fuentes: Proakis & Salehi, Digital Communications 5.a ed., cap. 4 y 9 (RRC,
BER de BPSK); Kaplan & Hegarty, Understanding GPS/GNSS 3.a ed., cap. 5 y 8
(modelo de la portadora y del ruido); Rubiola, Phase Noise and Frequency
Stability in Oscillators (2009), cap. 1 (ruido de frecuencia blanco).
"""
import cmath
import math

import numpy as np

C_LUZ_M_S = 299792458.0


# ---------------------------------------------------------------- pulso RRC

def rrc(beta=0.35, span=8, sps=8):
    """Filtro de raiz de coseno alzado, normalizado a sum(h^2) = sps.

    `span` simbolos a cada lado del pico en total (longitud span*sps + 1).
    Valido para 0 < beta <= 1, sps >= 2. El par transmisor/receptor es un
    coseno alzado (cumple Nyquist: ISI nula en los instantes de simbolo,
    salvo el truncamiento a `span` simbolos)."""
    n = int(span * sps)
    t = (np.arange(n + 1) - n / 2.0) / float(sps)       # en periodos de simbolo
    h = np.empty_like(t)
    b = float(beta)
    for i, x in enumerate(t):
        if abs(x) < 1e-12:
            h[i] = 1.0 - b + 4.0 * b / math.pi
        elif b > 0 and abs(abs(x) - 1.0 / (4.0 * b)) < 1e-9:
            h[i] = (b / math.sqrt(2.0)) * (
                (1 + 2 / math.pi) * math.sin(math.pi / (4 * b))
                + (1 - 2 / math.pi) * math.cos(math.pi / (4 * b)))
        else:
            h[i] = (math.sin(math.pi * x * (1 - b))
                    + 4 * b * x * math.cos(math.pi * x * (1 + b))) / (
                math.pi * x * (1 - (4 * b * x) ** 2))
    return h * math.sqrt(sps / np.sum(h * h))


# ---------------------------------------------------------------- Doppler

def doppler_de_vr(vr_km_s, f0_hz):
    """fd = -vr/c f0, vr > 0 si el satelite se aleja (no relativista; el error
    relativo es del orden de v/c = 2.5e-5)."""
    return -np.asarray(vr_km_s, dtype=float) * 1000.0 / C_LUZ_M_S * float(f0_hz)


def doppler_de_perfil(perfil, f0_hz):
    """Doppler y tasa Doppler de un pase. `perfil` es el diccionario de
    `model.perfil_pase` o `orbit.perfil_tle` (claves t, vr). Devuelve
    (t, fd, tasa) con fd en Hz y tasa = d fd / dt en Hz/s. Valido mientras el
    perfil este muestreado mas fino que la dinamica del Doppler (0.5 s sobra:
    la tasa de LEO cambia en decenas de segundos)."""
    t = np.asarray(perfil['t'], dtype=float)
    fd = doppler_de_vr(perfil['vr'], f0_hz)
    return t, fd, np.gradient(fd, t)


def ventana_doppler_hz(f0_hz, h_km=550.0, margen_hz=0.0):
    """Semiancho (Hz) de la ventana de busqueda de Doppler que la estacion
    conoce a priori: la velocidad radial no supera la circular de la orbita,
    |vr| <= sqrt(mu/(R+h)). Sirve para dimensionar fs y la busqueda FFT;
    `margen_hz` cubre el error de oscilador. Para h = 550 km: 7.6 km/s."""
    mu, r_t = 398600.0, 6371.0
    v = math.sqrt(mu / (r_t + float(h_km))) * 1000.0
    return 1.05 * v / C_LUZ_M_S * float(f0_hz) + float(margen_hz)


# ---------------------------------------------------------------- el beacon

class Beacon:
    """Generador de bloques IQ de un beacon con Doppler, ruido y oscilador.

    modo: 'portadora' (tono puro), 'bpsk' o 'qpsk' (simbolos aleatorios de una
    secuencia PN con semilla, conformados con RRC). Los bloques se encadenan con
    fase continua: el estado (fase, cola del filtro, ruido de fase) pasa de un
    bloque al siguiente, asi que la senal de un pase entero es una sola
    realizacion continua.

    Parametros:
      fs_hz            tasa de muestreo (Hz)
      tasa_simbolos    Rs (simbolos/s) para bpsk/qpsk; fs/Rs debe ser entero
      beta, span       RRC
      offset_lo_hz     desplazamiento constante del oscilador local (Hz): lo que
                       el receptor ve de mas respecto a f0 (p. ej. 1 ppm de un
                       TCXO a 2.2 GHz = 2.2 kHz)
      deriva_lo_hz_s   deriva lineal del oscilador (Hz/s)
      lw_lo_hz         ancho de linea lorentziano del oscilador (Hz): ruido de
                       fase de caminata aleatoria; var(dphi) = 2 pi lw dt
      semilla          entero; mismo valor, misma senal bit a bit

    Cada llamada a `bloque` deja en `fase_verdad` la fase de portadora de
    cada muestra (rad, continua, con el ruido de fase del oscilador), para que
    las pruebas midan el error de fase del receptor contra la verdad."""

    def __init__(self, fs_hz, modo='portadora', tasa_simbolos=25e3, beta=0.35,
                 span=8, offset_lo_hz=0.0, deriva_lo_hz_s=0.0, lw_lo_hz=0.0,
                 semilla=1):
        if modo not in ('portadora', 'bpsk', 'qpsk'):
            raise ValueError(f'modo desconocido: {modo}')
        self.fs = float(fs_hz)
        self.modo = modo
        self.offset_lo = float(offset_lo_hz)
        self.deriva_lo = float(deriva_lo_hz_s)
        self.lw = float(lw_lo_hz)
        ss = np.random.SeedSequence(int(semilla))
        s_ruido, s_bits, s_fase = ss.spawn(3)
        self.rng_ruido = np.random.default_rng(s_ruido)
        self.rng_bits = np.random.default_rng(s_bits)
        self.rng_fase = np.random.default_rng(s_fase)
        self.fase = 0.0              # fase de portadora acumulada (rad, mod 2 pi)
        self.fase_ruido = 0.0        # camino aleatorio del oscilador (rad)
        self.t = 0.0                 # tiempo acumulado (s) para la deriva
        self._fase_par = (0.0, 0.0, 0.0, 0, None)
        self._bits = np.zeros(1 << 16, dtype=np.int8)   # bits transmitidos (0/1)
        self._n_bits = 0
        if modo != 'portadora':
            self.sps = int(round(self.fs / float(tasa_simbolos)))
            if abs(self.sps * float(tasa_simbolos) - self.fs) > 1e-6 * self.fs:
                raise ValueError('fs/Rs debe ser entero')
            if self.sps < 2:
                raise ValueError('hacen falta al menos 2 muestras por simbolo')
            self.rs = self.fs / self.sps
            self.h = rrc(beta, span, self.sps)
            self.retardo = (len(self.h) - 1) // 2    # retardo de grupo (muestras)
            self._cola = np.zeros(len(self.h) - 1, dtype=complex)

    def _forma(self, n):
        """Envolvente compleja de potencia media 1 de `n` muestras."""
        if self.modo == 'portadora':
            return np.ones(n)
        if n % self.sps:
            raise ValueError('n debe ser multiplo de fs/Rs')
        ns = n // self.sps
        if self.modo == 'bpsk':
            b = self.rng_bits.integers(0, 2, ns)
            sim = (1.0 - 2.0 * b).astype(complex)
        else:
            b = self.rng_bits.integers(0, 2, 2 * ns)
            sim = ((1.0 - 2.0 * b[0::2]) + 1j * (1.0 - 2.0 * b[1::2])) / math.sqrt(2.0)
        self._guardar_bits(b.astype(np.int8))
        up = np.zeros(n, dtype=complex)
        up[::self.sps] = sim
        y = np.convolve(up.real, self.h) + 1j * np.convolve(up.imag, self.h) \
            if self.modo == 'qpsk' else np.convolve(up.real, self.h).astype(complex)
        y[:len(self._cola)] += self._cola
        self._cola = y[n:].copy()
        return y[:n]

    def _guardar_bits(self, b):
        n = self._n_bits + len(b)
        if n > len(self._bits):
            self._bits = np.concatenate(
                [self._bits, np.zeros(max(len(self._bits), n), dtype=np.int8)])
        self._bits[self._n_bits:n] = b
        self._n_bits = n

    def bits_tx(self):
        """Todos los bits transmitidos hasta ahora (BPSK: 1 por simbolo; QPSK: 2),
        como vista del buffer (sin copia: crece por duplicacion)."""
        return self._bits[:self._n_bits]

    def bloque(self, n, fd_ini_hz, fd_fin_hz, cn0_dbhz, ruido=True):
        """`n` muestras IQ. El Doppler varia LINEALMENTE de `fd_ini_hz` (primera
        muestra) a `fd_fin_hz` (instante n/fs): es la aproximacion de tasa
        constante dentro del bloque (valida si la tasa cambia poco en 50 ms;
        el error de fase es <= jerk dt^3/24). C/N0 en dB-Hz; `ruido=False`
        devuelve la senal limpia (para medir)."""
        n = int(n)
        dur = n / self.fs
        k = (fd_fin_hz - fd_ini_hz) / dur               # tasa Doppler (Hz/s)
        f_ini = fd_ini_hz + self.offset_lo + self.deriva_lo * self.t
        k_tot = k + self.deriva_lo
        dos_pi = 2.0 * math.pi
        # fase(tau) = fase0 + 2 pi (f_ini tau + k_tot tau^2 / 2) + ruido de fase
        pn = None
        if self.lw > 0.0:
            inc = self.rng_fase.normal(0.0, math.sqrt(dos_pi * self.lw / self.fs), n)
            pn = self.fase_ruido + np.cumsum(inc)
            self.fase_ruido = float(pn[-1])
        self._fase_par = (self.fase, f_ini, k_tot, n, pn)
        # la portadora por recurrencia (dos cumprod: el termino lineal y el
        # cuadratico), en vez de una exponencial compleja por muestra: 2-3 veces
        # mas rapido; el error de fase crece ~ n^2 eps/2 ~ 1e-6 rad en 100k muestras
        w1 = dos_pi * f_ini / self.fs
        q = dos_pi * k_tot / (self.fs * self.fs)
        a = np.full(n, cmath.exp(1j * w1), dtype=complex)
        a[0] = cmath.exp(1j * self.fase)
        np.cumprod(a, out=a)
        if q != 0.0:
            c = np.full(max(n - 1, 0), cmath.exp(1j * q), dtype=complex)
            if n > 1:
                c[0] = cmath.exp(0.5j * q)
            np.cumprod(c, out=c)                    # c_m = exp(j q (2m+1) / 2)
            b = np.ones(n, dtype=complex)
            np.cumprod(c, out=b[1:])                # b_m = exp(j q m^2 / 2)
            a *= b
        if pn is not None:
            a *= np.exp(1j * pn)
        self.fase = (self.fase + dos_pi * (f_ini * dur + 0.5 * k_tot * dur * dur)) % dos_pi
        self.t += dur
        amp = math.sqrt(10.0 ** (cn0_dbhz / 10.0) / self.fs)
        forma = self._forma(n)
        x = a * forma if self.modo != 'portadora' else a
        x *= amp
        if ruido:
            z = self.rng_ruido.standard_normal(2 * n).view(np.complex128)
            x += z * math.sqrt(0.5)
        return x

    @property
    def fase_verdad(self):
        """Fase de portadora (rad, con el ruido de fase del oscilador) de cada
        muestra del ULTIMO bloque; sirve para medir el error del receptor."""
        fase0, f_ini, k_tot, n, pn = self._fase_par
        tau = np.arange(n) / self.fs
        arg = fase0 + 2.0 * math.pi * (f_ini * tau + 0.5 * k_tot * tau * tau)
        return arg if pn is None else arg + pn


# ---------------------------------------------------------------- medidas de apoyo

def q_func(x):
    """Q(x) = P(N(0,1) > x)."""
    return 0.5 * math.erfc(float(x) / math.sqrt(2.0))


def ber_bpsk_teorica(ebn0_db):
    """BER de BPSK/QPSK coherente en AWGN con filtro adaptado: Q(sqrt(2 Eb/N0)).
    Proakis & Salehi, ec. 4.3-5 / 8.1-?. Receptor ideal (fase y reloj perfectos)."""
    return q_func(math.sqrt(2.0 * 10.0 ** (float(ebn0_db) / 10.0)))


def intervalo_wilson(errores, n, z=1.96):
    """Intervalo de confianza de Wilson para una proporcion binomial
    (errores de n); z = 1.96 da 95 %, 3.29 da 99.9 %. Valido para n >= 10 y
    funciona bien con 0 errores. Agresti & Coull (1998)."""
    n = float(n)
    p = errores / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / d
    m = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - m), min(1.0, c + m)
