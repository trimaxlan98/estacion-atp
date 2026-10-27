"""Receptor BPSK a nivel IQ: filtro adaptado, reloj de simbolo (Gardner) y Costas.

Solo numpy, sin ROS. Va DESPUES de la etapa de portadora de `receptor.py`
(que quita el Doppler): aqui la senal es BPSK con RRC y una frecuencia residual
de unos hertz. Fuentes: Gardner, IEEE Trans. Commun. 34(5), 1986; Proakis &
Salehi, Digital Communications 5.a ed., cap. 6 (Costas, reloj); Rice, Digital
Communications: A Discrete-Time Approach (2009), cap. 7-8 (ganancias del lazo
de reloj).
"""
import cmath
import math

import numpy as np

from atp_sdr.receptor import (DOS_PI, SeguidorPortadora, _reducir_pi, coef_lazo,
                              m2m4)
from atp_sdr.senal import rrc


# ---------------------------------------------------------------- BPSK: reloj y Costas

class LazoEscalar:
    """Lazo digital de 2.o/3.er orden a la tasa de simbolo (la misma ley de
    `SeguidorPortadora`): `paso(e)` devuelve la frecuencia (rad/s) a aplicar."""

    def __init__(self, orden, bn_hz, t_s):
        self.c0, self.c1, self.c2, self.wn = coef_lazo(orden, bn_hz)
        self.orden = orden
        self.t = float(t_s)
        self.i1 = 0.0
        self.i2 = 0.0

    def paso(self, e):
        if self.orden == 3:
            self.i2 += self.c0 * e * self.t
        self.i1 += (self.c1 * e + self.i2) * self.t
        return self.c2 * e + self.i1


class LazoCostas:
    """Lazo de Costas de BPSK a UNA muestra por simbolo (salida del filtro
    adaptado y del reloj de simbolo). Detector de fase de 2 cuadrantes
    e = atan(Q/I) (independiente de la amplitud y de los datos: Proakis &
    Salehi 2008, cap. 6; Kaplan & Hegarty cap. 8), lazo de 2.o o 3.er orden con
    B_L = `bn_rel` * Rs. Ambiguedad de 180 grados (el lazo puede enganchar a
    fase o fase + pi: los bits salen invertidos; se resuelve con un preambulo
    o con codificacion diferencial). El 3.er orden sigue una rampa de
    frecuencia residual; el 2.o la sigue con error de fase constante."""

    def __init__(self, rs_hz, bn_rel=0.01, orden=2):
        self.ts = 1.0 / float(rs_hz)
        self.lazo = LazoEscalar(orden, bn_rel * rs_hz, self.ts)
        self.fase = 0.0

    def procesar(self, s):
        """Corrige los simbolos complejos `s`. Devuelve (z, th): simbolos
        corregidos y fase del NCO por simbolo."""
        # bucle por simbolo en Python escalar: la ley del lazo va en linea (mismas
        # ecuaciones que LazoEscalar) y los resultados en listas, no en arreglos
        zs, ths = [], []
        fase, ts, lazo = self.fase, self.ts, self.lazo
        c0, c1, c2 = lazo.c0, lazo.c1, lazo.c2
        i1, i2, orden3 = lazo.i1, lazo.i2, lazo.orden == 3
        atan2, cexp, pi, mpi = math.atan2, cmath.exp, math.pi, -math.pi
        mitad = 0.5 * pi
        for v in s.tolist():
            c = v * cexp(-1j * fase)
            zs.append(c)
            ths.append(fase)
            e = atan2(c.imag, c.real)
            if e > mitad:
                e -= pi
            elif e <= -mitad:
                e += pi
            if orden3:
                i2 += c0 * e * ts
            i1 += (c1 * e + i2) * ts
            fase += (c2 * e + i1) * ts
        lazo.i1, lazo.i2 = i1, i2
        self.fase = fase % DOS_PI
        return np.array(zs, dtype=complex), np.array(ths)


class SincronizadorGardner:
    """Sincronizacion de simbolo con el detector de Gardner e interpolacion
    lineal (valida con sps >= 8: el error de interpolar un RRC sobremuestreado
    es < 1 %).

    e_k = Re{ conj(y_{k-1/2}) (y_k - y_{k-1}) }, normalizado por la potencia
    media de los simbolos. No depende de la fase de portadora (el producto
    conj() lo cancela) ni de los datos: puede ir ANTES del Costas (Gardner
    1986). Lazo PI de 2.o orden con B_L T_s = `bn_rel` y detector de
    pendiente `kd` (ver `pendiente_gardner`); amortiguamiento 1.

    Estado continuo entre bloques: llamar `procesar(y)` con cada bloque de
    salida del filtro adaptado, a `sps` muestras por simbolo nominales."""

    KD = 1.0       # medida con pendiente_gardner(): 1.00 +- 0.01 (sps 8..20)

    def __init__(self, sps, bn_rel=0.005, zeta=1.0):
        self.sps = float(sps)
        kd = self.KD
        th = bn_rel / (zeta + 1.0 / (4.0 * zeta))
        den = 1.0 + 2.0 * zeta * th + th * th
        self.kp = 4.0 * zeta * th / den / kd             # ganancia proporcional
        self.ki = 4.0 * th * th / den / kd               # ganancia integral
        self.buf = np.zeros(0, dtype=complex)
        self.pos = float(sps)         # posicion del proximo instante de simbolo
        self.integ = 0.0
        self.prev = 0j
        self.pot = 1.0
        self.n_sim = 0

    def procesar(self, y):
        """Devuelve los simbolos (complejos, 1 por simbolo) que caben en `y`."""
        self.buf = np.concatenate([self.buf, np.asarray(y)])
        b = self.buf.tolist()
        n = len(b)
        sps, pos = self.sps, self.pos
        kp, ki, integ = self.kp, self.ki, self.integ
        prev, pot = self.prev, self.pot
        out = []
        mitad = 0.5 * sps
        while pos + 1.0 < n - 1:
            i = int(pos)
            mu = pos - i
            yk = b[i] + mu * (b[i + 1] - b[i])
            pm = pos - mitad
            j = int(pm)
            nu = pm - j
            ym = b[j] + nu * (b[j + 1] - b[j])
            e = (ym.conjugate() * (yk - prev)).real
            pot += 0.01 * ((yk.real * yk.real + yk.imag * yk.imag) - pot)
            e /= (pot + 1e-30)
            integ += ki * e
            pos += sps - sps * (kp * e + integ)     # signo: ver pendiente_gardner
            prev = yk
            out.append(yk)
        # recorta lo ya usado, dejando 2 muestras de holgura para la mitad
        corte = max(0, int(pos - mitad) - 2)
        self.buf = self.buf[corte:]
        self.pos = pos - corte
        self.integ, self.prev, self.pot = integ, prev, pot
        self.n_sim += len(out)
        return np.array(out, dtype=complex)


def pendiente_gardner(beta=0.35, sps=16, n_sim=3000, semilla=7):
    """Pendiente Kd del detector de Gardner normalizado (por simbolo de
    desfase) para BPSK con RRC: mide la curva S desplazando el instante de
    muestreo y ajusta la recta en torno a 0. La curva S de Gardner con RRC y
    beta = 0.35 tiene su cero en el centro del ojo y pendiente POSITIVA para
    muestreo tardio; el lazo corrige restando (ver `SincronizadorGardner`)."""
    rng = np.random.default_rng(semilla)
    h = rrc(beta, 8, sps)
    sim = 1.0 - 2.0 * rng.integers(0, 2, n_sim)
    up = np.zeros(n_sim * sps)
    up[::sps] = sim
    y = np.convolve(np.convolve(up, h), h) / sps            # coseno alzado
    ret = len(h) - 1
    taus = np.array([-0.1, -0.05, 0.0, 0.05, 0.1])
    medio = []
    for tau in taus:
        e = 0.0
        pot = 0.0
        for k in range(20, n_sim - 20):
            c = ret + k * sps + tau * sps
            def v(t):
                i = int(math.floor(t))
                m = t - i
                return y[i] + m * (y[i + 1] - y[i])
            yk, yp, ym = v(c), v(c - sps), v(c - sps / 2.0)
            e += ym * (yk - yp)
            pot += yk * yk
        medio.append(e / pot)
    return float(np.polyfit(taus, np.array(medio), 1)[0])


class ReceptorBPSK:
    """Cadena de recepcion BPSK sobre una portadora con Doppler:

    1. `etapa1`: SeguidorPortadora en modo 'fll' con discriminador de la
       senal ELEVADA AL CUADRADO (quita la modulacion BPSK) a epocas de
       `t_int_s`: sigue la frecuencia (Doppler y su rampa) y la quita; la
       fase queda libre (un FLL de 2.o orden no la engancha).
    2. Filtro adaptado RRC (estado continuo entre bloques).
    3. `SincronizadorGardner`: reloj de simbolo, antes del Costas.
    4. `LazoCostas`: engancha la fase residual y la ambiguedad de 180 grados.

    `procesar(x)` devuelve los simbolos corregidos del bloque. Los simbolos de
    todo el historial quedan en `self.z` para medir BER y C/N0 (`evaluar`).
    Condiciones: fs/Rs entero >= 8 y 2 f_max < fs/2 (la busqueda por
    cuadrado duplica la frecuencia)."""

    def __init__(self, fs_hz, tasa_simbolos, beta=0.35, span=8, t_int_s=1e-3,
                 bn_fll_hz=20.0, bn_costas_rel=0.01, orden_costas=2,
                 bn_gardner_rel=0.005):
        self.fs = float(fs_hz)
        self.sps = int(round(self.fs / float(tasa_simbolos)))
        self.rs = self.fs / self.sps
        self.etapa1 = SeguidorPortadora(fs_hz, t_int_s, orden=2, bn_hz=10.0,
                                        bn_fll_hz=bn_fll_hz, disc='cuadrado',
                                        modo='fll', asistencia='nunca')
        self.h_mf = rrc(beta, span, self.sps)[::-1] / self.sps
        self.cola = np.zeros(len(self.h_mf) - 1, dtype=complex)
        self.gardner = SincronizadorGardner(self.sps, bn_gardner_rel)
        self.costas = LazoCostas(self.rs, bn_costas_rel, orden_costas)
        self.z = np.zeros(0, dtype=complex)       # los ultimos `n_guardar` simbolos
        self.n_guardar = 2000
        self._bits = np.zeros(1 << 16, dtype=np.int8)   # decisiones duras de todo el historial
        self._n_bits = 0
        self.n_simbolos = 0

    @property
    def f_hz(self):
        return self.etapa1.f_hz

    def iniciar(self, f_hz):
        self.etapa1.iniciar(f_hz)

    def procesar(self, x):
        r = self.etapa1.procesar(x, rotada=True)
        rot = r['rot']
        ent = np.concatenate([self.cola, rot])
        y = np.convolve(ent, self.h_mf, 'valid')
        self.cola = ent[-(len(self.h_mf) - 1):]
        sim = self.gardner.procesar(y)
        z, _ = self.costas.procesar(sim)
        self.z = np.concatenate([self.z, z])[-self.n_guardar:]
        b = (z.real < 0).astype(np.int8)
        n = self._n_bits + len(b)
        if n > len(self._bits):
            self._bits = np.concatenate([self._bits, np.zeros(max(len(self._bits), n), dtype=np.int8)])
        self._bits[self._n_bits:n] = b
        self._n_bits = n
        self.n_simbolos += len(z)
        r['z'] = z
        return r

    def bits_rx(self):
        """Decisiones duras (bit 1 si Re z < 0) de todos los simbolos."""
        return self._bits[:self._n_bits]

    def evaluar(self, n_vent=500, ruido_simbolo=None):
        """Metrica de enganche del Costas y C/N0 sobre los ultimos `n_vent`
        simbolos: <I^2 - Q^2> / S compensado en ruido (como
        `SeguidorPortadora.evaluar`, con S = <|z|^2> - ruido_simbolo si se
        conoce, o la de M2M4) y C/N0 = (S/N) Rs por M2M4 (cada simbolo integra
        Ts = 1/Rs)."""
        z = self.z[-n_vent:]
        if len(z) < n_vent:
            return {'metrica': float('nan'), 'cn0_dbhz': float('nan'),
                    'amp': float('nan')}
        s, nz = m2m4(z)
        if ruido_simbolo is not None:
            s_det = float(np.mean(z.real ** 2 + z.imag ** 2)) - float(ruido_simbolo)
            s_det = s_det if s_det > 0.05 * float(ruido_simbolo) else 0.0
        else:
            s_det = s
        met = float(np.mean(z.real ** 2 - z.imag ** 2)) / s_det if s_det > 0 else 0.0
        cn0 = (10.0 * math.log10(s / nz * self.rs)
               if s > 0 and nz > 0 else float('nan'))
        return {'metrica': met, 'cn0_dbhz': cn0,
                'amp': math.sqrt(s) if s > 0 else float('nan')}


def alinear_ber(bits_tx, bits_rx, rango=(-64, 1500), n_sync=400, descartar=0):
    """BER de las decisiones duras `bits_rx` (BPSK: bit 0 <-> simbolo +1, es
    decir bit = 1 si Re z < 0) contra los bits transmitidos. El desfase
    d = (indice tx) - (indice rx) y la polaridad del Costas se buscan
    correlacionando `n_sync` bits tras `descartar` simbolos de convergencia con
    todos los d de `rango` (el retardo de los dos RRC son -8 simbolos; si el
    receptor arranco tarde, d > 0). Devuelve dict(ber, errores, n, desfase,
    invertido). Es un banco de pruebas: el receptor real resolveria la
    ambiguedad de 180 grados con una palabra de sincronia o codificacion
    diferencial."""
    rx = np.asarray(bits_rx, dtype=np.int8)
    tx = np.asarray(bits_tx, dtype=np.int8)
    mejor = (-1.0, 0, False)
    a = rx[descartar:descartar + n_sync]
    for d in range(int(rango[0]), int(rango[1]) + 1):
        i0 = descartar + d
        if i0 < 0 or i0 + len(a) > len(tx):
            continue
        c = float(np.mean(a == tx[i0:i0 + len(a)]))
        for inv, cc in ((False, c), (True, 1.0 - c)):
            if cc > mejor[0]:
                mejor = (cc, d, inv)
    _, d, inv = mejor
    n = min(len(rx) - descartar, len(tx) - (descartar + d))
    r = rx[descartar:descartar + n]
    t = tx[descartar + d:descartar + d + n]
    if inv:
        r = 1 - r
    err = int(np.sum(r != t))
    return {'ber': err / max(n, 1), 'errores': err, 'n': int(n),
            'desfase': int(d), 'invertido': bool(inv)}
