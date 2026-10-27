"""Seguimiento fino + grueso de un terminal optico: lazo lineal, perturbaciones
de plataforma y ruido del sensor de cuadrante (QPD).

Todo en el espacio del HAZ (urad de apuntamiento del haz, un eje; los dos
ejes son identicos e independientes). Estructura (maestro-esclavo, la clasica
de un terminal de lasercom):

    d (perturbacion de la plataforma) --+--> e = d - (y + theta_c) --> QPD (+ruido n) --> C(z) --> FSM P(z) --> y
                                                                                                      |
              theta_c = H_c(s) y   (la montura gruesa sigue la posicion del FSM y lo DESCARGA)  <-----+

    C(z) = Ki Ts / (1 - z^-1)            integrador (el FSM tiene resonancia, no hace falta mas)
    P(s) = wn^2 / (s^2 + 2 zeta wn s + wn^2)  FSM, retencion de orden cero y retardo de `retardo_muestras`
    H_c(s) = (w_d / s) wg^2/(s^2 + 2 zg wg s + wg^2)   cardan (o ADCS) que integra la posicion del FSM

Con G_f = C z^-D P y lazo L = G_f (1 + H_c):
    e = S d - T n,   S = 1/(1+L),  T = L/(1+L),   y = G_f (e + n).
Con H_c (un integrador) el lazo es de tipo 2: error nulo ante rampa, que es lo
que hace falta para la linea de vista que gira ~1-4 mrad/s en LEO.

El lazo es LINEAL e invariante: la respuesta temporal se calcula con
`lfilter` con estado continuo entre bloques, y el error cuadratico medio
tambien tiene forma analitica (integral de |S|^2 por la densidad de la
perturbacion): las pruebas contrastan ambas. La saturacion del FSM (carrera)
NO se simula en el lazo: se vigila `max |y|` y se declara perdida de enganche.

PERTURBACIONES (SUPUESTO de diseno; no hay una medicion publicada que las
fije): suma de procesos con forma de PSD tipica de una plataforma,
  * deriva lenta de actitud (ADCS): proceso de Ornstein-Uhlenbeck, esquina 0.05 Hz;
  * banda media 0.5-20 Hz (movimiento de paneles, deslizamiento, termico): resonancia a 5 Hz;
  * microvibracion de ruedas y modos estructurales: resonancias a 35, 133 y 330 Hz.
El nivel total antes del lazo es de decenas de urad rms, del orden del
semiangulo del haz (el motivo de que el FSM sea necesario). La linea de las
ruedas se ancla en el desbalance dinamico: a la frecuencia de giro la
perturbacion angular libre es U_d / I (U_d desbalance dinamico, kg m^2; I
inercia del satelite), ver `jitter_rueda_urad`.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import signal

from sat_isl.enlace import H_PLANCK, C_LUZ

_trapz = getattr(np, 'trapezoid', None) or np.trapz
K_QPD = 2.0 * math.sqrt(2.0 / math.pi)       # pendiente del error normalizado: 1.596 x/w


# ---------------------------------------------------------------- QPD

def error_cuadrante(x_urad, w_urad):
    """Senal normalizada (A+B-C-D)/(suma) de un QPD con mancha gaussiana de radio
    1/e^2 `w` desplazada `x` sobre el eje (el otro eje centrado): erf(sqrt(2) x/w).
    Pendiente en el origen K_QPD / w."""
    from scipy.special import erf
    return erf(math.sqrt(2.0) * np.asarray(x_urad, float) / w_urad)


def ruido_qpd_urad(p_rx_w, lam_m, fs_hz, w_urad, fraccion, nep_w_rthz, eta=0.8):
    """Error equivalente en angulo (urad rms POR MUESTRA) del QPD:
    sigma_e^2 = (NEP^2 B + 2 h nu P_q B / eta) / P_q^2 con B = fs/2 y P_q la
    potencia sobre el QPD (termico + de disparo del cociente normalizado), y
    NEA = sigma_e w / K_QPD. Crece al bajar la potencia recibida."""
    pq = max(p_rx_w * fraccion, 1e-18)
    B = 0.5 * fs_hz
    nu_h = H_PLANCK * C_LUZ / lam_m
    s2 = (nep_w_rthz ** 2 * B + 2.0 * nu_h * pq * B / eta) / pq ** 2
    return math.sqrt(s2) * w_urad / K_QPD


# ---------------------------------------------------------------- lazo

def _suma(a, b):
    """Suma de polinomios en z^-1 (relleno por el final)."""
    n = max(len(a), len(b))
    out = np.zeros(n)
    out[:len(a)] += a
    out[:len(b)] += b
    return out


def _zoh(num, den, Ts):
    nd, dd, _ = signal.cont2discrete((num, den), Ts, method='zoh')
    return np.atleast_1d(nd).ravel(), np.asarray(dd, float)


class LazoLineal:
    """Funciones de transferencia discretas del lazo (en z^-1) para un `Lazo`."""

    def __init__(self, lazo):
        self.lazo = lazo
        Ts = self.Ts = 1.0 / lazo.fs_hz
        wn = 2 * math.pi * lazo.fsm_fn_hz
        self.bp, self.ap = _zoh([wn * wn], [1.0, 2 * lazo.fsm_zeta * wn, wn * wn], Ts)
        wg = 2 * math.pi * lazo.cardan_fn_hz
        wd = 2 * math.pi * lazo.f_descarga_hz
        self.bh, self.ah = _zoh([wd * wg * wg], np.convolve([1.0, 0.0], [1.0, 2 * lazo.cardan_zeta * wg, wg * wg]), Ts)
        self.D = int(round(lazo.retardo_muestras))
        # L = Ki * L1 ; L1 sin la ganancia del integrador
        self.ac = np.array([1.0, -1.0])
        bc1 = np.array([Ts])                               # C = Ki * Ts / (1 - z^-1)
        self.bg1 = np.concatenate([np.zeros(self.D), np.convolve(bc1, self.bp)])
        self.ag = np.convolve(self.ac, self.ap)
        self.bL1 = np.convolve(self.bg1, _suma(self.ah, self.bh))
        self.aL = np.convolve(self.ag, self.ah)
        w_c = 2 * math.pi * lazo.f_cruce_hz
        self.Ki = 1.0 / abs(self._resp(self.bL1, self.aL, np.array([w_c]))[0])   # |L(f_c)| = 1
        self.bL = self.Ki * self.bL1
        self.bg = self.Ki * self.bg1
        den = _suma(self.aL, self.bL)
        self.bS, self.aS = self.aL, den                    # S = aL / (aL + bL)
        self.bT, self.aT = self.bL, den                    # T = bL / (aL + bL)

    def _resp(self, b, a, w_rad_s):
        z = np.exp(1j * np.asarray(w_rad_s) * self.Ts)
        k = np.arange(max(len(b), len(a)))
        zi = z[:, None] ** (-k[None, :])
        return (zi[:, :len(b)] @ b) / (zi[:, :len(a)] @ a)

    def lazo_abierto(self, f_hz):
        return self._resp(self.bL, self.aL, 2 * math.pi * np.asarray(f_hz))

    def S(self, f_hz):
        return self._resp(self.bS, self.aS, 2 * math.pi * np.asarray(f_hz))

    def T(self, f_hz):
        return self._resp(self.bT, self.aT, 2 * math.pi * np.asarray(f_hz))

    def G_fsm(self, f_hz):
        """G_f = C z^-D P: lo que el FSM hace por unidad de error medido."""
        return self._resp(self.bg, self.ag, 2 * math.pi * np.asarray(f_hz))

    def estable(self):
        r = np.roots(self.aS)
        return bool(np.all(np.abs(r) < 1.0))

    def margenes(self, n=40000):
        """Margenes del lazo abierto (rejilla log): dict con f_cruce_hz, pm_deg,
        f_fase180_hz, gm_db, ms (pico de |S|), f_bw_hz (-3 dB de T)."""
        f = np.logspace(0.0, math.log10(0.499 * self.lazo.fs_hz), n)
        L = self.lazo_abierto(f)
        mag = np.abs(L)
        # ultimo cruce de |L| = 1 de arriba abajo: el cruce de ganancia
        idx = np.where((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
        i_c = int(idx[-1]) if len(idx) else None
        out = {'f_cruce_hz': None, 'pm_deg': None, 'f_fase180_hz': None, 'gm_db': None}
        if i_c is not None:
            ph = np.unwrap(np.angle(L[i_c:]))
            ph = ph - 2 * math.pi * math.ceil(ph[0] / (2 * math.pi))     # ph[0] en (-2 pi, 0]
            out['f_cruce_hz'] = float(f[i_c])
            out['pm_deg'] = float(180.0 + math.degrees(ph[0]))
            k180 = np.where((ph[:-1] > -math.pi) & (ph[1:] <= -math.pi))[0]
            if len(k180):
                k = i_c + int(k180[0])
                out['f_fase180_hz'] = float(f[k])
                out['gm_db'] = float(-20 * math.log10(mag[k]))
        out['ms'] = float(np.max(np.abs(1.0 / (1.0 + L))))
        Tm = np.abs(L / (1.0 + L))
        bajo = np.where(Tm < 10 ** (-3 / 20))[0]
        out['f_bw_hz'] = float(f[bajo[0]]) if len(bajo) else None      # primera caida bajo -3 dB
        return out


LAZO_CACHE = {}


def lazo_lineal(lazo):
    if lazo not in LAZO_CACHE:
        LAZO_CACHE[lazo] = LazoLineal(lazo)
    return LAZO_CACHE[lazo]


# ---------------------------------------------------------------- perturbaciones

@dataclass(frozen=True)
class Componente:
    nombre: str
    tipo: str          # 'modo': respuesta de un modo de 2.o orden a ruido blanco (pasa bajos resonante)
    f_hz: float
    zeta: float
    rms_urad: float


def jitter_rueda_urad(u_d_kg_m2, inercia_kg_m2):
    """Amplitud (urad) de la perturbacion angular LIBRE que produce el desbalance
    dinamico U_d de una rueda a su frecuencia de giro: el par es U_d Omega^2, la
    aceleracion angular U_d Omega^2 / I y el desplazamiento (a la frecuencia
    Omega) U_d / I, independiente de la velocidad."""
    return 1e6 * u_d_kg_m2 / inercia_kg_m2


def _componentes_defecto():
    # La linea de las ruedas: U_d = 3e-8 kg m^2 (0.3 g cm^2, SUPUESTO tipico de una
    # rueda pequena) sobre la inercia transversal del 3U de sat_adcs.
    from sat_adcs.fisica import cubesat_3u
    ixx = float(cubesat_3u().J[0, 0])
    rueda = jitter_rueda_urad(3e-8, ixx) / math.sqrt(2.0)       # amplitud -> rms
    return (Componente('deriva_adcs', 'modo', 0.05, 0.7, 400.0),
            Componente('media_banda', 'modo', 5.0, 0.3, 12.0),
            Componente('estructura_35', 'modo', 35.0, 0.03, 1.0),
            Componente('rueda_133', 'modo', 133.0, 0.02, rueda),
            Componente('estructura_330', 'modo', 330.0, 0.02, 0.4))


try:
    PLATAFORMA_DEFECTO = _componentes_defecto()
except Exception:                 # sat_adcs no instalado: valores fijos equivalentes
    PLATAFORMA_DEFECTO = (Componente('deriva_adcs', 'modo', 0.05, 0.7, 400.0),
                          Componente('media_banda', 'modo', 5.0, 0.3, 12.0),
                          Componente('estructura_35', 'modo', 35.0, 0.03, 1.0),
                          Componente('rueda_133', 'modo', 133.0, 0.02, 0.5),
                          Componente('estructura_330', 'modo', 330.0, 0.02, 0.4))


def _rejilla_hz(fs_hz, n=200_001):
    """Frecuencias (Hz) logaritmicas de 1e-4 Hz a Nyquist, para integrar PSD con lineas estrechas."""
    return np.logspace(-4.0, math.log10(0.5 * fs_hz), n)


class FormadorRuido:
    """Filtro discreto que da la perturbacion de un `Componente` a partir de ruido
    blanco unitario, normalizado para que el rms de salida sea el pedido."""

    def __init__(self, comp, fs_hz):
        Ts = 1.0 / fs_hz
        w = 2 * math.pi * comp.f_hz
        if comp.tipo == 'modo':
            num, den = [w * w], [1.0, 2 * comp.zeta * w, w * w]
        else:
            raise ValueError(comp.tipo)
        self.b, self.a = _zoh(np.atleast_1d(num), np.asarray(den, float), Ts)
        # norma H2 (varianza con ruido blanco unitario) = (1/pi) int_0^pi |H|^2 dw, en rejilla
        # LOGARITMICA (las lineas de 0.05 Hz y de 133 Hz con zeta 0.02 son estrechas)
        wgrid = 2.0 * math.pi * _rejilla_hz(fs_hz) / fs_hz
        _, H = signal.freqz(self.b, self.a, worN=wgrid)
        var1 = float(_trapz(np.abs(H) ** 2, wgrid) / math.pi)
        self.ganancia = comp.rms_urad / math.sqrt(var1)
        self.comp = comp
        self.fs = fs_hz

    def psd_unilateral(self, f_hz):
        """Densidad espectral unilateral (urad^2/Hz) de la salida."""
        w = 2 * math.pi * np.asarray(f_hz) / self.fs
        _, H = signal.freqz(self.b, self.a, worN=w)
        return 2.0 * self.ganancia ** 2 / self.fs * np.abs(H) ** 2


def psd_plataforma(f_hz, fs_hz, comps=PLATAFORMA_DEFECTO):
    return sum((FormadorRuido(c, fs_hz).psd_unilateral(f_hz) for c in comps),
               np.zeros_like(np.asarray(f_hz, float)))


def carrera_baja_frecuencia_urad(lazo, comps=PLATAFORMA_DEFECTO, n=1 << 16):
    """rms (urad) de la posicion del FSM debida a la parte LENTA de la
    perturbacion, en tiempo continuo: por debajo del cruce el lazo fino deja
    e ~ 0, asi que y + theta_c = d y theta_c = H_c y dan y = d / (1 + H_c).
    Sirve para decidir si la montura gruesa descarga lo bastante al FSM (la
    carrera `fsm_carrera_urad` debe superar ~4 veces este rms); funciona aun
    cuando la montura es tan lenta (ADCS, 0.016 Hz) que el lazo discreto a
    10 kHz esta mal condicionado numericamente."""
    f = np.logspace(-4, math.log10(0.1 * lazo.f_cruce_hz), n)
    s = 2j * math.pi * f
    wd = 2 * math.pi * lazo.f_descarga_hz
    wg = 2 * math.pi * lazo.cardan_fn_hz
    Hc = wd / s * wg * wg / (s * s + 2 * lazo.cardan_zeta * wg * s + wg * wg)
    phi = psd_plataforma(f, lazo.fs_hz, comps)
    integrando = phi * np.abs(1.0 / (1.0 + Hc)) ** 2
    return float(math.sqrt(_trapz(integrando, f)))


def rms_analitico(lazo, comps=PLATAFORMA_DEFECTO, nea_urad=0.0):
    """Valores rms (urad, por eje) calculados SIN simular, integrando en
    frecuencia (rejilla logaritmica, regla del trapecio): perturbacion sin lazo,
    error residual e = S d - T n, y posicion del FSM y = G_f (e + n)
    (= G_f S (d + n)). Las componentes y el ruido son independientes: las
    varianzas suman."""
    lin = lazo_lineal(lazo)
    f = _rejilla_hz(lazo.fs_hz)
    Sf, Tf, Gf = lin.S(f), lin.T(f), lin.G_fsm(f)
    phi_d = psd_plataforma(f, lazo.fs_hz, comps)
    phi_n = 2.0 * nea_urad ** 2 / lazo.fs_hz            # blanco, unilateral
    var_d = float(_trapz(phi_d, f))
    var_e_d = float(_trapz(phi_d * np.abs(Sf) ** 2, f))
    var_e_n = float(_trapz(phi_n * np.abs(Tf) ** 2, f)) if nea_urad > 0 else 0.0
    var_y = float(_trapz((phi_d + phi_n) * np.abs(Gf * Sf) ** 2, f))
    return {'d_rms': math.sqrt(var_d), 'e_rms_perturbacion': math.sqrt(var_e_d),
            'e_rms_ruido': math.sqrt(var_e_n), 'e_rms': math.sqrt(var_e_d + var_e_n),
            'y_rms': math.sqrt(var_y)}


# ---------------------------------------------------------------- simulacion

class Canal:
    """Un eje de un terminal: genera la perturbacion y corre el lazo con estado
    continuo entre bloques (`lfilter` con `zi`). El generador de numeros
    aleatorios es el que se le pase (los sorteos van siempre en el mismo
    orden: el resultado es reproducible)."""

    def __init__(self, lazo, comps=PLATAFORMA_DEFECTO):
        if lazo.f_descarga_hz < 0.1:
            raise ValueError('lazo discreto mal condicionado con una montura de < 0.1 Hz: '
                             'usar carrera_baja_frecuencia_urad (analisis continuo)')
        self.lin = lazo_lineal(lazo)
        self.formadores = [FormadorRuido(c, lazo.fs_hz) for c in comps]
        self.zi_f = [np.zeros(max(len(f.a), len(f.b)) - 1) for f in self.formadores]
        self.zi_S = np.zeros(max(len(self.lin.bS), len(self.lin.aS)) - 1)
        self.zi_T = np.zeros_like(self.zi_S)
        self.zi_G = np.zeros(max(len(self.lin.bg), len(self.lin.ag)) - 1)
        self.d_cte = 0.0          # escalon (offset del arranque): el lazo lo decae

    def bloque(self, n, rng, nea_urad):
        """Simula `n` muestras: devuelve (e, y, d) en urad."""
        d = np.zeros(n)
        for k, f in enumerate(self.formadores):
            w = rng.standard_normal(n)
            x, self.zi_f[k] = signal.lfilter(f.b, f.a, w * f.ganancia, zi=self.zi_f[k])
            d += x
        d += self.d_cte
        ruido = nea_urad * rng.standard_normal(n)
        eS, self.zi_S = signal.lfilter(self.lin.bS, self.lin.aS, d, zi=self.zi_S)
        eT, self.zi_T = signal.lfilter(self.lin.bT, self.lin.aT, ruido, zi=self.zi_T)
        e = eS - eT
        y, self.zi_G = signal.lfilter(self.lin.bg, self.lin.ag, e + ruido, zi=self.zi_G)
        return e, y, d
