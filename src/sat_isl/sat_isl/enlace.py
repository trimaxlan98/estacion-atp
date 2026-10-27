"""Enlace optico entre satelites: haz gaussiano truncado, ganancia, perdida por
apuntamiento y detectores (APD de deteccion directa y BPSK homodino coherente).

Solo numpy/scipy. Cada funcion dice bajo que condicion vale su cifra.

HAZ. El transmisor ilumina una apertura circular de diametro D con un haz
gaussiano de radio 1/e^2 w en la apertura, truncado por ella; sea
gamma = (D/2)/w. La ganancia sobre el eje respecto de un radiador isotropico
de la potencia que SALE por la apertura se integra en forma cerrada
(campo E = exp(-r^2/w^2) para r < a; ganancia = 4 pi/lam^2 |int E dA|^2 /
int |E|^2 dA):

    G_tx = (pi D / lam)^2 * (2/gamma^2) tanh(gamma^2 / 2)           (1)

(gamma -> 0 es la apertura uniforme, (pi D/lam)^2). La fraccion de la
potencia del laser que cruza la apertura es 1 - exp(-2 gamma^2). El producto
de ambas se maximiza en gamma ~ 1.12 (Klein y Degnan, Appl. Opt. 13, 1974),
que la prueba reproduce. La anchura equivalente del haz se DEFINE con la
ganancia: theta_h = sqrt(8 / G_tx) es el semiangulo 1/e^2 de un haz gaussiano
sin truncar con esa ganancia (G = 8/theta_h^2); en gamma = 1.12 vale
0.956 lam/D (angulo completo 1.91 lam/D). La prueba contrasta esa gaussiana con
el patron de campo lejano REAL del haz truncado (integral de Bessel): dentro
del 4 % en intensidad hasta el 10 % del pico. La perdida por apuntamiento es

    L(e) = exp(-2 e^2 / theta_h^2).                                  (2)

Con error de apuntamiento gaussiano de sigma por eje (Rayleigh), la perdida
MEDIA es 1/(1 + 4 sigma^2/theta_h^2) (media de (2); la prueba lo verifica por
Monte Carlo).

ECUACION DE ENLACE (vacio, sin turbulencia):
    P_r = P_t eta_t * frac(gamma) * G_tx L_tx * (lam / 4 pi R)^2 * A_r-gain * eta_r L_r
con ganancia de recepcion G_r = (pi D_r / lam)^2 (apertura sin obstruccion;
la obstruccion central y el acoplo a fibra van en eta_r). Reproduce la
cascada del curso 18 (10 cm @ 1550 nm, 1 W, 5000 km -> -30 dBm).

DETECTORES. `Apd`: OOK con APD y TIA (modelo de Gaussiana de ruido, Agrawal
cap. 4 / Personick); `Homodino`: BPSK homodino con receptor balanceado,
BER = 1/2 erfc(sqrt(2 Np / L_impl)) con Np = fotones por bit y L_impl la
penalizacion de implementacion sobre el limite cuantico (en L = 1: 9 fotones
por bit dan BER 1e-9, la cifra clasica de Gagliardi y Karp / Kazovsky).
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy.special import erfc

C_LUZ = 299792458.0          # m/s (exacta)
H_PLANCK = 6.62607015e-34    # J s (exacta, SI 2019)
Q_ELECTRON = 1.602176634e-19  # C (exacta)
K_BOLTZ = 1.380649e-23       # J/K (exacta)


def db(x):
    return 10.0 * np.log10(x)


def dbm(p_w):
    return 10.0 * np.log10(np.maximum(p_w, 1e-300) / 1e-3)


def w_de_dbm(p_dbm):
    return 1e-3 * 10.0 ** (np.asarray(p_dbm) / 10.0)


def energia_foton(lam_m):
    return H_PLANCK * C_LUZ / lam_m


# ---------------------------------------------------------------- haz

def ganancia_apertura(D_m, lam_m):
    """(pi D / lam)^2: apertura uniforme ideal (cota de la ganancia)."""
    return (math.pi * D_m / lam_m) ** 2


def eta_gauss(gamma):
    """Factor de la ec. (1): (2/gamma^2) tanh(gamma^2/2)."""
    g2 = gamma * gamma
    return 1.0 if g2 < 1e-9 else (2.0 / g2) * math.tanh(0.5 * g2)


def fraccion_apertura(gamma):
    """Fraccion de la potencia del haz que cruza la apertura: 1 - exp(-2 gamma^2)."""
    return 1.0 - math.exp(-2.0 * gamma * gamma)


def gamma_optimo():
    """gamma que maximiza fraccion_apertura * eta_gauss (ganancia efectiva
    por vatio de laser): ~ 1.12 (Klein y Degnan 1974)."""
    lo, hi = 0.3, 3.0
    f = lambda g: fraccion_apertura(g) * eta_gauss(g)  # noqa: E731
    for _ in range(100):                      # seccion aurea
        a = lo + 0.381966 * (hi - lo)
        b = hi - 0.381966 * (hi - lo)
        if f(a) < f(b):
            lo = a
        else:
            hi = b
    return 0.5 * (lo + hi)


GAMMA_KLEIN = 1.12            # redondeo de gamma_optimo() usado por defecto


def ganancia_tx(D_m, lam_m, gamma=GAMMA_KLEIN):
    """Ec. (1), lineal, respecto de la potencia que sale por la apertura."""
    return ganancia_apertura(D_m, lam_m) * eta_gauss(gamma)


def semiangulo_equiv_rad(G):
    """theta_h = sqrt(8/G): semiangulo 1/e^2 de la gaussiana de ganancia G."""
    return math.sqrt(8.0 / G)


def semiangulo_haz_rad(D_m, lam_m, gamma=GAMMA_KLEIN):
    return semiangulo_equiv_rad(ganancia_tx(D_m, lam_m, gamma))


def perdida_apuntamiento(err_rad, theta_h_rad):
    """Ec. (2) (lineal)."""
    e = np.asarray(err_rad, float)
    return np.exp(-2.0 * e * e / (theta_h_rad * theta_h_rad))


def perdida_apuntamiento_media(sigma_rad, theta_h_rad):
    """Media de (2) con error gaussiano de sigma por eje (2 ejes)."""
    return 1.0 / (1.0 + 4.0 * sigma_rad ** 2 / theta_h_rad ** 2)


def fspl_lineal(rango_m, lam_m):
    """(lam / 4 pi R)^2."""
    return (lam_m / (4.0 * math.pi * rango_m)) ** 2


def potencia_recibida_w(p_tx_w, lam_m, D_tx_m, D_rx_m, rango_m, eta_tx=1.0, eta_rx=1.0,
                        gamma=GAMMA_KLEIN, l_tx=1.0, l_rx=1.0, perdidas_db=0.0):
    """P_r (W) de la ecuacion de enlace; `l_tx`, `l_rx` son las perdidas por
    apuntamiento LINEALES de cada extremo (ec. 2) y `perdidas_db` otras."""
    g_t = ganancia_tx(D_tx_m, lam_m, gamma)
    g_r = ganancia_apertura(D_rx_m, lam_m)
    return (p_tx_w * eta_tx * fraccion_apertura(gamma) * g_t * l_tx * fspl_lineal(rango_m, lam_m)
            * g_r * eta_rx * l_rx * 10.0 ** (-perdidas_db / 10.0))


# ---------------------------------------------------------------- detectores

def _q(x):
    return 0.5 * erfc(np.asarray(x) / math.sqrt(2.0))


@dataclass(frozen=True)
class Apd:
    """OOK con fotodiodo de avalancha y amplificador de transimpedancia.

    Corriente de senal I1 = R M 2 P (potencia media P, OOK con extincion
    infinita: P1 = 2 P, P0 = 0); ruido cuadratico medio
    sigma^2 = 2 q (I_d + R P_k) M^2 F B + (i_n)^2 B, B = 0.7 tasa (ancho de
    banda de ruido del TIA), F = k M + (1 - k)(2 - 1/M) (McIntyre).
    BER = Q((I1 - I0)/(sigma1 + sigma0)) con Q la cola gaussiana.

    Valores por omision: SUPUESTOS tipicos de un APD InGaAs de 10 Gb/s con TIA
    (eficiencia cuantica 0.8, M = 10, k = 0.4, 1 nA de oscuridad primaria,
    5 pA/rtHz de ruido de entrada del TIA). Dan ~ -36 dBm a 10 Gb/s para
    BER 1e-9, en el orden de las hojas de datos de receptores de 10G."""
    eta_q: float = 0.8
    M: float = 10.0
    k_ion: float = 0.4
    i_oscuro_a: float = 1e-9
    i_n_a_rthz: float = 5e-12
    nombre: str = 'APD-OOK'

    def responsividad(self, lam_m):
        return self.eta_q * Q_ELECTRON * lam_m / (H_PLANCK * C_LUZ)

    def ber(self, p_w, tasa_bps, lam_m):
        p = np.maximum(np.asarray(p_w, float), 1e-30)
        R = self.responsividad(lam_m)
        B = 0.7 * tasa_bps
        F = self.k_ion * self.M + (1 - self.k_ion) * (2.0 - 1.0 / self.M)
        s_th2 = (self.i_n_a_rthz ** 2) * B

        def var(pk):
            return 2 * Q_ELECTRON * (self.i_oscuro_a + R * pk) * self.M ** 2 * F * B + s_th2
        s1, s0 = np.sqrt(var(2.0 * p)), np.sqrt(var(0.0))
        Q = R * self.M * 2.0 * p / (s1 + s0)
        return _q(Q)

    def sensibilidad_w(self, tasa_bps, lam_m, ber_obj=1e-9):
        return _sensibilidad(self, tasa_bps, lam_m, ber_obj)


@dataclass(frozen=True)
class Homodino:
    """BPSK homodino ideal con penalizacion de implementacion.

    BER = 1/2 erfc(sqrt(2 Np / L)), Np = P / (h nu tasa) fotones por bit EN LA
    ENTRADA del receptor y L = 10^(penalizacion_db/10) >= 1 la penalizacion
    (ruido de fase del lazo de enganche, desbalance, ruido de intensidad
    relativa del oscilador local, eficiencia del hibrido de 90 grados). En
    L = 1 es el limite cuantico (9 fotones por bit -> 1e-9). El valor por
    omision (5 dB) es un SUPUESTO de diseno; el de un terminal operativo no es
    publico en las fuentes consultadas."""
    penalizacion_db: float = 5.0
    nombre: str = 'BPSK-homodino'

    def ber(self, p_w, tasa_bps, lam_m):
        np_bit = np.maximum(np.asarray(p_w, float), 1e-30) / (energia_foton(lam_m) * tasa_bps)
        return 0.5 * erfc(np.sqrt(2.0 * np_bit / 10.0 ** (self.penalizacion_db / 10.0)))

    def sensibilidad_w(self, tasa_bps, lam_m, ber_obj=1e-9):
        return _sensibilidad(self, tasa_bps, lam_m, ber_obj)


@lru_cache(maxsize=None)
def _sensibilidad(det, tasa_bps, lam_m, ber_obj):
    """Potencia media (W) que da `ber_obj` (biseccion en log, BER decreciente)."""
    lo, hi = -90.0, 10.0          # dBm
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if float(det.ber(w_de_dbm(mid), tasa_bps, lam_m)) > ber_obj:
            lo = mid
        else:
            hi = mid
    return float(w_de_dbm(0.5 * (lo + hi)))


def fotones_por_bit(p_w, tasa_bps, lam_m):
    return p_w / (energia_foton(lam_m) * tasa_bps)


# Escalera de tasas (b/s): 10 Mb/s .. 10 Gb/s, las mismas para ambos detectores
ESCALERA_TASAS = (1e7, 5e7, 1e8, 2.5e8, 6.22e8, 1.25e9, 2.5e9, 5e9, 1e10)


def tasa_maxima(det, p_w, lam_m, ber_obj=1e-6, margen_db=3.0, escalera=ESCALERA_TASAS):
    """Mayor tasa de `escalera` cuya potencia requerida (para `ber_obj`) mas
    `margen_db` cabe en `p_w`; 0 si ninguna (enlace abierto)."""
    mejor = 0.0
    for r in escalera:
        if p_w >= det.sensibilidad_w(r, lam_m, ber_obj) * 10.0 ** (margen_db / 10.0):
            mejor = r
    return mejor
