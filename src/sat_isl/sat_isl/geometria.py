"""Geometria de una constelacion para enlaces opticos entre satelites (ISL).

Solo numpy (sin ROS). Una constelacion Walker delta se propaga con el
propagador numerico de la Estacion (`atp_ros.propagador`: gravedad central +
J2, DP45, efemeride de Hermite quintica) y de cada par de satelites se
sacan: vector de linea de vista, rango, velocidad relativa, tasa angular de la
linea de vista, angulo de adelanto (point-ahead), tiempo de luz, Doppler optico
y visibilidad (altura minima del rayo sobre la Tierra).

Marco: TEME (el del propagador), km, km/s, rad. Tierra ESFERICA de radio
ecuatorial (6378.137 km): conservador (la Tierra real es 21 km mas baja en los
polos, asi que un rayo que la esfera deja libre tambien la libra).

Cada funcion dice bajo que condicion vale su cifra.
"""
import math
from dataclasses import dataclass

import numpy as np

from atp_ros import propagador as P

C_LUZ_KM_S = 299792.458           # km/s, exacta por definicion del metro
R_TIERRA_KM = P.WGS84.re          # 6378.137 km (WGS84 ecuatorial)

# Altura minima del rayo sobre la superficie para dar el enlace por visible.
# JUSTIFICACION (calculada abajo, `refraccion_urad`): un rayo con tangente a
# 100 km se desvia ~0.01 urad por refraccion (despreciable frente a un haz de
# decenas de urad); a 50 km se desvia decenas de urad (el haz entero) y a
# 30 km cientos. Por debajo de ~70 km la refraccion pasa de 1 urad. 100 km
# (mesopausa, linea de Karman) deja ademas margen para la turbulencia y los
# aerosoles que el modelo exponencial no ve.
H_RAYO_MIN_KM = 100.0

# Refractividad del aire seco a 15 C y 101325 Pa en 1550 nm: n - 1 ~ 2.73e-4
# (Ciddor 1996 / Edlen; de memoria, SUPUESTO con +-2 %: la conclusion no
# depende de la tercera cifra).
N_MENOS_1_SUELO = 2.73e-4


# ---------------------------------------------------------------- Walker

@dataclass(frozen=True)
class ElementosCirculares:
    plano: int
    indice: int          # posicion dentro del plano
    a_km: float
    inc_rad: float
    raan_rad: float
    u0_rad: float        # argumento de latitud en t = 0

    @property
    def nombre(self):
        return f'P{self.plano}S{self.indice}'


def walker_delta(T, Pl, Fase, h_km, inc_deg, raan0_deg=0.0):
    """Constelacion Walker delta T/P/F (Walker 1984): T satelites en P planos
    igualmente espaciados en RAAN (360/P), S = T/P por plano espaciados 360/S
    en argumento de latitud, con desfase entre planos contiguos de 360 F/T.
    Devuelve la lista de `ElementosCirculares`, orden plano-mayor."""
    if T % Pl:
        raise ValueError('T debe ser multiplo de P')
    S = T // Pl
    a = R_TIERRA_KM + h_km
    out = []
    for p in range(Pl):
        for j in range(S):
            u = 2 * math.pi * j / S + 2 * math.pi * Fase * p / T
            out.append(ElementosCirculares(p, j, a, math.radians(inc_deg),
                                           math.radians(raan0_deg) + 2 * math.pi * p / Pl,
                                           u % (2 * math.pi)))
    return out


def estado_circular(el, mu=P.WGS84.mu):
    """(r, v) TEME de una orbita circular en t = 0 (geometria de Kepler)."""
    cu, su = math.cos(el.u0_rad), math.sin(el.u0_rad)
    ci, si = math.cos(el.inc_rad), math.sin(el.inc_rad)
    cr, sr = math.cos(el.raan_rad), math.sin(el.raan_rad)
    n = math.sqrt(mu / el.a_km ** 3)
    r = el.a_km * np.array([cr * cu - sr * su * ci, sr * cu + cr * su * ci, su * si])
    v = el.a_km * n * np.array([-cr * su - sr * cu * ci, -sr * su + cr * cu * ci, cu * si])
    return r, v


class Constelacion:
    """Constelacion propagada con el propagador numerico (J2, sin arrastre:
    a 550-1200 km con paneles de ~1e-2 m2/kg el arrastre es una deriva de
    metros por dia que no cambia la geometria relativa de minutos).

    `rv(i, t)` da (r, v) del satelite i a t segundos de la epoca (Hermite
    quintica sobre una rejilla de `paso_s` = 30 s: < 1 mm en LEO, segun
    `propagador.Efemeride`)."""

    def __init__(self, T=18, Pl=3, Fase=1, h_km=1200.0, inc_deg=55.0,
                 t_fin_s=7200.0, paso_s=30.0, modelo=None):
        self.T, self.Pl, self.Fase, self.h_km, self.inc_deg = T, Pl, Fase, h_km, inc_deg
        self.elementos = walker_delta(T, Pl, Fase, h_km, inc_deg)
        self.modelo = modelo or P.Modelo(grav=P.WGS84, grado=2, arrastre=False)
        self.ts = np.arange(0.0, t_fin_s + paso_s, paso_s)
        self.t_fin_s = float(self.ts[-1])
        self._efem = {}             # se propaga cada satelite la primera vez que se pide

    @property
    def n(self):
        return len(self.elementos)

    def efem(self, i):
        if i not in self._efem:
            r0, v0 = estado_circular(self.elementos[i], self.modelo.grav.mu)
            self._efem[i] = P.Efemeride.desde_modelo(
                self.modelo, np.concatenate([r0, v0]), 0.0, self.ts)
        return self._efem[i]

    def rv(self, i, t):
        r, v = self.efem(i).estado(float(t))
        return r[0], v[0]

    def indice_de(self, plano, j):
        return plano * (self.T // self.Pl) + j

    def vecinos(self, i):
        """Candidatos a enlace de i: los dos del mismo plano (adelante y atras)
        y, en cada plano contiguo (RAAN +-360/P, con vuelta), el satelite que
        en t = 0 esta mas cerca. Indices ordenados."""
        el = self.elementos[i]
        S = self.T // self.Pl
        out = [self.indice_de(el.plano, (el.indice + 1) % S),
               self.indice_de(el.plano, (el.indice - 1) % S)]
        r0 = self.rv(i, 0.0)[0]
        for dp in (1, -1):
            if self.Pl < 2:
                break
            pl = (el.plano + dp) % self.Pl
            cand = [self.indice_de(pl, j) for j in range(S)]
            out.append(min(cand, key=lambda k: float(np.linalg.norm(self.rv(k, 0.0)[0] - r0))))
        return sorted(set(k for k in out if k != i))

    def enlace(self, i, j, t, lam_m=1550e-9):
        r1, v1 = self.rv(i, t)
        r2, v2 = self.rv(j, t)
        return geometria_enlace(r1, v1, r2, v2, lam_m)


# ---------------------------------------------------------------- enlace

def altura_minima_rayo_km(r1, r2):
    """Altura minima sobre la esfera de radio R_TIERRA_KM del SEGMENTO r1-r2
    (km). Si el punto mas cercano al centro cae fuera del segmento, es el de
    un extremo."""
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    d = r2 - r1
    s = -float(np.dot(r1, d)) / float(np.dot(d, d))
    s = min(1.0, max(0.0, s))
    return float(np.linalg.norm(r1 + s * d)) - R_TIERRA_KM


def rango_maximo_visible_km(h_km, h_rayo_km=H_RAYO_MIN_KM):
    """Rango maximo entre dos satelites a la MISMA altura `h` con el rayo a
    `h_rayo` o mas sobre la superficie: la cuerda tangente a la esfera de
    radio R+h_rayo, 2 sqrt((R+h)^2 - (R+h_rayo)^2). Forma cerrada (Pitagoras)."""
    a, b = R_TIERRA_KM + h_km, R_TIERRA_KM + h_rayo_km
    return 2.0 * math.sqrt(max(a * a - b * b, 0.0))


def separacion_angular_max_deg(h_km, h_rayo_km=H_RAYO_MIN_KM):
    """Separacion en argumento de latitud (grados) maxima de dos satelites
    circulares coplanares a `h` con el rayo libre: 2 acos((R+h_rayo)/(R+h))."""
    return math.degrees(2.0 * math.acos((R_TIERRA_KM + h_rayo_km) / (R_TIERRA_KM + h_km)))


def base_perpendicular(u):
    """Base ortonormal (e1, e2) del plano normal a `u` (unitario): e1 = u x z
    normalizado (u x x si u es casi paralelo a z) y e2 = u x e1. Determinista,
    la misma para planta y controlador."""
    u = np.asarray(u, float)
    e1 = np.cross(u, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(u, np.array([1.0, 0.0, 0.0]))
    e1 = e1 / np.linalg.norm(e1)
    return e1, np.cross(u, e1)


def geometria_enlace(r1, v1, r2, v2, lam_m=1550e-9):
    """Geometria del enlace 1 -> 2 en el instante dado (todo en TEME).

    rango_km, u (linea de vista 1 -> 2), v_rel = v2 - v1, v_rad (> 0 se alejan),
    v_perp (vector, km/s: velocidad relativa transversal), tasa_los (rad/s:
    giro de la linea de vista, |v_perp|/rango), adelanto_rad = 2 |v_perp| / c
    (point-ahead: el receptor se mueve v_perp t mientras la luz va (rango/c) y
    vuelve otra vez; vale a primer orden en v/c, error relativo ~1e-4 en LEO),
    t_luz_s, doppler_hz (v_rad/lam: corrimiento optico a primer orden), y la
    visibilidad (altura minima del rayo >= H_RAYO_MIN_KM)."""
    r1, v1, r2, v2 = (np.asarray(x, float) for x in (r1, v1, r2, v2))
    d = r2 - r1
    rho = float(np.linalg.norm(d))
    u = d / rho
    vrel = v2 - v1
    vrad = float(np.dot(vrel, u))
    vperp = vrel - vrad * u
    vp = float(np.linalg.norm(vperp))
    h = altura_minima_rayo_km(r1, r2)
    return {'rango_km': rho, 'u': u, 'v_rel': vrel, 'v_rad_km_s': vrad,
            'v_perp': vperp, 'v_perp_km_s': vp, 'tasa_los_rad_s': vp / rho,
            'adelanto_rad': 2.0 * vp / C_LUZ_KM_S,
            'dir_adelanto': vperp / vp if vp > 0 else np.zeros(3),
            't_luz_s': rho / C_LUZ_KM_S,
            'doppler_hz': vrad * 1e3 / lam_m,
            'h_rayo_km': h, 'visible': bool(h >= H_RAYO_MIN_KM)}


def adelanto_exacto_rad(rv1, rv2, t, n_iter=6):
    """Point-ahead 'a la fuerza bruta' para contrastar la formula 2 v_perp / c.

    rv1(t), rv2(t) -> (r, v) en un marco inercial. Con tiempos de luz
    retardados (se resuelven por iteracion) y aberracion de primer orden de la
    velocidad del propio terminal 1:
      * direccion con que 1 VE a 2 en t: la luz sale de 2 en t - tau', llega
        desde s_in; el observador en movimiento la ve en s_in + v1_perp/c;
      * direccion a la que 1 debe EMITIR en t para alcanzar a 2 en t + tau:
        n_in; en el marco de 1 es n_in - v1_perp/c.
    El adelanto es el angulo entre ambas. Despreciando la aceleracion durante
    tau (~1e-2 s: a 8e-3 km/s^2 es < 1e-3 de v) coincide con 2 |v2 - v1|_perp / c."""
    r1, v1 = rv1(t)
    r2, _ = rv2(t)
    tau = np.linalg.norm(r2 - r1) / C_LUZ_KM_S
    for _ in range(n_iter):                         # emision hacia 2 en t + tau
        r2f, _ = rv2(t + tau)
        tau = np.linalg.norm(r2f - r1) / C_LUZ_KM_S
    r2f, _ = rv2(t + tau)
    n_in = (r2f - r1) / np.linalg.norm(r2f - r1)
    taup = np.linalg.norm(r2 - r1) / C_LUZ_KM_S
    for _ in range(n_iter):                         # luz que llega a 1 en t, salio en t - tau'
        r2p, _ = rv2(t - taup)
        taup = np.linalg.norm(r2p - r1) / C_LUZ_KM_S
    r2p, _ = rv2(t - taup)
    s_in = (r2p - r1) / np.linalg.norm(r2p - r1)
    u = (r2 - r1) / np.linalg.norm(r2 - r1)

    def perp(x):
        return x - np.dot(x, u) * u
    v1p = perp(v1) / C_LUZ_KM_S
    n1 = n_in - v1p
    s1 = s_in + v1p
    return float(np.linalg.norm(perp(n1 - s1)))


# ---------------------------------------------------------------- refraccion

def refraccion_urad(h_tangente_km, atm=P.ATMOSFERA):
    """Desviacion TOTAL (urad) de un rayo de 1550 nm cuyo punto mas bajo esta a
    `h_tangente_km`, en la atmosfera exponencial por capas del propagador
    (Vallado, tabla 8-4): alfa = (n_t - 1) sqrt(2 pi r_t / H), con (n_t - 1) =
    (n_0 - 1) rho(h_t)/rho_0 y H la escala local de la capa (rayo con
    H << r_t; Born y Wolf / Kliore). Es una ESTIMACION de primer orden: la
    atmosfera real tiene estructura (inversiones, turbulencia, aerosoles) que
    no se modela."""
    h = float(h_tangente_km)
    rho = float(atm.densidad(h))
    i = int(np.clip(np.searchsorted(atm.h0, h, side='right') - 1, 0, len(atm.h0) - 1))
    esc = float(atm.esc[i])
    n_t = N_MENOS_1_SUELO * rho / float(atm.rho0[0])
    return 1e6 * n_t * math.sqrt(2.0 * math.pi * (R_TIERRA_KM + h) / esc)


def altura_para_refraccion_km(alfa_max_urad, lo=20.0, hi=200.0):
    """Altura de tangente por debajo de la cual la refraccion pasa de
    `alfa_max_urad` (biseccion; la refraccion decrece con la altura)."""
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if refraccion_urad(mid) > alfa_max_urad:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)
