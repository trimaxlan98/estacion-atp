"""Modelos de medicion de una estacion terrena: acimut, elevacion, rango y
tasa de rango (Doppler), con sus parciales respecto del estado TEME.

Solo numpy. Es el "h(x)" de la determinacion de orbita (`od.py`): lo que la
estacion de la casa mide de verdad. La cadena es la de `orbit.py` (TEME ->
R3(GMST) -> ECEF -> ENU, estacion WGS84) y las pruebas exigen que coincida
con `orbit.observar` (SGP4) a precision de maquina.

Tiempo: segundos desde una epoca de referencia `Reloj` (UTC como UT1, igual
que `orbit.gmst_rad`; corregible con `dut1_s`).

QUE SE DESPRECIA (cada uno es un sesgo posible en datos reales):
  * Tiempo de luz (2-7 ms en LEO: el satelite avanza 15-50 m, 0.001-0.003 deg
    a 1000 km) y aberracion: la verdad simulada y el modelo del estimador
    usan la misma geometria instantanea, asi que son consistentes entre si;
    en datos reales habria que corregirlos.
  * Refraccion atmosferica (hasta ~0.17 deg a 5 deg de elevacion; <0.01 deg
    por encima de ~30 deg). Corregirla con un modelo estandar (Saastamoinen,
    Bennett) antes de ingerir angulos reales.
  * Retardo ionosferico/troposferico en el rango (~2 m en S, cenit) y el
    sesgo/deriva del oscilador en el Doppler: absorbidos en las sigmas.
  * Movimiento del polo (~10 m) y UT1-UTC (hasta 0.44 km en LEO): igual que
    en `orbit.py`.

Convenciones: angulos en radianes, rango en km, tasa de rango en km/s
(+ = se aleja), igual que `orbit.observar` (que da grados). El Doppler en Hz
es -f0 * rr / c (primer orden).
"""
import datetime as dt
import math
from dataclasses import dataclass

import numpy as np

from atp_ros import orbit
from atp_ros.propagador import OMEGA_TIERRA

C_KM_S = 299792.458

CANALES = ('az', 'el', 'rng', 'rr')

# Sigmas por defecto (1 sigma), con su justificacion. Son SUPUESTOS de la
# casa, no cifras medidas en una estacion real.
#  * az, el: 0.01 deg. Tras el autotrack el apuntamiento da 0.004 deg rms
#    (N5, scripts/validar.sh n5b); el encoder de 14 bits (config C2) cuantiza
#    a 360/2^14 = 0.022 deg -> sigma de cuantizacion LSB/sqrt(12) = 0.0063 deg;
#    en cuadratura: 0.0075 deg, redondeado a 0.01 deg.
#  * rng: 10 m. El limite termico de un rango por tonos/PN con C/N0 70 dB-Hz
#    es de centimetros (CRLB 1/(2 pi beta sqrt(2 C/N0 T))); manda la
#    calibracion del retardo de la estacion y el retardo ionosferico residual
#    (~2 m en S, cenit; mas a baja elevacion).
#  * rr: 0.05 m/s (0.37 Hz a 2.2 GHz). Un GPSDO de 1e-10 de estabilidad da
#    0.22 Hz = 0.03 m/s; se suma la ionosfera residual. El limite termico del
#    PLL (milihertz) queda muy por debajo.
SIGMAS_DEFECTO = {'az': math.radians(0.01), 'el': math.radians(0.01),
                  'rng': 0.010, 'rr': 0.05e-3}


# ---------------------------------------------------------------- tiempo

class Reloj:
    """Epoca de referencia UTC; los tiempos son segundos desde ella."""

    def __init__(self, epoca_utc, dut1_s=0.0):
        self.epoca = epoca_utc
        self.jd, self.fr = orbit.a_jd(epoca_utc)
        self.dut1_s = float(dut1_s)

    def jdfr(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        return np.full_like(t, self.jd), self.fr + t / 86400.0

    def gmst(self, t):
        jd, fr = self.jdfr(t)
        return orbit.gmst_rad(jd, fr, self.dut1_s)

    def utc(self, t):
        return self.epoca + dt.timedelta(seconds=float(t))

    def segundos(self, utc):
        return (utc - self.epoca).total_seconds()


# ---------------------------------------------------------------- estacion

@dataclass
class Sitio:
    """Estacion preparada para medir: posicion ECEF (km) y matriz ENU."""
    est: orbit.Estacion
    r_ecef: np.ndarray
    enu: np.ndarray

    @property
    def nombre(self):
        return self.est.nombre


def sitio(est):
    """`orbit.Estacion` -> `Sitio` (WGS84 exacta)."""
    return Sitio(est, orbit.geodetica_a_ecef(est.lat_deg, est.lon_deg, est.alt_m),
                 orbit.matriz_enu(est.lat_deg, est.lon_deg))


def _rot(th):
    """R3(th) por instante, forma (N, 3, 3): x_ecef = R x_teme."""
    c, s = np.cos(th), np.sin(th)
    z, u = np.zeros_like(th), np.ones_like(th)
    return np.stack([np.stack([c, s, z], -1), np.stack([-s, c, z], -1),
                     np.stack([z, z, u], -1)], -2)


def _geometria(reloj, sit, t, r, v):
    """Cantidades comunes a h(x) y a su jacobiano. r, v: (N, 3) TEME."""
    t = np.atleast_1d(np.asarray(t, float))
    th = reloj.gmst(t)
    R = _rot(th)                                      # TEME -> ECEF
    T = np.einsum('ij,njk->nik', sit.enu, R)          # TEME -> ENU
    rs = np.einsum('nji,j->ni', R, sit.r_ecef)        # estacion en TEME
    vs = OMEGA_TIERRA * np.stack([-rs[:, 1], rs[:, 0], np.zeros(len(t))], -1)
    rho_t = r - rs
    u = v - vs
    e = np.einsum('nij,nj->ni', T, rho_t)
    return T, rho_t, u, e


def observables(reloj, sit, t, r, v):
    """h(x): dict con az (rad, [0, 2pi)), el (rad), rng (km), rr (km/s) para
    estados TEME (N, 3). Mismo resultado que `orbit.observar`."""
    r, v = np.atleast_2d(r), np.atleast_2d(v)
    _, rho_t, u, e = _geometria(reloj, sit, t, r, v)
    rng = np.sqrt(np.sum(e * e, axis=1))
    return {'az': np.arctan2(e[:, 0], e[:, 1]) % (2 * np.pi),
            'el': np.arcsin(np.clip(e[:, 2] / rng, -1, 1)),
            'rng': rng, 'rr': np.sum(rho_t * u, axis=1) / rng}


def jacobiano(reloj, sit, t, r, v, canales=CANALES):
    """Parciales de los `canales` respecto de (r, v) TEME: array (N, nc, 6).
        d az/d e = [e_N, -e_E, 0] / q            q = e_E^2 + e_N^2
        d el/d e = [-e_E e_U, -e_N e_U, q] / (rho^2 sqrt(q))
        d rng/d e = e / rho
        d rr/d r = (u - rr rho^) / rho ;  d rr/d v = rho^
    con e = T (r - r_s) y d/dr = (d/de) T. Valido fuera del cenit (q > 0:
    el acimut no esta definido sobre la vertical)."""
    r, v = np.atleast_2d(r), np.atleast_2d(v)
    T, rho_t, u, e = _geometria(reloj, sit, t, r, v)
    n = len(r)
    rho = np.sqrt(np.sum(e * e, axis=1))
    q = e[:, 0] ** 2 + e[:, 1] ** 2
    sq = np.sqrt(q)
    rhat = rho_t / rho[:, None]
    rr = np.sum(rhat * u, axis=1)
    J = np.zeros((n, len(canales), 6))
    for k, c in enumerate(canales):
        if c == 'az':
            de = np.stack([e[:, 1], -e[:, 0], np.zeros(n)], -1) / q[:, None]
            J[:, k, :3] = np.einsum('ni,nij->nj', de, T)
        elif c == 'el':
            de = np.stack([-e[:, 0] * e[:, 2], -e[:, 1] * e[:, 2], q], -1) / (
                (rho ** 2 * sq)[:, None])
            J[:, k, :3] = np.einsum('ni,nij->nj', de, T)
        elif c == 'rng':
            J[:, k, :3] = rhat
        elif c == 'rr':
            J[:, k, :3] = (u - rr[:, None] * rhat) / rho[:, None]
            J[:, k, 3:] = rhat
        else:
            raise ValueError(c)
    return J


def doppler_hz(rr_km_s, f0_hz):
    """Doppler de bajada de un solo sentido: -f0 rr / c (primer orden en
    v/c; el error segundo orden es ~1e-5 relativo en LEO)."""
    return -np.asarray(rr_km_s, float) / C_KM_S * float(f0_hz)


def envolver(d):
    """Diferencia angular envuelta a (-pi, pi]."""
    return (np.asarray(d, float) + np.pi) % (2 * np.pi) - np.pi


def separacion_deg(az1, el1, az2, el2):
    """Angulo (deg) entre dos direcciones (az, el en rad): arccos del
    producto punto (via atan2, estable para angulos pequenos). Es el
    desapuntamiento REAL de la antena, incluida la compresion cos(el) del
    acimut."""
    def u(az, el):
        return np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)], -1)
    a, b = u(np.asarray(az1, float), np.asarray(el1, float)), u(
        np.asarray(az2, float), np.asarray(el2, float))
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b), axis=-1),
                                 np.sum(a * b, axis=-1)))


# ---------------------------------------------------------------- conjunto de medidas

class Medidas:
    """Medidas de uno o varios pases de una o varias estaciones.

    Arrays de largo N: `t` (s desde la epoca del `Reloj`), `isit` (indice en
    `sitios`) y un valor por canal en CANALES (NaN = ese canal no se midio en
    ese instante). Angulos en rad, rango en km, rr en km/s."""

    def __init__(self, reloj, sitios, t, isit, valores):
        self.reloj, self.sitios = reloj, list(sitios)
        self.t = np.asarray(t, float)
        self.isit = np.asarray(isit, int)
        self.val = {c: np.asarray(valores.get(c, np.full(len(self.t), np.nan)), float)
                    for c in CANALES}

    def __len__(self):
        return len(self.t)

    def ordenar(self):
        i = np.argsort(self.t, kind='stable')
        return Medidas(self.reloj, self.sitios, self.t[i], self.isit[i],
                       {c: v[i] for c, v in self.val.items()})

    def concatenar(self, otra):
        if otra.reloj is not self.reloj or otra.sitios != self.sitios:
            raise ValueError('mismo reloj y mismos sitios para concatenar')
        return Medidas(self.reloj, self.sitios, np.r_[self.t, otra.t],
                       np.r_[self.isit, otra.isit],
                       {c: np.r_[self.val[c], otra.val[c]] for c in CANALES}).ordenar()

    def solo(self, canales):
        """Copia con SOLO los canales dados (los demas pasan a NaN)."""
        return Medidas(self.reloj, self.sitios, self.t, self.isit,
                       {c: (v if c in canales else np.full(len(v), np.nan))
                        for c, v in self.val.items()})

    def con_sesgo(self, sesgo_az_deg=0.0, sesgo_el_deg=0.0):
        """Copia con un sesgo constante de apuntamiento en az y el (deg):
        error sistematico de la montura (el estimador no lo conoce)."""
        val = {c: v.copy() for c, v in self.val.items()}
        val['az'] = val['az'] + math.radians(sesgo_az_deg)
        val['el'] = val['el'] + math.radians(sesgo_el_deg)
        return Medidas(self.reloj, self.sitios, self.t, self.isit, val)


def generar(reloj, sitios, isit, t, r, v, sigmas=None, rng=None, canales=CANALES,
            sesgo_ang_deg=(0.0, 0.0)):
    """Medidas simuladas de los estados TEME (r, v) en los instantes t de la
    estacion `sitios[isit]` (isit escalar): h(x) mas ruido gaussiano de
    sigma `sigmas` (SIGMAS_DEFECTO). `rng` es un numpy Generator (None =
    sin ruido). `sesgo_ang_deg` (az, el) suma un error sistematico."""
    sg = dict(SIGMAS_DEFECTO)
    sg.update(sigmas or {})
    t = np.atleast_1d(np.asarray(t, float))
    h = observables(reloj, sitios[isit], t, r, v)
    val = {}
    for c in canales:
        x = h[c].copy()
        if rng is not None:
            x = x + rng.normal(0.0, sg[c], len(t))
        val[c] = x
    if 'az' in val:
        val['az'] = (val['az'] + math.radians(sesgo_ang_deg[0])) % (2 * np.pi)
    if 'el' in val:
        val['el'] = val['el'] + math.radians(sesgo_ang_deg[1])
    return Medidas(reloj, sitios, t, np.full(len(t), int(isit)), val)
