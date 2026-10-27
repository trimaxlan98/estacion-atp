"""Comunicacion del rover por antenas (CONTRATO sec. 19, equipo R7): rele orbital, banda X directa, terreno y DTN.

Python puro (solo numpy), determinista: el unico azar sale de `numpy.random.default_rng(semilla)`.

    from atp_rover.comunicacion import Enlace
    e = Enlace(planeta='marte', rele='mro', lat_deg=-4.6, lon_deg=137.4, antena='omni_helice')
    m = e.muestras(range(0, 10 * 88775, 30))       # elevacion, rango, visibilidad, tasa por instante
    pases = e.pases(0, 10 * 88775)                  # ventanas de visibilidad con bits por pase

Que modela (todo en simulacion, con los supuestos de abajo; NADA esta validado contra telemetria de un rover real):
  * Orbita del rele: kepleriana de dos cuerpos con elementos de `RELES` (MRO, Odyssey, MAVEN: Mars Relay Description for
    Discovery 2019, JPL, tablas 4.1-1, 5.1-1, 6.1-1; Luna: aproximaciones propias). Sin J2, sin precesion: valido para
    unos dias, no para meses. La NRHO lunar real es un problema de tres cuerpos; aqui es una elipse de dos cuerpos con
    el mismo periodo y perilunio (APROXIMACION GRUESA que NO reproduce la NRHO).
  * Geometria: el cuerpo gira a su periodo sideral; el rover esta en (lat, lon). El rele se ve desde el rover con
    elevacion y azimut locales. Visible = elevacion >= max(`el_min_deg`, horizonte local del terreno en ese azimut).
  * Horizonte local: se traza, desde la antena (a `h_ant` m del suelo), el angulo de elevacion maximo del relieve de
    `Terreno.altura` en cada azimut hasta `alcance` m (x = este, y = norte: CONVENCION asumida). Mas alla del mapa
    el terreno se repite en el borde (asi lo hace `Terreno.altura`). Un cuenco de 15 m de radio y 3 m de profundidad,
    con el rover al centro, da un horizonte de ~14 deg.
  * UHF (relé): umbrales de potencia recibida de Electra (tabla 6.2-2 de la guia 2019, el signo se interpreta como dBm
    NEGATIVOS: el texto solo dice «dB»); la tasa adaptativa elige la mas alta con margen >= `margen_db`. Potencia del
    rover, ganancias y perdidas: SUPUESTOS (marcados en `UHF`).
  * Banda X directa a Tierra: presupuesto C/N0 con G/T de una antena de 34 m (supuesto) y escalera de tasas 0.5-32 kbps.
  * Retardo de luz: distancia geometrica entre orbitas circulares coplanares (Tierra, Marte) con longitudes medias de J2000
    de los elementos aproximados de Standish (JPL «Keplerian Elements for Approximate Positions», recordados, no
    recontrastados); ignora excentricidades: Marte entre 0.52 y 2.52 UA (el valor real va de ~0.37 a ~2.67 UA).
  * DTN: `ColaDTN`/`simular_dtn`: bundles FIFO con almacenamiento acotado, retransmision en el siguiente contacto.
"""
import heapq
import json
import math
import os
from bisect import bisect_left
from collections import deque

import numpy as np

from atp_rover.planetas import PLANETAS

C_KM_S = 299792.458
UA_KM = 149597870.7
K_BOLTZMANN_DBW = -228.6          # 10 log10(k) en dBW/Hz/K
DIA_TIERRA_S = 86400.0

# ------------------------------------------------------------------------------------------------------------------
# Cuerpos y reles. Mu y radios: NSSDCA Fact Sheets (Marte 42828.37 km3/s2, 3389.5 km; Luna 4902.80, 1737.4 km).
# dir_tierra: direccion de la Tierra en el marco inercial del cuerpo (ra, dec fijos; SUPUESTO dec = 0). En la Luna la
# Tierra queda fija en el cielo (acoplamiento de marea, sin libracion): se gira con el cuerpo.
# ------------------------------------------------------------------------------------------------------------------
CUERPOS = {
    'marte': dict(radio_km=3389.5, gm=42828.375, rot_s=88642.66, dia_s=88775.244, tierra_fija_en_cuerpo=False,
                  tierra_ra_deg=0.0, tierra_dec_deg=0.0),
    'luna': dict(radio_km=1737.4, gm=4902.800, rot_s=2360591.5, dia_s=DIA_TIERRA_S, tierra_fija_en_cuerpo=True,
                 tierra_ra_deg=0.0, tierra_dec_deg=0.0),
}

_NRHO_T_S = 6.56 * DIA_TIERRA_S                       # periodo de la NRHO 9:2 (~6.56 d, valor recordado, sin recontrastar)
_NRHO_A = (4902.8 * (_NRHO_T_S / (2 * math.pi)) ** 2) ** (1.0 / 3.0)
_NRHO_RP = 1737.4 + 1500.0                            # perilunio ~1500 km de altura (recordado)

# a_km, e, inc, raan, argp (deg), tp_s = tiempo desde el periapsis en t = 0, down_ref: (tasa a Tierra, distancia) de referencia
RELES = {
    'marte': {
        'mro': dict(a_km=3663.7, e=0.0083, inc_deg=92.7, raan_deg=260.6, argp_deg=272.6, tp_s=2197.8,
                    down_bps=500e3, down_d_km=2.67 * UA_KM, down_max_bps=5.2e6, fuente='JPL 2019 tabla 5.1-1 (epoch 2019-04-01); 500 kbps a 34 m a distancia maxima'),
        'odyssey': dict(a_km=3795.3, e=0.00786, inc_deg=93.0, raan_deg=142.8, argp_deg=270.0, tp_s=3537.9, tasa_max_bps=256e3,
                        down_bps=14.22e3, down_d_km=2.67 * UA_KM, down_max_bps=124.4e3, fuente='JPL 2019 tabla 4.1-1; 14.22 kbps a 34 m a distancia maxima'),
        'maven': dict(a_km=5722.4, e=0.3851, inc_deg=74.8, raan_deg=273.4, argp_deg=242.5, tp_s=5528.5,
                      down_bps=500e3, down_d_km=2.67 * UA_KM, down_max_bps=5.2e6, fuente='JPL 2019 tabla 6.1-1; tasa a Tierra SUPUESTA igual a la de MRO'),
    },
    'luna': {
        'nrho_kepler': dict(a_km=_NRHO_A, e=1.0 - _NRHO_RP / _NRHO_A, inc_deg=90.0, raan_deg=0.0, argp_deg=90.0, tp_s=0.0,
                            down_bps=1e6, down_d_km=384400.0, down_max_bps=1e6, fuente='SUPUESTO: elipse de dos cuerpos con T = 6.56 d y perilunio ~1500 km; NO es la NRHO'),
        'polar_100km': dict(a_km=1837.4, e=0.0, inc_deg=90.0, raan_deg=0.0, argp_deg=0.0, tp_s=0.0,
                            down_bps=1e6, down_d_km=384400.0, down_max_bps=1e6, fuente='SUPUESTO: orbita polar circular a 100 km'),
    },
}

# ------------------------------------------------------------------------------------------------------------------
# Enlaces. UHF Electra: frecuencia rover->orbitador 401.585625 MHz (JPL 2019). Potencia del rover (7 W), ganancias y perdidas
# del rover y del rele: SUPUESTOS de orden de magnitud (no verificados). perd_rx_db = 2 dB «front end» de la guia.
# ------------------------------------------------------------------------------------------------------------------
UHF = dict(f_hz=401.585625e6, p_tx_w=7.0, perd_tx_db=1.0, g_rele_dbi=4.0, perd_rx_db=2.0, margen_db=2.0, eficiencia=0.8)
# (tasa bps, umbral dBm). 1-8 kbps: convolucional con portadora residual; 32-2048 kbps: LDPC con portadora suprimida (guia JPL 2019,
# tabla 6.2-2). Se omite 16 kbps (su umbral convolucional -122.1 dBm es peor que el de 32 kbps LDPC -122.75 dBm).
ESCALERA_UHF = ((1e3, -133.8), (2e3, -131.2), (4e3, -128.1), (8e3, -125.1), (32e3, -122.75), (64e3, -119.7), (128e3, -116.7),
                (256e3, -113.7), (512e3, -110.7), (1024e3, -107.6), (2048e3, -104.8))
# S (relé lunar, SUPUESTO): enlace generico por Eb/N0.
BANDA_S = dict(f_hz=2.2e9, p_tx_w=5.0, perd_tx_db=1.0, g_rele_dbi=12.0, t_sys_k=500.0, ebn0_db=4.0, margen_db=3.0, eficiencia=0.8,
               escalera_bps=(1e3, 4e3, 16e3, 64e3, 256e3, 1024e3, 2048e3))
# X directo a Tierra (SUPUESTOS: recordados de Curiosity, 0.28 m HGA, sin verificar; DSN 34 m).
BANDA_X = dict(f_hz=7.17e9, p_tx_w=15.0, perd_db=2.0, gt_dbk=51.0, ebn0_db=2.0, margen_db=2.0,
               escalera_bps=(500.0, 1e3, 2e3, 4e3, 8e3, 16e3, 32e3), horas_dia=2.0, tasa_comando_bps=500.0)

ANTENAS_UHF = ('omni_lineal', 'omni_helice', 'direccional')
ANTENAS_X = ('hga_x', 'lga_x')


def fspl_db(d_km, f_hz):
    """Perdida de espacio libre (igual a `atp_ros.model.fspl_db`: 92.45 + 20 log d_km + 20 log f_GHz)."""
    return 92.45 + 20.0 * np.log10(np.asarray(d_km, dtype=float)) + 20.0 * np.log10(float(f_hz) / 1e9)


def perdida_apuntamiento_db(error_deg, ancho_3db_deg):
    """Perdida gaussiana de apuntamiento: 12 (e/theta_3dB)^2 dB (aproximacion de haz gaussiano)."""
    return 12.0 * (np.asarray(error_deg, dtype=float) / float(ancho_3db_deg)) ** 2


def ganancia_rover_uhf_dbi(tipo, el_deg, g_dbi=None, error_apunt_deg=5.0, ancho_deg=60.0):
    """Ganancia de la antena UHF del rover hacia el rele (`el_deg` = elevacion sobre el horizonte local). SUPUESTOS:
    omni_lineal (monopolo vertical): 2.15 dBi con diagrama cos(el), nulo en el cenit (limitado a -8 dBi);
    omni_helice (cuadrifilar, hemisferica): 3 dBi en el cenit y -3 dBi en el horizonte;
    direccional (patch/yagi orientable): g_dbi (10 por defecto), ancho de haz `ancho_deg`, apuntada al rele con error."""
    el = np.asarray(el_deg, dtype=float)
    if tipo == 'omni_lineal':
        g = 2.15 if g_dbi is None else g_dbi
        return g + 20.0 * np.log10(np.maximum(np.cos(np.radians(el)), 0.4))
    if tipo == 'omni_helice':
        g = 3.0 if g_dbi is None else g_dbi
        return g - 6.0 * (1.0 - np.sin(np.radians(np.maximum(el, 0.0))))
    if tipo == 'direccional':
        g = 10.0 if g_dbi is None else g_dbi
        return g - perdida_apuntamiento_db(error_apunt_deg, ancho_deg) + 0.0 * el
    raise ValueError(f'antena UHF desconocida: {tipo!r} (hay: {", ".join(ANTENAS_UHF)})')


def ganancia_rover_x_dbi(tipo, g_dbi=None, error_apunt_deg=1.0):
    """X directo: hga_x = 25 dBi con haz ~ sqrt(41253/G) deg y error de apuntamiento; lga_x = 7 dBi fija (SUPUESTOS)."""
    if tipo == 'hga_x':
        g = 25.0 if g_dbi is None else g_dbi
        bw = math.sqrt(41253.0 / 10.0 ** (g / 10.0))
        return g - float(perdida_apuntamiento_db(error_apunt_deg, bw))
    if tipo == 'lga_x':
        return 7.0 if g_dbi is None else g_dbi
    raise ValueError(f'antena X desconocida: {tipo!r} (hay: {", ".join(ANTENAS_X)})')


def ganancia_rele_dbi(g0_dbi, el_deg, r_rele_km, radio_km):
    """Ganancia del rele hacia el rover: g0 - 6 (theta/60)^2, theta = angulo desde el nadir (SUPUESTO), con
    sin(theta) = R cos(el) / r_rele; minimo g0 - 12."""
    s = np.clip(radio_km * np.cos(np.radians(el_deg)) / r_rele_km, 0.0, 1.0)
    th = np.degrees(np.arcsin(s))
    return g0_dbi - np.minimum(6.0 * (th / 60.0) ** 2, 12.0)


def tasa_uhf(prx_dbm, margen_db, max_bps=None):
    """(tasa bps, margen dB a la tasa elegida; `max_bps` = tope del transceptor, p. ej. 256 kbps en Odyssey): la mas alta de ESCALERA_UHF con prx - umbral >= margen (0 si ninguna)."""
    prx = np.asarray(prx_dbm, dtype=float)
    tasa = np.zeros_like(prx)
    marg = np.full_like(prx, -99.0)
    for r, thr in ESCALERA_UHF:
        if max_bps is not None and r > max_bps:
            continue
        m = prx - thr
        ok = (m >= margen_db) & (r > tasa)
        tasa = np.where(ok, r, tasa)
        marg = np.where(ok, m, marg)
    return tasa, marg


def tasa_ebn0(cn0_dbhz, ebn0_db, margen_db, escalera):
    """Tasa de la escalera con C/N0 - 10 log R >= Eb/N0 + margen (0 si ninguna), y su margen."""
    cn0 = np.asarray(cn0_dbhz, dtype=float)
    tasa = np.zeros_like(cn0)
    marg = np.full_like(cn0, -99.0)
    for r in escalera:
        m = cn0 - 10.0 * math.log10(r) - ebn0_db
        ok = (m >= margen_db) & (r > tasa)
        tasa = np.where(ok, r, tasa)
        marg = np.where(ok, m, marg)
    return tasa, marg


def prob_perdida_trama(margen_db):
    """Probabilidad de perder una trama dado el margen sobre el umbral: logistica 1/(1+exp(2 m)). SUPUESTO de forma
    (50 % en el umbral, 1.8 % con 2 dB); no es una curva de BER medida."""
    return 1.0 / (1.0 + np.exp(np.clip(2.0 * np.asarray(margen_db, dtype=float), -50, 50)))


# ------------------------------------------------------------------------------------------------------------------
# Retardo de luz Tierra-cuerpo
# ------------------------------------------------------------------------------------------------------------------
def jd_de_iso(texto):
    """Dia juliano de 'AAAA-MM-DD[THH:MM:SS]' (calendario gregoriano, UTC ~ TT para este uso)."""
    t = texto.replace(' ', 'T')
    f, _, h = t.partition('T')
    y, mo, d = (int(v) for v in f.split('-'))
    hh = 0.0
    if h:
        p = [float(v) for v in h.rstrip('Z').split(':')] + [0.0, 0.0]
        hh = p[0] / 24.0 + p[1] / 1440.0 + p[2] / 86400.0
    if mo <= 2:
        y, mo = y - 1, mo + 12
    a = y // 100
    b = 2 - a + a // 4
    return int(365.25 * (y + 4716)) + int(30.6001 * (mo + 1)) + d + b - 1524.5 + hh


# longitud media en J2000 y velocidad (deg/siglo) de Standish (aproximado, circular coplanar; ver docstring)
_ELEM = {'tierra': (100.46457166, 35999.37306329, 1.00000261), 'marte': (-4.55343205, 19140.30268499, 1.52371034)}


def distancia_tierra_km(planeta, jd):
    """Distancia geometrica Tierra-cuerpo [km] en el dia juliano `jd` (puede ser un arreglo). Marte: orbitas circulares
    coplanares (error hasta ~0.15 UA respecto de la real); Luna: 384400 km fijos (no se modela la excentricidad de 0.055)."""
    jd = np.asarray(jd, dtype=float)
    if planeta == 'luna':
        return np.full_like(jd, 384400.0)
    if planeta != 'marte':
        raise KeyError(f'distancia a la Tierra solo para marte y luna, no {planeta!r}')
    T = (jd - 2451545.0) / 36525.0
    le0, ler, ae = _ELEM['tierra']
    lm0, lmr, am = _ELEM['marte']
    dl = np.radians((lm0 + lmr * T) - (le0 + ler * T))
    return UA_KM * np.sqrt(ae ** 2 + am ** 2 - 2.0 * ae * am * np.cos(dl))


def owlt_s(planeta, jd):
    """Retardo de luz de una via [s]."""
    return distancia_tierra_km(planeta, jd) / C_KM_S


# ------------------------------------------------------------------------------------------------------------------
# Orbita y geometria
# ------------------------------------------------------------------------------------------------------------------
def _kepler_E(M, e):
    E = np.where(e < 0.8, M, math.pi * np.ones_like(M))
    for _ in range(60):
        d = (E - e * np.sin(E) - M) / (1.0 - e * np.cos(E))
        E = E - d
        if np.max(np.abs(d)) < 1e-12:
            break
    return E


def posicion_orbita(gm, el, t_s):
    """Posicion inercial (3, N) [km] en un marco ecuatorial del cuerpo de un rele con elementos `el` (dict de RELES)."""
    t = np.asarray(t_s, dtype=float)
    a, e = el['a_km'], el['e']
    n = math.sqrt(gm / a ** 3)
    M = np.mod(n * (t + el.get('tp_s', 0.0)), 2 * math.pi)
    E = _kepler_E(M, e)
    xp = a * (np.cos(E) - e)
    yp = a * math.sqrt(1.0 - e * e) * np.sin(E)
    w, i, om = (math.radians(el[k]) for k in ('argp_deg', 'inc_deg', 'raan_deg'))
    cw, sw, ci, si, co, so = math.cos(w), math.sin(w), math.cos(i), math.sin(i), math.cos(om), math.sin(om)
    x = (co * cw - so * sw * ci) * xp + (-co * sw - so * cw * ci) * yp
    y = (so * cw + co * sw * ci) * xp + (-so * sw + co * cw * ci) * yp
    z = (sw * si) * xp + (cw * si) * yp
    return np.array([x, y, z])


def mascara_horizonte(terreno, x=0.0, y=0.0, h_ant=1.0, n_az=72, alcance=60.0, paso=0.25):
    """Horizonte local [deg] por azimut (0 = norte, horario; x = este, y = norte). `terreno.altura(x, y)` acepta arreglos.
    Con terreno None: horizonte plano 0. Devuelve un arreglo (n_az,) con el angulo de elevacion maximo del relieve."""
    if terreno is None:
        return np.zeros(n_az)
    az = np.radians(np.arange(n_az) * 360.0 / n_az)
    d = np.arange(paso, alcance + 1e-9, paso)
    X = x + d[None, :] * np.sin(az)[:, None]
    Y = y + d[None, :] * np.cos(az)[:, None]
    h0 = float(terreno.altura(x, y)) + h_ant
    ang = np.degrees(np.arctan2(terreno.altura(X, Y) - h0, d[None, :]))
    return np.maximum(ang.max(axis=1), -90.0)


def _interp_mascara(mascara, az_deg):
    n = len(mascara)
    u = (np.asarray(az_deg) % 360.0) * n / 360.0
    i = np.floor(u).astype(int) % n
    f = u - np.floor(u)
    return mascara[i] * (1 - f) + mascara[(i + 1) % n] * f


def _siguiente(indices, i):
    """Primer elemento de la lista ordenada `indices` que es >= i, o None."""
    k = bisect_left(indices, i)
    return indices[k] if k < len(indices) else None


# ------------------------------------------------------------------------------------------------------------------
# Escena de enlace
# ------------------------------------------------------------------------------------------------------------------
class Enlace:
    """Geometria y presupuesto del rover en (lat, lon) de `planeta` con un `rele` en orbita (clave de RELES o dict)."""

    def __init__(self, planeta='marte', rele='mro', lat_deg=-4.6, lon_deg=137.4, terreno=None, antena='omni_helice',
                 antena_params=None, antena_x='hga_x', antena_x_params=None, el_min_deg=10.0, margen_db=None,
                 ganancia_extra_db=0.0, rele_params=None, fase_rot_deg=0.0, jd0=None, h_ant=1.0, el_min_tierra_deg=10.0,
                 dsn_fraccion=1.0 / 3.0, banda_x=None):
        if planeta not in PLANETAS:
            raise KeyError(f'planeta desconocido: {planeta!r}')
        if planeta not in CUERPOS:
            raise KeyError(f'enlace solo definido para {", ".join(CUERPOS)} (no para {planeta!r})')
        self.planeta, self.cuerpo = planeta, dict(CUERPOS[planeta])
        if isinstance(rele, dict):
            self.rele_nombre, self.rele = 'personalizado', dict(rele)
        else:
            if rele not in RELES[planeta]:
                raise KeyError(f'rele {rele!r} no definido para {planeta} (hay: {", ".join(RELES[planeta])})')
            self.rele_nombre, self.rele = rele, dict(RELES[planeta][rele])
        self.rele.update(rele_params or {})
        self.lat, self.lon = math.radians(lat_deg), math.radians(lon_deg)
        self.lat_deg, self.lon_deg = lat_deg, lon_deg
        self.antena, self.antena_params = antena, dict(antena_params or {})
        self.antena_x, self.antena_x_params = antena_x, dict(antena_x_params or {})
        self.banda_x = {**BANDA_X, **(banda_x or {})}
        self.el_min = float(el_min_deg)
        self.el_min_tierra = float(el_min_tierra_deg)
        self.ganancia_extra_db = float(ganancia_extra_db)
        self.fase = math.radians(fase_rot_deg)
        self.jd0 = jd_de_iso('2026-10-07') if jd0 is None else float(jd0)
        self.dsn_fraccion = float(dsn_fraccion)
        self.mascara = mascara_horizonte(terreno, h_ant=h_ant)
        self.es_uhf = (planeta == 'marte')
        self.banda = dict(UHF if self.es_uhf else BANDA_S)
        if margen_db is not None:
            self.banda['margen_db'] = float(margen_db)
        self._cache = {}

    # --- geometria -----------------------------------------------------------------------------------------------
    def _triedro(self, t):
        th = self.fase + 2 * math.pi * np.asarray(t, dtype=float) / self.cuerpo['rot_s'] + self.lon
        cf, sf = math.cos(self.lat), math.sin(self.lat)
        up = np.array([cf * np.cos(th), cf * np.sin(th), sf * np.ones_like(th)])
        este = np.array([-np.sin(th), np.cos(th), np.zeros_like(th)])
        norte = np.array([-sf * np.cos(th), -sf * np.sin(th), cf * np.ones_like(th)])
        return up, este, norte

    def _dir_tierra(self, t):
        c = self.cuerpo
        ra, dec = math.radians(c['tierra_ra_deg']), math.radians(c['tierra_dec_deg'])
        w = 2 * math.pi / c['rot_s'] if c['tierra_fija_en_cuerpo'] else 0.0
        a = ra + w * np.asarray(t, dtype=float)
        return np.array([math.cos(dec) * np.cos(a), math.cos(dec) * np.sin(a), math.sin(dec) * np.ones_like(a)])

    def muestras(self, t_s):
        """Dict de arreglos por instante: el, az (deg, rele), rango_km, visible, prx_dbm, tasa_bps, margen_db, fer,
        tierra_el, tierra_az, tierra_visible (rover ve la Tierra), rele_ve_tierra (rele no ocultado), tasa_x_bps."""
        t = np.asarray(t_s, dtype=float)
        gm, R = self.cuerpo['gm'], self.cuerpo['radio_km']
        up, este, norte = self._triedro(t)
        r_rele = posicion_orbita(gm, self.rele, t)
        r_rov = R * up
        d = r_rele - r_rov
        rango = np.linalg.norm(d, axis=0)
        u = d / rango
        el = np.degrees(np.arcsin(np.clip((u * up).sum(0), -1, 1)))
        az = np.degrees(np.arctan2((u * este).sum(0), (u * norte).sum(0))) % 360.0
        masc = _interp_mascara(self.mascara, az)
        visible = (el >= self.el_min) & (el >= masc)
        # presupuesto del enlace rover -> rele
        b = self.banda
        g_rele = ganancia_rele_dbi(b['g_rele_dbi'], el, np.linalg.norm(r_rele, axis=0), R)
        g_rov = ganancia_rover_uhf_dbi(self.antena, el, **self.antena_params)
        ptx_dbw = 10 * math.log10(b['p_tx_w']) - b['perd_tx_db']
        prx_dbw = ptx_dbw + g_rov + self.ganancia_extra_db + g_rele - fspl_db(rango, b['f_hz'])
        if self.es_uhf:
            prx_dbm = prx_dbw + 30.0 - b['perd_rx_db']
            tasa, marg = tasa_uhf(prx_dbm, b['margen_db'], self.rele.get('tasa_max_bps'))
        else:
            prx_dbm = prx_dbw + 30.0
            cn0 = prx_dbw - K_BOLTZMANN_DBW - 10.0 * math.log10(b['t_sys_k']) - 0.0
            tasa, marg = tasa_ebn0(cn0, b['ebn0_db'], b['margen_db'], b['escalera_bps'])
        tasa = np.where(visible, tasa, 0.0)
        fer = np.where(tasa > 0, prob_perdida_trama(marg), 1.0)
        # Tierra desde el rover (X directo) y desde el rele (ocultacion por el cuerpo)
        uT = self._dir_tierra(t)
        t_el = np.degrees(np.arcsin(np.clip((uT * up).sum(0), -1, 1)))
        t_az = np.degrees(np.arctan2((uT * este).sum(0), (uT * norte).sum(0))) % 360.0
        t_vis = (t_el >= self.el_min_tierra) & (t_el >= _interp_mascara(self.mascara, t_az))
        a = (r_rele * uT).sum(0)
        perp = np.linalg.norm(r_rele - a * uT, axis=0)
        rele_ve = ~((a < 0) & (perp < R))
        return dict(t=t, el=el, az=az, rango_km=rango, visible=visible, prx_dbm=prx_dbm, tasa_bps=tasa, margen_db=marg,
                    fer=fer, tierra_el=t_el, tierra_az=t_az, tierra_visible=t_vis, rele_ve_tierra=rele_ve)

    def tasa_x_bps(self, dist_km, g_dbi=None):
        """Tasa X directa [bps] a la distancia dada (0 si ninguna de la escalera sostiene el margen)."""
        b = self.banda_x
        g = ganancia_rover_x_dbi(self.antena_x, **self.antena_x_params) if g_dbi is None else g_dbi
        eirp = 10 * math.log10(b['p_tx_w']) + g + self.ganancia_extra_db
        cn0 = eirp + b['gt_dbk'] - fspl_db(dist_km, b['f_hz']) - b['perd_db'] - K_BOLTZMANN_DBW
        return tasa_ebn0(cn0, b['ebn0_db'], b['margen_db'], b['escalera_bps'])[0]

    def tasa_rele_tierra_bps(self, dist_km):
        r = self.rele
        return np.minimum(r['down_bps'] * (r['down_d_km'] / np.asarray(dist_km, dtype=float)) ** 2, r['down_max_bps'])

    # --- pases ---------------------------------------------------------------------------------------------------
    def pases(self, t0_s, dur_s, dt_s=30.0):
        """Lista de pases (visibilidad rover-rele) en [t0, t0+dur): inicio, fin, dur_s, el_max, rango_min, bits (utiles, tras
        eficiencia y perdida de tramas) y tasa pico."""
        t = t0_s + dt_s * np.arange(int(dur_s / dt_s))
        m = self.muestras(t)
        eta = self.banda['eficiencia']
        bits = m['tasa_bps'] * dt_s * eta * (1.0 - m['fer'])
        v = m['visible']
        out = []
        i, n = 0, len(t)
        while i < n:
            if not v[i]:
                i += 1
                continue
            j = i
            while j < n and v[j]:
                j += 1
            sl = slice(i, j)
            out.append(dict(inicio_s=float(t[i]), fin_s=float(t[j - 1] + dt_s), dur_s=float((j - i) * dt_s),
                            el_max_deg=float(m['el'][sl].max()), rango_min_km=float(m['rango_km'][sl].min()),
                            bits=float(bits[sl].sum()), tasa_pico_bps=float(m['tasa_bps'][sl].max())))
            i = j
        return out

    def volumen_por_dia(self, dias, dt_s=30.0, t0_s=0.0):
        """(bits utiles por dia (arreglo), numero de pases por dia) en `dias` dias del cuerpo, UHF/S rover->rele."""
        dia = self.cuerpo['dia_s']
        ps = self.pases(t0_s, dias * dia, dt_s)
        vol, npas = np.zeros(dias), np.zeros(dias)
        for p in ps:
            k = min(int((p['inicio_s'] - t0_s) // dia), dias - 1)
            vol[k] += p['bits']
            npas[k] += 1
        return vol, npas

    # --- retardo de comandos y telemetria ------------------------------------------------------------------------
    def horario(self, horizonte_s, dt_s=30.0):
        """Rejilla de contactos para `entrega_*`: instantes, visibilidad rover-rele, rele-Tierra (no ocultado y dentro del
        turno de la DSN, `dsn_fraccion` del dia al comienzo de cada dia), rover-Tierra y tasas."""
        clave = (float(horizonte_s), float(dt_s))
        if clave in self._cache:
            return self._cache[clave]
        t = dt_s * np.arange(int(horizonte_s / dt_s) + 1)
        m = self.muestras(t)
        dia = self.cuerpo['dia_s']
        turno = np.mod(t, dia) < self.dsn_fraccion * dia
        h = dict(t=t, dt=dt_s, rover_rele=m['visible'], rele_tierra=m['rele_ve_tierra'] & turno,
                 rover_tierra=m['tierra_visible'] & (np.mod(t, dia) < self.banda_x['horas_dia'] * 3600.0),
                 tasa_uhf=m['tasa_bps'], idx_rr=np.flatnonzero(m['visible']).tolist(),
                 idx_rt=np.flatnonzero(m['rele_ve_tierra'] & turno).tolist(),
                 idx_xt=np.flatnonzero(m['tierra_visible'] & (np.mod(t, dia) < self.banda_x['horas_dia'] * 3600.0)).tolist())
        self._cache[clave] = h
        return h

    def _owlt(self, t_s):
        return float(owlt_s(self.planeta, self.jd0 + t_s / 86400.0))

    def entrega_comando(self, t_envio_s, horizonte_s, bits=2000.0, dt_s=30.0, modo='ventanas', vias=('rele', 'directo')):
        """Instante de llegada de un comando enviado en la Tierra en `t_envio_s` y la via ('rele' | 'directo').
        'rele': Tierra -> rele (retardo de luz y espera de ventana rele-Tierra) -> almacenado -> rover en el siguiente
        contacto UHF (tasa de la escalera, un solo bundle). 'directo': banda X cuando el rover ve la Tierra. Gana la
        que llegue antes. `modo='retardo'`: solo el retardo de luz (enlace siempre disponible). (None, None) si no llega
        antes del horizonte. `vias` restringe las vias consideradas."""
        ow = self._owlt(t_envio_s)
        if modo == 'retardo':
            return t_envio_s + ow, 'retardo'
        h = self.horario(horizonte_s, dt_s)
        dt = h['dt']
        mejor, via = None, None
        # via directa por banda X
        ta = t_envio_s + ow
        k = _siguiente(h['idx_xt'], int(math.ceil(ta / dt))) if 'directo' in vias else None
        if k is not None:
            rx = float(self.tasa_x_bps(self._dist(ta)))
            if rx > 0:
                mejor, via = max(ta, k * dt) + bits / rx, 'directo'
        # via rele
        k1 = _siguiente(h['idx_rt'], int(math.ceil(ta / dt))) if 'rele' in vias else None
        if k1 is not None:
            k2 = _siguiente(h['idx_rr'], k1)
            while k2 is not None and h['tasa_uhf'][k2] <= 0:
                k2 = _siguiente(h['idx_rr'], k2 + 1)
            if k2 is not None:
                tl = k2 * dt + bits / float(h['tasa_uhf'][k2])
                if mejor is None or tl < mejor:
                    mejor, via = tl, 'rele'
        return mejor, via

    def _dist(self, t_s):
        return float(distancia_tierra_km(self.planeta, self.jd0 + t_s / 86400.0))

    def entrega_telemetria(self, t_gen_s, horizonte_s, bits=2400.0, dt_s=30.0, modo='ventanas'):
        """Instante de llegada a la Tierra de una muestra generada en el rover en `t_gen_s` (DTN por el rele: espera el
        contacto rover-rele, luego la ventana rele-Tierra, mas el retardo de luz). None si no llega antes del horizonte."""
        if modo == 'retardo':
            return t_gen_s + self._owlt(t_gen_s)
        h = self.horario(horizonte_s, dt_s)
        dt = h['dt']
        i0 = int(math.ceil(t_gen_s / dt))
        k = _siguiente(h['idx_rr'], i0)
        while k is not None and h['tasa_uhf'][k] <= 0:
            k = _siguiente(h['idx_rr'], k + 1)
        if k is None:
            return None
        t_rele = max(t_gen_s, k * dt) + bits / float(h['tasa_uhf'][k])
        k2 = _siguiente(h['idx_rt'], int(math.ceil(t_rele / dt)))
        if k2 is None:
            return None
        t_sal = max(t_rele, k2 * dt)
        return t_sal + self._owlt(t_sal) + bits / float(self.tasa_rele_tierra_bps(self._dist(t_sal)))


# ------------------------------------------------------------------------------------------------------------------
# Politica de comandos y telemetria (filtro del nodo; Python puro)
# ------------------------------------------------------------------------------------------------------------------
class FiltroEnlace:
    """Retrasa /rover/cmd_vel (Tierra -> rover) y /rover/estado (rover -> Tierra) segun el enlace.

    `cmd(t, valor)` y `estado(t, valor)` encolan; `listos(t)` devuelve (comandos, estados) cuyo instante de entrega ya paso,
    en orden de entrega (desempate por orden de llegada: determinista). `modo`: 'ideal' (sin retardo), 'retardo' (solo
    retardo de luz, enlace siempre disponible) o 'ventanas' (retardo + contactos del rele). Perdida: cada elemento se
    pierde con `p_perdida_cmd` / `p_perdida_estado` (semilla `semilla`); ademas lo que no llega antes del horizonte se
    descarta y se cuenta."""

    def __init__(self, enlace, modo='ventanas', horizonte_s=None, p_perdida_cmd=0.0, p_perdida_estado=0.0, semilla=0,
                 dt_s=30.0, capacidad_estados=5000):
        if modo not in ('ideal', 'retardo', 'ventanas'):
            raise ValueError(f'modo desconocido: {modo!r}')
        self.enlace, self.modo = enlace, modo
        self.horizonte_s = float(horizonte_s if horizonte_s is not None else 10 * enlace.cuerpo['dia_s'])
        self.p_cmd, self.p_est = p_perdida_cmd, p_perdida_estado
        self.rng = np.random.default_rng(semilla)
        self.dt_s = dt_s
        self.capacidad = int(capacidad_estados)
        self._cmds, self._ests, self._n = [], [], 0
        self.stats = dict(cmd_enviados=0, cmd_perdidos=0, cmd_no_llegan=0, est_enviados=0, est_perdidos=0, est_no_llegan=0,
                          est_desbordados=0)
        self.ultimo_via = None

    def _prog(self, cola, t_ent, valor):
        self._n += 1
        heapq.heappush(cola, (t_ent, self._n, valor))

    def cmd(self, t, valor):
        self.stats['cmd_enviados'] += 1
        if self.rng.random() < self.p_cmd:
            self.stats['cmd_perdidos'] += 1
            return None
        if self.modo == 'ideal':
            t_ent = t
        else:
            t_ent, self.ultimo_via = self.enlace.entrega_comando(t, self.horizonte_s, dt_s=self.dt_s, modo=self.modo)
        if t_ent is None:
            self.stats['cmd_no_llegan'] += 1
            return None
        self._prog(self._cmds, t_ent, valor)
        return t_ent

    def estado(self, t, valor):
        self.stats['est_enviados'] += 1
        if self.rng.random() < self.p_est:
            self.stats['est_perdidos'] += 1
            return None
        if self.modo == 'ideal':
            t_ent = t
        else:
            t_ent = self.enlace.entrega_telemetria(t, self.horizonte_s, dt_s=self.dt_s, modo=self.modo)
        if t_ent is None:
            self.stats['est_no_llegan'] += 1
            return None
        if len(self._ests) >= self.capacidad:          # almacenamiento acotado: se descarta lo mas viejo en cola
            self._ests.remove(min(self._ests))
            heapq.heapify(self._ests)
            self.stats['est_desbordados'] += 1
        self._prog(self._ests, t_ent, valor)
        return t_ent

    def listos(self, t):
        c, e = [], []
        while self._cmds and self._cmds[0][0] <= t:
            c.append(heapq.heappop(self._cmds))
        while self._ests and self._ests[0][0] <= t:
            e.append(heapq.heappop(self._ests))
        return [(a, v) for a, _, v in c], [(a, v) for a, _, v in e]

    def pendientes(self):
        return len(self._cmds), len(self._ests)


# ------------------------------------------------------------------------------------------------------------------
# DTN: cola de bundles con almacenamiento acotado y retransmision por pase
# ------------------------------------------------------------------------------------------------------------------
class ColaDTN:
    """Cola FIFO de bundles (id, bits, t_creacion) con capacidad `capacidad_bits`; al desbordar se descarta el mas antiguo.
    `transmitir(capacidad_bits, t, rng, p_perdida)` envia en orden; un bundle terminado se pierde con probabilidad
    `p_perdida` y vuelve al frente (retransmision desde cero en el siguiente contacto) gastando la capacidad."""

    def __init__(self, capacidad_bits):
        self.cap = float(capacidad_bits)
        self.q = deque()
        self.bits = 0.0
        self.descartados = 0
        self.retransmisiones = 0
        self._id = 0

    def agregar(self, bits, t, creado=None):
        self._id += 1
        self.q.append([self._id, float(bits), float(bits), t if creado is None else creado])
        self.bits += bits
        while self.bits > self.cap and len(self.q) > 1:
            b = self.q.popleft()
            self.bits -= b[1]
            self.descartados += 1
        return self._id

    def transmitir(self, capacidad_bits, t, rng, p_perdida=0.0):
        """Devuelve la lista de (id, t_creacion, t_entrega) entregados en este contacto."""
        hechos = []
        cap = float(capacidad_bits)
        while self.q and cap > 0:
            b = self.q[0]
            gasto = min(cap, b[2])
            b[2] -= gasto
            cap -= gasto
            if b[2] > 1e-9:
                break
            if p_perdida > 0 and rng.random() < p_perdida:
                b[2] = b[1]
                self.retransmisiones += 1
                continue
            self.q.popleft()
            self.bits -= b[1]
            hechos.append((b[0], b[3], t))
        return hechos


def simular_dtn(enlace, dur_s, gen_bits_por_dia, bundle_bits=8e6, capacidad_bits=16e9, p_perdida_bundle=0.01, semilla=0,
                dt_s=30.0):
    """Telemetria del rover por DTN: genera `gen_bits_por_dia` en bundles de `bundle_bits`, los pasa al rele en cada
    pase (tasa x eficiencia x (1 - fer)) y el rele los baja a la Tierra en sus ventanas (tasa a Tierra x turno DSN). Devuelve
    metricas: bits entregados por dia, latencia de bundle (mediana, p95, max, en horas) incluyendo el retardo de luz,
    descartados y retransmisiones. Todo SUPUESTO salvo la geometria."""
    rng = np.random.default_rng(semilla)
    h = enlace.horario(dur_s, dt_s)
    t = h['t']
    m = enlace.muestras(t)
    eta = enlace.banda['eficiencia']
    cap_rr = m['tasa_bps'] * dt_s * eta * (1.0 - m['fer']) * h['rover_rele']
    dist = distancia_tierra_km(enlace.planeta, enlace.jd0 + t / 86400.0)
    cap_rt = enlace.tasa_rele_tierra_bps(dist) * dt_s * h['rele_tierra']
    owl = dist / C_KM_S
    dia = enlace.cuerpo['dia_s']
    por_gen = bundle_bits / (gen_bits_por_dia / dia)
    rover, rele = ColaDTN(capacidad_bits), ColaDTN(capacidad_bits)
    proximo, ent = por_gen, []
    for i, ti in enumerate(t):
        while proximo <= ti:
            rover.agregar(bundle_bits, proximo)
            proximo += por_gen
        if cap_rr[i] > 0:
            for bid, tc, te in rover.transmitir(cap_rr[i], ti, rng, p_perdida_bundle):
                rele.agregar(bundle_bits, ti, creado=tc)
        if cap_rt[i] > 0:
            for bid, tc, te in rele.transmitir(cap_rt[i], ti, rng, 0.0):
                ent.append((tc, ti + owl[i]))
    lat_h = np.array([(te - tc) / 3600.0 for tc, te in ent]) if ent else np.zeros(0)
    dias = dur_s / dia
    return dict(bundles_entregados=len(ent), bits_entregados_por_dia=len(ent) * bundle_bits / dias,
                bits_generados_por_dia=gen_bits_por_dia, latencia_mediana_h=float(np.median(lat_h)) if len(lat_h) else None,
                latencia_p95_h=float(np.percentile(lat_h, 95)) if len(lat_h) else None,
                latencia_max_h=float(lat_h.max()) if len(lat_h) else None, descartados=rover.descartados + rele.descartados,
                retransmisiones=rover.retransmisiones, en_cola_rover=len(rover.q), en_cola_rele=len(rele.q))


# ------------------------------------------------------------------------------------------------------------------
# Escena (kind: proceso, area rf): resumen de enlaces de un rover durante `dias` dias
# ------------------------------------------------------------------------------------------------------------------
DEFECTOS_ENLACE = dict(planeta='marte', rele='mro', lat_deg=-4.6, lon_deg=137.4, terreno='llano', semilla=0, antena='omni_helice',
                       el_min_deg=10.0, dias=5, fecha='2026-10-07', dt_s=30.0, gen_mbit_dia=300.0, terreno_params={})


def correr_enlace(cfg, carpeta=None):
    """Simula `dias` dias de enlace y escribe `enlace.json` (pases) y `metrics.json` (resumen) en `carpeta`. Devuelve el resumen."""
    c = {**DEFECTOS_ENLACE, **cfg}
    terreno = None
    if c['terreno'] != 'llano':
        from atp_rover.terreno import Terreno
        tp = dict(c['terreno_params'])
        if c['terreno'] == 'crater' and not tp:        # rover en el centro de un hoyo de 15 m x 3 m (no a 25 m del borde)
            tp = dict(cx=0.0, cy=0.0, radio=15.0, profundidad=3.0)
        terreno = Terreno(c['terreno'], c['semilla'], **tp)
    e = Enlace(c['planeta'], c['rele'], c['lat_deg'], c['lon_deg'], terreno, c['antena'], el_min_deg=c['el_min_deg'],
               jd0=jd_de_iso(c['fecha']))
    dia = e.cuerpo['dia_s']
    dias = int(c['dias'])
    ps = e.pases(0.0, dias * dia, c['dt_s'])
    vol, npas = e.volumen_por_dia(dias, c['dt_s'])
    dtn = simular_dtn(e, dias * dia, c['gen_mbit_dia'] * 1e6, semilla=int(c['semilla']), dt_s=c['dt_s'])
    met = dict(planeta=c['planeta'], rele=e.rele_nombre, antena=c['antena'], terreno=c['terreno'], dias=dias,
               pases_por_dia=float(npas.mean()), volumen_mbit_por_dia=float(vol.mean() / 1e6),
               dur_pase_media_min=float(np.mean([p['dur_s'] for p in ps]) / 60.0) if ps else 0.0,
               el_max_media_deg=float(np.mean([p['el_max_deg'] for p in ps])) if ps else 0.0,
               retardo_luz_una_via_min=float(e._owlt(0.0) / 60.0), dtn=dtn)
    if carpeta:
        os.makedirs(carpeta, exist_ok=True)
        with open(os.path.join(carpeta, 'enlace.json'), 'w') as f:
            json.dump(dict(pases=ps, volumen_bits_por_dia=vol.tolist()), f, sort_keys=True)
        with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
            json.dump(met, f, sort_keys=True, indent=1)
    return met


def main(argv=None):
    import argparse
    import sys
    ap = argparse.ArgumentParser(allow_abbrev=False, description='Escena de enlace del rover (CONTRATO sec. 19)')
    ap.add_argument('--escena')
    ap.add_argument('--salida', default='/tmp/rover_enlace')
    ap.add_argument('--id', default='enlace')
    a, resto = ap.parse_known_args(argv)
    cfg = {}
    if a.escena:
        import yaml
        with open(a.escena) as f:
            cfg = dict((yaml.safe_load(f).get('args') or {}))
    i = 0
    while i < len(resto):
        r = resto[i]
        i += 1
        if not r.startswith('--'):
            ap.error(f'argumento no reconocido: {r}')
        if '=' in r:
            k, v = r[2:].split('=', 1)
        elif i < len(resto):
            k, v = r[2:], resto[i]
            i += 1
        else:
            ap.error(f'falta el valor de {r}')
        if k not in DEFECTOS_ENLACE:
            ap.error(f'argumento desconocido: --{k} (hay: {", ".join(sorted(DEFECTOS_ENLACE))})')
        try:
            cfg[k] = json.loads(v)
        except ValueError:
            cfg[k] = v
    met = correr_enlace(cfg, os.path.join(a.salida, a.id))
    print('RESUMEN ' + json.dumps(met))
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
