"""El mundo de la mision: lo que el operador SABE (los TLE) y lo que ES (la verdad).

Una mision integrada solo tiene sentido si la prediccion y la realidad difieren,
si no el seguimiento no tiene nada que corregir y la OD nada que mejorar. Aqui
cada satelite tiene:

  * lo que el operador conoce: el TLE de `data/tle` (SGP4);
  * la VERDAD: el escenario de `atp_ros.od_experimento.Escenario` (docs/OD.md 3.1):
    el estado SGP4 de la epoca del TLE mas un desfase de epoca (el error propio de
    un TLE: ~1 km a lo largo de la pista), propagado con un propagador numerico
    mas rico que el del estimador (J4, arrastre con BC +25 % y variacion temporal
    de la densidad). Con eso el TLE se aleja ~1.6 km a 0.5 d, 2 km a 1 d y 6 km a
    2 d, lo documentado en `orbit.py`.

La verdad se guarda como una efemeride (`od.guardar_efemeride`, .npz) que lee el
generador de referencia (`truth_file`). El codigo de la mision NUNCA la usa para
estimar ni para decidir: solo la corrida (lo que ve la estacion) y el informe
(para medir el error de la efemeride que el operador uso).

Supuestos (docs/MISION.md): el desfase de epoca, el +25 % de BC y la variacion de
densidad son los del escenario de la OD (supuestos, no medidas); el BC de cada
satelite es una suposicion (ISS: 2.2*1600 m2/335 000 kg calibrado contra SGP4;
polares a 800-850 km: 0.0119 m2/kg, el de NOAA 19 en docs/OD.md).
"""
import math
import os
import re
import threading
from dataclasses import dataclass

import numpy as np

from atp_ros import medidas as M
from atp_ros import od
from atp_ros import od_experimento as X
from atp_ros import orbit
from atp_ros import planificador as pl

BC_ISS = 0.0105           # Cd A / m (m2/kg), docs/OD.md
P_CAL_ISS = 0.406         # BC efectivo calibrado contra SGP4 en 24 h (docs/OD.md 1.2)
BC_POLAR = 0.0119         # NOAA 19, docs/OD.md ("una suposicion: 2.2 x 8 m2 / 1479 kg")
SIGMAS_OD = dict(M.SIGMAS_DEFECTO)


@dataclass(frozen=True)
class Carga:
    """Carga util de bajada de UN satelite (HIPOTETICA: ni la ISS ni los NOAA/METEOR
    bajan con estos EIRP y tasas) y su dinamica (BC) para la verdad."""
    banda: str = 'X'
    eirp_dbw: float = 12.0
    rs_baud: float = 10e6
    bc: float = BC_POLAR
    p_cal: float = 1.0

    def perfil_enlace(self, margen_acm_db=1.0):
        return pl.PerfilEnlace(banda=self.banda, eirp_dbw=self.eirp_dbw,
                               rs_baud=self.rs_baud, margen_acm_db=margen_acm_db)


# Red por defecto de la mision M1 (8 satelites, bandas realistas por familia): la
# ISS y los NOAA 15/18/19 bajan en S (la banda S/L de la que son capaces); NOAA 20/21
# y METEOR-M2 en X. Los numeros son hipoteticos, ver docs/MISION.md.
CARGAS_M1 = {
    'ISS (ZARYA)': Carga('S', 10.0, 4e6, BC_ISS, P_CAL_ISS),
    'NOAA 15': Carga('S', 10.0, 4e6),
    'NOAA 18': Carga('S', 10.0, 4e6),
    'NOAA 19': Carga('S', 10.0, 4e6),
    'NOAA 20 (JPSS-1)': Carga('X', 12.0, 10e6),
    'NOAA 21 (JPSS-2)': Carga('X', 12.0, 10e6),
    'METEOR-M2 3': Carga('X', 12.0, 10e6),
    'METEOR-M2 4': Carga('X', 12.0, 10e6),
}
CARGA_DEFECTO = Carga()


def slug(nombre):
    """Nombre de satelite -> parte de nombre de archivo."""
    return re.sub(r'[^a-z0-9]+', '_', nombre.lower()).strip('_')


class Mundo:
    """Verdad + TLE de cada satelite de la mision.

    `tles`: lista de `orbit.Tle`. `estaciones`: dict clave -> `orbit.Estacion`.
    `t_fin`: ultimo instante que se volara (la verdad se integra hasta ahi).
    `cargas`: dict nombre -> `Carga` (por defecto `CARGAS_M1`, luego la generica)."""

    def __init__(self, tles, estaciones, t_fin, cargas=None, carpeta=None):
        self.estaciones = dict(estaciones)
        self.claves = list(self.estaciones)
        self.cargas = dict(CARGAS_M1)
        self.cargas.update(cargas or {})
        self.carpeta = os.path.abspath(carpeta) if carpeta else carpeta
        self.tles = {t.nombre: t for t in tles}
        self.esc = {}
        self.t_fin = t_fin
        self._lock = threading.RLock()
        self._efemerides = {}
        for tle in tles:
            c = self.carga(tle.nombre)
            dias = max(0.5, (t_fin - tle.epoca).total_seconds() / 86400.0 + 0.25)
            self.esc[tle.nombre] = X.Escenario(
                tle, self.estaciones, dias=math.ceil(dias * 4) / 4.0, p_cal=c.p_cal,
                bc_fisico=c.bc)

    def carga(self, sat):
        return self.cargas.get(sat, CARGA_DEFECTO)

    def archivo_verdad(self, sat):
        """Ruta del .npz de la verdad de `sat` (se escribe la primera vez)."""
        ruta = os.path.join(self.carpeta, f'{slug(sat)}.npz')
        if not os.path.exists(ruta):
            os.makedirs(self.carpeta, exist_ok=True)
            e = self.esc[sat]
            od.guardar_efemeride(e.ef, e.tle.epoca, ruta, sat)
        return ruta

    # ------------------------------------------------------------ evaluacion (solo informe)
    def efemeride_orbita(self, sat, od_json):
        """Efemeride numerica de una orbita estimada (dict o ruta) con el eje de tiempo
        del escenario de `sat` (s desde la epoca de su TLE)."""
        clave = (sat, od_json if isinstance(od_json, str) else id(od_json))
        with self._lock:
            if clave in self._efemerides:
                return self._efemerides[clave]
            e = self.esc[sat]
            x, modelo, reloj, _ = od.orbita_de_json(od_json)
            dt0 = e.reloj.segundos(reloj.epoca)       # epoca de la OD en el eje del TLE
            t_fin = e.reloj.segundos(self.t_fin) - dt0 + 120.0
            ef = od.efemeride(modelo, x, 0.0, 0.0, max(t_fin, 600.0), 30.0)
            self._efemerides[clave] = _Desfasada(ef, dt0)
            return self._efemerides[clave]

    def error_efemeride(self, sat, estacion, aos, duracion_s, ef=None):
        """Desapuntamiento (deg) de una antena que apuntara a la efemeride `ef` (por
        defecto SGP4 con el TLE) hacia el satelite VERDADERO en el pase [aos, aos +
        duracion] sobre `estacion`: dict aos/max/rms/p95/medio (`od.resumen_error_pase`)."""
        e = self.esc[sat]
        sit = e.sitios[e.isit(estacion)]
        pase = {'aos': e.reloj.segundos(aos), 'los': e.reloj.segundos(aos) + duracion_s}
        return od.resumen_error_pase(ef if ef is not None else e.sgp4, e.ef, e.reloj, sit, pase)

    def error_posicion_km(self, sat, ef, t_utc):
        """|r_pred - r_verdad| (km) en `t_utc`."""
        e = self.esc[sat]
        t = np.array([e.reloj.segundos(t_utc)])
        r1, _ = ef.estado(t)
        r2, _ = e.ef.estado(t)
        return float(np.linalg.norm(r1[0] - r2[0]))


class _Desfasada:
    """Efemeride con el tiempo desplazado `dt0` (la de la OD tiene su propia epoca)."""

    def __init__(self, ef, dt0):
        self.ef, self.dt0 = ef, float(dt0)

    def estado(self, t):
        return self.ef.estado(np.atleast_1d(np.asarray(t, float)) - self.dt0)
