"""Escenario completo DETUMBLE -> NADIR -> TRACK_GS sin ROS (y su CLI).

`preparar` elige el pase real (SGP4 + data/tle + data/estaciones.yaml, con
`orbit.py`) y `simular` encadena planta, controlador y medidor en el mismo
orden que el lockstep de ROS. Sirve para las pruebas rapidas y para
cruzar el RESUMEN de la corrida ROS: deben coincidir.

    python3 -m sat_adcs.escenario --sat FUNCUBE-1 --estacion cdmx \
        --inicio_utc 2026-10-02T00:00:00
"""
import argparse
import dataclasses
import datetime as dt
import math
import sys
import time
from dataclasses import dataclass

import numpy as np

from atp_ros import orbit
from sat_adcs import fisica as F
from sat_adcs.misiones import (Config, ControladorAdcs, Efemeride, Medidor,
                               PlantaAdcs)


@dataclass
class Escenario:
    cfg: Config
    sat: F.Satelite
    tle: object
    estacion: object
    t0: dt.datetime
    efem: Efemeride
    pase: dict
    aos_s: float
    los_s: float
    t_fin_s: float
    edad_tle_dias: float


def cargar_base(cfg, sat=None, tle=None, estacion=None, dir_tle=None):
    """(sat, tle, estacion, t0, efemeride): lo que comparten planta, controlador
    y monitor (la misma orbita, sin elegir pase)."""
    sat = sat or F.cubesat_3u()
    tle = tle or orbit.buscar_tle(orbit.cargar_catalogo(dir_tle), cfg.sat)
    estacion = estacion or orbit.buscar_estacion(cfg.estacion)
    t0 = orbit.a_utc(cfg.inicio_utc)
    return sat, tle, estacion, t0, Efemeride(tle, t0)


def preparar(cfg, sat=None, tle=None, estacion=None, dir_tle=None):
    """Elige el pase `pase_idx`-esimo con AOS >= t0 + anticipo y elevacion
    maxima >= el_max_min_deg, y fija el fin de la corrida (LOS + cola).
    Lanza LookupError si no hay. Vale lo de `orbit.predecir_pases` (TLE
    cerca de su epoca: avisa si pasa de 7 dias)."""
    sat, tle, estacion, t0, efem = cargar_base(cfg, sat, tle, estacion, dir_tle)
    horas = cfg.anticipo_min_s / 3600.0 + 72.0
    pases = [p for p in orbit.predecir_pases(tle, estacion, t0, horas,
                                              el_max_min_deg=cfg.el_max_min_deg)
             if (p['aos'] - t0).total_seconds() >= cfg.anticipo_min_s
             and not p['aos_truncado'] and not p['los_truncado']]
    if not 0 <= cfg.pase_idx < len(pases):
        raise LookupError(f'{tle.nombre} sobre {estacion.nombre}: solo hay {len(pases)} '
                          f'pases con el_max >= {cfg.el_max_min_deg} desde {cfg.inicio_utc}')
    p = pases[cfg.pase_idx]
    aos = (p['aos'] - t0).total_seconds()
    los = (p['los'] - t0).total_seconds()
    return Escenario(cfg, sat, tle, estacion, t0, efem, p, aos, los,
                     los + cfg.cola_s, tle.edad_dias(p['aos']))


def crear(esc):
    """(planta, controlador, medidor) del escenario."""
    cfg = esc.cfg
    planta = PlantaAdcs(cfg, esc.sat, esc.efem)
    ctrl = ControladorAdcs(cfg, esc.sat, esc.efem, esc.estacion)
    med = Medidor(cfg, esc.sat, esc.efem, esc.estacion)
    return planta, ctrl, med


def simular(esc, t_fin_s=None, progreso=None):
    """Corre el escenario hasta `t_fin_s` (por omision, el del pase).
    Devuelve (medidor, planta, controlador)."""
    planta, ctrl, med = crear(esc)
    paso = esc.cfg.paso_s
    t_fin = esc.t_fin_s if t_fin_s is None else t_fin_s
    n = int(math.ceil(t_fin / paso - 1e-9))       # el primer paso con t >= t_fin (como la planta ROS)
    for k in range(n + 1):
        s = planta.medir()
        cmd = ctrl.paso(s['t'], s)
        med.muestra(s, cmd)
        if k == n:
            break
        planta.avanzar(paso, cmd['u_w'], cmd['m'])
        if progreso and k % 600 == 0:
            progreso(k * paso, t_fin)
    return med, planta, ctrl


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    for f in dataclasses.fields(Config):
        if f.type is bool:
            continue
        ap.add_argument('--' + f.name, type=f.type, default=f.default)
    a = ap.parse_args(argv)
    cfg = Config(**{f.name: getattr(a, f.name) for f in dataclasses.fields(Config)})
    esc = preparar(cfg)
    p = esc.pase
    print(f"pase: {esc.tle.nombre} sobre {esc.estacion.nombre}: AOS {p['aos']:%Y-%m-%d %H:%M:%S} UTC "
          f"(t={esc.aos_s:.0f} s), duracion {p['duracion_s']:.0f} s, el max {p['el_max_deg']:.1f} deg")
    t_wall = time.time()
    med, _, _ = simular(esc, progreso=lambda t, T: print(f'  t={t:.0f}/{T:.0f} s', file=sys.stderr))
    for linea in med.resumen():
        print(linea)
    print(f'(tiempo de pared {time.time() - t_wall:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
