"""Escenario ISL sin ROS (y su CLI): encadena planta, controlador y medidor en el
mismo orden que el lockstep de ROS, asi que da el MISMO RESUMEN.

    python3 -m sat_isl.escenario                       # par del mismo plano (P0S0-P0S1)
    python3 -m sat_isl.escenario --sat_b 11 --inicio_s 900   # par de planos distintos
    python3 -m sat_isl.escenario --tabla               # tabla de vecinos de la constelacion
"""
import argparse
import dataclasses
import sys
import time

import numpy as np

from sat_isl import geometria as G
from sat_isl.misiones import Config, ControladorIsl, Medidor, PlantaIsl, constelacion_de


def simular(cfg, const=None, progreso=False):
    """Corre el escenario entero (dur_s) y devuelve el `Medidor`."""
    const = const or constelacion_de(cfg)
    planta = PlantaIsl(cfg, const)
    ctrl = ControladorIsl(cfg, const)
    med = Medidor(cfg)
    n = int(round(cfg.dur_s / cfg.paso_s))
    for k in range(n + 1):
        st = planta.medir()
        cmd = ctrl.paso(st['t'], st)
        med.muestra(st, cmd)
        if k < n:
            planta.avanzar(cfg.paso_s, cmd)
        if progreso and k % 200 == 0:
            print(f't={st["t"]:.1f} modo={st["modo"]}', file=sys.stderr)
    return med


def tabla_vecinos(const, i=0, dur_s=6600.0, paso_s=30.0):
    """Filas (vecino, rango min/max, % visible, v_perp max, tasa de la linea de
    vista max, adelanto max) del satelite `i` contra cada candidato, sobre una orbita."""
    ts = np.arange(0.0, dur_s, paso_s)
    filas = []
    for j in const.vecinos(i):
        gs = [const.enlace(i, j, t) for t in ts]
        vis = np.array([g['visible'] for g in gs])
        filas.append({'de': const.elementos[i].nombre, 'a': const.elementos[j].nombre, 'j': j,
                      'rango_min_km': min(g['rango_km'] for g in gs),
                      'rango_max_km': max(g['rango_km'] for g in gs),
                      'visible_pct': 100.0 * vis.mean(),
                      'v_perp_max_km_s': max(g['v_perp_km_s'] for g in gs),
                      'tasa_los_max_mrad_s': 1e3 * max(g['tasa_los_rad_s'] for g in gs),
                      'adelanto_max_urad': 1e6 * max(g['adelanto_rad'] for g in gs)})
    return filas


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for f in dataclasses.fields(Config):
        ap.add_argument('--' + f.name, type=f.type, default=f.default)
    ap.add_argument('--tabla', action='store_true', help='tabla de vecinos del satelite sat_a y salir')
    a = ap.parse_args(argv)
    cfg = Config(**{f.name: getattr(a, f.name) for f in dataclasses.fields(Config)})
    const = constelacion_de(cfg)
    if a.tabla:
        print(f'Walker {cfg.n_sats}/{cfg.planos}/{cfg.fase} a {cfg.h_km:.0f} km, {cfg.inc_deg:.0f} deg; '
              f'cuerda maxima con rayo >= {G.H_RAYO_MIN_KM:.0f} km: '
              f'{G.rango_maximo_visible_km(cfg.h_km):.0f} km ({G.separacion_angular_max_deg(cfg.h_km):.1f} deg)')
        for r in tabla_vecinos(const, cfg.sat_a):
            print('{de}->{a} (idx {j}): rango {rango_min_km:.0f}-{rango_max_km:.0f} km, visible {visible_pct:.0f} %, '
                  'v_perp max {v_perp_max_km_s:.2f} km/s, LOS {tasa_los_max_mrad_s:.2f} mrad/s, '
                  'adelanto max {adelanto_max_urad:.1f} urad'.format(**r))
        return 0
    t0 = time.time()
    med = simular(cfg, const)
    for linea in med.resumen():
        print(linea)
    print(f'({time.time() - t0:.1f} s de pared)', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
