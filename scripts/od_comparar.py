#!/usr/bin/env python3
"""Compara el pase predicho por una ORBITA ESTIMADA (od.json del nodo
od_estimator) con el mismo pase del simulador (SGP4 + TLE), que en el lazo
ROS hace de verdad. Sirve de verificacion de punta a punta de la cadena
"medir -> estimar -> publicar -> predecir el pase siguiente" (docs/OD.md).

Uso: python3 scripts/od_comparar.py od.json --sat ISS --station cdmx \
         --start-utc 2026-10-03T00:00:00 [--pass-index 0]

Imprime la diferencia de AOS/LOS y el desapuntamiento (grados) que tendria la
antena si siguiera la prediccion de la orbita estimada en vez del TLE, en
cada instante del pase del TLE.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'src', 'atp_ros'))

from atp_ros import medidas as M    # noqa: E402
from atp_ros import od, orbit       # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('od_json')
    ap.add_argument('--sat', default='ISS')
    ap.add_argument('--station', default='')
    ap.add_argument('--start-utc', default='')
    ap.add_argument('--pass-index', type=int, default=0)
    ap.add_argument('--horas', type=float, default=72.0)
    a = ap.parse_args()
    est_arg = a.station or None
    ini = a.start_utc or None
    perfil_t, pase_t, tle, est = orbit.perfil_tle(a.sat, est_arg, a.pass_index, ini, a.horas)
    perfil_o, pase_o, orb, _ = od.perfil_od(a.od_json, est_arg, a.pass_index, ini, a.horas)
    t_t = (pase_t['aos'] - pase_o['aos']).total_seconds() + perfil_t['t']
    # perfil OD en los instantes del pase del TLE (az en [0, 360): se interpola el desenroscado)
    az_o = np.interp(t_t, perfil_o['t'], perfil_o['az_continuo'])
    el_o = np.interp(t_t, perfil_o['t'], perfil_o['el'])
    az_t = np.radians(perfil_t['az'])
    err = M.separacion_deg(np.radians(az_o) % (2 * np.pi), np.radians(el_o), az_t,
                           np.radians(perfil_t['el']))
    edad = (pase_t['aos'] - orbit.a_utc(orb['epoca_utc'])).total_seconds() / 86400.0
    print(f'ORBITA ESTIMADA vs TLE ({tle.nombre} sobre {est.nombre}): orbita de '
          f'{orb["epoca_utc"]} (sigma_pos formal {orb["sigma_pos_km"]:.3f} km), '
          f'pase a {edad:.2f} dias de ella')
    print(f'  AOS TLE {pase_t["aos"]:%H:%M:%S.%f}  AOS OD {pase_o["aos"]:%H:%M:%S.%f}  '
          f'(dif {(pase_o["aos"] - pase_t["aos"]).total_seconds():+.2f} s); '
          f'el max {pase_t["el_max_deg"]:.1f} / {pase_o["el_max_deg"]:.1f} deg')
    print(f'DESAPUNTAMIENTO OD-vs-TLE: aos={err[0]:.4f} rms={np.sqrt(np.mean(err ** 2)):.4f} '
          f'max={err.max():.4f} deg (haz Ka 0.233 deg, S 3.18 deg)')


if __name__ == '__main__':
    main()
