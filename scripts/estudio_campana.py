#!/usr/bin/env python3
"""Estudio (d): campana Monte Carlo de PID / LQR / LQR+Kalman (docs/CONTROL.md).

Python puro (sin ROS): ./run.sh python3 scripts/estudio_campana.py [n] [config...]
Misma semilla, mismos n pases al azar y mismas perturbaciones para los tres
controladores (comparacion pareada). p95 contra el objetivo de 0.1 grados.
"""
import sys
import time

import numpy as np

from atp_ros import control, model

REAL = dict(coulomb_nm=0.05, stribeck_nm=0.02, backlash_deg=0.05, encoder_bits=14)
CONFIGS = {
    # d1: las perturbaciones de siempre de campana_lazo (sesgo, ruido, latencia, rafaga)
    'd1_nominal': dict(),
    # d2: ademas planta realista: backlash 0.05 deg, Coulomb+Stribeck, encoder de 14 bits
    'd2_realista': dict(**REAL),
    # d3: d2 + desbalance de elevacion 0.05 N m + viento Dryden de 0.02 N m rms
    'd3_realista_viento': dict(**REAL, unbalance_nm=0.05, viento_sigma_nm=0.02),
    # d4: d3 sin sesgo de encoder ni latencia: lo que queda es del controlador y la planta
    'd4_sin_sesgo_ni_latencia': dict(**REAL, unbalance_nm=0.05, viento_sigma_nm=0.02,
                                      sesgo_max_deg=0.0, latencia_max_s=0.0),
}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    nombres = sys.argv[2:] or list(CONFIGS)
    base = model.campana_lazo(n=n, semilla=2029)           # referencia: el PID de model.py
    pc = model.percentiles(base['rms'])
    print(f'referencia model.campana_lazo n={n}: p50={pc["p50"]:.4f} p95={pc["p95"]:.4f} '
          f'p99={pc["p99"]:.4f} peor={pc["peor"]:.4f}')
    print(f'incertidumbre del p95: +-{model.incertidumbre_percentil(n):.1f} puntos percentiles')
    print('config | controlador | p50 | p95 | p99 | peor | pasa p95<0.1 | %pases<0.1 rms')
    for nom in nombres:
        for ctl in control.CONTROLADORES:
            t0 = time.monotonic()
            r = control.campana_lazo_ctrl(ctl, n=n, semilla=2029, **CONFIGS[nom])
            p = model.percentiles(r['rms'])
            dentro = 100.0 * float(np.mean(r['rms'] < model.OBJETIVO_DEG))
            print(f'{nom} | {ctl} | {p["p50"]:.4f} | {p["p95"]:.4f} | {p["p99"]:.4f} | '
                  f'{p["peor"]:.4f} | {"SI" if p["pasa"] else "NO"} | {dentro:.1f} '
                  f'({time.monotonic() - t0:.0f} s)', flush=True)


if __name__ == '__main__':
    main()
