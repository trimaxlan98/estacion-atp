#!/usr/bin/env python3
"""Genera los vectores de referencia del lazo del ESP32 desde atp_ros.

La planta es LA de atp_ros: `atp_ros.planta.PlantaMontura` (rama sin capas
opcionales) con J, b de `atp_ros.model` (J_EJE = 2 kg m^2, B_EJE = 0.5 N m s/rad).
El PID es una reimplementacion en numpy (float64) del de
`atp_ros/pid_controller.py` (derivada del error filtrada, saturacion y
anti-windup por clamping), con el lazo corrido a 1 kHz como en el firmware.
El C (test_vectores.c) corre su PID en float contra su planta en double y se
compara con estos CSV.

Uso (desde la raiz del repo, con numpy):
    PYTHONPATH=src/atp_ros python3 firmware/esp32_eje/test/gen_vectores.py
    ... --comprobar    # falla si los CSV del repo no son los que salen hoy

Formato de cada CSV: cabecera `# clave=valor ...` con los parametros del
escenario, y filas `k,theta,omega,u` cada DECIMACION muestras del lazo de 1 kHz
(theta, omega ANTES de aplicar u_k; u_k es el mando calculado en ese instante).
"""
import math
import os
import sys

import numpy as np

from atp_ros import model
from atp_ros.planta import PlantaMontura, cuantizar

H = 0.001                  # s, periodo del lazo (1 kHz)
DECIMACION = 50            # una fila cada 50 ms (20 Hz), para que el repo no pese
KP, KI, KD, TAU = 25.0, 2.0, 8.0, 0.08     # defaults de pid_controller.py
U_MAX = 8.0                # N m, u_max de mount_sim.py / pid_controller.py
AQUI = os.path.dirname(os.path.abspath(__file__))
DIR = os.path.join(AQUI, 'vectores')

# nombre: parametros. tipo: paso | rampa | ctrl_lazo (planta en lazo abierto)
ESCENARIOS = {
    'planta_abierta': dict(tipo='abierta', u=1.0, t_fin=5.0),
    'paso_5deg': dict(tipo='paso', ref=math.radians(5.0), t_fin=6.0),
    'rampa_9degs': dict(tipo='rampa', vel=math.radians(9.0), t_fin=20.0),
    'paso_90deg_saturado': dict(tipo='paso', ref=math.radians(90.0), t_fin=20.0),
    'rampa_cuantizada_16bits': dict(tipo='rampa', vel=math.radians(9.0), t_fin=10.0, bits=16),
    'rafaga_1nm': dict(tipo='paso', ref=math.radians(10.0), t_fin=10.0,
                       rafaga_t0=3.0, rafaga_dur=2.0, rafaga_nm=1.0),
}


def referencia(p, t):
    if p['tipo'] == 'paso':
        return p['ref']
    if p['tipo'] == 'rampa':
        return p['vel'] * t
    return 0.0


def correr(p):
    planta = PlantaMontura(model.J_EJE, model.B_EJE, h=H)   # sin capas: camino original
    planta.enganchar([0.0])
    integral = e_prev = d_filt = 0.0
    n = int(round(p['t_fin'] / H))
    filas = []
    for k in range(n):
        t = k * H
        theta, omega = float(planta.theta[0]), float(planta.omega[0])
        if p['tipo'] == 'abierta':
            u = p['u']
        else:
            y = theta
            if 'bits' in p:
                y = float(cuantizar(theta, p['bits']))
            e = referencia(p, t) - y
            d_bruta = (e - e_prev) / H
            alfa = H / (TAU + H)
            d_filt += alfa * (d_bruta - d_filt)
            u_raw = KP * e + KI * integral + KD * d_filt
            u = min(max(u_raw, -U_MAX), U_MAX)
            sat = abs(u_raw) > U_MAX + 1e-12
            if not (sat and u_raw * e > 0.0):
                integral += e * H
            e_prev = e
        if k % DECIMACION == 0:
            filas.append((k, theta, omega, u))
        par = 0.0
        if 'rafaga_t0' in p and p['rafaga_t0'] <= t < p['rafaga_t0'] + p['rafaga_dur']:
            par = p['rafaga_nm']
        planta.paso(np.array([u]), np.array([par]), H)
    return filas


def escribir(nombre, p, filas):
    cab = ' '.join(f'{k}={v:.12g}' if isinstance(v, float) else f'{k}={v}'
                   for k, v in p.items())
    cab += (f' h={H:.12g} J={model.J_EJE:.12g} b={model.B_EJE:.12g} kp={KP:g} ki={KI:g} '
            f'kd={KD:g} tau={TAU:g} u_max={U_MAX:g} decimacion={DECIMACION}')
    lineas = [f'# {cab}', 'k,theta,omega,u']
    lineas += [f'{k},{th:.12e},{om:.12e},{u:.12e}' for k, th, om, u in filas]
    return '\n'.join(lineas) + '\n'


def main():
    comprobar = '--comprobar' in sys.argv
    os.makedirs(DIR, exist_ok=True)
    mal = 0
    for nombre, p in ESCENARIOS.items():
        txt = escribir(nombre, p, correr(p))
        ruta = os.path.join(DIR, nombre + '.csv')
        if comprobar:
            ok = os.path.exists(ruta) and open(ruta).read() == txt
            print(f'{"ok  " if ok else "DIFIERE"} {nombre}')
            mal += 0 if ok else 1
        else:
            with open(ruta, 'w') as f:
                f.write(txt)
            print(f'escrito {ruta} ({len(txt)} bytes)')
    return 1 if mal else 0


if __name__ == '__main__':
    sys.exit(main())
