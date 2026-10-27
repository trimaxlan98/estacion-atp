#!/usr/bin/env python3
"""Identifica J y b de un eje de la montura desde datos de banco (solo numpy).

Es la MISMA identificacion del equipo de ros2_control (`atp_control/scripts/identify_plant.py`,
docs/ROS2_CONTROL.md 8.4: escalon de par, J = tau b) con dos entradas mas, para poder usar cualquier
registro y no solo el que graba ese nodo:

  escalon  DATOS.csv --par 1.0 [--t-par 2.0]
      Escalon de PAR en lazo abierto (el procedimiento de ros2_control, `csv_file` de
      `atp_identify.launch.py`: columnas t,theta,omega; con la planta de planta.py da J=2, b=0.5).
      Ajusta por minimos cuadrados w(t) = w_inf (1 - exp(-(t - t0 - ts)/tau)) (la latencia ts del
      par es parametro) y da b = u / w_inf, J = tau b. Hace falta que la velocidad llegue cerca del
      estacionario (> 3 tau) o la inercia sale sesgada.
  lazo     DATOS.csv --kp 25 --ki 2 --kd 8
      Escalon de POSICION en el lazo cerrado del FIRMWARE (el ESP32 solo cierra posicion: no hay
      interfaz de par; `/atp/reference` -> `~/joint_states`). Columnas t,ref,theta (rad). Simula el
      PID del firmware (el de pid_controller.py: derivada filtrada, anti-windup por clamping, a
      `--dt-lazo`, 1 ms) sobre J theta'' + b theta' = u y ajusta (J, b) a theta(t) medido por
      Levenberg-Marquardt. Identifica bien J y b solo si el escalon excita la dinamica (de 1 a 10 grados,
      sin saturar el par) y se conocen las ganancias REALES y la escala de par (`u_pleno`).
  autotest
      Genera datos con la planta de planta.py (J=2.7, b=0.8, ruido) y comprueba que recupera J y b.

La inercia que se identifica es la reflejada al eje de carga y la que importa para sintonizar
. NO identifica friccion de Coulomb, holgura ni desbalance
(docs/CONTROL.md 1): con ellas el escalon da una J y b EFECTIVAS; repetir con dos amplitudes.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_ros'))


# ---------------------------------------------------------------- escalon de par

def ajustar_escalon_par(t, omega, par_nm, t_par_s):
    """w(t) = w_inf (1 - exp(-(t - t_par - ts)/tau)) para t > t_par. Para cada (tau, ts) el w_inf
    optimo es lineal; se barre tau en escala logaritmica y ts en [0, 0.5] s y se refina.
    Devuelve dict con b, J, tau, w_inf, ts y el rms del residuo."""
    t, w = np.asarray(t, float), np.asarray(omega, float)
    s = t > t_par_s
    x, y = t[s] - t_par_s, w[s]

    def costo(tau, ts):
        f = 1.0 - np.exp(-np.clip(x - ts, 0.0, None) / tau)
        winf = float(f @ y / (f @ f))
        return float(np.sum((y - winf * f) ** 2)), winf

    mejor = (math.inf, 0.0, 0.0, 0.0)
    for tau in np.geomspace(0.05, 200.0, 240):
        for ts in np.linspace(0.0, 0.5, 11):
            c, wi = costo(tau, ts)
            if c < mejor[0]:
                mejor = (c, tau, ts, wi)
    _, tau, ts, wi = mejor
    for paso in (0.05, 0.01, 0.002):                 # refinamiento local
        for _ in range(3):
            for dtau in np.linspace(-5, 5, 21) * paso * tau:
                for dts in np.linspace(-5, 5, 11) * paso:
                    tt, st = max(tau + dtau, 1e-3), max(ts + dts, 0.0)
                    c, w2 = costo(tt, st)
                    if c < mejor[0]:
                        mejor = (c, tt, st, w2)
            _, tau, ts, wi = mejor
    b = par_nm / wi
    return {'b_nm_s_rad': float(b), 'J_kg_m2': float(tau * b), 'tau_s': float(tau),
            'w_inf_rad_s': float(wi), 'latencia_par_s': float(ts),
            'rms_residuo_rad_s': float(math.sqrt(mejor[0] / max(1, len(y))))}


# ---------------------------------------------------------------- lazo cerrado

def simular_lazo(J, b, t_eval, t_ref, ref, kp, ki, kd, tau_d=0.08, u_max=8.0, dt=1e-3, theta0=0.0):
    """theta(t_eval) del lazo PID discreto (a `dt`) sobre J theta'' + b theta' = u, con la referencia
    `ref(t_ref)` retenida (ZOH). Planta integrada exacta por intervalo (retencion de orden cero)."""
    n = int(math.ceil((t_eval[-1] - t_ref[0]) / dt)) + 1
    tt = t_ref[0] + dt * np.arange(n)
    # referencia con retencion de orden cero (lo que publica un generador a 20 Hz): interpolar un
    # escalon lo adelanta hasta un periodo de muestreo y sesga b
    r = ref[np.clip(np.searchsorted(t_ref, tt + 1e-12, side='right') - 1, 0, len(ref) - 1)]
    a = b / J
    ea = math.exp(-a * dt)
    th, w = theta0, 0.0
    integ, e_prev, d_f = 0.0, 0.0, 0.0
    alfa = dt / (tau_d + dt)
    out = np.empty(n)
    for k in range(n):
        out[k] = th
        e = r[k] - th
        d_f += alfa * ((e - e_prev) / dt - d_f)
        u_raw = kp * e + ki * integ + kd * d_f
        u = max(-u_max, min(u_max, u_raw))
        if not (abs(u_raw) > u_max and u_raw * e > 0.0):      # anti-windup por clamping
            integ += e * dt
        e_prev = e
        w_nuevo = ea * w + (1.0 - ea) * u / b
        th += (u / b) * dt + (w - u / b) * (1.0 - ea) / a
        w = w_nuevo
    return np.interp(t_eval, tt, out)


def ajustar_lazo(t, ref, theta, kp, ki, kd, tau_d=0.08, u_max=8.0, dt=1e-3, J0=2.0, b0=0.5,
                 iteraciones=40):
    """Levenberg-Marquardt sobre (log J, log b) minimizando |theta_sim - theta_medido|^2."""
    t, ref, theta = (np.asarray(x, float) for x in (t, ref, theta))
    p = np.log([J0, b0])

    def res(q):
        return simular_lazo(math.exp(q[0]), math.exp(q[1]), t, t, ref, kp, ki, kd, tau_d, u_max, dt,
                            theta[0]) - theta
    lam = 1e-2
    r0 = res(p)
    c0 = float(r0 @ r0)
    for _ in range(iteraciones):
        Jm = np.empty((len(t), 2))
        for i in range(2):
            h = 1e-4
            q = p.copy()
            q[i] += h
            Jm[:, i] = (res(q) - r0) / h
        A = Jm.T @ Jm
        g = Jm.T @ r0
        for _ in range(8):
            dp = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            r1 = res(p + dp)
            c1 = float(r1 @ r1)
            if c1 < c0:
                p, r0, c0, lam = p + dp, r1, c1, max(lam / 3.0, 1e-9)
                break
            lam *= 4.0
        else:
            break
        if float(np.max(np.abs(dp))) < 1e-7:
            break
    cov = np.linalg.inv(A) * c0 / max(1, len(t) - 2)         # sobre (log J, log b): errores relativos
    return {'J_kg_m2': math.exp(p[0]), 'b_nm_s_rad': math.exp(p[1]),
            'sigma_rel_J': float(math.sqrt(cov[0, 0])), 'sigma_rel_b': float(math.sqrt(cov[1, 1])),
            'rms_residuo_rad': float(math.sqrt(c0 / len(t)))}


# ---------------------------------------------------------------- CSV y autotest

def leer_csv(ruta, columnas):
    filas = []
    with open(ruta, newline='') as f:
        lector = csv.reader(f)
        cab = next(lector)
        if not cab[0].lstrip('#').strip().replace('.', '').replace('-', '').replace('e', '').isalpha():
            f.seek(0)
            lector = csv.reader(f)            # sin cabecera
        else:
            cab = [c.lstrip('#').strip() for c in cab]
        for r in lector:
            if r and not r[0].startswith('#'):
                filas.append([float(x) for x in r])
    return np.array(filas)[:, :columnas]


def autotest():
    from atp_ros.planta import PlantaMontura
    rng = np.random.default_rng(3)
    J, b = 2.7, 0.8
    # escalon de par: 1 N m durante 6 tau = 20 s
    pl = PlantaMontura(J, b, math.inf, 0.002)
    pl.enganchar([0.0, 0.0])
    t, w = [], []
    for k in range(int(20.0 / 0.002)):
        pl.paso(np.array([1.0, 0.0]), 0.0, 0.002)
        if k % 10 == 0:
            t.append((k + 1) * 0.002)
            w.append(pl.omega[0] + rng.normal(0, 0.002))
    r = ajustar_escalon_par(np.array(t), np.array(w), 1.0, 0.0)
    print(f'escalon de par: J {r["J_kg_m2"]:.4f} (verdad {J}) b {r["b_nm_s_rad"]:.4f} (verdad {b}) '
          f'tau {r["tau_s"]:.3f} (verdad {J / b:.3f})')
    ok1 = abs(r['J_kg_m2'] / J - 1) < 0.02 and abs(r['b_nm_s_rad'] / b - 1) < 0.02
    # lazo cerrado: escalon de 5 grados, datos a 50 Hz con ruido de encoder de 0.005 grados. Los datos
    # salen de la planta INDEPENDIENTE de planta.py (integracion semi-implicita) con el PID de
    # pid_controller.py a 1 kHz, no del simulador de este script
    kp, ki, kd, tau_d, h = 25.0, 2.0, 8.0, 0.08, 1e-3
    pl = PlantaMontura(J, b, math.inf, h)
    pl.enganchar([0.0, 0.0])
    integ = e_prev = d_f = 0.0
    tm, th = [], []
    ref_t = lambda tt: math.radians(5.0) if tt >= 1.0 else 0.0     # noqa: E731
    for k in range(int(15.0 / h)):
        tt = k * h
        e = ref_t(tt) - pl.theta[0]
        d_f += h / (tau_d + h) * ((e - e_prev) / h - d_f)
        u_raw = kp * e + ki * integ + kd * d_f
        u = max(-8.0, min(8.0, u_raw))
        if not (abs(u_raw) > 8.0 and u_raw * e > 0.0):
            integ += e * h
        e_prev = e
        if k % 20 == 0:
            tm.append(tt)
            th.append(pl.theta[0] + rng.normal(0, math.radians(0.005)))
        pl.paso(np.array([u, 0.0]), 0.0, h)
    tm, th = np.array(tm), np.array(th)
    ref = np.array([ref_t(x) for x in tm])
    r2 = ajustar_lazo(tm, ref, th, kp, ki, kd, J0=1.5, b0=0.3)
    print(f'lazo cerrado: J {r2["J_kg_m2"]:.4f} +-{100 * r2["sigma_rel_J"]:.1f}% b {r2["b_nm_s_rad"]:.4f} '
          f'+-{100 * r2["sigma_rel_b"]:.1f}% (verdad {J}, {b})')
    ok2 = abs(r2['J_kg_m2'] / J - 1) < 0.05 and abs(r2['b_nm_s_rad'] / b - 1) < 0.15
    print('AUTOTEST', 'OK' if ok1 and ok2 else 'FALLA')
    return 0 if ok1 and ok2 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='modo', required=True)
    e = sub.add_parser('escalon')
    e.add_argument('csv')
    e.add_argument('--par', type=float, required=True, help='par aplicado (N m)')
    e.add_argument('--t-par', type=float, default=2.0, help='instante en que entra el par (s)')
    la = sub.add_parser('lazo')
    la.add_argument('csv')
    la.add_argument('--kp', type=float, required=True)
    la.add_argument('--ki', type=float, required=True)
    la.add_argument('--kd', type=float, required=True)
    la.add_argument('--tau-d', type=float, default=0.08)
    la.add_argument('--u-max', type=float, default=8.0)
    la.add_argument('--dt-lazo', type=float, default=1e-3)
    sub.add_parser('autotest')
    a = ap.parse_args()
    if a.modo == 'autotest':
        sys.exit(autotest())
    if a.modo == 'escalon':
        d = leer_csv(a.csv, 3)
        r = ajustar_escalon_par(d[:, 0] - d[0, 0], d[:, 2], a.par, a.t_par)
    else:
        d = leer_csv(a.csv, 3)
        r = ajustar_lazo(d[:, 0] - d[0, 0], d[:, 1], d[:, 2], a.kp, a.ki, a.kd, a.tau_d, a.u_max, a.dt_lazo)
    print(json.dumps(r, indent=1))


if __name__ == '__main__':
    main()
