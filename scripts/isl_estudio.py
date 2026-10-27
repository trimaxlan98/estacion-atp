#!/usr/bin/env python3
"""Tablas del estudio ISL (las que cita src/sat_isl/README.md), calculadas, no escritas a mano.

    ./run.sh python3 scripts/isl_estudio.py [geometria|haz|enlace|adquisicion|lazo|todo]

Solo numpy/scipy (sin ROS). Cada seccion imprime tablas en Markdown. Determinista (semillas fijas).
"""
import dataclasses
import math
import os
import sys

import numpy as np

_RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('atp_ros', 'sat_adcs', 'sat_isl'):
    sys.path.insert(0, os.path.join(_RAIZ, 'src', _p))

from sat_isl import adquisicion as A  # noqa: E402
from sat_isl import enlace as E  # noqa: E402
from sat_isl import geometria as G  # noqa: E402
from sat_isl import seguimiento as S  # noqa: E402
from sat_isl import terminal as T  # noqa: E402
from sat_isl.escenario import tabla_vecinos  # noqa: E402
from sat_isl.misiones import Config, constelacion_de  # noqa: E402

LAM = 1550e-9


def tabla(cab, filas):
    print('| ' + ' | '.join(cab) + ' |')
    print('|' + '|'.join('---' for _ in cab) + '|')
    for f in filas:
        print('| ' + ' | '.join(str(x) for x in f) + ' |')
    print()


def geometria():
    cfg = Config(inicio_s=0.0, dur_s=6600.0)
    c = constelacion_de(cfg)
    print(f'### Vecinos de P0S0 en Walker {cfg.n_sats}/{cfg.planos}/{cfg.fase}, {cfg.h_km:.0f} km, {cfg.inc_deg:.0f} deg (una orbita, 6600 s)\n')
    filas = []
    for r in tabla_vecinos(c, 0, 6600.0, 30.0):
        filas.append([f"{r['de']} - {r['a']}", f"{r['rango_min_km']:.0f}-{r['rango_max_km']:.0f}", f"{r['visible_pct']:.0f}",
                      f"{r['v_perp_max_km_s']:.2f}", f"{r['tasa_los_max_mrad_s']:.2f}", f"{r['adelanto_max_urad']:.1f}"])
    tabla(['par', 'rango (km)', 'visible (%)', 'v_perp max (km/s)', 'giro de la LOS max (mrad/s)', 'adelanto max (urad)'], filas)
    print(f'Cuerda maxima con el rayo >= {G.H_RAYO_MIN_KM:.0f} km a {cfg.h_km:.0f} km: {G.rango_maximo_visible_km(cfg.h_km):.0f} km '
          f'({G.separacion_angular_max_deg(cfg.h_km):.1f} deg en el plano). Con 4 satelites por plano (90 deg) el rayo '
          f'pasaria a {(G.R_TIERRA_KM + cfg.h_km) * math.cos(math.pi / 4) - G.R_TIERRA_KM:.0f} km de altura, dentro de la Tierra: '
          f'un plano de 4 a esta altura no tiene enlaces intraplano.\n')
    print('### Refraccion de un rayo de 1550 nm segun la altura de su punto mas bajo\n')
    tabla(['altura de la tangente (km)', 'desviacion (urad)'],
          [[h, f'{G.refraccion_urad(h):.4g}'] for h in (30, 40, 50, 60, 70, 80, 100, 150)])
    print(f'La desviacion pasa de 1 urad por debajo de {G.altura_para_refraccion_km(1.0):.0f} km y de 10 urad por debajo de '
          f'{G.altura_para_refraccion_km(10.0):.0f} km.\n')


def haz():
    print('### Haz de comunicacion a 1550 nm (gamma = 1.12, Klein y Degnan)\n')
    filas = []
    for D in (0.03, 0.05, 0.08, 0.10, 0.135):
        g = E.ganancia_tx(D, LAM)
        th = E.semiangulo_equiv_rad(g)
        filas.append([f'{D * 100:g}', f'{10 * math.log10(g):.1f}', f'{th * 1e6:.1f}', f'{2 * th * 1e6:.1f}', f'{th * D / LAM:.3f}',
                      f'{-10 * math.log10(float(E.perdida_apuntamiento(1e-6, th))):.3f}',
                      f'{-10 * math.log10(float(E.perdida_apuntamiento(0.1 * th, th))):.2f}',
                      f'{th * 1e6 * math.sqrt(math.log(2) / 2):.1f}'])
    tabla(['D (cm)', 'G_tx (dBi)', 'semiangulo 1/e2 (urad)', 'angulo completo (urad)', 'theta*D/lam', 'perdida con 1 urad (dB)',
           'perdida con 0.1 theta (dB)', 'error para 3 dB (urad)'], filas)


def enlace():
    print('### Presupuesto de enlace del terminal del escenario (5 cm, 1 W, 1550 nm), BER 1e-6, margen 3 dB\n')
    tm = T.ISL_COHERENTE
    filas = []
    for rho in (2600.0, 5500.0, 10500.0):
        p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, rho * 1e3, tm.eta_tx, tm.eta_rx, tm.gamma)
        coh, apd = T.ISL_COHERENTE.detector, T.ISL_APD.detector
        r_c, r_a = E.tasa_maxima(coh, p, LAM, 1e-6, 3.0), E.tasa_maxima(apd, p, LAM, 1e-6, 3.0)
        filas.append([f'{rho:.0f}', f'{float(E.dbm(p)):.1f}', f'{E.fotones_por_bit(p, 1e9, LAM):.0f}',
                      f'{r_c / 1e6:.0f}', f'{r_a / 1e6:.0f}',
                      f'{float(E.dbm(coh.sensibilidad_w(1e9, LAM, 1e-6))):.1f}', f'{float(E.dbm(apd.sensibilidad_w(1e9, LAM, 1e-6))):.1f}'])
    tabla(['rango (km)', 'Pr (dBm)', 'fotones/bit a 1 Gb/s', 'tasa coherente (Mb/s)', 'tasa APD (Mb/s)', 'sens. coherente 1 Gb/s (dBm)', 'sens. APD 1 Gb/s (dBm)'], filas)
    print('### Contraste con los Tesat LCT publicados (BER 1e-9)\n')
    filas = []
    for tm, rho, rate, nota in ((T.LCT_TSX_NFIRE, 5100.0, 5.625e9, 'TSX-NFIRE 2008: 5.625 Gb/s, ~5100 km'),
                                (T.LCT_135, 45000.0, 1.8e9, 'LCT 135: 1.8 Gb/s LEO-GEO 45 000 km')):
        p = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, rho * 1e3, tm.eta_tx, tm.eta_rx, tm.gamma)
        s = tm.detector.sensibilidad_w(rate, tm.lam_m, 1e-9)
        lo, hi = 1e3, 5e5
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            pm = E.potencia_recibida_w(tm.p_tx_w, tm.lam_m, tm.D_m, tm.D_m, mid * 1e3, tm.eta_tx, tm.eta_rx, tm.gamma)
            lo, hi = (mid, hi) if pm > s else (lo, mid)
        filas.append([nota, f'{float(E.dbm(p)):.1f}', f'{E.fotones_por_bit(p, rate, tm.lam_m):.0f}',
                      f'{float(E.db(p / s)):.1f}', f'{lo:.0f}'])
    tabla(['caso', 'Pr modelo (dBm)', 'fotones/bit', 'margen (dB)', 'rango maximo del modelo (km)'], filas)


def adquisicion():
    tm = T.ISL_COHERENTE
    rho = 5500.0
    th_opt = A.theta_baliza_optimo_urad(tm, tm, rho * 1e3)
    print(f'### Adquisicion (rango {rho:.0f} km, baliza {tm.p_baliza_w} W): P0 = {float(E.dbm(A.p0_baliza_w(tm, tm, rho * 1e3))):.1f} dBm con '
          f'theta_b = {tm.theta_baliza_urad:.0f} urad; P_det = {float(E.dbm(A.potencia_deteccion_w(tm))):.1f} dBm; '
          f'r_hit = {A.radio_acierto_urad(A.p0_baliza_w(tm, tm, rho * 1e3), tm, tm.theta_baliza_urad):.0f} urad; '
          f'theta_b optimo = {th_opt:.0f} urad\n')
    fuentes = (('GNSS a bordo (supuesto 0.01 km)', 0.01), ('OD de la Estacion (0.036 km, OD.md)', 0.036),
               ('TLE de 1 dia (2.1 km, OD.md)', 2.1), ('TLE de 3 dias (11 km, OD.md)', 11.0))
    # error de conocimiento del ADCS de sat_adcs (README: rms en el eje de la antena, pase de luz): grado 8
    # 0.14 deg y grado 4 (por omision) 0.89 deg; por eje = rms / sqrt(2)
    por_eje = lambda deg: math.radians(deg) * 1e6 / math.sqrt(2.0)  # noqa: E731
    actitudes = (('star tracker (supuesto) 145 urad', 145.0),
                 (f'sat_adcs grado 8 (0.14 deg rms) {por_eje(0.14):.0f} urad', por_eje(0.14)),
                 (f'sat_adcs grado 4 (0.89 deg rms) {por_eje(0.89):.0f} urad', por_eje(0.89)))
    for nombre_b, th in (('baliza por omision (300 urad)', None), (f'baliza optima ({th_opt:.0f} urad)', th_opt)):
        print(f'**{nombre_b}**\n')
        filas = []
        for na, sa in actitudes:
            for nf, sp in fuentes:
                sig = A.sigma_incertidumbre_urad(sp, rho, sa, tm.sigma_cardan_urad)
                t, info = A.monte_carlo(sig, tm, tm, rho * 1e3, n=3000, semilla=11, theta_b_urad=th)
                dwell = tm.cam_dwell_s if 3.5 * sig <= tm.lazo.fsm_carrera_urad else max(tm.cam_dwell_s, tm.t_asiento_cardan_s)
                esc = dwell / tm.cam_dwell_s            # el barrido fuera del FSM va a 'dwell' del cardan
                tt = t * esc
                ok = np.isfinite(tt)
                filas.append([na, nf, f'{sig:.0f}', f'{info["paso_urad"]:.0f}', f'{info["pasos"]}', f'{dwell * 1e3:.0f}',
                              f'{np.median(tt[ok]):.2f}', f'{np.percentile(tt[ok], 95):.2f}',
                              f'{100 * np.mean(tt <= 1.0):.0f}', f'{100 * np.mean(tt <= 10.0):.0f}'])
        tabla(['actitud', 'efemerides', 'sigma (urad)', 'paso (urad)', 'pasos de barrido', 'paso (ms)', 'mediana (s)', 'p95 (s)', 'P(<1 s) %', 'P(<10 s) %'], filas)


def lazo():
    print('### Lazo fino (FSM) + montura gruesa\n')
    for lz in (T.LAZO_CARDAN,):
        lin = S.lazo_lineal(lz)
        m = lin.margenes()
        print(f'- cruce {m["f_cruce_hz"]:.0f} Hz, margen de fase {m["pm_deg"]:.1f} deg, margen de ganancia {m["gm_db"]:.1f} dB '
              f'(a {m["f_fase180_hz"]:.0f} Hz), pico de |S| {m["ms"]:.2f} ({20 * math.log10(m["ms"]):.1f} dB), '
              f'ancho de banda de T (-3 dB) {m["f_bw_hz"]:.0f} Hz; polos estables: {lin.estable()}; Ki = {lin.Ki:.0f} 1/s\n')
    print('| frecuencia (Hz) | 0.1 | 1 | 5 | 35 | 133 | 330 | 1000 |\n|---|---|---|---|---|---|---|---|')
    lin = S.lazo_lineal(T.LAZO_CARDAN)
    fs = np.array([0.1, 1, 5, 35, 133, 330, 1000.0])
    print('| \\|S\\| (dB) | ' + ' | '.join(f'{20 * math.log10(abs(x)):.1f}' for x in lin.S(fs)) + ' |\n')
    print('Perturbacion de plataforma (SUPUESTO, urad rms por eje) y error residual por componente (analitico):\n')
    filas = []
    for c in S.PLATAFORMA_DEFECTO:
        a = S.rms_analitico(T.LAZO_CARDAN, comps=(c,))
        filas.append([c.nombre, f'{c.f_hz:g}', f'{c.zeta:g}', f'{c.rms_urad:.2f}', f'{a["e_rms"]:.3f}', f'{a["y_rms"]:.2f}'])
    a = S.rms_analitico(T.LAZO_CARDAN)
    filas.append(['**total**', '', '', f'{a["d_rms"]:.1f}', f'{a["e_rms"]:.3f}', f'{a["y_rms"]:.1f}'])
    tabla(['componente', 'f (Hz)', 'zeta', 'rms entrada (urad)', 'rms residual (urad)', 'rms posicion FSM (urad)'], filas)
    tm = T.ISL_COHERENTE
    th = E.semiangulo_haz_rad(tm.D_m, tm.lam_m, tm.gamma) * 1e6
    rapidas = S.PLATAFORMA_DEFECTO[1:]
    sig_sin = math.sqrt(S.rms_analitico(T.LAZO_CARDAN, comps=rapidas)['d_rms'] ** 2)
    a_con = S.rms_analitico(T.LAZO_CARDAN, nea_urad=0.57)
    print(f'Perdida media por apuntamiento (semiangulo del haz {th:.1f} urad): sin lazo fino y solo con la parte rapida '
          f'(sigma {sig_sin:.1f} urad por eje) {-10 * math.log10(E.perdida_apuntamiento_media(sig_sin, th)):.2f} dB por extremo; '
          f'con el lazo (sigma {a_con["e_rms"]:.2f} urad) {-10 * math.log10(E.perdida_apuntamiento_media(a_con["e_rms"], th)):.4f} dB.\n')
    print('Carrera del FSM que pide la deriva lenta (y = d/(1 + H_c)):\n')
    tabla(['montura gruesa', 'f descarga (Hz)', 'rms de y (urad)', '4 sigma (urad)', 'carrera disponible (urad)', 'cabe'],
          [[lz.nombre_grueso, f'{lz.f_descarga_hz:g}', f'{S.carrera_baja_frecuencia_urad(lz):.1f}',
            f'{4 * S.carrera_baja_frecuencia_urad(lz):.0f}', f'{lz.fsm_carrera_urad:.0f}',
            'si' if 4 * S.carrera_baja_frecuencia_urad(lz) < lz.fsm_carrera_urad else 'NO'] for lz in (T.LAZO_CARDAN, T.LAZO_ADCS)])


def escenario():
    """El escenario completo (lockstep sin ROS) con 8 semillas: mediana y rango de las cifras del RESUMEN."""
    from sat_isl.escenario import simular
    print('### Escenario completo, 8 semillas (20 s por corrida)\n')
    casos = (('par del plano P0S0-P0S1, TLE de 1 dia, star tracker', dict()),
             ('par de planos P0S0-P2S2 (rango 2.7-3 Mm), TLE de 1 dia', dict(sat_b=18, inicio_s=400.0)),
             ('par del plano, TLE de 3 dias (11 km)', dict(sigma_pos_km=11.0)),
             ('par del plano, actitud sat_adcs grado 8 (1730 urad por eje)', dict(sigma_actitud_urad=1728.0, dur_s=60.0)),
             ('par del plano, detector APD', dict(detector='apd')))
    filas = []
    for nombre, kw in casos:
        base = Config(dur_s=20.0)
        res = []
        const = None
        for sem in range(1, 9):
            cfg = dataclasses.replace(base, semilla=sem, **kw)
            const = const or constelacion_de(cfg)
            res.append(simular(cfg, const).numeros())

        def agr(k, f='{:.3f}'):
            v = [r[k] for r in res if r.get(k) is not None]
            return f'{f.format(np.median(v))} ({f.format(min(v))}-{f.format(max(v))})' if v else '-'
        filas.append([nombre, agr('t_adq_s', '{:.2f}'), agr('err_rms_urad'), agr('err_p95_urad'), agr('cerrado_pct_visible', '{:.1f}'),
                      agr('tasa_cerrado_bps', '{:.3g}')])
    tabla(['caso', 't adquisicion (s)', 'error rms (urad)', 'error p95 (urad)', 'cerrado (% visible)', 'tasa (b/s)'], filas)


SECCIONES = {'geometria': geometria, 'haz': haz, 'enlace': enlace, 'adquisicion': adquisicion, 'lazo': lazo,
             'escenario': escenario}

if __name__ == '__main__':
    pedidas = sys.argv[1:] or ['todo']
    for n in (SECCIONES if 'todo' in pedidas else pedidas):
        SECCIONES[n]()
