#!/usr/bin/env python3
"""Estudio P7: ESKF consistente (Q, H del magnetometro, NEES chi-cuadrado), GPS correlado con latencia, INDI.

Uso:  python3 scripts/estudio_eskf_indi.py [seccion ...]      (sin argumentos: todas)
Secciones: h_mag, nees, sesgo, gps, indi_verdad, indi_sensores, indi_cruce
Escribe/actualiza docs/dron_resultados/estudio_eskf_indi.json (fusiona por seccion). Todo es "en simulacion, con estos supuestos".
Determinista: las semillas son fijas (0..29 para NEES, 0..9 para INDI) y no hay estado global.
"""
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_dron'))

from atp_dron import consistencia as C                      # noqa: E402
from atp_dron.cascada_indi import correr_indi               # noqa: E402
from atp_dron.dron import DT, euler, q_de_euler, rot        # noqa: E402
from atp_dron.estimador import ESKF                         # noqa: E402
from atp_dron.sensores import SENSORES                      # noqa: E402

SALIDA = os.path.join(RAIZ, 'docs', 'dron_resultados', 'estudio_eskf_indi.json')
T95 = {9: 2.262, 29: 2.045}                                  # t de Student bilateral 95 % (valores de tabla)
SEM_NEES, SEM_INDI = list(range(30)), list(range(10))

ESCENARIOS = {
    'hover_rafagas': {'ruta': [[0, 0, 8]], 'duracion_s': 30.0, 'rafaga_sigma': 1.0, 'viento': [2.0, 0, 0], 'parar_al_llegar': False},
    'maniobra': {'ruta': [[20, 0, 8], [20, 20, 8], [0, 20, 8], [0, 0, 8]], 'duracion_s': 40.0, 'rafaga_sigma': 1.0,
                 'viento': [2.0, 0, 0], 'parar_al_llegar': False},
}
OBJ = (0.0, 0.0, 10.0)


def _pool():
    return mp.get_context('fork').Pool(4)


def _ic(x, df):
    x = np.asarray(x, float)
    h = T95[df] * x.std(ddof=1) / np.sqrt(len(x))
    return [round(float(x.mean() - h), 4), round(float(x.mean() + h), 4)]


# ---------------------------------------------------------------- NEES
def _nees_uno(a):
    cfg, sp, s = a
    ts, acc, pl = C.nees_corrida({**cfg, 'semilla': s, 'sensores_params': sp}, 1.0, t_ini_s=5.0)
    m = pl.metricas()
    return acc, {k: m[k] for k in ('est_error_pos_rms_m', 'est_error_vel_rms_ms', 'est_error_actitud_rms_deg',
                                   'est_sesgo_acel_error_ms2', 'est_sesgo_gyro_error_rads')}


def conjunto(pool, escenario, sp, semillas=SEM_NEES):
    cfg = ESCENARIOS[escenario]
    r = pool.map(_nees_uno, [(cfg, sp, s) for s in semillas])
    res = C.resumen_nees([x[0] for x in r])
    mets = {k: round(float(np.mean([x[1][k] for x in r])), 5) for k in r[0][1]}
    mets['est_sesgo_acel_error_ms2_ic95'] = _ic([x[1]['est_sesgo_acel_error_ms2'] for x in r], 29)
    return {'nees': res, 'metricas_medias': mets}


def seccion_nees(pool):
    cfgs = {'defecto': {}, 'q_corregida': {'q_corregida': True}, 'q_corregida+h_mag': {'q_corregida': True, 'h_mag_exacto': True}}
    out = {}
    for esc in ESCENARIOS:
        out[esc] = {}
        for n, sp in cfgs.items():
            t0 = time.time()
            out[esc][n] = conjunto(pool, esc, sp)
            print(f'  nees {esc} {n} ({time.time() - t0:.0f} s)', flush=True)
    # contraste analitico: la Q por defecto es exactamente 1/DT = 250 veces la corregida en los bloques v y theta
    a = ESKF(9.80665, [0, 0, 0], [1, 0, 0, 0], {**SENSORES})
    b = ESKF(9.80665, [0, 0, 0], [1, 0, 0, 0], {**SENSORES, 'q_corregida': True})
    out['razon_Q_defecto_sobre_corregida'] = {'vel': round(float(a.Q[3, 3] / b.Q[3, 3]), 6), 'actitud': round(float(a.Q[6, 6] / b.Q[6, 6]), 6),
                                              '1_sobre_DT': round(1.0 / DT, 6)}
    return out


def seccion_sesgo(pool):
    """Re-sintonia del paseo aleatorio del sesgo del acelerometro que ASUME el filtro (el del sensor es 2e-3)."""
    out = {}
    for k in (1, 2, 5, 10, 25, 100):
        sp = {'q_corregida': True, 'h_mag_exacto': True, 'est_acel_sesgo_rw': SENSORES['acel_sesgo_rw'] * k}
        r = conjunto(pool, 'hover_rafagas', sp)
        out[f'x{k}'] = {'factor': k, 'sesgo_acel_error_ms2': r['metricas_medias']['est_sesgo_acel_error_ms2'],
                        'sesgo_acel_ic95': r['metricas_medias']['est_sesgo_acel_error_ms2_ic95'],
                        'vel_rms': r['metricas_medias']['est_error_vel_rms_ms'], 'pos_rms': r['metricas_medias']['est_error_pos_rms_m'],
                        'nees_vel': r['nees']['vel']['nees_medio'], 'nees_sesgo_a': r['nees']['sesgo_a']['nees_medio'],
                        'banda_instante': r['nees']['vel']['banda_instante'],
                        'frac_vel_en_banda': r['nees']['vel']['fraccion_instantes_en_banda'],
                        'frac_sesgo_a_en_banda': r['nees']['sesgo_a']['fraccion_instantes_en_banda']}
        print(f'  sesgo x{k}', out[f'x{k}']['sesgo_acel_error_ms2'], out[f'x{k}']['nees_vel'], flush=True)
    return out


# ---------------------------------------------------------------- H del magnetometro
def seccion_h_mag():
    """H del rumbo contra diferencias finitas, en funcion del cabeceo (roll 20, yaw 40 grados)."""
    out = []
    for pitch in (0, 10, 20, 35, 50, 70):
        q = q_de_euler(np.radians(20), np.radians(pitch), np.radians(40))
        fila = {'cabeceo_deg': pitch}
        for nombre, flag in (('actual', False), ('exacto', True)):
            e = ESKF(9.80665, [0, 0, 0], q, {**SENSORES, 'h_mag_exacto': flag})
            cap = []
            e._corregir = lambda y, H, Rm: cap.append(H.copy())
            e.mag(euler(q)[2])
            H = cap[0][0, 6:9]
            fd = np.zeros(3)
            for i in range(3):
                dth = np.zeros(3)
                dth[i] = 1e-6
                qp = C_qmul(q, np.r_[1.0, 0.5 * dth])
                qp /= np.linalg.norm(qp)
                fd[i] = (euler(qp)[2] - euler(q)[2]) / 1e-6
            fila['error_max_' + nombre] = round(float(np.max(np.abs(H - fd))), 6)
        out.append(fila)
    return out


def C_qmul(a, b):
    from atp_dron.estimador import _qmul
    return _qmul(a, b)


# ---------------------------------------------------------------- GPS correlado y con latencia
def seccion_gps(pool):
    out = {}
    for tau in (0.0, 10.0, 60.0):
        for lat in (0.0, 0.2, 0.5):
            sp = {'q_corregida': True, 'h_mag_exacto': True, 'gps_gm_tau_s': tau, 'gps_latencia_s': lat}
            r = conjunto(pool, 'maniobra', sp)
            out[f'tau{tau:g}_lat{lat:g}'] = {'tau_s': tau, 'latencia_s': lat,
                                             'pos_rms_m': r['metricas_medias']['est_error_pos_rms_m'],
                                             'vel_rms_ms': r['metricas_medias']['est_error_vel_rms_ms'],
                                             'nees_pos': r['nees']['pos']['nees_medio'], 'nees_vel': r['nees']['vel']['nees_medio'],
                                             'banda_instante': r['nees']['pos']['banda_instante'],
                                             'frac_pos_en_banda': r['nees']['pos']['fraccion_instantes_en_banda']}
            print('  gps', tau, lat, out[f'tau{tau:g}_lat{lat:g}']['pos_rms_m'], out[f'tau{tau:g}_lat{lat:g}']['nees_pos'], flush=True)
    pos = lambda t, l: out[f'tau{t:g}_lat{l:g}']['pos_rms_m']
    out['monotono_en_latencia_tau0'] = bool(pos(0, 0) < pos(0, 0.2) < pos(0, 0.5))
    out['monotono_en_tau_lat0'] = bool(pos(0, 0) < pos(10, 0) < pos(60, 0))
    return out


# ---------------------------------------------------------------- INDI
def _rms(filas, t0=10.0):
    e = [np.linalg.norm([r['x'] - OBJ[0], r['y'] - OBJ[1], r['z'] - OBJ[2]]) for r in filas if r['t'] > t0]
    return float(np.sqrt(np.mean(np.square(e))))


def _indi_uno(a):
    cfg, g = a
    m, f, pl = correr_indi(cfg, g)
    return _rms(f), m.get('est_error_pos_rms_m')


def _cfg_indi(sigma, viento, s, sens, sp=None):
    return {'ruta': [list(OBJ)], 'duracion_s': 40.0, 'rafaga_sigma': sigma, 'viento': [viento, 0, 0], 'semilla': s,
            'parar_al_llegar': False, 'sensores': sens, 'sensores_params': sp or {}}


def _celda(pool, sigma, viento, sens, sp=None, ganancias=(0.0, 1.0, -1.0)):
    jobs = [(_cfg_indi(sigma, viento, s, sens, sp), g) for g in ganancias for s in SEM_INDI]
    r = pool.map(_indi_uno, jobs)
    n = len(SEM_INDI)
    rms = {g: np.array([x[0] for x in r[i * n:(i + 1) * n]]) for i, g in enumerate(ganancias)}
    base = rms[0.0]
    out = {'rms_sin_indi_m': round(float(base.mean()), 4)}
    for g in ganancias[1:]:
        red = 1.0 - rms[g] / base                           # reduccion relativa por semilla (pareada)
        out[f'rms_g{g:g}_m'] = round(float(rms[g].mean()), 4)
        out[f'reduccion_g{g:g}_pct'] = round(float(100 * red.mean()), 2)
        out[f'reduccion_g{g:g}_ic95_pct'] = [round(100 * x, 2) for x in _ic(red, 9)]
    return out


def seccion_indi_verdad(pool):
    out = {}
    for sigma in (0.5, 1.0, 2.0, 3.0):
        for viento in (0.0, 6.0, 12.0):
            out[f'sigma{sigma:g}_viento{viento:g}'] = {'sigma': sigma, 'viento': viento, **_celda(pool, sigma, viento, False)}
            print('  indi verdad', sigma, viento, out[f'sigma{sigma:g}_viento{viento:g}'], flush=True)
    # control negativo de rafaga 0: no hay nada que rechazar
    out['rafaga0_viento0'] = {'sigma': 0.0, 'viento': 0.0, **_celda(pool, 0.0, 0.0, False)}
    # sensibilidad a la frecuencia de corte del filtro (sigma 2, viento 0): se recorre con una sola ganancia
    return out


def seccion_indi_sensores(pool):
    out = {}
    for sigma in (0.5, 1.0, 2.0, 3.0):
        for viento in (0.0, 6.0, 12.0):
            out[f'sigma{sigma:g}_viento{viento:g}'] = {'sigma': sigma, 'viento': viento, **_celda(pool, sigma, viento, True, ganancias=(0.0, 1.0))}
            print('  indi sensores', sigma, viento, out[f'sigma{sigma:g}_viento{viento:g}'], flush=True)
    return out


def seccion_indi_cruce(pool):
    """Con sensores, sigma 2 y sin viento: escala del ruido de posicion del GPS x sesgo inicial del acelerometro."""
    out = {}
    for esc in (0.0125, 0.05, 0.2, 1.0):
        for ba in (0.0, 0.05, 0.2):
            sp = {'gps_pos_h': 0.8 * esc, 'gps_pos_v': 1.5 * esc, 'acel_sesgo0': ba}
            k = f'gps_x{esc:g}_sesgo{ba:g}'
            out[k] = {'escala_ruido_gps': esc, 'sesgo_acel0_ms2': ba, **_celda(pool, 2.0, 0.0, True, sp, ganancias=(0.0, 1.0))}
            print('  cruce', k, out[k], flush=True)
    return out


SECCIONES = {'h_mag': seccion_h_mag, 'nees': seccion_nees, 'sesgo': seccion_sesgo, 'gps': seccion_gps,
             'indi_verdad': seccion_indi_verdad, 'indi_sensores': seccion_indi_sensores, 'indi_cruce': seccion_indi_cruce}


def main(argv):
    quedan = argv or list(SECCIONES)
    res = {}
    if os.path.exists(SALIDA):
        with open(SALIDA) as f:
            res = json.load(f)
    res['_meta'] = {'semillas_nees': len(SEM_NEES), 'semillas_indi': len(SEM_INDI), 'nota': 'en simulacion, con estos supuestos'}
    for s in quedan:
        t0 = time.time()
        print(f'== {s}', flush=True)
        if s == 'h_mag':
            res[s] = seccion_h_mag()
        else:
            with _pool() as pool:
                res[s] = SECCIONES[s](pool)
        print(f'== {s} listo en {time.time() - t0:.0f} s', flush=True)
        os.makedirs(os.path.dirname(SALIDA), exist_ok=True)
        with open(SALIDA, 'w') as f:
            json.dump(res, f, indent=1, sort_keys=True)


if __name__ == '__main__':
    main(sys.argv[1:])
