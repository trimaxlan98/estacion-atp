#!/usr/bin/env python3
"""Estudio P6: control del dron en Titan (guiado con autoridad, antiwindup, mezclador con prioridad de par) y metrica de rafagas.

Uso: python3 scripts/estudio_control_titan.py [--rapido] [--salida docs/dron_resultados/estudio_control_titan.json]
Todo es en simulacion, con los supuestos de atp_dron (parametros del vehiculo de orden de magnitud). 
"""
import argparse
import hashlib
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_dron'))
from atp_dron import control, guiado_autoridad as GA, rafagas_metrica as RM  # noqa: E402
from atp_dron.control import Cascada  # noqa: E402
from atp_dron.dron import DT, Dron  # noqa: E402
from atp_dron.lazo import correr  # noqa: E402
from atp_dron.lockstep import Guiado, PlantaLock, _angulo_cero, cargar_cfg  # noqa: E402

RUTA = [[20, 0, 10], [40, 0, 10]]
META_X = 40.0
BASE = {'semilla': 2, 'ruta': RUTA, 'duracion_s': 60.0, 'rafaga_sigma': 0.5, 'viento': [1.0, 0.0, 0.0], 'v_max': 5.0,
        'parar_al_llegar': False}
CUERPOS = {
    'tierra': {'cuerpo': 'tierra'},
    'titan': {'cuerpo': 'titan'},
    'marte_x25': {'cuerpo': 'marte', 'dron': {'rotor_escala': 25.0}},
    'titan_carga3kg': {'cuerpo': 'titan', 'dron': {'masa_kg': 3.0}},
}
CONFIGS = {
    'base': {},
    'aw': {'antiwindup': True},
    'mezcla': {'mezcla_par': True},
    'aut': {'autoridad_guiado': True},
    'aut_aw': {'autoridad_guiado': True, 'antiwindup': True},
    'aut_recorte': {'autoridad_guiado': True, 'autoridad_acel': 'recorte'},
    'aut_recorte_aw': {'autoridad_guiado': True, 'autoridad_acel': 'recorte', 'antiwindup': True},
    'aut_rampa': {'autoridad_guiado': True, 'autoridad_acel': 'rampa'},
    'aut_rampa_aw': {'autoridad_guiado': True, 'autoridad_acel': 'rampa', 'antiwindup': True},
    'aut_rampa_aw_mezcla': {'autoridad_guiado': True, 'autoridad_acel': 'rampa', 'antiwindup': True, 'mezcla_par': True},
}


def run_instr(cfg, post_guiado=None, lazo_alt=False):
    """Una corrida de la cadena con medidores: sobreimpulso a la meta, RMS del error de velocidad, rotor en cero, inclinacion."""
    cfg = cargar_cfg(None, cfg)
    pl, gu = PlantaLock(cfg), Guiado(cfg)
    if post_guiado:
        post_guiado(gu)
    if lazo_alt:
        pl.ctl.__class__ = CascadaOrdenAlterno
    orig = pl.ctl.mando
    rot_cero, tilt = [], []

    def mando(v, y=0.0):
        tilt.append(_angulo_cero(pl.d.q))
        T = orig(v, y)
        rot_cero.append(bool((T <= 0.0).any()))
        return T
    pl.ctl.mando = mando
    ev, xs = [], []
    while not pl.fin:
        sp = gu.decidir(json.loads(pl.estado_txt()))
        pl.aplicar(json.loads(json.dumps(sp)))
        ev.append(float(np.linalg.norm(np.array(sp['v']) - pl.d.v)))
        xs.append(float(pl.d.p[0]))
    met = pl.cerrar()
    t_ll = met['t_llegada_s']
    kt = len(ev) if t_ll is None else min(len(ev), int(round(t_ll / (5 * DT))))
    ks = len(rot_cero) if t_ll is None else min(len(rot_cero), int(round(t_ll / DT)))
    return {'llego': bool(met['llego']), 't_llegada_s': t_ll,
            'sobreimpulso_m': max(0.0, max(xs) - cfg['ruta'][-1][0]),
            'rms_error_vel_ms': float(np.sqrt(np.mean(np.square(ev[:max(1, kt)])))),
            'rotor_cero_pct': 100.0 * float(np.mean(rot_cero[:max(1, ks)])),
            'inclinacion_max_deg': float(max(tilt[:max(1, ks)])) if tilt else 0.0,
            'inclinacion_max_toda_deg': float(max(tilt)) if tilt else 0.0,
            'choque': bool(met['choque'])}


class CascadaOrdenAlterno(Cascada):
    """Lo mismo que Cascada pero el lazo de velocidad USA la integral vieja y la actualiza despues (solo cambia el orden)."""

    def _lazo_externo(self, v_des, dt_ext):
        d = self.d
        ev = v_des - d.v
        a_des = self.kv * ev + self.ki_v * self.int_v
        int_nuevo = np.clip(self.int_v + ev * dt_ext, -3.0, 3.0)
        F = d.m * (a_des + np.array([0.0, 0.0, d.g]))
        F[2] = max(F[2], 0.2 * d.m * d.g)
        h = np.hypot(F[0], F[1])
        hmax = F[2] * np.tan(self.tilt_max)
        sat = h > hmax
        if sat:
            F[0:2] *= hmax / h
        self.int_v = self.int_v if (self.antiwindup and sat) else int_nuevo
        self.T_des = float(np.linalg.norm(F))
        self.z_b_des = F / self.T_des


def _job_barrido(a):
    cuerpo, conf, semilla = a
    cfg = {**BASE, **CUERPOS[cuerpo], **CONFIGS[conf], 'semilla': semilla}
    return (cuerpo, conf, semilla, run_instr(cfg))


def _resumen(filas, claves):
    out = {'n': len(filas), 'llegaron': int(sum(f['llego'] for f in filas)), 'choques': int(sum(f['choque'] for f in filas))}
    for k in claves:
        v = np.array([f[k] for f in filas if f[k] is not None], float)
        out[k] = {'media': round(float(v.mean()), 4), 'desv': round(float(v.std(ddof=1)), 4) if len(v) > 1 else 0.0,
                  'min': round(float(v.min()), 4), 'max': round(float(v.max()), 4)} if len(v) else None
    return out


CLAVES = ['t_llegada_s', 'sobreimpulso_m', 'rms_error_vel_ms', 'rotor_cero_pct', 'inclinacion_max_deg']


def barrido(pool, semillas):
    trabajos = [(c, k, s) for c in CUERPOS for k in CONFIGS for s in semillas]
    res = {}
    for cuerpo, conf, s, r in pool.map(_job_barrido, trabajos):
        res.setdefault(cuerpo, {}).setdefault(conf, []).append(r)
    return {c: {k: _resumen(v, CLAVES) for k, v in d.items()} for c, d in res.items()}, res


# ------------------------------------------------------------------------------------------------ escalon de actitud
def modelo_lineal(theta_d_deg, s=1.0, k_att=6.0, k_rate=12.0, tau_m=0.05, con_motor=True, dt=0.0002, t_fin=1.5):
    """Respuesta del lazo de actitud linealizado (un eje): th'' = alfa; alfa_cmd = k_rate (-s k_att sin(th - th_d) - w);
    con motor: alfa' = (alfa_cmd - alfa) / tau_m (la linealizacion de om' = (om_c - om)/tau con T ~ om^2 es exactamente un retardo de primer
    orden del empuje). Devuelve (t, th en grados)."""
    th_d = np.radians(theta_d_deg)
    f = lambda x: np.array([x[1], x[2] if con_motor else k_rate * (-s * k_att * np.sin(x[0] - th_d) - x[1]),
                            (k_rate * (-s * k_att * np.sin(x[0] - th_d) - x[1]) - x[2]) / tau_m if con_motor else 0.0])
    x = np.zeros(3)
    ts, ths = [], []
    for i in range(int(t_fin / dt)):
        k1 = f(x); k2 = f(x + 0.5 * dt * k1); k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        ts.append((i + 1) * dt); ths.append(np.degrees(x[0]))
    return np.array(ts), np.array(ths)


def _metr_escalon(t, th, th_d):
    sob = (th.max() - th_d) / th_d * 100.0
    i10, i90 = int(np.argmax(th >= 0.1 * th_d)), int(np.argmax(th >= 0.9 * th_d))
    t_r = float(t[i90] - t[i10]) if th.max() >= 0.9 * th_d else None
    return float(sob), t_r


def escalon_actitud(cuerpo, theta_d_deg, s=1.0, mezcla=False, dron=None, t_fin=1.5):
    d = Dron(cuerpo, dron or {}, pos=(0, 0, 100.0))
    ctl = Cascada(d, k_att=6.0 * s, mezcla_par=mezcla, mezcla_dT_g=0.1)
    ctl.n_ext, ctl.k = 10 ** 9, 1                     # sin lazo externo: se manda la actitud deseada directamente
    ctl.T_des = d.m * d.g
    for _ in range(int(1.5 / DT)):                     # arranque de motores y vuelo estacionario
        d.paso(ctl.mando(np.zeros(3)))
    th = np.radians(theta_d_deg)
    ctl.z_b_des = np.array([np.sin(th), 0.0, np.cos(th)])
    ctl.T_des = d.m * d.g / np.cos(th)
    t, tilt, cero = [], [], []
    for i in range(int(t_fin / DT)):
        T = ctl.mando(np.zeros(3))
        cero.append(bool((T <= 0).any()))
        d.paso(T)
        t.append((i + 1) * DT); tilt.append(_angulo_cero(d.q))
    t, tilt = np.array(t), np.array(tilt)
    sob, t_r = _metr_escalon(t, tilt, theta_d_deg)
    return {'sobreimpulso_pct': round(sob, 2), 't_subida_10_90_s': None if t_r is None else round(t_r, 4),
            'rotor_cero_pct': round(100.0 * float(np.mean(cero)), 2), 'pico_deg': round(float(tilt.max()), 2)}


def banco_actitud():
    out = {}
    for cuerpo in ('tierra', 'titan'):
        for ang in (10, 20, 35):
            for s in (0.5, 0.7, 1.0, 1.5):
                r = escalon_actitud(cuerpo, ang, s)
                t, th = modelo_lineal(ang, s, con_motor=True)
                tA, thA = modelo_lineal(ang, s, con_motor=False)
                sB, rB = _metr_escalon(t, th, ang)
                sA, rA = _metr_escalon(tA, thA, ang)
                r.update({'lineal_con_motor_sobreimpulso_pct': round(sB, 2), 'lineal_con_motor_t_subida_s': None if rB is None else round(rB, 4),
                          'lineal_2do_orden_sobreimpulso_pct': round(sA, 2), 'lineal_2do_orden_t_subida_s': None if rA is None else round(rA, 4)})
                out[f'{cuerpo}_{ang}deg_s{s}'] = r
    out['titan_35deg_s1.0_mezcla_par'] = escalon_actitud('titan', 35, 1.0, mezcla=True)
    out['titan_35deg_s1.0_sin_mezcla_par'] = escalon_actitud('titan', 35, 1.0, mezcla=False)
    return out


# ------------------------------------------------------------------------------------------------ velocidad terminal
def v_terminal_medida():
    out = {}
    for nombre, cfg in (('tierra', {'cuerpo': 'tierra'}), ('titan', {'cuerpo': 'titan'}),
                        ('titan_carga3kg', {'cuerpo': 'titan', 'dron': {'masa_kg': 3.0}})):
        c = {'ruta': [[5000, 0, 10]], 'v_max': 60.0, 'duracion_s': 80.0, 'hz_telemetria': 25.0, 'parar_al_llegar': False, **cfg}
        _, f = correr(c)
        v = np.array([np.hypot(r['vx'], r['vy']) for r in f])
        cola = v[int(0.85 * len(v)):]
        teor = GA.limites_autoridad(c['cuerpo'], c.get('dron'), 35.0)[2]['v_term_ms']
        out[nombre] = {'v_term_formula_ms': round(teor, 3), 'v_medida_cola_ms': round(float(cola.mean()), 3),
                       'v_max_alcanzada_ms': round(float(v.max()), 3),
                       'error_pct': round(100.0 * (float(cola.mean()) - teor) / teor, 2),
                       'pendiente_cola_ms_por_s': round(float((cola[-1] - cola[0]) / (0.15 * 80.0)), 4)}
    return out


# ------------------------------------------------------------------------------------------------ controles
def controles_titan(pool, semillas):
    def forzar_a2(gu):
        gu.a_brake = 2.0

    def forzar_v5(gu):
        gu.v_tope = None                       # sin el tope por autoridad (antes de la integracion: gu.v_max = 5.0)
        gu.c['v_max'] = 5.0
    final = {**CUERPOS['titan'], **CONFIGS['aut_rampa_aw']}
    casos = {'final': (final, None), 'final_con_a_brake_forzado_2': (final, forzar_a2),
             'final_con_v_max_forzado_5': (final, forzar_v5),
             'final_sin_acel': ({**CUERPOS['titan'], **CONFIGS['aut_aw']}, None),
             'airmode_sin_tope_con_aut': ({**CUERPOS['titan'], **CONFIGS['aut'], 'mezcla_par': True, 'mezcla_dT_g': 1e9}, None),
             'airmode_sin_tope_sin_aut': ({**CUERPOS['titan'], 'mezcla_par': True, 'mezcla_dT_g': 1e9}, None)}
    out = {}
    for nombre, (extra, hook) in casos.items():
        filas = [run_instr({**BASE, **extra, 'semilla': s}, hook) for s in semillas]
        out[nombre] = _resumen(filas, CLAVES)
    return out


def viento_sostenido():
    out = {}
    for u in (12.0, 26.0):
        for aw in (False, True):
            _, f = correr({'ruta': [[0, 0, 10]], 'duracion_s': 40.0, 'parar_al_llegar': False, 'viento': [u, 0, 0],
                           'antiwindup': aw, 'hz_telemetria': 10.0})
            cola = [r for r in f if r['t'] > 20.0]
            out[f'viento_{u:g}_aw_{aw}'] = {'desviacion_max_m': round(max(float(np.hypot(r['x'], r['y'])) for r in cola), 3),
                                           'inclinacion_max_deg': round(max(abs(r['pitch_deg']) for r in f), 2)}
    return out


# ------------------------------------------------------------------------------------------------ metrica de rafagas y seg0
def _job_rafaga(a):
    sem, sigmas = a
    ruta = [[30, 0, 10], [30, 30, 10], [0, 0, 10]]
    base = {'ruta': ruta, 'duracion_s': 90.0, 'viento': [2.0, 0, 0], 'semilla': sem}
    _, gem = correr({**base, 'rafaga_sigma': 0.0})
    out = {}
    for sg in sigmas:
        m, _, _ = RM.correr_con_metrica({**base, 'rafaga_sigma': sg}, gem)
        out[sg] = (m['desv_vs_sin_rafagas_rms_m'], m['error_transversal_rms_m'])
    return out


def rafagas(pool, n):
    sigmas = (0.0, 0.5, 1.0, 2.0)
    res = pool.map(_job_rafaga, [(100 + k, sigmas) for k in range(n)])
    out = {}
    for sg in sigmas:
        d = [r[sg][0] for r in res]
        e = [r[sg][1] for r in res]
        md, hd = RM.ic95(d)
        me, he = RM.ic95(e)
        out[str(sg)] = {'n': n, 'desv_vs_sin_rafagas_rms_m': {'media': round(md, 4), 'ic95_semiancho': round(hd, 4),
                                                              'lo': round(md - hd, 4), 'hi': round(md + hd, 4)},
                        'error_transversal_rms_m': {'media': round(me, 4), 'ic95_semiancho': round(he, 4)}}
    ms = [out[str(s)]['desv_vs_sin_rafagas_rms_m'] for s in sigmas]
    out['estrictamente_creciente'] = bool(all(ms[i]['media'] < ms[i + 1]['media'] for i in range(3)))
    out['ic95_sin_solape_en_0.5_1_2'] = bool(ms[1]['hi'] < ms[2]['lo'] and ms[2]['hi'] < ms[3]['lo'])
    out['t_student_975_gl_n_menos_1'] = round(RM.t_cuantil_975(n - 1), 4)
    return out


def seg0_y_ruta_vacia():
    out = {}
    for nombre, ruta in (('primer_wp_en_el_eje_20m', [[20, 0, 10]]), ('primer_wp_fuera_del_eje', [[20, 10, 10]]),
                         ('primer_wp_fuera_del_eje_lejos_134m', [[120, 60, 10]]),
                         ('dos_tramos_fuera_del_eje', [[10, 20, 10], [30, 20, 10]])):
        for flag in (False, True):
            m, _ = correr({'ruta': ruta, 'duracion_s': 90.0, 'seg0_corregido': flag})
            out[f'{nombre}_seg0_corregido_{flag}'] = {'error_transversal_rms_m': m['error_transversal_rms_m'], 'llego': m['llego']}
    try:
        m, f = correr({'ruta': [], 'duracion_s': 2.0})
        out['ruta_vacia'] = {'excepcion': None, 'filas': len(f), 'metas_total': m['metas_total'], 'z_final': f[-1]['z']}
    except Exception as e:  # noqa: BLE001
        out['ruta_vacia'] = {'excepcion': repr(e)}
    return out


# ------------------------------------------------------------------------------------------------ hash y costo de la cruz
def _huella(cfg):
    m, f = correr(cfg)
    return hashlib.sha256((json.dumps(m, sort_keys=True) + ''.join(json.dumps(r, sort_keys=True) for r in f)).encode()).hexdigest()[:16]


def cruz_hash_y_tiempo():
    cfgs = {'tierra_ruta': {'ruta': [[10, 0, 10], [10, 10, 10], [0, 10, 10]], 'duracion_s': 40, 'viento': [2, 0, 0], 'rafaga_sigma': 1.0, 'semilla': 5},
            'titan': {'cuerpo': 'titan', 'semilla': 2, 'ruta': RUTA, 'duracion_s': 60, 'rafaga_sigma': 0.5, 'viento': [1, 0, 0]},
            'sensores': {'ruta': [[10, 0, 10], [10, 10, 10]], 'duracion_s': 30, 'sensores': True, 'semilla': 3, 'rafaga_sigma': 0.5}}
    nueva = control._cruz
    viejo = lambda a, b: np.cross(a, b)           # noqa: E731
    out = {}
    for modo, fn in (('np.cross', viejo), ('cruz_manual', nueva)):
        control._cruz = fn
        h, tmin = {}, 1e9
        for rep in range(3):
            t0 = time.perf_counter()
            h = {k: _huella(c) for k, c in cfgs.items()}
            tmin = min(tmin, time.perf_counter() - t0)
        out[modo] = {'huellas': h, 't_s': round(tmin, 3)}
    control._cruz = nueva
    out['huellas_identicas'] = out['np.cross']['huellas'] == out['cruz_manual']['huellas']
    out['ahorro_pct'] = round(100.0 * (1.0 - out['cruz_manual']['t_s'] / out['np.cross']['t_s']), 1)
    return out


def orden_actualizacion(semillas):
    out = {}
    for nombre, extra in (('base', {}), ('aw', {'antiwindup': True})):
        for alt in (False, True):
            filas = [run_instr({**BASE, **CUERPOS['titan'], **extra, 'semilla': s}, lazo_alt=alt) for s in semillas]
            out[f'{nombre}_{"orden_alterno" if alt else "orden_normal"}'] = {
                'semilla2': next(f for f, s in zip(filas, semillas) if s == 2) if 2 in semillas else None,
                **_resumen(filas, ['t_llegada_s', 'inclinacion_max_deg'])}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rapido', action='store_true')
    ap.add_argument('--salida', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'dron_resultados',
                                                    'estudio_control_titan.json'))
    a = ap.parse_args()
    ns, nr = (3, 6) if a.rapido else (10, 30)
    semillas = list(range(1, ns + 1))
    t0 = time.time()
    res = {'aviso': 'En simulacion, con los supuestos de atp_dron. Titan: g 1.352, rho 5.4; vehiculo de orden de magnitud.',
           'semillas': semillas, 'escenario': {'base': BASE, 'cuerpos': CUERPOS, 'configs': CONFIGS}}
    with Pool(4) as pool:
        res['barrido'], _ = barrido(pool, semillas)
        print('barrido', round(time.time() - t0), flush=True)
        res['rafagas'] = rafagas(pool, nr)
        print('rafagas', round(time.time() - t0), flush=True)
        res['controles_titan'] = controles_titan(pool, semillas)
    res['orden_actualizacion'] = orden_actualizacion(semillas if a.rapido else semillas)
    res['banco_actitud'] = banco_actitud()
    res['v_terminal'] = v_terminal_medida()
    res['viento_sostenido'] = viento_sostenido()
    res['seg0_y_ruta_vacia'] = seg0_y_ruta_vacia()
    res['cruz'] = cruz_hash_y_tiempo()
    res['limites_autoridad'] = {c: GA.limites_autoridad(v['cuerpo'], v.get('dron'), 35.0)[2] for c, v in CUERPOS.items()}
    res['t_total_s'] = round(time.time() - t0, 1)
    os.makedirs(os.path.dirname(os.path.abspath(a.salida)), exist_ok=True)
    with open(a.salida, 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=float)
    print('listo', a.salida, res['t_total_s'])


if __name__ == '__main__':
    main()
