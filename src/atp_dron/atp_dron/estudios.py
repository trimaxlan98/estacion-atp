"""Banco de experimentos del dron: cada funcion corre simulaciones con semilla y devuelve cifras (sin ROS).

Todo es "en simulacion, con estos supuestos": los parametros del vehiculo son de orden de magnitud (ver dron.py).
"""
import numpy as np

from atp_dron.cuerpos import RHO_REF, cuerpo
from atp_dron.dron import G0, PARAMS
from atp_dron.lazo import correr


def empuje_peso(cuerpo_nombre, masa_kg=None, rotor_escala=1.0, empuje_peso_ref=None):
    """Empuje maximo / peso (analitico): kT escala con rho y rotor_escala; el peso con g del cuerpo."""
    c = cuerpo(cuerpo_nombre)
    tw0 = PARAMS['empuje_peso'] if empuje_peso_ref is None else empuje_peso_ref
    m_ref = PARAMS['masa_kg']
    m = m_ref if masa_kg is None else masa_kg
    # el empuje maximo es el de la masa de referencia; con otra masa solo cambia el peso
    return tw0 * (m_ref / m) * (c['rho'] / RHO_REF) * rotor_escala * (G0 / c['g'])


def rotor_escala_minima(cuerpo_nombre, tw_objetivo=1.3):
    """Factor de tamano de rotor para tener `tw_objetivo` de empuje/peso (kT ~ rho * escala)."""
    return tw_objetivo / empuje_peso(cuerpo_nombre)


def potencia_hover_w(cuerpo_nombre, masa_kg=None, params=None):
    """Potencia de hover analitica (teoria de cantidad de movimiento) con los parametros dados."""
    P = {**PARAMS, **(params or {})}
    m = P['masa_kg'] if masa_kg is None else masa_kg
    c = cuerpo(cuerpo_nombre)
    A = np.pi * P['radio_rotor_m'] ** 2
    T = m * c['g'] / 4.0
    return float(4 * T ** 1.5 / (np.sqrt(2 * c['rho'] * A) * P['fom'] * P['eta_motor']) + P['p_avionica_w'])


def respuesta_escalon(delta_m=5.0, z=10.0, cuerpo_nombre='tierra', semilla=0, **cfg):
    """Escalon de posicion horizontal en vuelo estacionario: tiempo de subida 10-90 %, sobreimpulso y asentamiento (2 %).

    La meta usa frenar=True, asi que el perfil es el de frenado sqrt(2 a d): lo medido es el sistema completo
    (guiado + cascada), no la planta sola.
    """
    c = {'cuerpo': cuerpo_nombre, 'semilla': semilla, 'ruta': [[0, 0, z], [delta_m, 0, z]], 'duracion_s': 40.0,
         'parar_al_llegar': False, 'hz_telemetria': 50.0, 'radio_meta_m': 0.3}
    c.update(cfg)
    m, f = correr(c)
    t = np.array([r['t'] for r in f])
    x = np.array([r['x'] for r in f])
    meta = np.array([r['meta'] for r in f])
    k0 = int(np.argmax(meta >= 1)) if (meta >= 1).any() else len(f)
    if k0 >= len(f):
        return {'llego': False}
    t0, xs = t[k0:], x[k0:]
    def cruce(frac):
        i = np.argmax(xs >= frac * delta_m)
        return float(t0[i] - t0[0]) if xs.max() >= frac * delta_m else None
    t10, t90 = cruce(0.1), cruce(0.9)
    fuera = np.abs(xs - delta_m) > 0.02 * delta_m
    t_as = float(t0[np.nonzero(fuera)[0][-1] + 1] - t0[0]) if fuera.any() and np.nonzero(fuera)[0][-1] + 1 < len(t0) else (
        None if fuera.any() else 0.0)
    return {'llego': bool(m['llego'] or xs.max() > 0.9 * delta_m), 'delta_m': delta_m,
            't_subida_10_90_s': None if t10 is None or t90 is None else round(t90 - t10, 3),
            'sobreimpulso_pct': round(float(max(0.0, (xs.max() - delta_m) / delta_m * 100.0)), 2),
            't_asentamiento_2pct_s': None if t_as is None else round(t_as, 3)}


def barrido_viento(velocidades=(0, 2, 4, 6, 8, 10, 12), z=10.0, duracion_s=40.0, semilla=0, rafaga_sigma=0.0, **cfg):
    """Mantener posicion contra viento constante (+ rafagas): desviacion, inclinacion y potencia en cada nivel."""
    filas = []
    for u in velocidades:
        c = {'ruta': [[0, 0, z]], 'duracion_s': duracion_s, 'parar_al_llegar': False, 'viento': [float(u), 0, 0],
             'semilla': semilla, 'rafaga_sigma': rafaga_sigma, 'hz_telemetria': 10.0}
        c.update(cfg)
        m, f = correr(c)
        cola = [r for r in f if r['t'] > duracion_s * 0.5]
        dev = max((np.hypot(r['x'], r['y']) for r in cola), default=float('nan'))
        filas.append({'viento_ms': float(u), 'desviacion_max_m': round(float(dev), 3),
                      'inclinacion_max_deg': m['inclinacion_max_deg'], 'energia_Wh': m['energia_Wh'],
                      'choque': m['choque']})
    return filas


def barrido_carga(cargas_kg=(0.0, 0.25, 0.5, 0.75, 1.0), cuerpo_nombre='tierra', bateria_wh=60.0, params=None):
    """Carga util: empuje/peso, potencia de hover y autonomia en hover (analiticos) con bateria de `bateria_wh`."""
    filas = []
    m0 = {**PARAMS, **(params or {})}['masa_kg']
    for q in cargas_kg:
        m = m0 + q
        tw = empuje_peso(cuerpo_nombre, masa_kg=m, rotor_escala=(params or {}).get('rotor_escala', 1.0))
        p = potencia_hover_w(cuerpo_nombre, masa_kg=m, params=params)
        filas.append({'carga_kg': q, 'masa_total_kg': m, 'empuje_peso': round(float(tw), 3),
                      'puede_volar': bool(tw > 1.0), 'p_hover_w': round(p, 1),
                      'autonomia_min': round(bateria_wh / p * 60.0, 1) if tw > 1.0 else 0.0})
    return filas


def comparar_cuerpos(rotor_escalas=None, z=10.0):
    """Mismo dron en cada cuerpo: empuje/peso, rotor minimo para volar con margen 1.3, hover simulado si puede."""
    rotor_escalas = rotor_escalas or {}
    filas = []
    for nombre in ('tierra', 'marte', 'titan', 'luna'):
        esc = rotor_escalas.get(nombre, 1.0)
        tw = empuje_peso(nombre, rotor_escala=esc)
        fila = {'cuerpo': nombre, 'rotor_escala': esc, 'empuje_peso': round(float(tw), 4),
                'rotor_escala_min_tw1.3': round(float(rotor_escala_minima(nombre)), 2) if cuerpo(nombre)['rho'] > 1e-6 else None}
        if tw > 1.2 and cuerpo(nombre)['rho'] > 1e-6:
            m, _ = correr({'cuerpo': nombre, 'dron': {'rotor_escala': esc}, 'ruta': [[0, 0, z]], 'duracion_s': 30.0,
                           'parar_al_llegar': False})
            fila.update({'p_hover_w': round(potencia_hover_w(nombre, params={'rotor_escala': esc}), 1),
                         'energia_Wh_30s': m['energia_Wh'], 'choque': m['choque']})
        filas.append(fila)
    return filas


def montecarlo_rafagas(n=20, sigma=1.0, viento=2.0, ruta=((30, 0, 10), (30, 30, 10), (0, 0, 10)), semilla0=100, **cfg):
    """n semillas de rafagas sobre la misma ruta: media, desviacion y percentil 95 del error transversal y la energia."""
    et, en, tl, ch = [], [], [], 0
    for k in range(n):
        c = {'ruta': [list(p) for p in ruta], 'duracion_s': 90.0, 'viento': [viento, 0, 0], 'rafaga_sigma': sigma,
             'semilla': semilla0 + k}
        c.update(cfg)
        m, _ = correr(c)
        et.append(m['error_transversal_rms_m'])
        en.append(m['energia_Wh'])
        if m['llego']:
            tl.append(m['t_llegada_s'])
        ch += int(m['choque'])
    f = lambda a: {'media': round(float(np.mean(a)), 4), 'desv': round(float(np.std(a, ddof=1)), 4) if len(a) > 1 else 0.0,
                   'p95': round(float(np.percentile(a, 95)), 4)} if len(a) else None
    return {'n': n, 'llegaron': len(tl), 'choques': ch, 'error_transversal_rms_m': f(et), 'energia_Wh': f(en),
            't_llegada_s': f(tl)}


def efecto_suelo_hover(alturas=(0.1, 0.2, 0.3, 0.5, 1.0, 2.0), cuerpo_nombre='tierra'):
    """Empuje extra por efecto suelo a igual velocidad de giro (analitico) y reduccion de potencia a igual empuje."""
    R = PARAMS['radio_rotor_m']
    out = []
    for z in alturas:
        k = 1.0 / (1.0 - (R / (4.0 * max(z, R / 2.0))) ** 2)
        out.append({'z_m': z, 'z_sobre_R': round(z / R, 2), 'T_sobre_Tinf': round(float(k), 4),
                    'P_sobre_Pinf_a_igual_T': round(float(k ** -0.5), 4)})
    return out


def estimador_montecarlo(n=10, ruta=((30, 0, 10), (30, 30, 10), (0, 0, 10)), semilla0=200, **cfg):
    """n semillas con sensores y ESKF en lazo cerrado: error de estimacion y su efecto sobre el seguimiento de la ruta."""
    ep, ea, et, ll = [], [], [], 0
    for k in range(n):
        c = {'ruta': [list(p) for p in ruta], 'duracion_s': 90.0, 'viento': [2.0, 0, 0], 'rafaga_sigma': 1.0,
             'semilla': semilla0 + k, 'sensores': True}
        c.update(cfg)
        m, _ = correr(c)
        ep.append(m['est_error_pos_rms_m'])
        ea.append(m['est_error_actitud_rms_deg'])
        et.append(m['error_transversal_rms_m'])
        ll += int(m['llego'])
    f = lambda a: {'media': round(float(np.mean(a)), 4), 'p95': round(float(np.percentile(a, 95)), 4)}
    return {'n': n, 'llegaron': ll, 'error_pos_rms_m': f(ep), 'error_actitud_rms_deg': f(ea), 'error_transversal_rms_m': f(et)}


def perdida_gps(cortes_s=(None, 30.0, 15.0), duracion_s=60.0, semilla=11):
    """Mantener posicion a 10 m y perder el GPS en `corte`: deriva de la estimacion y de la posicion verdadera al final."""
    filas = []
    for corte in cortes_s:
        m, f = correr({'ruta': [[0, 0, 10]], 'duracion_s': duracion_s, 'parar_al_llegar': False, 'viento': [1.0, 0, 0],
                       'rafaga_sigma': 0.5, 'semilla': semilla, 'sensores': True,
                       'sensores_params': {'gps_corte_s': corte}})
        ult = f[-1]
        filas.append({'corte_gps_s': corte, 'est_error_pos_final_m': m['est_error_pos_final_m'],
                      'pos_verdadera_final_m': round(float(np.hypot(ult['x'], ult['y'])), 3),
                      'est_error_actitud_rms_deg': m['est_error_actitud_rms_deg'], 'choque': m['choque']})
    return filas
