#!/usr/bin/env python3
"""Estudio del enlace optico tierra-espacio (equipo Y, docs/OPTICO.md). Cada cifra sale del codigo de
`src/atp_optico`; las tablas de docs/OPTICO.md son la salida de este script.

    python3 scripts/optico_estudio.py clave          # cifras clave en ~30 s (el nivel `optico` de validar.sh)
    python3 scripts/optico_estudio.py atmosfera      # r0, theta0, centelleo, tilt, tau0 contra lo publicado
    python3 scripts/optico_estudio.py enlace         # tasa / outage / margen vs elevacion (bajada y subida)
    python3 scripts/optico_estudio.py atp            # residuo del ATP con y sin FSM en pases reales (~25 min)
    python3 scripts/optico_estudio.py keyhole        # la montura hacia el cenit: cuando el FSM se satura (~8 min)
    python3 scripts/optico_estudio.py sensibilidad   # Cn2 en tierra y apertura
    python3 scripts/optico_estudio.py nubes          # disponibilidad con 1..4 estaciones y comparacion con Ka
    python3 scripts/optico_estudio.py todo           # todo lo anterior; guarda docs/optico_resultados/*.json
    python3 scripts/optico_estudio.py comparar A B   # compara dos carpetas de salida (ROS vs sin ROS)

Los parametros del satelite y del telescopio son los de `atp_optico.escenario.ConfigOptico` (SUPUESTOS declarados alli).
"""
import argparse
import dataclasses
import datetime as dt
import json
import math
import os
import sys
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
for p in ('atp_optico', 'atp_ros', 'sat_isl', 'sat_adcs'):
    sys.path.insert(0, os.path.join(RAIZ, 'src', p))
os.environ.setdefault('ATP_DATA_DIR', os.path.join(RAIZ, 'data'))

from atp_optico import atmosfera as A            # noqa: E402
from atp_optico import atp as P                  # noqa: E402
from atp_optico import diversidad as D           # noqa: E402
from atp_optico import enlace as L               # noqa: E402
from atp_optico import escenario as E            # noqa: E402
from atp_optico import montura as M              # noqa: E402
from atp_optico import nubes as N                # noqa: E402
from atp_ros import model, orbit                 # noqa: E402

SALIDA = os.path.join(RAIZ, 'docs', 'optico_resultados')
CFG = E.ConfigOptico()
ELEV = (10, 15, 20, 30, 45, 60, 90)
H_ORBITA_KM = 550.0
SIGMA_ATP_URAD = 0.6            # residuo tipico del FSM de `atp` (por eje) para las tablas casi estaticas


def guardar(nombre, datos):
    os.makedirs(SALIDA, exist_ok=True)
    with open(os.path.join(SALIDA, nombre + '.json'), 'w') as f:
        json.dump(E.redondear(datos), f, indent=1, sort_keys=True)


def tabla(cabecera, filas, fmt=None):
    fmt = fmt or ['{}'] * len(cabecera)
    print('| ' + ' | '.join(cabecera) + ' |')
    print('|' + '|'.join(['---'] * len(cabecera)) + '|')
    for f in filas:
        print('| ' + ' | '.join(fm.format(v) if v is not None else '-' for fm, v in zip(fmt, f)) + ' |')
    print()


def montar(cfg=CFG, estacion=None):
    sat, est, atm, catp = E.construir(dataclasses.replace(cfg, station=estacion or cfg.station))
    return sat, est, atm, catp


# ------------------------------------------------------------------ atmosfera

def estudio_atmosfera():
    out = {}
    print('### Atmosfera: HV 5/7 (A = 1.7e-14, v = 21 m/s)\n')
    hv = A.HV57
    cdmx = dataclasses.replace(hv, alt_estacion_m=2240.0)
    filas = []
    for nom, atm in (('nivel del mar', hv), ('CDMX 2240 m', cdmx)):
        for lam, tag in ((A.LAM_500, '500'), (A.LAM_1064, '1064'), (A.LAM_1550, '1550')):
            r0 = A.fried_r0(atm, lam)
            th = A.theta0(atm, lam)
            filas.append((nom, tag, 100 * r0, 1e6 * th, A.rytov(atm, lam), 1e3 * A.tiempo_coherencia(atm, lam),
                          A.frecuencia_greenwood(atm, lam)))
            out[f'{nom}|{tag}'] = {'r0_cm': 100 * r0, 'theta0_urad': 1e6 * th, 'rytov': A.rytov(atm, lam)}
    tabla(['sitio', 'lam nm', 'r0 cenit cm', 'theta0 urad', 'Rytov cenit', 'tau0 ms (sin barrido)', 'f_G Hz'], filas,
          ['{}', '{}', '{:.2f}', '{:.1f}', '{:.4f}', '{:.2f}', '{:.0f}'])
    print(f'Contraste: HV 5/7 publicado r0 = 5 cm y theta0 = 7 urad a 500 nm: calculado '
          f'{100 * A.fried_r0(hv, A.LAM_500):.2f} cm y {1e6 * A.theta0(hv, A.LAM_500):.2f} urad. '
          f'Escala lam^(6/5): r0(1550)/r0(500) = {A.fried_r0(hv, A.LAM_1550) / A.fried_r0(hv, A.LAM_500):.3f} '
          f'(formula {(A.LAM_1550 / A.LAM_500) ** 1.2:.3f}).\n')
    print('### Efecto de la altitud (modo asl, por defecto, contra agl) a 1550 nm, cenit y 30 deg\n')
    filas = []
    for modo in ('asl', 'agl'):
        a = dataclasses.replace(cdmx, modo_altitud=modo)
        filas.append((modo, 100 * A.fried_r0(a, A.LAM_1550), 100 * A.fried_r0(a, A.LAM_1550, 30.0), A.rytov(a, A.LAM_1550, 30.0)))
    tabla(['modo', 'r0 cenit cm', 'r0 30 deg cm', 'Rytov 30 deg'], filas, ['{}', '{:.2f}', '{:.2f}', '{:.3f}'])
    print('### Por elevacion (CDMX, 1550 nm, telescopio 40 cm, barrido de la linea de vista 10 mrad/s)\n')
    filas = []
    for el in ELEV:
        c = A.centelleo(cdmx, A.LAM_1550, el, 0.4)
        c0 = A.centelleo(cdmx, A.LAM_1550, el, 0.0)
        bw = A.beam_wander(cdmx, 0.4, el, float(model.rango_oblicuo(H_ORBITA_KM, el)) * 1e3)
        filas.append((el, 100 * A.fried_r0(cdmx, A.LAM_1550, el), 1e6 * A.theta0(cdmx, A.LAM_1550, el), c0['sigma_R2'], c0['sigma_I2'],
                      c['sigma_I2'], c['regimen'], 1e6 * A.tilt_llegada_rad(cdmx, A.LAM_1550, 0.4, el), bw['sigma_urad'],
                      1e3 * A.tiempo_coherencia(cdmx, A.LAM_1550, el, 0.010), A.transmision(cdmx, A.LAM_1550, el)))
    out['elevacion'] = filas
    tabla(['el', 'r0 cm', 'theta0 urad', 'sigma_R^2', 'sigma_I^2 puntual', 'sigma_I^2 (40 cm)', 'regimen', 'tilt urad (1 eje)',
           'beam wander urad', 'tau0 ms', 'T atm'], filas,
          ['{}', '{:.1f}', '{:.1f}', '{:.3f}', '{:.3f}', '{:.4f}', '{}', '{:.2f}', '{:.2f}', '{:.2f}', '{:.3f}'])
    ch = [(el, A.centelleo(cdmx, A.LAM_1550, el, 0.4)['sigma_I2'] / A.centelleo(cdmx, A.LAM_1550, el, 0.0)['sigma_I2'],
           A.promediado_apertura_churnside(cdmx, A.LAM_1550, el, 0.4)) for el in (20, 45, 90)]
    print('Promediado por apertura de 40 cm (A&P con L efectiva / Churnside por capas): ' +
          '; '.join(f'{el} deg: {a:.3f} / {c:.3f}' for el, a, c in ch) + '\n')
    guardar('atmosfera', out)
    return out


# ------------------------------------------------------------------ enlace

def estudio_enlace(cfg=CFG):
    sat, est, atm, _ = montar(cfg)
    ppm = dataclasses.replace(est, modem=L.ModemPPM())
    out = {'bajada': [], 'subida': []}
    print(f'### Bajada: satelite de 550 km, {sat.p_tx_w:g} W, theta_h {sat.theta_h_urad:g} urad, telescopio {est.D_m:g} m, '
          f'campo {est.campo_urad:g} urad, error residual {SIGMA_ATP_URAD} urad, outage <= 1 %\n')
    filas = []
    for el in ELEV:
        R = float(model.rango_oblicuo(H_ORBITA_KM, el))
        o = L.evaluar_enlace(sat, est, atm, el, R, SIGMA_ATP_URAD, 1e-2)
        pp = L.evaluar_enlace(sat, ppm, atm, el, R, SIGMA_ATP_URAD, 1e-2)
        des, _ = L.desvanecimiento_enlace(sat, est, atm, el)
        m1 = 1e9
        ber_sin = float(est.modem.ber(o['presupuesto']['p_rx_w'], m1, sat.lam_m))
        ber_con = L.ber_con_centelleo(est.modem, o['presupuesto']['p_rx_w'], m1, sat.lam_m, des)
        m10g = o['p_rx_dbm'] - float(L.E.dbm(est.modem.sensibilidad_w(1e10, sat.lam_m)))
        mppm = pp['p_rx_dbm'] - float(L.E.dbm(ppm.modem.sensibilidad_w(1e9, sat.lam_m, 1e-4)))
        filas.append((el, R, o['p_rx_dbm'], o['sigma_I2'], o['regimen'], o['margen_desv_db'], 1e-9 * o['tasa_bps'], o['outage'],
                      m10g, mppm, ber_sin, ber_con))
        out['bajada'].append(dict(zip(('el', 'R_km', 'p_rx_dbm', 'sigma_I2', 'regimen', 'margen_desv_db', 'ook_gbps', 'outage',
                                       'margen_10G_db', 'margen_ppm_250M_db', 'ber_sin', 'ber_con'), filas[-1])))
    tabla(['el', 'R km', 'P_rx dBm', 'sigma_I^2', 'regimen', 'margen desv. dB (1 %)', 'OOK Gb/s (tope 10)', 'outage', 'margen s/ 10 Gb/s dB',
           'margen PPM-16 a 250 Mb/s dB', 'BER 1 Gb/s sin centelleo', 'BER 1 Gb/s con centelleo'], filas,
          ['{}', '{:.0f}', '{:.1f}', '{:.4f}', '{}', '{:.2f}', '{:.2f}', '{:.1e}', '{:.1f}', '{:.1f}', '{:.1e}', '{:.1e}'])
    s_ook = est.modem.sensibilidad_w(2.5e8 * 255 / 239, sat.lam_m)
    s_ppm = ppm.modem.sensibilidad_w(1e9, sat.lam_m, 1e-4)
    print(f'A la misma tasa bruta de ~250 Mb/s: sensibilidad OOK-APD {float(L.E.dbm(s_ook)):.1f} dBm contra PPM-16 con conteo de fotones '
          f'{float(L.E.dbm(s_ppm)):.1f} dBm (BER 1e-4): el PPM gana {float(L.E.dbm(s_ook)) - float(L.E.dbm(s_ppm)):.1f} dB, a costa de detector de conteo (SNSPD) y de la '
          f'limitacion de ~1e9 ranuras/s.\n')
    print('### Bajada de un microsatelite de 35 mW (clase SOTA: 1549 nm, 35 mW, 5 cm; mismo telescopio y mismo haz de 100 urad)\n')
    sota = dataclasses.replace(sat, p_tx_w=0.035)
    filas = []
    for el in ELEV:
        R = float(model.rango_oblicuo(H_ORBITA_KM, el))
        o = L.evaluar_enlace(sota, est, atm, el, R, SIGMA_ATP_URAD, 1e-2)
        pp = L.evaluar_enlace(sota, ppm, atm, el, R, SIGMA_ATP_URAD, 1e-2)
        mppm = pp['p_rx_dbm'] - float(L.E.dbm(ppm.modem.sensibilidad_w(1e9, sat.lam_m, 1e-4)))
        filas.append((el, R, o['p_rx_dbm'], o['margen_desv_db'], 1e-6 * o['tasa_bps'], o['outage'], mppm))
        out.setdefault('bajada_sota', []).append(dict(zip(('el', 'R_km', 'p_rx_dbm', 'margen_desv_db', 'ook_mbps', 'outage', 'margen_ppm_250M_db'), filas[-1])))
    tabla(['el', 'R km', 'P_rx dBm', 'margen desv. dB (1 %)', 'OOK Mb/s', 'outage', 'margen PPM-16 a 250 Mb/s dB'], filas,
          ['{}', '{:.0f}', '{:.1f}', '{:.2f}', '{:.0f}', '{:.1e}', '{:.1f}'])
    print('### Subida: laser de 20 W de la estacion, desde el telescopio de 40 cm (receptor del satelite de 5 cm)\n')
    filas = []
    variantes = (('theta_up 12 urad, sin precompensar', dataclasses.replace(est)),
                 ('theta_up 12 urad, 80 % precompensado', dataclasses.replace(est, corr_tilt_up=0.8)),
                 ('theta_up 6 urad', dataclasses.replace(est, theta_up_urad=6.0)),
                 ('theta_up 25 urad', dataclasses.replace(est, theta_up_urad=25.0)))
    for nom, e2 in variantes:
        for el in (10, 20, 30, 60, 90):
            R = float(model.rango_oblicuo(H_ORBITA_KM, el))
            r = L.evaluar_enlace(sat, e2, atm, el, R, SIGMA_ATP_URAD, 1e-2, subida=True)
            des, _ = L.desvanecimiento_enlace(sat, e2, atm, el, subida=True)
            sens = sat.modem_sub.sensibilidad_w(1e7, sat.lam_m)         # canal de mando de 10 Mb/s
            filas.append((nom, el, R, r['p_rx_dbm'], r['presupuesto']['beam_wander_urad'], r['presupuesto']['perdida_apunt_db'],
                          r['sigma_I2'], r['p_rx_dbm'] - float(L.E.dbm(sens)), des.outage(sens / r['presupuesto']['p_rx_w'])))
            out['subida'].append(dict(zip(('variante', 'el', 'R_km', 'p_rx_dbm', 'beam_wander_urad', 'perdida_apunt_db', 'sigma_I2',
                                           'margen_10M_db', 'outage_10M'), filas[-1])))
    tabla(['variante', 'el', 'R km', 'P_rx dBm', 'beam wander urad', 'perdida apunt. dB', 'sigma_I^2', 'margen s/ 10 Mb/s dB', 'outage a 10 Mb/s'],
          filas, ['{}', '{}', '{:.0f}', '{:.1f}', '{:.2f}', '{:.2f}', '{:.4f}', '{:.1f}', '{:.1e}'])
    # contraste con OPALS
    o = L.presupuesto_bajada(L.OPALS_SAT, L.OPALS_EST, dataclasses.replace(A.HV57, alt_estacion_m=2200.0), 30.0, 820.0)
    print(f'Contraste OPALS (ISS, 2.5 W, FWHM 1 mrad, OCTL de 1 m, 30 deg, 820 km): G_tx = '
          f'{o["db"]["G_tx"]:.1f} dB (publicado 70.4) y P_rx = {1e9 * o["p_rx_w"]:.0f} nW (prediccion publicada: cientos de nW; medida: decenas).\n')
    guardar('enlace', out)
    return out


# ------------------------------------------------------------------ ATP

def correr_pase(cfg, sat, pase_idx, inicio, mask, mont=None, perfil=None, montura=None):
    t0 = time.time()
    med, filas, meta = E.simular(cfg, sat, pase_idx, inicio, mask, mont=mont, perfil=perfil, montura=montura)
    r = med.resumen_dict()
    r['aos'] = str(meta['pase'].get('aos')) if meta['pase'].get('aos') else None
    r['el_max_deg'] = meta['pase']['el_max_deg']
    r['t_pared_s'] = time.time() - t0
    return r, meta, filas


def estudio_atp(cfg=CFG, rapido=False):
    inicio = '2026-10-02T15:00:00'
    out = {'pases': [], 'variantes': []}
    pases = [('ISS', i) for i in (0, 1, 2, 3)] + [('NOAA 20', 0)]
    if rapido:
        pases = pases[:1]
    print('### ATP optico en pases SGP4 reales sobre CDMX (mascara 10 deg; montura de atp_ros, PID de 20 Hz, sin viento)\n')
    filas = []
    monts = {}
    for sat, idx in pases:
        r, meta, _ = correr_pase(cfg, sat, idx, inicio, 10.0)
        monts[(sat, idx)] = meta['montura']
        e_m = 1e6 * math.radians(math.sqrt(np.mean(meta['montura']['e_pt_deg'] ** 2)) / math.sqrt(2.0))
        filas.append((f'{sat} #{idx}', r['el_max_deg'], r['duracion_s'], e_m, r['rms_tilt_urad'], r['rms_sin_fsm_urad'], r['rms_con_fsm_urad'],
                      r['p95_con_fsm_urad'], r['rms_sobre_campo'], r['y_max_urad'], r['enganchado_pct'],
                      r['sin_fsm']['cerrado_pct'], r['con_fsm']['cerrado_pct'], r['sin_fsm']['gb'], r['con_fsm']['gb']))
        r['montura_rms_eje_urad'] = e_m
        out['pases'].append({'pase': f'{sat} #{idx}', **r})
    tabla(['pase', 'el max', 'dur s', 'montura rms/eje urad', 'tilt rms urad', 'sin FSM rms urad', 'con FSM rms urad', 'con FSM p95',
           'rms/campo', 'y max urad', 'FSM enganchado %', 'cerrado sin FSM %', 'cerrado con FSM %', 'GB sin FSM', 'GB con FSM'],
          filas, ['{}', '{:.1f}', '{:.0f}', '{:.1f}', '{:.2f}', '{:.1f}', '{:.2f}', '{:.2f}', '{:.3f}', '{:.0f}', '{:.1f}', '{:.1f}', '{:.1f}',
                  '{:.1f}', '{:.1f}'])
    if not rapido:
        print('### Variantes sobre dos pases (la misma montura): receptor, viento en la montura, camino no comun\n')
        filas = []
        for sat, idx in (('ISS', 3), ('ISS', 0)):
            mont0 = monts[(sat, idx)]
            for nom, c2, viento in (('base (campo 12 urad)', cfg, None),
                                    ('campo 4 urad', dataclasses.replace(cfg, campo_urad=4.0), None),
                                    ('fibra monomodo (campo = 3.7 urad)', dataclasses.replace(cfg, receptor='smf'), None),
                                    ('sin camino no comun ni modelo de apuntamiento', dataclasses.replace(cfg, ncpa_urad=0.0, modelo_apunt_urad=0.0), None),
                                    ('modelo de apuntamiento SIN calibrar (0.52 deg)', dataclasses.replace(cfg, modelo_apunt_urad=0.52 * math.pi / 180 * 1e6 / math.sqrt(2)), None),
                                    ('sensor de camara de 1 kHz (cruce del lazo a 80 Hz)', dataclasses.replace(cfg, f_cruce_hz=80.0), None),
                                    ('viento en la montura (0.02 N m rms, el doble del de la campana de atp_ros)', cfg, 0.02)):
                mont = mont0
                if viento:
                    perfil, _, _, _ = M.perfil_pase(sat, cfg.station, idx, inicio, 72.0, 10.0)
                    mont = M.simular_montura(perfil, M.ConfigMontura(paso_s=cfg.paso_s, wind_sigma_nm=viento, semilla=cfg.semilla))
                r, _, _ = correr_pase(c2, sat, idx, inicio, 10.0, mont=mont)
                filas.append((f'{sat} #{idx}', nom, r['rms_sin_fsm_urad'], r['rms_con_fsm_urad'], r['rms_sobre_campo'], r['y_max_urad'],
                              r['enganchado_pct'], r['con_fsm']['cerrado_pct'], r['con_fsm']['gb'], r['sin_fsm']['gb']))
                out['variantes'].append(dict(zip(('pase', 'variante', 'rms_sin', 'rms_con', 'rms_sobre_campo', 'y_max', 'enganchado_pct',
                                                  'cerrado_con_pct', 'gb_con', 'gb_sin'), filas[-1])))
        tabla(['pase', 'variante', 'sin FSM rms', 'con FSM rms', 'rms/campo', 'y max', 'enganchado %', 'cerrado con FSM %', 'GB con FSM', 'GB sin'],
              filas, ['{}', '{}', '{:.1f}', '{:.2f}', '{:.3f}', '{:.0f}', '{:.1f}', '{:.1f}', '{:.1f}', '{:.1f}'])
    # linea base: montura cerrada sobre el sensor (analitica), a 30 y 60 deg
    cdmx = dataclasses.replace(A.HV57, alt_estacion_m=2240.0)
    print('### Linea base analitica: montura cerrada sobre el sensor optico (sin FSM), tilt de 40 cm\n')
    filas = []
    for el, slew in ((20, 0.006), (45, 0.012), (60, 0.020)):
        sig = 1e6 * A.tilt_llegada_rad(cdmx, A.LAM_1550, 0.4, el)
        v = A.viento_tilt(cdmx, el, slew)
        fc, fl = 0.3 * v / 0.4, v / 25.0
        filas.append((el, 1e3 * slew, sig, fc, P.residuo_montura_cerrada_urad(sig, fc, fl), P.residuo_fsm_analitico_urad(sig, fc, fl)))
    out['linea_base'] = filas
    tabla(['el', 'barrido mrad/s', 'tilt rms urad', 'f_c tilt Hz', 'montura cerrada sobre sensor urad', 'FSM (analitico, solo tilt) urad'], filas,
          ['{}', '{:.0f}', '{:.2f}', '{:.0f}', '{:.2f}', '{:.3f}'])
    guardar('atp', out)
    return out


def estudio_keyhole(cfg=CFG, rapido=False, vel_max=0.0):
    """Pases sinteticos de 550 km (modelo circular de atp_ros) con elevacion maxima creciente: el error de la montura
    (la de atp_ros, PID de 20 Hz, tope de par 8 N m, SIN estrategia de keyhole) crece hacia el cenit hasta salirse del recorrido del FSM."""
    tope = f'rotor de {vel_max:g} deg/s por eje (el de' if vel_max > 0 else 'sin tope de velocidad (el defecto de atp_loop)'
    print(f'### Hacia el cenit: pases sinteticos de 550 km, mascara 10 deg, montura de atp_ros sin estrategia de keyhole, {tope}\n')
    out = []
    filas = []
    for el_max in ((60.0, 85.0) if rapido else ((60.0, 80.0, 85.0, 88.0, 89.5) if vel_max <= 0 else (60.0, 80.0, 85.0, 88.0))):
        perfil = model.perfil_pase(H_ORBITA_KM, el_max, 10.0, 140.0, n=12000)
        r, meta, _ = correr_pase(cfg, 'sintetico', 0, '', 10.0, perfil=perfil,
                                 montura=M.ConfigMontura(paso_s=cfg.paso_s, semilla=cfg.semilla, vel_max_deg_s=vel_max))
        m = meta['montura']
        e_m = 1e6 * math.radians(math.sqrt(np.mean(m['e_pt_deg'] ** 2)) / math.sqrt(2.0))
        pico = 1e6 * math.radians(float(np.max(m['e_pt_deg'])))
        filas.append((el_max, r['duracion_s'], float(np.max(np.abs(perfil['az_pto']))), e_m, pico, r['y_max_urad'], r['enganchado_pct'],
                      r['rms_con_fsm_urad'], r['con_fsm']['cerrado_pct'], r['con_fsm']['gb'], r['sin_fsm']['gb']))
        out.append(dict(zip(('el_max', 'dur_s', 'az_pto_max_deg_s', 'montura_rms_eje_urad', 'montura_pico_urad', 'y_max_urad', 'enganchado_pct',
                             'rms_con_fsm', 'cerrado_con_pct', 'gb_con', 'gb_sin'), filas[-1])))
    tabla(['el max', 'dur s', 'az max deg/s', 'montura rms/eje urad', 'montura pico urad', 'y max urad', 'FSM enganchado %', 'con FSM rms urad',
           'cerrado con FSM %', 'GB con FSM', 'GB sin FSM'], filas, ['{}', '{:.0f}', '{:.1f}', '{:.0f}', '{:.0f}', '{:.0f}', '{:.1f}', '{:.2f}', '{:.1f}', '{:.0f}', '{:.0f}'])
    guardar('keyhole' if vel_max <= 0 else 'keyhole_rotor6', out)
    return out


# ------------------------------------------------------------------ sensibilidades

def estudio_sensibilidad(cfg=CFG):
    sat, est0, atm0, _ = montar(cfg)
    out = []
    print('### Sensibilidad a Cn2 en tierra y a la apertura (bajada a 30 deg y a 15 deg de un satelite de 550 km, OOK, error residual 0.6 urad)\n')
    filas = []
    for el in (30, 15):
        R = float(model.rango_oblicuo(H_ORBITA_KM, el))
        for A0 in (1e-15, 5e-15, 1.7e-14, 5e-14, 1e-13):
            atm = dataclasses.replace(atm0, cn2_suelo=A0)
            for D in (0.2, 0.4, 0.6):
                est = dataclasses.replace(est0, D_m=D, D_tx_m=D)
                r = L.evaluar_enlace(sat, est, atm, float(el), R, SIGMA_ATP_URAD, 1e-2)
                r0 = A.fried_r0(atm, sat.lam_m, el)
                sig = 1e6 * A.tilt_llegada_rad(atm, sat.lam_m, D, el)
                v = A.viento_tilt(atm, el, 0.010)
                resid = P.residuo_fsm_analitico_urad(sig, 0.3 * v / D, v / 25.0)
                filas.append((el, f'{A0:.1e}', D, D / r0, r['sigma_I2'], r['margen_desv_db'], r['p_rx_dbm'], 1e-9 * r['tasa_bps'], sig, resid))
                out.append(dict(zip(('el', 'cn2_suelo', 'D', 'D_r0', 'sigma_I2', 'margen_desv_db', 'p_rx_dbm', 'tasa_gbps', 'tilt_urad', 'resid_fsm_urad'), filas[-1])))
    tabla(['el', 'Cn2 suelo', 'D m', 'D/r0', 'sigma_I^2', 'margen desv. dB', 'P_rx dBm', 'OOK Gb/s', 'tilt urad', 'residuo FSM (solo tilt) urad'], filas,
          ['{}', '{}', '{:.1f}', '{:.2f}', '{:.4f}', '{:.2f}', '{:.1f}', '{:.2f}', '{:.2f}', '{:.3f}'])
    guardar('sensibilidad', out)
    return out


# ------------------------------------------------------------------ nubes y diversidad

def estudio_nubes(cfg=CFG, dias=14, semillas=40, rapido=False):
    estaciones, _ = orbit.cargar_estaciones()
    sitios, mod = N.cargar_clima()
    orden = ['cdmx', 'madrid', 'ecuador', 'tromso']
    out = {'tiempo': [], 'pases': []}
    print('### Disponibilidad de linea de vista (fraccion del TIEMPO) con 1..4 estaciones (1 ano horario, 40 semillas)\n')
    filas = []
    n_anio = 8760 if not rapido else 2000
    for k in range(1, 5):
        sub = orden[:k]
        ps = [sitios[c].p_claro for c in sub]
        disp = [N.disponibilidad(N.serie_nubes([sitios[c] for c in orden], n_anio, mod['paso_h'], mod['tau_h'], mod['L_km'], 3000 + s), range(k))
                for s in range(semillas if not rapido else 5)]
        filas.append((k, '+'.join(sub), 100 * np.mean(disp), 100 * np.std(disp), 100 * N.disponibilidad_independiente(ps), 100 * max(ps)))
        out['tiempo'].append(dict(zip(('n', 'estaciones', 'disp_pct', 'std_pct', 'indep_pct', 'mejor_sola_pct'), filas[-1])))
    tabla(['N', 'estaciones', 'disponibilidad % (media)', 'desv. entre anos %', 'cota independiente %', 'mejor sola %'], filas,
          ['{}', '{}', '{:.1f}', '{:.1f}', '{:.1f}', '{:.1f}'])
    # por pases reales
    t0 = dt.datetime(2026, 10, 2, 0, 0, tzinfo=dt.timezone.utc)
    sat, est, atm0, _ = montar(cfg)
    atms = {k: dataclasses.replace(atm0, alt_estacion_m=float(estaciones[k].alt_m)) for k in orden}
    t_ini = time.time()
    pases = D.pases_utiles('NOAA 20', orden, estaciones, t0, dias if not rapido else 2, 10.0, atms, (sat, est), SIGMA_ATP_URAD)
    print(f'### Pases de NOAA 20 (824 km, polar) en {dias} dias: GB entregados por dia con el enlace optico de bajada '
          f'({len(pases)} pases, {time.time() - t_ini:.0f} s de calculo)\n')
    por_est = {k: [p for p in pases if p.estacion == k] for k in orden}
    resumen_p = [(k, len(v), np.mean([p.el_max for p in v]), np.mean([p.dur_s for p in v]), np.mean([p.gb for p in v])) for k, v in por_est.items()]
    tabla(['estacion', 'pases', 'el max media', 'duracion media s', 'GB por pase (sin nubes)'], resumen_p, ['{}', '{}', '{:.1f}', '{:.0f}', '{:.0f}'])
    out['pases_resumen'] = resumen_p
    rng = np.random.default_rng(77)
    tasa_rf = 0.5e9
    filas = []
    n_h = (dias if not rapido else 2) * 24
    for k in range(1, 5):
        sub = orden[:k]
        opt, pos, nada, rf999, rf9999, dias_opt, dias_rf = [], [], [], [], [], [], []
        for s in range(semillas if not rapido else 5):
            claro = N.serie_nubes([sitios[c] for c in orden], n_h, mod['paso_h'], mod['tau_h'], mod['L_km'], 5000 + s)
            g, p_, pd = D.entrega(pases, claro, orden, t0, set(sub))
            opt.append(g / (dias if not rapido else 2))
            pos.append(p_ / (dias if not rapido else 2))
            nada.append(float(np.mean(pd < 1.0)))
            dias_opt.extend(pd.tolist())
            for lista, disp in ((rf999, 0.999), (rf9999, 0.9999)):
                g_rf, pd_rf = D.entrega_rf(pases, set(sub), t0, dias if not rapido else 2, tasa_rf, disp, rng)
                lista.append(g_rf / (dias if not rapido else 2))
                if disp == 0.999:
                    dias_rf.extend(pd_rf.tolist())
        filas.append((k, '+'.join(sub), np.mean(pos), np.mean(opt), 100 * np.mean(opt) / max(np.mean(pos), 1e-9), 100 * np.mean(nada),
                      float(np.percentile(dias_opt, 10)), np.mean(rf999), float(np.percentile(dias_rf, 10))))
        out['pases'].append(dict(zip(('n', 'estaciones', 'gb_dia_posible', 'gb_dia_optico', 'frac_pct', 'dias_sin_datos_pct', 'gb_dia_optico_p10',
                                      'gb_dia_ka_999', 'gb_dia_ka_p10'), filas[-1])))
    tabla(['N', 'estaciones', 'GB/dia posible (sin nubes)', 'GB/dia optico (media)', '% de lo posible', 'dias sin ningun GB %',
           'GB/dia optico (p10)', 'GB/dia Ka 0.5 Gb/s 99.9 % (media)', 'GB/dia Ka (p10)'], filas,
          ['{}', '{}', '{:.0f}', '{:.0f}', '{:.1f}', '{:.1f}', '{:.0f}', '{:.0f}', '{:.0f}'])
    # una estacion sola: cada una
    filas = []
    for c in orden:
        pos, opt, nada = [], [], []
        for s in range(semillas if not rapido else 5):
            claro = N.serie_nubes([sitios[x] for x in orden], n_h, mod['paso_h'], mod['tau_h'], mod['L_km'], 5000 + s)
            g, p_, pd = D.entrega(pases, claro, orden, t0, {c})
            opt.append(g / (dias if not rapido else 2))
            pos.append(p_ / (dias if not rapido else 2))
            nada.append(float(np.mean(pd < 1.0)))
        filas.append((c, sitios[c].p_claro * 100, np.mean(pos), np.mean(opt), 100 * np.mean(nada)))
    tabla(['estacion sola', 'CFLOS %', 'GB/dia posible', 'GB/dia optico', 'dias sin datos %'], filas, ['{}', '{:.0f}', '{:.0f}', '{:.0f}', '{:.1f}'])
    out['una_sola'] = filas
    print('RF Ka: margen de lluvia para 99.9 % = ' + f'{D.margen_para_disponibilidad(0.999):.1f} dB, para 99.99 % = {D.margen_para_disponibilidad(0.9999):.1f} dB (templado); '
          f'ecuatorial 99.9 % = 31 dB.\n')
    guardar('nubes', out)
    return out


# ------------------------------------------------------------------ clave y comparar

def estudio_clave(cfg=CFG):
    hv = A.HV57
    print('=== CIFRAS CLAVE (atp_optico) ===')
    print(f'HV 5/7: r0(500 nm, cenit) = {100 * A.fried_r0(hv, A.LAM_500):.2f} cm (publicado 5), theta0 = {1e6 * A.theta0(hv, A.LAM_500):.2f} urad (7); '
          f'r0(1550) = {100 * A.fried_r0(hv, A.LAM_1550):.1f} cm, escala lam^(6/5) {A.fried_r0(hv, A.LAM_1550) / A.fried_r0(hv, A.LAM_500):.3f} = {(A.LAM_1550 / A.LAM_500) ** 1.2:.3f}')
    cdmx = dataclasses.replace(hv, alt_estacion_m=2240.0)
    for el in (90, 30, 10):
        c = A.centelleo(cdmx, A.LAM_1550, el, 0.4)
        print(f'CDMX 1550 nm el={el}: sigma_R^2 = {c["sigma_R2"]:.3f} ({c["regimen"]}), sigma_I^2 (40 cm) = {c["sigma_I2"]:.4f}, '
              f'tilt {1e6 * A.tilt_llegada_rad(cdmx, A.LAM_1550, 0.4, el):.2f} urad, beam wander subida '
              f'{A.beam_wander(cdmx, 0.4, el, float(model.rango_oblicuo(H_ORBITA_KM, el)) * 1e3)["sigma_urad"]:.2f} urad')
    sat, est, atm, _ = montar(cfg)
    print('enlace de bajada (550 km, OOK-APD, outage <= 1 %): el  P_rx dBm  sigma_I^2  Gb/s  outage')
    for el in (10, 20, 30, 60, 90):
        r = L.evaluar_enlace(sat, est, atm, float(el), float(model.rango_oblicuo(H_ORBITA_KM, el)), SIGMA_ATP_URAD, 1e-2)
        print(f'  {el:3d}  {r["p_rx_dbm"]:7.1f}  {r["sigma_I2"]:.4f}  {1e-9 * r["tasa_bps"]:5.2f}  {r["outage"]:.1e}')
    o = L.presupuesto_bajada(L.OPALS_SAT, L.OPALS_EST, dataclasses.replace(hv, alt_estacion_m=2200.0), 30.0, 820.0)
    print(f'OPALS: G_tx = {o["db"]["G_tx"]:.1f} dB (70.4 publicado), P_rx = {1e9 * o["p_rx_w"]:.0f} nW (cientos de nW predichos)')
    cl = N.serie_nubes([N.Sitio('a', 36.84, -2.46, .65), N.Sitio('b', 52.9, 10., .25), N.Sitio('c', 48.08, 11.67, .34), N.Sitio('d', 53.36, 13.07, .32)],
                       200000, 1.0, 8.0, 500.0, 6)
    print(f'red alemana (4 sitios de 25-65 %): modelo {100 * N.disponibilidad(cl):.1f} % (publicado 83.68; independiente 88.2)')


def comparar(a, b):
    ja = json.load(open(os.path.join(a, 'optico.json')))
    jb = json.load(open(os.path.join(b, 'optico.json')))
    print('comparacion ROS (lockstep) vs sin ROS, campo por campo del resumen:')
    dif = 0
    for k in ('n_pasos', 'rms_montura_urad', 'rms_tilt_urad', 'rms_con_fsm_urad', 'rms_sin_fsm_urad', 'p95_con_fsm_urad', 'y_max_urad'):
        rel = abs(ja[k] - jb[k]) / max(abs(jb[k]), 1e-12)
        print(f'  {k:20s} ROS {ja[k]:<12.6g} sin ROS {jb[k]:<12.6g} dif relativa {rel:.1e}')
        dif = max(dif, rel)
    print(f"  cerrado con FSM {ja['con_fsm']['cerrado_pct']:.1f} % vs {jb['con_fsm']['cerrado_pct']:.1f} %; GB {ja['con_fsm']['gb']:.2f} vs {jb['con_fsm']['gb']:.2f}")
    print('EQUIVALENTES (dif max %.1e)' % dif if dif < 5e-3 else 'DIFIEREN (dif max %.1e)' % dif)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('que', choices=('clave', 'atmosfera', 'enlace', 'atp', 'keyhole', 'keyhole6', 'sensibilidad', 'nubes', 'todo', 'comparar'))
    ap.add_argument('dirs', nargs='*')
    ap.add_argument('--rapido', action='store_true')
    a = ap.parse_args()
    if a.que == 'clave':
        estudio_clave()
    elif a.que == 'comparar':
        comparar(*a.dirs)
    elif a.que == 'todo':
        estudio_atmosfera(); estudio_enlace(); estudio_sensibilidad(); estudio_nubes(rapido=a.rapido); estudio_atp(rapido=a.rapido); estudio_keyhole(rapido=a.rapido); estudio_keyhole(rapido=a.rapido, vel_max=6.0)
    else:
        {'atmosfera': estudio_atmosfera, 'enlace': estudio_enlace, 'sensibilidad': estudio_sensibilidad,
         'nubes': lambda: estudio_nubes(rapido=a.rapido), 'atp': lambda: estudio_atp(rapido=a.rapido),
         'keyhole': lambda: estudio_keyhole(rapido=a.rapido), 'keyhole6': lambda: estudio_keyhole(rapido=a.rapido, vel_max=6.0)}[a.que]()


if __name__ == '__main__':
    main()
