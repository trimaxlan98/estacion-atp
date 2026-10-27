#!/usr/bin/env python3
"""Estudio R8: operacion del rover con retardo de enlace (docs/ROVER.md «Resultados R8», CD-2026-36).

    python3 scripts/estudio_operacion_rover.py --rapido --trabajadores 4   # 1 semilla, rejillas gruesas; escribe en runs/rover_operacion_rapido/
    python3 scripts/estudio_operacion_rover.py --trabajadores 6            # completo (~10 min); escribe docs/rover_resultados/operacion.{json,md}
    python3 scripts/estudio_operacion_rover.py --recalibrar                # rehace la calibracion con la fisica de R1 (~6 min con 6 procesos)

Todo es simulacion con los supuestos de `atp_rover/operacion.py` (rover sustituto cinematico calibrado con la dinamica de R1; peligros
por regla; enlace de R7). Determinista: dos corridas dan los mismos bytes. Un resultado «a favor de la autonomia» o «en contra» vale
solo para estos supuestos; ver «Resultados R8 / Sin probar».
"""
import argparse
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_rover'))
from atp_rover import operacion as O  # noqa: E402

CAL_ARCHIVO = os.path.join(RAIZ, 'docs', 'rover_resultados', 'operacion_calibracion.json')
TERRENOS = ['llano', 'rocoso', 'dunas', 'rocas', 'pendiente']
MODOS = list(O.MODOS)
T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365}
FUENTES = [
    ('Mars Relay Description for Discovery 2019 Proposals (JPL, 2019): ventanas y volumen de los reles (via R7)',
     'https://discovery.larc.nasa.gov/PDF_FILES/21_Proposers_Guide_To_Mars_Orbiters_-_Discovery_2019_AO_-_Rev_190411b.pdf'),
    ('Sheridan, T. B. (1993) Space teleoperation through time delay: review and prognosis, IEEE Trans. Robotics and Automation 9(5) (mover y esperar; '
     'recordado, no recontrastado)', None),
    ('Smith, O. J. M. (1957) Closer control of loops with dead time, Chem. Eng. Progress 53(5) (predictor de Smith; recordado)', None),
]


def _run(cfg):
    return O.correr_operacion(cfg)


def _mapa(trabajadores, cfgs):
    if trabajadores > 1:
        with Pool(trabajadores) as p:
            return p.map(_run, cfgs, chunksize=2)
    return [_run(c) for c in cfgs]


def stats(v):
    v = np.asarray([x for x in v if x is not None], dtype=float)
    if len(v) == 0:
        return None
    n = len(v)
    ic = T95.get(n - 1, 1.96) * float(v.std(ddof=1)) / math.sqrt(n) if n > 1 else None
    return dict(media=float(v.mean()), ic95=ic, min=float(v.min()), max=float(v.max()), n=n)


def agrupa(filas, claves, campo):
    g = {}
    for f in filas:
        g.setdefault(tuple(f[k] for k in claves), []).append(f.get(campo))
    return {k: stats(v) for k, v in g.items()}


def fmt(s, dec=0, factor=1.0):
    if s is None:
        return '-'
    m = s['media'] * factor
    if s['ic95'] is None:
        return f'{m:.{dec}f}'
    return f'{m:.{dec}f} ± {s["ic95"] * factor:.{dec}f}'


def cruce(xs, ys, nivel):
    """Primer x (interpolando lineal en log x) donde la curva baja de `nivel`; None si nunca lo hace; xs[0] si ya empieza debajo."""
    for i, y in enumerate(ys):
        if y < nivel:
            if i == 0:
                return xs[0]
            x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
            f = (y0 - nivel) / (y0 - y1) if y0 != y1 else 0.0
            if x0 <= 0:
                return x0 + f * (x1 - x0)
            return float(math.exp(math.log(x0) + f * (math.log(x1) - math.log(x0))))
    return None


def _txt_cruce(v, unidad=''):
    return '> rango' if v is None else (f'{v:.0f}{unidad}' if v >= 10 else f'{v:.1f}{unidad}')


# ------------------------------------------------------------------------------------------------------------------
def calibracion(args):
    if args.recalibrar or not os.path.exists(CAL_ARCHIVO):
        print('calibrando con la dinamica de R1 ...', flush=True)
        tabla = O.calibrar_tabla(['marte', 'luna'], TERRENOS, [0, 1, 2], 50.0, args.trabajadores)
        extra = {}
        for te in ('llano', 'rocoso', 'dunas', 'rocas'):
            extra[f'marte|{te}|0'] = O.calibrar('marte', te, 0, 50.0, navegacion='completa', terreno_params=O.TERRENO_ROCAS if te == 'rocas' else None)
        os.makedirs(os.path.dirname(CAL_ARCHIVO), exist_ok=True)
        with open(CAL_ARCHIVO, 'w') as f:
            json.dump({'metodo': 'micro-corridas de 50 s de la dinamica de R1 (rodadura_en_motor: true, navegacion basico, ruta recta al este, v_max 0.5); '
                       'v_ratio = mediana de v / v_max desde t = 10 s; p_mov_w = potencia media con v > 0.05 m/s',
                       'tabla': tabla, 'extra_completa': extra}, f, indent=1, sort_keys=True)
            f.write('\n')
    O._CAL.clear()
    with open(CAL_ARCHIVO) as f:
        return json.load(f)


def estudio(args):
    rap = args.rapido
    sem = [0] if rap else [0, 1, 2]
    n_soles = 1 if rap else 3
    base = {'n_soles': n_soles}
    cal = calibracion(args)
    out = {}

    # ---- E1: tabla principal: cuerpo x terreno x modo
    cfgs = []
    for pl in ('marte', 'luna'):
        for te in (TERRENOS if not rap else ['llano', 'rocoso', 'rocas']):
            for mo in MODOS:
                for s in sem:
                    cfgs.append({**base, 'planeta': pl, 'terreno': te, 'modo': mo, 'semilla': s})
    for te in ('llano', 'rocoso'):                                         # extra: teleop idealizada en Marte (retardo de luz sin ventanas)
        for s in sem:
            cfgs.append({**base, 'planeta': 'marte', 'terreno': te, 'modo': 'teleop', 'enlace': 'retardo', 'semilla': s})
    print(f'E1: {len(cfgs)} corridas', flush=True)
    out['E1'] = _mapa(args.trabajadores, cfgs)

    # ---- E2: retardo vs distancia util (enlace de retardo constante)
    owls = [0.0, 1.3, 5.0, 15.0, 45.0, 150.0, 450.0, 900.0, 1440.0] if not rap else [0.0, 1.3, 15.0, 150.0, 900.0]
    cfgs = []
    for te in (('llano', 'rocoso') if not rap else ('rocoso',)):
        for mo in MODOS:
            for ow in owls:
                for s in sem:
                    c = {**base, 'planeta': 'marte', 'terreno': te, 'modo': mo, 'semilla': s}
                    c.update({'enlace': 'ideal'} if ow == 0 else {'enlace': 'retardo', 'owl_s': ow})
                    cfgs.append(c)
    print(f'E2: {len(cfgs)} corridas', flush=True)
    out['E2'] = _mapa(args.trabajadores, cfgs)

    # ---- E3: retardo x intervalo entre ventanas (ventanas sinteticas de 10 min)
    owls3 = [1.3, 60.0, 300.0, 900.0] if not rap else [1.3, 300.0]
    gaps = [900.0, 1800.0, 3600.0, 7200.0, 14400.0, 28800.0, 57600.0] if not rap else [900.0, 7200.0, 57600.0]
    cfgs = []
    for mo in MODOS:
        for ow in owls3:
            for g in gaps:
                for s in sem:
                    cfgs.append({**base, 'n_soles': max(n_soles - 1, 1) if not rap else 1, 'planeta': 'marte', 'terreno': 'rocoso', 'modo': mo,
                                 'enlace': 'ventanas', 'owl_s': ow, 'intervalo_ventanas_s': g, 'dur_ventana_s': 600.0, 'semilla': s})
    print(f'E3: {len(cfgs)} corridas', flush=True)
    out['E3'] = _mapa(args.trabajadores, cfgs)

    # ---- E4: gemelo digital: prediccion y conocimiento del terreno
    variantes = [('plan_sol', 'plan_sol', {}), ('plan_sol+MDE', 'plan_sol', {'p_dem': 0.6}),
                 ('gemelo sin MDE', 'gemelo', {'p_dem': 0.0}), ('gemelo', 'gemelo', {})]
    for p in ((0.3, 0.9) if not rap else ()):
        variantes.append((f'gemelo p_dem={p}', 'gemelo', {'p_dem': p}))
    cfgs = []
    for pl in ('marte', 'luna'):
        for te in ('llano', 'rocoso', 'rocas'):
            for nom, mo, extra in variantes:
                for s in sem:
                    cfgs.append({**base, 'planeta': pl, 'terreno': te, 'modo': mo, 'semilla': s, 'variante': nom, **extra})
    # y la ganancia de la prediccion en teleoperacion continua (enlace de retardo, sin ventanas)
    for ow in ((15.0, 150.0, 450.0) if not rap else (150.0,)):
        for nom, mo, extra in (('teleop', 'teleop', {}), ('teleop+alarmas', 'teleop', {'seguridad_local': True}),
                               ('gemelo sin alarmas', 'gemelo', {'seguridad_local': False}), ('gemelo', 'gemelo', {})):
            for s in sem:
                cfgs.append({**base, 'planeta': 'marte', 'terreno': 'rocoso', 'modo': mo, 'enlace': 'retardo', 'owl_s': ow, 'semilla': s,
                             'variante': f'{nom} OWL={ow:g}', **extra})
    print(f'E4: {len(cfgs)} corridas', flush=True)
    rows = _mapa(args.trabajadores, [{k: v for k, v in c.items() if k != 'variante'} for c in cfgs])
    for r, c in zip(rows, cfgs):
        r['variante'] = c['variante']
    out['E4'] = rows

    # ---- E5: alarmas locales (plan de sol con y sin) y metas con y sin
    cfgs = []
    for pl in ('marte', 'luna'):
        for te in ('llano', 'rocoso', 'rocas'):
            for mo, loc in (('plan_sol', True), ('plan_sol', False), ('metas', True), ('metas', False)):
                for s in sem:
                    cfgs.append({**base, 'planeta': pl, 'terreno': te, 'modo': mo, 'seguridad_local': loc, 'semilla': s})
    print(f'E5: {len(cfgs)} corridas', flush=True)
    out['E5'] = _mapa(args.trabajadores, cfgs)

    # ---- E6: sensibilidad a la tasa de zonas blandas y a la perdida de comandos (Luna, llano)
    cfgs = []
    for bk in ((0.0, 2.0, 6.0) if not rap else (0.0, 6.0)):
        for mo in MODOS:
            for s in sem:
                cfgs.append({**base, 'planeta': 'luna', 'terreno': 'llano', 'modo': mo, 'bache_por_km': bk, 'semilla': s, 'variante': f'bache={bk:g}/km'})
    for pp in ((0.2,) if not rap else ()):
        for mo in MODOS:
            for s in sem:
                cfgs.append({**base, 'planeta': 'luna', 'terreno': 'llano', 'modo': mo, 'p_perdida_cmd': pp, 'p_perdida_estado': pp, 'semilla': s,
                             'variante': f'perdida={pp:g}'})
    print(f'E6: {len(cfgs)} corridas', flush=True)
    rows = _mapa(args.trabajadores, [{k: v for k, v in c.items() if k != 'variante'} for c in cfgs])
    for r, c in zip(rows, cfgs):
        r['variante'] = c['variante']
    out['E6'] = rows
    # ---- E7: metas por pase en Marte: cuanto se compromete por adelantado, con y sin alarmas locales
    cfgs = []
    for te in ('llano', 'rocoso'):
        for ad in (100.0, 300.0, 1000.0, 3400.0):
            for loc in (False, True):
                for s in sem:
                    cfgs.append({**base, 'planeta': 'marte', 'terreno': te, 'modo': 'metas', 'adelanto_m': ad, 'seguridad_local': loc, 'semilla': s})
    print(f'E7: {len(cfgs)} corridas', flush=True)
    out['E7'] = _mapa(args.trabajadores, cfgs)
    return out, cal, dict(semillas=sem, n_soles=n_soles, owls=owls, owls3=owls3, gaps=gaps)


# ------------------------------------------------------------------------------------------------------------------
def analiza(out, meta):
    res = {}
    # umbral de teleoperacion (E2): retardo donde la distancia util cae por debajo de la mitad de la del enlace ideal
    e2 = out['E2']
    owls = meta['owls']
    umbrales = {}
    for te in sorted({r['terreno'] for r in e2}):
        for mo in MODOS:
            filas = [r for r in e2 if r['terreno'] == te and r['modo'] == mo]
            sems = sorted({r['semilla'] for r in filas})
            curva = {}
            for s in sems:
                curva[s] = [next(r['dist_util_m_sol'] for r in filas if r['semilla'] == s and (r['owl_s'] if r['enlace'] != 'ideal' else 0.0) == ow)
                            for ow in owls]
            media = np.mean([curva[s] for s in sems], axis=0)
            ref = media[0]
            c50 = cruce(owls, list(media), 0.5 * ref) if ref > 0 else None
            c50_s = [cruce(owls, curva[s], 0.5 * curva[s][0]) if curva[s][0] > 0 else None for s in sems]
            conocidos = [v for v in c50_s if v is not None]
            umbrales[f'{te}|{mo}'] = dict(owl_50pct_s=c50, por_semilla=c50_s, ref_ideal_m_sol=float(ref),
                                          rango=[min(conocidos), max(conocidos)] if conocidos else None,
                                          n_sin_cruce=sum(v is None for v in c50_s))
    res['umbral_E2'] = umbrales
    # teleoperacion frente a la autonomia (c) con el mismo retardo: retardo al que la teleop cae por debajo del 25 % de (c)
    rel = {}
    for te in sorted({r['terreno'] for r in e2}):
        fr = []
        for ow in owls:
            def m(mo):
                v = [r['dist_util_m_sol'] for r in e2 if r['terreno'] == te and r['modo'] == mo and (r['owl_s'] if r['enlace'] != 'ideal' else 0.0) == ow]
                return float(np.mean(v))
            fr.append(m('teleop') / m('plan_sol') if m('plan_sol') > 0 else 0.0)
        rel[te] = dict(cociente_teleop_sobre_plan_sol=fr, owl_25pct_s=cruce(owls, fr, 0.25), owl_10pct_s=cruce(owls, fr, 0.10))
    res['teleop_sobre_autonomia_E2'] = rel
    # intervalo entre ventanas al que cada modo cae por debajo del 50 % de su valor con enlace ideal, para cada retardo (E3)
    e3, gaps = out['E3'], meta['gaps']
    ref = {(te, mo): float(np.mean([r['dist_util_m_sol'] for r in e2 if r['terreno'] == te and r['modo'] == mo and r['enlace'] == 'ideal']))
           for te in {r['terreno'] for r in e2} for mo in MODOS}
    gp = {}
    for mo in MODOS:
        for ow in meta['owls3']:
            ys = [float(np.mean([r['dist_util_m_sol'] for r in e3 if r['modo'] == mo and r['owl_s'] == ow and r['intervalo_ventanas_s'] == g])) for g in gaps]
            r0 = ref.get(('rocoso', mo))
            gp[f'{mo}|{ow:g}'] = dict(intervalo_50pct_s=cruce(gaps, ys, 0.5 * r0) if r0 else None, ref_ideal_m_sol=r0)
    res['intervalo_E3'] = gp
    return res


def md_tabla(cab, filas):
    s = '| ' + ' | '.join(cab) + ' |\n|' + '|'.join('---' for _ in cab) + '|\n'
    for f in filas:
        s += '| ' + ' | '.join(str(x) for x in f) + ' |\n'
    return s


def escribe_md(out, cal, meta, an, ruta):
    L = []
    L.append('# Operación del rover con retardo de enlace (R8)\n')
    L.append('Simulación determinista; supuestos en `src/atp_rover/atp_rover/operacion.py` y en `docs/ROVER.md` «Resultados R8». '
             f'Semillas {meta["semillas"]}, {meta["n_soles"]} sol(es) por corrida, conducción de 2 h por sol, rover de 0.5 m/s. '
             'Cada celda es media ± intervalo de confianza del 95 % (t de Student) entre semillas (n pequeño: los intervalos son anchos).\n')
    c = cal['tabla']
    L.append('## Calibración con la dinámica de R1 (micro-corridas de 50 s, `rodadura_en_motor: true`, navegación `basico`)\n')
    L.append(md_tabla(['Cuerpo', 'Terreno', 'v efectiva / v_max (3 semillas)', 'Potencia de marcha (W)', 'Wh/m'],
                      [(pl, te, ' / '.join(f"{c[f'{pl}|{te}|{s}']['v_ratio']:.2f}" for s in (0, 1, 2)),
                        ' / '.join(f"{c[f'{pl}|{te}|{s}']['p_mov_w']:.0f}" for s in (0, 1, 2)),
                        ' / '.join(('-' if c[f'{pl}|{te}|{s}']['wh_por_m'] is None else f"{c[f'{pl}|{te}|{s}']['wh_por_m']:.3f}") for s in (0, 1, 2)))
                       for pl in ('marte', 'luna') for te in TERRENOS if f'{pl}|{te}|0' in c]))
    ex = cal.get('extra_completa') or {}
    if ex:
        L.append('Extra (navegación `completa` de R6, solo calibración, Marte, semilla 0; no se usa en los soles): ' + '; '.join(
            f"{k.split('|')[1]}: v/v_max {v['v_ratio']:.2f} (básico {c[k]['v_ratio']:.2f}), atascos {v['atascos']}" for k, v in ex.items()) + '\n')
    # E1
    e1 = out['E1']
    L.append('## E1. Distancia útil por sol según el modo (cuerpo × terreno)\n')
    L.append('Marte: enlace de ventanas reales del relé MRO (R7, Gale 4.6 S 137.4 E, 2026-10-07); Luna: retardo de luz de una vía 1.28 s (ida y vuelta 2.6 s) '
             'sin ventanas. (a) teleoperación, (b) metas por pase, (c) plan de sol con alarmas locales, (d) gemelo digital.\n')
    for pl in ('marte', 'luna'):
        tes = [t for t in TERRENOS if any(r['planeta'] == pl and r['terreno'] == t for r in e1)]
        filas = []
        for te in tes:
            fila = [te]
            for mo in MODOS:
                sub = [r for r in e1 if r['planeta'] == pl and r['terreno'] == te and r['modo'] == mo and r['enlace'] != ('retardo' if pl == 'marte' else '#')]
                fila.append(fmt(stats([r['dist_util_m_sol'] for r in sub]), 0))
            filas.append(fila)
        L.append(f'**{pl.capitalize()}**, distancia útil (m/sol)\n')
        L.append(md_tabla(['Terreno', '(a) teleop', '(b) metas', '(c) plan de sol', '(d) gemelo'], filas))
    ex = [r for r in e1 if r['planeta'] == 'marte' and r['enlace'] == 'retardo']
    if ex:
        L.append('Extra, Marte con teleoperación idealizada (retardo de luz sin ventanas, OWL real ' + f"{ex[0]['owl_s']:.0f} s): " + '; '.join(
            f"{te}: {fmt(stats([r['dist_util_m_sol'] for r in ex if r['terreno'] == te]), 0)} m/sol" for te in sorted({r['terreno'] for r in ex})) + '\n')
    # E1 detalle
    L.append('### Detalle por modo (media de los terrenos y semillas)\n')
    filas = []
    for pl in ('marte', 'luna'):
        for mo in MODOS:
            sub = [r for r in e1 if r['planeta'] == pl and r['modo'] == mo and r['enlace'] != ('retardo' if pl == 'marte' else '#')]
            if not sub:
                continue
            g = lambda k: stats([r[k] for r in sub])
            filas.append([pl, O.LETRA[mo] + ' ' + mo, fmt(g('dist_util_m_sol'), 0), fmt(g('t_ocioso_s_sol'), 0, 1 / 60), fmt(g('t_atascado_s_sol'), 0, 1 / 60),
                          fmt(g('cmd_enviados'), 0), fmt(g('cmd_caducados'), 0), fmt(g('atascos_sol'), 1), sum(r['vuelcos'] for r in sub),
                          fmt(g('energia_wh_sol'), 0), fmt(g('wh_por_m_util'), 3)])
    L.append(md_tabla(['Cuerpo', 'Modo', 'm/sol', 'ocioso (min/sol)', 'atascado (min/sol)', 'cmd enviados', 'cmd caducados', 'atascos/sol', 'vuelcos (corridas)',
                       'Wh/sol de conducción', 'Wh por m útil'], filas))
    # reaccion
    L.append('### Tiempo hasta reaccionar a un peligro\n')
    L.append('Desde que el rover entra en el peligro (o lo detecta) hasta que empieza la maniobra de salida. «Local» = alarma a bordo: obstáculo de pendiente visto a `sensado_m` '
             '(se detiene y rodea) o atasco detectado tras 3 s sin avanzar. «Vía Tierra» = la telemetría llega, el operador decide y la orden de recuperación llega al rover. '
             'Media y mediana de las medias por corrida.\n')
    filas = []
    for pl in ('marte', 'luna'):
        for mo in MODOS:
            sub = [r for r in e1 if r['planeta'] == pl and r['modo'] == mo and r['enlace'] != ('retardo' if pl == 'marte' else '#')]
            if not sub:
                continue

            def cel(k, nk=None):
                v = [r[k] for r in sub if r.get(k) is not None]
                return f'{np.mean(v):.1f} / {np.median(v):.1f}' if v else '-'
            filas.append([pl, O.LETRA[mo] + ' ' + mo, cel('reaccion_local_pendiente_s'), cel('reaccion_local_atasco_s'), cel('reaccion_tierra_s'),
                          sum(r['reaccion_tierra_n'] for r in sub), sum(r['peligro_sin_respuesta'] for r in sub)])
    L.append(md_tabla(['Cuerpo', 'Modo', 'Local, obstáculo visto (s)', 'Local, atasco (s)', 'Vía Tierra (s)', 'eventos vía Tierra', 'corridas con peligro sin respuesta al terminar'], filas))
    # E2
    L.append('## E2. Retardo contra distancia útil (Marte, enlace de retardo constante, sin ventanas)\n')
    e2, owls = out['E2'], meta['owls']
    for te in sorted({r['terreno'] for r in e2}):
        filas = []
        for ow in owls:
            fila = [f'{ow:g}']
            for mo in MODOS:
                sub = [r for r in e2 if r['terreno'] == te and r['modo'] == mo and (r['owl_s'] if r['enlace'] != 'ideal' else 0.0) == ow]
                fila.append(fmt(stats([r['dist_util_m_sol'] for r in sub]), 0))
            filas.append(fila)
        L.append(f'**{te}**, distancia útil (m/sol) contra retardo de una vía (s); 0 = enlace ideal\n')
        L.append(md_tabla(['OWL (s)', '(a) teleop', '(b) metas', '(c) plan de sol', '(d) gemelo'], filas))
    L.append('**Umbral** = retardo de una vía al que la distancia útil cae por debajo del 50 % de la del enlace ideal del mismo modo (interpolación lineal en log del retardo '
             'sobre la media; rango = mínimo y máximo de la misma cuenta por semilla).\n')
    filas = []
    for k, u in an['umbral_E2'].items():
        te, mo = k.split('|')
        rng = '-' if u['rango'] is None else f"{_txt_cruce(u['rango'][0])} a {_txt_cruce(u['rango'][1])}"
        filas.append([te, O.LETRA[mo] + ' ' + mo, f"{u['ref_ideal_m_sol']:.0f}", _txt_cruce(u['owl_50pct_s'], ' s'), rng, u['n_sin_cruce']])
    L.append(md_tabla(['Terreno', 'Modo', 'ideal (m/sol)', 'OWL al 50 %', 'rango entre semillas (s)', 'semillas sin cruce'], filas))
    L.append('**Teleoperación frente a la autonomía (c)**: cociente de distancia útil teleop / plan de sol con el mismo retardo; retardo al que baja del 25 % y del 10 %.\n')
    filas = [[te, ' / '.join(f'{q:.2f}' for q in v['cociente_teleop_sobre_plan_sol']), _txt_cruce(v['owl_25pct_s'], ' s'), _txt_cruce(v['owl_10pct_s'], ' s')]
             for te, v in an['teleop_sobre_autonomia_E2'].items()]
    L.append(md_tabla(['Terreno', 'cociente en cada retardo ' + str(owls), 'OWL al 25 %', 'OWL al 10 %'], filas))
    # E3
    L.append('## E3. Retardo × intervalo entre ventanas (ventanas sintéticas de 10 min, terreno rocoso)\n')
    e3 = out['E3']
    for mo in MODOS:
        filas = []
        for ow in meta['owls3']:
            fila = [f'{ow:g}']
            for g in meta['gaps']:
                sub = [r for r in e3 if r['modo'] == mo and r['owl_s'] == ow and r['intervalo_ventanas_s'] == g]
                fila.append(fmt(stats([r['dist_util_m_sol'] for r in sub]), 0))
            filas.append(fila)
        L.append(f'**({O.LETRA[mo]}) {mo}**, distancia útil (m/sol); columnas = intervalo entre ventanas (min)\n')
        L.append(md_tabla(['OWL (s)'] + [f'{g / 60:g}' for g in meta['gaps']], filas))
    L.append('**Intervalo entre ventanas al que cada modo cae por debajo del 50 % de su valor con enlace ideal (terreno rocoso)**, para cada retardo de una vía (s):\n')
    def _gap(mo, ow):
        v = an['intervalo_E3'][f'{mo}|{ow:g}']['intervalo_50pct_s']
        if v is None:
            return '> rango'
        return ('≤ ' if v <= meta['gaps'][0] + 1e-9 else '') + _txt_cruce(v / 60.0, ' min')
    filas = [[O.LETRA[mo] + ' ' + mo] + [_gap(mo, ow) for ow in meta['owls3']] for mo in MODOS]
    L.append(md_tabla(['Modo'] + [f'OWL {ow:g} s' for ow in meta['owls3']], filas))
    # E4
    L.append('## E4. Gemelo digital: predicción y conocimiento del terreno (MDE)\n')
    e4 = out['E4']
    nombres = []
    for r in e4:
        if r['variante'] not in nombres:
            nombres.append(r['variante'])
    filas = []
    for pl in ('marte', 'luna'):
        for te in ('llano', 'rocoso', 'rocas'):
            fila = [pl, te]
            for nom in [n for n in nombres if 'OWL' not in n]:
                sub = [r for r in e4 if r['variante'] == nom and r['planeta'] == pl and r['terreno'] == te]
                fila.append(fmt(stats([r['dist_util_m_sol'] for r in sub]), 0))
            filas.append(fila)
    L.append(md_tabla(['Cuerpo', 'Terreno'] + [n for n in nombres if 'OWL' not in n], filas))
    filas = []
    for nom in [n for n in nombres if 'OWL' in n]:
        sub = [r for r in e4 if r['variante'] == nom]
        filas.append([nom, fmt(stats([r['dist_util_m_sol'] for r in sub]), 0), fmt(stats([r['t_ocioso_s_sol'] for r in sub]), 0, 1 / 60)])
    if filas:
        L.append('Teleoperación continua con enlace de retardo (Marte, rocoso): «teleop» = mover y esperar; «gemelo» = predicción y envío adelantado.\n')
        L.append(md_tabla(['Variante', 'm/sol', 'ocioso (min/sol)'], filas))
    # E5
    L.append('## E5. Valor de las alarmas locales\n')
    e5 = out['E5']
    filas = []
    for pl in ('marte', 'luna'):
        for te in ('llano', 'rocoso', 'rocas'):
            fila = [pl, te]
            for mo in ('plan_sol', 'metas'):
                for loc in (True, False):
                    sub = [r for r in e5 if r['planeta'] == pl and r['terreno'] == te and r['modo'] == mo and r['seguridad_local'] == loc]
                    fila.append(fmt(stats([r['dist_util_m_sol'] for r in sub]), 0) + f" (vuelcos {sum(r['vuelcos'] for r in sub)})")
            filas.append(fila)
    L.append(md_tabla(['Cuerpo', 'Terreno', 'plan de sol, con alarmas', 'plan de sol, sin', 'metas, con alarmas', 'metas, sin'], filas))
    # E6
    L.append('## E6. Sensibilidad (Luna, llano): zonas blandas por km y pérdida de comandos/telemetría\n')
    e6 = out['E6']
    nombres = []
    for r in e6:
        if r['variante'] not in nombres:
            nombres.append(r['variante'])
    filas = [[nom] + [fmt(stats([r['dist_util_m_sol'] for r in e6 if r['variante'] == nom and r['modo'] == mo]), 0) for mo in MODOS] for nom in nombres]
    L.append(md_tabla(['Variante', '(a) teleop', '(b) metas', '(c) plan de sol', '(d) gemelo'], filas))
    L.append('## E7. Metas por pase en Marte: cuánto se compromete por adelantado y alarmas locales\n')
    e7 = out['E7']
    ads = sorted({r['adelanto_m'] for r in e7})
    filas = []
    for te in ('llano', 'rocoso'):
        fila = [te]
        for ad in ads:
            cel = []
            for loc in (False, True):
                cel.append(fmt(stats([r['dist_util_m_sol'] for r in e7 if r['terreno'] == te and r['adelanto_m'] == ad and r['seguridad_local'] == loc]), 0))
            fila.append(' / '.join(cel))
        filas.append(fila)
    L.append('Distancia útil (m/sol), «sin alarmas / con alarmas locales», según el adelanto (m) por delante de la última posición recibida.\n')
    L.append(md_tabla(['Terreno'] + [f'{a:g} m' for a in ads], filas))
    L.append('## Fuentes\n')
    for t, u in FUENTES:
        L.append(f'- {t}' + (f' ({u})' if u else ''))
    with open(ruta, 'w') as f:
        f.write('\n'.join(L) + '\n')


def main(argv=None):
    ap = argparse.ArgumentParser(allow_abbrev=False, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true')
    ap.add_argument('--trabajadores', type=int, default=4)
    ap.add_argument('--recalibrar', action='store_true')
    ap.add_argument('--salida', default=None)
    args = ap.parse_args(argv)
    salida = args.salida or (os.path.join(RAIZ, 'runs', 'rover_operacion_rapido') if args.rapido else os.path.join(RAIZ, 'docs', 'rover_resultados'))
    os.makedirs(salida, exist_ok=True)
    out, cal, meta = estudio(args)
    an = analiza(out, meta)
    CLAVES = ('planeta', 'terreno', 'semilla', 'modo', 'enlace', 'owl_s', 'intervalo_ventanas_s', 'seguridad_local', 'gemelo_prediccion', 'p_dem',
              'variante', 'adelanto_m', 'dist_util_m_sol', 'recorrido_m_sol', 't_ocioso_s_sol', 't_atascado_s_sol', 'cmd_enviados', 'cmd_perdidos', 'cmd_caducados',
              'atascos_sol', 'vuelcos', 'reaccion_local_s', 'reaccion_local_pendiente_s', 'reaccion_local_atasco_s', 'reaccion_tierra_s', 'reaccion_tierra_n', 'energia_wh_sol', 'wh_por_m_util', 'calibracion')
    compacto = {k: [{c: r.get(c) for c in CLAVES if c in r} for r in v] for k, v in out.items()}
    with open(os.path.join(salida, 'operacion.json'), 'w') as f:
        json.dump(dict(meta=meta, analisis=an, corridas=compacto, calibracion=cal), f, sort_keys=True, indent=0)
        f.write('\n')
    escribe_md(out, cal, meta, an, os.path.join(salida, 'operacion.md'))
    print('escrito', salida)
    return 0


if __name__ == '__main__':
    sys.exit(main())
