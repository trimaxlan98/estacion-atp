#!/usr/bin/env python3
"""Estudio R4 del rover: planeta x terreno x semilla, navegacion completa vs basica (docs/ROVER.md «Resultados R4»).

    python3 scripts/estudio_rover.py --rapido                 # 2 planetas x 3 terrenos x 1 semilla + barrido de pendiente corto
    python3 scripts/estudio_rover.py --trabajadores 5         # matriz completa (~1 h de pared con 5 trabajadores)

Escribe `docs/rover_resultados/estudio.json` y `estudio.md` (con `--rapido`, en `runs/rover_estudio_rapido/`, que no se versiona).
Cada corrida es independiente y determinista; los resultados se guardan en un cache jsonl (`--cache`) para reanudar.
Misma ruta de metas para todo: [[12, 0], [12, 8]] (20 m de recta). Parametros del suelo: ver planetas.py (Marte, Titan y
Ceres son SUPUESTOS de orden de magnitud).
"""
import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_rover'))

PLANETAS = ('luna', 'marte', 'titan', 'ceres', 'tierra')
TERRENOS = ('llano', 'rocoso', 'dunas', 'crater', 'pendiente')
SEMILLAS = (1, 2, 3)
MODOS = ('basico', 'completa')          # con --con-r4 se agrega 'completa_r4' (la navegacion de R4, para comparar)
RUTA = [[12, 0], [12, 8]]
RECTA_M = 20.0           # 12 m + 8 m
DURACION_S = 150.0
V_MAX = 0.5
ANGULOS = (5, 10, 15, 20, 25, 30)
MASA = 150.0             # kg, ParametrosRover.masa
P_BASE_W = 20.0          # ParametrosRover.p_base_w
DESL_SEGURO = 0.5        # criterio de «pendiente segura»: deslizamiento medio por debajo de esto (SUPUESTO)


def _clave(c):
    return json.dumps(c, sort_keys=True)


def _correr(c):
    from atp_rover.lazo import correr
    t0 = time.time()
    met, _ = correr(c)
    met['pared_s'] = round(time.time() - t0, 1)
    return {'cfg': c, 'met': met}


def configuraciones(rapido, modos=MODOS):
    pls = ('luna', 'marte') if rapido else PLANETAS
    trs = ('llano', 'rocoso', 'pendiente') if rapido else TERRENOS
    sem = SEMILLAS[:1] if rapido else SEMILLAS
    angs = (10, 20) if rapido else ANGULOS
    mat = [{'planeta': p, 'terreno': t, 'semilla': s, 'ruta': RUTA, 'duracion_s': DURACION_S, 'v_max': V_MAX,
            'navegacion': m, 'estudio': 'matriz'}
           for p in pls for t in trs for s in sem for m in modos]
    pen = [{'planeta': p, 'terreno': 'pendiente', 'semilla': 1, 'terreno_params': {'angulo_deg': a},
            'ruta': [[12, 0]], 'duracion_s': DURACION_S, 'v_max': V_MAX, 'navegacion': 'basico', 'estudio': 'pendiente'}
           for p in pls for a in angs]
    return mat + pen


def ejecutar(cfgs, cache, trab):
    hechos = {}
    if cache and os.path.exists(cache):
        with open(cache) as f:
            for linea in f:
                r = json.loads(linea)
                hechos[_clave(r['cfg'])] = r
    falta = [c for c in cfgs if _clave(c) not in hechos]
    print(f'{len(cfgs)} corridas, {len(cfgs) - len(falta)} en cache, {len(falta)} por correr con {trab} trabajadores', flush=True)
    if falta:
        t0 = time.time()
        with ProcessPoolExecutor(max_workers=trab) as ex:
            for i, r in enumerate(ex.map(_correr, falta), 1):
                hechos[_clave(r['cfg'])] = r
                if cache:
                    with open(cache, 'a') as f:
                        f.write(json.dumps(r) + '\n')
                if i % 5 == 0 or i == len(falta):
                    print(f'  {i}/{len(falta)}  {time.time() - t0:.0f} s', flush=True)
    return [hechos[_clave(c)] for c in cfgs]


def _media(xs):
    return float(np.mean(xs)) if len(xs) else float('nan')


def agregar(filas):
    """Agrupa la matriz por (planeta, terreno, modo) promediando semillas."""
    g = {}
    for r in filas:
        if r['cfg']['estudio'] != 'matriz':
            continue
        c, m = r['cfg'], r['met']
        g.setdefault((c['planeta'], c['terreno'], c['navegacion']), []).append(m)
    out = []
    for (p, t, modo), ms in g.items():
        d = np.array([m['distancia_m'] for m in ms])
        e = np.array([m['energia_wh'] for m in ms])
        tt = np.array([m['tiempo_s'] for m in ms])
        ok = d > 1.0
        wh_m = e[ok] / d[ok] if ok.any() else np.array([])
        wh_tr = (e[ok] - P_BASE_W * tt[ok] / 3600.0) / d[ok] if ok.any() else np.array([])
        out.append({
            'planeta': p, 'terreno': t, 'navegacion': modo, 'n': len(ms),
            'meta_alcanzada': sum(m['meta_alcanzada'] for m in ms),
            'tiempo_s': _media(tt), 'distancia_m': _media(d), 'dist_sobre_recta': _media(d) / RECTA_M,
            'wh_por_m': _media(wh_m), 'wh_por_m_traccion': _media(wh_tr),
            'cdt': _media(wh_m * 3600.0 / (MASA * _g(p))) if len(wh_m) else float('nan'),
            'deslizamiento_medio': _media([m['deslizamiento_medio'] for m in ms]),
            'pendiente_max_deg': float(max(m['pendiente_max_deg'] for m in ms)),
            'atascos': sum(m['atascos'] for m in ms), 'vuelcos': sum(m['vuelcos'] for m in ms),
            'sin_ruta': sum(1 for m in ms if m.get('fallo') == 'sin_ruta'),
            'abortadas_seguridad': sum(1 for m in ms if m.get('fallo') == 'seguridad')})
    return out


def _g(p):
    from atp_rover.planetas import PLANETAS as P
    return P[p]['g']


def barrido_pendiente(filas):
    """Por planeta: lista de (angulo, ok, meta, atascos, vuelco, desl) y el maximo contiguo «seguro» desde el menor angulo."""
    por = {}
    for r in filas:
        c, m = r['cfg'], r['met']
        if c['estudio'] != 'pendiente':
            continue
        a = c['terreno_params']['angulo_deg']
        ok = bool(m['meta_alcanzada'] and m['atascos'] == 0 and m['vuelcos'] == 0 and m['deslizamiento_medio'] <= DESL_SEGURO)
        por.setdefault(c['planeta'], []).append({'angulo_deg': a, 'seguro': ok, 'meta': m['meta_alcanzada'],
                                                'atascos': m['atascos'], 'vuelcos': m['vuelcos'],
                                                'deslizamiento_medio': m['deslizamiento_medio'],
                                                'tiempo_s': m['tiempo_s'], 'distancia_m': m['distancia_m'],
                                                'phi_deg': _phi(c['planeta'])})
    res = {}
    for p, ls in por.items():
        ls.sort(key=lambda x: x['angulo_deg'])
        mx = None
        for x in ls:
            if not x['seguro']:
                break
            mx = x['angulo_deg']
        res[p] = {'maximo_seguro_deg': mx, 'puntos': ls}
    return res


def _phi(p):
    from atp_rover.planetas import PLANETAS as P
    return P[p]['suelo']['phi_deg']


def resumen_preguntas(agg, pen):
    """Respuestas numericas a las tres preguntas (promedios sobre las celdas con datos)."""
    q1 = {}
    for p in PLANETAS:
        for modo in MODOS:
            v = [a for a in agg if a['planeta'] == p and a['navegacion'] == modo and a['terreno'] == 'llano' and a['n']]
            if v:
                q1[f'{p}/{modo}'] = {'wh_por_m': v[0]['wh_por_m'], 'wh_por_m_traccion': v[0]['wh_por_m_traccion'], 'cdt': v[0]['cdt']}
    q2 = {}
    for modo in [m for m in ('basico', 'completa', 'completa_r4') if any(a['navegacion'] == m for a in agg)]:
        v = [a for a in agg if a['navegacion'] == modo]
        w = [a for a in v if a['terreno'] != 'crater']
        q2[modo] = {'corridas': sum(a['n'] for a in v), 'metas': sum(a['meta_alcanzada'] for a in v),
                    'atascos': sum(a['atascos'] for a in v), 'vuelcos': sum(a['vuelcos'] for a in v),
                    'sin_ruta': sum(a['sin_ruta'] for a in v), 'abortadas_seguridad': sum(a['abortadas_seguridad'] for a in v),
                    'corridas_sin_crater': sum(a['n'] for a in w), 'metas_sin_crater': sum(a['meta_alcanzada'] for a in w),
                    'atascos_sin_crater': sum(a['atascos'] for a in w)}
    return {'costo_transporte_llano': q1, 'basico_vs_completa': q2,
            'pendiente_maxima_segura_deg': {p: pen[p]['maximo_seguro_deg'] for p in pen}}


def _f(x, d=2):
    return '-' if x != x else f'{x:.{d}f}'


def markdown(agg, pen, preg, rapido):
    L = ['# Estudio del rover R4: planeta x terreno, navegacion completa vs basica', '']
    L += ['Generado por `scripts/estudio_rover.py' + (' --rapido' if rapido else '') + '`. Ruta [[12,0],[12,8]] (recta 20 m), '
          f'v_max {V_MAX} m/s, tope {DURACION_S:.0f} s simulados. En simulacion, con parametros de suelo supuestos '
          '(Marte, Titan y Ceres de orden de magnitud). Cada celda promedia las semillas; pendiente max = maximo; '
          'atascos y vuelcos = suma.', '']
    L += ['## Matriz', '',
          '| Planeta | Terreno | Nav. | Meta | t (s) | Dist/recta | Wh/m | Wh/m trac. | Desliz. | Pend. max (°) | Atascos | Vuelcos | Sin ruta | Abort. seg. |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    orden = {p: i for i, p in enumerate(PLANETAS)}
    ordt = {t: i for i, t in enumerate(TERRENOS)}
    for a in sorted(agg, key=lambda a: (orden[a['planeta']], ordt[a['terreno']], a['navegacion'])):
        L.append(f"| {a['planeta']} | {a['terreno']} | {a['navegacion']} | {a['meta_alcanzada']}/{a['n']} | {_f(a['tiempo_s'], 1)} | "
                 f"{_f(a['dist_sobre_recta'])} | {_f(a['wh_por_m'], 3)} | {_f(a['wh_por_m_traccion'], 3)} | {_f(a['deslizamiento_medio'])} | "
                 f"{_f(a['pendiente_max_deg'], 1)} | {a['atascos']} | {a['vuelcos']} | {a['sin_ruta']} | {a['abortadas_seguridad']} |")
    L += ['', '`Wh/m trac.` descuenta el consumo fijo de 20 W (computador y radio) multiplicado por el tiempo: es la parte que depende del suelo y la gravedad.', '']
    L += ['## Costo de transporte en llano (navegacion basica y completa)', '',
          '| Planeta | g (m/s²) | Wh/m basica | Wh/m completa | Wh/m trac. basica | CdT basica |', '|---|---|---|---|---|---|']
    for p in PLANETAS:
        b, c = preg['costo_transporte_llano'].get(f'{p}/basico'), preg['costo_transporte_llano'].get(f'{p}/completa')
        if b or c:
            L.append(f"| {p} | {_g(p):.2f} | {_f(b['wh_por_m'], 3) if b else '-'} | {_f(c['wh_por_m'], 3) if c else '-'} | "
                     f"{_f(b['wh_por_m_traccion'], 3) if b else '-'} | {_f(b['cdt'], 2) if b else '-'} |")
    L += ['', 'CdT = energia / (m g d), adimensional (m = 150 kg, g del cuerpo).', '']
    L += ['## Basica contra completa', '', '| Navegacion | Corridas | Metas | Atascos | Vuelcos | Sin ruta | Abortadas | Metas sin crater | Atascos sin crater |',
          '|---|---|---|---|---|---|---|---|---|']
    for m, v in preg['basico_vs_completa'].items():
        L.append(f"| {m} | {v['corridas']} | {v['metas']} | {v['atascos']} | {v['vuelcos']} | {v['sin_ruta']} | {v['abortadas_seguridad']} | "
                 f"{v['metas_sin_crater']}/{v['corridas_sin_crater']} | {v['atascos_sin_crater']} |")
    L += ['', f'## Barrido de pendiente (navegacion basica, ruta recta cuesta arriba 12 m; «seguro» = meta, 0 atascos, 0 vuelcos, deslizamiento medio <= {DESL_SEGURO})', '']
    angs = sorted({x['angulo_deg'] for v in pen.values() for x in v['puntos']})
    L += ['| Planeta | phi suelo (°) | ' + ' | '.join(f'{a}°' for a in angs) + ' | Max. seguro (°) |', '|---|---|' + '---|' * (len(angs) + 1)]
    for p in PLANETAS:
        if p not in pen:
            continue
        cel = {x['angulo_deg']: x for x in pen[p]['puntos']}
        fila = []
        for a in angs:
            x = cel.get(a)
            fila.append('-' if x is None else (f"si ({x['deslizamiento_medio']:.2f})" if x['seguro'] else
                        ('vuelco' if x['vuelcos'] else 'atasco' if x['atascos'] else f"no ({x['deslizamiento_medio']:.2f})")))
        mx = pen[p]['maximo_seguro_deg']
        L.append(f"| {p} | {_phi(p):.0f} | " + ' | '.join(fila) + f" | {'<' + str(angs[0]) if mx is None else mx} |")
    L.append('')
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true', help='subconjunto (2 planetas, 3 terrenos, 1 semilla)')
    ap.add_argument('--trabajadores', type=int, default=5)
    ap.add_argument('--salida', help='carpeta de salida (por defecto docs/rover_resultados o runs/rover_estudio_rapido)')
    ap.add_argument('--cache', help='jsonl de corridas ya hechas (reanuda)')
    ap.add_argument('--sembrar', help='jsonl de otro estudio (p. ej. docs/rover_resultados/corridas.jsonl de R4): se copian al cache las corridas '
                    '`basico` y del barrido de pendiente cuya configuracion sea identica (esa ruta y esa dinamica no cambiaron; ver docs/ROVER.md)')
    ap.add_argument('--con-r4', action='store_true', help="agrega el modo 'completa_r4' (navegacion de R4) para comparar")
    ap.add_argument('--rodadura-en-motor', action='store_true', help='fisica corregida de R6 (el motor vence la resistencia por compactacion); '
                    'repite el estudio de R4 sin el defecto de energia que halló R5. Usa --salida distinta de la historica')
    a = ap.parse_args(argv)
    salida = a.salida or os.path.join(RAIZ, 'runs', 'rover_estudio_rapido') if a.rapido else (a.salida or os.path.join(RAIZ, 'docs', 'rover_resultados'))
    os.makedirs(salida, exist_ok=True)
    cache = a.cache or os.path.join(salida, 'corridas.jsonl')
    modos = MODOS + (('completa_r4',) if a.con_r4 else ())
    cfgs = configuraciones(a.rapido, modos)
    if a.rodadura_en_motor:
        for c in cfgs:
            c['rover'] = {'rodadura_en_motor': True}
    if a.sembrar and not a.rodadura_en_motor and not (os.path.exists(cache) and os.path.getsize(cache) > 0):
        claves = {_clave(c) for c in cfgs if c['navegacion'] == 'basico'}
        n = 0
        with open(a.sembrar) as f, open(cache, 'a') as g:
            for linea in f:
                if _clave(json.loads(linea)['cfg']) in claves:
                    g.write(linea)
                    n += 1
        print(f'{n} corridas sembradas desde {a.sembrar}', flush=True)
    filas = ejecutar(cfgs, cache, a.trabajadores)
    agg = agregar(filas)
    pen = barrido_pendiente(filas)
    preg = resumen_preguntas(agg, pen)
    with open(os.path.join(salida, 'estudio.json'), 'w') as f:
        json.dump({'rapido': a.rapido, 'ruta': RUTA, 'duracion_s': DURACION_S, 'v_max': V_MAX, 'criterio_pendiente_segura': {
            'deslizamiento_medio_max': DESL_SEGURO, 'meta': True, 'atascos': 0, 'vuelcos': 0},
            'matriz': agg, 'pendiente': pen, 'preguntas': preg}, f, indent=1, sort_keys=True)
        f.write('\n')
    md = markdown(agg, pen, preg, a.rapido)
    with open(os.path.join(salida, 'estudio.md'), 'w') as f:
        f.write(md)
    print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
