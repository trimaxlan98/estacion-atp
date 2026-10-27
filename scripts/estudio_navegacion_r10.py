#!/usr/bin/env python3
"""Estudio R10: validacion fuera de muestra de la navegacion del rover (docs/ROVER.md «Resultados R10»).

El PRE-REGISTRO (conjuntos, metricas, estadistica y criterios C1-C7) esta en el docstring de
`src/atp_rover/atp_rover/validacion_nav.py` y se fijo antes de correr ningun caso nuevo.

    python3 scripts/estudio_navegacion_r10.py --rapido                              # 6 casos x 2 navegaciones (~3 min)
    python3 scripts/estudio_navegacion_r10.py --conjunto validacion --trabajadores 4 # 150 casos x basico, completa
    python3 scripts/estudio_navegacion_r10.py --conjunto ajuste --ablaciones         # 60 casos de R4/R6: ablaciones de completa
    python3 scripts/estudio_navegacion_r10.py --conjunto tercero --modos basico,completa,completa_r10   # UNA vez, al final

Resultados en docs/rover_resultados_r10/ (`estudio_<conjunto>.json|md`, `corridas.jsonl` = cache por configuracion para reanudar;
con --rapido, en runs/rover_r10_rapido/, que no se versiona). Cada corrida es independiente y determinista.
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
sys.path.insert(0, AQUI)

from atp_rover import validacion_nav as vn   # noqa: E402

# Ablaciones de `completa` (R6) en el conjunto de AJUSTE: cada una apaga UN componente (sobreescribe `nav` / `traccion` de la escena).
VARIANTES = {
    'sin_graduado': {'nav': {'graduado': False}},
    'sin_ff_pi': {'nav': {'ff_desliz': False, 'ki_vel': 0.0, 'kp_vel': 0.0, 'boost_max': 0.0}},
    'sin_guarda_encabritado': {'nav': {'cab_exceso_deg': 1e9}},
    'sin_vaiven': {'nav': {'vaiven': False}},
    'sin_limite_aceleracion': {'nav': {'acc_max': 0.0}},
    'sin_replan': {'nav': {'replan_s': 0.0}},
    'umbral_trac_05': {'traccion': {'umbral_desliz': 0.5}},
    'frenar_en_pendiente': {'nav': {'frenar_pend': True}},
    # Anadida DESPUES de ver la validacion (24 de 30 abortos por `seguridad` eran la alarma `pendiente` en `rocas`); se prueba solo en el ajuste.
    'sin_alarma_pendiente': {'seguridad': {'pendiente_deg': 89.0}},
}

# Pasos intermedios hacia `completa_r10` (= huella + sin limite de aceleracion + sin vaiven), elegidos tras ver las ablaciones de ajuste2 y comparados en TODO
# el ajuste, nunca en la validacion. Son `completa` con esos `nav` explicitos; `completa_r10` es la navegacion versionada.
CANDIDATOS = {
    'r10_solo_huella': {'nav': {'pendiente_huella_m': 0.6}},
    'r10_sin_acc': {'nav': {'pendiente_huella_m': 0.6, 'acc_max': 0.0}},
}


def _clave(c):
    return json.dumps(c, sort_keys=True)


def _correr(c):
    from atp_rover.lazo import correr
    t0 = time.time()
    met, filas = correr(c)
    met['pared_s'] = round(time.time() - t0, 1)
    return {'cfg': c, 'met': met, 'val': vn.resumen_corrida(met, filas, c['ruta'])}


def casos_ajuste():
    """Los 60 casos de R4/R6 sin el crater (5 cuerpos x llano, rocoso, dunas, pendiente de 15 grados x semillas 1-3, ruta [[12,0],[12,8]])."""
    import estudio_rover as er
    out = []
    for c in er.configuraciones(False, ('basico',)):
        if c['estudio'] == 'matriz' and c['terreno'] != 'crater':
            c = {k: v for k, v in c.items() if k != 'navegacion'}
            c['rover'] = {'rodadura_en_motor': True}      # como en R4b (hoy es el defecto)
            c['regimen'] = c['terreno']
            c['ruta_nombre'] = 'fija_R4'
            out.append(c)
    return out


def configuraciones(conjunto, modos, ablaciones, candidatos=False):
    if conjunto == 'rapido':
        base = vn.casos_rapidos(101)
    elif conjunto == 'validacion':
        base = vn.casos(101)
    elif conjunto == 'tercero':
        base = vn.casos(201)
    elif conjunto == 'ajuste':
        base = casos_ajuste()
    elif conjunto == 'ajuste2':          # ajuste AMPLIADO: mismo generador que la validacion, semillas 301-330, 2 casos por celda (50), solo para diagnosticar y ajustar
        base = vn.casos(301, 2)
    else:
        raise SystemExit(f'conjunto desconocido: {conjunto}')
    cfgs = [{**b, 'navegacion': m} for b in base for m in modos]
    if ablaciones:
        for b in base:
            for nombre, ov in VARIANTES.items():
                cfgs.append({**b, 'navegacion': 'completa', 'variante': nombre, **ov})
    if candidatos:
        for b in base:
            for nombre, ov in CANDIDATOS.items():
                cfgs.append({**b, 'navegacion': 'completa', 'variante': nombre, **ov})
    return cfgs


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


# ---------------------------------------------------------------------------------------------------------------------------
def analizar(filas, modos, ablaciones, referencia='basico', candidatos=False):
    """Comparaciones pareadas de cada modo (y variante) contra `referencia`, criterios y desgloses por regimen."""
    res = {'modos': {}, 'variantes': {}, 'conteos': {}}
    for r in filas:
        k = r['cfg']['navegacion'] + ('/' + r['cfg']['variante'] if r['cfg'].get('variante') else '')
        c = res['conteos'].setdefault(k, {'n': 0, 'metas': 0, 'vuelcos': 0, 'atascos_nativos': 0, 'atasco_real_episodios': 0,
                                          'corridas_con_atasco_real': 0, 'atasco_terminal': 0, 'abortadas_seguridad': 0, 'inalcanzable': 0})
        c['n'] += 1
        c['metas'] += int(r['met']['meta_alcanzada'])
        c['vuelcos'] += int(r['met']['vuelcos'] > 0)
        c['atascos_nativos'] += r['met']['atascos']
        c['atasco_real_episodios'] += r['val']['atasco_real_episodios']
        c['corridas_con_atasco_real'] += int(r['val']['atasco_real_episodios'] > 0)
        c['atasco_terminal'] += int(r['val']['atasco_terminal'])
        c['abortadas_seguridad'] += int(r['met'].get('fallo') == 'seguridad')
        c['inalcanzable'] += int(r['met'].get('fallo') == 'inalcanzable')
    for m in modos:
        if m == referencia:
            continue
        pares = vn.emparejar(filas, m, referencia)
        comp = vn.comparar_modos(pares)
        if not pares:
            continue
        ter = vn.por_grupo(pares, lambda c: c['regimen'])
        cue = vn.por_grupo(pares, lambda c: c['planeta'])
        rut = vn.por_grupo(pares, lambda c: c['ruta_nombre'])
        res['modos'][m] = {'comparacion': comp, 'por_terreno': ter, 'por_cuerpo': cue, 'por_ruta': rut,
                           'criterios': vn.evaluar_criterios(comp, ter, cue)}
        if m != 'completa' and 'completa' in modos:        # informativo: la version nueva contra la completa de R6
            pc = vn.emparejar(filas, m, 'completa')
            if pc:
                res['modos'][m]['contra_completa'] = vn.comparar_modos(pc)
    if ablaciones:
        for nombre in VARIANTES:
            pares = vn.emparejar(filas, 'completa', referencia, nombre, None)
            if pares:
                res['variantes'][nombre] = vn.comparar_modos(pares)
        pares = vn.emparejar(filas, 'completa', referencia)
        if pares:
            res['variantes']['(completa R6 completa)'] = vn.comparar_modos(pares)
    if candidatos:
        for nombre in CANDIDATOS:
            pares = vn.emparejar(filas, 'completa', referencia, nombre, None)
            if pares:
                res['variantes'][nombre] = vn.comparar_modos(pares)
    return res


def _pc(x):
    return '-' if x != x else f'{100 * x:.1f}'


def markdown(conjunto, res, modos, rapido, n_casos):
    L = [f'# Estudio R10 · {conjunto}', '',
         f'Generado por `scripts/estudio_navegacion_r10.py` ({"--rapido" if rapido else "--conjunto " + conjunto}). {n_casos} casos; '
         'pares = mismo caso con las dos navegaciones; IC 95 % por bootstrap percentil (10 000 remuestreos, semilla '
         f'{vn.SEMILLA_BOOTSTRAP}). En simulacion, con los suelos supuestos de R4 y la fisica corregida. Criterios pre-registrados en '
         '`validacion_nav.py`.', '']
    L += ['## Conteos por navegacion', '',
          '| Navegacion | Corridas | Metas | Vuelcos | Atascos nativos | Corridas con atasco real | Episodios de atasco real | Atasco terminal | Abort. seg. | Inalcanzable |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for k, c in res['conteos'].items():
        L.append(f"| {k} | {c['n']} | {c['metas']} | {c['vuelcos']} | {c['atascos_nativos']} | {c['corridas_con_atasco_real']} | "
                 f"{c['atasco_real_episodios']} | {c['atasco_terminal']} | {c['abortadas_seguridad']} | {c['inalcanzable']} |")
    L += ['', '`Atascos nativos` = contador del rover (episodios de 5 s con |v| < 20 % de la orden: cuenta cada pausa de un vaiven). '
          '`Atasco real` = 30 s sin que el progreso maximo a lo largo de la ruta suba 0.25 m.', '']
    for m, d in res['modos'].items():
        comp = d['comparacion']
        ll = comp['llegada']
        L += [f'## {m} contra basico (n = {comp["n"]} pares)', '',
              f"Llegada: {_pc(ll['tasa_a'])} % contra {_pc(ll['tasa_b'])} %; diferencia {_pc(ll['dif'])} puntos "
              f"[{_pc(ll['ic_lo'])}, {_pc(ll['ic_hi'])}]; solo {m} llega en {ll['solo_a']}, solo basico en {ll['solo_b']}; "
              f"p exacta del signo {ll['p_signo']:.3g}; d_z {ll['d_z']:.2f}.", '',
              '| Criterio | Qué | Valor | Cumple |', '|---|---|---|---|']
        for c in d['criterios']:
            L.append(f"| {c['id']} | {c['texto']} | {c['valor']} | {'SI' if c['cumple'] else 'NO'} |")
        for titulo, clave in (('terreno', 'por_terreno'), ('cuerpo', 'por_cuerpo'), ('ruta', 'por_ruta')):
            L += ['', f'Llegada por {titulo} ({m} - basico):', '', f'| {titulo.capitalize()} | n | {m} | basico | dif. (puntos) | IC95 | solo {m} / solo basico |', '|---|---|---|---|---|---|---|']
            for k, v in d[clave].items():
                L.append(f"| {k} | {v['n']} | {_pc(v['tasa_a'])} % | {_pc(v['tasa_b'])} % | {_pc(v['dif'])} | [{_pc(v['ic_lo'])}, {_pc(v['ic_hi'])}] | {v['solo_a']} / {v['solo_b']} |")
        if 'contra_completa' in d:
            cc = d['contra_completa']
            ll = cc['llegada']
            en = cc['energia_por_m_util']
            L += ['', f"Informativo, {m} contra `completa` (R6), {cc['n']} pares: llegada {_pc(ll['tasa_a'])} % contra {_pc(ll['tasa_b'])} %, diferencia {_pc(ll['dif'])} puntos "
                  f"[{_pc(ll['ic_lo'])}, {_pc(ll['ic_hi'])}] (solo {m} {ll['solo_a']}, solo completa {ll['solo_b']}, p exacta {ll['p_signo']:.3g}); vuelcos {cc['vuelcos_a']} contra {cc['vuelcos_b']}; "
                  f"energia por metro util x{en['razon']:.3f} [{en['ic_lo']:.3f}, {en['ic_hi']:.3f}] (n={en['n']})."]
        L += ['']
    if res['variantes']:
        L += ['## Ablaciones de `completa` (cada fila apaga un componente) contra basico', '',
              '| Variante | n | Llegada | dif. vs basico (puntos) [IC95] | Vuelcos | Corridas con atasco real | Atascos nativos |', '|---|---|---|---|---|---|---|']
        for k, v in res['variantes'].items():
            ll = v['llegada']
            L.append(f"| {k} | {v['n']} | {_pc(ll['tasa_a'])} % | {_pc(ll['dif'])} [{_pc(ll['ic_lo'])}, {_pc(ll['ic_hi'])}] | "
                     f"{v['vuelcos_a']} | {int(round(v['atasco_real']['tasa_a'] * v['n']))} | {v['atascos_nativos_a']} |")
        L += ['']
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true', help='6 casos x basico y completa (para validar_todo.sh)')
    ap.add_argument('--conjunto', default='validacion', choices=('validacion', 'ajuste', 'ajuste2', 'tercero'))
    ap.add_argument('--modos', default='basico,completa', help='navegaciones separadas por coma (basico es la referencia)')
    ap.add_argument('--ablaciones', action='store_true', help='agrega las variantes de VARIANTES (solo tiene sentido en el conjunto de ajuste)')
    ap.add_argument('--candidatos', action='store_true', help='agrega los CANDIDATOS sobre completa_r10 (solo en conjuntos de ajuste)')
    ap.add_argument('--trabajadores', type=int, default=4)
    ap.add_argument('--salida')
    ap.add_argument('--cache')
    a = ap.parse_args(argv)
    conjunto = 'rapido' if a.rapido else a.conjunto
    salida = a.salida or (os.path.join(RAIZ, 'runs', 'rover_r10_rapido') if a.rapido else os.path.join(RAIZ, 'docs', 'rover_resultados_r10'))
    os.makedirs(salida, exist_ok=True)
    cache = a.cache or os.path.join(salida, 'corridas.jsonl')
    modos = [m for m in a.modos.split(',') if m]
    if conjunto in ('ajuste', 'ajuste2') and a.ablaciones:
        modos = ['basico', 'completa'] if not a.modos or a.modos == 'basico,completa' else modos
    cfgs = configuraciones(conjunto, modos, a.ablaciones, a.candidatos)
    filas = ejecutar(cfgs, cache, a.trabajadores)
    res = analizar(filas, modos, a.ablaciones, candidatos=a.candidatos)
    n_casos = len({vn.clave_caso(f['cfg']) for f in filas})
    nombre = 'rapido' if a.rapido else conjunto
    with open(os.path.join(salida, f'estudio_{nombre}.json'), 'w') as f:
        json.dump({'conjunto': conjunto, 'modos': modos, 'n_casos': n_casos, 'bootstrap': {'semilla': vn.SEMILLA_BOOTSTRAP, 'n': vn.N_BOOT},
                   'resultados': res}, f, indent=1, sort_keys=True, default=float)
        f.write('\n')
    md = markdown(conjunto, res, modos, a.rapido, n_casos)
    with open(os.path.join(salida, f'estudio_{nombre}.md'), 'w') as f:
        f.write(md + '\n')
    print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
