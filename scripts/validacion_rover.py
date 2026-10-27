#!/usr/bin/env python3
"""Validacion cientifica del modelo del rover (R5): convergencia numerica, sensibilidad global, contraste con la
literatura e invariancias (docs/ROVER.md «Resultados R5 (validacion)»,.

    python3 scripts/validacion_rover.py --rapido                  # unos minutos; escribe en runs/rover_validacion_rapido/
    python3 scripts/validacion_rover.py --trabajadores 5          # completo (~1 h de pared); escribe docs/rover_resultados/validacion.{json,md}

NO modifica el modelo (rover.py, terreno.py, planetas.py): lo usa tal cual. Para variar el paso se reasigna la constante
de modulo `atp_rover.rover.DT` dentro de cada corrida (y se restaura); las metricas se calculan aqui (no con
`Rover.metricas`, cuyo muestreo esta fijado a 10 pasos). Cada corrida es independiente y determinista; los resultados se
guardan en un cache jsonl (`--cache`) para reanudar. Todo es SIMULACION con los supuestos de planetas.py.

Muestreo de la sensibilidad: secuencia de Sobol con mezcla (scipy.stats.qmc) con semilla fija -> mismos numeros siempre.
"""
import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_rover'))

SEMILLA = 20260507          # semilla del muestreo de Sobol y del bootstrap
P_BASE_W = 20.0             # ParametrosRover.p_base_w
V_CMD = 0.5                 # m/s mandado en todos los estudios (el v_max de R4)
DEF_SIM = dict(planeta='luna', suelo=None, g=None, terreno='llano', semilla=0, terreno_params={}, extension=40.0, res=0.1,
               dt=0.01, T=6.0, settle=0.6, v=V_CMD, w=0.0, rover=None, ventana_s=3.0, traza_cada=0)

# Incertidumbre del suelo por planeta (MIS ELECCIONES, no datos): factor multiplicativo (log-uniforme) o semiancho (uniforme).
# clase: medido (Tierra, arena de Wong), estimado (Luna, Marte: orden de magnitud con respaldo parcial), supuesto (Titan, Ceres).
INCERT = {
    'tierra': dict(clase='medido', phi=3.0, c=2.0, K=2.0, n=0.05, kc=1.3, kphi=1.3),
    'luna': dict(clase='estimado', phi=5.0, c=3.0, K=2.0, n=0.10, kc=2.0, kphi=1.5),
    'marte': dict(clase='estimado', phi=5.0, c=3.0, K=2.0, n=0.10, kc=2.0, kphi=2.0),
    'titan': dict(clase='supuesto', phi=8.0, c=5.0, K=3.0, n=0.15, kc=3.0, kphi=3.0),
    'ceres': dict(clase='supuesto', phi=8.0, c=5.0, K=3.0, n=0.15, kc=3.0, kphi=3.0),
}
PARAMS = ('phi_deg', 'c', 'k_deslizamiento', 'n', 'kc', 'kphi')
INCERT_CLAVE = {'phi_deg': 'phi', 'c': 'c', 'k_deslizamiento': 'K', 'n': 'n', 'kc': 'kc', 'kphi': 'kphi'}
G_RANGO = (0.27, 9.80665)   # rango de gravedades de los cinco cuerpos
ANG_SOBOL = 10.0            # pendiente de la corrida «subida» de la sensibilidad (grados)


# ---------------------------------------------------------------------------------------------------------------------
# Simulacion de una corrida (con DT y constantes de regularizacion configurables)
# ---------------------------------------------------------------------------------------------------------------------
def simular(c):
    """Una corrida de lazo abierto (v, w constantes) con el modelo de R1. Devuelve metricas propias (y traza opcional)."""
    import atp_rover.rover as R
    from atp_rover.planetas import planeta
    from atp_rover.terreno import Terreno
    d = {**DEF_SIM, **c}
    pl = planeta(d['planeta'])
    if d['suelo']:
        pl['suelo'].update(d['suelo'])
    if d['g'] is not None:
        pl['g'] = float(d['g'])
    for k, v in (d.get('atm') or {}).items():
        pl[k] = v
    ter = Terreno(d['terreno'], d['semilla'], pl['suelo'], extension=d['extension'], res=d['res'], **(d['terreno_params'] or {}))
    guardado = (R.DT, R.EPS_V, R.VS, R.V_ROD)
    R.DT = float(d['dt'])
    esc = d.get('escala_vel')
    if esc:
        R.EPS_V, R.VS, R.V_ROD = guardado[1] * esc, guardado[2] * esc, guardado[3] * esc
    try:
        rov = R.Rover(pl, ter, d['rover'], x=d.get('x0', 0.0), y=0.0, rumbo_deg=d.get('rumbo_deg', 0.0))
        rov.asentar(d['settle'])
        dt = R.DT
        n = int(round(d['T'] / dt))
        nvent = int(round(d['ventana_s'] / dt))
        p0 = rov.s[:3].copy()   # tras asentar
        sl_sum = 0.0
        sl_v = 0.0
        xs_v = None
        traza = [] if d['traza_cada'] else None
        roll_max = pitch_max = 0.0
        for k in range(n):
            rov.paso(d['v'], d['w'])
            u = rov._u1
            sl_sum += float(np.mean(np.abs(u['sl'])))
            if k >= n - nvent:
                sl_v += float(np.mean(np.abs(u['sl'])))
            if k == n - nvent - 1:
                xs_v = rov.s[:3].copy()
            if traza is not None and k % d['traza_cada'] == 0:
                e = rov.estado()
                traza.append([rov.t, e['x'], e['y'], e['z'], e['rumbo_deg'], e['cabeceo_deg'], e['alabeo_deg'], e['v'],
                              float(np.mean(np.abs(u['sl']))), float(np.sum(u['fn']))])
            if k % 20 == 0:
                e = rov.estado()
                roll_max = max(roll_max, abs(e['alabeo_deg']))
                pitch_max = max(pitch_max, abs(e['cabeceo_deg']))
            if rov.vuelco:
                break
        p1 = rov.s[:3]
        dist = float(np.linalg.norm(p1 - p0))
        if xs_v is None:
            xs_v = p0
        v_fin = float((p1[0] - xs_v[0]) / max(d['ventana_s'], 1e-9))   # velocidad de avance en x (rumbo 0), ultimos s
        out = dict(dist_m=dist, dx=float(p1[0] - p0[0]), dy=float(p1[1] - p0[1]), wh=rov.energia_ws / 3600.0,
                   wh_m=rov.energia_ws / 3600.0 / max(dist, 0.2),
                   wh_m_traccion=(rov.energia_ws - P_BASE_W * rov.t) / 3600.0 / max(dist, 0.2),
                   desl=sl_sum / max(rov.n_pasos, 1), desl_vent=sl_v / max(nvent, 1), v_fin=v_fin, vuelco=bool(rov.vuelco),
                   alabeo_max=roll_max, cabeceo_max=pitch_max, t=float(rov.t))
        if traza is not None:
            out['traza'] = traza
        return out
    finally:
        R.DT, R.EPS_V, R.VS, R.V_ROD = guardado


def _worker_init():
    try:
        os.nice(10)
    except OSError:
        pass


def _clave(c):
    return json.dumps(c, sort_keys=True)


def _correr(c):
    t0 = time.time()
    r = simular(c)
    r['pared_s'] = round(time.time() - t0, 2)
    return {'cfg': c, 'met': r}


def ejecutar(cfgs, cache, trab, etiqueta=''):
    """Corre las configuraciones (con cache jsonl) y devuelve la lista de metricas en el mismo orden."""
    hechos = {}
    if cache and os.path.exists(cache):
        with open(cache) as f:
            for linea in f:
                r = json.loads(linea)
                hechos[_clave(r['cfg'])] = r['met']
    vistos, falta = set(), []
    for c in cfgs:
        k = _clave(c)
        if k not in hechos and k not in vistos:
            vistos.add(k)
            falta.append(c)
    print(f'[{etiqueta}] {len(cfgs)} corridas, {len(cfgs) - len(falta)} en cache, {len(falta)} por correr ({trab} trab.)', flush=True)
    if falta:
        t0 = time.time()
        if trab <= 1:
            it = map(_correr, falta)
            ex = None
        else:
            ex = ProcessPoolExecutor(max_workers=trab, initializer=_worker_init)
            it = ex.map(_correr, falta, chunksize=1)
        for i, r in enumerate(it, 1):
            hechos[_clave(r['cfg'])] = r['met']
            if cache:
                with open(cache, 'a') as f:
                    f.write(json.dumps(r) + '\n')
            if i % 20 == 0 or i == len(falta):
                print(f'  [{etiqueta}] {i}/{len(falta)}  {time.time() - t0:.0f} s', flush=True)
        if ex:
            ex.shutdown()
    return [hechos[_clave(c)] for c in cfgs]


# ---------------------------------------------------------------------------------------------------------------------
# (1) Convergencia numerica
# ---------------------------------------------------------------------------------------------------------------------
TOL_REL = 0.02          # criterio: el dt de produccion (0.01) difiere < 2 % del dt mas fino en distancia y Wh/m
TOL_DESL = 0.01         # ... y < 0.01 en deslizamiento medio (absoluto)
TOL_REL_TERRENO = 0.05  # criterio de resolucion del terreno: res 0.1 vs 0.05, media de semillas, < 5 % en dist y Wh/m
TOL_DESL_TERRENO = 0.02
MET_CONV = ('dist_m', 'wh_m', 'desl')


def casos_dt(rapido):
    casos = [('marte', 'llano', 0, {}), ('luna', 'llano', 0, {}), ('titan', 'llano', 0, {}), ('ceres', 'llano', 0, {}),
             ('tierra', 'llano', 0, {}), ('marte', 'pendiente', 0, {'angulo_deg': 10.0}), ('marte', 'rocoso', 1, {})]
    return casos[:2] if rapido else casos


def convergencia_dt(rapido, cache, trab):
    dts = (0.02, 0.01, 0.005) if rapido else (0.02, 0.01, 0.005, 0.0025)
    T = 4.0 if rapido else 10.0
    cfgs = [dict(planeta=p, terreno=t, semilla=s, terreno_params=tp, dt=dt, T=T, settle=0.6)
            for (p, t, s, tp) in casos_dt(rapido) for dt in dts]
    res = ejecutar(cfgs, cache, trab, 'dt')
    filas, i = [], 0
    for (p, t, s, tp) in casos_dt(rapido):
        por_dt = {dt: res[i + j] for j, dt in enumerate(dts)}
        i += len(dts)
        ref = por_dt[dts[-1]]
        fila = {'caso': f'{p}/{t}' + (f'/{tp.get("angulo_deg")}°' if tp else ''), 'dts': list(dts)}
        ok = True
        for m in MET_CONV:
            vals = [por_dt[dt][m] for dt in dts]
            fila[m] = vals
            e01 = vals[1] - ref[m]
            if m == 'desl':
                fila[f'err_dt0.01_{m}'] = e01
                ok &= abs(e01) <= TOL_DESL
            else:
                fila[f'err_dt0.01_{m}'] = e01 / ref[m]
                ok &= abs(e01 / ref[m]) <= TOL_REL
            # orden observado (tres niveles consecutivos 0.02, 0.01, 0.005); None si no es monotono / sin sentido
            d1, d2 = vals[0] - vals[1], vals[1] - vals[2]
            fila[f'orden_{m}'] = float(math.log2(abs(d1) / abs(d2))) if (d1 * d2 > 0 and d2 != 0) else None
        fila['aceptado'] = bool(ok)
        filas.append(fila)
    return {'dts': list(dts), 'T_s': T, 'criterio': f'|m(0.01) - m(dt mas fino)| <= {TOL_REL:.0%} (dist, Wh/m) y <= {TOL_DESL} (deslizamiento medio)',
            'casos': filas, 'aceptado': all(f['aceptado'] for f in filas)}


def convergencia_terreno(rapido, cache, trab):
    resols = (0.2, 0.1, 0.05)
    tipos = [('rocoso', s) for s in ((1,) if rapido else (1, 2, 3))] + [('dunas', s) for s in ((1,) if rapido else (1, 2, 3))]
    if rapido:
        tipos = tipos[:1]
    T = 8.0 if rapido else 16.0
    cfgs = [dict(planeta='marte', terreno=t, semilla=s, res=r, T=T, settle=0.6, extension=40.0, x0=-14.0) for (t, s) in tipos for r in resols]
    res = ejecutar(cfgs, cache, trab, 'terreno')
    casos, i = [], 0
    for (t, s) in tipos:
        por = {r: res[i + j] for j, r in enumerate(resols)}
        i += len(resols)
        casos.append({'caso': f'{t}/s{s}', **{f'res{r}': {m: por[r][m] for m in MET_CONV + ('alabeo_max', 'cabeceo_max')} for r in resols}})
    resumen = {}
    ok = True
    for t in sorted({t for t, _ in tipos}):
        cs = [c for c in casos if c['caso'].startswith(t)]
        # media de semillas de la diferencia relativa res 0.1 vs 0.05 (y 0.2 vs 0.05 para contexto)
        r = {}
        for m in MET_CONV:
            ref = np.array([c['res0.05'][m] for c in cs])
            for rr in (0.2, 0.1):
                v = np.array([c[f'res{rr}'][m] for c in cs])
                r[f'{m}_res{rr}_vs_0.05'] = float(np.mean(np.abs(v - ref)) if m == 'desl' else np.mean(np.abs(v - ref) / np.abs(ref)))
        ok_t = (r['dist_m_res0.1_vs_0.05'] <= TOL_REL_TERRENO and r['wh_m_res0.1_vs_0.05'] <= TOL_REL_TERRENO
                and r['desl_res0.1_vs_0.05'] <= TOL_DESL_TERRENO)
        r['aceptado'] = bool(ok_t)
        ok &= ok_t
        resumen[t] = r
    return {'resoluciones_m': list(resols), 'T_s': T, 'planeta': 'marte',
            'criterio': f'media sobre semillas de |m(res 0.1) - m(res 0.05)| <= {TOL_REL_TERRENO:.0%} relativo (dist, Wh/m) y <= {TOL_DESL_TERRENO} (deslizamiento)',
            'casos': casos, 'resumen': resumen, 'aceptado': bool(ok)}


# ---------------------------------------------------------------------------------------------------------------------
# (2) Sensibilidad global: Sobol (Saltelli 2010 para S1, Jansen para ST) con secuencia de Sobol mezclada y semilla fija
# ---------------------------------------------------------------------------------------------------------------------
def disenio_saltelli(k, N, semilla=SEMILLA):
    from scipy.stats import qmc
    s = qmc.Sobol(d=2 * k, scramble=True, seed=semilla).random(N)
    A, B = s[:, :k], s[:, k:]
    mats = [A, B]
    for i in range(k):
        AB = A.copy()
        AB[:, i] = B[:, i]
        mats.append(AB)
    return mats           # [A, B, AB_0..AB_{k-1}], cada una (N, k) en [0,1]


def indices_sobol(fA, fB, fAB, n_boot=300, semilla=SEMILLA):
    """S1 (Saltelli 2010) y ST (Jansen 1999) con IC95 % por bootstrap sobre las filas. fAB: lista de arreglos (N,)."""
    fA, fB = np.asarray(fA, float), np.asarray(fB, float)
    k, N = len(fAB), len(fA)

    def calc(ia):
        a, b = fA[ia], fB[ia]
        var = np.var(np.concatenate([a, b]), ddof=1)
        if var <= 0:
            return np.zeros(k), np.zeros(k)
        s1 = np.array([np.mean(b * (np.asarray(f)[ia] - a)) / var for f in fAB])
        st = np.array([0.5 * np.mean((a - np.asarray(f)[ia]) ** 2) / var for f in fAB])
        return s1, st
    s1, st = calc(np.arange(N))
    rng = np.random.default_rng(semilla)
    bs = [calc(rng.integers(0, N, N)) for _ in range(n_boot)]
    s1b, stb = np.array([b[0] for b in bs]), np.array([b[1] for b in bs])
    return {'S1': s1.tolist(), 'ST': st.tolist(),
            'S1_ic95': [[float(np.percentile(s1b[:, i], 2.5)), float(np.percentile(s1b[:, i], 97.5))] for i in range(k)],
            'ST_ic95': [[float(np.percentile(stb[:, i], 2.5)), float(np.percentile(stb[:, i], 97.5))] for i in range(k)]}


def _mapear(u, planeta_clave, con_g=False):
    """Vector u en [0,1]^k -> dict de suelo (y g). Orden: PARAMS (+ 'g' al final si con_g)."""
    from atp_rover.planetas import PLANETAS
    nom = PLANETAS[planeta_clave if not con_g else 'marte']['suelo']
    inc = INCERT[planeta_clave if not con_g else 'marte']
    suelo = {}
    for j, p in enumerate(PARAMS):
        a = inc[INCERT_CLAVE[p]]
        x = float(u[j])
        if p in ('phi_deg', 'n'):
            v = nom[p] + a * (2 * x - 1)
        else:
            v = nom[p] * math.exp(math.log(a) * (2 * x - 1))
        suelo[p] = round(v, 9)
    suelo['n'] = max(0.5, suelo['n'])
    suelo['phi_deg'] = min(max(suelo['phi_deg'], 5.0), 50.0)
    g = None
    if con_g:
        g = round(math.exp(math.log(G_RANGO[0]) + (math.log(G_RANGO[1]) - math.log(G_RANGO[0])) * float(u[len(PARAMS)])), 9)
    return suelo, g


RESP = (('dist_m', 'distancia en T (plano)'), ('wh_m', 'Wh/m (plano)'), ('desl', 'deslizamiento (plano)'),
        ('s_dist_m', f'distancia en T (subida {ANG_SOBOL:g}°)'), ('s_desl', f'deslizamiento (subida {ANG_SOBOL:g}°)'))


def _cfgs_muestra(clave, suelo, g, T):
    base = dict(planeta=clave, suelo=suelo, g=g, T=T, settle=0.6)
    return (dict(base, terreno='llano'),
            dict(base, terreno='pendiente', terreno_params={'angulo_deg': ANG_SOBOL}))


def sobol_planeta(clave, N, T, cache, trab, con_g=False):
    k = len(PARAMS) + (1 if con_g else 0)
    mats = disenio_saltelli(k, N)
    cfgs, idx = [], []
    for m in mats:
        for u in m:
            suelo, g = _mapear(u, clave, con_g)
            cl = 'marte' if con_g else clave
            a, b = _cfgs_muestra(cl, suelo, g, T)
            idx.append(len(cfgs))
            cfgs += [a, b]
    res = ejecutar(cfgs, cache, trab, f'sobol-{"g" if con_g else clave}')
    F = {}
    for r_name, _ in RESP:
        F[r_name] = []
    for ii in idx:
        pl, sl = res[ii], res[ii + 1]
        F['dist_m'].append(pl['dist_m'])
        F['wh_m'].append(pl['wh_m'])
        F['desl'].append(pl['desl'])
        F['s_dist_m'].append(sl['dist_m'] * (1.0 if sl['dx'] >= 0 else -1.0))
        F['s_desl'].append(sl['desl'])
    F = {k_: np.array(v).reshape(len(mats), N) for k_, v in F.items()}
    nombres = list(PARAMS) + (['g'] if con_g else [])
    out = {'N': N, 'k': k, 'corridas': len(cfgs), 'T_s': T, 'parametros': nombres, 'respuestas': {}}
    allv = np.concatenate([F['dist_m'][0], F['dist_m'][1]])
    for r_name, etiqueta in RESP:
        f = F[r_name]
        ind = indices_sobol(f[0], f[1], f[2:])
        pool = np.concatenate([f[0], f[1]])
        out['respuestas'][r_name] = {
            'etiqueta': etiqueta, **ind,
            'p05': float(np.percentile(pool, 5)), 'p50': float(np.percentile(pool, 50)), 'p95': float(np.percentile(pool, 95)),
            'min': float(pool.min()), 'max': float(pool.max()),
            'suma_S1': float(np.sum(ind['S1']))}
    # valor nominal (suelo del planeta), mismos T
    cl = 'marte' if con_g else clave
    nomc = _cfgs_muestra(cl, None, (None if not con_g else None), T)
    nom = ejecutar(list(nomc), cache, 1, 'nominal')
    out['nominal'] = {'dist_m': nom[0]['dist_m'], 'wh_m': nom[0]['wh_m'], 'desl': nom[0]['desl'],
                      's_dist_m': nom[1]['dist_m'], 's_desl': nom[1]['desl']}
    out['rangos'] = {p: _rango_txt(p, INCERT[cl]) for p in PARAMS}
    if con_g:
        out['rangos']['g'] = f'log-uniforme {G_RANGO[0]}-{G_RANGO[1]:.2f} m/s2'
    del allv
    return out


def _rango_txt(p, inc):
    a = inc[INCERT_CLAVE[p]]
    return f'±{a:g}' + (' °' if p == 'phi_deg' else '') if p in ('phi_deg', 'n') else f'x/÷ {a:g} (log-unif.)'


# ---- pendiente maxima segura bajo incertidumbre (biseccion por muestra) ---------------------------------------------------
ANG_MAX = 30.0
DESL_SEGURO = 0.5
V_AVANCE_MIN = 0.2 * V_CMD      # igual que el criterio de «atasco» de R1: avanzar al menos 0.2 de lo mandado
T_PEND = 8.0


def _seguro(m):
    return bool((not m['vuelco']) and m['desl'] <= DESL_SEGURO and m['v_fin'] >= V_AVANCE_MIN)


def _biseccion(c):
    """Pendiente maxima segura (grados) por biseccion en [0, 30] con 5 evaluaciones (resolucion ~0.9°). Un solo proceso."""
    lo, hi = 0.0, ANG_MAX
    base = dict(c, terreno='pendiente', T=T_PEND, settle=0.6)
    m = simular(dict(base, terreno_params={'angulo_deg': hi}))
    if _seguro(m):
        return {'max_seguro_deg': hi, 'tope': True}
    for _ in range(5):
        mid = 0.5 * (lo + hi)
        if _seguro(simular(dict(base, terreno_params={'angulo_deg': mid}))):
            lo = mid
        else:
            hi = mid
    return {'max_seguro_deg': lo, 'tope': False}


def _correr_bis(c):
    t0 = time.time()
    r = _biseccion(c)
    r['pared_s'] = round(time.time() - t0, 1)
    return {'cfg': dict(c, _tipo='biseccion'), 'met': r}


def pendiente_segura_mc(clave, N, cache, trab):
    """Propaga la incertidumbre del suelo a la pendiente maxima segura: N muestras Sobol de A (+ nominal). Spearman por parametro."""
    from scipy.stats import spearmanr
    A = disenio_saltelli(len(PARAMS), N)[0]
    cfgs = [dict(planeta=clave, suelo=_mapear(u, clave)[0]) for u in A] + [dict(planeta=clave)]
    cfgs_t = [dict(c, _tipo='biseccion') for c in cfgs]
    hechos = {}
    if cache and os.path.exists(cache):
        with open(cache) as f:
            for linea in f:
                r = json.loads(linea)
                hechos[_clave(r['cfg'])] = r['met']
    falta = [c for c in cfgs if _clave(dict(c, _tipo='biseccion')) not in hechos]
    print(f'[pend-{clave}] {len(cfgs)} biseciones, {len(falta)} por correr', flush=True)
    if falta:
        ex = ProcessPoolExecutor(max_workers=trab, initializer=_worker_init) if trab > 1 else None
        it = ex.map(_correr_bis, falta, chunksize=1) if ex else map(_correr_bis, falta)
        for r in it:
            hechos[_clave(r['cfg'])] = r['met']
            if cache:
                with open(cache, 'a') as f:
                    f.write(json.dumps(r) + '\n')
        if ex:
            ex.shutdown()
    v = np.array([hechos[_clave(c)]['max_seguro_deg'] for c in cfgs_t[:-1]])
    nominal = hechos[_clave(cfgs_t[-1])]['max_seguro_deg']
    sp = {}
    for j, p in enumerate(PARAMS):
        x = np.array([_mapear(u, clave)[0][p] for u in A])
        r = spearmanr(x, v)[0] if np.ptp(v) > 0 else 0.0
        sp[p] = float(r) if r == r else 0.0
    return {'N': N, 'nominal_deg': nominal, 'p05': float(np.percentile(v, 5)), 'p50': float(np.percentile(v, 50)),
            'p95': float(np.percentile(v, 95)), 'min': float(v.min()), 'max': float(v.max()),
            'frac_tope': float(np.mean(v >= ANG_MAX - 1e-9)), 'spearman': sp,
            'criterio': f'sin vuelco, deslizamiento medio <= {DESL_SEGURO}, velocidad de avance en los ultimos 3 s >= {V_AVANCE_MIN:g} m/s, '
                        f'subida de {T_PEND:g} s a {V_CMD} m/s mandado; biseccion en [0, {ANG_MAX:g}]°'}


def barrido_g(cache, trab, rapido):
    """Metricas planas con el suelo de Marte y la g variada (aislar el efecto de la gravedad) + elasticidad local."""
    gs = [0.27, 1.62, 3.71, 9.80665] if rapido else [0.27, 0.5, 1.0, 1.62, 2.5, 3.71, 6.0, 9.80665]
    T = 6.0
    cfgs = [dict(planeta='marte', g=g, T=T, settle=0.6, terreno='llano') for g in gs]
    cfgs += [dict(planeta='marte', g=g, T=T, settle=0.6, terreno='pendiente', terreno_params={'angulo_deg': ANG_SOBOL}) for g in gs]
    res = ejecutar(cfgs, cache, trab, 'barrido-g')
    n = len(gs)
    filas = [{'g': g, 'plano': {k: res[i][k] for k in ('dist_m', 'wh_m', 'wh_m_traccion', 'desl')},
              'subida': {k: res[n + i][k] for k in ('dist_m', 'dx', 'desl', 'v_fin')}} for i, g in enumerate(gs)]
    lg = np.log(gs)
    el = {}
    for m in ('dist_m', 'wh_m', 'wh_m_traccion', 'desl'):
        y = np.array([f['plano'][m] for f in filas])
        el[m] = float(np.polyfit(lg, np.log(np.maximum(y, 1e-9)), 1)[0])
    return {'suelo': 'marte', 'T_s': T, 'filas': filas, 'elasticidad_loglog_plano': el}


# ---------------------------------------------------------------------------------------------------------------------
# (4) Invariancias
# ---------------------------------------------------------------------------------------------------------------------
def _energia_total(rov, R):
    """Mecanica del chasis y las ruedas + energia elastica de los resortes de contacto (J). Misma geometria que Rover._fuerzas."""
    p = rov.p
    s = rov.s
    Rm = R._quat_mat(s[3:7] / np.linalg.norm(s[3:7]))
    hub = s[0:3] + rov.hubs @ Rm.T
    h, n = rov.ter.altura_normal(hub[:, 0], hub[:, 1])
    d = np.einsum('ij,ij->i', n, hub - np.column_stack([hub[:, 0], hub[:, 1], h]))
    pen = np.maximum(p.radio_rueda - d, 0.0)
    cin = 0.5 * p.masa * float(s[7:10] @ s[7:10]) + 0.5 * float(np.sum(rov.I * s[10:13] ** 2))
    return (cin + p.masa * rov.g * float(s[2]) + 0.5 * p.inercia_rueda * float(np.sum(s[13:19] ** 2))
            + 0.5 * p.k_susp * float(np.sum(pen ** 2)))


def _rover_inv(pl='marte', terreno='llano', suelo=None, rover=None, tp=None, atm0=True, semilla=0, extension=40.0):
    import atp_rover.rover as R
    from atp_rover.planetas import planeta
    from atp_rover.terreno import Terreno
    P = planeta(pl)
    if atm0:
        P['densidad_atm'] = 0.0
    if suelo:
        P['suelo'].update(suelo)
    T = Terreno(terreno, semilla, P['suelo'], extension=extension, **(tp or {}))
    return R, R.Rover(P, T, rover)


def inv_g_estatica():
    """Carga estatica: la suma de fuerzas normales en reposo es m g (error relativo) para la g de cada cuerpo."""
    from atp_rover.planetas import PLANETAS
    out = {}
    for pl in sorted(PLANETAS):
        R, rov = _rover_inv(pl)
        rov.asentar(2.0)
        rov.paso(0.0, 0.0)
        fn = float(np.sum(rov._u1['fn']))
        out[pl] = {'g': rov.g, 'suma_fn_N': fn, 'm_g_N': rov.p.masa * rov.g, 'err_rel': fn / (rov.p.masa * rov.g) - 1.0,
                   'hundimiento_rueda_m': float(np.mean(rov._u1['z']))}
    return out


def inv_similitud(lam=4.0, T0=8.0, dt0=0.01):
    """Similitud dinamica: g' = lam g con k, c, tau_max, kw, suelo (kc, kphi, c) y velocidades reescalados, tiempo / sqrt(lam).
    Sin el termino viscoso fijo 0.02*omega de la rueda (rover.py), que no escala, la trayectoria adimensional seria identica."""
    sq = math.sqrt(lam)
    from atp_rover.planetas import PLANETAS
    from atp_rover.rover import ParametrosRover
    s0 = PLANETAS['luna']['suelo']
    p0 = ParametrosRover()
    ref = simular(dict(planeta='luna', atm={'densidad_atm': 0.0}, T=T0, dt=dt0, settle=0.6, v=0.5, w=0.1, traza_cada=100))
    esc = simular(dict(planeta='luna', g=PLANETAS['luna']['g'] * lam, atm={'densidad_atm': 0.0},
                       suelo={'kc': s0['kc'] * lam, 'kphi': s0['kphi'] * lam, 'c': s0['c'] * lam},
                       rover={'k_susp': p0.k_susp * lam, 'c_susp': p0.c_susp * sq, 'tau_max': p0.tau_max * lam, 'kw': p0.kw * sq},
                       T=T0 / sq, dt=dt0 / sq, settle=0.6 / sq, v=0.5 * sq, w=0.1 * sq, escala_vel=sq, traza_cada=100))
    a, b = np.array(ref['traza']), np.array(esc['traza'])
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    esc_len = max(float(np.max(np.abs(a[:, 1:3]))), 1e-9)
    return {'lambda': lam, 'ref': {'x': float(a[-1, 1]), 'y': float(a[-1, 2]), 'rumbo': float(a[-1, 4]), 'desl': ref['desl']},
            'escalada': {'x': float(b[-1, 1]), 'y': float(b[-1, 2]), 'rumbo': float(b[-1, 4]), 'desl': esc['desl']},
            'err_pos_max_rel': float(np.max(np.abs(a[:, 1:3] - b[:, 1:3])) / esc_len),
            'err_rumbo_max_deg': float(np.max(np.abs(a[:, 4] - b[:, 4]))),
            'err_desl': float(abs(ref['desl'] - esc['desl'])),
            'razon_wh_traccion': float((esc['wh'] - P_BASE_W * esc['t'] / 3600.0) / max(ref['wh'] - P_BASE_W * ref['t'] / 3600.0, 1e-12))}


def inv_simetria(T=8.0):
    """Simetria izquierda-derecha: terreno reflejado (y -> -y) y orden (v, -w) deben dar la trayectoria reflejada."""
    import atp_rover.rover as R
    from atp_rover.planetas import planeta
    from atp_rover.terreno import Terreno

    def corre(espejo, w, tipo, semilla):
        P = planeta('marte')
        ter = Terreno(tipo, semilla, P['suelo'], extension=40.0)
        if espejo:
            ter.z = ter.z[::-1, :].copy()
            ter.zona = ter.zona[::-1, :].copy()
        rov = R.Rover(P, ter)
        rov.asentar(0.6)
        tr = []
        for k in range(int(round(T / R.DT))):
            rov.paso(0.5, w)
            if k % 100 == 0:
                e = rov.estado()
                tr.append([e['x'], e['y'], e['rumbo_deg'], e['alabeo_deg'], e['cabeceo_deg'], float(np.mean(np.abs(rov._u1['sl'])))])
        return np.array(tr)
    out = {}
    plano = corre(False, 0.0, 'llano', 0)
    out['llano_recto'] = {'max_abs_y_m': float(np.max(np.abs(plano[:, 1]))), 'max_abs_rumbo_deg': float(np.max(np.abs(plano[:, 2]))),
                          'max_abs_alabeo_deg': float(np.max(np.abs(plano[:, 3])))}
    for tipo, sem in (('llano', 0), ('rocoso', 2), ('dunas', 1)):
        a = corre(False, 0.12, tipo, sem)
        b = corre(True, -0.12, tipo, sem)
        sg = np.array([1, -1, -1, -1, 1, 1])
        d = np.abs(a - b * sg)
        out[f'espejo_{tipo}'] = {'max_err_x_m': float(d[:, 0].max()), 'max_err_y_m': float(d[:, 1].max()),
                                 'max_err_rumbo_deg': float(d[:, 2].max()), 'max_err_alabeo_deg': float(d[:, 3].max()),
                                 'max_err_cabeceo_deg': float(d[:, 4].max()), 'max_err_desl': float(d[:, 5].max()),
                                 'y_final_m': float(a[-1, 1]), 'recorrido_x_m': float(a[-1, 0])}
    return out


def inv_energia(T=5.0):
    """(a) sin traccion (phi = 0, c = 0), sin amortiguador ni motores, en una pendiente de 10°: la energia total se conserva
    (con rodadura_en_motor=False: con la fisica corregida la resistencia por compactacion disipa, y es correcto que lo haga).
    (b) igual con el amortiguador real: la energia total no sube. (c) con suelo real y motores apagados, rodando por inercia:
    la energia total no debe subir (el suelo es pasivo)."""
    out = {}
    for nombre, c_susp in (('sin_traccion_sin_amortiguador', 0.0), ('sin_traccion_amortiguador_real', 990.0)):
        R, rov = _rover_inv('marte', 'pendiente', suelo={'phi_deg': 0.0, 'c': 0.0}, rover={'c_susp': c_susp, 'kw': 0.0, 'rodadura_en_motor': False},   # sin perdida alguna: la compactacion (Rc) SI disipa
                            tp={'angulo_deg': 10.0})
        E = [_energia_total(rov, R)]
        for _ in range(int(round(T / R.DT))):
            rov.paso(0.0, 0.0)
            E.append(_energia_total(rov, R))
        E = np.array(E)
        pe = rov.p.masa * rov.g * abs(float(rov.s[0])) * math.sin(math.radians(10.0))   # energia potencial liberada
        out[nombre] = {'E0_J': float(E[0]), 'deriva_max_J': float(np.max(np.abs(E - E[0]))),
                       'deriva_rel_a_PE_liberada': float(np.max(np.abs(E - E[0])) / max(pe, 1e-9)),
                       'mayor_subida_por_paso_J': float(np.max(np.diff(E))), 'desplazamiento_m': float(rov.s[0]),
                       'PE_liberada_J': float(pe)}
    R, rov = _rover_inv('marte', 'llano', rover={'kw': 0.0})
    rov.asentar(1.0)
    rov.s[7] = 1.0
    rov.s[13:19] = 1.0 / rov.p.radio_rueda
    E = [_energia_total(rov, R)]
    V = [float(rov.s[7])]
    for _ in range(int(round(T / R.DT))):
        rov.paso(0.0, 0.0)
        E.append(_energia_total(rov, R))
        V.append(float(rov.s[7]))
    E = np.array(E)
    out['inercia_suelo_real_motores_apagados'] = {
        'E0_J': float(E[0]), 'E_final_J': float(E[-1]), 'mayor_subida_por_paso_J': float(np.max(np.diff(E))),
        'subida_total_max_J': float(np.max(E - E[0])), 'v_final_ms': V[-1], 'v_max_ms': float(np.max(V)),
        'desaceleracion_media_ms2': float((V[0] - V[-1]) / T)}
    return out


def inv_potencia_compactacion(cuerpos=('luna', 'marte', 'titan', 'ceres', 'tierra'), rodadura_en_motor=True):
    """Auditoria del balance de energia en regimen (llano, 0.5 m/s mandado): la potencia electrica de traccion del modelo contra la
    potencia minima que exige la resistencia por compactacion Rc (Rc * v / eta). Si el modelo cobra la resistencia, razon ~ 1."""
    from atp_rover.planetas import PLANETAS
    out = {}
    for pl in cuerpos:
        R, rov = _rover_inv(pl, atm0=False, rover={'rodadura_en_motor': bool(rodadura_en_motor)})
        rov.asentar(1.0)
        for _ in range(900):
            rov.paso(0.5, 0.0)
        u = rov._u1
        b, so = rov.p.ancho_rueda, PLANETAS[pl]['suelo']
        kp = so['kc'] / b + so['kphi']
        Rc = b * kp * u['z'] ** (so['n'] + 1.0) / (so['n'] + 1.0)
        v = float(rov.s[7])
        p_trac = rov.potencia_w - rov.p.p_base_w
        minimo = float(np.sum(Rc)) * v / rov.p.eta
        out[pl] = {'v_ms': v, 'Rc_total_N': float(np.sum(Rc)), 'P_traccion_modelo_W': float(p_trac),
                   'P_minima_Rc_v_sobre_eta_W': minimo, 'razon': float(p_trac / minimo), 'par_medio_Nm': float(np.mean(u['tau']))}
    return out


# ---------------------------------------------------------------------------------------------------------------------
# (3) Contraste con la literatura: lado del modelo + tabla de fuentes
# ---------------------------------------------------------------------------------------------------------------------
# Parametros de la «soil model B» del LRV (Apollo 17 Preliminary Science Report, sec. 8): phi 35°, c 0.17 kN/m2, kphi 0.81 N/cm3,
# kc 0.35 N/cm2, n 1.0, K 1.0 cm. Convertidos a SI (kc: 0.35 N/cm2 = 3.5 kPa; kphi: 0.81 N/cm3 = 810 kN/m3; K = 0.01 m).
LUNA_APOLLO17 = dict(kc=3500.0, kphi=810000.0, n=1.0, c=170.0, phi_deg=35.0, k_deslizamiento=0.01)
ANGULOS_LIT = (0, 5, 10, 15, 20, 25, 30)

LITERATURA = [
    {'id': 'L1', 'fuente': 'Schepelmann et al., NASA/TM-20250006958 (2025), Tabla 1 (valores tipicos de Bekker)',
     'url': 'https://ntrs.nasa.gov/api/citations/20250006958/downloads/TM-20250006958.pdf',
     'dato': 'Arena seca: n 1.1, kc 0.99, kphi 1528.43, c 1.04 kPa, phi 28°. «No existen parametros Bekker verdaderos para el regolito lunar» (se necesitarian in situ). GRC-3 suelto: n 1.0, kc 23.2, kphi 606.7, c 0.13 kPa, phi 36.7°.',
     'verificado': 'leido el PDF (texto extraido)'},
    {'id': 'L2', 'fuente': 'Mitchell et al., Apollo 17 Preliminary Science Report, sec. 8 «Soil Mechanics» (1973)',
     'url': 'https://www.nasa.gov/wp-content/uploads/static/history/alsj/WOTM//a17psr_Section8_SoilMechanics.pdf',
     'dato': 'Modelo B del LRV (consumo de energia medido en Apollo 16/17 coincide con la prediccion): phi 35°, c 0.17 kN/m2, kphi 0.81 N/cm3, kc 0.35 N/cm2, n 1.0, K 1.0 cm. Regolito: cohesion 0.1-1 kN/m2, phi 30-50°.',
     'verificado': 'leido el PDF (texto extraido, OCR con ruido en la figura 8-21; valores leidos de la leyenda)'},
    {'id': 'L3', 'fuente': 'Sullivan et al., JGR Planets 116, E02006 (2011), «Cohesions, friction angles, and other physical properties of Martian regolith from MER wheel trenches and wheel scuffs»',
     'url': 'https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2010JE003625',
     'dato': 'phi 30-37°, cohesion 0-2 kPa (zanjas de rueda); 0-11 kPa en rascaduras de rueda.',
     'verificado': 'solo el resumen devuelto por la busqueda (la pagina de Wiley dio 403)'},
    {'id': 'L4', 'fuente': 'Maimone, Cheng, Matthies, «Two years of visual odometry on the Mars Exploration Rovers», J. Field Robotics 24 (2007)',
     'url': 'https://www-robotics.jpl.nasa.gov/media/documents/rob-06-0081.R4.pdf',
     'dato': 'Opportunity en terreno arenoso de 17-23° de pendiente cerca de Wopmay (sol 249-268): semanas de intentos para alejarse; hasta 85 % de deslizamiento sobre obstaculos; los modelos de deslizamiento de la mecanica solo cubrian arena pura a 0, 45 y 90° del gradiente.',
     'verificado': 'leido el PDF (texto extraido)'},
    {'id': 'L5', 'fuente': 'JPL/NASA, «Opportunity Mars Rover Goes Six-Wheeling up a Ridge» (25-feb-2016)',
     'url': 'https://www.jpl.nasa.gov/news/opportunity-mars-rover-goes-six-wheeling-up-a-ridge/',
     'dato': 'Las ruedas patinaron menos del 20 % cuesta arriba en pendientes de hasta 30° (la mas empinada desde 2004), sobre afloramiento rocoso con material suelto.',
     'verificado': 'leida la pagina. NO es comparable con suelo suelto: es afloramiento.'},
    {'id': 'L6', 'fuente': 'Lindemann & Voorhees, «Mars Exploration Rover mobility assembly design, test and performance», IEEE SMC 2005',
     'url': 'https://www.researchgate.net/publication/4210296_Mars_Exploration_Rover_mobility_assembly_design_test_and_performance',
     'dato': 'Pruebas en arena con plataforma inclinable hasta 20°; el resumen de la busqueda dice «hasta 91 % de deslizamiento a 20° en suelo blando».',
     'verificado': 'NO VERIFICADO: no pude abrir el articulo (solo un resumen de buscador); no lo uso como cifra, solo como orden de magnitud.'},
    {'id': 'L7', 'fuente': 'Arvidson et al., JGR Planets 119 (2014), «Terrain physical properties ... first 360 sols of MSL Curiosity», y Arvidson et al. LPSC 2016 (cruce de megarizaduras)',
     'url': 'https://agupubs.onlinelibrary.wiley.com/doi/full/10.1002/2013JE004605',
     'dato': 'El deslizamiento de Curiosity depende sobre todo de la inclinacion; hasta 77 % en rizaduras de arena (>70 % 3D en megarizaduras, sol 1181); evita pendientes > 25° y pendientes arenosas > 12.5°.',
     'verificado': 'NO VERIFICADO: la pagina dio 403 y el PDF de LPSC no era texto; cifras tomadas del resumen del buscador.'},
    {'id': 'L8', 'fuente': 'Wong, «Theory of Ground Vehicles» (4a ed., 2008) y «Terramechanics and Off-Road Vehicle Engineering» (2010); Bekker (1969)',
     'url': 'https://www.researchgate.net/publication/243626623_Theory_of_Ground_Vehicles',
     'dato': 'Fuente de las ecuaciones (presion-hundimiento, Janosi-Hanamoto, resistencia por compactacion de una rueda rigida).',
     'verificado': 'NO VERIFICADO el texto: libros de pago, no accesibles. Sin coeficientes de resistencia al movimiento citables; no se compara la cifra Rc/W del modelo con un valor publicado.'},
    {'id': 'L9', 'fuente': 'Schepelmann et al. 2025 (L1) enlace usado por R4: predicting.pdf del KISS (Caltech)',
     'url': 'https://www.kiss.caltech.edu/papers/xterramechanics/papers/predicting.pdf',
     'dato': 'URL citada en ROVER.md R4.', 'verificado': 'hoy devuelve HTTP 404 (junto con simulations.pdf del mismo sitio); la cita de R4 no se puede reabrir.'},
]


def literatura_modelo(rapido, cache, trab):
    """Lado del modelo: (a) tabla de parametros contra la literatura, (b) deslizamiento y avance contra la pendiente (Marte, Tierra, Luna,
    Luna con los parametros de Apollo 17), (c) efecto de usar K y kc de Apollo 17 en la Luna."""
    angs = (0, 10, 20) if rapido else ANGULOS_LIT
    variantes = [('marte', None), ('tierra', None), ('luna', None), ('luna', LUNA_APOLLO17)]
    if rapido:
        variantes = variantes[:1] + variantes[3:]
    cfgs = []
    for pl, su in variantes:
        for a in angs:
            cfgs.append(dict(planeta=pl, suelo=su, terreno='pendiente', terreno_params={'angulo_deg': float(a)}, T=T_PEND, settle=0.6))
    res = ejecutar(cfgs, cache, trab, 'literatura')
    curvas, i = {}, 0
    for pl, su in variantes:
        nombre = pl + ('_apollo17' if su else '')
        curvas[nombre] = [{'angulo_deg': a, 'desl_final': res[i + j]['desl_vent'], 'desl_medio': res[i + j]['desl'], 'v_avance_ms': res[i + j]['v_fin'],
                           'seguro': _seguro(res[i + j])} for j, a in enumerate(angs)]
        i += len(angs)
    from atp_rover.planetas import PLANETAS
    luna = PLANETAS['luna']['suelo']
    comp = [{'cuerpo': 'tierra', 'contra': 'L1 arena seca', 'modelo': PLANETAS['tierra']['suelo'], 'lit': dict(n=1.1, kc=990.0, kphi=1528430.0, c=1040.0, phi_deg=28.0), 'acuerdo': 'identico (es la misma fila; K = 0.025 m sin contrastar)'},
            {'cuerpo': 'luna', 'contra': 'L2 modelo B del LRV (Apollo)', 'modelo': luna, 'lit': LUNA_APOLLO17,
             'acuerdo': f'phi, c, n, kphi coinciden; kc del modelo {luna["kc"] / LUNA_APOLLO17["kc"]:.2f}x el publicado; K {luna["k_deslizamiento"] / LUNA_APOLLO17["k_deslizamiento"]:.2f}x'},
            {'cuerpo': 'marte', 'contra': 'L3 Sullivan 2011', 'modelo': PLANETAS['marte']['suelo'], 'lit': dict(phi_deg='30-37', c='0-2 kPa (zanjas)'),
             'acuerdo': 'phi 33° y c 0.8 kPa caen dentro de los rangos; kc, kphi, n, K sin dato'},
            {'cuerpo': 'titan', 'contra': 'ninguno', 'modelo': PLANETAS['titan']['suelo'], 'lit': None, 'acuerdo': 'sin datos medidos de terramecanica: no contrastable'},
            {'cuerpo': 'ceres', 'contra': 'ninguno', 'modelo': PLANETAS['ceres']['suelo'], 'lit': None, 'acuerdo': 'sin datos medidos de terramecanica: no contrastable'}]
    return {'T_s': T_PEND, 'curvas': curvas, 'parametros': comp, 'fuentes': LITERATURA, 'luna_apollo17': LUNA_APOLLO17}


# ---------------------------------------------------------------------------------------------------------------------
# Informe
# ---------------------------------------------------------------------------------------------------------------------
def _f(x, d=3):
    return '-' if x is None or x != x else f'{x:.{d}f}'


def _pc(x):
    return '-' if x is None else f'{100 * x:+.2f} %'


def markdown(R, rapido):
    L = ['# Validacion cientifica del rover (R5)', '',
         f'Generado por `scripts/validacion_rover.py' + (' --rapido' if rapido else '') + '`. Todo es SIMULACION con el modelo de R1 sin modificar y con '
         'los suelos de `planetas.py` (Marte, Titan y Ceres son supuestos de orden de magnitud). Misma semilla de muestreo -> mismos numeros.', '']
    c = R.get('convergencia_dt')
    if c:
        L += ['## 1a. Convergencia en el paso de integracion', '', f'Criterio: {c["criterio"]}. Lazo abierto, 0.5 m/s mandado, T = {c["T_s"]:g} s tras asentar 0.6 s; '
              f'referencia = dt {c["dts"][-1]}. Paso de produccion: 0.01.', '',
              '| Caso | dist(0.02) | dist(0.01) | dist(ref) | Wh/m(0.02) | Wh/m(0.01) | Wh/m(ref) | desl(0.01) | err dist | err Wh/m | err desl | orden Wh/m | OK |', '|' + '---|' * 13]
        for f in c['casos']:
            L.append(f"| {f['caso']} | {_f(f['dist_m'][0], 4)} | {_f(f['dist_m'][1], 4)} | {_f(f['dist_m'][-1], 4)} | {_f(f['wh_m'][0], 5)} | {_f(f['wh_m'][1], 5)} | {_f(f['wh_m'][-1], 5)} | "
                     f"{_f(f['desl'][1], 4)} | {_pc(f['err_dt0.01_dist_m'])} | {_pc(f['err_dt0.01_wh_m'])} | {_f(f['err_dt0.01_desl'], 4)} | {_f(f['orden_wh_m'], 2)} | {'si' if f['aceptado'] else 'NO'} |")
        L += ['', f'**Veredicto: dt = 0.01 s {"cumple" if c["aceptado"] else "NO cumple"} el criterio en estos casos.** El orden observado de Wh/m ronda 1 (no 4 del RK4): '
              'la energia se integra con la potencia del inicio del paso (rectangulo). La distancia converge mas rapido que la energia.', '']
    t = R.get('convergencia_terreno')
    if t:
        L += ['## 1b. Convergencia en la resolucion del terreno', '', f'Criterio: {t["criterio"]}. Marte, T = {t["T_s"]:g} s, extension 40 m, mismo campo continuo (la semilla y la extension fijan las muestras del ruido; solo cambia el muestreo de la rejilla y la distancia de las diferencias finitas de la normal, que es `res`).', '',
              '| Terreno | dist res0.2 vs 0.05 | dist 0.1 vs 0.05 | Wh/m 0.2 vs 0.05 | Wh/m 0.1 vs 0.05 | desl 0.2 vs 0.05 | desl 0.1 vs 0.05 | OK |', '|' + '---|' * 8]
        for k, r in t['resumen'].items():
            L.append(f"| {k} | {_pc(r['dist_m_res0.2_vs_0.05'])} | {_pc(r['dist_m_res0.1_vs_0.05'])} | {_pc(r['wh_m_res0.2_vs_0.05'])} | {_pc(r['wh_m_res0.1_vs_0.05'])} | "
                     f"{_f(r['desl_res0.2_vs_0.05'], 4)} | {_f(r['desl_res0.1_vs_0.05'], 4)} | {'si' if r['aceptado'] else 'NO'} |")
        L += ['', f'**Veredicto: res = 0.1 m {"cumple" if t["aceptado"] else "NO cumple"} el criterio.** (valores absolutos medios sobre semillas).', '']
    S = R.get('sobol')
    if S:
        L += ['## 2. Sensibilidad global (Sobol) del suelo y de g', '',
              'Indices de primer orden S1 y totales ST (IC95 % por bootstrap) por respuesta. Muestreo Sobol-Saltelli con mezcla y semilla fija; rangos de incertidumbre = elecciones de este estudio '
              '(no datos): Marte/Luna «estimado», Titan/Ceres «supuesto» (rangos mas anchos). Respuestas: distancia en T y deslizamiento sobre llano y sobre una subida de '
              f'{ANG_SOBOL:g}°; Wh/m sobre llano. Cada celda `S1 / ST`.', '']
        for nombre, s in S.items():
            L += [f'### {nombre}  (N = {s["N"]}, {s["corridas"]} corridas, T = {s["T_s"]:g} s)', '', 'Rangos: ' + '; '.join(f'{p} {v}' for p, v in s['rangos'].items()), '',
                  '| Respuesta | nominal | p05 | p50 | p95 | ' + ' | '.join(s['parametros']) + ' | suma S1 |', '|' + '---|' * (6 + len(s['parametros']))]
            for rn, d in s['respuestas'].items():
                nom = s['nominal'].get(rn)
                cel = ' | '.join(f"{d['S1'][i]:.2f} / {d['ST'][i]:.2f}" for i in range(len(s['parametros'])))
                L.append(f"| {d['etiqueta']} | {_f(nom, 3)} | {_f(d['p05'], 3)} | {_f(d['p50'], 3)} | {_f(d['p95'], 3)} | {cel} | {d['suma_S1']:.2f} |")
            L.append('')
    P = R.get('pendiente_segura')
    if P:
        L += ['## 2b. Pendiente maxima segura bajo incertidumbre del suelo', '']
        k0 = next(iter(P.values()))
        L += [k0['criterio'] + '.', '', '| Cuerpo | nominal (°) | p05 | p50 | p95 | min-max | fraccion en el tope 30° | N | Spearman (parametro: rho) |', '|' + '---|' * 9]
        for pl, d in P.items():
            sp = ', '.join(f'{k} {v:+.2f}' for k, v in sorted(d['spearman'].items(), key=lambda kv: -abs(kv[1])))
            L.append(f"| {pl} | {_f(d['nominal_deg'], 1)} | {_f(d['p05'], 1)} | {_f(d['p50'], 1)} | {_f(d['p95'], 1)} | {_f(d['min'], 1)}-{_f(d['max'], 1)} | {d['frac_tope']:.2f} | {d['N']} | {sp} |")
        L.append('')
    G = R.get('barrido_g')
    if G:
        L += ['## 2c. Efecto aislado de la gravedad (suelo de Marte, g variada)', '',
              '| g (m/s²) | dist plano (m) | Wh/m | Wh/m tracc. | desl plano | desl subida 10° | v avance subida 10° (m/s) |', '|---|---|---|---|---|---|---|']
        for f in G['filas']:
            L.append(f"| {f['g']:.3g} | {_f(f['plano']['dist_m'], 3)} | {_f(f['plano']['wh_m'], 4)} | {_f(f['plano']['wh_m_traccion'], 4)} | {_f(f['plano']['desl'], 3)} | {_f(f['subida']['desl'], 3)} | {_f(f['subida']['v_fin'], 3)} |")
        e = G['elasticidad_loglog_plano']
        L += ['', 'Elasticidad (pendiente log-log sobre el rango de g) en el llano: ' + ', '.join(f'{k} {v:+.2f}' for k, v in e.items()) + '.', '']
    Li = R.get('literatura')
    if Li:
        L += ['## 3. Contraste con la literatura', '', '### Parametros del suelo del modelo contra datos publicados', '',
              '| Cuerpo | Contra | Acuerdo |', '|---|---|---|']
        for p in Li['parametros']:
            L.append(f"| {p['cuerpo']} | {p['contra']} | {p['acuerdo']} |")
        L += ['', f'### Deslizamiento y avance contra la pendiente (modelo, {Li["T_s"]:g} s a {V_CMD} m/s mandado; deslizamiento medio de los ultimos 3 s)', '',
              '| Variante | ' + ' | '.join(f'{c_["angulo_deg"]:g}°' for c_ in next(iter(Li['curvas'].values()))) + ' |', '|' + '---|' * (1 + len(next(iter(Li['curvas'].values()))))]
        for nom, cur in Li['curvas'].items():
            L.append(f'| {nom} (desl. / v m/s) | ' + ' | '.join(f"{c_['desl_final']:.2f} / {c_['v_avance_ms']:+.2f}" for c_ in cur) + ' |')
        L += ['', '### Fuentes (con URL; lo no verificado esta marcado)', '', '| Id | Fuente | Dato | Verificacion |', '|---|---|---|---|']
        for f in Li['fuentes']:
            L.append(f"| {f['id']} | [{f['fuente']}]({f['url']}) | {f['dato']} | {f['verificado']} |")
        L.append('')
    I = R.get('invariancias')
    if I:
        L += ['## 4. Invariancias', '']
        e = I['g_estatica']
        L += ['**Carga estatica.** Suma de fuerzas normales en reposo / (m g) - 1: ' + ', '.join(f"{k} {v['err_rel']:.1e}" for k, v in e.items()) + '.', '']
        s = I['similitud']
        for d in (s if isinstance(s, list) else [s]):
            L += [f"**Similitud dinamica (g x {d['lambda']:g}).** k, c, tau_max, kw, kc, kphi, c y velocidades reescalados, tiempo / sqrt(lambda): error de posicion maximo "
                  f"{d['err_pos_max_rel']:.1e} (relativo), de rumbo {d['err_rumbo_max_deg']:.1e}°, de deslizamiento {d['err_desl']:.1e}; energia de traccion x{d['razon_wh_traccion']:.2f} (esperado ~ lambda).", '']
        m = I['simetria']
        L += ['**Simetria izquierda-derecha.** ' + '; '.join(f"{k}: err y {v['max_err_y_m']:.1e} m, rumbo {v['max_err_rumbo_deg']:.1e}°, alabeo {v['max_err_alabeo_deg']:.1e}°" for k, v in m.items() if k.startswith('espejo'))
              + f"; recto en llano: |y| max {m['llano_recto']['max_abs_y_m']:.1e} m.", '']
        en = I['energia']
        for k, v in en.items():
            if 'deriva_max_J' in v:
                L.append(f"- `{k}`: deriva maxima {v['deriva_max_J']:.2e} J sobre {v['PE_liberada_J']:.0f} J de energia potencial liberada ({v['deriva_rel_a_PE_liberada']:.1e}); mayor subida por paso {v['mayor_subida_por_paso_J']:.1e} J.")
            else:
                L.append(f"- `{k}`: E0 {v['E0_J']:.2f} J -> {v['E_final_J']:.2f} J; subida maxima sobre E0 {v['subida_total_max_J']:.1e} J (el suelo es pasivo); desaceleracion media {v['desaceleracion_media_ms2']:.4f} m/s².")
        L += ['', '### Hallazgo: la resistencia por compactacion no se cobra en la energia electrica', '',
              'Auditoria en regimen (llano, 0.5 m/s mandado): potencia electrica de traccion del modelo contra la minima que exige la resistencia por compactacion Rc (Rc·v/eta).', '',
              '| Cuerpo | v (m/s) | Rc total (N) | P traccion modelo (W) | P minima Rc·v/eta (W) | razon |', '|---|---|---|---|---|---|']
        for k, v in I['potencia_compactacion'].items():
            L.append(f"| {k} | {v['v_ms']:.3f} | {v['Rc_total_N']:.1f} | {v['P_traccion_modelo_W']:.2f} | {v['P_minima_Rc_v_sobre_eta_W']:.2f} | {v['razon']:.3f} |")
        L += ['', 'En `rover.py`, `fx` incluye `-Rc·tanh(vx/V_ROD)` y el mismo `fx` entra en el par de la rueda (`- fx*Rw`): en regimen `fx = 0` y el motor solo da 0.04 N·m. '
              'La compactacion frena el chasis pero devuelve ese trabajo a la rueda; la resistencia al avance no cuesta bateria. En una rueda rigida la presion normal es radial y no da par '
              'respecto al cubo, asi que el par motor deberia ser `Rw·T` con `T = Rc` en regimen. Efecto: el Wh/m «de traccion» de R4 (P1) y su lectura sobre la gravedad **no incluyen el costo de rodar** (solo arranque, giro y deslizamiento). '
              'Reportado a R6 (no se modifico `rover.py`).', '']
    N = R.get('notas')
    if N:
        L += ['## Notas', ''] + [f'- {n}' for n in N] + ['']
    return '\n'.join(L)


def _num(o):
    if isinstance(o, dict):
        return {k: _num(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_num(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


NOTAS = [
    'Las metricas de este estudio se calculan en el script (desplazamiento 3D, deslizamiento promediado en cada paso, energia de `Rover.energia_ws`), no con `Rover.metricas`: '
    'esa funcion muestrea cada 10 pasos y su temporizador de atasco suma 0.1 s por muestra, de modo que solo es correcta con DT = 0.01 (defecto latente si se cambia DT).',
    'Los rangos de incertidumbre del suelo son decisiones de este estudio, no datos medidos; los intervalos describen «si el suelo estuviera en este rango», no una probabilidad real.',
    'Corridas cortas (6 s de sim.) y de lazo abierto: el deslizamiento y Wh/m incluyen el arranque; la sensibilidad es la de estas metricas, no la de una mision.',
    'Sin ruido de sensores, una sola masa de rover (150 kg), una sola velocidad (0.5 m/s) y una sola rueda (0.25 m): no se exploro la geometria.',
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true', help='subconjunto minimo (minutos); escribe en runs/rover_validacion_rapido/')
    ap.add_argument('--trabajadores', type=int, default=5)
    ap.add_argument('--salida')
    ap.add_argument('--cache', help='jsonl de corridas ya hechas (reanuda)')
    ap.add_argument('--secciones', default='dt,terreno,sobol,pendiente,g,literatura,invariancias',
                    help='lista separada por comas; si falta alguna, el informe la omite')
    a = ap.parse_args(argv)
    salida = a.salida or (os.path.join(RAIZ, 'runs', 'rover_validacion_rapido') if a.rapido else os.path.join(RAIZ, 'docs', 'rover_resultados'))
    os.makedirs(salida, exist_ok=True)
    cache = a.cache or os.path.join(RAIZ, 'runs', 'rover_validacion' + ('_rapido' if a.rapido else ''), 'corridas.jsonl')
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    sec = set(a.secciones.split(','))
    R = {'rapido': a.rapido, 'semilla_muestreo': SEMILLA}
    t0 = time.time()
    if 'dt' in sec:
        R['convergencia_dt'] = convergencia_dt(a.rapido, cache, a.trabajadores)
    if 'terreno' in sec:
        R['convergencia_terreno'] = convergencia_terreno(a.rapido, cache, a.trabajadores)
    if 'sobol' in sec:
        N, T = (4, 3.0) if a.rapido else (32, 6.0)
        R['sobol'] = {}
        for pl in (('marte',) if a.rapido else ('marte', 'titan', 'ceres')):
            R['sobol'][pl] = sobol_planeta(pl, N, T, cache, a.trabajadores)
        R['sobol']['marte + g (log-uniforme 0.27-9.81)'] = sobol_planeta('marte', N, T, cache, a.trabajadores, con_g=True)
    if 'pendiente' in sec:
        R['pendiente_segura'] = {}
        plan = (('marte', 4),) if a.rapido else (('marte', 24), ('titan', 24), ('ceres', 24), ('luna', 24), ('tierra', 16))
        cache_p = cache.replace('corridas.jsonl', 'pendiente.jsonl')
        for pl, N in plan:
            R['pendiente_segura'][pl] = pendiente_segura_mc(pl, N, cache_p, a.trabajadores)
    if 'g' in sec:
        R['barrido_g'] = barrido_g(cache, a.trabajadores, a.rapido)
    if 'literatura' in sec:
        R['literatura'] = literatura_modelo(a.rapido, cache, a.trabajadores)
    if 'invariancias' in sec:
        R['invariancias'] = {'g_estatica': inv_g_estatica(), 'similitud': [inv_similitud(4.0)] + ([] if a.rapido else [inv_similitud(0.25)]),
                             'simetria': inv_simetria(4.0 if a.rapido else 8.0), 'energia': inv_energia(3.0 if a.rapido else 5.0),
                             'potencia_compactacion': inv_potencia_compactacion()}
    R['notas'] = NOTAS
    R['pared_s'] = round(time.time() - t0, 1)
    R = _num(R)
    sufijo = ''
    with open(os.path.join(salida, f'validacion{sufijo}.json'), 'w') as f:
        json.dump(R, f, indent=1, sort_keys=True)
        f.write('\n')
    md = markdown(R, a.rapido)
    with open(os.path.join(salida, f'validacion{sufijo}.md'), 'w') as f:
        f.write(md)
    print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
