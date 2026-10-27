#!/usr/bin/env python3
"""Experimento cientifico de la OD: cuanto mejora el apuntamiento predicho
cuando la estacion determina la orbita con sus propias mediciones.

Uso (dentro del contenedor, ver docs/OD.md):
  python3 scripts/experimento_od.py --sat ISS --edades 1,2,3,5 --repeticiones 6 \
      --salida docs/od_resultados_iss.json

Para cada edad N del TLE (dias desde su epoca) y cada pase de arranque (los
primeros pases de CDMX con elevacion maxima >= 20 deg tras N dias):
  * TLE viejo: SGP4 sin tocar.
  * OD con 1 pase de CDMX, con tres conjuntos de medidas: az/el, az/el +
    Doppler, az/el + rango + Doppler.
  * OD con 3 pases de la red (el de arranque + 2 mas en las 12 h siguientes),
    con los 2 pases elegidos por la regla "rapida" (los 2 siguientes con
    elevacion >= 20 deg) o por el PLANIFICADOR de informacion
    (od_experimento.seleccionar_pases_od).
y se mide el desapuntamiento sobre los pases de CDMX posteriores (el primero
tras 13 h del arranque, y +24 h, +48 h). Imprime tablas en Markdown y guarda
todo en JSON. Determinista: semilla fija por (edad, repeticion).
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'src', 'atp_ros'))

from atp_ros import medidas as M          # noqa: E402
from atp_ros import od                    # noqa: E402
from atp_ros import od_experimento as X   # noqa: E402
from atp_ros import orbit                 # noqa: E402

CONJUNTOS = {'az/el': ('az', 'el'), 'az/el+Dopp': ('az', 'el', 'rr'),
             'az/el+rng+Dopp': ('az', 'el', 'rng', 'rr')}
HORIZONTES = ('sig', '+24h', '+48h')
UMBRALES = {'Ka 3 dB (0.117)': 0.5 * 0.233, 'Ka lock -10 dB (0.213)': 0.913 * 0.233,
            'S 3 dB (1.59)': 0.5 * 3.18}

_G = {}


def construir(args, p_cal=None):
    cat = orbit.cargar_catalogo()
    tle = orbit.buscar_tle(cat, args.sat)
    est, _ = orbit.cargar_estaciones()
    claves = args.estaciones.split(',')
    return X.Escenario(tle, {k: est[k] for k in claves}, dias=args.dias, p_cal=p_cal,
                       bc_fisico=args.bc)


def _init(args, p_cal):
    _G['esc'] = construir(args, p_cal)
    _G['args'] = args


def pases_eval(esc, t_ref, clave, el_min=10.0):
    """Los pases de evaluacion: primero tras `t_ref`, y los primeros tras
    +24 h y +48 h."""
    out = {}
    for nombre, dh in zip(HORIZONTES, (0.0, 24.0, 48.0)):
        t = t_ref + dh * 3600.0
        ps = esc.pases(clave, t, t + 36 * 3600.0, el_min)
        out[nombre] = ps[0] if ps else None
    return out


def evaluar(esc, ef, evals, clave):
    return {h: (esc.error_pase(ef, p, clave) if p else None) for h, p in evals.items()}


def corrida(par):
    """Un escenario (edad N, repeticion k). Devuelve un dict serializable."""
    N, k = par
    esc, args = _G['esc'], _G['args']
    clave = args.estacion
    claves_red = args.estaciones.split(',')
    t_busca = N * 86400.0
    arranques = esc.pases(clave, t_busca, t_busca + 4 * 86400.0, args.el_arranque)
    if len(arranques) <= k:
        return {'N': N, 'k': k, 'error': 'sin pase de arranque'}
    A1 = arranques[k]
    rng_base = np.random.SeedSequence([N, k, 20261002])
    T_ref = A1['los'] + args.ventana_red_h * 3600.0 + 3600.0
    evals = pases_eval(esc, T_ref, clave)
    if any(p is None for p in evals.values()):
        return {'N': N, 'k': k, 'error': 'sin pases de evaluacion'}
    t_fin = max(p['los'] for p in evals.values()) + 120.0
    out = {'N': N, 'k': k, 'arranque_h': A1['aos'] / 3600.0,
           'arranque_el_max': A1['el_max_deg'],
           'evals': {h: {'t_h': p['aos'] / 3600.0, 'el_max': p['el_max_deg']}
                     for h, p in evals.items()},
           'res': {}}
    # --- TLE viejo
    out['res']['TLE'] = evaluar(esc, esc.sgp4, evals, clave)
    out['tle_pos_err_km'] = float(np.linalg.norm(
        esc.estado_sgp4(A1['aos'])[:3] - esc.estado_verdad(A1['aos'])[:3]))
    # --- medidas (mismo ruido para todos los conjuntos: se generan con todos los canales)
    rng = np.random.default_rng(rng_base.spawn(1)[0])
    med_A1 = esc.medidas(A1, rng=rng, dt_s=args.dt_medida)
    # --- candidatos de la red
    claves_red = [c for c in claves_red]
    verdaderos = esc.pases_red(claves_red, A1['los'] + 300.0,
                               A1['los'] + args.ventana_red_h * 3600.0, args.el_extra)
    rapida = [A1] + verdaderos[:2]
    t_sel = time.time()
    cands = X.candidatos_planificador(
        esc, claves_red, A1['aos'] - 60.0,
        (A1['los'] - A1['aos']) / 3600.0 + args.ventana_red_h + 0.1, args.el_extra)
    cands = [c for c in cands if c['aos'] > A1['los'] and c['los'] < A1['los']
             + args.ventana_red_h * 3600.0]
    obj = evals['sig']
    prim = {'clave': clave, 'aos': A1['aos'], 'los': A1['los']}
    elegidos, tabla = X.seleccionar_pases_od(
        esc, cands, prim, 2, obj, CONJUNTOS['az/el+Dopp'])
    plan = [A1]
    for c in elegidos:
        if c is prim:
            continue
        pv = X.pase_verdadero(esc, c)
        if pv is not None:
            plan.append(pv)
    out['red_rapida'] = [(p['clave'], round(p['aos'] / 3600.0, 2), round(p['el_max_deg'], 1))
                         for p in rapida]
    out['red_planificada'] = [(p['clave'], round(p['aos'] / 3600.0, 2),
                               round(p['el_max_deg'], 1)) for p in plan]
    out['t_seleccion_s'] = time.time() - t_sel

    def medidas_de(lista):
        ms = []
        for p in lista:
            if p is A1:
                ms.append(med_A1)
            else:
                ms.append(esc.medidas(p, rng=np.random.default_rng(
                    rng_base.spawn(1)[0]), dt_s=args.dt_medida))
        return ms
    ms_rap, ms_pla = medidas_de(rapida), medidas_de(plan)

    def correr(nombre, lista_med, canales, estima_p, filtro=False, sigma_p=0.3):
        t = time.time()
        if filtro:
            r = esc.filtrar(lista_med, canales, estima_p=estima_p, sigma_p=sigma_p)
        else:
            r = esc.ajustar(lista_med, canales, prior=True, estima_p=estima_p, sigma_p=sigma_p)
        ef = od.propagar_resultado(r, t_fin, 30.0)
        d = evaluar(esc, ef, evals, clave)
        e_pos, rtn = esc.error_posicion_km(r)
        out['res'][nombre] = d
        out.setdefault('est', {})[nombre] = {
            'pos_err_km': e_pos, 'sigma_pos_km': r.sigma_pos_km(),
            'chi2_dof': float(getattr(r, 'chi2_dof', float('nan'))),
            'iter': int(getattr(r, 'iteraciones', 0)),
            'convergio': bool(getattr(r, 'convergio', True)),
            'p': float(r.x[6]) if estima_p else None, 't_s': time.time() - t}
    for cn, canales in CONJUNTOS.items():
        correr(f'OD1 {cn}', [med_A1], canales, False)
    for cn, canales in CONJUNTOS.items():
        # el factor de arrastre NO se estima: con <= 3 pases en 12 h es casi inobservable
        # y, con datos muy precisos y un modelo imperfecto, se lo lleva el error de modelo
        # (ver docs/OD.md, ablacion); la variante '+p' lo estima (sigma 30 %, acotado)
        correr(f'ODred-rapida {cn}', ms_rap, canales, False)
        correr(f'ODred-planificada {cn}', ms_pla, canales, False)
    correr('ODred-rapida+p az/el+Dopp', ms_rap, CONJUNTOS['az/el+Dopp'], True)
    correr('EKF red-rapida az/el+Dopp', ms_rap, CONJUNTOS['az/el+Dopp'], False, filtro=True)
    return out


# ---------------------------------------------------------------- tablas

def percentil(x, q):
    x = [v for v in x if v is not None and np.isfinite(v)]
    return float(np.percentile(x, q)) if x else float('nan')


def tabla(resultados, N, metrica='max'):
    filas = {}
    for r in resultados:
        if r.get('N') != N or 'res' not in r:
            continue
        for est, d in r['res'].items():
            for h, v in d.items():
                if v is not None:
                    filas.setdefault(est, {}).setdefault(h, []).append(v[metrica])
    return filas


def imprimir(resultados, edades, args):
    n_ok = sum(1 for r in resultados if 'res' in r)
    print(f'\n# Resultados de la OD: {args.sat}, estacion {args.estacion}, '
          f'{n_ok} escenarios\n')
    for N in edades:
        for metrica, nombre in (('max', 'maximo del pase'), ('aos', 'en AOS')):
            filas = tabla(resultados, N, metrica)
            if not filas:
                continue
            n = len(next(iter(filas.values()))['sig'])
            print(f'\n## TLE de {N} dias al arranque ({n} escenarios), '
                  f'desapuntamiento {nombre} (grados), mediana / p95\n')
            print('| Metodo | pase siguiente | +24 h | +48 h |')
            print('|---|---|---|---|')
            for est, hs in filas.items():
                celdas = [f'{percentil(hs.get(h, []), 50):.3f} / {percentil(hs.get(h, []), 95):.3f}'
                          for h in HORIZONTES]
                print(f'| {est} | ' + ' | '.join(celdas) + ' |')
    # fraccion bajo umbrales (todas las edades), pase siguiente
    print('\n## Fraccion de pases con el maximo bajo el umbral (pase siguiente)\n')
    print('| Metodo | ' + ' | '.join(UMBRALES) + ' |')
    print('|---|' + '---|' * len(UMBRALES))
    por = {}
    for r in resultados:
        if 'res' not in r:
            continue
        for est, d in r['res'].items():
            v = d.get('sig')
            if v is not None:
                por.setdefault(est, []).append(v['max'])
    for est, xs in por.items():
        print(f'| {est} | ' + ' | '.join(
            f'{100.0 * np.mean(np.array(xs) < u):.0f} %' for u in UMBRALES.values()) + ' |')


def consistencia(args):
    """NEES con Monte Carlo del OD por lotes y del EKF: (a) un pase y modelo
    exacto, (b) 3 pases de la red en 12 h y modelo exacto, (c) los mismos 3
    pases con la verdad REAL (J4, +25 % de arrastre, variacion de densidad)
    y el estimador estimando el arrastre."""
    esc = construir(args, args.p_cal)
    A1 = esc.pases(args.estacion, 1.5 * 86400.0, 3.0 * 86400.0, 20.0)[0]
    otros = esc.pases_red(args.estaciones.split(','), A1['los'] + 300.0,
                          A1['los'] + 12 * 3600.0, 20.0)[:2]
    casos = (('(a) 1 pase, modelo exacto', [A1], None, False),
             ('(b) 3 pases/12 h, modelo exacto', [A1] + otros, None, False),
             ('(c) 3 pases/12 h, verdad real + estimar arrastre', [A1] + otros,
              esc.modelo_verdad, True))
    print(f'\n# Consistencia: NEES medio (6 gdl) con {args.nees} corridas Monte Carlo, '
          'az/el + rango + Doppler\n')
    print('| Caso | NEES lotes | NEES EKF | intervalo 95 % de un estimador consistente |')
    print('|---|---|---|---|')
    out = {}
    for nombre, ps, mv, estp in casos:
        r = X.monte_carlo_nees(esc, ps, M.CANALES, args.nees, 7, mv, estp)
        lo, hi = r['limites_95']
        print(f"| {nombre} | {r['media_lotes']:.2f} | {r['media_ekf']:.2f} | "
              f"[{lo:.2f}, {hi:.2f}] |", flush=True)
        out[nombre] = {'lotes': r['lotes'].tolist(), 'ekf': r['ekf'].tolist()}
    if args.salida:
        with open(args.salida, 'w') as f:
            json.dump(out, f, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sat', default='ISS')
    ap.add_argument('--estacion', default='cdmx')
    ap.add_argument('--estaciones', default='cdmx,madrid,ecuador')
    ap.add_argument('--edades', default='1,2,3,5')
    ap.add_argument('--repeticiones', type=int, default=6)
    ap.add_argument('--dias', type=float, default=14.0)
    ap.add_argument('--bc', type=float, default=X.BC_FISICO_ISS)
    ap.add_argument('--p-cal', type=float, default=None)
    ap.add_argument('--el-arranque', type=float, default=20.0)
    ap.add_argument('--el-extra', type=float, default=20.0)
    ap.add_argument('--ventana-red-h', type=float, default=12.0)
    ap.add_argument('--dt-medida', type=float, default=5.0)
    ap.add_argument('--procesos', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--salida', default='')
    ap.add_argument('--nees', type=int, default=0,
                    help='si > 0: solo el Monte Carlo de consistencia con ese numero de corridas')
    args = ap.parse_args()
    if args.nees:
        return consistencia(args)
    edades = [int(e) for e in args.edades.split(',')]
    t0 = time.time()
    esc0 = construir(args, args.p_cal)
    p_cal = esc0.p_cal
    print(f'# BC efectivo calibrado contra SGP4: p = {p_cal:.4f} '
          f'(Cd A/m = {args.bc * p_cal:.5f} m2/kg); verdad con +25 %; '
          f'TLE {esc0.tle.nombre} epoca {esc0.tle.epoca:%Y-%m-%d %H:%M} UTC', flush=True)
    pares = [(N, k) for N in edades for k in range(args.repeticiones)]
    with mp.Pool(args.procesos, initializer=_init, initargs=(args, p_cal)) as pool:
        resultados = []
        for r in pool.imap_unordered(corrida, pares):
            resultados.append(r)
            print(f'  listo N={r["N"]} k={r["k"]} ({len(resultados)}/{len(pares)}) '
                  f'{r.get("error", "")}', file=sys.stderr, flush=True)
    resultados.sort(key=lambda r: (r['N'], r['k']))
    imprimir(resultados, edades, args)
    print(f'\n(tiempo total {time.time() - t0:.0f} s)')
    if args.salida:
        with open(args.salida, 'w') as f:
            json.dump({'args': vars(args), 'p_cal': p_cal, 'resultados': resultados}, f,
                      indent=1, default=float)


if __name__ == '__main__':
    main()
