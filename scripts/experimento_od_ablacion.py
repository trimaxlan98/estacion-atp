#!/usr/bin/env python3
"""Ablacion de la estimacion del factor de arrastre y del peso de rango/Doppler
en la OD de 3 pases de la red (docs/OD.md, 3.3): 12 escenarios (ISS, CDMX,
edades 2 y 3 dias), regla rapida, pase siguiente y +24 h. Uso:
  python3 scripts/experimento_od_ablacion.py
Tarda ~30 min con 6 procesos."""
import argparse
import collections
import multiprocessing as mp
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'src', 'atp_ros'))
import experimento_od as E   # noqa: E402
from atp_ros import od, od_experimento as X, medidas as M   # noqa: E402

args = argparse.Namespace(
    sat='ISS', estaciones='cdmx,madrid,ecuador', dias=14.0, bc=X.BC_FISICO_ISS,
    estacion='cdmx', el_arranque=20.0, el_extra=20.0, ventana_red_h=12.0, dt_medida=5.0)
G = {}
def init():
    G['esc'] = E.construir(args, 0.406)

VARS = {
  'V0 sp.5': dict(estp=True, sp=0.5, k=1.0),
  'V1 sp.3': dict(estp=True, sp=0.3, k=1.0),
  'V2 p fijo': dict(estp=False, sp=0.5, k=1.0),
  'V3 sp.5 x3': dict(estp=True, sp=0.5, k=3.0),
  'V4 sp.3 x3': dict(estp=True, sp=0.3, k=3.0),
  'V5 p fijo x3': dict(estp=False, sp=0.5, k=3.0),
  'V6 sp.3 x10': dict(estp=True, sp=0.3, k=10.0),
}
CAN = {'az/el+Dopp': ('az','el','rr'), 'az/el+rng+Dopp': ('az','el','rng','rr'), 'az/el': ('az','el')}

def job(par):
    N, k = par
    esc = G['esc']
    arr = esc.pases('cdmx', N*86400.0, N*86400.0+4*86400.0, 20.0)
    A1 = arr[k]
    T_ref = A1['los'] + 12*3600.0 + 3600.0
    evals = E.pases_eval(esc, T_ref, 'cdmx')
    t_fin = max(p['los'] for p in evals.values()) + 120.0
    rng_base = np.random.SeedSequence([N, k, 20261002])
    rng = np.random.default_rng(rng_base.spawn(1)[0])
    med_A1 = esc.medidas(A1, rng=rng, dt_s=5.0)
    ver = esc.pases_red(['cdmx','madrid','ecuador'], A1['los']+300.0, A1['los']+12*3600.0, 20.0)
    rap = [A1] + ver[:2]
    ms = [med_A1] + [esc.medidas(p, rng=np.random.default_rng(rng_base.spawn(1)[0]), dt_s=5.0) for p in rap[1:]]
    out = {}
    for vn, v in VARS.items():
        for cn, can in CAN.items():
            sg = {c: s*(v['k'] if c in ('rng','rr') else 1.0) for c, s in M.SIGMAS_DEFECTO.items()}
            r = esc.ajustar(ms, can, prior=True, estima_p=v['estp'], sigma_p=v['sp'], sigmas=sg)
            ef = od.propagar_resultado(r, t_fin, 30.0)
            e = {h: esc.error_pase(ef, p, 'cdmx')['max'] for h, p in evals.items() if p}
            out[(vn, cn)] = (e, float(r.x[6]) if v['estp'] else None, r.chi2_dof)
    return (N, k, out)

if __name__ == '__main__':
    pares = [(N, k) for N in (2, 3) for k in range(6)]
    with mp.Pool(6, initializer=init) as pool:
        res = pool.map(job, pares)
    agg = collections.defaultdict(list)
    for N, k, out in res:
        for key, (e, p, c) in out.items():
            agg[key].append((e['sig'], e['+24h'], p, c))
    print('variante / canales : mediana,p95 error pase sig | +24h | p mediano, chi2 mediano')
    for key, v in agg.items():
        a = np.array([[x[0], x[1]] for x in v])
        ps = [x[2] for x in v if x[2] is not None]
        print(key, 'sig %.3f/%.3f  +24h %.3f/%.3f' % (np.median(a[:,0]), np.percentile(a[:,0],95), np.median(a[:,1]), np.percentile(a[:,1],95)),
              'p~%s' % (round(float(np.median(ps)),2) if ps else '-'), 'chi2~%.2f' % np.median([x[3] for x in v]))
