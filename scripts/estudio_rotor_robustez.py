#!/usr/bin/env python3
"""Robustez de la curva en U de la potencia en avance del rotor (P8, barrida de k_mu y de P0).

En el modelo del rotor el mínimo de la potencia en avance (≈ 8.5 m/s) depende de dos parámetros que el propio modelo marca como
PARAMÉTRICOS o SUPUESTOS: el coeficiente de perfil k_mu = 4.65 (fuente secundaria) y la división perfil/inducida P0 = Pi (1/FoM − 1).
Este script barre ambos y dice si el mínimo sigue existiendo y dónde cae.

Hipótesis fijadas ANTES de correr:
  R1. Para todo k_mu en [0, 8] y todo factor c en [0.5, 2] sobre P0, la curva tiene un mínimo interior (no en V = 0 ni en V = 30 m/s).
  R2. La velocidad del mínimo se queda en [5, 12] m/s en todo el rango barrido (si no, el resultado de P8 no es robusto en su ubicación).
  R3. Sin término de Glauert (control negativo, induccion de hover constante) no hay mínimo interior para ningún par.

Todo es en simulación, con los supuestos declarados en rotor_fisica.py. Solo potencia analítica; no usa el simulador.
    python3 scripts/estudio_rotor_robustez.py [--salida docs/dron_resultados/estudio_rotor_robustez.json]
"""
import argparse
import json
import os
import sys

import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_dron'))

from atp_dron import rotor_fisica as RF   # noqa: E402
from atp_dron.cuerpos import cuerpo      # noqa: E402
from atp_dron.dron import PARAMS         # noqa: E402

R = PARAMS['radio_rotor_m']
FOM = 0.6
KMUS = (0.0, 2.0, 4.65, 6.0, 8.0)
CS = (0.5, 1.0, 2.0)
VS = np.linspace(0.0, 30.0, 301)


def potencia_total(k_mu, c, Vs=VS):
    """Potencia de los 4 rotores (W) en la curva de avance, con P0 multiplicado por c (c = 1 es el modelo de P8)."""
    cd = cuerpo('tierra')
    Th = 1.5 * cd['g'] / 4
    vh = float(RF.v_hover(Th, cd['rho'], R))
    Pi = RF.potencia_ideal(Th, cd['rho'], R)
    P0 = c * Pi * (1.0 / FOM - 1.0)
    from atp_dron.dron import Dron
    om = float(np.sqrt(Th / Dron('tierra').kT))            # velocidad de giro de hover, como en estudio_rotor.h4
    tot = []
    for V in Vs:
        mu = V / (om * R)
        ind = Pi * RF.razon_induccion_avance(V, vh)
        perfil = P0 * (1.0 + k_mu * mu * mu)
        par = 0.5 * PARAMS['cda_m2'] * cd['rho'] * V ** 3 / 4.0
        tot.append(4.0 * (ind + perfil + par))
    return np.array(tot)


def sin_glauert(k_mu, c, Vs=VS):
    """Control negativo: la inducida queda en su valor de hover (sin el termino de avance)."""
    cd = cuerpo('tierra')
    Th = 1.5 * cd['g'] / 4
    Pi = RF.potencia_ideal(Th, cd['rho'], R)
    P0 = c * Pi * (1.0 / FOM - 1.0)
    from atp_dron.dron import Dron
    om = float(np.sqrt(Th / Dron('tierra').kT))
    out = []
    for V in Vs:
        mu = V / (om * R)
        out.append(4.0 * (Pi + P0 * (1.0 + k_mu * mu * mu) + 0.5 * PARAMS['cda_m2'] * cd['rho'] * V ** 3 / 4.0))
    return np.array(out)


def interior(tot):
    i = int(np.argmin(tot))
    return 0 < i < len(tot) - 1, i


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--salida', default=os.path.join(RAIZ, 'docs', 'dron_resultados', 'estudio_rotor_robustez.json'))
    a = ap.parse_args(argv)
    filas = []
    for k in KMUS:
        for c in CS:
            tot = potencia_total(k, c)
            ok, i = interior(tot)
            ng = sin_glauert(k, c)
            ok_ng, _ = interior(ng)
            filas.append({'k_mu': k, 'c_P0': c, 'minimo_interior': bool(ok), 'V_minimo_m_s': round(float(VS[i]), 3),
                          'P_minimo_W': round(float(tot[i]), 2), 'P_V0_W': round(float(tot[0]), 2),
                          'control_negativo_minimo_interior': bool(ok_ng)})
            print(json.dumps(filas[-1]), flush=True)
    vs = [f['V_minimo_m_s'] for f in filas if f['minimo_interior']]
    ver = {
        'R1_minimo_interior_en_todo_el_rango': all(f['minimo_interior'] for f in filas),
        'R2_V_minimo_en_5_12': all(5.0 <= v <= 12.0 for v in vs) and len(vs) == len(filas),
        'R3_control_negativo_sin_minimo': not any(f['control_negativo_minimo_interior'] for f in filas),
        'V_minimo_rango_m_s': [min(vs), max(vs)] if vs else None,
    }
    res = {'supuestos': 'rotor 0.127 m, FoM 0.6, Tierra, T = m g / 4; potencia analitica (rotor_fisica.potencia_avance); en simulacion',
           'filas': filas, 'criterios': ver}
    os.makedirs(os.path.dirname(a.salida), exist_ok=True)
    with open(a.salida, 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True)
    print('CRITERIOS', json.dumps(ver))
    return 0


if __name__ == '__main__':
    sys.exit(main())
