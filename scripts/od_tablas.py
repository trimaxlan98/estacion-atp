#!/usr/bin/env python3
"""Tablas en Markdown a partir de docs/od_resultados_iss.json (salida de
scripts/experimento_od.py). Uso: python3 scripts/od_tablas.py [json]"""
import json
import sys

import numpy as np

FILAS = [
    ('TLE viejo (SGP4)', 'TLE'),
    ('OD 1 pase: az/el', 'OD1 az/el'),
    ('OD 1 pase: az/el + Doppler', 'OD1 az/el+Dopp'),
    ('OD 1 pase: az/el + rango + Doppler', 'OD1 az/el+rng+Dopp'),
    ('OD 3 pases de la red (regla rápida): az/el', 'ODred-rapida az/el'),
    ('OD 3 pases de la red (regla rápida): az/el + Doppler', 'ODred-rapida az/el+Dopp'),
    ('OD 3 pases de la red (planificador): az/el', 'ODred-planificada az/el'),
    ('OD 3 pases de la red (planificador): az/el + Doppler', 'ODred-planificada az/el+Dopp'),
    ('OD 3 pases, rápida, estimando el arrastre (az/el + Doppler)',
     'ODred-rapida+p az/el+Dopp'),
    ('EKF 3 pases, rápida (az/el + Doppler)', 'EKF red-rapida az/el+Dopp'),
]
HOR = ('sig', '+24h', '+48h')
UMB = {'Ka 3 dB (0.117°)': 0.5 * 0.233, 'Ka lock −10 dB (0.213°)': 0.913 * 0.233,
       'S 3 dB (1.59°)': 0.5 * 3.18}


def main():
    ruta = sys.argv[1] if len(sys.argv) > 1 else 'docs/od_resultados_iss.json'
    d = json.load(open(ruta))
    rs = [r for r in d['resultados'] if 'res' in r]
    edades = sorted({r['N'] for r in rs})
    print(f'<!-- generado por scripts/od_tablas.py desde {ruta}: {len(rs)} escenarios -->\n')
    for N in edades:
        sub = [r for r in rs if r['N'] == N]
        tle_err = np.median([r['tle_pos_err_km'] for r in sub])
        edad = [r['arranque_h'] / 24.0 for r in sub]
        print(f'**Arranque a partir de {N} día(s) de la época del TLE** ({len(sub)} escenarios; '
              f'edad real {min(edad):.1f}-{max(edad):.1f} días; error de posición del TLE en el '
              f'arranque, mediana {tle_err:.1f} km). Desapuntamiento **máximo del pase** en '
              f'grados, mediana (p95):\n')
        print('| Método | pase siguiente (13 h) | +24 h | +48 h | en AOS, pase siguiente |')
        print('|---|---|---|---|---|')
        for nombre, clave in FILAS:
            celdas = []
            for h in HOR:
                v = [r['res'][clave][h]['max'] for r in sub
                     if r['res'].get(clave, {}).get(h)]
                celdas.append(f'{np.median(v):.3f} ({np.percentile(v, 95):.3f})')
            v = [r['res'][clave]['sig']['aos'] for r in sub if r['res'].get(clave, {}).get('sig')]
            celdas.append(f'{np.median(v):.3f} ({np.percentile(v, 95):.3f})')
            print(f'| {nombre} | ' + ' | '.join(celdas) + ' |')
        print()
    dif = sum(1 for r in rs if r['red_rapida'] != r['red_planificada'])
    print(f'El planificador de información eligió pases distintos de la regla rápida en {dif} '
          f'de {len(rs)} escenarios.\n')
    print('**Fracción de pases (todas las edades, pase siguiente) con el máximo bajo el umbral:**\n')
    print('| Método | ' + ' | '.join(UMB) + ' |')
    print('|---|' + '---|' * len(UMB))
    for nombre, clave in FILAS:
        v = np.array([r['res'][clave]['sig']['max'] for r in rs if r['res'].get(clave, {}).get('sig')])
        print(f'| {nombre} | ' + ' | '.join(f'{100 * np.mean(v < u):.0f} %' for u in UMB.values())
              + ' |')
    print()
    print('**Fracción de pases con el error en AOS (donde se adquiere) bajo el umbral, todas las edades:**\n')
    print('| Método | < 0.117° (Ka 3 dB): pase siguiente / +24 h / +48 h | < 0.213° (captura del conical scan, Ka): pase siguiente / +24 h / +48 h |')
    print('|---|---|---|')
    for nombre, clave in FILAS:
        col = []
        for umbral in (0.5 * 0.233, 0.913 * 0.233):
            celdas = []
            for h in HOR:
                v = np.array([r['res'][clave][h]['aos'] for r in rs if r['res'].get(clave, {}).get(h)])
                celdas.append(f'{100 * np.mean(v < umbral):.0f} %')
            col.append(' / '.join(celdas))
        print(f'| {nombre} | ' + ' | '.join(col) + ' |')


if __name__ == '__main__':
    main()
