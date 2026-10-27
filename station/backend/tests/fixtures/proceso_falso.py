"""Proceso de prueba para `kind: proceso` (sin ROS): escribe una corrida con el contrato `red`
(escena.json, red.jsonl, eventos.jsonl) y `metricas.json`, como lo haria `atp_demo.cli`.

    python3 proceso_falso.py --salida <runs> --id <run_id> --n_sats 3 --modo ok
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--salida', required=True)
    ap.add_argument('--id', required=True)
    ap.add_argument('--n_sats', type=int, default=3)
    ap.add_argument('--dur_s', type=float, default=10.0)
    ap.add_argument('--modo', default='ok')
    ap.add_argument('--nombre', default='')
    a = ap.parse_args()
    d = Path(a.salida) / a.id
    d.mkdir(parents=True, exist_ok=True)
    print('dominio', os.environ.get('ROS_DOMAIN_ID'), 'n_sats', a.n_sats, flush=True)
    if a.modo == 'falla':
        sys.exit(2)
    if a.modo == 'dormir':
        time.sleep(120)
    (d / 'dominio.txt').write_text(os.environ.get('ROS_DOMAIN_ID', ''))
    sats = [{'id': f'S{i:02d}', 'nombre': f'SAT-{i}'} for i in range(a.n_sats)]
    escena = {'version': 1, 'inicio_utc': '2026-10-05T12:00:00Z', 'dt_s': 1.0,
              'estaciones': [{'id': 'cdmx', 'nombre': 'CDMX', 'lat_deg': 19.4, 'lon_deg': -99.1, 'alt_m': 2240,
                              'antenas': [{'id': 'A1', 'banda': 'X', 'pos_enu_m': [0, 0, 0]}]}],
              'satelites': sats, 'parametros': {'n_sats': a.n_sats}}
    (d / 'escena.json').write_text(json.dumps(escena))
    n = int(a.dur_s) + 1
    with open(d / 'red.jsonl', 'w') as f:
        for k in range(n):
            geo = [[s['id'], (350.0 + 2 * k + 40 * i) % 360, 30.0 + i, 1500.0 + k, -1.0] for i, s in enumerate(sats)]
            f.write(json.dumps({'t': float(k), 'geo': {'cdmx': geo},
                                'antenas': {'cdmx/A1': {'sat': 'S00', 'estado': 'seguimiento',
                                                        'ref': [350.0 + 2 * k, 30.0],
                                                        'med': [350.0 + 2 * k, 30.0]}}}) + '\n')
    (d / 'eventos.jsonl').write_text(json.dumps(
        {'t': 3.0, 'antena': 'cdmx/A1', 'de': 'S00', 'a': 'S01', 'tipo': 'break-before-make', 'hueco_s': 1.5}) + '\n')
    if a.modo == 'sinhecho':
        return
    (d / 'metricas.json').write_text(json.dumps({'n_sats': a.n_sats, 'handovers': 1, 'veredicto': 'OK'}))
    print('listo', flush=True)


if __name__ == '__main__':
    main()
