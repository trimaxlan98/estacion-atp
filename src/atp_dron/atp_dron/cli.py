"""Corrida del dron sin ROS: lee una escena YAML (kind: proceso, args:), simula y escribe telemetry.jsonl y metrics.json.

    python3 -m atp_dron.cli --escena experiments/dron-hover-tierra.yaml --salida /tmp/dron --id d1
    python3 -m atp_dron.cli --cuerpo=titan --ruta='[[0,0,10],[40,0,10]]' --rafaga_sigma=1.0

Un argumento suelto `--k=v` pisa el de la escena.
"""
import argparse
import json
import os
import sys

from atp_dron.lazo import DEFECTOS, correr


def _valor(s):
    try:
        return json.loads(s)
    except ValueError:
        return s


def main(argv=None):
    ap = argparse.ArgumentParser(allow_abbrev=False, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--escena')
    ap.add_argument('--salida', default='/tmp/dron')
    ap.add_argument('--id', default='dron')
    a, resto = ap.parse_known_args(argv)
    cfg = {}
    if a.escena:
        import yaml
        with open(a.escena) as f:
            cfg = dict((yaml.safe_load(f).get('args')) or {})
    i = 0
    while i < len(resto):
        r = resto[i]
        i += 1
        if not r.startswith('--'):
            ap.error(f'argumento no reconocido: {r}')
        if '=' in r:
            k, v = r[2:].split('=', 1)
        elif i < len(resto):
            k, v = r[2:], resto[i]
            i += 1
        else:
            ap.error(f'falta el valor de {r}')
        if k not in DEFECTOS:
            ap.error(f'argumento desconocido: --{k} (hay: {", ".join(sorted(DEFECTOS))})')
        cfg[k] = _valor(v)
    met, _ = correr(cfg, os.path.join(a.salida, a.id))
    print('RESUMEN ' + json.dumps(met))
    return 0


if __name__ == '__main__':
    sys.exit(main())
