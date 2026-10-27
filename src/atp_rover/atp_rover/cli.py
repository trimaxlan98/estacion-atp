"""Corrida del rover sin ROS (CONTRATO sec. 18): lee una escena, simula y escribe telemetry.jsonl y metrics.json.

    python3 -m atp_rover.cli --escena experiments/rover-luna-rocoso.yaml --salida /tmp/rover --id r1
    python3 -m atp_rover.cli --planeta=marte --terreno=dunas --semilla=3 --ruta='[[20,0],[20,15]]' --duracion_s=90

Un argumento suelto `--k=v` pisa el de la escena. Misma escena, misma semilla -> mismos bytes (los archivos no llevan
sello de reloj de pared). El seguidor de metas de aqui es MINIMO (rumbo proporcional): sirve para ejercitar el modelo;
el navegador de verdad (A*, pure pursuit, traccion, seguridad; R2) se elige con `--navegacion=completa` (lazo.py).
La suspension se elige con `--suspension=simple|rocker_bogie` (R6; defecto simple = el modelo de R1, mismos bytes).
"""
import argparse
import json
import os
import sys

import numpy as np

from atp_rover.planetas import planeta
from atp_rover.rover import DT, Rover
from atp_rover.terreno import Terreno

DEFECTOS = {'planeta': 'luna', 'terreno': 'llano', 'semilla': 0, 'ruta': [], 'duracion_s': 60.0,
            'rumbo_deg': 0.0, 'x0': 0.0, 'y0': 0.0, 'v_max': 0.5, 'radio_meta_m': 0.5,
            'terreno_params': {}, 'rover': {}, 'navegacion': 'basico', 'suspension': 'simple'}


class SeguidorBasico:
    """Lleva el rover por la lista de metas `ruta` [[x, y], ...] con rumbo proporcional."""

    def __init__(self, ruta, v_max=0.5, radio=0.5, kw=1.5, w_max=0.6):
        self.ruta = [tuple(map(float, m)) for m in ruta]
        self.i = 0
        self.v_max, self.radio, self.kw, self.w_max = v_max, radio, kw, w_max

    @property
    def terminado(self):
        return self.i >= len(self.ruta)

    def mando(self, e):
        if self.terminado:
            return 0.0, 0.0
        mx, my = self.ruta[self.i]
        dx, dy = mx - e['x'], my - e['y']
        if np.hypot(dx, dy) < self.radio:
            self.i += 1
            return self.mando(e)
        err = (np.degrees(np.arctan2(dy, dx)) - e['rumbo_deg'] + 180.0) % 360.0 - 180.0
        w = float(np.clip(self.kw * np.radians(err), -self.w_max, self.w_max))
        v = self.v_max * max(0.0, np.cos(np.radians(err))) if abs(err) < 75.0 else 0.0
        return v, w


def construir(cfg):
    """(Rover, Terreno, planeta) a partir de la configuracion de una escena."""
    pl = planeta(cfg['planeta'])
    ter = Terreno(cfg['terreno'], cfg['semilla'], pl['suelo'], **(cfg.get('terreno_params') or {}))
    pr = dict(cfg.get('rover') or {})
    if cfg.get('suspension', 'simple') != 'simple':      # R6: 'simple' (defecto) no toca nada: mismos bytes que antes
        pr['suspension'] = cfg['suspension']
    rov = Rover(pl, ter, pr, x=cfg['x0'], y=cfg['y0'], rumbo_deg=cfg['rumbo_deg'])
    return rov, ter, pl


def correr(cfg, carpeta=None):
    """Simula la escena (lazo en `atp_rover.lazo`; `navegacion`: basico | completa). Devuelve (metricas, filas)."""
    from atp_rover.lazo import correr as _correr
    return _correr(cfg, carpeta)


def cargar_escena(ruta):
    import yaml
    with open(ruta) as f:
        y = yaml.safe_load(f)
    return dict(y.get('args') or {})


def _valor(s):
    try:
        return json.loads(s)
    except ValueError:
        return s


def main(argv=None):
    ap = argparse.ArgumentParser(allow_abbrev=False, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--escena', help='YAML de experimentos/ con kind: rover')
    ap.add_argument('--salida', default='/tmp/rover')
    ap.add_argument('--id', default='rover')
    a, resto = ap.parse_known_args(argv)
    cfg = cargar_escena(a.escena) if a.escena else {}
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
    met, filas = correr(cfg, os.path.join(a.salida, a.id))
    print('RESUMEN ' + json.dumps(met))
    return 0


if __name__ == '__main__':
    sys.exit(main())
