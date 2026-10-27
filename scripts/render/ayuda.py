#!/usr/bin/env python3
"""Ayudas en Python de los envoltorios `scripts/render/*.sh` (solo biblioteca estandar + numpy del modelo).

  ayuda.py args-json k1 v1 k2 v2 ...    -> JSON {k: valor} (numeros y booleanos como tales)
  ayuda.py demo-params <run_dir>        -> JSON con el_max, mask, band, h_km, dur_s del pase de una corrida de lazo
  ayuda.py red-info <run_dir>           -> JSON con t_ini, t_fin, estaciones, satelites, antenas de una corrida de red
  ayuda.py tipo-corrida <run_dir>       -> una palabra: red | lazo | campana | adcs | mision | plan | desconocida
"""
import json
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def valor(s):
    try:
        return json.loads(s)
    except ValueError:
        return s


def leer(ruta, defecto=None):
    try:
        with open(ruta, encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return defecto


def filas(ruta, campos):
    """Genera dicts con solo `campos` de un .jsonl, saltando lineas rotas (la ultima puede estar cortada)."""
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            try:
                d = json.loads(linea)
            except ValueError:
                continue
            yield {k: d[k] for k in campos if k in d}


def tipo_corrida(run):
    e = os.path.exists
    j = lambda n: e(os.path.join(run, n))   # noqa: E731
    if j('escena.json') and j('red.jsonl'):
        return 'red'
    if j('informe.json'):
        return 'mision'
    if j('campaign.json'):
        return 'campana'
    if j('plan.json'):
        return 'plan'
    if j('telemetry.jsonl'):
        m = leer(os.path.join(run, 'metrics.json'), {})
        return 'adcs' if m.get('kind') == 'adcs' else 'lazo'
    return 'desconocida'


def demo_params(run):
    """Geometria del pase de la corrida para repetirlo en Gazebo (perfil sintetico de atp_gz)."""
    p = leer(os.path.join(run, 'params.json'), {}) or {}
    a = p.get('args', {}) or {}
    m = leer(os.path.join(run, 'metrics.json'), {}) or {}
    el_max = a.get('el_max')
    mask = a.get('mask')
    ref = os.path.join(run, 'telemetry.jsonl')
    if (el_max is None or mask is None) and os.path.isfile(ref):
        els = [f['el'] for f in filas(ref, ('el',)) if 'el' in f]
        refs = [f['ref_el'] for f in filas(ref, ('ref_el',)) if 'ref_el' in f]
        if el_max is None and els:
            el_max = round(max(els), 1)
        if mask is None and refs:
            mask = round(min(refs), 1)
    band = a.get('band') or m.get('link', {}).get('band') or 'S'
    h_km = a.get('h_km', 550.0)
    sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_ros'))
    dur = 140.0
    try:
        from atp_ros import model
        if el_max is not None and mask is not None:
            dur = model.duracion_pase(float(h_km), float(el_max), float(mask))
    except Exception:
        pass
    return {'el_max': el_max, 'mask': mask, 'band': band, 'h_km': h_km, 'dur_s': round(dur, 1),
            'perfil': a.get('profile', 'pass')}


def red_info(run):
    e = leer(os.path.join(run, 'escena.json'), {}) or {}
    t_ini = t_fin = None
    ruta = os.path.join(run, 'red.jsonl')
    if os.path.isfile(ruta):
        for f in filas(ruta, ('t',)):
            if 't' in f:
                if t_ini is None:
                    t_ini = f['t']
                t_fin = f['t']
    return {'t_ini': t_ini, 't_fin': t_fin,
            'estaciones': [s['id'] for s in e.get('estaciones', [])],
            'satelites': len(e.get('satelites', [])),
            'antenas': {s['id']: len(s.get('antenas', [])) for s in e.get('estaciones', [])}}


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cmd = argv[1]
    if cmd == 'args-json':
        kv = argv[2:]
        print(json.dumps({kv[i]: valor(kv[i + 1]) for i in range(0, len(kv) - 1, 2)}, ensure_ascii=False))
    elif cmd == 'demo-params':
        d = demo_params(argv[2])
        if '--sh' in argv[3:]:      # para `read`: el_max mask band h_km dur_s (vacio = '-')
            print(*[('-' if d[k] is None else d[k]) for k in ('el_max', 'mask', 'band', 'h_km', 'dur_s')])
        else:
            print(json.dumps(d))
    elif cmd == 'red-info':
        print(json.dumps(red_info(argv[2])))
    elif cmd == 'tipo-corrida':
        print(tipo_corrida(argv[2]))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
