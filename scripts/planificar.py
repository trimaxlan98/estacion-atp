#!/usr/bin/env python3
"""Planifica los contactos de una red y guarda la corrida en runs/<id>/.

  runs/<run_id>/plan.json     ventanas, planes (codicioso x3, optimo), brecha
                              y, con --sin, el aporte de cada estacion
  runs/<run_id>/resumen.txt   las tablas del estudio en texto

run_id = YYYYmmdd-HHMMSS-<6 hex> (CONTRATO 1). Sin ROS: solo numpy + sgp4.
Con --inicio fijo el plan es repetible byte a byte.

Ejemplo (estudio de caso de docs/PLANIFICADOR.md, dentro del contenedor):
  ./run.sh python3 scripts/planificar.py \\
      --sats "ISS,NOAA 15,NOAA 18,NOAA 19,NOAA 20,NOAA 21,METEOR-M2 3,METEOR-M2 4" \\
      --estaciones cdmx,tromso,ecuador,madrid --inicio 2026-10-02T12:00:00 \\
      --horas 24 --sin tromso
"""
import argparse
import datetime as dt
import json
import os
import secrets
import sys
import time

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
try:
    from atp_ros import orbit, planificador
except ImportError:                  # fuera de colcon: usa el arbol de fuentes
    sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_ros'))
    from atp_ros import orbit, planificador


def _lista(texto):
    return [x.strip() for x in str(texto).split(',') if x.strip()]


def _clave_estacion(todas, q):
    for c, e in todas.items():
        if q.lower() in (c.lower(), e.nombre.lower()):
            return c
    raise SystemExit(f'estacion {q!r} no existe; hay: {", ".join(todas)}')


def tabla(titulo, filas, cols):
    """Texto de una tabla simple; `filas` = lista de listas de str."""
    anchos = [max(len(str(x)) for x in [c] + [f[i] for f in filas])
              for i, c in enumerate(cols)]
    lin = lambda f: '  '.join(str(x).rjust(a) if i else str(x).ljust(a)    # noqa: E731
                              for i, (x, a) in enumerate(zip(f, anchos)))
    return '\n'.join([titulo, lin(cols), '  '.join('-' * a for a in anchos)]
                     + [lin(f) for f in filas]) + '\n'


def texto_resumen(res):
    out = [f"Plan de contactos  inicio {res['inicio_utc']}  horizonte "
           f"{res['horas']:g} h  banda {res['banda']}",
           f"Estaciones: {', '.join(res['estaciones'])}   Satelites: "
           f"{', '.join(s['nombre'] for s in res['satelites'])}",
           f"Ventanas candidatas: {res['n_ventanas']}", '']
    filas = []
    for k, p in res['planes'].items():
        r = p['resumen']
        filas.append([k, r['contactos'], f"{r['minutos']:.1f}", f"{r['gb']:.2f}",
                      f"{p['valor']:.2f}", 'si' if p['optimo_probado'] else 'no'])
    d = res['disponible']
    filas.insert(0, ['(todas las ventanas)', d['contactos'], f"{d['minutos']:.1f}",
                     f"{d['gb']:.2f}", '', ''])
    out.append(tabla('Planes', filas, ['plan', 'contactos', 'minutos', 'GB',
                                        'valor', 'optimo']))
    if 'brecha_codicioso_pct' in res:
        out.append(f"Brecha del codicioso (peso) frente al optimo: "
                   f"{res['brecha_codicioso_pct']:.2f} %  (cota maxima "
                   f"{res['brecha_max_pct']:.2f} %)\n")
    mejor = res['planes'].get('optimo') or res['planes']['codicioso_peso']
    r = mejor['resumen']
    out.append(tabla(f"GB por satelite ({mejor['algoritmo']})",
                     [[s, x['contactos'], f"{x['minutos']:.1f}", f"{x['gb']:.2f}"]
                      for s, x in r['por_satelite'].items()],
                     ['satelite', 'contactos', 'minutos', 'GB']))
    out.append(tabla(f"GB por estacion ({mejor['algoritmo']})",
                     [[s, x['contactos'], f"{x['minutos']:.1f}", f"{x['gb']:.2f}"]
                      for s, x in r['por_estacion'].items()],
                     ['estacion', 'contactos', 'minutos', 'GB']))
    for est, e in res.get('aporte_estacion', {}).items():
        out.append(f"Aporte de {est}: con {e['gb_con']:.2f} GB / {e['contactos_con']} "
                   f"contactos; sin {e['gb_sin']:.2f} GB / {e['contactos_sin']} "
                   f"contactos; ganancia {e['delta_gb']:+.2f} GB ({e['delta_pct']:+.1f} %)\n")
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sats', default='ISS,NOAA 19,METEOR-M2 3',
                    help="nombres o NORAD separados por coma ('' = catalogo entero)")
    ap.add_argument('--estaciones', default='cdmx,tromso',
                    help="claves de data/estaciones.yaml ('' = todas)")
    ap.add_argument('--inicio', default='',
                    help='UTC ISO 8601; vacio = ahora (no repetible)')
    ap.add_argument('--horas', type=float, default=24.0)
    ap.add_argument('--banda', default='X', choices=sorted(planificador.model.BANDAS))
    ap.add_argument('--eirp', type=float, default=12.0, help='EIRP del satelite (dBW)')
    ap.add_argument('--rs', type=float, default=10e6, help='tasa de simbolos (baud)')
    ap.add_argument('--margen-acm', type=float, default=1.0, help='dB sobre ETSI')
    ap.add_argument('--vel-az', type=float, default=6.0)
    ap.add_argument('--vel-el', type=float, default=3.0)
    ap.add_argument('--asent', type=float, default=5.0, help='asentamiento (s)')
    ap.add_argument('--keyhole', default='marcar',
                    choices=['marcar', 'recortar', 'descartar'])
    ap.add_argument('--el-min', type=float, default=0.0)
    ap.add_argument('--sep-sat', type=float, default=0.0)
    ap.add_argument('--prioridades', default='', help='"NOAA 19=2,ISS=1"')
    ap.add_argument('--metodo', default='auto', choices=['auto', 'bb', 'milp'])
    ap.add_argument('--sin', default='',
                    help='estaciones a quitar, de a una, para medir su aporte')
    ap.add_argument('--tle-dir', default=None)
    ap.add_argument('--salida', default=os.path.join(RAIZ, 'runs'))
    ap.add_argument('--id', default='', help='run_id fijo (por defecto, uno nuevo)')
    a = ap.parse_args(argv)

    todas, _ = orbit.cargar_estaciones()
    claves = [_clave_estacion(todas, q) for q in _lista(a.estaciones)] or list(todas)
    estaciones = {c: todas[c] for c in claves}
    cat = orbit.cargar_catalogo(a.tle_dir)
    nombres = _lista(a.sats)
    tles = [orbit.buscar_tle(cat, s) for s in nombres] if nombres else cat
    t0 = (orbit.a_utc(a.inicio) if a.inicio
          else dt.datetime.now(orbit.UTC).replace(microsecond=0))
    prior = {}
    for par in _lista(a.prioridades):
        n, _, v = par.rpartition('=')
        prior[n.strip()] = float(v)
    enlace = planificador.PerfilEnlace(banda=a.banda, eirp_dbw=a.eirp, rs_baud=a.rs,
                                       margen_acm_db=a.margen_acm)
    rotor = planificador.Rotor(a.vel_az, a.vel_el, a.asent, a.keyhole)

    reloj = time.perf_counter()
    vs = planificador.construir_ventanas(tles, estaciones, t0, a.horas, enlace,
                                         rotor, None, prior, a.el_min)
    kw = dict(enlace=enlace, rotor=rotor, prioridades=prior, el_min_deg=a.el_min,
              sep_sat_s=a.sep_sat, metodo=a.metodo)
    res = planificador.calcular_plan(tles, estaciones, t0, a.horas, ventanas=vs, **kw)
    ref = res['planes'].get('optimo') or res['planes']['codicioso_peso']

    aporte = {}
    for q in _lista(a.sin):
        c = _clave_estacion(todas, q)
        if c not in estaciones:
            raise SystemExit(f'--sin {q}: no esta en --estaciones')
        resto = {k: e for k, e in estaciones.items() if k != c}
        sub = planificador.subred(vs, estaciones=resto)
        orig = [v.id for v in vs if v.estacion in resto]     # subred renumera: id nuevo -> id en `ventanas`
        rs = planificador.calcular_plan(tles, resto, t0, a.horas, ventanas=sub, **kw)
        r2 = rs['planes'].get('optimo') or rs['planes']['codicioso_peso']
        gb_c, gb_s = ref['resumen']['gb'], r2['resumen']['gb']
        aporte[c] = {
            'gb_con': gb_c, 'gb_sin': gb_s, 'delta_gb': round(gb_c - gb_s, 3),
            'delta_pct': round(100.0 * (gb_c - gb_s) / gb_s, 2) if gb_s else None,
            'contactos_con': ref['resumen']['contactos'],
            'contactos_sin': r2['resumen']['contactos'],
            'minutos_con': ref['resumen']['minutos'],
            'minutos_sin': r2['resumen']['minutos'],
            'valor_con': ref['valor'], 'valor_sin': r2['valor'],
            'optimo_probado_sin': r2['optimo_probado'],
            'por_satelite_sin': r2['resumen']['por_satelite'],
            'brecha_codicioso_sin_pct': rs.get('brecha_codicioso_pct'),
            # el plan sin la estacion, con los ids de `ventanas` (la Estacion lo dibuja en el Gantt)
            'ids_sin': [orig[i] for i in r2['ids']],
        }
    if aporte:
        res['aporte_estacion'] = aporte
    res['estaciones_lista'] = claves

    run_id = a.id or (dt.datetime.now(orbit.UTC).strftime('%Y%m%d-%H%M%S-')
                      + secrets.token_hex(3))
    carpeta = os.path.join(a.salida, run_id)
    os.makedirs(carpeta, exist_ok=True)
    with open(os.path.join(carpeta, 'plan.json'), 'w') as f:
        json.dump(res, f, sort_keys=True)
    txt = texto_resumen(res)
    with open(os.path.join(carpeta, 'resumen.txt'), 'w') as f:
        f.write(txt)
    print(txt)
    print(f'-> {carpeta}/plan.json, resumen.txt  ({time.perf_counter() - reloj:.0f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
