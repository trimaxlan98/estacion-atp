#!/usr/bin/env python3
"""Genera las fixtures del modo mock a partir de corridas REALES de la Estacion.

    python3 station/frontend/tools/mock_fixtures.py runs/<id-del-plan> runs/<id-de-adcs>

  src/mock/fixtures/plan.json   plan.json de un `s6-plan-24h` (solo los campos que usa la vista)
  src/mock/fixtures/adcs.json   {metrics, rows}: metrics.json y telemetry.jsonl de un
                                `e1-adcs-funcube`, con 1 fila de cada `--cada` (por omision 8)

El mock sirve esto tal cual (mismo formato que CONTRATO 1-3), asi que no hay forma de que la
vista funcione con el mock y no con el backend por una diferencia de formato.
"""
import argparse
import json
import os

VENTANA = ('id', 'sat', 'norad', 'estacion', 'aos', 'los', 'duracion_s', 'el_max_deg', 'vol_gb',
           'keyhole', 't_perdido_s', 'modcod_max')
PLAN = ('resumen', 'ids', 'optimo_probado', 'algoritmo', 'valor', 'cota_sup')


def plan(src, dst):
    p = json.load(open(os.path.join(src, 'plan.json')))
    out = {k: p[k] for k in ('banda', 'horas', 'inicio_utc', 'n_ventanas', 'estaciones', 'estaciones_lista',
                             'satelites', 'disponible', 'brecha_codicioso_pct', 'brecha_max_pct',
                             'aporte_estacion') if k in p}
    out['ventanas'] = [{k: v[k] for k in VENTANA} for v in p['ventanas']]
    out['planes'] = {n: {k: x[k] for k in PLAN} for n, x in p['planes'].items()}
    json.dump(out, open(dst, 'w'), separators=(',', ':'), ensure_ascii=False)
    print(dst, os.path.getsize(dst) // 1024, 'kB')


def adcs(src, dst, cada):
    rows = [json.loads(ln) for ln in open(os.path.join(src, 'telemetry.jsonl'))]
    keep = [r for i, r in enumerate(rows) if i % cada == 0]
    if keep[-1] is not rows[-1]:
        keep.append(rows[-1])
    out = {'metrics': json.load(open(os.path.join(src, 'metrics.json'))), 'rows': keep}
    json.dump(out, open(dst, 'w'), separators=(',', ':'))
    print(dst, os.path.getsize(dst) // 1024, 'kB', len(keep), 'de', len(rows), 'filas')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('plan_run')
    ap.add_argument('adcs_run')
    ap.add_argument('--cada', type=int, default=8)
    a = ap.parse_args()
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'mock', 'fixtures')
    os.makedirs(d, exist_ok=True)
    plan(a.plan_run, os.path.join(d, 'plan.json'))
    adcs(a.adcs_run, os.path.join(d, 'adcs.json'), a.cada)
