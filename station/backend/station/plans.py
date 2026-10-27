"""Resumen de una corrida `kind: plan` (CONTRATO 3.2) a partir de su plan.json.

`scripts/planificar.py` escribe `plan.json` (ventanas, planes, brecha, aporte de
estaciones); el backend deriva de el las `metrics` de la corrida: el bloque
`planes.optimo.resumen` (o el del codicioso por peso si no hay optimo) mas la
brecha del codicioso y un poco de contexto. Todo sale del plan, nada se recalcula.
"""


def plan_metrics(plan):
    """metrics de una corrida de plan. `plan` es el dict de plan.json."""
    planes = plan['planes']
    nombre = 'optimo' if 'optimo' in planes else 'codicioso_peso'
    ref = planes[nombre]
    out = {'kind': 'plan', 'plan': nombre, 'algoritmo': ref['algoritmo'],
           'optimo_probado': bool(ref['optimo_probado'])}
    out.update(ref['resumen'])                  # contactos, minutos, gb, gb_shannon, por_*
    cod = planes.get('codicioso_peso')
    if cod:
        out['codicioso'] = {k: cod['resumen'][k] for k in ('contactos', 'minutos', 'gb')}
    if 'brecha_codicioso_pct' in plan:
        out['brecha_codicioso_pct'] = plan['brecha_codicioso_pct']
    out['ventanas'] = plan['n_ventanas']
    out['disponible'] = {k: plan['disponible'][k] for k in ('contactos', 'minutos', 'gb')}
    out['horas'] = plan['horas']
    out['inicio_utc'] = plan['inicio_utc']
    out['banda'] = plan['banda']
    out['estaciones'] = list(plan.get('estaciones_lista') or plan['estaciones'])
    out['satelites'] = [s['nombre'] for s in plan['satelites']]
    out['aporte_estacion'] = {
        est: {k: a[k] for k in ('gb_con', 'gb_sin', 'delta_gb', 'delta_pct',
                                'contactos_con', 'contactos_sin')}
        for est, a in plan.get('aporte_estacion', {}).items()}
    return out
