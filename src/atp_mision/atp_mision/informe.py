"""Texto y metricas del informe de la mision (`informe.json`, ver docs/MISION.md 5)."""


def tabla(titulo, cols, filas):
    """Tabla de texto simple (la primera columna a la izquierda, el resto a la derecha)."""
    filas = [[('' if x is None else str(x)) for x in f] for f in filas]
    anchos = [max(len(str(c)), *(len(f[i]) for f in filas)) if filas else len(str(c))
              for i, c in enumerate(cols)]
    lin = lambda f: '  '.join(str(x).rjust(a) if i else str(x).ljust(a)    # noqa: E731
                              for i, (x, a) in enumerate(zip(f, anchos)))
    return '\n'.join([titulo, lin(cols), '  '.join('-' * a for a in anchos)]
                     + [lin(f) for f in filas]) + '\n'


def _f(x, nd=2):
    return '' if x is None else f'{x:.{nd}f}'


def texto(inf, max_contactos=None):
    m, t = inf['mision'], inf['totales']
    out = [f"Mision {m['inicio_utc']}  {m['horas']:g} h  estaciones: {', '.join(m['estaciones'])}",
           f"Satelites: {', '.join(m['satelites'])}",
           f"Plan: {m['plan_algoritmo']}{' (optimo probado)' if m['plan_optimo_probado'] else ''}; "
           f"cargas utiles hipoteticas: "
           + ', '.join(f"{k}: {v['banda']} {v['eirp_dbw']:g} dBW {v['rs_baud'] / 1e6:g} Mbaud"
                       for k, v in m['cargas'].items()), '']
    out.append(tabla('Planificado contra ejecutado', ['', 'planificado', 'ejecutado'], [
        ['contactos', t['contactos_planificados'], t['contactos_ejecutados']],
        ['minutos', _f(t['minutos_plan']), _f(t['minutos'])],
        ['GB', _f(t['gb_plan'], 3), _f(t['gb'], 3)]]))
    out.append(
        f"GB con apuntamiento perfecto por el mismo camino: {_f(t['gb_sin_perdida_apunt'], 3)}; "
        f"perdida por apuntamiento: {_f(t['perdida_apunt_gb'], 3)} GB "
        f"({100.0 * t['perdida_apunt_gb'] / t['gb_sin_perdida_apunt']:.2f} %)" if t['gb_sin_perdida_apunt']
        else 'sin datos ejecutados')
    out.append(f"Otras diferencias con el plan (verdad contra TLE, muestreo, ventana): "
               f"{_f(t['gb_otras_diferencias'], 3)} GB\n")
    for clave, tit in (('por_satelite', 'satelite'), ('por_estacion', 'estacion')):
        out.append(tabla(f'Por {tit}', [tit, 'contactos', 'minutos', 'GB plan', 'GB', 'perd. apunt. GB'],
                         [[k, v['contactos'], _f(v['minutos'], 1), _f(v['gb_plan'], 2),
                           _f(v['gb'], 2), _f(v['perdida_apunt_gb'], 3)]
                          for k, v in inf[clave].items()]))
    filas = inf['contactos'][:max_contactos] if max_contactos else inf['contactos']
    out.append(tabla('Contactos', ['id', 'satelite', 'est', 'AOS', 'el', 'bnd', 'rol', 'seguim', 'efem',
                                   'GB plan', 'GB', 'rms deg', 'max deg', 'estado'],
                     [[f['id'], f['sat'].split(' (')[0], f['estacion'], f['aos'][5:16],
                       _f(f['el_max_deg'], 1), f['banda'], f['rol'], f['seguimiento'],
                       f.get('efemeride', ''), _f(f['gb_plan'], 3), _f(f.get('gb'), 3),
                       _f(f.get('apunt_rms_deg'), 3), _f(f.get('apunt_max_deg'), 3), f['estado']]
                      for f in filas]))
    if inf['od']:
        out.append(tabla('Determinacion de orbita', [
            'satelite', 'contactos', 'medidas', 'canales', 'sigma pos km', 'chi2/dof',
            'error pos km', 'TLE km', 'veredicto'],
            [[o['sat'].split(' (')[0], ','.join(str(i) for i in o['contactos']), o['n_medidas'],
              '+'.join(o['canales']), _f(o['sigma_pos_km'], 3), _f(o['chi2_dof'], 2),
              _f(o.get('error_pos_epoca_km'), 3), _f(o.get('error_pos_tle_epoca_km'), 3),
              'aceptada' if o['aceptada'] else 'rechazada: ' + o['motivo']] for o in inf['od']]))
    j = inf['mejora_od']
    out.append(f"Mejora por la OD: {j['contactos_con_od']} contactos volados con la orbita "
               f"estimada, {j['contactos_con_gemelo']} con gemelo (el mismo contacto con TLE).")
    if j['contactos_con_gemelo']:
        out.append(f"  GB con OD {_f(j['gb_con_od'], 3)} contra {_f(j['gb_con_tle'], 3)} con TLE: "
                   f"{j['delta_gb']:+.3f} GB ({j['delta_pct']:+.2f} %)")
        out.append(f"  apuntamiento rms mediano {_f(j['apunt_rms_od_mediana_deg'], 4)} deg con OD "
                   f"contra {_f(j['apunt_rms_tle_mediana_deg'], 4)} con TLE; maximo mediano "
                   f"{_f(j['apunt_max_od_mediana_deg'], 4)} contra {_f(j['apunt_max_tle_mediana_deg'], 4)}")
        out.append(f"  error de efemeride (maximo del pase, mediana) {_f(j['err_efemeride_max_od_mediana_deg'], 4)} "
                   f"deg con OD contra {_f(j['err_efemeride_max_tle_mediana_deg'], 4)} con TLE")
    out.append('')
    inc = inf['incidencias']
    out.append(f'Incidencias: {len(inc)}')
    for i in inc[:30]:
        out.append(f"  {i['contacto']} {i['sat']}: {i['tipo']}: {i['detalle']}")
    if len(inc) > 30:
        out.append(f'  ... y {len(inc) - 30} mas (informe.json)')
    v = inf['verificacion']
    out.append(f"\nVerificacion: {v['corridas_releidas']} corridas releidas desde su telemetria "
               f"-> {v['gb_releido']} GB, {v['minutos_releidos']} min; cuadra con los totales: "
               f"GB {'si' if v['cuadra_gb'] else 'NO'}, minutos {'si' if v['cuadra_minutos'] else 'NO'}")
    return '\n'.join(out) + '\n'


def metricas(inf):
    """`metrics.json` de la corrida en la Estacion (`kind: mision`): los totales y
    la mejora por la OD, sin las listas largas."""
    t, j = inf['totales'], inf['mejora_od']
    out = {'kind': 'mision', 'inicio_utc': inf['mision']['inicio_utc'], 'horas': inf['mision']['horas'],
           'estaciones': inf['mision']['estaciones'], 'satelites': inf['mision']['satelites']}
    out.update(t)
    out['plan_algoritmo'] = inf['mision']['plan_algoritmo']
    out['contactos_cal'] = sum(1 for f in inf['contactos'] if f['rol'] == 'cal')
    out['contactos_od'] = j['contactos_con_od']
    out['contactos_gemelo'] = j['contactos_con_gemelo']
    for k in ('gb_con_od', 'gb_con_tle', 'delta_gb', 'delta_pct'):
        out[k] = j.get(k)
    out['incidencias'] = len(inf['incidencias'])
    out['cuadra'] = bool(inf['verificacion']['cuadra_gb'] and inf['verificacion']['cuadra_minutos'])
    return out
