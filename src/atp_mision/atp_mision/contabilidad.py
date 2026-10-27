"""Datos descargados por un contacto, con lo MEDIDO en su corrida.

El planificador estima el volumen de un contacto con el rango del TLE y un
apuntamiento perfecto (`PerfilEnlace.error_apunt_deg = 0`). Aqui se recalcula con
la telemetria que grabo la corrida en lockstep (`telemetry.jsonl`, 10 Hz de
tiempo simulado): en cada muestra, el C/N0 que el enlace calculo con el error de
apuntamiento REAL de ese instante (contra el satelite verdadero) y el rango
verdadero. El resto es el mismo modelo del planificador:

  Es/N0 = C/N0 - 10 log10(Rs)                       (`planificador.esn0_db`)
  ACM ideal: el MODCOD DVB-S2 de mayor eficiencia con Es/N0 >= ETSI + margen
  tasa = Rs * eficiencia; volumen = integral (trapecios) de la tasa sobre el pase

`gb_sin_apunt` repite la cuenta quitando la perdida por desapuntamiento de cada
muestra (`l_point_db`): lo que habria bajado con apuntamiento perfecto POR EL MISMO
camino del satelite. La diferencia es la perdida por apuntamiento de la corrida
(error de efemeride + seguimiento + lo que el rotor no alcanza).

Valido bajo los supuestos del enlace (espacio libre, sin atmosfera, ACM ideal sin
latencia) y del muestreo de 0.1 s: la tasa ACM es escalonada y los trapecios a
10 Hz difieren de los de 1 s del planificador en una fraccion de por mil.
"""
import json
import math

import numpy as np

from atp_ros import planificador as pl

BITS_POR_GB = pl.BITS_POR_GB
_trapz = getattr(np, 'trapezoid', None) or np.trapz


def leer_telemetria(ruta):
    """Filas de un `telemetry.jsonl` (lista de dicts); [] si no existe."""
    filas = []
    try:
        with open(ruta, encoding='utf-8') as f:
            for linea in f:
                linea = linea.strip()
                if linea:
                    filas.append(json.loads(linea))
    except OSError:
        return []
    return filas


def _volumen_gb(t, cn0, rs_baud, margen_db):
    esn0 = pl.esn0_db(cn0, rs_baud)
    idx, eta = pl.elegir_modcod(esn0, margen_db)
    return float(_trapz(float(rs_baud) * eta, t)) / BITS_POR_GB, idx, esn0


def contabilizar(filas, rs_baud, margen_acm_db=1.0, umbral_perdida_db=1.0):
    """Cifras de un contacto a partir de las filas de su telemetria. Solo cuentan
    las muestras DENTRO del pase (las que traen `cn0_dbhz`). Devuelve un dict
    (todo float/int/str serializable) o `None` si no hay ninguna muestra en pase."""
    f = [r for r in filas if 'cn0_dbhz' in r]
    if len(f) < 2:
        return None
    t = np.array([r['t'] for r in f], float)
    cn0 = np.array([r['cn0_dbhz'] for r in f], float)
    lp = np.array([r.get('l_point_db', 0.0) for r in f], float)
    ept = np.array([r['e_pt'] for r in f if 'e_pt' in r], float)
    gb, idx, esn0 = _volumen_gb(t, cn0, rs_baud, margen_acm_db)
    gb_ideal, _, _ = _volumen_gb(t, cn0 + lp, rs_baud, margen_acm_db)
    i = int(np.argmax(esn0))
    nombre = pl._MC_NOMBRE[int(idx[i])] if idx[i] >= 0 else 'sin enlace'
    cerrado = np.array([bool(r.get('link_closed', True)) for r in f])
    return {
        'muestras': int(len(f)),
        'minutos': float(t[-1] - t[0]) / 60.0,
        'gb': gb,
        'gb_sin_apunt': gb_ideal,
        'perdida_apunt_gb': gb_ideal - gb,
        'cn0_min_dbhz': float(cn0.min()), 'cn0_max_dbhz': float(cn0.max()),
        'esn0_max_db': float(esn0.max()), 'modcod_max': nombre,
        'sin_modcod_pct': float(np.mean(idx < 0)) * 100.0,
        'enlace_cerrado_pct': float(np.mean(cerrado)) * 100.0,
        'l_point_max_db': float(lp.max()),
        'perdida_mayor_db_pct': float(np.mean(lp > umbral_perdida_db)) * 100.0,
        'apunt_rms_deg': float(math.sqrt(np.mean(ept ** 2))) if len(ept) else float('nan'),
        'apunt_p95_deg': float(np.percentile(ept, 95)) if len(ept) else float('nan'),
        'apunt_max_deg': float(ept.max()) if len(ept) else float('nan'),
    }


def contabilizar_a_mano(filas, rs_baud, margen_acm_db=1.0):
    """GB y minutos con un bucle escalar, SIN numpy y sin `elegir_modcod` vectorizado:
    la tabla ETSI se recorre fila a fila. Es la verificacion independiente del total
    del informe (`contabilizar` debe coincidir a 1e-9 relativo)."""
    t_prev = None
    rate_prev = None
    bits = 0.0
    t0 = None
    t1 = None
    for r in filas:
        if 'cn0_dbhz' not in r:
            continue
        esn0 = r['cn0_dbhz'] - 10.0 * math.log10(float(rs_baud))
        eta = 0.0
        for _, req, e in pl.MODCOD_DVBS2:
            if esn0 >= req + margen_acm_db and e > eta:
                eta = e
        rate = float(rs_baud) * eta
        if t_prev is not None:
            bits += 0.5 * (rate + rate_prev) * (r['t'] - t_prev)
        else:
            t0 = r['t']
        t_prev, rate_prev, t1 = r['t'], rate, r['t']
    if t0 is None:
        return None
    return {'gb': bits / BITS_POR_GB, 'minutos': (t1 - t0) / 60.0}
