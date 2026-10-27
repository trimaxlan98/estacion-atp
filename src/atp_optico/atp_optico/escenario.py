"""Escenario del ATP optico de la estacion SIN ROS (y su CLI): encadena el pase SGP4, la montura
gruesa de `atp_ros` (`montura.simular_montura`) y el motor FSM + atmosfera + enlace (`atp.AtpOptico`) en el
mismo orden que el lockstep de ROS, asi que da el MISMO RESUMEN (solo cambia de donde sale el error de
montura: de la planta simulada aqui, del topico `/atp/pointing_error` alla).

    python3 -m atp_optico.escenario                                   # ISS sobre CDMX, pase 0
    python3 -m atp_optico.escenario --pase_idx 3 --D_m 0.6 --modem ppm
    python3 -m atp_optico.escenario --campo_urad 4 --salida /tmp/opt   # escribe optico.json y optico_serie.jsonl

`ConfigOptico` agrupa todo lo que fija el enlace (terminal del satelite, telescopio, atmosfera, ATP);
cada campo es un argumento de la CLI y un parametro del nodo ROS `optico_atp`.
"""
import argparse
import dataclasses
import json
import math
import sys
import time
from dataclasses import dataclass

import numpy as np

from atp_optico import atmosfera as A
from atp_optico import atp as P
from atp_optico import enlace as L
from atp_optico import montura as M
from atp_ros import model, orbit


@dataclass(frozen=True)
class ConfigOptico:
    # satelite (SUPUESTOS de un microsatelite; ver docs/OPTICO.md)
    p_tx_w: float = 1.0
    theta_h_urad: float = 100.0
    sigma_sat_urad: float = 3.0
    lam_nm: float = 1550.0
    # estacion
    station: str = 'cdmx'
    D_m: float = 0.4
    receptor: str = 'campo'          # campo | smf
    campo_urad: float = 12.0
    modem: str = 'ook'               # ook | ppm
    # atmosfera
    cn2_suelo: float = 1.7e-14
    viento_rms: float = 21.0
    visibilidad_km: float = 23.0
    # ATP
    semilla: int = 7
    paso_s: float = 0.05
    modelo_apunt_urad: float = 89.0
    ncpa_urad: float = 0.5
    vibracion_urad: float = 0.5
    carrera_urad: float = 1000.0
    f_cruce_hz: float = 400.0
    p_out: float = 1e-2
    # salida
    salida: str = ''


def config_campos():
    """(nombre, tipo, defecto) de cada campo, para declararlos como parametros ROS."""
    return [(f.name, type(f.default), f.default) for f in dataclasses.fields(ConfigOptico)]


def construir(cfg):
    """(sat, est, atm, cfg_atp) desde un `ConfigOptico`."""
    lam = cfg.lam_nm * 1e-9
    sat = L.TerminalSat(lam_m=lam, p_tx_w=cfg.p_tx_w, theta_h_urad=cfg.theta_h_urad,
                        sigma_apunt_urad=cfg.sigma_sat_urad)
    modem = L.ModemOOK() if cfg.modem == 'ook' else L.ModemPPM()
    est = L.EstacionOptica(D_m=cfg.D_m, D_tx_m=cfg.D_m, receptor=cfg.receptor, campo_urad=cfg.campo_urad, modem=modem)
    e = orbit.buscar_estacion(cfg.station or None)
    atm = A.Atmosfera(cn2_suelo=cfg.cn2_suelo, viento_rms=cfg.viento_rms, alt_estacion_m=float(e.alt_m),
                      visibilidad_km=cfg.visibilidad_km)
    lazo = dataclasses.replace(P.LAZO_FSM, f_cruce_hz=cfg.f_cruce_hz)
    catp = P.ConfigAtp(paso_s=cfg.paso_s, lazo=lazo, semilla=cfg.semilla, vibracion_urad=cfg.vibracion_urad,
                       ncpa_urad=cfg.ncpa_urad, modelo_apunt_urad=cfg.modelo_apunt_urad, p_out=cfg.p_out,
                       carrera_urad=cfg.carrera_urad)
    return sat, est, atm, catp


def redondear(x, cifras=7):
    """Redondeo a `cifras` significativas, recursivo, para JSON estables."""
    if isinstance(x, dict):
        return {k: redondear(v, cifras) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [redondear(v, cifras) for v in x]
    if isinstance(x, (float, np.floating)):
        return float(f'{float(x):.{cifras}g}') if math.isfinite(float(x)) else None
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


def fila_serie(r):
    """Fila compacta de un paso para el archivo de serie (determinista)."""
    return {'t': r['t'], 'el': r['el'], 'rango_km': r['rango_km'], 'rms_montura': r['rms_montura_urad'],
            'rms_tilt': r['rms_tilt_urad'], 's_fsm': r['sigma_fsm_urad'], 's_sin': r['sigma_sin_urad'],
            'y_max': r['y_max_urad'], 'nea': r['nea_urad'], 'p_rx_dbm': r['con_fsm']['p_rx_dbm'],
            'tasa': r['con_fsm']['tasa_bps'], 'outage': r['con_fsm']['outage'], 'tasa_sin': r['sin_fsm']['tasa_bps']}


def escribir_salida(directorio, med, filas):
    import os
    os.makedirs(directorio, exist_ok=True)
    with open(os.path.join(directorio, 'optico.json'), 'w') as f:
        json.dump(redondear(med.resumen_dict()), f, sort_keys=True, indent=1)
    with open(os.path.join(directorio, 'optico_serie.jsonl'), 'w') as f:
        for r in filas:
            f.write(json.dumps(redondear(fila_serie(r)), sort_keys=True) + '\n')


def simular(cfg, sat='ISS', pase_idx=0, inicio_utc='2026-10-02T15:00:00', mascara_deg=10.0, montura=None,
            max_pasos=None, perfil=None, mont=None):
    """Pase completo sin ROS. Devuelve (Medidor, filas, meta). `max_pasos` recorta (pruebas); `mont` reutiliza
    la salida de `montura.simular_montura` (para barrer parametros opticos sobre la misma montura)."""
    s, est, atm, catp = construir(cfg)
    if perfil is None:
        perfil, pase, tle, e = M.perfil_pase(sat, cfg.station or None, pase_idx, inicio_utc, 72.0, mascara_deg)
    else:
        pase = {'aos': None, 'duracion_s': perfil['duracion'], 'el_max_deg': float(np.max(perfil['el']))}
    if mont is None:
        mont = M.simular_montura(perfil, montura or M.ConfigMontura(paso_s=cfg.paso_s, semilla=cfg.semilla))
    eng = P.AtpOptico(catp, s, est, atm)
    med = P.Medidor(catp, est)
    filas = []
    n = len(mont['t']) if max_pasos is None else min(len(mont['t']), max_pasos)
    for i in range(n):
        r = eng.paso(mont['t'][i], mont['az'][i], mont['el'][i], mont['rango_km'][i], M.error_cielo_urad(mont, i))
        med.muestra(r)
        filas.append(r)
    meta = {'pase': pase, 'montura': mont}
    return med, filas, meta


# ---------------------------------------------------------------- analisis casi estatico (sin tiempo)

def curva_elevacion(sat, est, atm, h_km=550.0, elevaciones=(10, 15, 20, 30, 45, 60, 90), sigma_urad=None,
                    p_out=1e-2, subida=False, modem=None):
    """Filas de `enlace.evaluar_enlace` a las elevaciones dadas para una orbita circular de h_km
    (rango oblicuo de `atp_ros.model.rango_oblicuo`)."""
    out = []
    for el in elevaciones:
        R = float(model.rango_oblicuo(h_km, el))
        out.append(L.evaluar_enlace(sat, est, atm, float(el), R, sigma_urad, p_out, subida, modem))
    return out


def volumen_pase(sat, est, atm, perfil, sigma_urad, p_out=1e-2, dt_s=1.0):
    """Datos (GB) que entrega un pase con enlace de bajada de error residual `sigma_urad` fijo y tasa adaptada por
    elevacion (casi estatico: sin simular el lazo). Devuelve {'gb', 'cerrado_s', 'tasa_media_gbps', 'duracion_s'}."""
    t = np.arange(0.0, perfil['duracion'], dt_s)
    el = np.interp(t, perfil['t'], perfil['el'])
    R = np.interp(t, perfil['t'], perfil['d'])
    bits = 0.0
    cerrado = 0.0
    tasas = []
    for e, r in zip(el, R):
        x = P._evaluar_cache(sat, est, atm, round(float(e) * 4.0) / 4.0, round(float(r), 0), P._cuant_sigma(sigma_urad), p_out)
        if x['tasa_bps'] > 0:
            bits += x['util_bps'] * (1.0 - x['outage']) * dt_s
            cerrado += dt_s
        tasas.append(x['tasa_bps'])
    return {'gb': bits / 8e9, 'cerrado_s': cerrado, 'tasa_media_gbps': 1e-9 * float(np.mean(tasas)),
            'duracion_s': float(perfil['duracion'])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for f in dataclasses.fields(ConfigOptico):
        ap.add_argument('--' + f.name, type=type(f.default), default=f.default)
    ap.add_argument('--sat', default='ISS')
    ap.add_argument('--pase_idx', type=int, default=0)
    ap.add_argument('--inicio_utc', default='2026-10-02T15:00:00')
    ap.add_argument('--mascara_deg', type=float, default=10.0)
    ap.add_argument('--max_pasos', type=int, default=0)
    a = ap.parse_args(argv)
    cfg = ConfigOptico(**{f.name: getattr(a, f.name) for f in dataclasses.fields(ConfigOptico)})
    t0 = time.time()
    med, filas, meta = simular(cfg, a.sat, a.pase_idx, a.inicio_utc, a.mascara_deg, max_pasos=a.max_pasos or None)
    p = meta['pase']
    print(f"pase {a.sat} sobre {cfg.station}: {p['duracion_s']:.0f} s, el max {p['el_max_deg']:.1f} deg, "
          f"error de montura rms {1e6 * math.radians(np.sqrt(np.mean(meta['montura']['e_pt_deg'] ** 2))):.1f} urad")
    for linea in med.resumen():
        print(linea)
    if cfg.salida:
        escribir_salida(cfg.salida, med, filas)
    print(f'({time.time() - t0:.1f} s de pared)', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
