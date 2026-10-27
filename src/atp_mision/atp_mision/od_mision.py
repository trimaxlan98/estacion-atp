"""La OD de la mision: con las medidas que REGISTRARON las corridas.

Tras los primeros pases de un satelite (los de calibracion: autotrack enganchado,
receptor SDR cuando la banda lo permite) se ajusta su orbita con lo que quedo en la
telemetria de cada corrida (`telemetry.jsonl`), no con la verdad:

  * acimut y elevacion: los ANGULOS DE LA MONTURA (`az`, `el`). Solo valen mientras el
    autotrack esta enganchado (si no, la montura apunta a la efemeride y no mide al
    satelite). La mision exige que el autotrack haya anunciado ENGANCHE (el aviso de
    su log; sin el, la corrida no aporta medidas), espera `ASENTAMIENTO_S` tras el AOS a
    que el lazo converja (tau ~ 1.6 s) y descarta las muestras con mas de
    `perdida_max_db` de perdida por apuntamiento (el operador conoce el nivel que debe
    recibir: detecta un desenganche por el nivel de la senal). Cerca del cenit el acimut
    no sirve: elevacion entre 3 y 85 grados.
  * Doppler (solo con receptor SDR en ese contacto): el `doppler_hz` del enlace mas
    ruido gaussiano de 0.05 m/s con semilla fija (`docs/OD.md` 1.3: lo que un GPSDO de
    1e-10 mas la ionosfera residual dejan). El rango NO se usa (la estacion no tiene
    transpondedor con *ranging*).
  * una medida cada `decim_s` s. La sigma de los angulos sale de la banda:
    1.5 % del ancho de haz (el autotrack mide 1.3 % en S y X en estas corridas;
    en N5 en Ka, 1.7 %).

El ajuste es el de `atp_ros.od.ajuste_lotes` (lotes, Levenberg-Marquardt), con el
mismo modelo del estimador (J2 + arrastre nominal) y el prior del TLE, igual que el
experimento de docs/OD.md 3.2 ("OD con 3 pases de la red"). No estima el arrastre.

Aceptacion: converge, chi2/dof < 5, sigma de posicion formal < 5 km y la orbita
ajustada a menos de 100 km del TLE en la epoca. Si falla, el operador sigue con el
TLE y la mision lo anota como incidencia. La covarianza formal NO cubre el error de
modelo (docs/OD.md 3.4): por eso la mejora se mide contra la verdad, en el informe.
"""
import json
import math

import numpy as np

from atp_ros import medidas as M
from atp_ros import model, od, orbit
from atp_ros import propagador as P

SIGMA_ANG_FRAC = 0.015        # sigma angular = fraccion del ancho de haz de la banda
SIGMA_DOPPLER_M_S = 0.05      # docs/OD.md 1.3
DECIM_S = 5.0
EL_MIN_DEG, EL_MAX_DEG = 3.0, 85.0
PERDIDA_MAX_DB = 0.5          # S: 0.4 deg de error, X: 0.1 deg
ASENTAMIENTO_S = 15.0         # espera tras el AOS a que el autotrack converja
MIN_MEDIDAS = 12              # por debajo de esto un pase no aporta (menos de 1 min enganchado)
LIMITES = {'chi2_dof': 5.0, 'sigma_pos_km': 5.0, 'desplazamiento_km': 100.0}


def sigma_angular_deg(banda, dish_m=model.DIAMETRO_PLATO_M):
    return SIGMA_ANG_FRAC * model.ancho_haz(dish_m, model.BANDAS[banda])


def medidas_de_telemetria(filas, reloj, sitios, isit, aos_utc, banda, con_doppler, rng,
                          decim_s=DECIM_S, perdida_max_db=PERDIDA_MAX_DB):
    """`Medidas` de un contacto a partir de las filas de su telemetria (None si quedan
    menos de MIN_MEDIDAS). `aos_utc`: el AOS del pase que volo la corrida (el `t` de
    cada fila es desde ese AOS). `rng`: Generator de numpy para el ruido del Doppler
    (se consume siempre en el mismo orden: repetible)."""
    t0 = reloj.segundos(aos_utc)
    f0 = model.BANDAS[banda]
    t, az, el, rr = [], [], [], []
    ult = -1e9
    for r in filas:
        if 'l_point_db' not in r or 'az' not in r or 'el' not in r:
            continue
        if (r['t'] < ASENTAMIENTO_S or r['l_point_db'] > perdida_max_db
                or not EL_MIN_DEG <= r['el'] <= EL_MAX_DEG):
            continue
        if r['t'] - ult < decim_s - 1e-9:
            continue
        ult = r['t']
        t.append(t0 + r['t'])
        az.append(math.radians(r['az']) % (2.0 * math.pi))
        el.append(math.radians(r['el']))
        if con_doppler and 'doppler_hz' in r:
            # fd = -rr f0 / c  ->  rr (km/s)
            rr.append(-r['doppler_hz'] * model.C_LUZ_M_S / f0 / 1000.0
                      + rng.normal(0.0, SIGMA_DOPPLER_M_S * 1e-3))
        else:
            rr.append(float('nan'))
    if len(t) < MIN_MEDIDAS:
        return None
    return M.Medidas(reloj, sitios, np.array(t), np.full(len(t), int(isit)),
                     {'az': np.array(az), 'el': np.array(el), 'rr': np.array(rr)})


def ajustar(tle, reloj, lista_medidas, banda, modelo=None):
    """OD por lotes con las `Medidas` de varios contactos de UN satelite (mismo `reloj` y
    mismos `sitios`). Devuelve (resultado, info): `info` es un dict serializable con la
    calidad del ajuste y el veredicto de aceptacion (`aceptada`, `motivo`)."""
    med = lista_medidas[0]
    for m in lista_medidas[1:]:
        med = med.concatenar(m)
    hay_rr = bool(np.any(np.isfinite(med.val['rr'])))
    canales = ['az', 'el'] + (['rr'] if hay_rr else [])
    med = med.solo(canales)
    modelo = modelo or P.Modelo(grado=2, cd_a_m=0.0105 * 0.406)
    sg = dict(M.SIGMAS_DEFECTO)
    sg['az'] = sg['el'] = math.radians(sigma_angular_deg(banda))
    t0 = float(med.t[0])
    r, v = orbit.propagar_teme(tle, *reloj.jdfr(t0))
    x0 = np.r_[r[0], v[0]]
    xb, Pb = od.prior_de_tle(x0, tle.edad_dias(reloj.utc(t0)))
    res = od.ajuste_lotes(med, xb[:6], t0, modelo, sg, False, (xb, Pb))
    desp = float(np.linalg.norm(res.x[:3] - x0[:3]))
    info = {'n_medidas': int(len(med)), 'canales': canales,
            'epoca_utc': reloj.utc(t0).strftime('%Y-%m-%dT%H:%M:%S.%fZ'),
            'chi2_dof': float(res.chi2_dof), 'sigma_pos_km': float(res.sigma_pos_km()),
            'iteraciones': int(res.iteraciones), 'convergio': bool(res.convergio),
            'desplazamiento_vs_tle_km': desp,
            'sigma_ang_deg': float(sigma_angular_deg(banda))}
    motivo = []
    if not res.convergio:
        motivo.append('no convergio')
    if not res.chi2_dof < LIMITES['chi2_dof']:
        motivo.append(f"chi2/dof {res.chi2_dof:.1f} >= {LIMITES['chi2_dof']:g}")
    if not info['sigma_pos_km'] < LIMITES['sigma_pos_km']:
        motivo.append(f"sigma de posicion {info['sigma_pos_km']:.1f} km")
    if not desp < LIMITES['desplazamiento_km']:
        motivo.append(f'orbita a {desp:.0f} km del TLE')
    info['aceptada'] = not motivo
    info['motivo'] = '; '.join(motivo)
    return res, info


def escribir(res, reloj, tle, estaciones, ruta, extra=None):
    """Guarda la orbita estimada como el JSON que lee `reference_generator`
    (`profile:=od od_file:=...`, CONTRATO 9)."""
    d = od.resultado_json(res, reloj, tle.nombre, tle.norad, estaciones, extra)
    with open(ruta, 'w', encoding='utf-8') as f:
        json.dump(d, f, sort_keys=True)
    return d
