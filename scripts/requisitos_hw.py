#!/usr/bin/env python3
"""Requisitos de hardware derivados de la simulacion (equipo S, ronda 5).

Responde con numeros calculados (no de memoria) a "que motor, que encoder y que
enlace hacen falta" para tres escenarios: banco de escritorio, Yagi VHF/UHF y
plato de 1-3 m en S/X. Reutiliza la geometria (`atp_ros.model.perfil_pase`),
los pases SGP4 reales de `data/tle` (`atp_ros.orbit`) y la planta de
`docs/CONTROL.md` (J, b). Cada funcion dice su formula y su fuente; lo que es
SUPUESTO esta rotulado como tal en `SUPUESTOS` y en docs/HARDWARE.md.

Uso:
    PYTHONPATH=src/atp_ros python3 scripts/requisitos_hw.py            # tablas en Markdown
    PYTHONPATH=src/atp_ros python3 scripts/requisitos_hw.py --json out.json
    PYTHONPATH=src/atp_ros python3 scripts/requisitos_hw.py --rapido   # sin SGP4 (para pruebas)
"""
import argparse
import datetime as dt
import json
import math
import sys

import numpy as np

from atp_ros import model

R = model.R_TIERRA_KM
ALTURAS_KM = (400.0, 550.0, 800.0)              # peor caso razonable de LEO (encargo)
ELMAX_GRID = (10.0, 30.0, 50.0, 70.0, 80.0, 85.0, 88.0)
G = 9.80665                                     # m/s^2 (valor convencional)

# --------------------------------------------------------------------------
# SUPUESTOS de diseno (no medidos): todos aparecen en docs/HARDWARE.md
# --------------------------------------------------------------------------
SUPUESTOS = {
    'fraccion_haz': 0.1,        # error total rms = theta3/10 -> 0.12 dB (PUESTA_EN_MARCHA A3; L=12 (e/theta)^2)
    'margen_par': 1.5,          # factor de seguridad sobre el par calculado (criterio de ingenieria)
    'rend_reductor': 0.85,      # rendimiento total reductor+transmision (tipico de tren sinfin/planetario, SUPUESTO)
    't_acel_s': 1.0,            # tiempo para llegar a vmax en un re-apuntamiento (SUPUESTO de diseno)
    'rafaga_m_s': 10.0,         # viento de operacion (par_viento del curso usa 10 m/s)
    'tension_bus_v': 24.0,      # bus del motor (SUPUESTO)
}
# reparto del presupuesto de apuntamiento: fraccion del CUADRADO del error total (suma RSS, fuentes
# independientes). Suma 1.0. Es una propuesta de ingenieria, no una cifra de fuente.
REPARTO_RSS = {
    'encoder (cuantizacion)': 0.10,
    'holgura (backlash)': 0.15,
    'nivelacion / orientacion / cero (calibrable)': 0.20,
    'control (seguimiento residual)': 0.25,
    'latencia y sellado de tiempo': 0.10,
    'flexion, termica, viento': 0.20,
}


def _lam_mask(h, mask=model.MASCARA_DEG):
    return float(model.angulo_central(h, mask))


# ------------------------------------------------------------ 1. cinematica
def cinematica_pase(h_km, el_max_deg, n=60000):
    """Picos de velocidad y aceleracion por eje de un pase de orbita circular
    (`model.perfil_pase`, esferico, SIN rotacion terrestre) en az/el.
    La aceleracion es el gradiente numerico de la velocidad (n grande para
    resolver el pico cenital); contraste con `model.tasa_acimut` (analitica)."""
    p = model.perfil_pase(h_km, el_max_deg, n=n)
    t = p['t']
    a_az = np.gradient(p['az_pto'], t)
    a_el = np.gradient(p['el_pto'], t)
    return {
        'h_km': h_km, 'el_max': el_max_deg,
        'v_az': float(np.max(np.abs(p['az_pto']))), 'v_el': float(np.max(np.abs(p['el_pto']))),
        'a_az': float(np.max(np.abs(a_az))), 'a_el': float(np.max(np.abs(a_el))),
        'v_az_analitica': model.tasa_acimut(h_km, el_max_deg),
        'duracion_s': p['duracion'],
    }


def fraccion_pases_sobre(h_km, el_deg):
    """Fraccion de pases al azar cuya elevacion maxima supera `el_deg`.
    `model.elmax_aleatorio` toma lam_min ~ U(0, lam_mascara): P(el_max > e) =
    lam(e)/lam_mascara (el_max > e  <=>  lam_min < lam(e))."""
    return float(model.angulo_central(h_km, el_deg) / _lam_mask(h_km))


def elmax_alcanzable(h_km, v_az_max):
    """Elevacion maxima de pase que una montura Az/El sigue sin perder el
    satelite con tope de acimut `v_az_max` (deg/s): `model.elevacion_keyhole`."""
    return model.elevacion_keyhole(h_km, v_az_max)


def elmax_por_aceleracion(h_km, a_az_max, paso=0.05):
    """Mayor el_max cuyo pico de aceleracion de acimut cabe en `a_az_max`
    (deg/s^2); barrido sobre `cinematica_pase` (monotono creciente)."""
    mejor = 0.0
    for el in np.arange(10.0, 89.9, paso * 20):       # 1 deg de paso
        if cinematica_pase(h_km, float(el), n=20000)['a_az'] <= a_az_max:
            mejor = float(el)
        else:
            break
    return mejor


CLASES_MONTURA = (   # (nombre, vmax_az deg/s, amax_az deg/s^2): clases tipicas, SUPUESTO de catalogo
    ('rotor de antena tipo Yaesu G-5500 (~6 deg/s az; aprox. 3 deg/s el)', 6.0, 2.0),
    ('servo/paso a paso de banco', 20.0, 40.0),
    ('servomotor con reductora (plato 1-3 m)', 30.0, 60.0),
    ('direct-drive / servo rapido', 90.0, 400.0),
)


def tabla_keyhole(alturas=ALTURAS_KM):
    """Para cada clase de motor/montura, hasta que el_max de pase alcanza,
    por velocidad y por aceleracion, y que fraccion de pases (al azar) deja
    sin seguir completamente. Incluye el caso cenital (el_max ~ 90 deg)."""
    filas = []
    for nombre, v, a in CLASES_MONTURA:
        for h in alturas:
            el_v = elmax_alcanzable(h, v)
            el_a = elmax_por_aceleracion(h, a)
            el = min(el_v, el_a)
            filas.append({'clase': nombre, 'vmax': v, 'amax': a, 'h_km': h,
                          'el_max_por_velocidad': el_v, 'el_max_por_aceleracion': el_a,
                          'el_max_alcanzable': el,
                          'fraccion_pases_perdidos': fraccion_pases_sobre(h, el)})
    return filas


def cenital_con_tope(h_km, v_cap, a_cap):
    """Pase cenital (el_max 89.9 deg) con la demanda de acimut limitada a
    v_cap/a_cap: error angular de acimut (deg) y duracion perdida (s). El
    acimut demandado salta ~180 deg en pocos segundos junto al cenit; con un
    rotor de tope finito el acimut real se atrasa. Se simula un seguidor ideal
    con tope de velocidad y aceleracion (sin lazo): es una cota inferior del
    atraso (un lazo real atrasa mas)."""
    p = model.perfil_pase(h_km, 89.9, n=200000)
    t, az = p['t'], p['az_continuo']
    dt_s = t[1] - t[0]
    pos, vel = az[0], 0.0
    err = np.empty_like(t)
    for i in range(len(t)):
        # velocidad deseada para alcanzar la referencia, con frenado sqrt (v^2 = 2 a d)
        d = az[i] - pos
        v_des = math.copysign(min(v_cap, math.sqrt(2.0 * a_cap * abs(d))), d)
        dv = max(-a_cap * dt_s, min(a_cap * dt_s, v_des - vel))
        vel += dv
        pos += vel * dt_s
        err[i] = az[i] - pos
    # un error de acimut e cuesta e*cos(el) en el cielo
    cielo = np.abs(err) * np.cos(np.radians(p['el']))
    return {'v_cap': v_cap, 'a_cap': a_cap, 'err_az_max_deg': float(np.max(np.abs(err))),
            'err_cielo_max_deg': float(np.max(cielo)),
            't_fuera_1deg_s': float(np.sum(cielo > 1.0) * dt_s)}


def elevacion_hasta_180(h_km=550.0):
    """Montura 'sobre el cenit' (el hasta 180 deg, el acimut no gira 180 deg):
    en el cenit el acimut de la montura es fijo y la elevacion pasa de 90 a >90.
    La velocidad de elevacion pico del pase cenital es la de la linea de vista
    (v/h ~ 0.8 deg/s a 550 km): sin singularidad. Requisito mecanico OPCIONAL:
    rango de elevacion 0-180 deg y una estrategia que decida el lado."""
    p = model.perfil_pase(h_km, 89.9, n=60000)
    return {'v_el_pico': float(np.max(np.abs(p['el_pto']))),
            'v_el_analitica': float(np.degrees(model.velocidad_circular(h_km) / h_km)),
            'rango_el_deg': 180.0, 'duracion_s': p['duracion']}


# --------------------------------------------------------- SGP4 (pases reales)
def pases_sgp4(estacion='cdmx', horas=72.0, inicio=None, lista=None):
    """Pases reales del catalogo `data/tle` sobre `estacion`. Devuelve filas con
    el_max, picos de velocidad/aceleracion por eje (derivadas numericas de
    `orbit.muestrear_pase`, dt = 0.25 s; cerca del cenit estan submuestreadas:
    es fisica del keyhole) y la altitud media del satelite."""
    from atp_ros import orbit
    est = orbit.buscar_estacion(estacion)
    cat = orbit.cargar_catalogo()
    inicio = inicio or dt.datetime(2026, 10, 2, tzinfo=dt.timezone.utc)
    lista = lista or ('ISS (ZARYA)', 'FUNCUBE-1', 'CUBESAT XI-IV', 'NOAA 15', 'NOAA 18', 'NOAA 19',
                      'NOAA 20', 'NOAA 21', 'METEOR-M2 3', 'METEOR-M2 4')
    filas, por_sat = [], {}
    for nom in lista:
        tle = orbit.buscar_tle(cat, nom)
        n = tle.n_rev_dia
        a_km = (model.MU_TIERRA / (n * 2.0 * math.pi / 86400.0) ** 2) ** (1.0 / 3.0)
        h = a_km - R
        ps = orbit.predecir_pases(tle, est, inicio, horas=horas, el_max_min_deg=0.0)
        por_sat[tle.nombre] = ps
        for pase in ps:
            if pase['aos_truncado'] or pase['los_truncado']:
                continue
            m = orbit.muestrear_pase(tle, est, pase, dt_s=0.25)
            t = m['t']
            filas.append({
                'sat': tle.nombre, 'h_km': h, 'el_max': pase['el_max_deg'],
                'v_az': float(np.max(np.abs(m['az_pto']))), 'v_el': float(np.max(np.abs(m['el_pto']))),
                'a_az': float(np.max(np.abs(np.gradient(m['az_pto'], t)))),
                'a_el': float(np.max(np.abs(np.gradient(m['el_pto'], t)))),
                'duracion_s': pase['duracion_s'],
                'aos': pase['aos'], 'los': pase['los'],
                'az_aos': pase['az_aos_deg'], 'az_los': pase['az_los_deg'],
            })
    return filas, por_sat


def distribucion_por_elmax(filas, cortes=(0, 20, 40, 60, 80, 90)):
    """Percentiles (p50, p95, max) de v_az, a_az, v_el por rango de el_max."""
    out = []
    for lo, hi in zip(cortes[:-1], cortes[1:]):
        s = [f for f in filas if lo <= f['el_max'] < hi]
        if not s:
            out.append({'rango': (lo, hi), 'n': 0})
            continue
        def pc(k, q):
            return float(np.percentile([f[k] for f in s], q))
        out.append({'rango': (lo, hi), 'n': len(s),
                    'v_az_p50': pc('v_az', 50), 'v_az_max': pc('v_az', 100),
                    'a_az_p50': pc('a_az', 50), 'a_az_max': pc('a_az', 100),
                    'v_el_max': pc('v_el', 100), 'a_el_max': pc('a_el', 100)})
    return out


def pase_cenital_sgp4(sat='NOAA 19', miss_km=(5.0, 20.0, 60.0, 120.0)):
    """Pases SGP4 sobre estaciones sinteticas colocadas bajo la traza del
    satelite, con un desplazamiento lateral `miss_km`: da pases reales de
    el_max alto (en la practica casi no ocurren con una estacion fija). Valida
    la geometria esferica contra SGP4 con rotacion terrestre."""
    from atp_ros import orbit
    cat = orbit.cargar_catalogo()
    tle = orbit.buscar_tle(cat, sat)
    t0 = dt.datetime(2026, 10, 2, 6, 0, 0, tzinfo=dt.timezone.utc)
    jd, fr = orbit.a_jd(t0)
    r, v = orbit.propagar_ecef(tle, np.array([jd]), np.array([fr]))
    lat, lon, _ = orbit.ecef_a_geodetica(r[0])
    jd2, fr2 = orbit.a_jd(t0 + dt.timedelta(seconds=10))
    r2, _ = orbit.propagar_ecef(tle, np.array([jd2]), np.array([fr2]))
    lat2, lon2, _ = orbit.ecef_a_geodetica(r2[0])
    rumbo = math.atan2(math.sin(math.radians(lon2 - lon)) * math.cos(math.radians(lat2)),
                       math.cos(math.radians(lat)) * math.sin(math.radians(lat2))
                       - math.sin(math.radians(lat)) * math.cos(math.radians(lat2))
                       * math.cos(math.radians(lon2 - lon)))
    out = []
    for d in miss_km:
        brg = rumbo + math.pi / 2.0
        ang = d / R
        la1, lo1 = math.radians(lat), math.radians(lon)
        la = math.asin(math.sin(la1) * math.cos(ang) + math.cos(la1) * math.sin(ang) * math.cos(brg))
        lo = lo1 + math.atan2(math.sin(brg) * math.sin(ang) * math.cos(la1),
                              math.cos(ang) - math.sin(la1) * math.sin(la))
        est = orbit.Estacion('sintetica', math.degrees(la), math.degrees(lo), 0.0, 5.0)
        ps = orbit.predecir_pases(tle, est, t0 - dt.timedelta(minutes=8), horas=0.3)
        ps = [p for p in ps if abs((p['t_el_max'] - t0).total_seconds()) < 300]
        if not ps:
            continue
        p = ps[0]
        m = orbit.muestrear_pase(tle, est, p, dt_s=0.05)
        a_km = (model.MU_TIERRA / (tle.n_rev_dia * 2.0 * math.pi / 86400.0) ** 2) ** (1.0 / 3.0)
        an = cinematica_pase(a_km - R, min(p['el_max_deg'], 89.5), n=60000)
        out.append({'miss_km': d, 'el_max': p['el_max_deg'],
                    'v_az_sgp4': float(np.max(np.abs(m['az_pto']))),
                    'v_az_esferico': an['v_az'], 'v_az_analitica': an['v_az_analitica']})
    return out


def re_apuntamiento(por_sat):
    """Cambio de satelite (handover): entre pases consecutivos (ordenados por
    AOS, de TODOS los satelites) el angulo que hay que recorrer en acimut
    (camino corto) y el tiempo disponible. Sale de los pases SGP4."""
    todos = []
    for ps in por_sat.values():
        todos += [p for p in ps if not (p['aos_truncado'] or p['los_truncado'])]
    todos.sort(key=lambda p: p['aos'])
    dazs, huecos = [], []
    for a, b in zip(todos[:-1], todos[1:]):
        daz = abs((b['az_aos_deg'] - a['az_los_deg'] + 180.0) % 360.0 - 180.0)
        hueco = max(0.0, (b['aos'] - a['los']).total_seconds())
        dazs.append(daz)
        huecos.append(hueco)
    cortos = [d for d, h in zip(dazs, huecos) if h <= 60.0]
    return {'n': len(dazs), 'n_hueco_menor_60s': len(cortos),
            'daz_max_hueco_menor_60s': float(max(cortos)) if cortos else 0.0,
            'daz_p50': float(np.percentile(dazs, 50)) if dazs else 0.0,
            'daz_p95': float(np.percentile(dazs, 95)) if dazs else 0.0,
            'daz_max': float(max(dazs)) if dazs else 0.0,
            'hueco_p50_s': float(np.percentile(huecos, 50)) if huecos else 0.0,
            'hueco_min_s': float(min(huecos)) if huecos else 0.0,
            'solapados': int(sum(1 for h in huecos if h == 0.0))}


def v_slew_necesaria(delta_deg, t_s, a_deg_s2):
    """Velocidad pico de un perfil trapezoidal de aceleracion `a` que recorre
    `delta` en `t`: t = d/v + v/a  ->  v = (a t/2)(1 - sqrt(1 - 4 d/(a t^2))).
    NaN si ni con velocidad infinita se llega (4 d > a t^2)."""
    disc = 1.0 - 4.0 * delta_deg / (a_deg_s2 * t_s * t_s)
    if disc < 0.0:
        return float('nan')
    return a_deg_s2 * t_s / 2.0 * (1.0 - math.sqrt(disc))


# ------------------------------------------------------------------ 2. haz
def theta3_plato(d_m, f_hz):
    return model.ancho_haz(d_m, f_hz)


def theta3_yagi(ganancia_dbi):
    """Yagi: HPBW ~ sqrt(41253/G) grados (haz de lapiz, Kraus, Antennas; mismos
    anchos en E y H: aproximacion). Contrasta con 70 lambda/D usando la longitud
    del boom como apertura equivalente."""
    return float(math.sqrt(41253.0 / 10.0 ** (ganancia_dbi / 10.0)))


ESCENARIOS = {
    'A banco (pan-tilt pequeno, sin RF)': {
        'clave': 'banco',
        'descripcion': 'laser/camara/antena de prueba; objetivo = demostrar el lazo al 0.1 deg del curso',
        'theta3_deg': None, 'e_total_deg': model.OBJETIVO_DEG,     # el objetivo del curso, no un haz
        'v_az_max': 20.0, 'a_az_max': 40.0, 'v_el_max': 10.0, 'a_el_max': 20.0,
        'J_kgm2': 0.05, 'b': 0.02, 'fric_nm': 0.05, 'desbalance_nm': 0.05, 'viento_nm': 0.0,
        'pasos': True,
    },
    'B Yagi VHF/UHF (radioaficionado)': {
        'clave': 'yagi',
        'descripcion': 'Yagi ~ 10-12 dBi (UHF, boom ~1.5 m) o ~ 12 dBi (VHF, boom ~3 m); NOAA/ISS/CubeSats',
        'f_hz': 145.8e6, 'ganancia_dbi': 12.0,
        'v_az_max': 6.0, 'a_az_max': 2.0, 'v_el_max': 3.0, 'a_el_max': 2.0,
        'J_kgm2': 3.0, 'b': 0.5, 'fric_nm': 1.0, 'desbalance_nm': 1.0, 'viento_nm': None,
        'area_m2': 0.3, 'cd': 1.2, 'brazo_m': 0.3,
        'pasos': False,
    },
    'C plato 1-3 m en S/X (la Estacion)': {
        'clave': 'plato',
        'descripcion': 'plato de 3 m en X (8.4 GHz): el caso duro; 1 m en S es el caso comodo',
        'diametro_m': 3.0, 'f_hz': 8.4e9,
        'v_az_max': 30.0, 'a_az_max': 30.0, 'v_el_max': 10.0, 'a_el_max': 10.0,
        'J_kgm2': 110.0, 'b': 2.0, 'fric_nm': 5.0, 'desbalance_nm': 10.0, 'viento_nm': None,
        'area_m2': math.pi * 1.5 ** 2, 'cd': 1.2, 'brazo_m': 0.15,
        'pasos': False,
    },
}


def theta3_escenario(e):
    if e.get('theta3_deg') is not None:
        return e['theta3_deg']
    if 'diametro_m' in e:
        return theta3_plato(e['diametro_m'], e['f_hz'])
    if 'ganancia_dbi' in e:
        return theta3_yagi(e['ganancia_dbi'])
    return None


def tabla_platos():
    """theta3 y e_total = theta3/10 para platos de 1, 2 y 3 m en S (2.2 GHz) y X (8.4 GHz) y Ka."""
    filas = []
    for d in (1.0, 2.0, 3.0):
        for nombre, f in (('S', 2.2e9), ('X', 8.4e9), ('Ka', 30e9)):
            th = theta3_plato(d, f)
            filas.append({'D_m': d, 'banda': nombre, 'f_GHz': f / 1e9, 'theta3_deg': th,
                          'e_total_deg': SUPUESTOS['fraccion_haz'] * th,
                          'gain_dbi': model.ganancia_plato(d, f)})
    return filas


# -------------------------------------------------------- 3. presupuesto encoder
def presupuesto(e_total_deg, reparto=REPARTO_RSS):
    """Reparte e_total (rms) en fuentes independientes: e_i = e_total sqrt(f_i)
    (suma RSS). Devuelve dict fuente -> grados rms."""
    s = sum(reparto.values())
    assert abs(s - 1.0) < 1e-9, s
    return {k: e_total_deg * math.sqrt(f) for k, f in reparto.items()}


def encoder_necesario(e_total_deg, relacion_reductora=1.0, lado='carga'):
    """Resolucion del encoder para que la cuantizacion cumpla su parte del
    presupuesto. Cuantizador uniforme de paso q: rms = q/sqrt(12) (Widrow &
    Kollar, Quantization Noise, 2008) -> q = sqrt(12) e_enc.
    cuentas/vuelta de CARGA N = 360/q; bits = ceil(log2 N); un incremental de PPR
    pulsos da 4 PPR cuentas (cuadratura x4). Con el encoder en el MOTOR el lazo no
    ve la holgura, y las cuentas por vuelta de carga son N_motor * relacion."""
    e_enc = presupuesto(e_total_deg)['encoder (cuantizacion)']
    q = math.sqrt(12.0) * e_enc
    n_carga = 360.0 / q
    bits = math.ceil(math.log2(n_carga))
    n_enc = n_carga / relacion_reductora if lado == 'motor' else n_carga
    # margen practico de 2 bits (4x mas fino): el PID deriva la posicion y la cuantizacion
    # se amplifica (ruido_velocidad_por_cuantizacion); es una recomendacion de diseno
    bits_rec = bits + 2
    return {'e_enc_rms_deg': e_enc, 'q_deg': q, 'cuentas_vuelta_carga': n_carga,
            'bits_recomendados': bits_rec, 'q_recomendado_deg': 360.0 / 2 ** bits_rec,
            'ruido_vel_cota_deg_s': ruido_velocidad_por_cuantizacion(360.0 / 2 ** bits_rec),
            'bits_carga': bits, 'ppr_cuadratura_carga': n_carga / 4.0,
            'cuentas_vuelta_encoder': n_enc, 'ppr_encoder': n_enc / 4.0, 'lado': lado}


def tasa_cuentas(cuentas_vuelta_encoder, v_deg_s_encoder):
    """Cuentas por segundo en el eje del encoder (cuenta de bordes de cuadratura);
    el PCNT con filtro de glitch de 1 us (README del firmware) admite hasta ~2 Mcuentas/s:
    cada canal necesita semiperiodo > 1 us -> f_canal < 500 kHz -> 4 f = 2 Mcuentas/s."""
    return cuentas_vuelta_encoder * v_deg_s_encoder / 360.0


LIMITE_PCNT_CUENTAS_S = 2.0e6


def ruido_velocidad_por_cuantizacion(q_deg, dt_s=1e-3, tau_s=0.02):
    """Desviacion tipica (deg/s) de la velocidad medida por diferencia de dos
    cuentas filtrada por un polo de constante tau, suponiendo ruido de
    cuantizacion BLANCO de rms q/sqrt(12) (valido si la senal cruza varios
    escalones por muestra; con la montura casi parada es correlado, SUPUESTO):
        var_raw = 2 sigma_q^2/dt^2 ;  var_f = var_raw * dt/(2 tau)  ->  sigma_v = sigma_q / sqrt(dt tau).
    Con el filtro tau_vel de 20 ms del firmware."""
    sq = q_deg / math.sqrt(12.0)
    return sq / math.sqrt(dt_s * tau_s)


def verificar_ruido_velocidad(q_deg=0.02197, dt_s=1e-3, tau_s=0.02, v_deg_s=9.0, n=200000, semilla=1):
    """Contraste numerico de la formula anterior: rampa a v con encoder de paso q
    (fase aleatoria), diferencia, filtro de primer orden; devuelve (formula, medido).
    A v = 9 deg/s y q = 0.022 deg hay 0.4 escalones por muestra: la hipotesis de
    ruido blanco es FLOJA y se reporta el cociente."""
    rng = np.random.default_rng(semilla)
    t = np.arange(n) * dt_s
    th = v_deg_s * t + rng.uniform(0, q_deg)
    meas = np.round(th / q_deg) * q_deg
    d = np.diff(meas) / dt_s
    a = dt_s / (tau_s + dt_s)
    f = np.empty_like(d)
    acc = d[0]
    for i, x in enumerate(d):
        acc += a * (x - acc)
        f[i] = acc
    medido = float(np.std(f[2000:] - v_deg_s))
    return ruido_velocidad_por_cuantizacion(q_deg, dt_s, tau_s), medido


# ------------------------------------------------------------ 4. par y potencia
def par_viento(v_m_s, area_m2, cd, brazo_m, rho=1.225):
    """tau = 1/2 rho v^2 Cd A r (igual que `par_viento` de atp.py del curso:
    3 m, 0.15 m, 10 m/s -> 78 N m)."""
    return 0.5 * rho * v_m_s ** 2 * cd * area_m2 * brazo_m


def par_motor(e, v_deg_s, a_deg_s2):
    """Par en el EJE DE CARGA para un mando (v, a):
        T = J alfa + b omega + T_friccion + T_viento + T_desbalance, x margen.
    J, b: docs/CONTROL.md sec. 1 (J theta'' + b theta' = u + d). El viento y el
    desbalance (m g d) se toman en el peor caso, como suma (conservador)."""
    w = math.radians(v_deg_s)
    al = math.radians(a_deg_s2)
    t_vis = e['b'] * w
    t_in = e['J_kgm2'] * al
    t_vi = e['viento_nm'] if e.get('viento_nm') is not None else par_viento(
        SUPUESTOS['rafaga_m_s'], e['area_m2'], e['cd'], e['brazo_m'])
    t_sum = t_in + t_vis + e['fric_nm'] + t_vi + e['desbalance_nm']
    return {'inercial': t_in, 'viscoso': t_vis, 'friccion': e['fric_nm'], 'viento': t_vi,
            'desbalance': e['desbalance_nm'], 'total': t_sum,
            'con_margen': t_sum * SUPUESTOS['margen_par']}


def dimensionar_motor(e, v_deg_s, a_deg_s2, rpm_motor=3000.0):
    """Reductora, par de motor, potencia y corriente de bus.
    - reduccion N = w_motor / w_carga_max (con rpm nominal del motor, SUPUESTO).
    - par de motor = T_carga / (N eta).
    - potencia mecanica de pico P = T omega (la del tramo a vmax con el par de
      regimen: viscoso + viento + friccion + desbalance, sin el inercial, que ya
      acabo la aceleracion) y la de aceleracion (J alfa) * omega/2 media; se
      toma la mayor, dividida por eta.
    - corriente de bus I = P/(V eta_electrico~0.9)  (SUPUESTO eta_elec)."""
    pm = par_motor(e, v_deg_s, a_deg_s2)
    w = math.radians(v_deg_s)
    eta = SUPUESTOS['rend_reductor']
    N = rpm_motor * 2.0 * math.pi / 60.0 / w
    t_regimen = pm['total'] - pm['inercial']
    p_reg = t_regimen * SUPUESTOS['margen_par'] * w / eta
    p_acel = pm['con_margen'] * w / eta                # al final de la aceleracion (pico)
    p_pico = max(p_reg, p_acel)
    v_bus = SUPUESTOS['tension_bus_v']
    i_pico = p_pico / (v_bus * 0.9)
    return {'par_carga_nm': pm['con_margen'], 'reduccion': N,
            'par_motor_nm': pm['con_margen'] / (N * eta), 'potencia_pico_w': p_pico,
            'corriente_pico_a': i_pico, 'detalle_par': pm,
            'inercia_motor_reflejada_ok_N_opt': math.sqrt(e['J_kgm2'] / 1e-4)}   # N optimo con Jm=1e-4 kg m^2


# --------------------------------------------------- 5. lazo y latencia
def lazo_y_latencia(e_total_deg, v_deg_s, a_deg_s2, f_bw_hz=2.0):
    """Frecuencia del lazo y latencia ROS-MCU admisibles.
    - Retardo puro T con rampa a v: error v T. Parte de latencia del presupuesto:
      e_lat = e_total sqrt(0.10) -> T_max = e_lat / v (SIN compensar).
    - Con la extrapolacion del firmware (usa la velocidad de la referencia) el
      error residual es a T^2/2 -> T_max = sqrt(2 e_lat / a).
    - Frecuencia de muestreo: fs >= 20 f_bw (Franklin, Powell, Workman, Digital
      Control of Dynamic Systems, 3.a ed., cap. 6: 'muestrear 20 veces el ancho de
      banda'); el ZOH agrega Ts/2 de retardo.
    - Contra el presupuesto del lazo ROS (CONTROL sec. 2): margen de retardo del
      LQR 92.5 ms."""
    e_lat = e_total_deg * math.sqrt(REPARTO_RSS['latencia y sellado de tiempo'])
    return {'e_lat_deg': e_lat,
            'T_max_sin_comp_ms': 1e3 * e_lat / v_deg_s,
            'T_max_con_extrapolacion_ms': 1e3 * math.sqrt(2.0 * e_lat / a_deg_s2),
            'fs_min_hz': 20.0 * f_bw_hz,
            'margen_retardo_lqr_ms': 92.5}


# ------------------------------------------------- 6. RF: enlace, Doppler y banco conducido
BANDAS_RF_HZ = {                       # frecuencias de bajada tipicas (SUPUESTO: verificar en SatNOGS DB)
    'VHF 137 MHz (APT/LRPT meteo)': 137.5e6,
    'VHF 145.8 MHz (ISS, CubeSats)': 145.8e6,
    'UHF 437 MHz (CubeSats; model.BANDAS)': 437e6,
    'S 2.2 GHz (model.BANDAS)': 2.2e9,
}


def doppler_pase(h_km, f_hz, el_max_deg=89.9, mascara_deg=model.MASCARA_DEG):
    """Doppler maximo (Hz) y tasa maxima (Hz/s) de un pase (geometria esferica, sin rotacion
    terrestre: la rotacion cambia vr en ~ +-6 %). fd = -vr/c f0 (model.doppler_hz)."""
    p = model.perfil_pase(h_km, el_max_deg, mascara_deg, n=20000)
    fd = model.doppler_hz(p['vr'], f_hz)
    tasa = np.gradient(fd, p['t'])
    return {'fd_max_hz': float(np.max(np.abs(fd))), 'tasa_max_hz_s': float(np.max(np.abs(tasa))),
            'tasa_cenit_analitica_hz_s': model.tasa_doppler_cenit(h_km, f_hz)}


def intervalo_resintonia(tasa_hz_s, tolerancia_hz):
    """Cada cuantos segundos hay que re-sintonizar el receptor con el Doppler predicho para que
    el residuo no pase de `tolerancia_hz`: T = tol / tasa (el Doppler cambia linealmente en T)."""
    return float(tolerancia_hz / tasa_hz_s)


def c_n0_enlace(f_hz, eirp_dbw, g_rx_dbi, t_sys_k, d_km, perdidas_db=3.0, perdida_apunt_db=0.0):
    """C/N0 (dB-Hz) = EIRP - FSPL - L + G/T + 228.6 (model.presupuesto_enlace, mismos terminos)."""
    gt = g_rx_dbi - 10.0 * math.log10(t_sys_k)
    return float(eirp_dbw - model.fspl_db(d_km, f_hz) - perdidas_db - perdida_apunt_db + gt
                 + model.K_BOLTZMANN_DB)


def rango_pase_km(h_km, el_deg):
    return float(model.rango_oblicuo(h_km, el_deg))


def banco_conducido(c_n0_dbhz, fs_hz, nf_db, p_tx_compuesta_dbm, exceso_ruido_db=20.0):
    """Camino conducido SDR TX -> atenuadores -> SDR RX. El ruido del canal se INYECTA en banda
    base (como `atp_sdr.senal`: piso fijo y amplitud fijada por C/N0), de modo que el nivel fisico
    sea alto y no haga falta aislar 120 dB:
      - ruido termico del receptor: N_rx = -174 + NF + 10 log10(fs) [dBm]
        (-174 dBm/Hz = 10 log10(k T0 / 1 mW), T0 = 290 K)
      - el ruido inyectado debe superar al termico en `exceso_ruido_db`, para que el C/N0 lo
        fije el canal digital y no el receptor: P_rx_ruido = N_rx + exceso
      - la senal es C/N0 sobre esa densidad: P_senal = P_rx_ruido - 10 log10(fs) + C/N0 [dBm]
      - la potencia COMPUESTA en el receptor es ~ P_rx_ruido (la senal esta bajo el ruido)
      - atenuacion total necesaria: A = P_tx_compuesta - P_rx_ruido
    Devuelve esos valores (dBm, dB)."""
    n_rx = -174.0 + nf_db + 10.0 * math.log10(fs_hz)
    p_rx_ruido = n_rx + exceso_ruido_db
    n0_inyectada = p_rx_ruido - 10.0 * math.log10(fs_hz)          # dBm/Hz
    p_senal = n0_inyectada + c_n0_dbhz                              # dBm
    return {'n_termico_rx_dbm': n_rx, 'p_rx_compuesta_dbm': p_rx_ruido, 'n0_inyectada_dbm_hz': n0_inyectada,
            'p_senal_dbm': p_senal, 'atenuacion_total_db': p_tx_compuesta_dbm - p_rx_ruido,
            'snr_banda_db': p_senal - p_rx_ruido}


def aislamiento_necesario(p_tx_dbm, p_rx_dbm, margen_db=20.0):
    """Fuga TX->RX que NO atraviesa los atenuadores (radiada, por masa/USB): debe quedar
    `margen_db` por debajo de la portadora conducida: aislamiento >= P_tx - (P_rx - margen)."""
    return float(p_tx_dbm - (p_rx_dbm - margen_db))


# --------------------------------------------------------------- informe
def calcular(rapido=False):
    r = {'supuestos': SUPUESTOS, 'reparto_rss': REPARTO_RSS}
    r['cinematica'] = [cinematica_pase(h, el, n=40000) for h in ALTURAS_KM for el in ELMAX_GRID]
    r['fraccion_sobre'] = {f'{h:.0f}': {f'{el:.0f}': fraccion_pases_sobre(h, el)
                                         for el in (60.0, 70.0, 80.0, 85.0, 88.0)}
                           for h in ALTURAS_KM}
    r['keyhole'] = tabla_keyhole()
    r['cenital_con_tope'] = [cenital_con_tope(550.0, v, a)
                             for v, a in ((6.0, 2.0), (20.0, 40.0), (30.0, 60.0), (90.0, 400.0))]
    r['el_180'] = elevacion_hasta_180(550.0)
    r['slew'] = {f'{d:.0f}deg_{t:.0f}s': {f'a{a:.0f}': v_slew_necesaria(d, t, a) for a in (10.0, 30.0, 100.0)}
                 for d in (90.0, 180.0) for t in (10.0, 20.0, 30.0)}
    if not rapido:
        filas, por_sat = pases_sgp4()
        r['sgp4_n'] = len(filas)
        r['sgp4_dist'] = distribucion_por_elmax(filas)
        r['sgp4_max'] = {k: float(max(f[k] for f in filas)) for k in ('v_az', 'v_el', 'a_az', 'a_el', 'el_max')}
        r['sgp4_cenital'] = pase_cenital_sgp4()
        r['handover'] = re_apuntamiento(por_sat)
    r['platos'] = tabla_platos()
    r['escenarios'] = {}
    for nombre, e in ESCENARIOS.items():
        th = theta3_escenario(e)
        e_tot = e.get('e_total_deg') or SUPUESTOS['fraccion_haz'] * th
        enc_carga = encoder_necesario(e_tot)
        enc_motor = encoder_necesario(e_tot, relacion_reductora=10.0, lado='motor')
        v, a = e['v_az_max'], e['a_az_max']
        mot = dimensionar_motor(e, v, a)
        lat = lazo_y_latencia(e_tot, min(v, 9.0), a)
        r['escenarios'][nombre] = {
            'theta3_deg': th, 'e_total_deg': e_tot, 'presupuesto': presupuesto(e_tot),
            'encoder_carga': enc_carga, 'encoder_motor_10a1': enc_motor,
            'tasa_cuentas_carga': tasa_cuentas(enc_carga['cuentas_vuelta_carga'], v),
            'motor': mot, 'lazo': lat,
        }
        if e.get('pasos'):
            r['escenarios'][nombre]['pasos'] = resolucion_paso_a_paso(e_tot, v)
    r['rf'] = {
        'doppler': {nom: {f'{h:.0f}': doppler_pase(h, f) for h in ALTURAS_KM} for nom, f in BANDAS_RF_HZ.items()},
        'resintonia_s': {nom: {f'{tol:.0f}': intervalo_resintonia(doppler_pase(550.0, f)['tasa_max_hz_s'], tol)
                               for tol in (100.0, 500.0, 1000.0)} for nom, f in BANDAS_RF_HZ.items()},
        'yagi_437': {f'{el:.0f}': c_n0_enlace(437e6, 0.0, 12.0, 300.0, rango_pase_km(550.0, el))
                     for el in (5.0, 30.0, 90.0)},
        'conducido_uhf': banco_conducido(45.0, 2.4e6, 5.0, -20.0),
    }
    r['ruido_vel'] = {'q_14bit': 360.0 / 2 ** 14,
                      'formula_deg_s': ruido_velocidad_por_cuantizacion(360.0 / 2 ** 14),
                      'medido_deg_s': verificar_ruido_velocidad()[1]}
    return r


def resolucion_paso_a_paso(e_total_deg, v_deg_s, pasos_rev=200, micropaso=16):
    """Paso a paso en lazo abierto: paso del eje de carga = 360/(pasos*micropaso*N).
    Se busca la menor N entera que cumple q <= sqrt(12) e_enc (misma regla que el
    encoder) y se da la frecuencia de STEP a v_max."""
    e_enc = presupuesto(e_total_deg)['encoder (cuantizacion)']
    q_obj = math.sqrt(12.0) * e_enc
    N = math.ceil(360.0 / (pasos_rev * micropaso * q_obj))
    q = 360.0 / (pasos_rev * micropaso * N)
    return {'pasos_rev': pasos_rev, 'micropaso': micropaso, 'reduccion': N, 'q_deg': q,
            'f_step_hz_a_vmax': v_deg_s / q}


def _f(x, n=3):
    return f'{x:.{n}f}'


def imprimir(r):
    p = print
    p('## Cinematica de pase (geometria esferica, `model.perfil_pase`)\n')
    p('| h (km) | el_max | v_az (deg/s) | v_az analitica | a_az (deg/s^2) | v_el | a_el |')
    p('|---|---|---|---|---|---|---|')
    for c in r['cinematica']:
        p(f"| {c['h_km']:.0f} | {c['el_max']:.0f} | {_f(c['v_az'],2)} | {_f(c['v_az_analitica'],2)} | "
          f"{_f(c['a_az'],2)} | {_f(c['v_el'],2)} | {_f(c['a_el'],2)} |")
    p('\n## Clases de montura: hasta que el_max llegan\n')
    p('| clase | vmax az | amax az | h | el_max (vel.) | el_max (acel.) | el_max alcanzable | pases perdidos |')
    p('|---|---|---|---|---|---|---|---|')
    for k in r['keyhole']:
        p(f"| {k['clase']} | {k['vmax']:.0f} | {k['amax']:.0f} | {k['h_km']:.0f} | "
          f"{_f(k['el_max_por_velocidad'],1)} | {_f(k['el_max_por_aceleracion'],1)} | "
          f"{_f(k['el_max_alcanzable'],1)} | {100*k['fraccion_pases_perdidos']:.1f} % |")
    p('\n## Pase cenital (550 km) con tope de velocidad y aceleracion\n')
    for c in r['cenital_con_tope']:
        p(f"- v<={c['v_cap']:.0f} deg/s, a<={c['a_cap']:.0f} deg/s^2: error de acimut max {_f(c['err_az_max_deg'],1)} deg, "
          f"en el cielo {_f(c['err_cielo_max_deg'],2)} deg, {_f(c['t_fuera_1deg_s'],1)} s fuera de 1 deg")
    p(f"\nMontura sobre el cenit (el hasta 180): v_el pico {_f(r['el_180']['v_el_pico'],2)} deg/s "
      f"(analitica v/h {_f(r['el_180']['v_el_analitica'],2)})")
    if 'sgp4_dist' in r:
        p(f"\n## Pases SGP4 reales ({r['sgp4_n']} pases en 72 h, CDMX)\n")
        for d in r['sgp4_dist']:
            if d['n']:
                p(f"- el_max {d['rango'][0]}-{d['rango'][1]}: n={d['n']}, v_az p50 {_f(d['v_az_p50'],2)} max {_f(d['v_az_max'],2)}, "
                  f"a_az max {_f(d['a_az_max'],2)}, v_el max {_f(d['v_el_max'],2)}")
        for c in r['sgp4_cenital']:
            p(f"- cenital sintetico miss {c['miss_km']:.0f} km: el_max {_f(c['el_max'],1)}, v_az SGP4 {_f(c['v_az_sgp4'],1)} / esferico {_f(c['v_az_esferico'],1)}")
        p(f"- handover: {r['handover']}")
    p('\n## Platos\n')
    for t in r['platos']:
        p(f"- D {t['D_m']:.0f} m {t['banda']}: theta3 {_f(t['theta3_deg'],3)} deg, e_total {_f(t['e_total_deg'],4)} deg")
    for nombre, e in r['escenarios'].items():
        p(f'\n## Escenario {nombre}\n')
        p(f"theta3 {e['theta3_deg']} e_total {e['e_total_deg']}")
        p(json.dumps(e, indent=1, default=float))
    p(f"\nruido de velocidad por cuantizacion: {r['ruido_vel']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--json', help='guarda el resultado completo en este archivo')
    ap.add_argument('--rapido', action='store_true', help='sin SGP4')
    a = ap.parse_args(argv)
    r = calcular(rapido=a.rapido)
    imprimir(r)
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(r, f, indent=1, default=lambda o: o.isoformat() if hasattr(o, 'isoformat') else float(o))
    return 0


if __name__ == '__main__':
    sys.exit(main())
