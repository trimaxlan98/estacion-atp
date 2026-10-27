#!/usr/bin/env python3
"""Estudio R7 del enlace del rover: volumen de datos por sol/dia y latencia de comando (docs/ROVER.md «Resultados R7»).

    python3 scripts/estudio_enlace_rover.py --rapido       # pocos soles, ~20 s; escribe en runs/rover_enlace_rapido/
    python3 scripts/estudio_enlace_rover.py                # completo (~2 min); escribe docs/rover_resultados/enlace.{json,md}

Todo es simulacion con los supuestos de `atp_rover/comunicacion.py` (potencia y antenas del rover supuestas; orbitas keplerianas
sin precesion; retardo de luz de orbitas circulares coplanares). Determinista: dos corridas dan los mismos bytes.
"""
import argparse
import json
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_rover'))
from atp_rover import comunicacion as C  # noqa: E402
from atp_rover.terreno import Terreno  # noqa: E402

GALE = (-4.6, 137.4)          # lat, lon del crater Gale (Curiosity)
FECHA = '2026-10-07'
# Cifras publicadas (guia JPL «Mars Relay Description for Discovery 2019 Proposals», tabla 2-1: sesiones de Curiosity el primer ano)
PUBLICADO = {
    'odyssey': dict(sesiones_dia=2.2, mbit_dia=173.0, mbit_sesion=78.5),
    'mro': dict(sesiones_dia=2.3, mbit_dia=498.2, mbit_sesion=221.2),
    'total': dict(sesiones_dia=4.5, mbit_dia=671.3),
}
FUENTES = [
    ('Mars Relay Description for Discovery 2019 Proposals (JPL, 10-abr-2019), tabla 2-1 y tablas de orbitas 4.1-1, 5.1-1, 6.1-1 y umbrales de Electra 6.2-2',
     'https://discovery.larc.nasa.gov/PDF_FILES/21_Proposers_Guide_To_Mars_Orbiters_-_Discovery_2019_AO_-_Rev_190411b.pdf'),
    ('Communications for the Planet Mars: Past, Present, and Future (arXiv 2211.14245): ~500 bps a 32 kbps directo; 100-250 Mbit por orbitador y sol',
     'https://arxiv.org/pdf/2211.14245'),
    ('Mars Relay Network (NASA Science)', 'https://science.nasa.gov/mars/mars-relay-network/'),
]


def _terrenos():
    return {
        'llano': None,
        'rocoso': Terreno('rocoso', 1),
        'crater_15x3': Terreno('crater', 1, cx=0.0, cy=0.0, radio=15.0, profundidad=3.0),
        'crater_15x8': Terreno('crater', 1, cx=0.0, cy=0.0, radio=15.0, profundidad=8.0),
    }


def _stats(a):
    a = np.asarray(a, dtype=float)
    return dict(media=float(a.mean()), min=float(a.min()), max=float(a.max()), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)))


def latencia_comando(e, dias, n_dias_horizonte, paso_s=1200.0, vias=('rele',)):
    """Estadisticos de la latencia de un comando [min] y de la ESPERA (latencia menos retardo de luz) sobre `dias`."""
    dia = e.cuerpo['dia_s']
    hor = n_dias_horizonte * dia
    lat, esp = [], []
    for t in np.arange(0.0, dias * dia, paso_s):
        ta, via = e.entrega_comando(float(t), hor, vias=vias)
        if ta is None:
            continue
        lat.append((ta - t) / 60.0)
        esp.append((ta - t - e._owlt(float(t))) / 60.0)
    if not lat:
        return None
    return dict(latencia_min=_stats(lat), espera_min=_stats(esp), n=len(lat))


def e1_base(dias):
    """Marte, Gale: los tres reles, antena helicoidal, contra la cifra publicada."""
    out = {}
    for rele in ('odyssey', 'mro', 'maven'):
        e = C.Enlace('marte', rele, *GALE, antena='omni_helice', jd0=C.jd_de_iso(FECHA))
        vol, npas = e.volumen_por_dia(dias)
        ps = e.pases(0.0, dias * e.cuerpo['dia_s'])
        out[rele] = dict(mbit_dia=_stats(vol / 1e6), sesiones_dia=float(npas.mean()),
                         mbit_sesion=float(np.mean([p['bits'] for p in ps]) / 1e6) if ps else 0.0,
                         dur_min=float(np.mean([p['dur_s'] for p in ps]) / 60.0) if ps else 0.0,
                         el_max_media_deg=float(np.mean([p['el_max_deg'] for p in ps])) if ps else 0.0,
                         publicado=PUBLICADO.get(rele))
    out['total_odyssey_mro'] = dict(mbit_dia=out['odyssey']['mbit_dia']['media'] + out['mro']['mbit_dia']['media'],
                                    sesiones_dia=out['odyssey']['sesiones_dia'] + out['mro']['sesiones_dia'],
                                    publicado=PUBLICADO['total'])
    return out


def e2_volumen_vs_elevacion(dias):
    """Bits por pase segun la elevacion maxima del pase (MRO, Gale)."""
    e = C.Enlace('marte', 'mro', *GALE, antena='omni_helice')
    ps = e.pases(0.0, dias * e.cuerpo['dia_s'])
    bins = [(10, 25), (25, 40), (40, 55), (55, 70), (70, 90)]
    filas = []
    for lo, hi in bins:
        s = [p for p in ps if lo <= p['el_max_deg'] < hi]
        filas.append(dict(el_max_deg=f'{lo}-{hi}', pases=len(s),
                          mbit_pase=float(np.mean([p['bits'] for p in s]) / 1e6) if s else None,
                          dur_min=float(np.mean([p['dur_s'] for p in s]) / 60.0) if s else None,
                          tasa_pico_kbps=float(np.mean([p['tasa_pico_bps'] for p in s]) / 1e3) if s else None))
    return dict(rele='mro', pases_total=len(ps), filas=filas)


def e3_inclinacion(dias, dias_cmd):
    """Inclinacion del relé tipo MRO (resto de elementos igual) a dos latitudes: pases, volumen y espera de comando (via relé)."""
    out = []
    for lat in (-4.6, 60.0):
        for inc in (30.0, 60.0, 74.8, 92.7):
            e = C.Enlace('marte', 'mro', lat, GALE[1], rele_params=dict(inc_deg=inc), jd0=C.jd_de_iso(FECHA), dsn_fraccion=1.0)
            vol, npas = e.volumen_por_dia(dias)
            out.append(dict(lat_deg=lat, inc_deg=inc, sesiones_dia=float(npas.mean()), mbit_dia=_stats(vol / 1e6),
                            comando=latencia_comando(e, dias_cmd, dias_cmd + 4)))
    return out


def e4_elevacion_minima(dias, dias_cmd):
    out = []
    for el in (5.0, 10.0, 15.0, 20.0, 30.0, 45.0):
        e = C.Enlace('marte', 'mro', *GALE, el_min_deg=el, jd0=C.jd_de_iso(FECHA), dsn_fraccion=1.0)
        e3 = C.Enlace('marte', 'mro', *GALE, el_min_deg=el, jd0=C.jd_de_iso(FECHA), dsn_fraccion=1.0 / 3.0)
        vol, npas = e.volumen_por_dia(dias)
        out.append(dict(el_min_deg=el, sesiones_dia=float(npas.mean()), mbit_dia=_stats(vol / 1e6),
                        comando=latencia_comando(e, dias_cmd, dias_cmd + 4),
                        comando_turno_dsn=latencia_comando(e3, dias_cmd, dias_cmd + 4)))
    return out


def e5_relieve(dias, dias_cmd):
    out = []
    for nombre, ter in _terrenos().items():
        e = C.Enlace('marte', 'mro', *GALE, terreno=ter, jd0=C.jd_de_iso(FECHA), dsn_fraccion=1.0)
        vol, npas = e.volumen_por_dia(dias)
        out.append(dict(terreno=nombre, horizonte_medio_deg=float(np.mean(e.mascara)), horizonte_max_deg=float(np.max(e.mascara)),
                        sesiones_dia=float(npas.mean()), mbit_dia=_stats(vol / 1e6),
                        comando=latencia_comando(e, dias_cmd, dias_cmd + 4)))
    return out


def e6_antenas(dias):
    """Antena UHF del rover (con MRO, llano y cráter) y sensibilidad a la ganancia (+-3, +-6 dB sobre la helicoidal)."""
    tipos = [('omni_lineal', {}), ('omni_helice', {}), ('direccional_10dBi_5deg', dict(antena='direccional', antena_params=dict(error_apunt_deg=5.0))),
             ('direccional_10dBi_25deg', dict(antena='direccional', antena_params=dict(error_apunt_deg=25.0))),
             ('direccional_10dBi_5deg_g-6dB', dict(antena='direccional', antena_params=dict(error_apunt_deg=5.0), ganancia_extra_db=-6.0))]
    filas = []
    for nombre, kw in tipos:
        kw = dict(kw)
        kw.setdefault('antena', nombre)
        for terr in ('llano', 'crater_15x3'):
            e = C.Enlace('marte', 'mro', *GALE, terreno=_terrenos()[terr], **kw)
            vol, npas = e.volumen_por_dia(dias)
            filas.append(dict(antena=nombre, terreno=terr, mbit_dia=float(vol.mean() / 1e6), sesiones_dia=float(npas.mean())))
    sens = []
    for g in (-9.0, -6.0, -3.0, 0.0, 3.0, 6.0):
        e = C.Enlace('marte', 'mro', *GALE, antena='omni_helice', ganancia_extra_db=g)
        vol, npas = e.volumen_por_dia(dias)
        sens.append(dict(delta_db=g, mbit_dia=float(vol.mean() / 1e6)))
    return dict(antenas=filas, sensibilidad_ganancia=sens)


def e7_directo_x(n_pts=14, horas_dia=None):
    """Banda X directa a Tierra durante un ciclo sinodico (~780 d): tasa y Mbit/sol segun la distancia, HGA y LGA."""
    hdia = C.BANDA_X['horas_dia'] if horas_dia is None else horas_dia
    e = C.Enlace('marte', 'mro', *GALE)
    jd0 = C.jd_de_iso('2026-01-01')
    out = []
    for k in range(n_pts):
        jd = jd0 + k * 780.0 / n_pts
        d = float(C.distancia_tierra_km('marte', jd))
        r_h = float(e.tasa_x_bps(d, C.ganancia_rover_x_dbi('hga_x')))
        r_l = float(e.tasa_x_bps(d, C.ganancia_rover_x_dbi('lga_x')))
        out.append(dict(dias=round(k * 780.0 / n_pts), dist_ua=d / C.UA_KM, owlt_min=d / C.C_KM_S / 60.0, hga_kbps=r_h / 1e3,
                        lga_kbps=r_l / 1e3, hga_mbit_sol=r_h * hdia * 3600 / 1e6, lga_mbit_sol=r_l * hdia * 3600 / 1e6))
    return dict(horas_dia=hdia, filas=out)


def e8_luna(dias):
    filas = []
    sitios = [('polo sur', -89.5, 0.0), ('ecuador cara visible', 0.0, 0.0), ('ecuador cara oculta', 0.0, 180.0)]
    for rele in ('polar_100km', 'nrho_kepler'):
        for nombre, lat, lon in sitios:
            e = C.Enlace('luna', rele, lat, lon, antena='omni_helice', jd0=C.jd_de_iso(FECHA), dsn_fraccion=1.0)
            dur = dias * 86400.0
            t = 120.0 * np.arange(int(dur / 120.0))
            m = e.muestras(t)
            ps = e.pases(0.0, dur, 120.0)
            vol = sum(p['bits'] for p in ps) / dias
            filas.append(dict(rele=rele, sitio=nombre, cobertura_rele=float(m['visible'].mean()), pases_dia=len(ps) / dias,
                              mbit_dia=float(vol / 1e6), tierra_visible=float(m['tierra_visible'].mean()),
                              comando_via_rele=latencia_comando(e, 4, 8, paso_s=3600.0, vias=('rele',)),
                              comando_directo=latencia_comando(e, 4, 8, paso_s=3600.0, vias=('directo',))))
    return filas


def e9_dtn(dias):
    out = []
    for gen in (100.0, 300.0, 500.0, 800.0):
        e = C.Enlace('marte', 'mro', *GALE, jd0=C.jd_de_iso(FECHA))
        out.append(dict(gen_mbit_dia=gen, **C.simular_dtn(e, dias * e.cuerpo['dia_s'], gen * 1e6, semilla=1)))
    return out


def _f(x, n=1):
    return '-' if x is None else f'{x:.{n}f}'


def _lat(c, clave='latencia_min', est='p50'):
    return '-' if not c else _f(c[clave][est], 0)


def a_markdown(R):
    L = ['# Estudio R7: enlace del rover por antenas',
         '', 'Generado por `python3 scripts/estudio_enlace_rover.py`. **Simulacion con los supuestos de `atp_rover/comunicacion.py`**: '
         'potencia (7 W), antenas y perdidas del rover SUPUESTAS; orbitas keplerianas sin precesion (datos de la guia JPL 2019); '
         'umbrales de Electra de la misma guia (signo interpretado como dBm negativos); retardo de luz con orbitas circulares coplanares. '
         f'Fecha de partida {FECHA}; sitio Gale (4.6 S, 137.4 E) salvo que se diga otra cosa. No hay validacion contra telemetria real.', '']
    b = R['e1']
    L += ['## E1. Volumen diario por relé frente a la cifra publicada (Marte, Gale, antena helicoidal)', '',
          f'{R["dias"]} soles simulados. «Publicado» = primer año de Curiosity (guia JPL 2019, tabla 2-1; el rover real usa UHF Electra-Lite y '
          'un plan de contactos que aqui no se reproduce).', '',
          '| Relé | Sesiones/sol (sim.) | Publicado | Mbit/sol (sim.: media [min-max]) | Publicado | Mbit/sesion (sim.) | Publicado | Duracion media (min) | El. max media (deg) |',
          '|---|---|---|---|---|---|---|---|---|']
    for r in ('odyssey', 'mro', 'maven'):
        x = b[r]
        p = x['publicado'] or {}
        L.append(f"| {r} | {_f(x['sesiones_dia'], 2)} | {_f(p.get('sesiones_dia'), 1)} | {_f(x['mbit_dia']['media'], 0)} "
                 f"[{_f(x['mbit_dia']['min'], 0)}-{_f(x['mbit_dia']['max'], 0)}] | {_f(p.get('mbit_dia'), 0)} | {_f(x['mbit_sesion'], 0)} | "
                 f"{_f(p.get('mbit_sesion'), 0)} | {_f(x['dur_min'], 1)} | {_f(x['el_max_media_deg'], 0)} |")
    t = b['total_odyssey_mro']
    L += ['', f"Odyssey + MRO: {_f(t['mbit_dia'], 0)} Mbit/sol y {_f(t['sesiones_dia'], 2)} sesiones/sol en la simulacion; publicado {_f(t['publicado']['mbit_dia'], 1)} Mbit/dia y "
          f"{_f(t['publicado']['sesiones_dia'], 1)} sesiones/dia. MAVEN no tiene cifra comparable (23 sesiones en el periodo). "
          'El acuerdo en el orden de magnitud NO valida el modelo: el plan real de contactos, la potencia real y las perdidas de Curiosity no se conocen aqui, '
          'y el volumen simulado varia de un sol a otro por la geometria del nodo (ver min-max).', '']
    e2 = R['e2']
    L += ['## E2. Volumen por pase frente a la elevacion maxima (MRO)', '', '| El. max del pase (deg) | Pases | Mbit/pase | Duracion (min) | Tasa pico (kbps) |', '|---|---|---|---|---|']
    for f in e2['filas']:
        L.append(f"| {f['el_max_deg']} | {f['pases']} | {_f(f['mbit_pase'], 0)} | {_f(f['dur_min'], 1)} | {_f(f['tasa_pico_kbps'], 0)} |")
    L += ['']
    L += ['## E3. Inclinacion orbital (relé tipo MRO con la inclinacion indicada) y latencia de comando por el relé', '',
          'Latencia = retardo de luz + espera de la ventana relé-Tierra (aqui la DSN siempre disponible: solo cuenta que el relé no esté ocultado por Marte) y del siguiente contacto UHF; solo via relé. '
          'Comandos cada 20 min durante varios soles. Se muestra la ESPERA (sin retardo de luz) en minutos: mediana / p95.', '',
          '| Lat (deg) | Inc (deg) | Sesiones/sol | Mbit/sol | Espera mediana (min) | Espera p95 (min) |', '|---|---|---|---|---|---|']
    for f in R['e3']:
        L.append(f"| {f['lat_deg']} | {f['inc_deg']} | {_f(f['sesiones_dia'], 2)} | {_f(f['mbit_dia']['media'], 0)} | "
                 f"{_lat(f['comando'], 'espera_min', 'p50')} | {_lat(f['comando'], 'espera_min', 'p95')} |")
    L += ['', '## E4. Elevacion minima de uso del relé (MRO, Gale)', '', '| El. min (deg) | Sesiones/sol | Mbit/sol | Espera mediana (min) | Espera p95 (min) | Espera mediana con turno DSN de 1/3 de sol (min) |', '|---|---|---|---|---|---|']
    for f in R['e4']:
        L.append(f"| {f['el_min_deg']:.0f} | {_f(f['sesiones_dia'], 2)} | {_f(f['mbit_dia']['media'], 0)} | "
                 f"{_lat(f['comando'], 'espera_min', 'p50')} | {_lat(f['comando'], 'espera_min', 'p95')} | "
                 f"{_lat(f['comando_turno_dsn'], 'espera_min', 'p50')} |")
    L += ['', 'El turno de la DSN (1/3 de sol al comienzo de cada sol) es un SUPUESTO arbitrario y domina la espera cuando se activa; '
          'las demás tablas usan la DSN siempre disponible para aislar la geometría.']
    L += ['', '## E5. Relieve local (horizonte por oclusion del terreno de R1, rover en el centro)', '',
          '| Terreno | Horizonte medio (deg) | Horizonte max (deg) | Sesiones/sol | Mbit/sol | Espera mediana (min) | Espera p95 (min) |', '|---|---|---|---|---|---|---|']
    for f in R['e5']:
        L.append(f"| {f['terreno']} | {_f(f['horizonte_medio_deg'], 1)} | {_f(f['horizonte_max_deg'], 1)} | {_f(f['sesiones_dia'], 2)} | "
                 f"{_f(f['mbit_dia']['media'], 0)} | {_lat(f['comando'], 'espera_min', 'p50')} | {_lat(f['comando'], 'espera_min', 'p95')} |")
    L += ['', 'El horizonte se calcula con el relieve de `Terreno.altura` hasta 60 m de la antena (a 1 m del suelo). Un cráter real mide cientos de metros: este '
          'caso es el rover en un hoyo pequeño, no en un cráter de decenas de km.', '']
    e6 = R['e6']
    L += ['## E6. Antena UHF del rover y sensibilidad a la ganancia (MRO)', '', '| Antena | Terreno | Mbit/sol | Sesiones/sol |', '|---|---|---|---|']
    for f in e6['antenas']:
        L.append(f"| {f['antena']} | {f['terreno']} | {_f(f['mbit_dia'], 0)} | {_f(f['sesiones_dia'], 2)} |")
    L += ['', '| Delta ganancia sobre la helicoidal (dB) | Mbit/sol |', '|---|---|']
    for f in e6['sensibilidad_ganancia']:
        L.append(f"| {f['delta_db']:+.0f} | {_f(f['mbit_dia'], 0)} |")
    base = [f for f in e6['sensibilidad_ganancia'] if f['delta_db'] == 0.0][0]['mbit_dia']
    mas6 = [f for f in e6['sensibilidad_ganancia'] if f['delta_db'] == 6.0][0]['mbit_dia']
    menos6 = [f for f in e6['sensibilidad_ganancia'] if f['delta_db'] == -6.0][0]['mbit_dia']
    L += ['', f'Sensibilidad: +6 dB da {mas6 / base:.2f}x y -6 dB da {menos6 / base:.2f}x el volumen de la helicoidal. ', '']
    x = R['e7']
    L += [f'## E7. Banda X directa a Tierra a lo largo de un ciclo sinodico ({x["horas_dia"]:.0f} h por sol de transmision, SUPUESTO)', '',
          '| Dia | Distancia (UA) | Retardo una via (min) | HGA (kbps) | LGA (kbps) | HGA (Mbit/sol) | LGA (Mbit/sol) |', '|---|---|---|---|---|---|---|']
    for f in x['filas']:
        L.append(f"| {f['dias']} | {f['dist_ua']:.2f} | {f['owlt_min']:.1f} | {f['hga_kbps']:.1f} | {f['lga_kbps']:.2f} | {f['hga_mbit_sol']:.0f} | {f['lga_mbit_sol']:.1f} |")
    L += ['', 'La literatura cita 500 bps a 32 kbps directos para Curiosity (arXiv 2211.14245); la escalera simulada es 0.5-32 kbps. '
          'Los Mbit/sol directos asumen el rover transmitiendo `horas_dia` con la Tierra sobre el horizonte; la energia que cuesta no se modela.', '',
          '## E8. Luna (banda S supuesta; relé polar circular de 100 km y «NRHO» kepleriana)', '',
          '| Relé | Sitio | Cobertura del relé | Pases/dia | Mbit/dia | Tierra visible | Espera mediana comando por relé (min) | por X directo (min) |', '|---|---|---|---|---|---|---|---|']
    for f in R['e8']:
        L.append(f"| {f['rele']} | {f['sitio']} | {f['cobertura_rele'] * 100:.1f} % | {_f(f['pases_dia'], 1)} | {_f(f['mbit_dia'], 0)} | {f['tierra_visible'] * 100:.0f} % | "
                 f"{_lat(f['comando_via_rele'], 'espera_min', 'p50')} | {_lat(f['comando_directo'], 'espera_min', 'p50')} |")
    L += ['', 'La «NRHO» es una elipse de dos cuerpos con el periodo y el perilunio de la NRHO 9:2: NO es la NRHO (problema de tres cuerpos). '
          'La banda S, las potencias y la tasa del relé a la Tierra de la Luna son supuestos propios. Cobertura del 100 % no significa enlace util: mira los Mbit/dia.', '',
          '## E9. Telemetria con DTN (MRO, Gale)', '', 'Bundles de 8 Mbit, perdida de bundle 1 %, relé a la Tierra con turno DSN de 1/3 de sol.', '',
          '| Generado (Mbit/sol) | Entregado (Mbit/sol) | Latencia mediana (h) | p95 (h) | Retransmisiones | En cola rover | En cola relé |', '|---|---|---|---|---|---|---|']
    for f in R['e9']:
        L.append(f"| {f['gen_mbit_dia']:.0f} | {f['bits_entregados_por_dia'] / 1e6:.0f} | {_f(f['latencia_mediana_h'], 1)} | {_f(f['latencia_p95_h'], 1)} | "
                 f"{f['retransmisiones']} | {f['en_cola_rover']} | {f['en_cola_rele']} |")
    L += ['', '## Fuentes', '']
    for t_, u in FUENTES:
        L.append(f'- [{t_}]({u})')
    return '\n'.join(L) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true')
    ap.add_argument('--salida', default=None)
    a = ap.parse_args(argv)
    dias, dcmd = (6, 3) if a.rapido else (20, 8)
    salida = a.salida or (os.path.join(RAIZ, 'runs', 'rover_enlace_rapido') if a.rapido else os.path.join(RAIZ, 'docs', 'rover_resultados'))
    os.makedirs(salida, exist_ok=True)
    R = dict(dias=dias, dias_comando=dcmd, fecha=FECHA, rapido=a.rapido)
    R['e1'] = e1_base(dias)
    R['e2'] = e2_volumen_vs_elevacion(dias * 2)
    R['e3'] = e3_inclinacion(dias, dcmd)
    R['e4'] = e4_elevacion_minima(dias, dcmd)
    R['e5'] = e5_relieve(dias, dcmd)
    R['e6'] = e6_antenas(dias)
    R['e7'] = e7_directo_x()
    R['e8'] = e8_luna(10 if a.rapido else 28)
    R['e9'] = e9_dtn(dias)
    with open(os.path.join(salida, 'enlace.json'), 'w') as f:
        json.dump(R, f, indent=1, sort_keys=True)
    with open(os.path.join(salida, 'enlace.md'), 'w') as f:
        f.write(a_markdown(R))
    print(f'estudio_enlace_rover: escrito en {salida}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
