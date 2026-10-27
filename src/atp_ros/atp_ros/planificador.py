"""Planificador de contactos de una RED de estaciones terrenas (N satelites x M
estaciones) con volumen de datos por contacto. Solo numpy + orbit/model (sin
ROS), para probarlo y reutilizarlo fuera del nodo `contact_scheduler`.

Cadena (cada paso repite abajo su condicion de validez):

  1. VENTANAS: `orbit.predecir_pases` por cada par (satelite, estacion), con
     mascara = max(mascara de la estacion, elevacion minima util).
  2. ENLACE: sobre cada pase muestreado (rango oblicuo REAL de SGP4) se
     calcula C/N0 con `model.presupuesto_enlace`, y de ahi Es/N0 = C/N0 -
     10 log10(Rs). Una codificacion y modulacion ADAPTATIVA (ACM) elige en cada
     instante el MODCOD de DVB-S2 mas eficiente cuya Es/N0 requerida (mas un
     margen) se cumple. Tasa de informacion = Rs * eficiencia espectral.
  3. VOLUMEN = integral (trapecios) de esa tasa a lo largo del pase. Se
     reporta tambien la cota de Shannon Rs*log2(1+Es/N0) integrada igual.
  4. RESTRICCIONES: una antena por estacion; entre dos contactos consecutivos
     de la misma estacion el rotor debe re-apuntarse (acimut por el camino
     corto con `vel_az`, elevacion con `vel_el`, mas un tiempo de asentamiento);
     un satelite no se descarga a dos estaciones a la vez (un solo enlace
     de bajada, datos distintos). El keyhole del rotor
     (`model.elevacion_keyhole`) marca el pase o recorta el tramo que el rotor
     no puede seguir.
  5. ALGORITMOS: codicioso por peso (prioridad x volumen) / densidad / orden
     de llegada, y OPTIMO (ramificacion y acotamiento propio con cotas de
     programacion dinamica; con scipy disponible, tambien programacion entera
     por HiGHS), para medir la brecha de optimalidad del codicioso.

Fuente de la tabla MODCOD: ETSI EN 302 307 V1.2.1 (2009-08), Tabla 13
("Es/No performance at Quasi Error Free PER = 10^-7 (AWGN channel)",
FECFRAME normal de 64 800 bits, sin pilotos). Los 28 valores se copiaron del
texto del PDF oficial de ETSI (no de memoria) y la prueba
`test_modcod_bajo_shannon` comprueba que cada uno queda por debajo de la
capacidad de Shannon a su Es/N0.

Lo que NO modela (cada cifra vale solo dentro de esto):
  - atmosfera (gases, lluvia, nubes) ni refraccion: el enlace es espacio
    libre + perdidas fijas `perdidas_db`; a baja elevacion en X/Ka sobrestima.
  - desapuntamiento: por defecto 0 deg (`error_apunt_deg`); los experimentos
    S4/N5 dan 0.001-0.004 deg rms, despreciable en el haz de un plato de 3 m.
  - ACM ideal: sin latencia ni histeresis del lazo de adaptacion; el margen
    `margen_acm_db` absorbe la perdida de implementacion.
  - memoria a bordo: el satelite siempre tiene datos que bajar.
  - el error del TLE (~1 km en la epoca, 1-3 km/dia): mueve AOS/LOS ~1-2 s
    por pase y se degrada con la edad del TLE.
"""
import bisect
import datetime as dt
import itertools
import math
import time
from dataclasses import dataclass, asdict, replace

import numpy as np

from atp_ros import model, orbit

BITS_POR_GB = 8e9           # 1 GB = 1e9 bytes (decimal, como en los enlaces)
_trapz = getattr(np, 'trapezoid', None) or np.trapz

# ---------------------------------------------------------------- MODCOD

# (nombre, Es/N0 ideal [dB], eficiencia espectral [bit/simbolo])
# ETSI EN 302 307 V1.2.1, Tabla 13. La eficiencia es por unidad de tasa de
# simbolos, FECFRAME normal, sin pilotos; incluye el overhead de la trama.
MODCOD_DVBS2 = (
    ('QPSK 1/4', -2.35, 0.490243), ('QPSK 1/3', -1.24, 0.656448),
    ('QPSK 2/5', -0.30, 0.789412), ('QPSK 1/2', 1.00, 0.988858),
    ('QPSK 3/5', 2.23, 1.188304), ('QPSK 2/3', 3.10, 1.322253),
    ('QPSK 3/4', 4.03, 1.487473), ('QPSK 4/5', 4.68, 1.587196),
    ('QPSK 5/6', 5.18, 1.654663), ('QPSK 8/9', 6.20, 1.766451),
    ('QPSK 9/10', 6.42, 1.788612),
    ('8PSK 3/5', 5.50, 1.779991), ('8PSK 2/3', 6.62, 1.980636),
    ('8PSK 3/4', 7.91, 2.228124), ('8PSK 5/6', 9.35, 2.478562),
    ('8PSK 8/9', 10.69, 2.646012), ('8PSK 9/10', 10.98, 2.679207),
    ('16APSK 2/3', 8.97, 2.637201), ('16APSK 3/4', 10.21, 2.966728),
    ('16APSK 4/5', 11.03, 3.165623), ('16APSK 5/6', 11.61, 3.300184),
    ('16APSK 8/9', 12.89, 3.523143), ('16APSK 9/10', 13.13, 3.567342),
    ('32APSK 3/4', 12.73, 3.703295), ('32APSK 4/5', 13.64, 3.951571),
    ('32APSK 5/6', 14.28, 4.119540), ('32APSK 8/9', 15.69, 4.397854),
    ('32APSK 9/10', 16.05, 4.453027),
)
_MC_NOMBRE = [m[0] for m in MODCOD_DVBS2]
_MC_REQ = np.array([m[1] for m in MODCOD_DVBS2])
_MC_ETA = np.array([m[2] for m in MODCOD_DVBS2])


# ---------------------------------------------------------------- parametros

@dataclass
class PerfilEnlace:
    """Enlace de bajada de UN satelite hacia el plato de la estacion.

    Valores por defecto = ESCENARIO HIPOTETICO (no son los de ningun satelite
    real): carga util en banda X, EIRP 12 dBW, 10 Mbaud, plato de 3 m, 150 K.
    Con ellos C/N0 ~ 91 dB-Hz a 1000 km (Es/N0 ~ 21 dB, el MODCOD mas alto) y
    ~ 83 dB-Hz a 2500 km (16APSK): el ACM trabaja a lo largo del pase.
    `rs_baud` es la tasa de SIMBOLOS; el ancho ocupado es rs*(1+alfa)."""
    banda: str = 'X'
    f_hz: float = 0.0               # 0 = usar model.BANDAS[banda]
    eirp_dbw: float = 12.0
    perdidas_db: float = 3.0
    rs_baud: float = 10e6
    diametro_m: float = model.DIAMETRO_PLATO_M
    eta: float = 0.6
    t_sys_k: float = model.T_SISTEMA_K
    margen_acm_db: float = 1.0      # sobre la Es/N0 ideal de ETSI (impl. + histeresis)
    error_apunt_deg: float = 0.0
    t_adq_s: float = 0.0            # segundos tras el AOS sin datos (adquisicion)

    def frecuencia(self):
        return float(self.f_hz) if self.f_hz else float(model.BANDAS[self.banda])


@dataclass
class Rotor:
    """Rotor az/el de la estacion y politica de keyhole.
    `keyhole`: 'marcar' (se marca, el volumen cuenta el pase completo),
    'recortar' (el volumen excluye los instantes en que la demanda del pase
    supera `vel_az`/`vel_el`) o 'descartar' (el pase no entra al plan).
    Supone un rotor de acimut sin tope de giro (camino corto, |dAz| <= 180)."""
    vel_az_deg_s: float = 6.0
    vel_el_deg_s: float = 3.0
    t_asent_s: float = 5.0          # asentamiento tras cada re-apuntamiento
    keyhole: str = 'marcar'


@dataclass
class Ventana:
    """Un contacto posible (satelite, estacion). Tiempos `t_*_s` en segundos
    desde el inicio del horizonte. `peso` = prioridad x volumen en GB."""
    id: int
    sat: str
    norad: int
    estacion: str
    aos: dt.datetime
    los: dt.datetime
    t_aos_s: float
    t_los_s: float
    az_aos_deg: float
    az_los_deg: float
    el_aos_deg: float
    el_los_deg: float
    el_max_deg: float
    rango_min_km: float
    aos_truncado: bool
    los_truncado: bool
    keyhole: bool
    el_keyhole_deg: float
    t_perdido_s: float              # tiempo en que la demanda supera al rotor
    az_pto_max_deg_s: float
    vol_bits: float
    vol_shannon_bits: float
    modcod_max: str
    esn0_max_db: float
    prioridad: float = 1.0
    peso: float = 0.0

    @property
    def duracion_s(self):
        return self.t_los_s - self.t_aos_s

    @property
    def vol_gb(self):
        return self.vol_bits / BITS_POR_GB


@dataclass
class Plan:
    """Seleccion de ventanas (ids, ordenados por AOS). `optimo_probado` solo
    es True si el algoritmo demostro que no hay mejor plan; `cota_sup` es una
    cota superior valida del valor optimo."""
    algoritmo: str
    ids: list
    valor: float
    optimo_probado: bool = False
    cota_sup: float = float('nan')
    nodos: int = 0
    tiempo_s: float = 0.0           # reloj de pared: NO entra al JSON


# ---------------------------------------------------------------- enlace + ACM

def esn0_db(cn0_dbhz, rs_baud):
    """Es/N0 (dB) = C/N0 (dB-Hz) - 10 log10(Rs)."""
    return np.asarray(cn0_dbhz, dtype=float) - 10.0 * math.log10(float(rs_baud))


def elegir_modcod(esn0, margen_db=0.0):
    """MODCOD de DVB-S2 mas eficiente cuya Es/N0 ideal + `margen_db` no
    supera `esn0` (dB, escalar o array). Devuelve (indice, eficiencia) con
    indice -1 y eficiencia 0 donde no cierra ni QPSK 1/4. Vale en AWGN, con
    la tabla ETSI; la eficiencia no es monotona en Es/N0 (16APSK 2/3 supera
    a 8PSK 5/6 con menos Es/N0), por eso se toma el maximo de los factibles."""
    e = np.atleast_1d(np.asarray(esn0, dtype=float))
    ok = e[:, None] >= (_MC_REQ + float(margen_db))[None, :]
    eta = np.where(ok, _MC_ETA[None, :], 0.0)
    idx = np.where(ok.any(axis=1), eta.argmax(axis=1), -1)
    ef = eta.max(axis=1)
    if np.ndim(esn0) == 0:
        return int(idx[0]), float(ef[0])
    return idx, ef


def cn0_enlace(d_km, perfil):
    """C/N0 (dB-Hz) para el rango oblicuo `d_km` (escalar o array), con
    `model.presupuesto_enlace`. Valido con las condiciones de ese modelo:
    espacio libre, perdidas fijas, sin atmosfera."""
    r = model.presupuesto_enlace(
        d_km, perfil.error_apunt_deg, f_hz=perfil.frecuencia(),
        eirp_dbw=perfil.eirp_dbw, perdidas_db=perfil.perdidas_db,
        tasa_bps=1.0, diametro_m=perfil.diametro_m, eta=perfil.eta,
        t_sys_k=perfil.t_sys_k)
    return np.asarray(r['cn0_dbhz'], dtype=float)


def tasas_enlace(d_km, perfil):
    """Tasas (bps) en cada rango: ACM, cota de Shannon, y el indice de MODCOD
    y la Es/N0. Dict de arrays. Vale como `cn0_enlace` + ACM ideal."""
    d = np.atleast_1d(np.asarray(d_km, dtype=float))
    esn0 = esn0_db(cn0_enlace(d, perfil), perfil.rs_baud)
    idx, eta = elegir_modcod(esn0, perfil.margen_acm_db)
    shannon = np.log2(1.0 + 10.0 ** (esn0 / 10.0))
    rs = float(perfil.rs_baud)
    return {'esn0_db': esn0, 'modcod_idx': idx, 'eta': eta,
            'tasa_bps': rs * eta, 'shannon_bps': rs * shannon}


# ---------------------------------------------------------------- ventanas

def _altitud_km(tle, est, t):
    r = orbit.observar_en(tle, est, t)['r_ecef_km']
    return float(orbit.ecef_a_geodetica(np.asarray(r))[2])


def ventana_de_pase(tle, est, clave_est, pase, t0, perfil, rotor, dt_s=1.0,
                    dut1_s=0.0):
    """Construye la `Ventana` de un pase de `orbit.predecir_pases`: muestrea el
    pase (`orbit.muestrear_pase`), integra la tasa ACM con trapecios y evalua
    el keyhole. `id` y `peso` los fija `construir_ventanas`. Validez: la de
    `predecir_pases` para los tiempos y la de `tasas_enlace` para el volumen;
    el volumen converge al reducir `dt_s` (la tasa ACM es escalonada: el
    error de trapecios es ~ salto*dt_s/2 por cambio de MODCOD)."""
    s = orbit.muestrear_pase(tle, est, pase, dt_s, dut1_s)
    t = s['t']
    tz = tasas_enlace(s['d'], perfil)
    tasa, shan = tz['tasa_bps'].copy(), tz['shannon_bps'].copy()
    if perfil.t_adq_s > 0.0:
        tasa[t < perfil.t_adq_s] = 0.0
        shan[t < perfil.t_adq_s] = 0.0
    # keyhole: demanda real de la trayectoria contra el rotor
    sobre = ((np.abs(s['az_pto']) > rotor.vel_az_deg_s)
             | (np.abs(s['el_pto']) > rotor.vel_el_deg_s))
    paso = np.gradient(t)
    t_perdido = float(np.sum(paso[sobre]))
    if rotor.keyhole == 'recortar':
        tasa[sobre] = 0.0
        shan[sobre] = 0.0
    h = _altitud_km(tle, est, pase['tca'])
    el_kh = model.elevacion_keyhole(h, rotor.vel_az_deg_s)
    i_best = int(np.argmax(tz['eta']))
    return Ventana(
        id=-1, sat=tle.nombre, norad=int(tle.norad), estacion=clave_est,
        aos=pase['aos'], los=pase['los'],
        t_aos_s=(pase['aos'] - t0).total_seconds(),
        t_los_s=(pase['los'] - t0).total_seconds(),
        az_aos_deg=float(s['az'][0]), az_los_deg=float(s['az'][-1]),
        el_aos_deg=float(s['el'][0]), el_los_deg=float(s['el'][-1]),
        el_max_deg=float(pase['el_max_deg']),
        rango_min_km=float(pase['rango_min_km']),
        aos_truncado=bool(pase['aos_truncado']),
        los_truncado=bool(pase['los_truncado']),
        keyhole=bool(pase['el_max_deg'] > el_kh), el_keyhole_deg=float(el_kh),
        t_perdido_s=t_perdido,
        az_pto_max_deg_s=float(np.max(np.abs(s['az_pto']))),
        vol_bits=float(_trapz(tasa, t)), vol_shannon_bits=float(_trapz(shan, t)),
        modcod_max=(_MC_NOMBRE[int(tz['modcod_idx'][i_best])]
                    if tz['modcod_idx'][i_best] >= 0 else 'sin enlace'),
        esn0_max_db=float(np.max(tz['esn0_db'])))


def construir_ventanas(tles, estaciones, t0, horas=24.0, enlace=None,
                       rotor=None, perfiles=None, prioridades=None,
                       el_min_deg=0.0, dt_s=1.0, paso_busqueda_s=20.0,
                       dut1_s=0.0):
    """Ventanas de contacto de todos los pares (satelite, estacion) en
    [t0, t0 + horas], ordenadas por (AOS, satelite, estacion) y numeradas
    desde 0 (ese orden es el que usan los algoritmos).

    `tles`: lista de `orbit.Tle`. `estaciones`: dict clave -> `orbit.Estacion`.
    `enlace`: `PerfilEnlace` comun; `perfiles`: dict nombre-de-satelite ->
    PerfilEnlace que lo sustituye. `prioridades`: dict nombre -> numero (1 por
    defecto). Mascara de cada par = max(mascara de la estacion, `el_min_deg`).
    Los tiempos coinciden con `orbit.predecir_pases` (misma funcion, mismos
    argumentos); con `keyhole='descartar'` se omiten los pases sobre el
    keyhole. Determinista: solo depende de los argumentos (t0 fijo)."""
    enlace = enlace or PerfilEnlace()
    rotor = rotor or Rotor()
    perfiles = perfiles or {}
    prioridades = prioridades or {}
    vs = []
    for tle in tles:
        perfil = perfiles.get(tle.nombre, enlace)
        for clave, est in estaciones.items():
            mask = max(float(est.mascara_deg), float(el_min_deg))
            for p in orbit.predecir_pases(tle, est, t0, horas, mask,
                                          paso_s=paso_busqueda_s, dut1_s=dut1_s):
                v = ventana_de_pase(tle, est, clave, p, t0, perfil, rotor,
                                    dt_s, dut1_s)
                if rotor.keyhole == 'descartar' and v.keyhole:
                    continue
                v.prioridad = float(prioridades.get(tle.nombre, 1.0))
                v.peso = v.prioridad * v.vol_gb
                vs.append(v)
    vs.sort(key=lambda v: (v.aos, v.sat, v.estacion))
    for i, v in enumerate(vs):
        v.id = i
    return vs


def subred(ventanas, estaciones=None, satelites=None):
    """Sub-conjunto de las ventanas (solo las de `estaciones` y/o `satelites`,
    iterables de claves / nombres; None = todas), renumerado desde 0 con el
    mismo orden. Una ventana depende solo de su par (satelite, estacion), asi
    que quitar una estacion no obliga a recalcular SGP4: sirve para medir lo
    que aporta cada estacion o satelite a la red."""
    est = None if estaciones is None else set(estaciones)
    sat = None if satelites is None else set(satelites)
    out = [replace(v) for v in ventanas
           if (est is None or v.estacion in est)
           and (sat is None or v.sat in sat)]
    for i, v in enumerate(out):
        v.id = i
    return out


# ---------------------------------------------------------------- restricciones

def tiempo_reapuntamiento(a, b, rotor):
    """Segundos que necesita el rotor para ir de donde termina el contacto `a`
    (LOS) a donde empieza `b` (AOS): max(|dAz|/vel_az, |dEl|/vel_el) +
    asentamiento. dAz por el camino corto (<= 180 deg). Vale para un rotor
    az/el con velocidades constantes y sin tope de giro."""
    daz = abs((b.az_aos_deg - a.az_los_deg + 180.0) % 360.0 - 180.0)
    d_el = abs(b.el_aos_deg - a.el_los_deg)
    return max(daz / rotor.vel_az_deg_s, d_el / rotor.vel_el_deg_s) + rotor.t_asent_s


def compatibles_estacion(a, b, rotor):
    """`b` (que empieza despues que `a`) puede seguir a `a` en la MISMA
    antena: b.AOS >= a.LOS + re-apuntamiento."""
    return b.t_aos_s >= a.t_los_s + tiempo_reapuntamiento(a, b, rotor) - 1e-9


def solapan_satelite(a, b, sep_sat_s=0.0):
    """El mismo satelite no se descarga a dos estaciones a la vez: dos
    ventanas de un satelite chocan si se solapan (con `sep_sat_s` de
    separacion minima entre una y otra)."""
    return (a.t_aos_s < b.t_los_s + sep_sat_s - 1e-9
            and b.t_aos_s < a.t_los_s + sep_sat_s - 1e-9)


def verificar_plan(ids, ventanas, rotor, sep_sat_s=0.0):
    """Lista de violaciones (vacia = plan valido). Revisa: ids repetidos o
    inexistentes, solape y re-apuntamiento entre contactos consecutivos de
    cada estacion, y solapes de cada satelite. Independiente de los
    algoritmos de planificacion."""
    out = []
    if len(set(ids)) != len(ids):
        out.append('ids repetidos')
    sel = [ventanas[i] for i in sorted(set(ids))]
    for clave in sorted({v.estacion for v in sel}):
        lst = sorted((v for v in sel if v.estacion == clave),
                     key=lambda v: (v.t_aos_s, v.id))
        for a, b in zip(lst, lst[1:]):
            if b.t_aos_s < a.t_los_s - 1e-9:
                out.append(f'{clave}: solape {a.sat}#{a.id} / {b.sat}#{b.id}')
            elif not compatibles_estacion(a, b, rotor):
                out.append(f'{clave}: re-apuntamiento insuficiente '
                           f'{a.sat}#{a.id} -> {b.sat}#{b.id}: hay '
                           f'{b.t_aos_s - a.t_los_s:.1f} s y hacen falta '
                           f'{tiempo_reapuntamiento(a, b, rotor):.1f} s')
    for sat in sorted({v.sat for v in sel}):
        lst = [v for v in sel if v.sat == sat]
        for a, b in itertools.combinations(lst, 2):
            if solapan_satelite(a, b, sep_sat_s):
                out.append(f'{sat}: contactos simultaneos #{a.id} ({a.estacion})'
                           f' y #{b.id} ({b.estacion})')
    return out


def _valor(ids, ventanas):
    return float(sum(ventanas[i].peso for i in ids))


# ---------------------------------------------------------------- codicioso

CRITERIOS = ('peso', 'densidad', 'aos')


def planificar_codicioso(ventanas, rotor, criterio='peso', sep_sat_s=0.0):
    """Planificador codicioso: recorre las ventanas en orden de `criterio`
    ('peso' = prioridad x volumen, de mayor a menor; 'densidad' = peso por
    segundo; 'aos' = orden de llegada) y acepta cada una si cabe: no choca con
    las ya aceptadas de su estacion (incluido el re-apuntamiento a ambos
    lados) ni con las de su satelite. No es optimo: una ventana grande puede
    bloquear dos que juntas valen mas."""
    t0 = time.perf_counter()
    claves = {'peso': lambda v: (-v.peso, v.t_aos_s, v.id),
              'densidad': lambda v: (-v.peso / max(v.duracion_s, 1e-9),
                                     v.t_aos_s, v.id),
              'aos': lambda v: (v.t_aos_s, v.id)}
    if criterio not in claves:
        raise ValueError(f'criterio debe ser uno de {CRITERIOS}')
    por_est, por_sat = {}, {}
    for v in sorted(ventanas, key=claves[criterio]):
        lst = por_est.setdefault(v.estacion, [])      # ids ordenados por id (= por AOS)
        pos = bisect.bisect(lst, v.id)
        if pos > 0 and not compatibles_estacion(ventanas[lst[pos - 1]], v, rotor):
            continue
        if pos < len(lst) and not compatibles_estacion(v, ventanas[lst[pos]], rotor):
            continue
        if any(solapan_satelite(ventanas[j], v, sep_sat_s)
               for j in por_sat.get(v.sat, [])):
            continue
        lst.insert(pos, v.id)
        por_sat.setdefault(v.sat, []).append(v.id)
    ids = sorted(i for lst in por_est.values() for i in lst)
    return Plan(f'codicioso-{criterio}', ids, _valor(ids, ventanas),
                tiempo_s=time.perf_counter() - t0)


# ---------------------------------------------------------------- optimo

def optimo_fuerza_bruta(ventanas, rotor, sep_sat_s=0.0, max_n=22):
    """Optimo por enumeracion de TODOS los subconjuntos (2^n) con
    `verificar_plan`. Referencia para validar a los otros algoritmos; solo
    para n <= `max_n`. Desempata por el menor numero de contactos y luego por
    el primer subconjunto en orden lexicografico de ids."""
    n = len(ventanas)
    if n > max_n:
        raise ValueError(f'fuerza bruta solo para n <= {max_n} (n={n})')
    t0 = time.perf_counter()
    mejor, mejor_v = [], 0.0
    for k in range(1, n + 1):
        for comb in itertools.combinations(range(n), k):
            v = _valor(comb, ventanas)
            if v > mejor_v + 1e-12 and not verificar_plan(comb, ventanas, rotor,
                                                           sep_sat_s):
                mejor, mejor_v = list(comb), v
    return Plan('fuerza-bruta', mejor, mejor_v, True, mejor_v,
                tiempo_s=time.perf_counter() - t0)


def _dp_estacion(lst, suc, w, prohibidas):
    """Secuencia de peso maximo en UNA estacion (programacion dinamica hacia
    atras): f[a] = w_a + max(0, max_{s en suc[a]} f[s]), saltando las ventanas
    `prohibidas`. `lst` = ids de la estacion por AOS; `suc[a]` = posiciones
    posteriores compatibles con a (re-apuntamiento incluido). Devuelve
    (valor, ids elegidos). Desempata por el id menor: determinista."""
    m = len(lst)
    f = [0.0] * m
    sig = [-1] * m
    for a in range(m - 1, -1, -1):
        if lst[a] in prohibidas:
            f[a] = -1.0
            continue
        mejor, arg = 0.0, -1
        for s_ in suc[a]:
            if f[s_] > mejor:
                mejor, arg = f[s_], s_
        f[a] = w[lst[a]] + mejor
        sig[a] = arg
    mejor, a0 = 0.0, -1
    for a in range(m):
        if f[a] > mejor:
            mejor, a0 = f[a], a
    ids = []
    while a0 >= 0:
        ids.append(lst[a0])
        a0 = sig[a0]
    return mejor, ids


def _optimo_bb(ventanas, rotor, sep_sat_s, inicial, limite_nodos, limite_s):
    """Ramificacion y acotamiento sobre los CHOQUES DE SATELITE.

    Relajacion: se quita la restriccion de satelite; entonces cada estacion
    es un problema independiente (secuencia de peso maximo con tiempos de
    re-apuntamiento) que se resuelve EXACTO por programacion dinamica. Su
    suma es una cota superior valida del nodo. Si la solucion relajada no
    tiene dos ventanas simultaneas del mismo satelite, es factible y por
    tanto optima del nodo. Si las tiene (a, b), todo plan factible deja fuera
    a `a` o a `b`: se ramifica en "a prohibida" / "b prohibida". Se explora
    primero el nodo de mayor cota (mejor-primero, desempate por orden de
    creacion: determinista). Con `limite_nodos` / `limite_s` se detiene y
    devuelve el mejor plan factible con `optimo_probado=False` y la mayor
    cota de los nodos abiertos como `cota_sup`."""
    import heapq
    t_ini = time.perf_counter()
    n = len(ventanas)
    w = [v.peso for v in ventanas]
    est_ids = {}
    for v in ventanas:
        est_ids.setdefault(v.estacion, []).append(v.id)
    suc = {}
    for st, lst in est_ids.items():
        suc[st] = [[b for b in range(a + 1, len(lst))
                    if compatibles_estacion(ventanas[lst[a]], ventanas[lst[b]], rotor)]
                   for a in range(len(lst))]

    def resolver(st, prohib):
        return _dp_estacion(est_ids[st], suc[st], w, prohib)

    def choque(sel):
        """Primer par (a, b) simultaneo del mismo satelite, o None."""
        por_sat = {}
        for i in sorted(sel):
            por_sat.setdefault(ventanas[i].sat, []).append(i)
        for lst in por_sat.values():
            for x in range(len(lst)):
                for y in range(x + 1, len(lst)):
                    if solapan_satelite(ventanas[lst[x]], ventanas[lst[y]],
                                        sep_sat_s):
                        return lst[x], lst[y]
        return None

    mejor_v, mejor_ids = inicial.valor, list(inicial.ids)
    cuenta = itertools.count()
    sols0 = {st: resolver(st, frozenset()) for st in est_ids}
    heap = []
    cota0 = sum(v for v, _ in sols0.values())
    heapq.heappush(heap, (-cota0, next(cuenta), frozenset(), sols0))
    nodos, cortado = 0, False
    while heap:
        negb, _, prohib, sols = heap[0]
        if -negb <= mejor_v + 1e-9:
            break                       # todo lo abierto no mejora el incumbente
        if nodos >= limite_nodos or time.perf_counter() - t_ini > limite_s:
            cortado = True
            break
        heapq.heappop(heap)
        nodos += 1
        sel = [i for _, ids in sols.values() for i in ids]
        par = choque(sel)
        if par is None:
            val = sum(v for v, _ in sols.values())
            if val > mejor_v + 1e-12:
                mejor_v, mejor_ids = val, sorted(sel)
            continue
        for x in par:
            nuevo = prohib | {x}
            st = ventanas[x].estacion
            sub = dict(sols)
            sub[st] = resolver(st, nuevo)
            cota = sum(v for v, _ in sub.values())
            if cota > mejor_v + 1e-9:
                heapq.heappush(heap, (-cota, next(cuenta), nuevo, sub))
    cota_abierta = max([-h[0] for h in heap], default=mejor_v)
    probado = not cortado
    return Plan('optimo-bb', sorted(mejor_ids), mejor_v, probado,
                mejor_v if probado else max(cota_abierta, mejor_v),
                nodos, time.perf_counter() - t_ini)


def _optimo_milp(ventanas, rotor, sep_sat_s, limite_s):
    """Programacion entera con scipy.optimize.milp (HiGHS): x_i en {0,1},
    maximizar sum(w_i x_i) sujeto a x_i + x_j <= 1 para cada par que no puede
    convivir (misma estacion sin tiempo de re-apuntamiento, o mismo satelite
    solapado). Exacta si el re-apuntamiento cumple la desigualdad triangular
    (cierto salvo contactos de menos de ~30 s); `optimo_probado` es True solo
    si HiGHS cierra la brecha. Lanza ImportError sin scipy."""
    from scipy.optimize import milp, LinearConstraint, Bounds
    from scipy.sparse import coo_matrix
    t_ini = time.perf_counter()
    n = len(ventanas)
    filas, cols = [], []
    r = 0
    for i in range(n):
        a = ventanas[i]
        for j in range(i + 1, n):
            b = ventanas[j]
            choque = (a.estacion == b.estacion
                      and not compatibles_estacion(a, b, rotor)) or (
                a.sat == b.sat and solapan_satelite(a, b, sep_sat_s))
            if choque:
                filas += [r, r]
                cols += [i, j]
                r += 1
    c = -np.array([v.peso for v in ventanas])
    if r:
        A = coo_matrix((np.ones(len(filas)), (filas, cols)), shape=(r, n)).tocsr()
        cons = [LinearConstraint(A, -np.inf, 1.0)]
    else:
        cons = []
    res = milp(c, constraints=cons, integrality=np.ones(n),
               bounds=Bounds(0, 1), options={'time_limit': float(limite_s)})
    if res.x is None:
        raise RuntimeError(f'milp sin solucion: {res.message}')
    ids = [i for i in range(n) if res.x[i] > 0.5]
    cota = -float(getattr(res, 'mip_dual_bound', -_valor(ids, ventanas)))
    return Plan('optimo-milp', ids, _valor(ids, ventanas), res.status == 0,
                max(cota, _valor(ids, ventanas)), 0, time.perf_counter() - t_ini)


def planificar_optimo(ventanas, rotor, sep_sat_s=0.0, metodo='auto',
                      limite_nodos=5_000, limite_s=120.0):
    """Plan de valor maximo (suma de pesos). `metodo`:
      'bb'   ramificacion y acotamiento propio sobre los choques de satelite
             con programacion dinamica por estacion (solo Python). Exacto si
             `optimo_probado`; si agota `limite_nodos` devuelve el mejor plan
             hallado y `cota_sup` (valida) lo acota. Es determinista (el corte
             es por nodos, no por reloj);
      'milp' programacion entera con scipy/HiGHS (si esta instalado), con
             `limite_s` segundos como tope de pared;
      'auto' 'bb' y, si no pudo probar el optimo y hay scipy, 'milp'.
    ALCANCE MEDIDO: con decenas de ventanas (instancias pequenas) 'bb' prueba
    el optimo en milisegundos; con ~170 ventanas y 40 choques de satelite (red
    de 4 estaciones x 8 satelites, 24 h) tras 5000 nodos la cota sigue 4.6 %
    sobre el optimo y 'bb' NO lo prueba: ahi se usa 'milp'.
    Toda salida pasa por `verificar_plan`."""
    if metodo not in ('auto', 'bb', 'milp'):
        raise ValueError("metodo debe ser 'auto', 'bb' o 'milp'")
    if metodo == 'milp':
        plan = _optimo_milp(ventanas, rotor, sep_sat_s, limite_s)
    else:
        inicial = max((planificar_codicioso(ventanas, rotor, c, sep_sat_s)
                       for c in CRITERIOS), key=lambda p: p.valor)
        plan = _optimo_bb(ventanas, rotor, sep_sat_s, inicial, limite_nodos,
                          float('inf'))
        if not plan.optimo_probado and metodo == 'auto':
            try:
                plan = _optimo_milp(ventanas, rotor, sep_sat_s, limite_s)
            except ImportError:
                pass
    mal = verificar_plan(plan.ids, ventanas, rotor, sep_sat_s)
    if mal:
        raise RuntimeError(f'plan {plan.algoritmo} invalido: {mal[:3]}')
    return plan


# ---------------------------------------------------------------- reportes

def resumen(ids, ventanas):
    """Totales de un plan: contactos, minutos de contacto, GB (decimales) y
    la cota de Shannon, en total y por satelite y por estacion."""
    sel = [ventanas[i] for i in sorted(ids)]

    def agrega(vs):
        return {'contactos': len(vs),
                'minutos': round(sum(v.duracion_s for v in vs) / 60.0, 2),
                'gb': round(sum(v.vol_gb for v in vs), 3),
                'gb_shannon': round(sum(v.vol_shannon_bits for v in vs)
                                    / BITS_POR_GB, 3)}
    out = agrega(sel)
    out['por_satelite'] = {s: agrega([v for v in sel if v.sat == s])
                           for s in sorted({v.sat for v in sel})}
    out['por_estacion'] = {e: agrega([v for v in sel if v.estacion == e])
                           for e in sorted({v.estacion for v in sel})}
    return out


def _iso(t):
    return t.astimezone(orbit.UTC).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def ventana_json(v):
    """Ventana -> dict serializable (fechas ISO 8601 con Z)."""
    return {
        'id': v.id, 'sat': v.sat, 'norad': v.norad, 'estacion': v.estacion,
        'aos': _iso(v.aos), 'los': _iso(v.los),
        'duracion_s': round(v.duracion_s, 1),
        'az_aos_deg': round(v.az_aos_deg, 1), 'az_los_deg': round(v.az_los_deg, 1),
        'el_max_deg': round(v.el_max_deg, 2),
        'rango_min_km': round(v.rango_min_km, 1),
        'aos_truncado': v.aos_truncado, 'los_truncado': v.los_truncado,
        'keyhole': v.keyhole, 't_perdido_s': round(v.t_perdido_s, 1),
        'modcod_max': v.modcod_max, 'esn0_max_db': round(v.esn0_max_db, 2),
        'vol_gb': round(v.vol_gb, 4),
        'vol_shannon_gb': round(v.vol_shannon_bits / BITS_POR_GB, 4),
        'prioridad': v.prioridad, 'peso': round(v.peso, 4)}


def plan_json(plan, ventanas):
    """Plan -> dict serializable: algoritmo, ids, valor, optimalidad, y el
    resumen. Sin sellos de reloj de pared: misma entrada, mismos bytes."""
    return {'algoritmo': plan.algoritmo, 'ids': list(plan.ids),
            'valor': round(plan.valor, 6),
            'optimo_probado': bool(plan.optimo_probado),
            'cota_sup': (None if math.isnan(plan.cota_sup)
                         else round(plan.cota_sup, 6)),
            'nodos': int(plan.nodos), 'resumen': resumen(plan.ids, ventanas)}


def calcular_plan(tles, estaciones, t0, horas=24.0, enlace=None, rotor=None,
                  perfiles=None, prioridades=None, el_min_deg=0.0,
                  sep_sat_s=0.0, metodo='auto', dt_s=1.0, optimo=True,
                  limite_nodos=5_000, limite_s=120.0, ventanas=None):
    """Todo el flujo: ventanas -> codicioso (los 3 criterios) -> optimo ->
    dict listo para JSON (`plan.json`; tambien lo publica el nodo
    `contact_scheduler`). `brecha_codicioso_pct` = (optimo - codicioso) /
    optimo con el codicioso de criterio 'peso'. Si el optimo no se pudo
    probar, ese numero usa el mejor plan hallado (la brecha real puede ser
    mayor) y `brecha_max_pct` la acota con `cota_sup`. `ventanas` (opcional)
    reutiliza ventanas ya construidas (p. ej. de `subred`): entonces `tles`,
    `estaciones`, `enlace`, `perfiles` y `prioridades` solo describen el
    resultado y deben ser los mismos con que se construyeron."""
    enlace = enlace or PerfilEnlace()
    rotor = rotor or Rotor()
    vs = (ventanas if ventanas is not None else
          construir_ventanas(tles, estaciones, t0, horas, enlace, rotor,
                             perfiles, prioridades, el_min_deg, dt_s))
    cod = {c: planificar_codicioso(vs, rotor, c, sep_sat_s) for c in CRITERIOS}
    planes = {f'codicioso_{c}': plan_json(p, vs) for c, p in cod.items()}
    out = {
        'inicio_utc': _iso(t0), 'horas': float(horas),
        'banda': enlace.banda,
        'enlace': asdict(enlace), 'rotor': asdict(rotor),
        'el_min_deg': float(el_min_deg), 'sep_sat_s': float(sep_sat_s),
        'estaciones': {k: {'nombre': e.nombre, 'lat_deg': e.lat_deg,
                           'lon_deg': e.lon_deg, 'alt_m': e.alt_m,
                           'mascara_deg': e.mascara_deg}
                       for k, e in estaciones.items()},
        'satelites': [{'nombre': t.nombre, 'norad': t.norad,
                       'edad_tle_dias': round(t.edad_dias(t0), 2)} for t in tles],
        'n_ventanas': len(vs),
        'disponible': resumen(range(len(vs)), vs),
        'ventanas': [ventana_json(v) for v in vs],
        'planes': planes,
    }
    if optimo:
        op = planificar_optimo(vs, rotor, sep_sat_s, metodo, limite_nodos,
                               limite_s)
        planes['optimo'] = plan_json(op, vs)
        base = cod['peso'].valor
        ref = op.valor if op.optimo_probado else op.cota_sup
        out['brecha_codicioso_pct'] = (round(100.0 * (op.valor - base) / op.valor, 4)
                                       if op.valor > 0 else 0.0)
        out['brecha_max_pct'] = (round(100.0 * (ref - base) / ref, 4)
                                 if ref > 0 else 0.0)
    return out
