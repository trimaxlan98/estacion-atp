"""Orbitas reales para la Estacion: TLE -> SGP4 -> TEME -> ECEF -> ENU -> Az/El.

Solo numpy + sgp4 (sin ROS), para probarlo y reutilizarlo fuera de los nodos.
Es la cadena de la leccion 1.2 del curso 30. A diferencia de `model.py`
(orbita circular, Tierra esferica), aqui el satelite sale de un TLE real.

Cadena de marcos y que se DESPRECIA (cada funcion repite lo que le toca):

  TLE --SGP4--> TEME (km, km/s)          error propio de SGP4/TLE: ~1 km en la
                                          epoca, crece ~1-3 km/dia en LEO
  TEME --R3(GMST)--> ECEF                 se desprecia: movimiento del polo
                                          (<= ~0.3 arcsec, ~10 m), la
                                          diferencia UT1-UTC (<= 0.9 s: rotacion
                                          de hasta 13.5 arcsec, ~0.44 km en LEO;
                                          corregible con `dut1_s`) y la
                                          nutacion fina (TEME ya esta referido al
                                          ecuador verdadero, asi que solo
                                          queda GMST)
  ECEF --ENU--> az, el, rango, v_radial   Tierra WGS84 (elipsoide) para la
                                          estacion; sin refraccion atmosferica
                                          (a 5 deg de elevacion subiria la
                                          elevacion aparente ~0.17 deg)

Tiempo: UTC como `datetime` con zona UTC, o par (jd, fr) como el de sgp4. No se
usan segundos intercalares: GMST se evalua con UTC como si fuera UT1.
"""
import datetime as dt
import glob
import math
import os
from dataclasses import dataclass, field

import numpy as np
from sgp4.api import Satrec, WGS72, jday

# ---------------------------------------------------------------- constantes

A_WGS84_KM = 6378.137                   # semieje mayor
F_WGS84 = 1.0 / 298.257223563           # aplanamiento
E2_WGS84 = F_WGS84 * (2.0 - F_WGS84)    # excentricidad^2
B_WGS84_KM = A_WGS84_KM * (1.0 - F_WGS84)
OMEGA_TIERRA = 7.2921158553e-5          # rad/s, d(GMST)/dt (incluye precesion)
UTC = dt.timezone.utc

DIR_DATOS_ENV = 'ATP_DATA_DIR'


# ---------------------------------------------------------------- tiempo

def a_jd(t):
    """datetime (aware o naive-UTC) -> par (jd, fr) para sgp4. Vale con
    precision de microsegundos."""
    if t.tzinfo is not None:
        t = t.astimezone(UTC)
    return jday(t.year, t.month, t.day, t.hour, t.minute,
                t.second + t.microsecond * 1e-6)


def de_jd(jd, fr):
    """Par (jd, fr) -> datetime UTC (precision ~ microsegundos)."""
    d = (dt.datetime(2000, 1, 1, 12, tzinfo=UTC)
         + dt.timedelta(days=float(jd - 2451545.0) + float(fr)))
    return d


def a_utc(texto):
    """'2026-10-02T12:00:00' (con o sin Z / zona) -> datetime UTC. Si no trae
    zona se interpreta como UTC."""
    t = dt.datetime.fromisoformat(str(texto).replace('Z', '+00:00'))
    return t.replace(tzinfo=UTC) if t.tzinfo is None else t.astimezone(UTC)


def gmst_rad(jd, fr, dut1_s=0.0):
    """Tiempo sidereo medio de Greenwich (rad, en [0, 2pi)), modelo IAU-82
    de Vallado (Algoritmo 15). `jd`, `fr` pueden ser escalares o arrays.

    Vale con UT1 = UTC + `dut1_s`: con `dut1_s = 0` el error es la diferencia
    real UT1-UTC (|.| < 0.9 s -> hasta 0.0038 deg de rotacion). Desprecia la
    ecuacion de los equinoccios, correcto porque TEME usa el equinoccio
    MEDIO. Verificado contra Vallado Ejemplo 3-5 (152.578787810 deg)."""
    jd = np.asarray(jd, dtype=float)
    fr = np.asarray(fr, dtype=float) + float(dut1_s) / 86400.0
    t = ((jd - 2451545.0) + fr) / 36525.0
    seg = (67310.54841 + (876600.0 * 3600.0 + 8640184.812866) * t
           + 0.093104 * t ** 2 - 6.2e-6 * t ** 3)
    return np.mod(np.radians(np.mod(seg, 86400.0) / 240.0), 2.0 * np.pi)


# ---------------------------------------------------------------- TLE

def checksum_tle(linea):
    """Suma de verificacion del TLE (modulo 10 de los digitos + 1 por cada
    '-') de los primeros 68 caracteres. Vale para una linea 1 o 2."""
    s = 0
    for c in linea[:68]:
        if c.isdigit():
            s += int(c)
        elif c == '-':
            s += 1
    return s % 10


@dataclass
class Tle:
    """Un TLE parseado. `satrec` es el objeto de sgp4 (constantes WGS72, que
    es lo que SGP4 espera de un TLE de CelesTrak/NORAD)."""
    nombre: str
    linea1: str
    linea2: str
    norad: int
    epoca: dt.datetime
    satrec: Satrec = field(repr=False)
    checksum_ok: bool = True

    @property
    def n_rev_dia(self):
        return float(self.linea2[52:63])

    def edad_dias(self, t):
        """Dias entre la epoca del TLE y `t` (negativo si t es anterior).
        Un TLE solo es fiable cerca de su epoca."""
        return (t - self.epoca).total_seconds() / 86400.0


def tle_desde_lineas(nombre, linea1, linea2):
    """Construye un `Tle`. Lanza ValueError si las lineas no son un TLE."""
    l1, l2 = linea1.rstrip(), linea2.rstrip()
    if not (l1.startswith('1 ') and l2.startswith('2 ')):
        raise ValueError(f'TLE mal formado para {nombre!r}')
    sat = Satrec.twoline2rv(l1, l2, WGS72)
    yy = int(l1[18:20])
    anio = 2000 + yy if yy < 57 else 1900 + yy
    epoca = (dt.datetime(anio, 1, 1, tzinfo=UTC)
             + dt.timedelta(days=float(l1[20:32]) - 1.0))
    ok = (len(l1) >= 69 and len(l2) >= 69
          and checksum_tle(l1) == int(l1[68]) and checksum_tle(l2) == int(l2[68]))
    return Tle(nombre.strip(), l1, l2, int(l1[2:7]), epoca, sat, ok)


def parsear_tle(texto):
    """Texto en formato TLE (3 lineas con nombre o 2 sin nombre) -> lista de
    `Tle` en el orden del archivo. Ignora lineas vacias y comentarios '#'."""
    lin = [x.rstrip() for x in texto.splitlines()
           if x.strip() and not x.lstrip().startswith('#')]
    out, i = [], 0
    while i < len(lin):
        if lin[i].startswith('1 ') and i + 1 < len(lin) and lin[i + 1].startswith('2 '):
            out.append(tle_desde_lineas(f'NORAD {lin[i][2:7].strip()}',
                                        lin[i], lin[i + 1]))
            i += 2
        elif i + 2 < len(lin) and lin[i + 1].startswith('1 ') and lin[i + 2].startswith('2 '):
            out.append(tle_desde_lineas(lin[i], lin[i + 1], lin[i + 2]))
            i += 3
        else:
            raise ValueError(f'TLE ilegible cerca de: {lin[i][:40]!r}')
    return out


def buscar_datos(subruta=''):
    """Carpeta `data/` de la Estacion (o una subruta). Orden: variable de
    entorno ATP_DATA_DIR, /ws/data (contenedor) y los ancestros de este
    archivo (cubre `src/` y `install/`). Lanza FileNotFoundError si no existe."""
    cand = []
    if os.environ.get(DIR_DATOS_ENV):
        cand.append(os.environ[DIR_DATOS_ENV])
    cand.append('/ws/data')
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        cand.append(os.path.join(d, 'data'))
        d = os.path.dirname(d)
    for c in cand:
        p = os.path.join(c, subruta)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f'no encuentro data/{subruta}; define {DIR_DATOS_ENV}')


def cargar_catalogo(ruta=None):
    """Lee un archivo .tle o todos los .tle de una carpeta (por defecto
    data/tle) -> lista de `Tle`. Si un nombre aparece repetido, gana el de
    epoca mas reciente."""
    ruta = ruta or buscar_datos('tle')
    archivos = (sorted(glob.glob(os.path.join(ruta, '*.tle')))
                if os.path.isdir(ruta) else [ruta])
    por_clave = {}
    for a in archivos:
        with open(a) as f:
            for t in parsear_tle(f.read()):
                k = (t.norad, t.nombre)
                if k not in por_clave or t.epoca > por_clave[k].epoca:
                    por_clave[k] = t
    return list(por_clave.values())


def buscar_tle(catalogo, consulta):
    """Elige un satelite por nombre o por numero NORAD. Prioridad: NORAD
    exacto > nombre exacto > nombre que EMPIEZA con la consulta > nombre que
    la CONTIENE (sin distinguir mayusculas); a igualdad, el primero del
    catalogo. 'ISS' da 'ISS (ZARYA)'. Lanza KeyError si no hay."""
    q = str(consulta).strip().upper()
    if q.isdigit():
        for t in catalogo:
            if t.norad == int(q):
                return t
    for pred in (lambda n: n == q, lambda n: n.startswith(q), lambda n: q in n):
        for t in catalogo:
            if pred(t.nombre.upper()):
                return t
    raise KeyError(f'satelite {consulta!r} no esta en el catalogo '
                   f'({", ".join(t.nombre for t in catalogo[:8])}...)')


# ---------------------------------------------------------------- estaciones

@dataclass
class Estacion:
    """Estacion terrena. `lat_deg`/`lon_deg` GEODESICOS (WGS84), `alt_m`
    sobre el elipsoide. `mascara_deg` es la elevacion minima de operacion."""
    nombre: str
    lat_deg: float
    lon_deg: float
    alt_m: float = 0.0
    mascara_deg: float = 5.0
    descripcion: str = ''


def cargar_estaciones(ruta=None):
    """Lee data/estaciones.yaml -> (dict clave -> Estacion, clave_por_defecto).
    Formato: `por_defecto: <clave>` y `estaciones: {clave: {lat, lon, alt_m,
    mascara_deg, nombre}}`."""
    import yaml
    ruta = ruta or buscar_datos('estaciones.yaml')
    with open(ruta) as f:
        y = yaml.safe_load(f)
    est = {}
    for k, v in y['estaciones'].items():
        est[str(k)] = Estacion(
            nombre=v.get('nombre', str(k)), lat_deg=float(v['lat']),
            lon_deg=float(v['lon']), alt_m=float(v.get('alt_m', 0.0)),
            mascara_deg=float(v.get('mascara_deg', 5.0)),
            descripcion=v.get('descripcion', ''))
    return est, str(y.get('por_defecto', next(iter(est))))


def buscar_estacion(nombre=None, ruta=None):
    """Estacion por clave o por nombre (sin distinguir mayusculas); vacio o
    None -> la `por_defecto` del YAML. Lanza KeyError si no existe."""
    est, defecto = cargar_estaciones(ruta)
    q = str(nombre or defecto).strip().lower()
    for k, e in est.items():
        if q in (k.lower(), e.nombre.lower()):
            return e
    raise KeyError(f'estacion {nombre!r} no existe; hay: {", ".join(est)}')


# ---------------------------------------------------------------- marcos

def geodetica_a_ecef(lat_deg, lon_deg, alt_m=0.0):
    """(lat, lon, alt) geodesicos WGS84 -> ECEF (km). Exacta para el
    elipsoide WGS84 (formula cerrada con el radio de curvatura N)."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = A_WGS84_KM / math.sqrt(1.0 - E2_WGS84 * math.sin(lat) ** 2)
    h = alt_m / 1000.0
    return np.array([(n + h) * math.cos(lat) * math.cos(lon),
                     (n + h) * math.cos(lat) * math.sin(lon),
                     (n * (1.0 - E2_WGS84) + h) * math.sin(lat)])


def ecef_a_geodetica(r_ecef_km):
    """ECEF (km), forma (3,) o (N, 3) -> (lat_deg, lon_deg, alt_km) WGS84.
    Iteracion de punto fijo sobre la latitud (converge a < 1e-9 rad en 5
    pasos para cualquier punto por encima de ~100 km de altitud; no vale
    cerca del centro de la Tierra). Sirve para la altitud y el punto
    subsatelital."""
    r = np.atleast_2d(np.asarray(r_ecef_km, dtype=float))
    x, y, z = r[:, 0], r[:, 1], r[:, 2]
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - E2_WGS84))
    for _ in range(8):
        n = A_WGS84_KM / np.sqrt(1.0 - E2_WGS84 * np.sin(lat) ** 2)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1.0 - E2_WGS84 * n / (n + alt)))
    n = A_WGS84_KM / np.sqrt(1.0 - E2_WGS84 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - n
    out = (np.degrees(lat), np.degrees(np.arctan2(y, x)), alt)
    return out if np.ndim(r_ecef_km) > 1 else tuple(float(v[0]) for v in out)


def teme_a_ecef(r_teme, v_teme, jd, fr, dut1_s=0.0):
    """TEME -> ECEF (rotacion R3(GMST), sin movimiento del polo).
    `r_teme`, `v_teme` de forma (N, 3) en km y km/s; `jd`, `fr` (N,).
    La velocidad es la RELATIVA a la Tierra que gira: v_ecef = R v - w x r.
    Vale a ~0.5 km (UT1-UTC = 0) y ~10 m (polo): ver el docstring del modulo."""
    th = gmst_rad(jd, fr, dut1_s)
    c, s = np.cos(th), np.sin(th)
    r_teme, v_teme = np.atleast_2d(r_teme), np.atleast_2d(v_teme)

    def rot(x):
        return np.stack([c * x[:, 0] + s * x[:, 1],
                         -s * x[:, 0] + c * x[:, 1], x[:, 2]], axis=1)
    r = rot(r_teme)
    v = rot(v_teme)
    v = v - np.cross(np.array([0.0, 0.0, OMEGA_TIERRA]), r)
    return r, v


def matriz_enu(lat_deg, lon_deg):
    """Filas = ejes E, N, U expresados en ECEF para la latitud GEODESICA
    `lat_deg` (no la geocentrica: confundirlas da ~0.2 deg de error)."""
    la, lo = math.radians(lat_deg), math.radians(lon_deg)
    return np.array([
        [-math.sin(lo), math.cos(lo), 0.0],
        [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
        [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def enu_a_azel(e, n, u):
    """(E, N, U) -> (az_deg en [0, 360) desde el norte hacia el este, el_deg,
    rango). Ejemplo resuelto del curso: (400, 300, 500) -> az 53.13, el 45.0,
    d 707.1. Cuidado: atan2(E, N), no atan2(N, E)."""
    e, n, u = (np.asarray(x, dtype=float) for x in (e, n, u))
    rho = np.sqrt(e ** 2 + n ** 2 + u ** 2)
    az = np.degrees(np.arctan2(e, n)) % 360.0
    el = np.degrees(np.arcsin(np.clip(u / np.where(rho > 0, rho, 1.0), -1, 1)))
    return az, el, rho


# ---------------------------------------------------------------- propagacion

def propagar_teme(tle, jd, fr):
    """SGP4: posicion (km) y velocidad (km/s) en TEME. `jd`, `fr` arrays (N,).
    Vale mientras SGP4 converja (excentricidad < 1, perigeo sobre la
    superficie) y cerca de la epoca del TLE: ~1 km en la epoca, creciendo
    1-3 km/dia en LEO. Si SGP4 falla en un instante devuelve NaN ahi."""
    jd = np.atleast_1d(np.asarray(jd, dtype=float))
    fr = np.atleast_1d(np.asarray(fr, dtype=float))
    e, r, v = tle.satrec.sgp4_array(jd, fr)
    r, v = np.asarray(r, dtype=float), np.asarray(v, dtype=float)
    mal = np.asarray(e) != 0
    if mal.any():
        r[mal] = np.nan
        v[mal] = np.nan
    return r, v


def propagar_ecef(tle, jd, fr, dut1_s=0.0):
    """Posicion (km) y velocidad relativa a la Tierra (km/s) en ECEF."""
    r, v = propagar_teme(tle, jd, fr)
    return teme_a_ecef(r, v, jd, fr, dut1_s)


def observar(tle, estacion, jd, fr, dut1_s=0.0):
    """Lo que ve la estacion en cada instante (arrays de largo N):
    az_deg [0, 360), el_deg, rango_km, vr_km_s (+ = se aleja), mas
    r_ecef_km. Cumple lo documentado en el modulo: error ~1 km del TLE +
    ~0.5 km por UT1-UTC; sin refraccion. `vr` es la derivada EXACTA del
    rango (proyeccion de la velocidad relativa sobre la linea de vista), no
    una diferencia numerica."""
    r_ecef, v_ecef = propagar_ecef(tle, jd, fr, dut1_s)
    est = geodetica_a_ecef(estacion.lat_deg, estacion.lon_deg, estacion.alt_m)
    rel = r_ecef - est
    enu = rel @ matriz_enu(estacion.lat_deg, estacion.lon_deg).T
    az, el, rho = enu_a_azel(enu[:, 0], enu[:, 1], enu[:, 2])
    vr = np.einsum('ij,ij->i', rel, v_ecef) / rho     # la estacion esta quieta en ECEF
    return {'az_deg': az, 'el_deg': el, 'rango_km': rho, 'vr_km_s': vr,
            'r_ecef_km': r_ecef}


def observar_en(tle, estacion, t_utc, dut1_s=0.0):
    """`observar` en un solo instante (datetime) -> dict de escalares."""
    jd, fr = a_jd(t_utc)
    o = observar(tle, estacion, np.array([jd]), np.array([fr]), dut1_s)
    return {k: v[0] for k, v in o.items()}


# ---------------------------------------------------------------- pases

def _t_a_jdfr(t0_jd, t0_fr, seg):
    """Instante t0 + seg (s) como (jd, fr) arrays, sin perder precision."""
    seg = np.asarray(seg, dtype=float)
    return np.full_like(seg, t0_jd), t0_fr + seg / 86400.0


def _f(tle, est, t0, seg, clave, dut1_s):
    jd, fr = _t_a_jdfr(t0[0], t0[1], np.atleast_1d(seg))
    return observar(tle, est, jd, fr, dut1_s)[clave]


def _biseccion(g, a, b, tol_s):
    """Cero de g en [a, b] (g(a), g(b) de signo contrario), a `tol_s`."""
    ga = g(a)
    while b - a > tol_s:
        m = 0.5 * (a + b)
        gm = g(m)
        if (gm > 0) == (ga > 0):
            a, ga = m, gm
        else:
            b = m
    return 0.5 * (a + b)


def _seccion_aurea(g, a, b, tol_s):
    """Minimo de g (unimodal) en [a, b] por seccion aurea."""
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - phi * (b - a), a + phi * (b - a)
    gc, gd = g(c), g(d)
    while b - a > tol_s:
        if gc < gd:
            b, d, gd = d, c, gc
            c = b - phi * (b - a)
            gc = g(c)
        else:
            a, c, gc = c, d, gd
            d = a + phi * (b - a)
            gd = g(d)
    return 0.5 * (a + b)


def predecir_pases(tle, estacion, t_ini, horas=24.0, mascara_deg=None,
                   paso_s=20.0, tol_s=0.1, dut1_s=0.0, el_max_min_deg=0.0):
    """Pases del satelite sobre la estacion en [t_ini, t_ini + horas].

    Un pase es el intervalo con elevacion >= mascara (por defecto la de la
    estacion). Busqueda gruesa cada `paso_s` y cruces refinados por biseccion
    a `tol_s` (AOS, LOS); TCA (rango minimo) y elevacion maxima por seccion
    aurea. Devuelve una lista ordenada de dicts con: nombre, norad, aos, tca,
    t_el_max, los (datetime UTC), duracion_s, el_max_deg, az_aos_deg,
    az_los_deg, az_el_max_deg, rango_min_km, aos_truncado, los_truncado
    (True si el pase ya estaba en curso al inicio / sigue al final de la
    ventana: entonces AOS/LOS son los bordes de la ventana, no cruces reales).

    Cifras validas con la incertidumbre de la cadena (TLE ~1 km, UT1-UTC):
    ~1-2 s en los tiempos de AOS/LOS de un pase en LEO, ~0.1 deg en la
    elevacion, y se degradan con la edad del TLE. Un pase que roza la mascara
    (< `paso_s` de duracion) puede no detectarse: la busqueda gruesa asume
    que la elevacion no cruza la mascara dos veces entre dos muestras."""
    mask = estacion.mascara_deg if mascara_deg is None else float(mascara_deg)
    t0 = a_jd(t_ini)
    total = float(horas) * 3600.0
    n = int(math.ceil(total / paso_s)) + 1
    seg = np.minimum(np.arange(n) * paso_s, total)
    el = _f(tle, estacion, t0, seg, 'el_deg', dut1_s) - mask

    def g_el(s):
        return float(_f(tle, estacion, t0, s, 'el_deg', dut1_s)[0]) - mask

    def g_rango(s):
        return float(_f(tle, estacion, t0, s, 'rango_km', dut1_s)[0])

    def g_nel(s):
        return -float(_f(tle, estacion, t0, s, 'el_deg', dut1_s)[0])

    sobre = np.nan_to_num(el, nan=-1.0) >= 0.0
    pases = []
    i = 0
    while i < n:
        if not sobre[i]:
            i += 1
            continue
        # i es la primera muestra sobre la mascara de un tramo
        j = i
        while j + 1 < n and sobre[j + 1]:
            j += 1
        if i == 0:
            aos, aos_trunc = 0.0, True
        else:
            aos, aos_trunc = _biseccion(g_el, seg[i - 1], seg[i], tol_s), False
        if j == n - 1:
            los, los_trunc = seg[-1], True
        else:
            los, los_trunc = _biseccion(g_el, seg[j], seg[j + 1], tol_s), False
        i = j + 1
        if los - aos <= 0.0:
            continue
        # elevacion y rango son unimodales dentro de un pase LEO: seccion
        # aurea sobre todo [aos, los]
        t_pico = _seccion_aurea(g_nel, aos, los, tol_s)
        t_tca = _seccion_aurea(g_rango, aos, los, tol_s)
        o = observar(tle, estacion, *_t_a_jdfr(t0[0], t0[1], [aos, t_pico, los, t_tca]),
                     dut1_s=dut1_s)
        if o['el_deg'][1] < float(el_max_min_deg):
            continue

        def utc(s):
            return de_jd(t0[0], t0[1] + s / 86400.0)
        pases.append({
            'nombre': tle.nombre, 'norad': tle.norad,
            'aos': utc(aos), 'tca': utc(t_tca), 't_el_max': utc(t_pico),
            'los': utc(los), 'duracion_s': float(los - aos),
            'el_max_deg': float(o['el_deg'][1]),
            'az_aos_deg': float(o['az_deg'][0]),
            'az_los_deg': float(o['az_deg'][2]),
            'az_el_max_deg': float(o['az_deg'][1]),
            'rango_min_km': float(o['rango_km'][3]),
            'aos_truncado': aos_trunc, 'los_truncado': los_trunc,
        })
    return pases


def muestrear_pase(tle, estacion, pase, dt_s=0.5, dut1_s=0.0):
    """Pase muestreado, con el MISMO formato que `model.perfil_pase` para
    servir de referencia: t (s desde AOS), az (deg [0, 360)), az_continuo
    (deg, desenroscado), el, d (km), vr (km/s, + = se aleja), az_pto y
    el_pto (deg/s), duracion, d_min. Mas `inicio_utc` (datetime del AOS).

    Vale como las cifras de `predecir_pases`. `vr` sale de la velocidad
    ECEF (exacta), el resto de derivar numericamente az/el (diferencias
    centrales: error de segundo orden en `dt_s`). Cerca del cenit, az
    cambia muy rapido y `dt_s` grande lo submuestrea: es la fisica del
    keyhole, no un error."""
    dur = float(pase['duracion_s'])
    n = max(16, int(math.ceil(dur / dt_s)) + 1)
    t = np.linspace(0.0, dur, n)
    t0 = a_jd(pase['aos'])
    o = observar(tle, estacion, *_t_a_jdfr(t0[0], t0[1], t), dut1_s=dut1_s)
    az_c = np.degrees(np.unwrap(np.radians(o['az_deg'])))
    return {
        't': t, 'az': o['az_deg'], 'az_continuo': az_c, 'el': o['el_deg'],
        'd': o['rango_km'], 'vr': o['vr_km_s'],
        'az_pto': np.gradient(az_c, t), 'el_pto': np.gradient(o['el_deg'], t),
        'duracion': dur, 'd_min': float(np.min(o['rango_km'])),
        'inicio_utc': pase['aos'],
    }


def pases_json(pases):
    """Lista de pases -> estructura serializable a JSON (fechas ISO 8601 con
    'Z', numeros redondeados). Para el nodo `pass_predictor` y el backend."""
    def iso(t):
        return t.astimezone(UTC).strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
    out = []
    for p in pases:
        out.append({
            'nombre': p['nombre'], 'norad': p['norad'],
            'aos': iso(p['aos']), 'tca': iso(p['tca']),
            'los': iso(p['los']),
            'duracion_s': round(p['duracion_s'], 1),
            'el_max_deg': round(p['el_max_deg'], 2),
            'az_aos_deg': round(p['az_aos_deg'], 1),
            'az_los_deg': round(p['az_los_deg'], 1),
            'az_el_max_deg': round(p['az_el_max_deg'], 1),
            'rango_min_km': round(p['rango_min_km'], 1),
            'aos_truncado': bool(p['aos_truncado']),
            'los_truncado': bool(p['los_truncado']),
        })
    return out


# ---------------------------------------------------------------- pase listo para el lazo

def perfil_tle(sat, estacion=None, pase_idx=0, inicio_utc=None, horas=72.0,
               mascara_deg=None, el_max_min_deg=0.0, dt_s=0.5, dut1_s=0.0,
               dir_tle=None, archivo_estaciones=None):
    """Elige el `pase_idx`-esimo pase de `sat` sobre `estacion` a partir de
    `inicio_utc` (por defecto, ahora) y lo muestrea como referencia.
    Devuelve (perfil, pase, tle, estacion); `perfil` tiene el formato de
    `muestrear_pase` (t = 0 en AOS). `sat` y `estacion` son nombres (ver
    `buscar_tle`, `buscar_estacion`). Lanza LookupError si en `horas` no hay
    tantos pases. Valido como `predecir_pases`; ademas, un TLE muy lejos de su
    epoca (> ~7 dias) degrada el resultado: el llamador debe avisarlo con
    `tle.edad_dias`."""
    tle = buscar_tle(cargar_catalogo(dir_tle), sat)
    est = buscar_estacion(estacion, archivo_estaciones)
    t0 = a_utc(inicio_utc) if inicio_utc else dt.datetime.now(UTC)
    pases = predecir_pases(tle, est, t0, horas, mascara_deg,
                           el_max_min_deg=el_max_min_deg, dut1_s=dut1_s)
    if not 0 <= int(pase_idx) < len(pases):
        raise LookupError(
            f'{tle.nombre} sobre {est.nombre}: pase_index={pase_idx} pero solo '
            f'hay {len(pases)} pases en {horas:g} h desde {t0:%Y-%m-%d %H:%M} UTC')
    pase = pases[int(pase_idx)]
    return muestrear_pase(tle, est, pase, dt_s, dut1_s), pase, tle, est
