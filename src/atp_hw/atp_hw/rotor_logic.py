"""Logica pura del puente al rotor: acimut continuo -> rango del rotor,
limites de elevacion, banda muerta y tasa maxima de comandos.

--- Acimut continuo y "flip" -------------------------------------------------
La referencia de la Estacion (`/atp/reference`) trae el acimut DESENROSCADO
(continuo): un pase que cruza el norte va ..., 358, 359, 360, 361, ... y no
vuelve a 0. Un rotor real solo acepta un rango [az_min, az_max]:

  * rango de 360 deg (0..360): cada acimut tiene UN solo valor valido. Al
    cruzar el norte la orden salta de ~360 a ~0: el rotor debe desenroscar
    360 deg (a 6 deg/s, un minuto entero sin ver el satelite). Eso es el
    "flip" y con 360 deg exactos es inevitable.
  * rango con solape (p. ej. 0..450 del Yaesu G-5500, o -180..540 de algunos
    SPID/Alfa): entre az_min+360 y az_max el mismo acimut tiene DOS valores
    validos (a y a+360). Mientras el pase siga dentro del solape no hay flip:
    se elige el valor mas cercano a la ultima orden (continuidad). El flip
    solo ocurre cuando la referencia sale por el extremo del rango.

Eleccion del valor:
  1. primera orden (rotor en reposo): `prefer='low'` toma el menor valido
     (deja el solape por delante para pases de acimut creciente),
     `prefer='high'` el mayor; `prefer='near'` lo decide el puente con la
     posicion leida del rotor (pasa esa posicion como ancla).
  2. ordenes siguientes: el valido mas cercano a la ultima orden enviada.
"""
import math


def wrap180(x):
    """Lleva un angulo (deg) a [-180, 180)."""
    return (float(x) + 180.0) % 360.0 - 180.0


def azimuth_candidates(az_c, az_min, az_max):
    """Todos los az_c + 360 k dentro de [az_min, az_max] (ordenados)."""
    eps = 1e-9
    k_lo = math.ceil((az_min - az_c) / 360.0 - eps)
    k_hi = math.floor((az_max - az_c) / 360.0 + eps)
    return [az_c + 360.0 * k for k in range(int(k_lo), int(k_hi) + 1)]


def map_azimuth(az_c, anchor, az_min, az_max, prefer='low'):
    """Acimut continuo -> orden dentro del rango del rotor.

    `anchor`: ultima orden enviada (o None). Devuelve (az_cmd, status) con
    status 'ok' o 'clamped' (el rango es < 360 y el objetivo cae en la zona
    muerta: se manda al limite mas cercano).
    """
    if az_max <= az_min:
        raise ValueError('az_max debe ser mayor que az_min')
    cands = azimuth_candidates(az_c, az_min, az_max)
    if not cands:
        # rango < 360 y objetivo en la zona que el rotor no cubre
        d_min = abs(wrap180(az_c - az_min))
        d_max = abs(wrap180(az_c - az_max))
        return (az_min if d_min <= d_max else az_max), 'clamped'
    if anchor is None:
        return (cands[-1] if prefer == 'high' else cands[0]), 'ok'
    # mas cercano al ancla; en empate, el menor (min devuelve el primero)
    return min(cands, key=lambda c: abs(c - anchor)), 'ok'


def is_flip(az_cmd, last_az_cmd):
    """Una orden que salta mas de media vuelta obliga al rotor a desenroscar."""
    return last_az_cmd is not None and abs(az_cmd - last_az_cmd) > 180.0


def clamp_elevation(el, el_min, el_max):
    """Recorta la elevacion a los limites. Devuelve (el, recortada)."""
    c = min(max(float(el), el_min), el_max)
    return c, c != el


def needs_update(cmd, last_cmd, res_az, res_el):
    """Banda muerta: no mandar si el cambio es menor que la resolucion del
    rotor en AMBOS ejes. `cmd`, `last_cmd` = (az, el); last_cmd None -> mandar."""
    if last_cmd is None:
        return True
    return (abs(cmd[0] - last_cmd[0]) >= res_az - 1e-9
            or abs(cmd[1] - last_cmd[1]) >= res_el - 1e-9)


def meas_in_ref_frame(rotor_az, ref_az):
    """Acimut del rotor (0..360 o similar) en el marco continuo de la
    referencia: el equivalente a `rotor_az` (modulo 360) mas cercano a
    `ref_az`. Asi `ref - meas` es el error angular FISICO, en [-180, 180),
    y no un +-360 espurio cuando el rotor y la referencia usan marcos
    distintos."""
    return ref_az - wrap180(ref_az - rotor_az)


class RateLimiter:
    """Tasa maxima de comandos: `ready(t)` dice si ya paso 1/max_hz desde el
    ultimo `mark`. El reloj lo pone quien llama (monotonic en el puente)."""

    def __init__(self, max_hz):
        self.min_dt = 0.0 if max_hz <= 0 else 1.0 / max_hz
        self.last = None

    def ready(self, t):
        return self.last is None or (t - self.last) >= self.min_dt - 1e-9

    def mark(self, t):
        self.last = t
