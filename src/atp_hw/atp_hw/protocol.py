"""Protocolo de texto de Hamlib `rotctld` (modo simple, sin '+' extendido).

Logica pura, sin sockets, para poder probarla sin hardware.

Comandos que usa la Estacion:
    P <az> <el>\\n   -> RPRT <n>\\n          (n = 0 ok, n < 0 error de Hamlib)
    p\\n             -> <az>\\n<el>\\n       (o RPRT <n>\\n si falla)
    S\\n             -> RPRT <n>\\n          (stop)
    \\dump_state\\n  -> lineas clave=valor ... done\\n  (limites del rotor)
"""
import math


class RotctldError(Exception):
    """Cualquier fallo del cliente rotctld."""


class RotctldConnectionError(RotctldError):
    """Socket caido, timeout o respuesta truncada: hay que reconectar."""


class RotctldProtocolError(RotctldError):
    """Respuesta que no se puede interpretar."""


# Codigos de hamlib/rig.h (los que importan para un rotor)
RPRT_NOMBRES = {
    -1: 'EINVAL: argumento invalido o fuera de rango',
    -2: 'ECONF: configuracion invalida',
    -3: 'ENOMEM',
    -4: 'ENIMPL: no implementado',
    -5: 'ETIMEOUT: el rotor no contesto',
    -6: 'EIO: error de entrada/salida',
    -7: 'EINTERNAL',
    -8: 'EPROTO: error de protocolo con el rotor',
    -9: 'ERJCTED: el rotor rechazo el comando',
    -11: 'ENAVAIL: funcion no disponible',
    -12: 'ENTARGET: objetivo no accesible',
}


class RotctldRPRT(RotctldError):
    """El demonio contesto `RPRT <n>` con n != 0 (rechazo del comando)."""

    def __init__(self, code):
        super().__init__(f'RPRT {code} ({RPRT_NOMBRES.get(code, "desconocido")})')
        self.code = code


def format_set_position(az_deg, el_deg):
    """`P az el\\n` con 4 decimales (0.0001 deg, mucho mas fino que cualquier rotor)."""
    if not (math.isfinite(az_deg) and math.isfinite(el_deg)):
        raise ValueError('posicion no finita')
    return f'P {az_deg:.4f} {el_deg:.4f}\n'


def parse_rprt(line):
    """`RPRT -1` -> -1. Devuelve None si la linea no es un RPRT."""
    s = line.strip()
    if not s.startswith('RPRT'):
        return None
    try:
        return int(s.split()[1])
    except (IndexError, ValueError) as e:
        raise RotctldProtocolError(f'RPRT malformado: {line!r}') from e


def check_rprt(line):
    """Valida la respuesta de un comando que solo contesta RPRT (P, S)."""
    code = parse_rprt(line)
    if code is None:
        raise RotctldProtocolError(f'se esperaba RPRT y llego {line!r}')
    if code != 0:
        raise RotctldRPRT(code)


def parse_float(line):
    try:
        v = float(line.strip())
    except ValueError as e:
        raise RotctldProtocolError(f'numero invalido: {line!r}') from e
    if not math.isfinite(v):
        raise RotctldProtocolError(f'numero no finito: {line!r}')
    return v


def parse_position(lines):
    """Respuesta de `p`: dos lineas (az, el). Si la primera es RPRT n != 0, lanza."""
    if lines and parse_rprt(lines[0]) not in (None, 0):
        raise RotctldRPRT(parse_rprt(lines[0]))
    if len(lines) < 2:
        raise RotctldProtocolError(f'respuesta de p incompleta: {lines!r}')
    return parse_float(lines[0]), parse_float(lines[1])


def parse_dump_state(lines):
    """Pares clave=valor de `\\dump_state` (min_az, max_az, min_el, max_el...).
    Ignora lo que no sea clave=valor; sirve para Hamlib 4.x."""
    out = {}
    for ln in lines:
        if '=' not in ln:
            continue
        k, v = ln.split('=', 1)
        try:
            out[k.strip()] = float(v)
        except ValueError:
            out[k.strip()] = v.strip()
    return out
