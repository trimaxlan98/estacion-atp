"""Exportar la telemetria de una corrida como MCAP (abrible en Foxglove).

No hay libreria `mcap` en la imagen (y `rosbag2_py` solo escribe mensajes ROS), asi que se escribe
el contenedor a mano: es un formato sencillo (https://mcap.dev/spec). Se usa la variante mas simple
y valida: **sin chunks, sin indices y sin CRC** (los lectores recorren el archivo de forma lineal);
mensajes `json` con esquema `jsonschema`, un canal por archivo de la corrida:

    /telemetria   telemetry.jsonl   (CONTRATO 2)
    /red          red.jsonl         (CONTRATO 12)
    /eventos      eventos.jsonl     (CONTRATO 12)

`log_time` = instante de creacion de la corrida + `t` (s simulados), o `t` solo si no hay base.
La salida es un generador de bloques: no se carga el archivo entero en memoria.
"""
import json
import struct

MAGIC = b'\x89MCAP0\r\n'
OP_HEADER, OP_FOOTER, OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_DATA_END = 0x01, 0x02, 0x03, 0x04, 0x05, 0x0F
SCHEMA_JSON = b'{"type":"object"}'
CANALES = (('/telemetria', 'telemetry.jsonl'), ('/red', 'red.jsonl'), ('/eventos', 'eventos.jsonl'))


def _s(texto):
    b = texto.encode('utf-8')
    return struct.pack('<I', len(b)) + b


def _rec(op, cuerpo):
    return struct.pack('<BQ', op, len(cuerpo)) + cuerpo


def header(perfil='', libreria='estacion-atp'):
    return _rec(OP_HEADER, _s(perfil) + _s(libreria))


def schema(sid, nombre):
    return _rec(OP_SCHEMA, struct.pack('<H', sid) + _s(nombre) + _s('jsonschema')
                + struct.pack('<I', len(SCHEMA_JSON)) + SCHEMA_JSON)


def channel(cid, sid, topico):
    return _rec(OP_CHANNEL, struct.pack('<HH', cid, sid) + _s(topico) + _s('json')
                + struct.pack('<I', 0))                       # metadata vacia


def message(cid, seq, t_ns, datos):
    return _rec(OP_MESSAGE, struct.pack('<HIQQ', cid, seq, t_ns, t_ns) + datos)


def cierre():
    return _rec(OP_DATA_END, struct.pack('<I', 0)) + _rec(OP_FOOTER, struct.pack('<QQI', 0, 0, 0)) + MAGIC


def archivos_disponibles(run_dir):
    return [(top, run_dir / n) for top, n in CANALES if (run_dir / n).is_file()]


def generar(run_dir, base_epoch_s=0.0, bloque=1 << 18):
    """Bloques de bytes de un MCAP con lo que haya en la carpeta de la corrida (vacio si no hay
    ningun archivo de series)."""
    fuentes = archivos_disponibles(run_dir)
    buf = bytearray(MAGIC + header())
    for cid, (top, _) in enumerate(fuentes, 1):
        buf += schema(cid, top.strip('/')) + channel(cid, cid, top)
    seq = 0
    for cid, (_, ruta) in enumerate(fuentes, 1):
        with open(ruta, encoding='utf-8', errors='replace') as f:
            for ln in f:
                ln = ln.strip()
                if not ln:
                    continue
                try:
                    obj = json.loads(ln)
                    t = float(obj['t']) if isinstance(obj, dict) and 't' in obj else 0.0
                except (ValueError, TypeError):
                    continue                    # linea a medio escribir (corrida en curso)
                seq += 1
                buf += message(cid, seq, int((base_epoch_s + t) * 1e9),
                               json.dumps(obj, separators=(',', ':')).encode('utf-8'))
                if len(buf) >= bloque:
                    yield bytes(buf)
                    buf.clear()
    buf += cierre()
    yield bytes(buf)


def leer(datos):
    """Lector minimo (para las pruebas y para comprobar el archivo): devuelve
    {'canales': {id: topico}, 'mensajes': [(canal, seq, log_time, dict)], 'perfil', 'libreria'}.
    Valida la estructura (magic, longitudes, DataEnd, Footer)."""
    if datos[:8] != MAGIC or datos[-8:] != MAGIC:
        raise ValueError('magic invalido')
    pos, fin = 8, len(datos) - 8
    out = {'canales': {}, 'esquemas': {}, 'mensajes': []}
    visto = []
    while pos < fin:
        op, n = struct.unpack_from('<BQ', datos, pos)
        cuerpo = datos[pos + 9: pos + 9 + n]
        if len(cuerpo) != n:
            raise ValueError('registro truncado')
        pos += 9 + n
        visto.append(op)
        if op == OP_HEADER:
            (n1,) = struct.unpack_from('<I', cuerpo, 0)
            out['perfil'] = cuerpo[4:4 + n1].decode()
            (n2,) = struct.unpack_from('<I', cuerpo, 4 + n1)
            out['libreria'] = cuerpo[8 + n1: 8 + n1 + n2].decode()
        elif op == OP_SCHEMA:
            sid, n1 = struct.unpack_from('<HI', cuerpo, 0)
            out['esquemas'][sid] = cuerpo[6:6 + n1].decode()
        elif op == OP_CHANNEL:
            cid, sid, n1 = struct.unpack_from('<HHI', cuerpo, 0)
            out['canales'][cid] = cuerpo[8:8 + n1].decode()
        elif op == OP_MESSAGE:
            cid, seq, lt, _pt = struct.unpack_from('<HIQQ', cuerpo, 0)
            out['mensajes'].append((cid, seq, lt, json.loads(cuerpo[22:])))
        elif op == OP_DATA_END:
            pass
        elif op == OP_FOOTER:
            if pos != fin:
                raise ValueError('el footer no es el ultimo registro')
        else:
            raise ValueError(f'registro desconocido {op:#x}')
    if visto[0] != OP_HEADER or visto[-1] != OP_FOOTER or OP_DATA_END not in visto:
        raise ValueError('estructura invalida')
    return out
