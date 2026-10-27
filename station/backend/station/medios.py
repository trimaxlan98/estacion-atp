"""Servir archivos de medios con soporte de `Range` (el navegador lo exige para buscar en un video).

Starlette 0.31 (la de la imagen) no trae `Range` en `FileResponse`, asi que se resuelve aqui:
una sola rango (`bytes=a-b`, `bytes=a-`, `bytes=-n`) responde 206 con `Content-Range`; un rango fuera
del archivo, 416; una cabecera `Range` mal formada (o con varios rangos) se ignora y se sirve el
archivo completo (RFC 7233 lo permite).
"""
import mimetypes
import re
from pathlib import Path

from fastapi import HTTPException
from fastapi.responses import Response, StreamingResponse

CHUNK = 1 << 20
RANGO_RE = re.compile(r'^bytes=(\d*)-(\d*)$')
TIPOS = {'.mp4': 'video/mp4', '.webm': 'video/webm', '.mkv': 'video/x-matroska', '.mov': 'video/quicktime',
         '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.png': 'image/png', '.gif': 'image/gif',
         '.svg': 'image/svg+xml', '.json': 'application/json', '.jsonl': 'application/x-ndjson',
         '.csv': 'text/csv', '.txt': 'text/plain; charset=utf-8', '.log': 'text/plain; charset=utf-8',
         '.ass': 'text/plain; charset=utf-8', '.pdf': 'application/pdf', '.mcap': 'application/octet-stream'}


def confinar(base, ruta):
    """Ruta absoluta de `ruta` (relativa, sin `..`) dentro de `base`, o 404. Resuelve enlaces
    simbolicos: uno que apunte fuera de `base` tambien es 404."""
    if (not ruta or ruta.startswith('/') or '\\' in ruta or '\0' in ruta
            or any(p in ('', '.', '..') for p in ruta.split('/'))):
        raise HTTPException(404, 'medio no encontrado')
    base = base.resolve()
    p = (base / ruta).resolve()
    try:
        p.relative_to(base)
    except ValueError:
        raise HTTPException(404, 'medio no encontrado') from None
    if not p.is_file():
        raise HTTPException(404, 'medio no encontrado')
    return p


def tipo_de(path):
    return TIPOS.get(path.suffix.lower()) or mimetypes.guess_type(path.name)[0] or 'application/octet-stream'


def parse_rango(cabecera, size):
    """(inicio, fin) inclusivos, o None si la cabecera no es un rango unico valido (se ignora).
    Lanza 416 si el rango no cae en el archivo."""
    if not cabecera:
        return None
    m = RANGO_RE.match(cabecera.strip())
    if not m or (m.group(1) == '' and m.group(2) == ''):
        return None
    a, b = m.group(1), m.group(2)
    if a == '':                                  # sufijo: los ultimos n bytes
        n = int(b)
        if n == 0:
            raise HTTPException(416, 'rango no satisfacible', headers={'Content-Range': f'bytes */{size}'})
        return max(size - n, 0), size - 1
    ini = int(a)
    fin = size - 1 if b == '' else min(int(b), size - 1)
    if ini >= size or fin < ini:
        raise HTTPException(416, 'rango no satisfacible', headers={'Content-Range': f'bytes */{size}'})
    return ini, fin


def _leer(path, ini, fin):
    restante = fin - ini + 1
    with open(path, 'rb') as f:
        f.seek(ini)
        while restante > 0:
            b = f.read(min(CHUNK, restante))
            if not b:
                break
            restante -= len(b)
            yield b


def servir(path, rango=None, descargar=False, metodo='GET'):
    """Respuesta 200/206 de un archivo. `rango` = valor de la cabecera `Range`."""
    st = path.stat()
    size = st.st_size
    cabeceras = {'Accept-Ranges': 'bytes', 'ETag': f'"{st.st_mtime_ns:x}-{size:x}"',
                 'Cache-Control': 'no-cache'}
    if descargar:
        cabeceras['Content-Disposition'] = f'attachment; filename="{path.name}"'
    r = parse_rango(rango, size)
    if r is None:
        ini, fin, codigo = 0, size - 1, 200
    else:
        (ini, fin), codigo = r, 206
        cabeceras['Content-Range'] = f'bytes {ini}-{fin}/{size}'
    largo = max(fin - ini + 1, 0)
    cabeceras['Content-Length'] = str(largo)
    if metodo == 'HEAD' or size == 0:
        return Response(status_code=codigo, headers=cabeceras, media_type=tipo_de(path))
    return StreamingResponse(_leer(path, ini, fin), status_code=codigo, headers=cabeceras,
                             media_type=tipo_de(path))
