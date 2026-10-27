#!/usr/bin/env python3
"""Escribe (y valida) el `media.json` de un trabajo de render (docs/APP.md seccion 4, docs/RENDER.md).

Solo biblioteca estandar + `ffprobe`/`ffmpeg` del host. Lo llaman los envoltorios `scripts/render/*.sh`
al terminar (siempre, tambien si fallaron: asi el backend siempre encuentra un `media.json` valido).

  python3 scripts/render/media_json.py --salida <dir> --render gz-demo --run <run_dir> \\
      --principal demo.mp4 --archivo eval.json --miniatura-de demo.mp4 \\
      --comando-json '["bash","scripts/render/gz-demo.sh",...]' --args-json '{"el_max":85}' --inicio 1759650000

  python3 scripts/render/media_json.py --validar <dir>/media.json      # sale con 1 y dice que falla

Campos (los de APP.md 4 mas `estado` y `error`, aditivos): `archivos[]` con `ruta`, `tipo`
(video|imagen|datos), `ancho`, `alto`, `duracion_s` (video), `sha256` y `principal`; `procedencia` con el
commit, si el arbol estaba sucio, el comando, los argumentos y la corrida de origen.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
VIDEO = ('.mp4', '.webm', '.mov', '.mkv')
IMAGEN = ('.png', '.jpg', '.jpeg', '.gif', '.webp')
MINI_W, MINI_H = 640, 360
ESTADOS = ('ok', 'parcial', 'error')


def sha256(ruta):
    h = hashlib.sha256()
    with open(ruta, 'rb') as f:
        for bloque in iter(lambda: f.read(1 << 20), b''):
            h.update(bloque)
    return h.hexdigest()


def tipo_de(ruta):
    ext = os.path.splitext(ruta)[1].lower()
    return 'video' if ext in VIDEO else 'imagen' if ext in IMAGEN else 'datos'


def sonda(ruta):
    """{'ancho','alto','duracion_s'} con ffprobe; vacio si no se puede leer."""
    try:
        out = subprocess.run(
            ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
             'stream=width,height:format=duration', '-of', 'json', ruta],
            capture_output=True, text=True, timeout=60, check=True).stdout
        d = json.loads(out)
        r = {}
        if d.get('streams'):
            r['ancho'] = int(d['streams'][0]['width'])
            r['alto'] = int(d['streams'][0]['height'])
        dur = d.get('format', {}).get('duration')
        if dur not in (None, 'N/A'):
            r['duracion_s'] = round(float(dur), 2)
        return r
    except Exception:
        return {}


def git(*args):
    try:
        return subprocess.run(['git', '-C', RAIZ, *args], capture_output=True, text=True,
                              timeout=30).stdout.strip()
    except Exception:
        return ''


def procedencia(comando, args, run_dir):
    prov = {'commit': git('rev-parse', 'HEAD') or None,
            'sucio': bool(git('status', '--porcelain')),
            'comando': comando, 'args': args}
    corrida = {'run_id': os.path.basename(os.path.normpath(run_dir)) if run_dir else None}
    p = os.path.join(run_dir or '', 'params.json')
    if run_dir and os.path.isfile(p):
        try:
            d = json.load(open(p, encoding='utf-8'))
            corrida['experiment_id'] = d.get('experiment_id')
            corrida['args'] = d.get('args', {})
        except Exception:
            pass
    prov['corrida'] = corrida
    return prov


def miniatura(salida, fuente, t=-1.0):
    """miniatura.jpg de 640x360 (con bandas si la proporcion no es 16:9) desde un video o una imagen."""
    destino = os.path.join(salida, 'miniatura.jpg')
    fuente_abs = os.path.join(salida, fuente)
    filtro = (f'scale={MINI_W}:{MINI_H}:force_original_aspect_ratio=decrease,'
              f'pad={MINI_W}:{MINI_H}:(ow-iw)/2:(oh-ih)/2:color=0x080F15')
    cmd = ['ffmpeg', '-v', 'error', '-y']
    if tipo_de(fuente) == 'video':
        dur = sonda(fuente_abs).get('duracion_s', 0.0)
        ss = t if t >= 0 else dur * 0.92          # ya con el resumen o el final a la vista
        cmd += ['-ss', f'{min(ss, max(dur - 0.1, 0)):.2f}']
    cmd += ['-i', fuente_abs, '-frames:v', '1', '-vf', filtro, '-q:v', '3', destino]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if r.returncode != 0 or not os.path.isfile(destino):
        return None
    return 'miniatura.jpg'


def entrada(salida, ruta, principal=False):
    p = os.path.join(salida, ruta)
    e = {'ruta': ruta, 'tipo': tipo_de(ruta)}
    if e['tipo'] in ('video', 'imagen'):
        e.update(sonda(p))
        if e['tipo'] == 'imagen':
            e.pop('duracion_s', None)
    e['sha256'] = sha256(p)
    e['bytes'] = os.path.getsize(p)
    if principal:
        e['principal'] = True
    return e


def escribir(a):
    salida = os.path.abspath(a.salida)
    os.makedirs(salida, exist_ok=True)
    estado, error = a.estado, a.error
    archivos = []
    vistos = set()

    def agregar(ruta, principal=False):
        if ruta and ruta not in vistos and os.path.isfile(os.path.join(salida, ruta)):
            vistos.add(ruta)
            archivos.append(entrada(salida, ruta, principal))
            return True
        return False

    if a.principal and not agregar(a.principal, True):
        estado = 'error'
        error = error or f'no se genero el medio principal ({a.principal})'
    for r in a.archivo:
        agregar(r)
    if a.miniatura_de and os.path.isfile(os.path.join(salida, a.miniatura_de)) and 'miniatura.jpg' not in vistos:
        m = miniatura(salida, a.miniatura_de, a.mini_t)
        if m:
            agregar(m)
    try:
        comando = json.loads(a.comando_json)
        args = json.loads(a.args_json)
    except ValueError:
        comando, args = [], {}
    doc = {'version': 1,
           'job_id': a.job_id or os.environ.get('RENDER_JOB_ID') or os.environ.get('JOB_ID') or os.path.basename(salida.rstrip('/')),
           'render': a.render,
           'run_id': os.path.basename(os.path.normpath(a.run)) if a.run else None,
           'creado_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
           'duracion_pared_s': round(time.time() - a.inicio, 1) if a.inicio else None,
           'estado': estado,
           'archivos': archivos,
           'procedencia': procedencia(comando, args, a.run),
           'notas': a.notas}
    if error:
        doc['error'] = error
    tmp = os.path.join(salida, '.media.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, ensure_ascii=False)
        f.write('\n')
    os.replace(tmp, os.path.join(salida, 'media.json'))
    return doc


def validar(ruta):
    """Lista de problemas de un media.json (vacia = valido)."""
    errs = []
    try:
        d = json.load(open(ruta, encoding='utf-8'))
    except Exception as e:
        return [f'no es JSON legible: {e}']
    base = os.path.dirname(os.path.abspath(ruta))
    for k in ('version', 'job_id', 'render', 'creado_utc', 'archivos', 'procedencia'):
        if k not in d:
            errs.append(f'falta `{k}`')
    if d.get('estado', 'ok') not in ESTADOS:
        errs.append(f'estado invalido: {d.get("estado")}')
    arch = d.get('archivos', [])
    if not isinstance(arch, list):
        return errs + ['`archivos` no es una lista']
    for e in arch:
        for k in ('ruta', 'tipo', 'sha256'):
            if k not in e:
                errs.append(f'archivo sin `{k}`: {e}')
        p = os.path.join(base, e.get('ruta', ''))
        if not os.path.isfile(p):
            errs.append(f'no existe {e.get("ruta")}')
        elif e.get('sha256') and sha256(p) != e['sha256']:
            errs.append(f'sha256 distinto en {e["ruta"]}')
        if e.get('tipo') in ('video', 'imagen') and not (e.get('ancho') and e.get('alto')):
            errs.append(f'{e.get("ruta")}: faltan ancho/alto')
        if e.get('tipo') == 'video' and not e.get('duracion_s'):
            errs.append(f'{e.get("ruta")}: falta duracion_s')
    if d.get('estado', 'ok') != 'error':
        if sum(1 for e in arch if e.get('principal')) != 1:
            errs.append('debe haber exactamente un archivo `principal`')
        for e in arch:
            if e.get('tipo') == 'video' and not any(
                    m.get('ruta') == 'miniatura.jpg' for m in arch):
                errs.append('un video exige miniatura.jpg')
                break
    for e in arch:
        if e.get('ruta') == 'miniatura.jpg' and (e.get('ancho'), e.get('alto')) != (MINI_W, MINI_H):
            errs.append(f'miniatura.jpg debe ser {MINI_W}x{MINI_H}')
    prov = d.get('procedencia', {})
    for k in ('commit', 'sucio', 'comando', 'args', 'corrida'):
        if k not in prov:
            errs.append(f'procedencia sin `{k}`')
    return errs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--validar', metavar='MEDIA_JSON')
    ap.add_argument('--salida')
    ap.add_argument('--render', default='')
    ap.add_argument('--run', default='')
    ap.add_argument('--job-id', default='')
    ap.add_argument('--inicio', type=float, default=0.0, help='epoch de arranque (duracion_pared_s)')
    ap.add_argument('--comando-json', default='[]')
    ap.add_argument('--args-json', default='{}')
    ap.add_argument('--principal', default='')
    ap.add_argument('--archivo', action='append', default=[])
    ap.add_argument('--miniatura-de', default='')
    ap.add_argument('--mini-t', type=float, default=-1.0, help='s del fotograma de la miniatura (-1: 92 %%)')
    ap.add_argument('--notas', default='')
    ap.add_argument('--estado', default='ok', choices=ESTADOS)
    ap.add_argument('--error', default='')
    a = ap.parse_args(argv)
    if a.validar:
        errs = validar(a.validar)
        for e in errs:
            print('media.json:', e, file=sys.stderr)
        print('media.json valido' if not errs else f'media.json INVALIDO ({len(errs)} problemas)')
        return 1 if errs else 0
    if not a.salida:
        ap.error('falta --salida')
    doc = escribir(a)
    print(f'media.json: {len(doc["archivos"])} archivos, estado {doc["estado"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
