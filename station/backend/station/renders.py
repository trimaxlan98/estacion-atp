"""Renders (docs/APP.md 4): catalogo `render/*.yaml`, trabajos, `media.json` y galeria de medios.

Un renderizador transforma una corrida en medios. Cada trabajo escribe en
`runs/<run_id>/media/<job_id>/` y el script deja ahi un `media.json` (version 1) con los archivos,
sus metadatos y la **procedencia** exacta (commit, comando, args, corrida). El backend NO inventa
medios: si el script termina bien pero `media.json` falta o no cumple el contrato, el trabajo es
`error` (y dice por que). Estados: `en_cola | esperando_carga | corriendo | hecho | error |
cancelado | interrumpida`.
"""
import asyncio
import hashlib
import json
import logging
import os
import re
import secrets
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

from station.config import CLASES, RENDER_ID_RE, es_final
from station.db import now_iso
from station.executor import ESPERA_MAX_S, GovernedExecutor
from station.experiments import (CONTRATO_RE, KINDS, ArgError, check_args_y_esquema, check_comando,
                                 check_estilo, check_recursos, kind_of_run, proceso_args,
                                 safe_relpath, sustituir_marcadores)
from station.governor import Governor, muerte_por_senal

log = logging.getLogger('station.renders')

MARCADORES_RENDER = ('run_id', 'run_dir', 'runs_dir', 'salida', 'pantalla', 'job_id', 'root', 'domain')
EJECUTORES = ('backend', 'host')      # quien lanza el render (APP 4.1); por defecto el propio backend
REQUIERE_CLAVES = ('contrato', 'kind', 'archivo', 'cualquiera')
TIPOS_MEDIO = ('video', 'imagen', 'audio', 'datos', 'documento')
SHA_RE = re.compile(r'^[0-9a-f]{64}$')
PROGRESO_RE = re.compile(r'^PROGRESO\s+([0-9]*\.?[0-9]+(?:[eE][-+]?\d+)?)\s*$')
# archivos canonicos de cada contrato de artefacto (si el YAML de la escena no los declara)
CONTRATO_ARCHIVOS = {'red': ('escena.json', 'red.jsonl'), 'red_eventos': ('eventos.jsonl',),
                     'topologia': ('topologia.jsonl',)}
MINIATURA = 'miniatura.jpg'


class MediaError(ValueError):
    """media.json que no cumple el contrato (el trabajo pasa a `error`)."""


# ================================================================ catalogo render/*.yaml
def _check_render(d, stem):
    if not isinstance(d, dict):
        raise ValueError('el YAML no es un mapa')
    for k in ('id', 'title', 'requiere', 'comando', 'recursos'):
        if k not in d:
            raise ValueError(f'falta la clave {k}')
    if not RENDER_ID_RE.match(str(d['id'])) or d['id'] != stem:
        raise ValueError('id invalido o distinto del nombre del archivo')
    req = d['requiere']
    if not isinstance(req, dict) or len(req) != 1 or next(iter(req)) not in REQUIERE_CLAVES:
        raise ValueError(f'requiere debe ser un mapa con una sola clave de {REQUIERE_CLAVES}')
    (clave, val), = req.items()
    if clave == 'contrato' and not (isinstance(val, str) and CONTRATO_RE.match(val)):
        raise ValueError('requiere.contrato invalido')
    # un kind o una lista de kinds (manim-corrida: lazo -> «El pase», campana -> «La campana»)
    if clave == 'kind' and not (val in KINDS if isinstance(val, str) else
                                isinstance(val, list) and val and all(isinstance(k, str) and k in KINDS for k in val)):
        raise ValueError(f'requiere.kind debe ser uno de {KINDS} o una lista de ellos')
    if clave == 'archivo':
        safe_relpath(val, 'requiere.archivo')
    if clave == 'cualquiera' and val is not True:
        raise ValueError('requiere.cualquiera solo admite true')
    d.setdefault('ejecutor', 'backend')
    if d['ejecutor'] not in EJECUTORES:
        raise ValueError(f'ejecutor debe ser uno de {EJECUTORES}')
    d.setdefault('pantalla', False)
    if not isinstance(d['pantalla'], bool):
        raise ValueError('pantalla debe ser booleano')
    check_comando(d, MARCADORES_RENDER)       # {pantalla} vale '' si el render no pide pantalla
    check_estilo(d)
    check_recursos(d)
    if 'duracion_estimada_s' in d and (isinstance(d['duracion_estimada_s'], bool)
                                       or not isinstance(d['duracion_estimada_s'], (int, float))
                                       or d['duracion_estimada_s'] <= 0):
        raise ValueError('duracion_estimada_s debe ser un numero positivo')
    d.setdefault('args', {})
    d.setdefault('params_schema', {})
    check_args_y_esquema(d)


def load_render_catalog(directory):
    """`render/*.yaml` validos por id; los invalidos se omiten con un aviso (no tumban la app)."""
    out = {}
    if not directory.is_dir():
        return out
    for p in sorted(directory.glob('*.yaml')):
        try:
            d = yaml.safe_load(p.read_text(encoding='utf-8'))
            _check_render(d, p.stem)
            d['description'] = ' '.join(str(d.get('description', '')).split())
            out[d['id']] = d
        except Exception as ex:  # noqa: BLE001
            log.warning('render %s ignorado: %s', p.name, ex)
    return out


def render_spec(render):
    """Lo que el trabajo congela del YAML del render."""
    return {k: render.get(k) for k in ('comando', 'estilo_args', 'recursos', 'pantalla',
                                       'duracion_estimada_s', 'ejecutor')}


# ================================================================ requisitos de una corrida
def artefactos_de(run, run_dir, catalogo=None):
    """Artefactos conocidos de una corrida: los de su spec (`proceso`) o los del YAML de su
    escena, con `existe`. Si no declara ninguno de un contrato canonico (p. ej. `red`), se infiere
    por sus archivos."""
    decl = list((run.get('spec') or {}).get('artefactos') or [])
    if not decl and catalogo:
        decl = list((catalogo.get(run['experiment_id']) or {}).get('artefactos') or [])
    out = [{'ruta': a['ruta'], 'contrato': a.get('contrato'),
            'existe': (run_dir / a['ruta']).is_file()} for a in decl]
    declarados = {a['contrato'] for a in out if a['contrato']}
    for c, archivos in CONTRATO_ARCHIVOS.items():
        if c not in declarados and all((run_dir / f).is_file() for f in archivos):
            out += [{'ruta': f, 'contrato': c, 'existe': True} for f in archivos[-1:]]
    return out


def cumple_requiere(req, run, run_dir, catalogo=None):
    """(True, '') si la corrida cumple `requiere` del render; si no (False, motivo)."""
    (clave, val), = req.items()
    if clave == 'cualquiera':
        return True, ''
    if clave == 'kind':
        k = kind_of_run(run)
        validos = [val] if isinstance(val, str) else val
        ok = k in validos
        return (ok, '' if ok else f'el render pide kind {" o ".join(validos)} y la corrida es {k}')
    if clave == 'archivo':
        ok = (run_dir / val).is_file()
        return ok, '' if ok else f'la corrida no tiene {val}'
    arts = [a for a in artefactos_de(run, run_dir, catalogo) if a['contrato'] == val]
    ok = bool(arts) and all(a['existe'] for a in arts)
    return ok, '' if ok else f'la corrida no cumple el contrato {val}'


# ================================================================ media.json
def sha256_de(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for bloque in iter(lambda: f.read(1 << 20), b''):
            h.update(bloque)
    return h.hexdigest()


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) != float('inf')


def validar_media(media, job_id, render_id, run_id, salida):
    """Valida el `media.json` de un trabajo (APP 4) contra los archivos de `salida`. Devuelve
    `media`; lanza MediaError con el primer problema."""
    def falla(msg):
        raise MediaError(f'media.json: {msg}')

    if not isinstance(media, dict):
        falla('no es un objeto')
    if media.get('version') != 1:
        falla(f'version {media.get("version")!r}, se espera 1')
    for k, v in (('job_id', job_id), ('render', render_id), ('run_id', run_id)):
        if media.get(k) != v:
            falla(f'{k} debe ser {v!r} (trae {media.get(k)!r})')
    c = media.get('creado_utc')
    try:
        datetime.fromisoformat(c.replace('Z', '+00:00'))
    except (AttributeError, ValueError):
        falla('creado_utc debe ser una fecha ISO 8601')
    if 'duracion_pared_s' in media and not _num(media['duracion_pared_s']):
        falla('duracion_pared_s debe ser un numero')
    arch = media.get('archivos')
    if not isinstance(arch, list) or not arch:
        falla('archivos debe ser una lista no vacia')
    rutas, hay_video = set(), False
    for a in arch:
        if not isinstance(a, dict):
            falla('cada archivo debe ser un objeto')
        try:
            ruta = safe_relpath(a.get('ruta'), 'archivos.ruta')
        except ValueError as ex:
            raise MediaError(f'media.json: {ex}') from None
        if ruta in rutas:
            falla(f'archivo repetido: {ruta}')
        rutas.add(ruta)
        if a.get('tipo') not in TIPOS_MEDIO:
            falla(f'{ruta}: tipo debe ser uno de {TIPOS_MEDIO}')
        p = salida / ruta
        try:
            dentro = p.is_file() and p.resolve().relative_to(salida.resolve()) is not None
        except ValueError:
            dentro = False                       # un enlace simbolico que sale de la carpeta
        if not dentro:
            falla(f'{ruta}: el archivo no existe en la carpeta del trabajo')
        for k in ('ancho', 'alto'):
            if k in a and (isinstance(a[k], bool) or not isinstance(a[k], int) or a[k] <= 0):
                falla(f'{ruta}: {k} debe ser un entero positivo')
        if 'duracion_s' in a and (not _num(a['duracion_s']) or a['duracion_s'] < 0):
            falla(f'{ruta}: duracion_s debe ser un numero >= 0')
        if 'principal' in a and not isinstance(a['principal'], bool):
            falla(f'{ruta}: principal debe ser booleano')
        if 'sha256' in a:
            if not (isinstance(a['sha256'], str) and SHA_RE.match(a['sha256'])):
                falla(f'{ruta}: sha256 invalido')
            if sha256_de(p) != a['sha256']:
                falla(f'{ruta}: el sha256 no coincide con el archivo')
        hay_video = hay_video or a['tipo'] == 'video'
    if hay_video and MINIATURA not in rutas:
        falla(f'toda salida de video lleva miniatura ({MINIATURA}, 640x360)')
    if MINIATURA in rutas and next(a for a in arch if a['ruta'] == MINIATURA)['tipo'] != 'imagen':
        falla(f'{MINIATURA} debe ser de tipo imagen')
    pr = media.get('procedencia')
    if not isinstance(pr, dict):
        falla('falta procedencia')
    if not (isinstance(pr.get('commit'), str) and pr['commit']):
        falla('procedencia.commit debe ser un texto')
    if not isinstance(pr.get('sucio'), bool):
        falla('procedencia.sucio debe ser booleano')
    if not (isinstance(pr.get('comando'), list) and all(isinstance(x, str) for x in pr['comando'])):
        falla('procedencia.comando debe ser una lista de textos')
    if not isinstance(pr.get('args'), dict):
        falla('procedencia.args debe ser un objeto')
    if not (isinstance(pr.get('corrida'), dict) and pr['corrida'].get('run_id') == run_id):
        falla('procedencia.corrida.run_id debe ser el de la corrida')
    if 'notas' in media and not isinstance(media['notas'], str):
        falla('notas debe ser texto')
    return media


def archivo_principal(media):
    """El archivo principal de un media.json (el marcado, o el primero que no es miniatura)."""
    arch = media.get('archivos') or []
    for a in arch:
        if a.get('principal'):
            return a
    for a in arch:
        if a['ruta'] != MINIATURA:
            return a
    return arch[0] if arch else None


def url_medio(job_id, ruta):
    return f'/api/medios/{job_id}/{ruta}'


def miniatura_de(job_id, media):
    arch = media.get('archivos') or []
    if any(a['ruta'] == MINIATURA for a in arch):
        return url_medio(job_id, MINIATURA)
    img = next((a for a in arch if a.get('tipo') == 'imagen'), None)
    return url_medio(job_id, img['ruta']) if img else None


def ultimo_progreso(log_path, tail=65536):
    """Ultima linea `PROGRESO <0-1>` del log (None si no hay)."""
    try:
        size = log_path.stat().st_size
        with open(log_path, 'rb') as f:
            if size > tail:
                f.seek(size - tail)
            texto = f.read().decode('utf-8', errors='replace')
    except OSError:
        return None
    ultimo = None
    for ln in texto.splitlines():
        m = PROGRESO_RE.match(ln.strip())
        if m:
            ultimo = min(max(float(m.group(1)), 0.0), 1.0)
    return ultimo


def new_job_id():
    return 'r' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(3)


# ================================================================ galeria
def galeria(db, tipo=None, escena=None, limit=100):
    """Archivos principales de los trabajos `hecho` (mas nuevos primero)."""
    out = []
    for j in db.list_jobs(estado='hecho', limit=2000):
        media = j['media']
        if not media:
            continue
        run = db.get_run(j['run_id'])
        exp_id = run['experiment_id'] if run else None
        if escena and exp_id != escena:
            continue
        a = archivo_principal(media)
        if a is None or (tipo and a['tipo'] != tipo):
            continue
        out.append({
            'job_id': j['job_id'], 'run_id': j['run_id'], 'escena': exp_id, 'render': j['render_id'],
            'titulo': j['title'], 'tipo': a['tipo'], 'ruta': a['ruta'],
            'url': url_medio(j['job_id'], a['ruta']), 'miniatura': miniatura_de(j['job_id'], media),
            'ancho': a.get('ancho'), 'alto': a.get('alto'), 'duracion_s': a.get('duracion_s'),
            'sha256': a.get('sha256'), 'creado_utc': media.get('creado_utc'),
            'procedencia': media.get('procedencia'), 'notas': media.get('notas'),
            'archivos': [dict(x, url=url_medio(j['job_id'], x['ruta'])) for x in media['archivos']]})
        if len(out) >= limit:
            break
    return out


def miniatura_de_escena(db, exp_id):
    """Miniatura del medio mas reciente de una corrida de esa escena (None si no hay)."""
    for j in db.list_jobs(estado='hecho', limit=500):
        run = db.get_run(j['run_id'])
        if run and run['experiment_id'] == exp_id and j['media']:
            m = miniatura_de(j['job_id'], j['media'])
            if m:
                return m
    return None


# ================================================================ gestor de trabajos
class RenderManager(GovernedExecutor):
    def __init__(self, settings, db, runs_dir_of, governor=None):
        super().__init__(settings, governor or Governor(settings))
        self.db = db
        self.runs_dir_of = runs_dir_of           # run_id -> Path de la corrida
        self.host = {}                           # job_id -> entrada de un trabajo `ejecutor: host` en curso
        self.backoff = {}                        # job_id -> Task de la espera antes de reintentar (host)
        self.vigia = None

    async def start(self):
        # mark_interrupted ya lo hace el Manager; el vigia da por perdidos los trabajos del host
        # que dejan de latir
        self.vigia = asyncio.create_task(self._vigilar_host())

    async def stop(self):
        if self.vigia:
            self.vigia.cancel()
            await asyncio.gather(self.vigia, return_exceptions=True)
        for t in list(self.backoff.values()):
            t.cancel()
        for jid in list(self.host):
            self._liberar_host(jid)
        await self.detener()

    def salida(self, run_id, job_id):
        return self.runs_dir_of(run_id) / 'media' / job_id

    def submit(self, render, run, args):
        job_id = new_job_id()
        self.salida(run['run_id'], job_id).mkdir(parents=True)
        timeout = min(int(render.get('timeout_s', self.s.render_timeout)), self.s.render_timeout)
        self.db.create_job(job_id, run['run_id'], render['id'], render['title'], args,
                           render_spec(render), render['recursos'], timeout)
        if render.get('ejecutor', 'backend') != 'host':      # los del host los reclama el trabajador
            self.lanzar(job_id)
        return job_id

    async def cancel(self, job_id):
        j = self.db.get_job(job_id)
        if j is None or es_final(j['estado'], j['finished_at']):
            return False
        if (j['spec'] or {}).get('ejecutor') == 'host':
            return self._cancelar_host(job_id, j)
        return await self.cancelar(job_id)

    def delete(self, job_id):
        j = self.db.get_job(job_id)
        if j:
            d = self.salida(j['run_id'], job_id)
            if d.exists() and d.resolve().parent.name == 'media':
                shutil.rmtree(d, ignore_errors=True)
        self.db.delete_job(job_id)

    # --------------------------------------------------------- ganchos del ciclo
    def _get(self, job_id):
        return self.db.get_job(job_id)

    def _clase(self, j):
        return j['clase']

    def _quiere_pantalla(self, j):
        return bool(j['spec'].get('pantalla'))

    def _label(self, job_id):
        return f'render:{job_id}'

    def _estado_espera(self, job_id, motivo):
        j = self.db.get_job(job_id)
        if j is None or j['estado'] not in ('en_cola', 'esperando_carga'):
            return
        nuevo = 'esperando_carga' if motivo == 'carga' else 'en_cola'
        if nuevo != j['estado']:
            self.db.update_job(job_id, estado=nuevo)

    def _arranca(self, job_id, domain, pantalla):
        self.db.update_job(job_id, estado='corriendo', started_at=now_iso(), ros_domain_id=domain,
                           pantalla=pantalla)

    def _lanzamiento(self, job_id, j, domain, pantalla):
        d = self.salida(j['run_id'], job_id)
        ctx = {'run_id': j['run_id'], 'run_dir': self.runs_dir_of(j['run_id']), 'runs_dir': self.s.runs_dir,
               'salida': d, 'pantalla': pantalla or '', 'job_id': job_id, 'root': self.s.root,
               'domain': domain}
        spec = j['spec']
        argv = sustituir_marcadores(spec['comando'], ctx) + proceso_args(j['args'], spec['estilo_args'])
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), PYTHONUNBUFFERED='1',
                   LP_NUM_THREADS=str(self.s.lp_threads), RENDER_JOB_ID=job_id,
                   RENDER_ID=j['render_id'])
        if pantalla:
            env['DISPLAY'] = pantalla
        return argv, env, str(self.s.root), d / 'render.log'

    def _fallo(self, job_id, ex):
        self.db.update_job(job_id, estado='error', error=str(ex)[:500], finished_at=now_iso())

    def _marcar_cancelada(self, job_id):
        j = self.db.get_job(job_id)
        if j and not es_final(j['estado'], j['finished_at']):
            self.db.update_job(job_id, estado='cancelado', finished_at=now_iso())

    def _log(self, job_id, texto):
        j = self.db.get_job(job_id)
        try:
            with open(self.salida(j['run_id'], job_id) / 'render.log', 'ab') as f:
                f.write(f'[estacion] {texto}\n'.encode('utf-8'))
        except (OSError, TypeError):
            pass

    def _interrumpida(self, job_id, intento, espera, rc):
        nota = (f'interrumpido por senal externa (codigo {rc}); reintento {intento}/{self.s.reintentos}'
                f' en {espera:g} s')
        self.db.update_job(job_id, estado='interrumpida', exit_code=rc, error=nota, reintentos=intento)
        self._log(job_id, nota)

    def _reintenta(self, job_id, intento):
        j = self.db.get_job(job_id)
        d = self.salida(j['run_id'], job_id)
        for p in d.iterdir():
            if p.name == 'render.log':
                continue
            shutil.rmtree(p, ignore_errors=True) if p.is_dir() else p.unlink(missing_ok=True)
        self.db.update_job(job_id, estado='en_cola', started_at=None, error=None, exit_code=None,
                           pantalla=None, trabajador=None)
        self._log(job_id, f'reintento {intento}: de nuevo a la cola')

    def _leer_media(self, j, d):
        """(media validado, None) o (None, motivo)."""
        mp = d / 'media.json'
        if not mp.is_file():
            return None, 'el script termino sin escribir media.json'
        try:
            raw = json.loads(mp.read_text(encoding='utf-8'))
            return validar_media(raw, j['job_id'], j['render_id'], j['run_id'], d), None
        except (OSError, ValueError) as ex:      # MediaError es ValueError
            return None, str(ex)

    def _resultado_ok(self, job_id, rc):
        j = self.db.get_job(job_id)
        return rc == 0 and (self.salida(j['run_id'], job_id) / 'media.json').is_file()

    def _final(self, job_id, rc, senal=False):
        j = self.db.get_job(job_id)
        d = self.salida(j['run_id'], job_id)
        media, error, estado = None, None, 'hecho'
        if job_id in self.cancelled:
            estado = 'cancelado'
        elif job_id in self.timed_out:
            estado, error = 'error', f'excedio el limite de {j["timeout_s"]} s'
        elif senal:
            estado = 'interrumpida'
            error = (f'interrumpido por senal externa (codigo {rc}) y agoto los '
                     f'{self.s.reintentos} reintentos')
        elif rc != 0:
            estado, error = 'error', f'el script termino con codigo {rc}'
        else:
            media, error = self._leer_media(j, d)
            if media is None:
                estado = 'error'
        if estado == 'hecho':
            error = None
        self.db.update_job(job_id, estado=estado, exit_code=rc, error=error, media=media,
                           finished_at=now_iso())

    # ================================================= ejecutor: host (docs/APP.md 4.1)
    # El backend corre en el contenedor `station` (sin Docker ni pantalla). Los renders con
    # `ejecutor: host` los ejecuta `scripts/render/trabajador_host.py`, que reclama, late y termina por
    # la API. El gobernador sigue decidiendo: `reclamar` solo entrega un trabajo si lo admite.
    def _rel(self, p):
        """Ruta relativa a la raiz del repo (la misma en el host y en el contenedor: bind mount ./:/ws)."""
        try:
            return str(Path(p).relative_to(self.s.root))
        except ValueError:
            return str(p)

    def reclamar(self, trabajador, clases=None):
        """El primer trabajo del host (por antiguedad) que el gobernador admita ahora: pasa a
        `corriendo` y se devuelve ya resuelto. None si no hay nada admisible (cupo, carga, pantalla)."""
        for j in self.db.pending_host_jobs():
            if clases and j['clase'] not in clases:
                continue
            lease, motivo = self.gov.try_acquire(j['clase'], bool(j['spec'].get('pantalla')),
                                                 self._label(j['job_id']))
            if lease is None:
                nuevo = 'esperando_carga' if motivo == 'carga' else 'en_cola'
                if nuevo != j['estado']:
                    self.db.update_job(j['job_id'], estado=nuevo)
                continue
            return self._arrancar_host(j, lease, trabajador)
        return None

    def _arrancar_host(self, j, lease, trabajador):
        jid = j['job_id']
        try:
            domain = self.gov.alloc_domain()
        except Exception:
            self.gov.release(lease)
            raise
        d = self.salida(j['run_id'], jid)
        ctx = {'run_id': j['run_id'], 'run_dir': self._rel(self.runs_dir_of(j['run_id'])),
               'runs_dir': self._rel(self.s.runs_dir), 'salida': self._rel(d), 'pantalla': lease.pantalla or '',
               'job_id': jid, 'root': '.', 'domain': domain}
        spec = j['spec']
        argv = sustituir_marcadores(spec['comando'], ctx) + proceso_args(j['args'], spec['estilo_args'])
        env = {'ROS_DOMAIN_ID': str(domain), 'PYTHONUNBUFFERED': '1', 'LP_NUM_THREADS': str(self.s.lp_threads),
               'RENDER_JOB_ID': jid, 'RENDER_ID': j['render_id']}
        if lease.pantalla:
            env['DISPLAY'] = lease.pantalla
        now = time.monotonic()
        self.host[jid] = {'lease': lease, 'domain': domain, 'ultimo': now, 'inicio': now,
                          'cancelando': False, 'libera_en': None, 'timeout_s': j['timeout_s']}
        self.db.update_job(jid, estado='corriendo', started_at=now_iso(), ros_domain_id=domain,
                           pantalla=lease.pantalla, trabajador=trabajador)
        return {'job_id': jid, 'render': j['render_id'], 'run_id': j['run_id'], 'clase': j['clase'],
                'comando': argv, 'cwd': '.', 'salida': ctx['salida'], 'run_dir': ctx['run_dir'],
                'pantalla': lease.pantalla, 'entorno': env, 'timeout_s': j['timeout_s'],
                'latido_cada_s': round(self.s.latido_timeout_s / 6, 1), 'raiz': str(self.s.root)}

    def _liberar_host(self, jid):
        e = self.host.pop(jid, None)
        if e:
            self.gov.release(e['lease'])
            self.gov.free_domain(e['domain'])

    def latido(self, jid, progreso=None, lineas=None):
        """Registra un latido del trabajador (escribe su log y progreso). Devuelve True si debe
        cancelar el trabajo (lo cancelo el usuario, vencio, o el backend ya no lo da por suyo)."""
        e = self.host.get(jid)
        j = self.db.get_job(jid)
        if e is None or j is None:
            return True
        if e['cancelando']:
            if e['libera_en'] is None:       # ya se le avisa: se le da un margen para matar el grupo
                e['libera_en'] = time.monotonic() + self.s.kill_grace_s + 10
            return True
        if j['estado'] != 'corriendo':
            return True
        e['ultimo'] = time.monotonic()
        texto = ''.join(f'{ln}\n' for ln in (lineas or []))
        if progreso is not None:
            texto += f'PROGRESO {progreso:.3f}\n'
        if texto:
            try:
                with open(self.salida(j['run_id'], jid) / 'render.log', 'ab') as f:
                    f.write(texto.encode('utf-8', errors='replace'))
            except OSError:
                pass
        return False

    def _cancelar_host(self, jid, j):
        t = self.backoff.pop(jid, None)
        if t:
            t.cancel()
        self.db.update_job(jid, estado='cancelado', finished_at=now_iso())
        e = self.host.get(jid)
        if e:
            e['cancelando'] = True               # el siguiente latido recibe cancelar:true
        return True

    def _interrumpir_host(self, jid, rc, motivo):
        """El trabajo del host murio por una causa externa o dejo de latir: `interrumpida` y, mientras
        queden reintentos, de nuevo a la cola tras una espera creciente."""
        j = self.db.get_job(jid)
        n = j['reintentos']
        if n < self.s.reintentos:
            n += 1
            espera = min(self.s.reintento_espera_s * 2 ** (n - 1), ESPERA_MAX_S)
            nota = f'interrumpido ({motivo}); reintento {n}/{self.s.reintentos} en {espera:g} s'
            self.db.update_job(jid, estado='interrumpida', exit_code=rc, error=nota, reintentos=n)
            self._log(jid, nota)
            self.backoff[jid] = asyncio.create_task(self._espera_host(jid, n, espera))
        else:
            self.db.update_job(jid, estado='interrumpida', exit_code=rc, finished_at=now_iso(),
                               error=f'interrumpido ({motivo}) y agoto los {self.s.reintentos} reintentos')

    async def _espera_host(self, jid, n, espera):
        try:
            await asyncio.sleep(espera)
            self._reintenta(jid, n)
        except asyncio.CancelledError:
            pass
        finally:
            self.backoff.pop(jid, None)

    def fin_host(self, jid, rc, senal=False):
        """Primera parte de `fin` (en el bucle de eventos): libera el turno y decide. Devuelve
        `('listo', estado)` si ya esta resuelto (fin tardio o a reintento) o `('validar', es_senal)`:
        entonces hay que llamar a `cerrar_host` (lee y valida media.json: puede tardar)."""
        j = self.db.get_job(jid)
        self._liberar_host(jid)
        if es_final(j['estado'], j['finished_at']) or j['estado'] != 'corriendo':
            return 'listo', j['estado']          # cancelado, vencido o ya interrumpido: el fin llega tarde
        es_senal = (bool(senal) or muerte_por_senal(rc)) and not self._resultado_ok(jid, rc)
        if es_senal and j['reintentos'] < self.s.reintentos:
            self._interrumpir_host(jid, rc, f'senal externa, codigo {rc}')
            return 'listo', 'interrumpida'
        return 'validar', es_senal

    def cerrar_host(self, jid, rc, es_senal):
        self._final(jid, rc, es_senal)
        return self.db.get_job(jid)

    async def _vigilar_host(self):
        """Da por interrumpidos los trabajos del host que dejan de latir, por vencidos los que pasan
        de su timeout y libera el turno de los cancelados."""
        paso = max(0.05, min(1.0, self.s.latido_timeout_s / 4))
        while True:
            await asyncio.sleep(paso)
            ahora = time.monotonic()
            for jid, e in list(self.host.items()):
                try:
                    sin_latido = ahora - e['ultimo'] > self.s.latido_timeout_s
                    if e['cancelando']:
                        if sin_latido or (e['libera_en'] and ahora >= e['libera_en']):
                            self._liberar_host(jid)
                    elif sin_latido:
                        self._liberar_host(jid)
                        self._interrumpir_host(jid, None, f'sin latido en {self.s.latido_timeout_s:g} s')
                    elif ahora - e['inicio'] > e['timeout_s']:
                        self.db.update_job(jid, estado='error', finished_at=now_iso(),
                                           error=f'excedio el limite de {e["timeout_s"]} s')
                        e['cancelando'] = True
                except Exception:  # noqa: BLE001
                    log.exception('vigilancia del trabajo %s', jid)
