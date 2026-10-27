#!/usr/bin/env python3
"""Valida los catalogos `render/*.yaml` y las escenas `kind: proceso` de `experiments/*.yaml`
contra docs/APP.md (secciones 2 y 4). Sencillo y sin dependencias salvo PyYAML.

  python3 scripts/render/validar_catalogos.py              # ambos catalogos
  python3 scripts/render/validar_catalogos.py --media <carpeta o media.json> ...   # ademas, media.json

Sale con 1 si algo no cumple. Para las escenas `proceso` comprueba, ademas del formato, que el comando exista y que
cada argumento (fijo o de params_schema) lo acepte la herramienta que se lanza (`add_argument` del script o campo de
`Config` de cada escena); para los renders, que el envoltorio exista, sea ejecutable y conozca cada argumento.
Los `kind` anteriores (loop, campaign, plan, adcs, mision) los valida el backend actual y aqui solo se revisan por encima.
"""
import argparse
import glob
import os
import re
import sys

import yaml

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(RAIZ, 'scripts', 'render'))

ID_RE = re.compile(r'^[a-z0-9][a-z0-9-]{0,62}$')
ARG_RE = re.compile(r'^[a-z][a-z0-9_]{0,40}$')
TIPOS = ('number', 'integer', 'enum', 'bool')
AREAS = ('red', 'seguidor', 'control', 'orbita', 'rf', 'adcs', 'optico', 'mision', 'hardware')
TESIS = ('red', 'seguidor')
RECURSOS = ('ligero', 'pesado', 'gazebo', 'manim')
KINDS_VIEJOS = ('loop', 'campaign', 'plan', 'adcs', 'mision')
MARCAS_ESCENA = {'run_id', 'run_dir', 'runs_dir', 'domain', 'root'}
MARCAS_RENDER = {'run_dir', 'salida', 'pantalla', 'run_id', 'runs_dir', 'job_id', 'root', 'domain'}
REQUIERE = ('contrato', 'kind', 'archivo', 'cualquiera')
CONTRATOS_CONOCIDOS = {'red', 'red_eventos', 'topologia', 'red_escena', 'red_metricas', 'keyhole_tabla',
                       'mision_informe', 'mision_metricas', 'plan'}
MARCA_RE = re.compile(r'\{([a-z_]+)\}')


class Informe:
    def __init__(self):
        self.errores, self.avisos = [], []

    def e(self, donde, msg):
        self.errores.append(f'{donde}: {msg}')

    def a(self, donde, msg):
        self.avisos.append(f'{donde}: {msg}')


def schema(inf, donde, sch, args):
    if not isinstance(sch, dict):
        return inf.e(donde, 'params_schema debe ser un mapa')
    for k, s in sch.items():
        if not ARG_RE.match(str(k)):
            inf.e(donde, f'params_schema.{k}: clave invalida (^[a-z][a-z0-9_]{{0,40}}$)')
        if not isinstance(s, dict) or s.get('type') not in TIPOS:
            inf.e(donde, f'params_schema.{k}: type debe ser uno de {TIPOS}')
            continue
        if not s.get('label'):
            inf.a(donde, f'params_schema.{k}: sin label')
        if s['type'] == 'enum':
            if not s.get('options'):
                inf.e(donde, f'params_schema.{k}: enum sin options')
            elif k in args and args[k] not in s['options']:
                inf.e(donde, f'args.{k}={args[k]!r} no esta en las options {s["options"]}')
            lab = s.get('labels', {})
            if lab and not set(map(str, lab)) <= set(map(str, s.get('options', []))):
                inf.e(donde, f'params_schema.{k}: labels con claves que no son options')
        elif s['type'] in ('number', 'integer'):
            lo, hi = s.get('min'), s.get('max')
            if lo is not None and hi is not None and lo > hi:
                inf.e(donde, f'params_schema.{k}: min > max')
            v = args.get(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                if lo is not None and v < lo or hi is not None and v > hi:
                    inf.e(donde, f'args.{k}={v} fuera de [{lo}, {hi}]')
            if s['type'] == 'integer' and isinstance(v, float) and not v.is_integer():
                inf.e(donde, f'args.{k}={v} debe ser entero')
        elif s['type'] == 'bool' and k in args and not isinstance(args[k], bool):
            inf.e(donde, f'args.{k} debe ser booleano')


def opciones_de(ruta):
    """Opciones largas (--x) que declara un script Python."""
    try:
        txt = open(ruta, encoding='utf-8').read()
    except OSError:
        return set()
    ops = set(re.findall(r"add_argument\(\s*['\"](--[\w-]+)", txt))
    if 'config_campos' in txt:
        cfg = os.path.join(os.path.dirname(ruta), 'config.py')
        if os.path.isfile(cfg):
            ops |= {'--' + m for m in re.findall(r'^    (\w+): (?:str|int|float|bool) = ', open(cfg).read(), re.M)}
    return ops


def localizar_modulo(mod):
    ruta = mod.replace('.', '/') + '.py'
    for base in glob.glob(os.path.join(RAIZ, 'src', '*')):
        p = os.path.join(base, ruta)
        if os.path.isfile(p):
            return p
    return None


def escena_proceso(inf, ruta, d):
    n = os.path.basename(ruta)
    for k in ('id', 'title', 'level', 'kind', 'area', 'description', 'comando', 'estilo_args',
              'hecho_cuando', 'artefactos', 'recursos', 'timeout_s', 'args', 'params_schema'):
        if k not in d:
            inf.e(n, f'falta la clave `{k}`')
    if inf.errores and any(e.startswith(n + ': falta') for e in inf.errores):
        return
    if d['id'] != os.path.splitext(n)[0] or not ID_RE.match(str(d['id'])):
        inf.e(n, 'id invalido o distinto del nombre del archivo')
    if d['area'] not in AREAS:
        inf.e(n, f'area debe ser una de {AREAS}')
    desc = ' '.join(str(d['description']).split())
    for pal in ('Qué se mide', 'Qué se espera', 'Por qué importa', 'Validez'):
        if pal not in desc:
            inf.a(n, f'la descripcion no trae «{pal}»')
    if d['estilo_args'] not in ('cli', 'ros'):
        inf.e(n, 'estilo_args debe ser cli o ros')
    if d['recursos'] not in RECURSOS:
        inf.e(n, f'recursos debe ser uno de {RECURSOS}')
    if not isinstance(d['timeout_s'], int) or d['timeout_s'] <= 0:
        inf.e(n, 'timeout_s debe ser un entero positivo')
    cmd = d['comando']
    if not isinstance(cmd, list) or not cmd or not all(isinstance(x, str) for x in cmd):
        return inf.e(n, 'comando debe ser una lista de cadenas')
    marcas = set(MARCA_RE.findall(' '.join(cmd)))
    if not marcas <= MARCAS_ESCENA:
        inf.e(n, f'marcadores desconocidos en comando: {sorted(marcas - MARCAS_ESCENA)} (validos: {sorted(MARCAS_ESCENA)})')
    if not ({'run_id', 'run_dir'} & marcas):
        inf.e(n, 'el comando debe llevar {run_id} o {run_dir} para que la corrida caiga en su carpeta')
    # ¿existe la herramienta y acepta los argumentos?
    ops = set()
    if cmd[0].startswith('python') and len(cmd) > 2 and cmd[1] == '-m':
        p = localizar_modulo(cmd[2])
        if not p:
            inf.e(n, f'no encuentro el modulo {cmd[2]} bajo src/*/')
        else:
            ops = opciones_de(p)
    elif cmd[0].startswith('python') and len(cmd) > 1:
        p = os.path.join(RAIZ, cmd[1])
        if not os.path.isfile(p):
            inf.e(n, f'no existe {cmd[1]}')
        else:
            ops = opciones_de(p)
    if ops:
        for x in cmd:
            if x.startswith('--') and x not in ops:
                inf.e(n, f'el comando pasa {x}, que la herramienta no declara')
        sch, args = d['params_schema'], d['args']
        if d['estilo_args'] == 'cli':
            for k in list(args) + list(sch):
                if '--' + k not in ops and '--' + k.replace('_', '-') not in ops:
                    inf.e(n, f'el argumento `{k}` no lo acepta la herramienta (opciones: {sorted(ops)})')
                elif '--' + k not in ops:
                    inf.e(n, f'`{k}` se pasaria como --{k} (guiones bajos se respetan) pero la herramienta '
                             f'declara --{k.replace("_", "-")}: fijelo en `comando`')
    # artefactos
    arts = d['artefactos']
    if not isinstance(arts, list) or not arts:
        inf.e(n, 'artefactos debe ser una lista no vacia')
    else:
        rutas = []
        for i, a in enumerate(arts):
            if not isinstance(a, dict) or not a.get('ruta'):
                inf.e(n, f'artefactos[{i}] sin ruta')
                continue
            rutas.append(a['ruta'])
            if os.path.isabs(a['ruta']) or '..' in a['ruta'].split('/'):
                inf.e(n, f'artefactos[{i}].ruta debe ser relativa a la corrida')
            if a.get('contrato') and a['contrato'] not in CONTRATOS_CONOCIDOS:
                inf.a(n, f'contrato `{a["contrato"]}` no esta en la lista conocida {sorted(CONTRATOS_CONOCIDOS)}')
        if d['hecho_cuando'] not in rutas:
            inf.a(n, f'hecho_cuando ({d["hecho_cuando"]}) no esta entre los artefactos')
        if d.get('metricas') and d['metricas'] not in rutas:
            inf.a(n, f'metricas ({d["metricas"]}) no esta entre los artefactos')
    if d['area'] == 'red' and not any(a.get('contrato') == 'red' for a in arts if isinstance(a, dict)):
        inf.a(n, 'escena de red sin artefacto `contrato: red` (no habilita el visor)')
    schema(inf, n, d['params_schema'], d['args'])
    for k in d['args']:
        if not ARG_RE.match(str(k)):
            inf.e(n, f'args.{k}: clave invalida')


def catalogo_escenas(inf):
    ids = []
    for ruta in sorted(glob.glob(os.path.join(RAIZ, 'experiments', '*.yaml'))):
        n = os.path.basename(ruta)
        try:
            d = yaml.safe_load(open(ruta, encoding='utf-8'))
        except Exception as ex:
            inf.e(n, f'YAML ilegible: {ex}')
            continue
        if not isinstance(d, dict):
            inf.e(n, 'no es un mapa')
            continue
        ids.append(d.get('id'))
        if d.get('kind') == 'proceso':
            escena_proceso(inf, ruta, d)
        elif d.get('kind') in KINDS_VIEJOS:
            for k in ('id', 'title', 'level', 'description', 'launch', 'args', 'params_schema'):
                if k not in d:
                    inf.e(n, f'falta `{k}`')
            if isinstance(d.get('params_schema'), dict) and isinstance(d.get('args'), dict):
                schema(inf, n, d['params_schema'], d['args'])
        else:
            inf.e(n, f'kind desconocido: {d.get("kind")}')
    for i in set(ids):
        if ids.count(i) > 1:
            inf.e('experiments', f'id repetido: {i}')
    return len(ids)


def catalogo_renders(inf):
    ids = []
    for ruta in sorted(glob.glob(os.path.join(RAIZ, 'render', '*.yaml'))):
        n = 'render/' + os.path.basename(ruta)
        try:
            d = yaml.safe_load(open(ruta, encoding='utf-8'))
        except Exception as ex:
            inf.e(n, f'YAML ilegible: {ex}')
            continue
        for k in ('id', 'title', 'requiere', 'comando', 'estilo_args', 'recursos', 'pantalla',
                  'duracion_estimada_s', 'args', 'params_schema'):
            if k not in d:
                inf.e(n, f'falta la clave `{k}`')
        if any(e.startswith(n + ': falta') for e in inf.errores):
            continue
        ids.append(d['id'])
        if d['id'] != os.path.splitext(os.path.basename(ruta))[0] or not ID_RE.match(str(d['id'])):
            inf.e(n, 'id invalido o distinto del nombre del archivo')
        rq = d['requiere']
        if not isinstance(rq, dict) or len(rq) != 1 or next(iter(rq)) not in REQUIERE:
            inf.e(n, f'requiere debe ser un mapa con UNA clave de {REQUIERE}')
        if d['estilo_args'] not in ('cli', 'ros'):
            inf.e(n, 'estilo_args debe ser cli o ros')
        if d['recursos'] not in RECURSOS:
            inf.e(n, f'recursos debe ser uno de {RECURSOS}')
        if d.get('ejecutor', 'backend') not in ('host', 'backend'):
            inf.e(n, 'ejecutor debe ser host o backend')
        if d['recursos'] in ('gazebo', 'manim') and d.get('ejecutor') != 'host':
            inf.e(n, 'los renders de Gazebo y Manim necesitan Docker y pantalla: `ejecutor: host` (APP 4.1)')
        if not isinstance(d['pantalla'], bool):
            inf.e(n, 'pantalla debe ser booleano')
        if not isinstance(d['duracion_estimada_s'], (int, float)) or d['duracion_estimada_s'] <= 0:
            inf.e(n, 'duracion_estimada_s debe ser un numero positivo')
        cmd = d['comando']
        if not isinstance(cmd, list) or len(cmd) < 4 or not all(isinstance(x, str) for x in cmd):
            inf.e(n, 'comando debe ser una lista de cadenas: [bash, <script>, "{run_dir}", "{salida}" (, "{pantalla}")]')
            continue
        marcas = set(MARCA_RE.findall(' '.join(cmd)))
        if not marcas <= MARCAS_RENDER:
            inf.e(n, f'marcadores desconocidos: {sorted(marcas - MARCAS_RENDER)} (validos: {sorted(MARCAS_RENDER)})')
        if not {'run_dir', 'salida'} <= marcas:
            inf.e(n, 'el comando debe llevar {run_dir} y {salida}')
        if d['pantalla'] != ('pantalla' in marcas):
            inf.e(n, 'pantalla: true exige {pantalla} en el comando (y al reves)')
        if d['pantalla'] and d['recursos'] != 'gazebo':
            inf.a(n, 'pide pantalla pero no es de la clase gazebo')
        script = os.path.join(RAIZ, cmd[1])
        if cmd[0] != 'bash' or not os.path.isfile(script):
            inf.e(n, f'comando: se espera `bash scripts/render/<id>.sh` y que exista ({cmd[1]})')
            continue
        if not os.access(script, os.X_OK):
            inf.a(n, f'{cmd[1]} no es ejecutable')
        if os.path.basename(script) != d['id'] + '.sh':
            inf.a(n, f'el script ({os.path.basename(script)}) no se llama como el id ({d["id"]}.sh)')
        txt = open(script, encoding='utf-8').read()
        for k in list(d['args']) + list(d['params_schema']):
            if not re.search(r'\b' + re.escape(k) + r'\b', txt):
                inf.e(n, f'el argumento `{k}` no aparece en {cmd[1]}: el envoltorio no lo conoce')
        schema(inf, n, d['params_schema'], d['args'])
        if d['pantalla']:
            if not re.search(r'pantalla_tomar', txt):
                inf.e(n, f'{cmd[1]} no toma una pantalla del pool (pantalla_tomar)')
    for i in set(ids):
        if ids.count(i) > 1:
            inf.e('render', f'id repetido: {i}')
    return len(ids)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--media', nargs='*', default=[], help='carpetas o media.json a validar tambien')
    a = ap.parse_args(argv)
    inf = Informe()
    ne = catalogo_escenas(inf)
    nr = catalogo_renders(inf)
    if a.media:
        from media_json import validar
        for m in a.media:
            ruta = m if m.endswith('.json') else os.path.join(m, 'media.json')
            for e in validar(ruta):
                inf.e(ruta, e)
    for av in inf.avisos:
        print('AVISO ', av)
    for er in inf.errores:
        print('ERROR ', er)
    print(f'{ne} escenas en experiments/, {nr} renders en render/, {len(a.media)} media.json: '
          f'{len(inf.errores)} errores, {len(inf.avisos)} avisos')
    return 1 if inf.errores else 0


if __name__ == '__main__':
    sys.exit(main())
