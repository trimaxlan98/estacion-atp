"""Catalogo de experimentos (experiments/*.yaml, CONTRATO 5) y validacion de args.

El usuario solo puede dar claves declaradas en `params_schema`; cada valor se
valida contra su tipo, rango u opciones. Los demas args del YAML son fijos.

`kind: proceso` (docs/APP.md 2): una escena nueva es solo un YAML con el comando a ejecutar.
Su validacion es estricta (comando como argv sin shell, marcadores conocidos, rutas relativas
confinadas a la carpeta de la corrida, area/clase del gobernador declaradas).
"""
import json
import logging
import math
import re

import yaml

from station.config import ARG_KEY_RE, CLASE_POR_KIND, CLASES, EXPERIMENT_ID_RE

log = logging.getLogger('station.experiments')

LAUNCH_RE = re.compile(r'^[a-z0-9_]+\.launch\.py$')
KINDS = ('loop', 'campaign', 'plan', 'adcs', 'mision', 'proceso')
# `kind: plan` no es un lanzamiento ROS: `launch` es el script (scripts/planificar.py);
# `kind: adcs` lanza `ros2 launch sat_adcs adcs.launch.py` (el resto, atp_ros)
PLAN_SCRIPT = 'planificar.py'
# `kind: mision` tampoco es un lanzamiento: scripts/mision.py planifica y lanza UNA corrida ROS por
# contacto (dominios 200-229), y deja informe.json (CONTRATO 4.2)
MISION_SCRIPT = 'mision.py'
ADCS_LAUNCH = 'adcs.launch.py'
PROCESO_LAUNCH = 'proceso'      # valor de `launch` que se guarda para un `kind: proceso`
TYPES = ('number', 'integer', 'enum', 'bool')

AREAS = ('red', 'seguidor', 'control', 'orbita', 'rf', 'adcs', 'optico', 'mision',
         'hardware', 'plataforma')
ESTILOS_ARGS = ('cli', 'ros')
MARCADORES_PROCESO = ('run_id', 'run_dir', 'runs_dir', 'domain', 'root')
MARCADOR_RE = re.compile(r'\{([A-Za-z_][A-Za-z0-9_]*)\}')
CONTRATO_RE = re.compile(r'^[a-z][a-z0-9_]{0,40}$')


class ArgError(ValueError):
    """Argumento de usuario invalido (el API lo devuelve como 422)."""


# ---------------------------------------------------------------- validacion del YAML
def safe_relpath(v, what):
    """Ruta relativa dentro de la carpeta de una corrida: sin `..`, sin absolutas, sin NUL."""
    if (not isinstance(v, str) or not v or len(v) > 200 or v.startswith('/') or '\\' in v
            or '\0' in v or any(p in ('', '.', '..') for p in v.split('/'))):
        raise ValueError(f'{what}: ruta relativa invalida ({v!r})')
    return v


def check_comando(d, permitidos, what='comando'):
    """`comando` = argv (lista de textos, sin shell) cuyos marcadores `{x}` estan en `permitidos`."""
    c = d.get('comando')
    if (not isinstance(c, list) or not c
            or not all(isinstance(a, str) and a and '\0' not in a for a in c)):
        raise ValueError(f'{what} debe ser una lista no vacia de textos (argv, sin shell)')
    for a in c:
        for m in MARCADOR_RE.findall(a):
            if m not in permitidos:
                raise ValueError(f'{what}: marcador desconocido {{{m}}} (validos: '
                                 f'{", ".join("{" + x + "}" for x in permitidos)})')


def check_estilo(d):
    if d.get('estilo_args', 'cli') not in ESTILOS_ARGS:
        raise ValueError(f'estilo_args debe ser uno de {ESTILOS_ARGS}')


def check_recursos(d):
    if 'recursos' in d and d['recursos'] not in CLASES:
        raise ValueError(f'recursos debe ser uno de {CLASES}')
    if 'timeout_s' in d and (isinstance(d['timeout_s'], bool) or not isinstance(d['timeout_s'], int)
                             or d['timeout_s'] < 1):
        raise ValueError('timeout_s debe ser un entero positivo')


def check_metadatos(d, exigir):
    """area / artefactos (APP 2). En `proceso` el area es obligatoria."""
    if (exigir or 'area' in d) and d.get('area') not in AREAS:
        raise ValueError(f'area debe ser una de {AREAS}')
    if exigir or 'artefactos' in d:
        a = d.get('artefactos', [])
        if not isinstance(a, list):
            raise ValueError('artefactos debe ser una lista')
        for it in a:
            if not isinstance(it, dict) or set(it) - {'ruta', 'contrato'} or 'ruta' not in it:
                raise ValueError('artefactos: cada elemento es {ruta, contrato?}')
            safe_relpath(it['ruta'], 'artefactos.ruta')
            if 'contrato' in it and not (isinstance(it['contrato'], str)
                                         and CONTRATO_RE.match(it['contrato'])):
                raise ValueError('artefactos.contrato invalido')


def check_args_y_esquema(d):
    if not isinstance(d['args'], dict) or not isinstance(d['params_schema'], dict):
        raise ValueError('args y params_schema deben ser mapas')
    for k in list(d['args']) + list(d['params_schema']):
        if not ARG_KEY_RE.match(str(k)):
            raise ValueError(f'clave de argumento invalida: {k}')
    for k, s in d['params_schema'].items():
        if not isinstance(s, dict) or s.get('type') not in TYPES:
            raise ValueError(f'params_schema.{k}: type debe ser uno de {TYPES}')
        if s['type'] == 'enum' and not s.get('options'):
            raise ValueError(f'params_schema.{k}: enum sin options')


def _check_proceso(d):
    check_comando(d, MARCADORES_PROCESO)
    check_estilo(d)
    if 'hecho_cuando' not in d:
        raise ValueError('kind proceso: falta hecho_cuando (el archivo que marca el fin correcto)')
    safe_relpath(d['hecho_cuando'], 'hecho_cuando')
    if d.get('metricas') is not None:
        safe_relpath(d['metricas'], 'metricas')


def _check_experiment(d, stem):
    if not isinstance(d, dict):
        raise ValueError('el YAML no es un mapa')
    if d.get('kind') not in KINDS:
        raise ValueError(f'kind debe ser uno de {KINDS}')
    proceso = d['kind'] == 'proceso'
    for k in ('id', 'title', 'level', 'kind', 'description', 'args', 'params_schema') + (
            () if proceso else ('launch',)):
        if k not in d:
            raise ValueError(f'falta la clave {k}')
    if not EXPERIMENT_ID_RE.match(str(d['id'])) or d['id'] != stem:
        raise ValueError('id invalido o distinto del nombre del archivo')
    if proceso:
        if d.setdefault('launch', PROCESO_LAUNCH) != PROCESO_LAUNCH:
            raise ValueError('kind proceso: no lleva launch (ejecuta su comando)')
    elif d['kind'] == 'plan':
        if d['launch'] != PLAN_SCRIPT:
            raise ValueError(f'kind plan: launch debe ser {PLAN_SCRIPT}')
    elif d['kind'] == 'mision':
        if d['launch'] != MISION_SCRIPT:
            raise ValueError(f'kind mision: launch debe ser {MISION_SCRIPT}')
    elif not LAUNCH_RE.match(str(d['launch'])):
        raise ValueError('launch invalido')
    elif (d['kind'] == 'adcs') != (d['launch'] == ADCS_LAUNCH):
        raise ValueError(f'kind adcs y launch {ADCS_LAUNCH} van juntos')
    check_args_y_esquema(d)
    check_recursos(d)
    check_metadatos(d, exigir=proceso)
    if proceso:
        _check_proceso(d)


def load_catalog(directory):
    """Lee todos los YAML validos, ordenados por id. Los invalidos se omiten
    con un aviso en el log (un YAML roto no tumba la Estacion)."""
    out = {}
    for p in sorted(directory.glob('*.yaml')):
        try:
            d = yaml.safe_load(p.read_text(encoding='utf-8'))
            _check_experiment(d, p.stem)
            d['description'] = ' '.join(str(d['description']).split())
            out[d['id']] = d
        except Exception as ex:  # noqa: BLE001
            log.warning('experimento %s ignorado: %s', p.name, ex)
    return out


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _check_value(key, spec, v):
    t = spec['type']
    if t == 'enum':
        if isinstance(v, (bool, list, dict)) or v not in spec['options']:
            raise ArgError(f'{key}: debe ser uno de {spec["options"]}')
        return v
    if t == 'bool':
        if not isinstance(v, bool):
            raise ArgError(f'{key}: debe ser booleano')
        return v
    if t == 'integer':
        if isinstance(v, float) and v.is_integer():
            v = int(v)
        if not isinstance(v, int) or isinstance(v, bool):
            raise ArgError(f'{key}: debe ser entero')
    elif not _num(v):
        raise ArgError(f'{key}: debe ser un numero')
    if 'min' in spec and v < spec['min']:
        raise ArgError(f'{key}: minimo {spec["min"]}')
    if 'max' in spec and v > spec['max']:
        raise ArgError(f'{key}: maximo {spec["max"]}')
    return v


def validate_args(exp, user_args):
    """Devuelve args = defaults del YAML + args del usuario ya validados."""
    if user_args is None:
        user_args = {}
    if not isinstance(user_args, dict):
        raise ArgError('args debe ser un objeto')
    schema = exp['params_schema']
    out = dict(exp['args'])
    for k, v in user_args.items():
        if k not in schema:
            raise ArgError(f'argumento no permitido: {k}')
        out[k] = _check_value(k, schema[k], v)
    return out


def kind_of(launch):
    """kind de una corrida a partir de su `launch` (los kinds de la ronda 3). Una corrida
    `proceso` guarda `launch: proceso`; en una fila de la base usar `kind_of_run`."""
    if launch == PROCESO_LAUNCH:
        return 'proceso'
    if launch == PLAN_SCRIPT:
        return 'plan'
    if launch == MISION_SCRIPT:
        return 'mision'
    if launch == ADCS_LAUNCH:
        return 'adcs'
    return 'campaign' if launch.startswith('campaign') else 'loop'


def kind_of_run(run):
    """kind de la fila de una corrida (el `kind` guardado, o el de su `launch` en las antiguas)."""
    return run.get('kind') or kind_of(run['launch'])


def clase_de(exp):
    """Clase del gobernador de un experimento: la de `recursos` o la de su kind (APP 3)."""
    return exp.get('recursos') or CLASE_POR_KIND[exp['kind']]


def result_file(launch):
    """Archivo cuya existencia dice que la corrida produjo su resultado (CONTRATO 1)."""
    return {'plan': 'plan.json', 'campaign': 'campaign.json',
            'mision': 'informe.json'}.get(kind_of(launch), 'metrics.json')


def cli_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return str(v)


def cli_args(args):
    """args -> ['k:=v', ...] para `ros2 launch` (sin shell: no hay inyeccion;
    las claves ya pasaron ARG_KEY_RE y los valores son numeros/enums)."""
    return [f'{k}:={cli_value(v)}' for k, v in args.items()]


def script_args(args):
    """args -> ['--clave=valor', ...] para un script (`kind: plan` o `mision`): `a_b` -> `--a-b`.
    Un valor vacio omite el argumento (el script lo toma como "ninguno"); la forma
    `--k=v` evita que un valor se lea como otra opcion. Sin shell: no hay inyeccion."""
    return [f'--{k.replace("_", "-")}={cli_value(v)}' for k, v in args.items() if v != '']


# ---------------------------------------------------------------- kind: proceso
def proceso_spec(exp):
    """Lo que la corrida guarda de un `kind: proceso`. Se congela al lanzar: editar el YAML
    despues no cambia una corrida que sigue en cola."""
    return {'comando': list(exp['comando']), 'estilo_args': exp.get('estilo_args', 'cli'),
            'hecho_cuando': exp['hecho_cuando'], 'metricas': exp.get('metricas'),
            'artefactos': list(exp.get('artefactos') or [])}


def sustituir_marcadores(comando, ctx):
    """Reemplaza `{marcador}` en cada elemento del argv (sin shell y sin str.format)."""
    return [MARCADOR_RE.sub(lambda m: str(ctx[m.group(1)]), a) for a in comando]


def proceso_args(args, estilo):
    """args -> argv segun `estilo_args`: `cli` = `--clave valor` (los guiones bajos se respetan; un
    valor que empiece por `-` va como `--clave=valor`), `ros` = `clave:=valor`. Un valor vacio
    omite el argumento. Sin shell: no hay inyeccion (claves por regex, valores numeros/enums)."""
    out = []
    for k, v in args.items():
        if v == '':
            continue
        # listas y diccionarios van como JSON: str() de un dict usa comillas simples (`{'altura': 0.18}`), el script no lo
        # entendia y recibia un texto (rover-luna-escalon, marte-rocas, tierra-lateral y titan-pendiente fallaban). Una lista
        # numerica sale igual que con str(), asi que las escenas que ya funcionaban no cambian.
        t = json.dumps(v) if isinstance(v, (dict, list)) else cli_value(v)
        if estilo == 'ros':
            out.append(f'{k}:={t}')
        elif t.startswith('-'):
            out.append(f'--{k}={t}')
        else:
            out += [f'--{k}', t]
    return out


def proceso_command(spec, args, ctx):
    """argv de una corrida `kind: proceso`: el `comando` con los marcadores resueltos y los args
    al final. `ctx` = {run_id, run_dir, runs_dir, domain, root}."""
    return sustituir_marcadores(spec['comando'], ctx) + proceso_args(args, spec['estilo_args'])
