#!/usr/bin/env python3
"""Dron en LOCKSTEP ROS (D5; CONTRATO sec. 8 y 21, docs/DRON.md «D5»): lanza `dron_lockstep.launch.py`, deja telemetry.jsonl,
metrics.json y setpoints.jsonl en <salida>/<id>/ y, con --verificar si (defecto), repite la misma escena SIN ROS
(`atp_dron.lockstep.correr_cadena`) y escribe equivalencia.json con las huellas y, si difieren, el error maximo.

    python3 scripts/dron_lockstep.py --escena_yaml experiments/dron-lockstep-ruta.yaml --salida /tmp/dl --id a
    python3 scripts/dron_lockstep.py --ruta '[[10,0,5],[10,10,5]]' --duracion_s 30 --viento '[2,0,0]' --rafaga_sigma 1.0

Los argumentos son los de `atp_dron.lazo.DEFECTOS` (`--k v`, valores JSON) mas --salida, --id, --dominio (ROS_DOMAIN_ID),
--time_scale (0 = lo mas rapido) y --verificar (si|no). Dentro del contenedor (o con ROS cargado) llama a `ros2 launch`; en el
anfitrion sin ROS, lo hace con `docker run ros2-jazzy-dev`.
"""
import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys

RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_dron'))
from atp_dron import lazo, lockstep  # noqa: E402

CLAVES = set(lazo.DEFECTOS)
ARCHIVOS = ('telemetry.jsonl', 'metrics.json', 'setpoints.jsonl')
_CERO_NEGATIVO = re.compile(rb'(?<![\d.])-0\.0(?![\d.eE])')
_ENTORNO = 'source /opt/ros/jazzy/setup.bash; [ -f "$RAIZ/install/setup.bash" ] && source "$RAIZ/install/setup.bash"; exec "$@"'


def _valor(s):
    try:
        return json.loads(s)
    except ValueError:
        return s


def huella(ruta, normalizar=True):
    b = open(ruta, 'rb').read()
    if normalizar:
        b = _CERO_NEGATIVO.sub(b'0.0', b)
    return hashlib.sha256(b).hexdigest()[:16]


def comando_launch(cfg, salida, id_, time_scale):
    return ['ros2', 'launch', 'atp_dron', 'dron_lockstep.launch.py', f'time_scale:={time_scale}', f'salida:={salida}',
            f'id:={id_}', 'cfg:=' + json.dumps(cfg, separators=(',', ':'))]


def lanzar(cfg, salida, id_, dominio, time_scale):
    """Corre el launch; devuelve el codigo de salida. Con ROS en el PATH lo llama directo; si no, desde un shell que carga
    /opt/ros y install/; si no hay ROS en el equipo, dentro de un contenedor ros2-jazzy-dev."""
    cmd = comando_launch(cfg, salida, id_, time_scale)
    env = dict(os.environ, ROS_DOMAIN_ID=str(dominio), PYTHONUNBUFFERED='1', RAIZ=RAIZ)
    if shutil.which('ros2'):
        return subprocess.call(cmd, env=env, cwd=RAIZ)
    if os.path.exists('/opt/ros/jazzy/setup.bash'):
        return subprocess.call(['bash', '-c', _ENTORNO, 'bash'] + cmd, env=env, cwd=RAIZ)
    if shutil.which('docker'):
        os.makedirs(salida, exist_ok=True)
        d = ['docker', 'run', '--rm', '-i', '-e', f'ROS_DOMAIN_ID={dominio}', '-e', 'RAIZ=/ws', '-v', f'{RAIZ}:/ws', '-w', '/ws']
        if not os.path.abspath(salida).startswith(RAIZ + os.sep):
            d += ['-v', f'{os.path.abspath(salida)}:{os.path.abspath(salida)}']
        return subprocess.call(d + ['ros2-jazzy-dev', 'bash', '-c', _ENTORNO, 'bash'] + cmd)
    print('dron_lockstep: no hay ROS 2 ni Docker en este equipo', file=sys.stderr)
    return 127


def comparar(carpeta_ros, carpeta_sin):
    """equivalencia.json: huella de cada archivo (con el cero negativo normalizado, como la huella del rover), si los
    bytes crudos coinciden y, si hay diferencia, el error maximo de posicion y de yaw fila a fila."""
    r = {'archivos': {}, 'identico': True}
    for f in ARCHIVOS:
        a, b = os.path.join(carpeta_ros, f), os.path.join(carpeta_sin, f)
        if not (os.path.exists(a) and os.path.exists(b)):
            r['archivos'][f] = {'ros': os.path.exists(a), 'sin_ros': os.path.exists(b), 'identico': False}
            r['identico'] = False
            continue
        crudo = open(a, 'rb').read() == open(b, 'rb').read()
        e = {'huella_ros': huella(a), 'huella_sin_ros': huella(b), 'bytes_identicos': crudo,
             'identico': huella(a) == huella(b)}
        r['archivos'][f] = e
        r['identico'] &= e['identico']
    a, b = os.path.join(carpeta_ros, 'telemetry.jsonl'), os.path.join(carpeta_sin, 'telemetry.jsonl')
    if os.path.exists(a) and os.path.exists(b):
        fa = [json.loads(x) for x in open(a)]
        fb = [json.loads(x) for x in open(b)]
        r['filas_ros'], r['filas_sin_ros'] = len(fa), len(fb)
        dp = dr = 0.0
        for x, y in zip(fa, fb):
            dp = max(dp, math.sqrt((x['x'] - y['x']) ** 2 + (x['y'] - y['y']) ** 2 + (x['z'] - y['z']) ** 2))
            dr = max(dr, abs((x['yaw_deg'] - y['yaw_deg'] + 180.0) % 360.0 - 180.0))
        r['error_max_posicion_m'], r['error_max_yaw_deg'] = dp, dr
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(allow_abbrev=False, description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--escena_yaml', help='YAML de experiments/ cuyos `args` son los de partida')
    ap.add_argument('--salida', default='/tmp/dron_lockstep')
    ap.add_argument('--id', default='dl')
    ap.add_argument('--dominio', default=os.environ.get('ROS_DOMAIN_ID', '160'))
    ap.add_argument('--time_scale', default='0')
    ap.add_argument('--verificar', default='si', choices=['si', 'no'])
    a, resto = ap.parse_known_args(argv)
    extra = {}
    i = 0
    while i < len(resto):
        r = resto[i]
        i += 1
        if not r.startswith('--'):
            ap.error(f'argumento no reconocido: {r}')
        if '=' in r:
            k, v = r[2:].split('=', 1)
        elif i < len(resto):
            k, v = r[2:], resto[i]
            i += 1
        else:
            ap.error(f'falta el valor de {r}')
        if k not in CLAVES:
            ap.error(f'argumento desconocido: --{k} (hay: {", ".join(sorted(CLAVES))})')
        extra[k] = _valor(v)
    cfg = lockstep.cargar_cfg(a.escena_yaml, extra)
    dominio = int(a.dominio) if str(a.dominio).isdigit() else 160
    carpeta = os.path.join(a.salida, a.id)
    os.makedirs(a.salida, exist_ok=True)
    rc = lanzar(cfg, a.salida, a.id, dominio, a.time_scale)
    if rc != 0 or not os.path.exists(os.path.join(carpeta, 'metrics.json')):
        print(f'dron_lockstep: el launch termino con codigo {rc} y sin metrics.json', file=sys.stderr)
        return rc or 1
    if a.verificar == 'si':
        sin = os.path.join(carpeta, 'sin_ros')
        lockstep.correr_cadena(cfg, sin)
        eq = comparar(carpeta, sin)
        with open(os.path.join(carpeta, 'equivalencia.json'), 'w') as f:
            json.dump(eq, f, indent=2)
            f.write('\n')
        shutil.rmtree(sin, ignore_errors=True)
        print('EQUIVALENCIA con y sin ROS: ' + ('IDENTICA (telemetry, metrics y setpoints)' if eq['identico'] else
              f'DIFIERE, error maximo {eq.get("error_max_posicion_m")} m / {eq.get("error_max_yaw_deg")} deg'))
    met = json.load(open(os.path.join(carpeta, 'metrics.json')))
    print('RESUMEN ' + json.dumps(met))
    return 0


if __name__ == '__main__':
    sys.exit(main())
