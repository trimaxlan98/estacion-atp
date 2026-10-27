#!/usr/bin/env python3
"""Comprueba que el modelo SDF del cuadrirrotor (src/atp_dron/models/cuadrirrotor/model.sdf) sigue a los parametros de dron.py.

No carga el modelo en Gazebo (no hay gz aqui): verifica que el XML es valido, que la masa, las inercias y la posicion de los
rotores coinciden con PARAMS y con el mezclador de dron.py (rx, ry), y que cada junta apunta a un eslabon existente.
    python3 scripts/comprobar_modelo_dron.py   # sale con 0 si todo coincide
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, 'src', 'atp_dron'))
from atp_dron.dron import PARAMS   # noqa: E402

SDF = os.path.join(RAIZ, 'src', 'atp_dron', 'models', 'cuadrirrotor', 'model.sdf')


def comprobar(ruta=SDF):
    errores = []
    raiz = ET.parse(ruta).getroot()
    modelo = raiz.find('model')
    links = {l.get('name'): l for l in modelo.findall('link')}
    base = links.get('base_link')
    if base is None:
        return ['falta base_link']
    def num(el, tag):
        return float(el.find(tag).text)
    ine = base.find('inertial')
    if abs(num(ine, 'mass') - PARAMS['masa_kg']) > 1e-9:
        errores.append(f"masa {num(ine, 'mass')} != PARAMS {PARAMS['masa_kg']}")
    ix = ine.find('inertia')
    for tag, val in (('ixx', PARAMS['inercia'][0]), ('iyy', PARAMS['inercia'][1]), ('izz', PARAMS['inercia'][2])):
        if abs(num(ix, tag) - val) > 1e-9:
            errores.append(f'{tag} {num(ix, tag)} != PARAMS {val}')
    d = PARAMS['brazo_m'] / math.sqrt(2.0)
    esperados = {'rotor_0': (d, -d), 'rotor_1': (d, d), 'rotor_2': (-d, d), 'rotor_3': (-d, -d)}   # rx, ry de dron.py
    for nombre, (x, y) in esperados.items():
        l = links.get(nombre)
        if l is None:
            errores.append(f'falta {nombre}')
            continue
        px, py, _ = (float(v) for v in l.find('pose').text.split()[:3])
        if abs(px - x) > 1e-5 or abs(py - y) > 1e-5:
            errores.append(f'{nombre} en ({px:.5f}, {py:.5f}) != ({x:.5f}, {y:.5f})')
    juntas = modelo.findall('joint')
    if len(juntas) != 4:
        errores.append(f'{len(juntas)} juntas, se esperaban 4')
    for j in juntas:
        if j.find('child').text not in links or j.find('parent').text not in links:
            errores.append(f"junta {j.get('name')} apunta a un eslabon inexistente")
    return errores


if __name__ == '__main__':
    e = comprobar()
    print('MODELO OK: coincide con dron.py' if not e else 'MODELO NO COINCIDE:\n  ' + '\n  '.join(e))
    sys.exit(1 if e else 0)
