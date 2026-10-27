"""Integracion: pase completo en tiempo real contra el rotor falso (6/3 deg/s).

Lento (~80 s por pase) y necesita ROS 2: solo corre con ATP_HW_INTEGRATION=1.

    ATP_HW_INTEGRATION=1 python3 -m pytest -q -s src/atp_hw/test/test_integration_hil.py

Lo que se comprueba es la fisica: un rotor de 6 deg/s NO puede seguir la
demanda de acimut de un pase de 85 deg de elevacion maxima (9 deg/s en la
culminacion, keyhole); uno que culmina a 60 deg si.
"""
import os
import re
import shutil
import subprocess

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ATP_HW_INTEGRATION') != '1' or shutil.which('ros2') is None,
    reason='integracion HIL: exige ATP_HW_INTEGRATION=1 y ROS 2 en el PATH')

PORT = '4655'    # puerto poco comun, lejos del 4533 estandar de Hamlib


def correr(*args, timeout=240):
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '53')
    cmd = ['ros2', 'launch', 'atp_hw', 'hil.launch.py', f'port:={PORT}', *args]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                       env=env)
    out = p.stdout + p.stderr
    print(out[-6000:])
    return out


def num(patron, texto):
    m = re.search(patron, texto)
    assert m, f'no aparece /{patron}/ en la salida'
    return float(m.group(1))


def lee(out):
    return {
        'az_max': num(r'RESULTADO.*?az: rms=[\d.]+ max=([\d.]+)', out),
        'pt_rms': num(r'apuntamiento: rms=([\d.]+)', out),
        'pt_max': num(r'apuntamiento: rms=[\d.]+ p95=[\d.]+ max=([\d.]+)', out),
        'dentro': num(r'dentro de [\d.]+ deg: ([\d.]+)%', out),
        'margen_min': num(r'ENLACE.*?margen min=(-?[\d.]+)', out),
        'cerrado': num(r'cerrado ([\d.]+)%', out),
        'flips': num(r'flips=(\d+)', out),
        'rechazadas': num(r'rechazadas=(\d+)', out),
        'ordenes': num(r'ordenes=(\d+)', out),
    }


def test_n2_keyhole_pierde_el_satelite_con_rotor_de_6_deg_s():
    out = correr('el_max:=85', 'mask:=70', 'band:=Ka')
    r = lee(out)
    print('N2 6/3 deg/s ->', r)
    assert 'TRACKING: rotor en AOS' in out
    assert 'EL RELOJ NO ES TIEMPO REAL' not in out
    # el acimut demandado llega a 9 deg/s: el rotor se queda atras decenas de grados
    assert r['az_max'] > 10.0
    # y el apuntamiento real supera el ancho de haz de la banda Ka (0.2 deg)
    assert r['pt_max'] > 1.0
    assert r['dentro'] < 50.0
    assert r['rechazadas'] == 0 and r['flips'] == 0
    assert r['ordenes'] > 100


def test_pase_bajo_el_keyhole_se_sigue_dentro_de_un_grado():
    out = correr('el_max:=60', 'mask:=50', 'band:=S')
    r = lee(out)
    print('control el_max=60 6/3 deg/s ->', r)
    assert r['pt_max'] < 1.0
    assert r['cerrado'] == 100.0
    assert r['rechazadas'] == 0 and r['flips'] == 0
