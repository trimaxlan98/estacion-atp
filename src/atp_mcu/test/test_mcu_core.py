"""Pruebas de la logica pura de atp_mcu (sin ROS)."""
import os
import re

from atp_mcu import mcu_core
from atp_mcu.mcu_core import Fusion, decodificar_diag, nombres_fallas

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def test_nombres_fallas():
    assert nombres_fallas(0) == []
    assert nombres_fallas(0x01) == ['ESTOP']
    assert nombres_fallas(0x01 | 0x08 | 0x40) == ['ESTOP', 'ATASCO', 'CFG']
    assert nombres_fallas(3.0) == ['ESTOP', 'LIMITE']      # el diag las manda como float32


def test_decodificar_diag():
    data = [0.0] * 16
    data[mcu_core.INDICE_DIAG['estado']] = 4.0
    data[mcu_core.INDICE_DIAG['fallas']] = 1.0
    data[mcu_core.INDICE_DIAG['t_lazo_max_us']] = 37.0
    d = decodificar_diag(data)
    assert d['estado_nombre'] == 'FALLA'
    assert d['fallas_nombres'] == ['ESTOP']
    assert d['t_lazo_max_us'] == 37.0
    assert decodificar_diag([0.0] * 3) is None


def test_indices_coinciden_con_el_firmware():
    """Los indices del diag y las mascaras de falla son los del C: leerlos del
    firmware en vez de copiarlos a mano evita que se desincronicen."""
    h = open(os.path.join(RAIZ, 'firmware/esp32_eje/uros/eje_uros.h')).read()
    enum = re.search(r'enum \{(.*?)\};', h, re.S).group(1)
    nombres = re.findall(r'DIAG_([A-Z_0-9]+)\s*(?:=\s*0)?\s*,', enum)
    py = sorted(mcu_core.INDICE_DIAG, key=mcu_core.INDICE_DIAG.get)
    assert py == [n.lower() for n in nombres]
    c = open(os.path.join(RAIZ, 'firmware/esp32_eje/core/eje_lazo.h')).read()
    mascaras = {n: int(v, 16) for n, v in re.findall(r'#define EJE_F_([A-Z]+)\s+(0x[0-9a-f]+)u', c)}
    assert {n: b for b, n in mcu_core.FALLAS} == mascaras
    estados = re.findall(r'EJE_([A-Z]+)\s*=\s*(\d)', c)
    assert tuple(n for n, v in sorted(estados, key=lambda x: int(x[1]))) == mcu_core.ESTADOS


def test_fusion_dos_ejes():
    f = Fusion(['az', 'el'], caduca_s=0.5)
    assert f.combinar(0.0) is None                    # aun no hay nada
    f.medida('az', 1.0, 0.1, 0.5, t=10.0)
    assert f.combinar(10.0) is None                   # falta el
    f.medida('el', 0.5, 0.0, -0.2, t=10.1)
    nombres, pos, vel, eff = f.combinar(10.2)
    assert nombres == ['az', 'el'] and pos == [1.0, 0.5] and vel == [0.1, 0.0] and eff == [0.5, -0.2]


def test_fusion_dato_caducado_no_se_rellena():
    f = Fusion(['az', 'el'], caduca_s=0.5)
    f.medida('az', 1.0, 0.0, 0.0, t=10.0)
    f.medida('el', 0.5, 0.0, 0.0, t=10.0)
    assert f.combinar(10.4) is not None
    assert f.combinar(10.6) is None                   # 0.6 s sin dato: no se publica una posicion congelada
    assert f.faltan(10.6) == ['az', 'el']
    f.medida('az', 1.1, 0.0, 0.0, t=10.6)
    assert f.faltan(10.6) == ['el']


def test_fusion_eje_simulado_usa_la_referencia():
    f = Fusion(['az', 'el'], simulados=['el'], caduca_s=0.5)
    f.medida('az', 2.0, 0.0, 0.0, t=1.0)
    assert f.combinar(1.0) is None                    # el simulado aun no tiene referencia
    f.referencia('el', 0.7, t=1.0)
    nombres, pos, vel, eff = f.combinar(1.0)
    assert pos == [2.0, 0.7]
    assert f.faltan(1.0) == []
    # el real sigue caducando; el simulado no
    assert f.combinar(2.0) is None and f.faltan(2.0) == ['az']
