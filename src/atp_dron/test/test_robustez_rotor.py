"""P8 (barrida de robustez de la curva en U): los criterios R1-R3 fijados antes de correr `scripts/estudio_rotor_robustez.py`."""
import importlib.util
import json
import os

_RUTA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts', 'estudio_rotor_robustez.py')
_spec = importlib.util.spec_from_file_location('estudio_rotor_robustez', _RUTA)
R = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(R)


def test_el_minimo_interior_sobrevive_a_k_mu_y_P0(tmp_path):
    salida = tmp_path / 'r.json'
    assert R.main(['--salida', str(salida)]) == 0
    crit = json.load(open(salida))['criterios']
    assert crit['R1_minimo_interior_en_todo_el_rango']
    assert crit['R2_V_minimo_en_5_12']
    assert crit['R3_control_negativo_sin_minimo']


def test_caso_base_reproduce_el_documento():
    import numpy as np
    t = R.potencia_total(4.65, 1.0)
    i = int(np.argmin(np.abs(R.VS - 8.5)))
    assert abs(t[i] - 114.98) < 0.05


def test_el_modelo_gazebo_del_cuadrirrotor_coincide_con_dron_py():
    """El SDF del cuadrirrotor (para Gazebo, no verificado en gz) tiene la masa, inercias y rotores de dron.py."""
    spec = importlib.util.spec_from_file_location('comprobar_modelo_dron', os.path.join(os.path.dirname(_RUTA), 'comprobar_modelo_dron.py'))
    C = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(C)
    assert C.comprobar() == []
