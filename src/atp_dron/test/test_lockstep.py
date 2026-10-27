"""D5: la cadena del dron en lockstep. Sin ROS (`atp_dron.lockstep`) y sobre un rclpy en memoria (`fake_rclpy`) que ejecuta los
nodos REALES y el `SimClock` real. El lockstep con DDS de verdad: `scripts/validar.sh dron` (necesita ROS 2)."""
import hashlib
import json
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fake_rclpy as F  # noqa: E402

from atp_dron import lazo, lockstep as L  # noqa: E402

ARCHIVOS = ('telemetry.jsonl', 'metrics.json', 'setpoints.jsonl')
_CERO_NEGATIVO = re.compile(rb'(?<![\d.])-0\.0(?![\d.eE])')


def _huella(ruta):
    return hashlib.sha256(_CERO_NEGATIVO.sub(b'0.0', open(ruta, 'rb').read())).hexdigest()[:16]


def correr_ros_falso(cfg, carpeta, semilla=0, descubrimiento=0, p_timer=0.3, desfase_datos=None):
    """sim_clock + dron_dinamica + dron_navegador sobre el bus en memoria. Devuelve el bus."""
    restaurar = F.instalar()
    try:
        bus = F.Bus.actual = F.Bus(semilla, descubrimiento, p_timer, desfase_datos)
        cfg = L.cargar_cfg(None, cfg)
        F.Node.parametros = {
            'sim_clock': {'time_scale': 0.0, 'step_s': L.PASO_S, 'ack_timeout_s': 3600.0},
            'dron_dinamica': {'cfg': json.dumps(cfg), 'salida': os.path.dirname(carpeta), 'id': os.path.basename(carpeta)},
            'dron_navegador': {'cfg': json.dumps(cfg)},
        }
        import atp_ros.sim_clock as sc
        sc.time = type('T', (), {'monotonic': staticmethod(bus.pared)})
        from atp_dron.nodo_dinamica import DronDinamica
        from atp_dron.nodo_navegador import DronNavegador
        reloj, planta, nav = sc.SimClock(), DronDinamica(), DronNavegador()
        assert bus.correr(max_it=600_000), 'la cadena no termino (se colgo o supero el limite de iteraciones)'
        bus.planta, bus.reloj = planta, reloj
        return bus
    finally:
        restaurar()
        F.Node.parametros = {}
        F.Bus.actual = None


CFG = {'ruta': [[6, 0, 4], [6, 4, 4]], 'duracion_s': 14.0, 'viento': [1.5, 0, 0], 'rafaga_sigma': 0.8, 'semilla': 3}


# ---------------------------------------------------------------------------------------------- la cadena sin ROS
def test_lazo_correr_es_la_cadena(tmp_path):
    a = lazo.correr(CFG, str(tmp_path / 'a'))
    b = L.correr_cadena(CFG, str(tmp_path / 'b'))
    assert a[0] == b[0] and a[1] == b[1]
    for f in ARCHIVOS:
        assert (tmp_path / 'a' / f).read_bytes() == (tmp_path / 'b' / f).read_bytes()


def test_el_tiempo_simulado_es_exacto_y_el_tick_son_cinco_pasos():
    assert L.PASO_S == pytest.approx(0.02)
    m, f, sp = L.correr_cadena({'ruta': [[0, 0, 5]], 'duracion_s': 3.0, 'parar_al_llegar': False})
    assert m['tiempo_s'] == pytest.approx(3.0, abs=1e-6) and len(sp) == 150           # 3 s / 0.02 s, sin perder pasos por meta
    assert [s['k'] for s in sp] == list(range(150))


def test_guiado_decide_solo_con_la_observacion_a_bordo_y_una_vez_por_sello():
    g = L.Guiado(L.cargar_cfg(None, {'ruta': [[10, 0, 5]]}))
    e = {'k': 0, 't': 0.0, 'obs_p': [0, 0, 0], 'obs_v': [0, 0, 0], 'x': 99.0, 'y': 99.0, 'z': 99.0}    # la verdad difiere: no se mira
    sp = g.decidir(e)
    assert sp['v'] == [0.0, 0.0, 2.0]                                   # en el suelo y la meta lejos: despega en vertical
    assert g.decidir(e) is None and g.decidir({**e, 'k': 0}) is None    # el mismo sello no se responde dos veces
    assert g.decidir({**e, 'k': 1})['k'] == 1


def test_guiado_termina_al_llegar_solo_si_se_pide_y_va_a_la_ultima_meta_despues():
    cfg = {'ruta': [[0, 0, 5]], 'radio_meta_m': 0.5}
    e = {'k': 0, 't': 1.0, 'obs_p': [0, 0, 5], 'obs_v': [0, 0, 0]}
    assert L.Guiado(L.cargar_cfg(None, cfg)).decidir(e)['guia']['terminado'] is True
    sp = L.Guiado(L.cargar_cfg(None, {**cfg, 'parar_al_llegar': False})).decidir(e)
    assert sp['guia']['terminado'] is False and sp['guia']['t_llegada'] == 1.0 and sp['guia']['dest'] is None


# --------------------------------------------------------------------------------------- los nodos ROS sobre el bus falso
@pytest.mark.parametrize('semilla, descub, desfase', [(0, 0, None), (1, 0, None), (2, 1500, None), (3, 300, {'/dron/estado': 1500, '/dron/setpoint': 900})],
                         ids=['orden0', 'orden1', 'emparejamiento_tardio', 'datos_tardios_se_pierde_el_estado_inicial'])
def test_ros_falso_da_los_mismos_bytes_que_sin_ros(semilla, descub, desfase, tmp_path):
    ros = tmp_path / 'ros'
    bus = correr_ros_falso(CFG, str(ros), semilla=semilla, descubrimiento=descub, desfase_datos=desfase)
    L.correr_cadena(CFG, str(tmp_path / 'sin'))
    for f in ARCHIVOS:
        assert (ros / f).read_bytes() == (tmp_path / 'sin' / f).read_bytes(), f
    assert bus.publicados['/atp/step_ack'] == len(open(ros / 'setpoints.jsonl').readlines())


def test_ros_falso_con_sensores_y_estimador(tmp_path):
    cfg = {**CFG, 'sensores': True, 'duracion_s': 10.0}
    correr_ros_falso(cfg, str(tmp_path / 'ros'), semilla=5)
    m, _, _ = L.correr_cadena(cfg, str(tmp_path / 'sin'))
    assert 'est_error_pos_rms_m' in m
    for f in ARCHIVOS:
        assert _huella(tmp_path / 'ros' / f) == _huella(tmp_path / 'sin' / f), f


def test_ros_falso_termina_al_llegar_y_escribe_resumen(tmp_path):
    cfg = {'ruta': [[2, 0, 3]], 'duracion_s': 30.0}
    bus = correr_ros_falso(cfg, str(tmp_path / 'ros'), semilla=9)
    met = json.load(open(tmp_path / 'ros' / 'metrics.json'))
    assert met['llego'] and met['tiempo_s'] < 30.0
    assert any(m.startswith('RESUMEN') for _, m in bus.planta._log.lineas)
    assert met == L.correr_cadena(cfg)[0]


def test_la_planta_ignora_un_setpoint_rancio_y_el_navegador_no_responde_dos_veces(tmp_path):
    restaurar = F.instalar()
    try:
        bus = F.Bus.actual = F.Bus(0, 0)
        cfg = L.cargar_cfg(None, {'ruta': [[3, 0, 3]], 'duracion_s': 2.0})
        F.Node.parametros = {'dron_dinamica': {'cfg': json.dumps(cfg), 'salida': str(tmp_path), 'id': 'x'},
                             'dron_navegador': {'cfg': json.dumps(cfg)}}
        from atp_dron.nodo_dinamica import DronDinamica
        from atp_dron.nodo_navegador import DronNavegador
        from fake_rclpy import Clock, String, Time
        planta, nav = DronDinamica().lock, DronNavegador().lock
        sp = lambda k: String(json.dumps({'k': k, 't': 0.0, 'v': [0, 0, 1.0], 'yaw': 0.0, 'guia': {
            'terminado': False, 'dest': [3, 0, 3], 'seg0': [0, 0, 0], 'i': 0, 'meta': 0, 'n': 1, 't_llegada': None}}))
        assert planta.pl.k == 0 and not planta.pend
        planta._sp(sp(0))                                          # sin estado pendiente: se ignora
        assert planta.pl.k == 0 and planta.pl.pasos == 0
        planta.on_clock(Clock(Time(0, int(L.PASO_S * 1e9))))      # /clock de k = 1: publica e_0 y queda pendiente
        assert planta.pend and planta.pl.k == 0
        planta._sp(sp(7))                                          # sello que no es el pendiente: rancio
        assert planta.pend and planta.pl.k == 0 and planta.pl.pasos == 0
        planta._sp(sp(0))                                          # el bueno: integra el tick y confirma
        assert not planta.pend and planta.pl.k == 1 and planta.pl.pasos == L.N_EXT
        assert bus.publicados['/atp/step_ack'] == 1
        # navegador: el mismo estado dos veces (latido del reloj) se responde una sola vez
        for _ in range(2):                                          # e_1 dos veces; e_0 ya estaba en cola desde on_clock
            bus.publicar('/dron/estado', String(planta.pl.estado_txt()))
        bus.correr(fin=lambda: False, max_it=bus.it + 300)
        assert bus.publicados['/dron/setpoint'] == 2                # 3 estados, 2 sellos distintos (0 y 1): uno por sello
        assert nav.gu.ultimo == 1
    finally:
        restaurar()
        F.Node.parametros = {}
        F.Bus.actual = None


def test_otro_nodo_puede_gobernar_la_planta_con_el_contrato_de_setpoint(tmp_path):
    """Un nodo propio (p. ej. teleoperado o de Gazebo) que publica /dron/setpoint en vez de dron_navegador."""
    restaurar = F.instalar()
    try:
        bus = F.Bus.actual = F.Bus(4, 0)
        cfg = L.cargar_cfg(None, {'ruta': [[0, 0, 5]], 'duracion_s': 4.0, 'parar_al_llegar': False})
        F.Node.parametros = {'dron_dinamica': {'cfg': json.dumps(cfg), 'salida': str(tmp_path), 'id': 'x'},
                             'sim_clock': {'time_scale': 0.0, 'step_s': L.PASO_S, 'ack_timeout_s': 3600.0}}
        import atp_ros.sim_clock as sc
        sc.time = type('T', (), {'monotonic': staticmethod(bus.pared)})
        from atp_dron.nodo_dinamica import DronDinamica
        from fake_rclpy import Node, String
        propio = Node('teleop')
        pub = propio.create_publisher(String, '/dron/setpoint')
        vistos = []

        def al_estado(m):
            e = json.loads(m.data)
            if e['k'] in vistos:
                return
            vistos.append(e['k'])
            guia = {'terminado': False, 'dest': None, 'seg0': [0, 0, 0], 'i': 1, 'meta': 0, 'n': 1, 't_llegada': None}
            pub.publish(String(json.dumps({'k': e['k'], 't': e['t'], 'v': [0.0, 0.0, 1.5], 'yaw': 0.0, 'guia': guia})))
        propio.create_subscription(String, '/dron/estado', al_estado)
        reloj, planta = sc.SimClock(), DronDinamica()
        assert bus.correr(max_it=600_000)
        z = json.loads(open(tmp_path / 'x' / 'telemetry.jsonl').readlines()[-1])['z']
        assert 4.0 < z < 6.5                                   # subio a ~1.5 m/s durante 4 s
    finally:
        restaurar()
        F.Node.parametros = {}
        F.Bus.actual = None


# ------------------------------------------------------------------------------------------------- el envoltorio con ROS
def _envoltorio():
    import importlib.util
    ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts', 'dron_lockstep.py')
    spec = importlib.util.spec_from_file_location('dron_lockstep_script', ruta)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_envoltorio_compara_carpetas_y_arma_el_launch(tmp_path):
    w = _envoltorio()
    L.correr_cadena(CFG, str(tmp_path / 'a'))
    L.correr_cadena(CFG, str(tmp_path / 'b'))
    eq = w.comparar(str(tmp_path / 'a'), str(tmp_path / 'b'))
    assert eq['identico'] and all(e['bytes_identicos'] for e in eq['archivos'].values()) and eq['error_max_posicion_m'] == 0.0
    L.correr_cadena({**CFG, 'semilla': 4}, str(tmp_path / 'c'))                 # otra semilla de rafagas: debe diferir
    eq = w.comparar(str(tmp_path / 'a'), str(tmp_path / 'c'))
    assert not eq['identico'] and eq['error_max_posicion_m'] > 0.0
    cmd = w.comando_launch(L.cargar_cfg(None, CFG), '/tmp/s', 'i', '0')
    assert cmd[:4] == ['ros2', 'launch', 'atp_dron', 'dron_lockstep.launch.py'] and json.loads(cmd[-1][len('cfg:='):])['semilla'] == 3


def test_el_guiado_lee_v_max_en_cada_tick_y_el_tope_por_autoridad_solo_lo_recorta():
    """Contrato que un guiado derivado usa (aire_keyhole.GuiadoPase cambia c['v_max'] por tramo). Lo rompio una integracion de dos
    paquetes: si se guarda v_max al construir, cambiarlo despues no hace nada."""
    e = {'k': 0, 't': 0.0, 'obs_p': [0, 0, 5.0], 'obs_v': [0, 0, 0]}
    g = L.Guiado(L.cargar_cfg(None, {'ruta': [[100, 0, 5]], 'v_max': 5.0}))
    g.c['v_max'] = 0.4
    assert float(sum(x * x for x in g.decidir(e)['v']) ** 0.5) == pytest.approx(0.4)
    g2 = L.Guiado(L.cargar_cfg(None, {'ruta': [[100, 0, 5]], 'v_max': 5.0, 'cuerpo': 'titan', 'autoridad_guiado': True}))
    v_con_tope = float(sum(x * x for x in g2.decidir(e)['v']) ** 0.5)
    g2.c['v_max'] = 0.2
    e2 = {**e, 'k': 1}
    assert float(sum(x * x for x in g2.decidir(e2)['v']) ** 0.5) == pytest.approx(0.2) and v_con_tope <= 5.0
