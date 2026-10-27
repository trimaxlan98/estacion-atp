"""Pruebas de la operacion del rover con retardo (R8): atp_rover.operacion. Rapidas (calibracion explicita: no corren la fisica de R1,
salvo `test_calibrar_fisica`)."""
import json

import pytest

from atp_rover import operacion as O

CAL = {'v_ratio': 0.85, 'p_mov_w': 45.0}
LIMPIO = {'planeta': 'luna', 'terreno': 'llano', 'calibracion': CAL, 'bache_por_km': 0.0, 'n_soles': 1, 'semilla': 0,
          'p_perdida_cmd': 0.0, 'p_perdida_estado': 0.0}


def corre(**kw):
    return O.correr_operacion({**LIMPIO, **kw})


def test_determinismo_byte_a_byte(tmp_path):
    cfg = {**LIMPIO, 'planeta': 'marte', 'terreno': 'rocoso', 'modo': 'gemelo', 'semilla': 2, 'bache_por_km': 3.0, 'p_perdida_cmd': 0.05}
    O.correr_operacion(cfg, str(tmp_path / 'a'))
    O.correr_operacion(cfg, str(tmp_path / 'b'))
    assert (tmp_path / 'a' / 'metrics.json').read_bytes() == (tmp_path / 'b' / 'metrics.json').read_bytes()
    assert json.loads((tmp_path / 'a' / 'metrics.json').read_text())['modo'] == 'gemelo'


def test_modo_y_enlace_desconocidos():
    with pytest.raises(ValueError):
        O.completar({'modo': 'volando'})
    with pytest.raises(ValueError):
        O.completar({'enlace': 'ondas'})
    with pytest.raises(KeyError):
        O.completar({'planeta': 'titan'})


def test_enlace_ideal_la_autonomia_recorre_lo_que_permite_la_velocidad():
    m = corre(modo='plan_sol', enlace='ideal')
    esperado = 0.5 * CAL['v_ratio'] * 2.0 * 3600.0
    assert abs(m['dist_util_m_sol'] - esperado) < 0.03 * esperado
    assert m['atascos'] == 0 and m['vuelcos'] == 0 and m['t_ocioso_s_sol'] < 5.0


def test_teleoperacion_se_degrada_con_el_retardo_y_la_autonomia_no():
    d = {ow: corre(modo='teleop', enlace='retardo', owl_s=ow)['dist_util_m_sol'] for ow in (1.3, 45.0, 450.0)}
    assert d[1.3] > d[45.0] > d[450.0] > 0
    assert d[450.0] < 0.1 * d[1.3]
    a = {ow: corre(modo='plan_sol', enlace='retardo', owl_s=ow)['dist_util_m_sol'] for ow in (1.3, 450.0)}
    assert abs(a[450.0] - a[1.3]) < 0.05 * a[1.3]


def test_teleop_mover_y_esperar_tiempo_ocioso_crece_con_el_retardo():
    a = corre(modo='teleop', enlace='retardo', owl_s=1.3)
    b = corre(modo='teleop', enlace='retardo', owl_s=150.0)
    assert b['t_ocioso_s_sol'] > a['t_ocioso_s_sol'] and b['frac_ocioso'] > 0.9


def test_comandos_caducan_si_la_ventana_los_retrasa():
    m = corre(modo='teleop', enlace='ventanas', owl_s=60.0, intervalo_ventanas_s=7200.0, dur_ventana_s=120.0, ttl_s=240.0)
    assert m['cmd_caducados'] > 0
    n = corre(modo='metas', enlace='ventanas', owl_s=60.0, intervalo_ventanas_s=7200.0, dur_ventana_s=120.0)
    assert n['cmd_caducados'] == 0


def test_perdidas_se_cuentan_y_son_reproducibles():
    a = corre(modo='metas', enlace='retardo', owl_s=5.0, p_perdida_cmd=0.5, p_perdida_estado=0.5)
    assert a['cmd_perdidos'] > 0 and a['tel_perdidas'] > 0
    assert a['cmd_perdidos'] <= a['cmd_enviados']
    assert a == corre(modo='metas', enlace='retardo', owl_s=5.0, p_perdida_cmd=0.5, p_perdida_estado=0.5)


def test_reaccion_local_es_mucho_mas_rapida_que_via_tierra():
    kw = dict(planeta='marte', enlace='retardo', owl_s=600.0, bache_por_km=15.0, p_recup=1.0)
    loc = corre(modo='plan_sol', **kw)
    sin = corre(modo='metas', **kw)
    assert loc['reaccion_local_n'] > 0 and loc['reaccion_local_s'] == pytest.approx(3.0, abs=0.01)
    assert sin['reaccion_tierra_n'] > 0 and sin['reaccion_tierra_s'] > 0.8 * 2 * 600.0     # del orden de la ida y vuelta de la luz (alguna orden de recuperacion en transito llega antes a otro atasco)
    assert loc['dist_util_m_sol'] > 5 * sin['dist_util_m_sol']


def test_alarma_local_evita_atascos_en_rocas():
    kw = dict(terreno='rocoso', planeta='luna', enlace='ideal', semilla=1, modo='plan_sol')
    con = corre(seguridad_local=True, **kw)
    sin = corre(seguridad_local=False, **kw)
    assert con['atascos'] <= sin['atascos'] and con['dist_util_m_sol'] >= sin['dist_util_m_sol']


def test_zonas_blandas_causan_atascos_sin_importar_el_modo():
    m = corre(modo='plan_sol', enlace='ideal', bache_por_km=15.0)
    assert m['atascos'] > 0 and m['recuperaciones_ok'] > 0


def test_el_gemelo_conoce_rocas_y_las_rodea():
    kw = dict(terreno='rocas', planeta='luna', enlace='retardo', semilla=1)
    g0 = corre(modo='gemelo', p_dem=0.0, **kw)
    g9 = corre(modo='gemelo', p_dem=0.9, **kw)
    assert g9['dist_util_m_sol'] >= g0['dist_util_m_sol'] and g9['atascos'] <= g0['atascos'] + 1


def test_enlace_sintetico_ventanas():
    import numpy as np
    cfg = O.completar({**LIMPIO, 'enlace': 'ventanas', 'owl_s': 100.0, 'intervalo_ventanas_s': 3600.0, 'dur_ventana_s': 600.0})
    e = O.EnlaceOp(cfg, 88775.0, np.random.default_rng(0))
    f = e.w_fase
    assert e._abre(f + 10.0) == f + 10.0                 # dentro de la ventana: inmediato
    assert e._abre(f + 700.0) == pytest.approx(f + 3600.0)   # fuera: la siguiente
    assert e._cmd_llega(f + 700.0 - 100.0 + 1.0) == pytest.approx(f + 3600.0)
    assert e._tel_llega(f + 700.0) == pytest.approx(f + 3600.0 + 100.0)


def test_enlace_real_marte_retardo_de_luz_y_latencia_de_comando():
    c = O.completar({**LIMPIO, 'planeta': 'marte', 'n_soles': 2})
    import numpy as np
    e = O.EnlaceOp(c, 88775.244, np.random.default_rng(0))
    assert 200.0 < e.owl < 1500.0                          # 4-25 min de una via en el modelo de R7
    assert e.lat_cmd > e.owl                               # espera de ventana por encima de la luz


def test_mundo_pendiente_y_tesela():
    c = O.completar({**LIMPIO, 'terreno': 'pendiente'})
    mu = O.Mundo(c)
    assert mu.pend(3.0, 4.0) == pytest.approx(15.0, abs=0.2) and not mu.roca(3.0, 4.0)
    assert mu.pend(250.0, -170.0) == pytest.approx(15.0, abs=0.2)          # fuera de la tesela: sigue definida
    c2 = O.completar({**LIMPIO, 'terreno': 'rocas', 'planeta': 'marte'})
    m2 = O.Mundo(c2)
    n = sum(m2.roca(x * 0.5, 0.0) for x in range(-100, 2000))
    assert 0 < n < 0.1 * 2100


def test_calibrar_fisica():
    c = O.calibrar('luna', 'llano', 0, duracion_s=14.0)
    assert 0.3 < c['v_ratio'] < 1.2 and 10.0 < c['p_mov_w'] < 200.0 and c['vuelcos'] == 0


def test_calibracion_del_archivo_si_existe():
    c, origen = O.calibracion_de({'planeta': 'marte', 'terreno': 'llano', 'semilla': 0, 'calibracion': None})
    assert origen in ('fisica', 'defecto') and 0.0 < c['v_ratio'] <= 1.3
