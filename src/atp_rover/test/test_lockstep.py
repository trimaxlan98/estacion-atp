"""R9: la cadena del rover en lockstep, sin ROS (`atp_rover.lockstep`). El lockstep ROS con DDS: `scripts/validar.sh rover`."""
import hashlib
import json
import math
import re

import pytest

from atp_rover import lazo, lockstep as L

_CERO_NEGATIVO = re.compile(rb'(?<![\d.])-0\.0(?![\d.eE])')


def _huella(ruta):
    """sha256 con el cero negativo escrito `0.0` (ver test_rover_bogie._huella: es el unico byte que cambia segun la ruta SIMD)."""
    return hashlib.sha256(_CERO_NEGATIVO.sub(b'0.0', ruta.read_bytes())).hexdigest()[:16]


# ---------------------------------------------------------------------------------------------------------- equivalencia con lazo
@pytest.mark.parametrize('cfg', [
    {'planeta': 'luna', 'terreno': 'rocoso', 'semilla': 7, 'ruta': [[3, 0], [3, 2]], 'duracion_s': 7, 'navegacion': 'basico'},
    {'planeta': 'marte', 'terreno': 'dunas', 'semilla': 3, 'ruta': [[3, 0]], 'duracion_s': 7, 'navegacion': 'completa'},
    {'planeta': 'luna', 'terreno': 'llano', 'duracion_s': 3.05, 'v_max': 0.4},          # sin ruta y duracion que no es multiplo de 0.1 s
], ids=['basico', 'completa', 'sin_ruta_parcial'])
def test_cadena_sin_enlace_da_los_mismos_bytes_que_lazo(cfg, tmp_path):
    lazo.correr(cfg, str(tmp_path / 'a'))
    L.correr_cadena(cfg, str(tmp_path / 'b'))
    for f in ('telemetry.jsonl', 'metrics.json'):
        assert (tmp_path / 'a' / f).read_bytes() == (tmp_path / 'b' / f).read_bytes(), f
        assert _huella(tmp_path / 'a' / f) == _huella(tmp_path / 'b' / f)


def test_cadena_corta_al_alcanzar_la_meta_como_lazo(tmp_path):
    cfg = {'planeta': 'luna', 'terreno': 'llano', 'ruta': [[0.6, 0]], 'duracion_s': 30, 'v_max': 0.5, 'radio_meta_m': 0.5}
    met_a, filas_a = lazo.correr(cfg)
    met_b, filas_b, _ = L.correr_cadena(cfg)
    assert met_a['meta_alcanzada'] and met_a == met_b and filas_a == filas_b
    assert len(filas_b) < 300


# ------------------------------------------------------------------------------------------------------------------ fichas
def test_buzon_empareja_por_ficha_aunque_el_orden_entre_topicos_cambie():
    b = L.Buzon()
    b.ficha({'t': 1.0, 'n': 2})                    # la ficha llega antes que sus mensajes
    assert b.sacar() is None
    b.meter((1, 0))
    assert b.sacar() is None
    b.meter((2, 0))
    f, it = b.sacar()
    assert f['t'] == 1.0 and it == [(1, 0), (2, 0)]
    b.meter((9, 9))
    b.ficha({'t': 1.0, 'n': 0})                    # ficha duplicada (mismo sello): se descarta, el mensaje queda para la siguiente
    assert b.sacar() is None
    b.ficha({'t': 1.1, 'n': 1})
    assert b.sacar()[1] == [(9, 9)]


# ------------------------------------------------------------------------------------------------------------------ enlace
def _ticks(enl, t_ini, t_fin, enviar=(), paso=0.1):
    """Maneja EnlaceLock como lo hace la cadena: un tick por instante; `enviar` {t: [(v, w)]} = ordenes del operador en ese t.
    Devuelve [(t_tick, [ordenes entregadas], [estados_tierra])]."""
    out, prev = [], []
    k = 0
    while True:
        t = round(t_ini + k * paso, 6)
        if t > t_fin + 1e-9:
            return out
        cmds, ests, _, _ = enl.tick(t, json.dumps({'t': t}), prev)
        out.append((t, cmds, [json.loads(e) for e in ests]))
        prev = enviar.get(t, [])
        k += 1


def test_retardo_de_luz_luna_en_tiempo_simulado_cuantizado_al_paso():
    cfg = L.cargar_cfg(None, {'planeta': 'luna', 'enlace': 'retardo'})
    enl = L.EnlaceLock(cfg)
    owl = enl.enlace._owlt(0.0)
    assert 1.2 < owl < 1.4                                   # Luna: ~1.28 s de una via
    out = _ticks(enl, 0.0, 6.0, {1.0: [(0.3, 0.1)]})
    llegadas = [(t, c) for t, c, _ in out if c]
    assert len(llegadas) == 1
    t_ll, c = llegadas[0]
    assert c == [(0.3, 0.1)]
    assert owl <= t_ll - 1.0 < owl + 0.1 + 1e-9              # primer tick con t >= t_envio + retardo
    # la telemetria vuelve: el estado de t llega a Tierra en el primer tick >= t + owl, con t_tierra = su instante de entrega
    for t, _, ests in out:
        for d in ests:
            assert d['t_tierra'] == pytest.approx(d['t'] + owl, abs=1e-3) and t >= d['t'] + owl - 1e-9 and t - d['t'] < owl + 0.1 + 1e-9


def test_modo_ideal_entrega_un_paso_despues_no_en_el_mismo_tick():
    enl = L.EnlaceLock(L.cargar_cfg(None, {'planeta': 'luna', 'enlace': 'ideal'}))
    out = _ticks(enl, 0.0, 0.5, {0.2: [(0.1, 0.0)]})
    assert [(t, c) for t, c, _ in out if c] == [(0.3, [(0.1, 0.0)])]


def test_ventanas_marte_la_orden_espera_a_los_contactos_del_rele():
    t0 = 117600.0 - 1.0                                       # la escena arranca 1 s antes de un instante cuya entrega tarda ~45 min
    enl = L.EnlaceLock(L.cargar_cfg(None, {'planeta': 'marte', 'enlace': 'ventanas', 't0_s': t0}))
    owl = enl.enlace._owlt(t0)
    assert 600 < owl < 1300                                   # Marte: 11-22 min de una via
    entrega, via = enl.enlace.entrega_comando(t0 + 1.0, enl.filtro.horizonte_s, dt_s=30.0, modo='ventanas')
    esperado = entrega - t0                                   # instante de escena de la entrega
    assert via == 'rele' and esperado - 1.0 > 2000            # mucho mas que el retardo de luz: espera los contactos
    # FiltroEnlace solo mira t: se salta de golpe a los instantes de interes
    enl.tick(0.0, json.dumps({'t': 0.0}), [])
    enl.tick(1.0, json.dumps({'t': 1.0}), [])
    cmds, _, _, info = enl.tick(1.1, json.dumps({'t': 1.1}), [(0.2, 0.0)])      # la orden mandada en t = 1.0 entra con ese sello
    assert cmds == [] and info['cmd_pendientes'] == 1
    assert enl.tick(esperado - 0.2, json.dumps({'t': esperado - 0.2}), [])[0] == []
    assert enl.tick(esperado + 0.1, json.dumps({'t': esperado + 0.1}), [])[0] == [(0.2, 0.0)]


def test_perdida_total_de_ordenes_no_entrega_nada_y_se_cuenta():
    enl = L.EnlaceLock(L.cargar_cfg(None, {'planeta': 'luna', 'enlace': 'retardo', 'p_perdida_cmd': 1.0}))
    out = _ticks(enl, 0.0, 4.0, {0.5: [(0.3, 0.0)]})
    assert not any(c for _, c, _ in out)
    assert enl.filtro.stats['cmd_perdidos'] == 1


# ------------------------------------------------------------------------------------------------- cadena completa con enlace
def _cadena_enlace(extra=None, **kw):
    cfg = {'planeta': 'luna', 'terreno': 'llano', 'semilla': 1, 'duracion_s': 8, 'enlace': 'retardo', 'operador': 'guion',
           'guion': [[1.0, 0.3, 0.0], [4.0, 0.0, 0.0]], 'dinamica': 'cinematica', **(extra or {}), **kw}
    return cfg


def test_cadena_con_enlace_aplica_la_orden_con_el_retardo_y_es_determinista(tmp_path):
    cfg = _cadena_enlace()
    owl = L.EnlaceLock(L.cargar_cfg(None, cfg)).enlace._owlt(0.0)
    met1, filas1, mandos1 = L.correr_cadena(cfg, str(tmp_path / 'a'))
    met2, filas2, mandos2 = L.correr_cadena(cfg, str(tmp_path / 'b'))
    for f in ('telemetry.jsonl', 'metrics.json', 'mandos.jsonl'):
        assert (tmp_path / 'a' / f).read_bytes() == (tmp_path / 'b' / f).read_bytes()
    assert [m['v'] for m in mandos1] == [0.3, 0.0]
    for m, t_env in zip(mandos1, (1.0, 4.0)):
        assert owl <= m['t'] - t_env < owl + 0.1 + 1e-9
    # el rover no se mueve hasta que llega la primera orden y se detiene cuando llega la segunda
    primera = next(r for r in filas1 if r['v'] > 0)
    assert primera['t'] == pytest.approx(mandos1[0]['t'] + 0.1, abs=1e-6)       # la orden rige [t, t+0.1): el primer estado con v > 0 es el siguiente
    assert met1['enlace']['cmd_enviados'] == 2 and met1['enlace']['cmd_no_llegan'] == 0


def test_cadena_con_enlace_y_navegador_en_tierra_mueve_al_rover_con_estados_atrasados(tmp_path):
    cfg = {'planeta': 'luna', 'terreno': 'llano', 'ruta': [[1.5, 0]], 'duracion_s': 14, 'v_max': 0.5, 'enlace': 'retardo',
           'operador': 'navegador', 'navegacion': 'basico', 'dinamica': 'cinematica'}
    met, filas, mandos = L.correr_cadena(cfg, str(tmp_path / 'c'))
    owl = L.EnlaceLock(L.cargar_cfg(None, cfg)).enlace._owlt(0.0)
    # el primer mando llega a 2*owl del primer estado (ida del estado + vuelta de la orden), cuantizado al paso
    assert 2 * owl <= mandos[0]['t'] < 2 * owl + 0.3
    assert filas[-1]['x'] > 0.3
    assert met['enlace']['est_enviados'] > 30


def test_cadena_con_enlace_de_la_dinamica_de_r1_es_determinista(tmp_path):
    cfg = {'planeta': 'luna', 'terreno': 'llano', 'duracion_s': 4, 'enlace': 'ideal', 'operador': 'guion', 'guion': [[0.5, 0.4, 0.0]]}
    L.correr_cadena(cfg, str(tmp_path / 'a'))
    L.correr_cadena(cfg, str(tmp_path / 'b'))
    for f in ('telemetry.jsonl', 'metrics.json', 'mandos.jsonl'):
        assert (tmp_path / 'a' / f).read_bytes() == (tmp_path / 'b' / f).read_bytes()
    assert json.loads((tmp_path / 'a' / 'mandos.jsonl').read_text().splitlines()[0])['t'] == pytest.approx(0.6)


def test_cargar_cfg_rechaza_enlace_y_navegacion_desconocidos():
    with pytest.raises(ValueError):
        L.cargar_cfg(None, {'enlace': 'magico'})
    with pytest.raises(ValueError):
        L.cargar_cfg(None, {'navegacion': 'magica'})
    assert math.isclose(L.PASO_S, 10 * L.DT)
