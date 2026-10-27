"""Pruebas de la orquestacion de la mision integrada, con corridas SIMULADAS
(`util_mision.EjecutorSimulado`: sin ROS) sobre el plan, el mundo y la OD reales.

    cd /ws && python3 -m pytest -q src/atp_mision/test            # dentro del contenedor
    PYTHONPATH=src/atp_ros python3 -m pytest -q src/atp_mision/test   # o en el host (numpy + sgp4)

La integracion con ROS de verdad (lockstep, autotrack, SDR) es `test_integracion.py`
(marca `integration`) y `scripts/validar.sh mision`.
"""
import datetime as dt
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

from atp_mision import contabilidad as C
from atp_mision import ejecucion as E
from atp_mision import informe as I
from atp_mision import mundo as W
from atp_mision.mision import Config, Mision
from atp_ros import orbit
from atp_ros import planificador as pl
from util_mision import EjecutorSimulado

CFG = dict(sats=('ISS', 'NOAA 19', 'NOAA 20'), estaciones=('cdmx', 'madrid', 'tromso'),
           inicio='2026-10-02T12:00:00', horas=6, n_od=3, n_min_od=3, latencia_od_s=300,
           concurrencia=3)


def _correr(carpeta, ejecutor=None, **kw):
    cfg = Config(**{**CFG, **kw})
    m = Mision(cfg, str(carpeta), ejecutor=ejecutor or EjecutorSimulado(), log=lambda *a: None)
    inf = m.correr()
    return m, inf


@pytest.fixture(scope='module')
def dia(tmp_path_factory):
    return _correr(tmp_path_factory.mktemp('dia'))


# ---------------------------------------------------------------- contabilidad

def _filas(cn0, lp=0.0, dur=100.0, dt_s=0.1, e_pt=0.01):
    n = int(dur / dt_s)
    return [{'t': round((i + 1) * dt_s, 3), 'cn0_dbhz': cn0, 'l_point_db': lp, 'e_pt': e_pt,
             'link_closed': True} for i in range(n)]


def test_contabilidad_volumen_exacto_con_modcod_conocido():
    rs = 10e6
    # Es/N0 = 16.05 + margen 1.0 + 0.1: 32APSK 9/10, eficiencia 4.453027
    cn0 = 10 * math.log10(rs) + 17.15
    k = C.contabilizar(_filas(cn0), rs, 1.0)
    esperado = rs * 4.453027 * (k['minutos'] * 60.0) / 8e9
    assert k['gb'] == pytest.approx(esperado, rel=1e-12)
    assert k['modcod_max'] == '32APSK 9/10'
    assert k['perdida_apunt_gb'] == pytest.approx(0.0, abs=1e-12)
    assert k['apunt_rms_deg'] == pytest.approx(0.01)


def test_contabilidad_perdida_por_apuntamiento():
    rs = 10e6
    cn0_ideal = 10 * math.log10(rs) + 17.15
    # con 6 dB de perdida: Es/N0 = 11.15 - 1.0 de margen = 10.15 dB disponibles para
    # el MODCOD; el mejor es 16APSK 3/4 (10.21 dB ideal + 1 > 11.15? no): se calcula contra la tabla
    k = C.contabilizar(_filas(cn0_ideal - 6.0, lp=6.0), rs, 1.0)
    ref = C.contabilizar(_filas(cn0_ideal), rs, 1.0)
    assert k['gb'] < ref['gb']
    assert k['gb_sin_apunt'] == pytest.approx(ref['gb'], rel=1e-12)
    assert k['perdida_apunt_gb'] == pytest.approx(ref['gb'] - k['gb'], rel=1e-9)
    assert k['perdida_mayor_db_pct'] == 100.0
    # sin enlace: ni QPSK 1/4 (-2.35 + 1 dB)
    k0 = C.contabilizar(_filas(10 * math.log10(rs) - 5.0), rs, 1.0)
    assert k0['gb'] == 0.0 and k0['modcod_max'] == 'sin enlace' and k0['sin_modcod_pct'] == 100.0


def test_contabilidad_a_mano_coincide_con_numpy():
    rng = np.random.default_rng(3)
    filas = [{'t': round((i + 1) * 0.1, 3), 'cn0_dbhz': float(80 + 12 * math.sin(i / 300)
                                                              + rng.normal(0, 0.3)),
              'l_point_db': 0.0} for i in range(4000)]
    for rs in (2e6, 10e6):
        a = C.contabilizar(filas, rs, 1.0)
        b = C.contabilizar_a_mano(filas, rs, 1.0)
        assert a['gb'] == pytest.approx(b['gb'], rel=1e-12)
        assert a['minutos'] == pytest.approx(b['minutos'], rel=1e-12)


def test_contabilidad_ignora_filas_fuera_de_pase():
    f = _filas(95.0)
    f.insert(0, {'t': 0.0})                       # la espera en AOS: sin enlace
    f.append({'t': 100.2, 'az': 1.0})
    assert C.contabilizar(f, 10e6)['muestras'] == len(f) - 2
    assert C.contabilizar([{'t': 0.0}], 10e6) is None


def test_planificador_y_contabilidad_coinciden_sin_error_de_apuntamiento():
    """Con el rango del TLE y 0 grados de error, lo medido (a 1 s) es lo que predice el
    planificador: el volumen de una ventana (dt = 1 s, trapecios) y el de `contabilizar`
    con las mismas muestras."""
    cat = orbit.cargar_catalogo()
    tle = orbit.buscar_tle(cat, 'NOAA 19')
    est, _ = orbit.cargar_estaciones()
    perfil = pl.PerfilEnlace(banda='S', eirp_dbw=10.0, rs_baud=4e6)
    p = orbit.predecir_pases(tle, est['tromso'], orbit.a_utc('2026-10-02T14:00:00'), 1.0, None)[0]
    v = pl.ventana_de_pase(tle, est['tromso'], 'tromso', p, orbit.a_utc('2026-10-02T14:00:00'),
                           perfil, pl.Rotor())
    s = orbit.muestrear_pase(tle, est['tromso'], p, 1.0)
    cn0 = pl.cn0_enlace(s['d'], perfil)
    filas = [{'t': float(t), 'cn0_dbhz': float(c), 'l_point_db': 0.0}
             for t, c in zip(s['t'], cn0)]
    k = C.contabilizar(filas, 4e6, perfil.margen_acm_db)
    assert k['gb'] == pytest.approx(v.vol_gb, rel=1e-9)


# ---------------------------------------------------------------- dominios y subprocesos

def test_argumentos_del_launch():
    a = E.argumentos({'sat': 'ISS (ZARYA)', 'autotrack': True, 'sdr': False, 'od_file': '',
                      'pass_index': 0, 'x': None})
    assert a == ['sat:=ISS (ZARYA)', 'autotrack:=true', 'sdr:=false', 'pass_index:=0']


def test_pool_de_dominios_propio_y_sin_repetir():
    assert min(E.DOMINIOS) == 200 and max(E.DOMINIOS) == 229
    assert not set(E.DOMINIOS) & set(range(61, 200))          # ni el backend ni las pruebas
    pool = E.PoolDominios()
    tomados = [pool.tomar() for _ in range(30)]
    assert sorted(tomados) == list(range(200, 230))
    pool.devolver(tomados[0])
    assert pool.tomar() == tomados[0]


def _comando_falso(args, carpeta):
    codigo = ("import os,sys,json;"
              "d=os.environ['ROS_DOMAIN_ID'];"
              f"open(r'{carpeta}/dominio.txt','w').write(d);"
              + ("" if args.get('falla') else f"json.dump({{'n':1}}, open(r'{carpeta}/metrics.json','w'))"))
    return [sys.executable, '-c', codigo]


def test_ejecutor_ros_usa_un_dominio_por_corrida_y_lo_devuelve(tmp_path):
    pool = E.PoolDominios(range(210, 212))
    ej = E.EjecutorROS(dominios=pool, comando=_comando_falso, plazo_s=30)
    r1 = ej({}, str(tmp_path / 'a'))
    r2 = ej({}, str(tmp_path / 'b'))
    assert r1['estado'] == 'ok' and r2['estado'] == 'ok'
    d = {int(open(tmp_path / n / 'dominio.txt').read()) for n in 'ab'}
    assert d <= {210, 211}
    # devueltos al pool: se pueden tomar los dos otra vez
    assert {pool.tomar(), pool.tomar()} == {210, 211}


def test_ejecutor_ros_corridas_en_paralelo_dominios_distintos(tmp_path):
    import concurrent.futures as cf
    codigo = ("import os,time,json;time.sleep(1.0);"
              "open('dominio.txt','w').write(os.environ['ROS_DOMAIN_ID'])")
    pool = E.PoolDominios(range(220, 223))

    def cmd(args, carpeta):
        return [sys.executable, '-c',
                codigo.replace("'dominio.txt'", repr(os.path.join(carpeta, 'dominio.txt')))
                + f";json.dump({{}}, open(r'{carpeta}/metrics.json','w'))"]
    ej = E.EjecutorROS(dominios=pool, comando=cmd, plazo_s=30)
    with cf.ThreadPoolExecutor(3) as ex:
        rs = list(ex.map(lambda i: ej({}, str(tmp_path / f'c{i}')), range(3)))
    assert all(r['estado'] == 'ok' for r in rs)
    assert sorted(r['dominio'] for r in rs) == [220, 221, 222]


def test_ejecutor_ros_fallo_y_plazo(tmp_path):
    ej = E.EjecutorROS(dominios=E.PoolDominios(range(225, 227)), comando=_comando_falso, plazo_s=30)
    assert ej({'falla': True}, str(tmp_path / 'f'))['estado'] == 'fallo'   # sin metrics.json
    lento = E.EjecutorROS(dominios=E.PoolDominios(range(225, 227)), plazo_s=1.0, gracia_s=1.0,
                          comando=lambda a, c: [sys.executable, '-c', 'import time; time.sleep(30)'])
    r = lento({}, str(tmp_path / 'l'))
    assert r['estado'] == 'plazo'


def test_ejecutor_ros_mata_la_corrida_colgada(tmp_path):
    """Una corrida que no escribe nada (DDS que no empareja: el generador espera el saludo de
    arranque para siempre) se mata por inactividad y se anota; una que escribe, no."""
    mudo = E.EjecutorROS(dominios=E.PoolDominios(range(225, 227)), plazo_s=60, gracia_s=1.0,
                         inactividad_s=3.0,
                         comando=lambda a, c: [sys.executable, '-c', 'import time; time.sleep(60)'])
    assert mudo({}, str(tmp_path / 'm'))['estado'] == 'inactiva'

    def hablador(args, carpeta):
        codigo = ("import time, json\nfor i in range(8):\n"
                  f"    open(r'{carpeta}/telemetry.jsonl', 'a').write('x\\n'); time.sleep(1)\n"
                  f"json.dump({{}}, open(r'{carpeta}/metrics.json', 'w'))")
        return [sys.executable, '-c', codigo]
    vivo = E.EjecutorROS(dominios=E.PoolDominios(range(225, 227)), plazo_s=60, inactividad_s=3.0,
                         comando=hablador)
    assert vivo({}, str(tmp_path / 'v'))['estado'] == 'ok'


def test_lectura_del_launch_log(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_text(
        '[autotrack-8] [INFO] [1.0] [autotrack]: ENGANCHE\n'
        '[autotrack-8] [INFO] [2.0] [autotrack]: perdida de enganche\n'
        '[autotrack-8] [INFO] [3.0] [autotrack]: ENGANCHE\n'
        '[sdr_gate-9] [WARN] [4.0] [sdr_gate]: sin confirmacion del receptor SDR; no repetible\n'
        '[sdr_receiver-10] [INFO] [5.0] [sdr_receiver]: RESUMEN SDR banda=S modo=portadora fs=150 kHz '
        '| bloques=8151 huecos=0 | Doppler rms=0.024 Hz sesgo=-0.001 Hz max=1.24 Hz | C/N0 '
        'medido-presupuesto sesgo=+0.002 dB rms=0.172 dB | enganchado 100.0% (adquisiciones=1 '
        'perdidas=0)\n')
    info, avisos = E.incidencias_de_log(str(log))
    assert info['enganches'] == 2 and info['perdidas_enganche'] == 1
    assert len(avisos) == 1 and 'no repetible' in avisos[0]
    sdr = E.parse_resumen_sdr(info['sdr'])
    assert sdr == {'doppler_rms_hz': 0.024, 'cn0_sesgo_db': 0.002, 'cn0_rms_db': 0.172,
                   'enganchado_pct': 100.0, 'huecos': 0}


def test_la_carrera_de_apagado_de_rclpy_no_es_una_incidencia(tmp_path):
    """El Traceback de `Task.__del__` al apagar depende de la carga: no debe volver
    distinto el informe (determinismo, CONTRATO 8). Un Traceback real sigue contando."""
    log = tmp_path / 'launch.log'
    log.write_text(
        '[link_budget-5] Traceback (most recent call last):\n'
        '[link_budget-5]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/task.py", line 56, in __del__\n'
        "[link_budget-5]     if self._exception is not None and not self._exception_fetched:\n"
        "[link_budget-5] AttributeError: 'Task' object has no attribute '_exception'\n"
        '[mount_sim-3] Traceback (most recent call last):\n'
        '[mount_sim-3]   File "/ws/src/atp_ros/atp_ros/mount_sim.py", line 10, in cb\n'
        '[mount_sim-3] ValueError: boom\n')
    _, avisos = E.incidencias_de_log(str(log))
    assert len(avisos) == 1 and avisos[0].startswith('[mount_sim-3]')


# ---------------------------------------------------------------- plan y papeles

def test_papeles_de_los_contactos(dia):
    m, inf = dia
    cs = m.contactos
    assert cs and len({c.id for c in cs}) == len(cs)
    for sat in {c.sat for c in cs}:
        propios = [c for c in cs if c.sat == sat]
        cal = [c for c in propios if c.rol == 'cal']
        assert all(m.cfg.el_cal_min_deg <= c.el_max_deg <= m.cfg.el_cal_max_deg and not c.keyhole
                   for c in cal)
        primeros = [c for c in propios
                    if m.cfg.el_cal_min_deg <= c.el_max_deg <= m.cfg.el_cal_max_deg
                    and not c.keyhole][:m.cfg.n_od]            # los n_od primeros utiles
        assert [c.id for c in cal] == [c.id for c in primeros]
        assert 0 < len(cal) <= m.cfg.n_od
    for c in cs:
        assert c.autotrack == (c.rol == 'cal' or c.banda in ('X', 'Ka'))
        assert c.sdr == (c.rol == 'cal' and c.banda in ('UHF', 'S'))
    assert {c.banda for c in cs} == {'S', 'X'}


def test_dependencias_de_la_efemeride(dia):
    """Un contacto solo vuela con una orbita hecha con calibraciones que TERMINARON antes
    de su AOS (mas la latencia), y esas calibraciones son de su satelite."""
    m, inf = dia
    por_id = {c.id: c for c in m.contactos}
    vistos = 0
    for f in inf['contactos']:
        if f['efemeride'] != 'od':
            continue
        vistos += 1
        c = por_id[f['id']]
        assert len(f['od_contactos']) >= m.cfg.n_min_od
        for i in f['od_contactos']:
            k = por_id[i]
            assert k.sat == c.sat and k.rol == 'cal'
            assert k.los + dt.timedelta(seconds=m.cfg.latencia_od_s) <= c.aos
    assert vistos >= 3


def test_el_plan_es_el_optimo_del_planificador(dia):
    m, inf = dia
    plan = m.plan
    opt = plan['planes']['optimo']
    assert opt['optimo_probado']
    assert sorted(c.id for c in m.contactos) == sorted(opt['ids'])
    assert inf['plan']['gb'] == opt['resumen']['gb']
    # cada contacto es una ventana valida del plan, con la banda de la carga de su satelite
    for c in m.contactos:
        assert c.banda == m.carga(c.sat).banda
    # las bandas de las cargas entraron al plan: los volumenes de S y X difieren
    assert plan['bandas']['NOAA 19'] == 'S' and plan['bandas']['NOAA 20 (JPSS-1)'] == 'X'


# ---------------------------------------------------------------- totales e informe

def test_totales_cuadran_con_la_suma_de_las_corridas(dia):
    m, inf = dia
    ok = [f for f in inf['contactos'] if f['estado'] == 'ok']
    t = inf['totales']
    assert t['contactos_ejecutados'] == len(ok) == len(m.contactos)
    assert t['gb'] == pytest.approx(sum(f['gb'] for f in ok), abs=0.002)
    assert t['minutos'] == pytest.approx(sum(f['minutos'] for f in ok), abs=0.05)
    assert t['perdida_apunt_gb'] == pytest.approx(sum(f['perdida_apunt_gb'] for f in ok), abs=0.002)
    # por satelite y por estacion suman lo mismo
    for clave in ('por_satelite', 'por_estacion'):
        assert sum(v['gb'] for v in inf[clave].values()) == pytest.approx(t['gb'], abs=0.005)
        assert sum(v['contactos'] for v in inf[clave].values()) == t['contactos_ejecutados']
    # releido desde cero, de las carpetas, con otro codigo
    v = inf['verificacion']
    assert v['cuadra_gb'] and v['cuadra_minutos'] and v['corridas_releidas'] == len(ok)
    gb = 0.0
    for f in ok:
        filas = C.leer_telemetria(os.path.join(m.carpeta, f['carpeta'], 'telemetry.jsonl'))
        if f['t_max_s'] is not None:
            filas = [x for x in filas if x['t'] <= f['t_max_s']]
        gb += C.contabilizar_a_mano(filas, m.carga(f['sat']).rs_baud, m.cfg.margen_acm_db)['gb']
    assert gb == pytest.approx(t['gb'], abs=0.003)
    # la suma de metricas de cada corrida existe en disco
    for f in ok:
        assert os.path.exists(os.path.join(m.carpeta, f['carpeta'], 'metrics.json'))


def test_perdida_por_apuntamiento_es_positiva_o_cero_y_acotada(dia):
    m, inf = dia
    for f in inf['contactos']:
        assert -1e-6 <= f['perdida_apunt_gb'] <= f['gb_sin_apunt'] + 1e-9
        assert f['gb_sin_apunt'] == pytest.approx(f['gb'] + f['perdida_apunt_gb'], abs=2e-4)


def test_la_od_mejora_la_efemeride_y_el_apuntamiento(dia):
    m, inf = dia
    j = inf['mejora_od']
    assert j['contactos_con_od'] >= 3 and j['contactos_con_gemelo'] == len(j['pares']) >= 3
    # error de efemeride contra la verdad: la orbita estimada gana a TLE en CADA par
    for p in j['pares']:
        assert p['err_efemeride_od_deg']['max'] < p['err_efemeride_tle_deg']['max']
        assert p['apunt_rms_od_deg'] < p['apunt_rms_tle_deg']
        assert p['seguimiento'] == 'program'
    assert j['err_efemeride_max_od_mediana_deg'] < 0.1 * j['err_efemeride_max_tle_mediana_deg']
    # y la orbita es mejor que el TLE en la epoca de la OD (contra la verdad)
    aceptadas = [o for o in inf['od'] if o['aceptada']]
    assert aceptadas
    for o in aceptadas:
        assert o['error_pos_epoca_km'] < o['error_pos_tle_epoca_km']
        assert 0.3 < o['chi2_dof'] < 3.0


def test_los_gemelos_son_el_mismo_contacto_con_tle(dia):
    m, inf = dia
    for p in inf['mejora_od']['pares']:
        c = next(c for c in m.contactos if c.id == p['id'])
        g = m.res[(c.id, 'tle')]
        r = m.res[(c.id, 'real')]
        assert g['efemeride'] == 'tle' and r['efemeride'] == 'od'
        assert g['carpeta'].endswith('_tle')


def test_informe_texto_y_metricas(dia):
    m, inf = dia
    txt = I.texto(inf)
    for s in ('Planificado contra ejecutado', 'Determinacion de orbita', 'Mejora por la OD',
              'Verificacion', 'cuadra con los totales: GB si'):
        assert s in txt
    met = I.metricas(inf)
    assert met['kind'] == 'mision' and met['cuadra'] is True
    assert met['gb'] == inf['totales']['gb'] and met['contactos_cal'] > 0
    json.dumps(met)
    for nombre in ('plan.json', 'informe.json', 'informe.txt', 'metrics.json', 'mision.json'):
        assert os.path.exists(os.path.join(m.carpeta, nombre))


def test_determinismo_dos_dias_dan_el_mismo_informe(dia, tmp_path):
    m1, _ = dia
    # otra ejecucion, otra carpeta, otra concurrencia: el informe es identico byte a byte
    m2, _ = _correr(tmp_path, concurrencia=1)
    # (la configuracion guardada difiere en la concurrencia, que no cambia el resultado)
    a = json.load(open(os.path.join(m1.carpeta, 'informe.json')))
    b = json.load(open(os.path.join(m2.carpeta, 'informe.json')))
    assert a['mision']['config'].pop('concurrencia') == 3
    assert b['mision']['config'].pop('concurrencia') == 1
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)
    # el plan y las ordenes de las corridas tambien
    assert open(os.path.join(m1.carpeta, 'plan.json')).read() == open(
        os.path.join(m2.carpeta, 'plan.json')).read()


# ---------------------------------------------------------------- incidencias

def test_corrida_fallida_se_reintenta_y_se_anota(tmp_path):
    ej = EjecutorSimulado(fallos={'primero': 1})              # falla una vez y el reintento sale
    m, inf = _correr(tmp_path, ej, sats=('ISS',), estaciones=('madrid', 'cdmx'), horas=4, n_od=2,
                     n_min_od=2)
    f = next(f for f in inf['contactos'] if f['contacto'] == ej.primero_nombre)
    assert f['estado'] == 'ok' and not any('intento' in s for s in f['incidencias'])
    # el fallo de infraestructura va a ejecucion.json, no al informe (no es determinista)
    assert m.res[(f['id'], 'real')]['infra'] == ['intento 1: fallo (rc 1)']
    ej_json = json.load(open(os.path.join(m.carpeta, 'ejecucion.json')))
    assert any(c['contacto'] == ej.primero_nombre and c['infra'] for c in ej_json['corridas'])
    assert sum(1 for a, c in ej.llamadas if c.endswith(ej.primero_nombre)) == 2


def test_corrida_que_siempre_falla_no_cuenta_y_no_rompe_el_dia(tmp_path):
    ej = EjecutorSimulado(fallos={'primero': 99})
    m, inf = _correr(tmp_path, ej, sats=('ISS',), estaciones=('madrid', 'cdmx'), horas=4, n_od=2,
                     n_min_od=2)
    f = next(f for f in inf['contactos'] if f['contacto'] == ej.primero_nombre)
    assert f['estado'] == 'fallo' and 'gb' not in f
    t = inf['totales']
    assert t['contactos_fallidos'] == 1
    assert t['contactos_ejecutados'] == t['contactos_planificados'] - 1
    assert any(i['contacto'] == ej.primero_nombre and i['tipo'] == 'no se ejecuto'
               for i in inf['incidencias'])
    assert inf['verificacion']['cuadra_gb']
    # con una calibracion perdida la OD no llega a n_min_od=2 con medidas y el resto vuela con TLE
    assert all(f['efemeride'] == 'tle' for f in inf['contactos'] if f['estado'] == 'ok')


def test_sin_enganche_no_hay_medidas_ni_od(tmp_path):
    """Si el autotrack no engancha (efemeride fuera del cono), los angulos de la
    montura no miden al satelite: no hay OD y el dia sigue con el TLE."""
    class SinEnganche(EjecutorSimulado):
        def __call__(self, args, carpeta):
            args = dict(args, autotrack=False)         # la montura se queda en program track
            return super().__call__(args, carpeta)
    m, inf = _correr(tmp_path, SinEnganche(err_montura_deg=0.5), sats=('ISS',),
                     estaciones=('madrid', 'cdmx'), horas=4, n_od=2, n_min_od=2)
    assert all(f['efemeride'] == 'tle' for f in inf['contactos'])
    assert inf['od'] == []
    assert any('sin medidas validas' in s for f in inf['contactos'] for s in f['incidencias'])
    assert any(i['tipo'] == 'autotrack' for i in inf['incidencias'])


def test_sin_od_el_dia_es_el_de_siempre(tmp_path):
    m, inf = _correr(tmp_path, n_od=0, horas=3)
    assert all(f['rol'] == 'normal' and f['efemeride'] == 'tle' for f in inf['contactos'])
    assert inf['od'] == [] and inf['mejora_od']['contactos_con_od'] == 0


def test_ventana_cortada_por_el_horizonte_se_cuenta_hasta_el_horizonte(tmp_path):
    """El pase de NOAA 20 sobre Madrid ya esta en curso a las 12:00: la corrida
    arranca en el inicio del horizonte y el contacto no cuenta mas que el plan."""
    ej = EjecutorSimulado()
    m, inf = _correr(tmp_path, ej, sats=('NOAA 20',), estaciones=('madrid',), horas=1, n_od=0)
    f = inf['contactos'][0]
    assert f['aos'] == '2026-10-02T12:00:00Z'
    a = next(a for a, c in ej.llamadas if c.endswith(f['contacto']))
    assert a['start_utc'] == '2026-10-02T12:00:00'
    assert f['minutos'] == pytest.approx(f['minutos_plan'], abs=0.1)
    assert f['gb'] == pytest.approx(f['gb_plan'], rel=0.02)


def test_args_de_la_corrida_usan_la_banda_y_la_verdad(dia):
    m, inf = dia
    por_carpeta = {os.path.basename(c): a for a, c in m.ejecutor.llamadas}
    for c in m.contactos:
        a = por_carpeta[c.nombre]
        assert a['band'] == c.banda and a['sat'] == c.sat and a['station'] == c.estacion
        assert a['truth_file'].endswith('.npz') and os.path.exists(a['truth_file'])
        assert a['autotrack'] == c.autotrack and a['sdr'] == c.sdr
        assert a['time_scale'] == 0 and a['pass_index'] == 0
        assert (a['profile'] == 'od') == bool(a['od_file'])
    # el maximo de concurrencia no se supera: lo comprueba la prueba de abajo


def test_concurrencia_maxima_respetada(tmp_path):
    import threading
    import time
    vivos = {'n': 0, 'max': 0}
    lock = threading.Lock()

    class Lento(EjecutorSimulado):
        def __call__(self, args, carpeta):
            with lock:
                vivos['n'] += 1
                vivos['max'] = max(vivos['max'], vivos['n'])
            time.sleep(0.15)
            try:
                return super().__call__(args, carpeta)
            finally:
                with lock:
                    vivos['n'] -= 1
    _correr(tmp_path, Lento(), concurrencia=2, n_od=0, horas=3)
    assert vivos['max'] == 2


def test_mundo_verdad_distinta_del_tle_y_creciente():
    cat = orbit.cargar_catalogo()
    est, _ = orbit.cargar_estaciones()
    tle = orbit.buscar_tle(cat, 'NOAA 19')
    w = W.Mundo([tle], {'tromso': est['tromso']}, orbit.a_utc('2026-10-04T00:00:00'))
    ef = w.esc['NOAA 19'].sgp4
    e1 = w.error_posicion_km('NOAA 19', ef, orbit.a_utc('2026-10-02T12:00:00'))
    e3 = w.error_posicion_km('NOAA 19', ef, orbit.a_utc('2026-10-03T22:00:00'))
    assert 0.1 < e1 < e3 < 60.0                    # el TLE envejece (docs/OD.md 3.2)
    p = w.error_efemeride('NOAA 19', 'tromso', orbit.a_utc('2026-10-02T14:09:00'), 600.0)
    assert 0.0 < p['aos'] < 1.0 and p['max'] >= p['rms'] > 0


def test_reanudar_reutiliza_las_corridas_hechas_y_da_el_mismo_informe(dia):
    """`--reanudar`: sobre la carpeta de un dia ya hecho no se lanza ninguna corrida y el
    informe sale identico (la ejecucion es determinista, reutilizarla = repetirla)."""
    m1, _ = dia
    antes = open(os.path.join(m1.carpeta, 'informe.json')).read()

    class NoDebeLlamarse:
        def __call__(self, args, carpeta):
            raise AssertionError('se lanzo una corrida ya hecha: ' + carpeta)
    m2 = Mision(Config(**{**CFG, 'reanudar': True}), m1.carpeta, ejecutor=NoDebeLlamarse(),
                log=lambda *a: None)
    m2.correr()
    assert open(os.path.join(m1.carpeta, 'informe.json')).read() == antes
    assert all(r.get('reanudada') for r in m2.res.values())
    # con otros argumentos (otra banda en la carga) NO se reutiliza
    assert not Mision._hecha(os.path.join(m1.carpeta, 'contactos', 'c001'), {'sat': 'otro'})
