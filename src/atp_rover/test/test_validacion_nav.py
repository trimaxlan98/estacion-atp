"""Pruebas rapidas (sin dinamica) de la validacion fuera de muestra R10: casos, progreso, atasco real y bootstrap."""
import math

import numpy as np

from atp_rover import validacion_nav as vn


def _fila(t, x, y):
    return {'t': t, 'x': x, 'y': y}


def test_casos_disjuntos_y_deterministas():
    a, b = vn.casos(101), vn.casos(201)
    assert len(a) == len(b) == 150
    assert a == vn.casos(101)
    assert {c['semilla'] for c in a} == set(range(101, 131))
    assert {c['semilla'] for c in b} == set(range(201, 231))
    assert not {c['semilla'] for c in a} & {c['semilla'] for c in b}
    assert not {c['semilla'] for c in a} & {1, 2, 3}
    assert len({vn.clave_caso(c) for c in a}) == 150            # ningun caso repetido
    assert {c['planeta'] for c in a} == set(vn.PLANETAS) and {c['regimen'] for c in a} == set(vn.REGIMENES)
    assert {c['ruta_nombre'] for c in a} == set(vn.ORDEN_RUTAS)
    lat = [c for c in a if c['regimen'] == 'lateral'][0]
    assert lat['terreno'] == 'pendiente' and lat['terreno_params']['direccion_deg'] == 90.0
    assert len(vn.casos_rapidos(101)) == 6


def test_progreso_ida_y_vuelta_no_se_confunde():
    ruta = [[10, 0], [0, 0]]
    filas = [_fila(i, x, 0.0) for i, x in enumerate([0, 5, 9.5, 8, 4, 0.4])]
    p = vn.progreso(filas, ruta)
    assert p == sorted(p)                                       # no decrece
    assert abs(p[2] - 10.5) < 1e-9                              # a 0.5 m de la primera meta (radio 1 m) ya cuenta: 10 + 0.5 sobre el tramo de vuelta
    assert abs(p[-1] - vn.largo_ruta(ruta)) < 1e-9              # llego al origen: ruta completa
    assert abs(vn.largo_ruta(ruta) - 20.0) < 1e-9


def _prog(t, x, ruta):
    return vn.progreso([_fila(tt, xx, 0.0) for tt, xx in zip(t, x)], ruta)


def test_atasco_real_ignora_vaiven_que_libera_y_cuenta_el_que_no():
    t = [0.1 * k for k in range(0, 1200)]                       # 120 s
    ruta = [[20, 0]]
    # avanza siempre (un vaiven que libera al rover sube el progreso): sin atasco real
    assert vn.atasco_real(t, _prog(t, [0.05 * tt for tt in t], ruta))['episodios'] == 0
    # quieto de 20 s a 60 s y luego sigue: un episodio de ~40 s, no terminal
    x2 = [0.05 * tt if tt < 20 else (1.0 if tt < 60 else 1.0 + 0.05 * (tt - 60)) for tt in t]
    r = vn.atasco_real(t, _prog(t, x2, ruta))
    assert r['episodios'] == 1 and not r['terminal'] and 40.0 < r['t_atascado_s'] < 60.0
    # quieto hasta el final sin llegar: episodio terminal (salvo que haya llegado)
    p3 = _prog(t, [min(0.05 * tt, 1.0) for tt in t], ruta)
    r3 = vn.atasco_real(t, p3)
    assert r3['episodios'] == 1 and r3['terminal']
    assert not vn.atasco_real(t, p3, llego=True)['terminal']
    # un forcejeo de +-0.4 m durante 25 s sin progreso neto no llega a la ventana de 30 s
    tt5 = t[:450]
    xv = [min(0.05 * s, 1.0) + (0.2 * math.sin(s) if 20 <= s < 45 else 0.0) for s in tt5]
    assert vn.atasco_real(tt5, _prog(tt5, xv, ruta))['episodios'] == 0


def test_bootstrap_semilla_fija_y_contiene_la_media():
    x = np.array([1, 0, 0, 1, 1, -1, 0, 0, 1, 0, 1, 1], dtype=float)
    a, b = vn.bootstrap_ic(x), vn.bootstrap_ic(x)
    assert a == b                                               # determinista
    est, lo, hi = a
    assert lo <= est <= hi and abs(est - x.mean()) < 1e-12
    assert vn.bootstrap_ic(x, semilla=1) != a                   # otra semilla, otro IC
    assert all(math.isnan(v) for v in vn.bootstrap_ic([]))
    assert vn.bootstrap_ic(np.zeros(20)) == (0.0, 0.0, 0.0)


def test_signo_exacto_y_comparaciones():
    assert vn.prueba_signo_exacta(0, 0) == 1.0
    assert abs(vn.prueba_signo_exacta(0, 10) - 2 / 1024) < 1e-12
    assert vn.prueba_signo_exacta(5, 5) == 1.0
    c = vn.comparar_binaria([1, 1, 1, 0, 0, 1], [1, 0, 0, 0, 1, 1])
    assert c['solo_a'] == 2 and c['solo_b'] == 1 and abs(c['dif'] - 1 / 6) < 1e-12
    r = vn.comparar_razon([2.0, 4.0, 8.0], [1.0, 2.0, 4.0])
    assert abs(r['razon'] - 2.0) < 1e-9 and abs(r['ic_lo'] - 2.0) < 1e-9
    assert vn.comparar_razon([], [])['n'] == 0


def _par(llega_a, llega_b, vuelco_a=0):
    def f(ll, vu):
        return {'cfg': {'navegacion': 'x', 'k': 1}, 'met': {'meta_alcanzada': ll, 'vuelcos': vu, 'atascos': 0, 'tiempo_s': 50.0, 'energia_wh': 0.1},
                'val': {'atasco_real_episodios': 0, 'atasco_terminal': False, 'wh_por_m_util': 0.01, 'dist_sobre_recta': 1.05}}
    return f(llega_a, vuelco_a), f(llega_b, 0)


def test_criterios_se_evaluan_con_los_umbrales_preregistrados():
    pares = [_par(True, True)] * 20 + [_par(False, True)] * 5
    comp = vn.comparar_modos(pares)
    ter = {'t': vn.comparar_binaria([0] * 5, [1] * 5)}
    crit = {c['id']: c for c in vn.evaluar_criterios(comp, ter, {})}
    assert not crit['C1']['cumple'] and not crit['C7']['cumple'] and crit['C2']['cumple']
    assert crit['C4']['cumple'] and crit['C5']['cumple']
    iguales = vn.comparar_modos([_par(True, True)] * 25)
    assert all(c['cumple'] for c in vn.evaluar_criterios(iguales, {}, {}) if c['id'] != 'C1b')
    con_vuelco = vn.evaluar_criterios(vn.comparar_modos([_par(True, True, vuelco_a=1)] * 25), {}, {})
    assert not {c['id']: c for c in con_vuelco}['C2']['cumple']


def test_completa_r10_registrada_y_la_huella_filtra_bordes_de_roca():
    from atp_rover import lazo
    from atp_rover.navegacion import Navegador, ParamsNav, pendiente_deg
    from atp_rover.planetas import planeta
    from atp_rover.terreno import Terreno
    assert 'completa_r10' in lazo.NAVEGACIONES and lazo.NAV_LAZO_R10['pendiente_huella_m'] > 0
    assert 'pendiente_huella_m' not in lazo.NAV_LAZO_R6 and ParamsNav().pendiente_huella_m == 0.0     # completa y completa_r4 no cambian
    suelo = planeta('marte')['suelo']
    rampa = Terreno('pendiente', 0, suelo, angulo_deg=15.0)
    nav = Navegador(rampa, ParamsNav(pendiente_huella_m=0.6))
    assert abs(nav._pendiente_huella({'x': 5.0, 'y': 1.0}) - 15.0) < 0.5
    esc = Terreno('escalones', 0, suelo, altura=0.2, x_inicio=5.0, n=1)
    nav = Navegador(esc, ParamsNav(pendiente_huella_m=0.6))
    punto = max(pendiente_deg(esc, 5.0 + dx, 0.0) for dx in (-0.1, 0.0, 0.1))
    assert punto > 25.0                                                          # el borde vertical dispara la alarma puntual de 25 grados
    assert nav._pendiente_huella({'x': 5.0, 'y': 0.0}) < punto
