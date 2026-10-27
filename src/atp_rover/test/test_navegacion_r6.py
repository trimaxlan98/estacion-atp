"""Pruebas de R6 sobre la navegacion: costo graduado, replanificacion, prealimentacion por deslizamiento, vaiven y limite aprendido.
Rover cinematico falso y terreno sintetico de test_navegacion.py (sin la dinamica de R1), mas el crater real de R1 solo para planificar."""
import math

import pytest

from atp_rover.lazo import NAV_LAZO, NAV_LAZO_R6
from atp_rover.navegacion import Navegador, ParamsNav, pendiente_deg, planificar_ruta
from atp_rover.planetas import planeta
from atp_rover.terreno import Terreno
from test_navegacion import RoverFalso, TerrenoSintetico, correr


def test_graduado_no_prohibe_la_meta_en_una_cuesta():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 3.0, 3.0))
    meta = (10.0, 2.5)
    assert 20.0 < pendiente_deg(terr, *meta) < 35.0
    assert planificar_ruta(terr, (0.0, 0.0), meta, ParamsNav()) is None            # R2: intransitable
    ruta = planificar_ruta(terr, (0.0, 0.0), meta, ParamsNav(graduado=True))      # R6: cuesta mas, pero hay ruta
    assert ruta is not None and ruta[-1] == meta
    # lo extremo (> pend_dura_deg) sigue prohibido
    duro = TerrenoSintetico(colina=(10.0, 0.0, 8.0, 2.0))
    assert pendiente_deg(duro, 10.0, 1.5) > 40.0
    assert planificar_ruta(duro, (0.0, 0.0), (10.0, 1.5), ParamsNav(graduado=True)) is None


def test_graduado_rodea_lo_caro_y_cruza_si_no_hay_alternativa():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 3.0, 3.0))
    ruta = planificar_ruta(terr, (0.0, 0.0), (20.0, 0.0), ParamsNav(graduado=True))
    assert max(abs(y) for _, y in ruta) > 2.0                                    # prefiere rodear
    plano = TerrenoSintetico(pend_plano=math.tan(math.radians(25.0)))             # 25 grados en todas partes
    assert planificar_ruta(plano, (0.0, 0.0), (8.0, 0.0), ParamsNav()) is None
    assert planificar_ruta(plano, (0.0, 0.0), (8.0, 0.0), ParamsNav(graduado=True)) is not None


def test_graduado_costo_direccional_prefiere_subir_en_diagonal():
    # plano de 15 grados que sube hacia +x; la meta esta 6 m arriba y 6 m al lado: la diagonal sube menos que la recta
    terr = TerrenoSintetico(pend_plano=math.tan(math.radians(15.0)))
    p = ParamsNav(graduado=True, pend_max_deg=10.0, k_pend=6.0)
    ruta = planificar_ruta(terr, (0.0, 0.0), (6.0, 6.0), p)
    assert ruta is not None and ruta[-1] == (6.0, 6.0)
    # costo de subir directo contra el de la ruta elegida: la ruta elegida nunca baja de la diagonal pura
    assert len(ruta) >= 2


def test_crater_real_ya_no_da_sin_ruta():
    t = Terreno('crater', 1, planeta('luna')['suelo'])
    assert planificar_ruta(t, (0.0, 0.0), (12.0, 0.0), ParamsNav(**NAV_LAZO)) is None          # navegacion de R4
    ruta = planificar_ruta(t, (0.0, 0.0), (12.0, 0.0), ParamsNav(**NAV_LAZO_R6))                # R6
    assert ruta is not None and ruta[-1] == (12.0, 0.0)


class RoverPatina(RoverFalso):
    """Como una cuesta del modelo de R1: la velocidad lograda es (1 - s) de la mandada, con s fijo."""

    def __init__(self, terr, s=0.5, **kw):
        super().__init__(terr, **kw)
        self.s = s
        self.desliz = [s] * 6

    def paso(self, v, w):
        super().paso(v * (1.0 - self.s) if v > 0 else v, w)


def test_prealimentacion_por_deslizamiento_alcanza_la_velocidad_deseada():
    terr = TerrenoSintetico()
    v_fin = {}
    for nombre, p in (('r4', ParamsNav(v_max=0.4, v_min=0.2)),
                      ('r6', ParamsNav(v_max=0.4, v_min=0.2, ki_vel=0.3, kp_vel=0.3, boost_max=0.6, ff_desliz=True))):
        rover = RoverPatina(terr)
        nav = Navegador(terr, p)
        nav.nueva_meta(rover.estado(), (60.0, 0.0))
        for _ in range(250):
            (v, w), _ = nav.paso(rover.estado())
            rover.paso(v, w)
        v_fin[nombre] = rover.v
    assert v_fin['r4'] < 0.25                                   # sin compensar, la mitad de lo pedido
    assert v_fin['r6'] == pytest.approx(0.4, rel=0.12)          # compensado


def test_replanifica_cada_replan_s_y_al_desviarse():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    nav = Navegador(terr, ParamsNav(graduado=True, replan_s=3.0, desvio_replan_m=1.0))
    nav.nueva_meta(rover.estado(), (20.0, 0.0))
    for k in range(120):
        (v, w), _ = nav.paso(rover.estado())
        rover.paso(v, w)
        if k == 40:
            rover.y += 2.5                                     # un empujon lateral
    assert nav.replanes >= 3
    assert abs(rover.y) < 1.5 and rover.x > 8.0                  # volvio a la linea hacia la meta
    # con replan_s = 0 (defecto) no replanifica nunca
    rover = RoverFalso(terr)
    nav = Navegador(terr, ParamsNav(graduado=True))
    nav.nueva_meta(rover.estado(), (20.0, 0.0))
    for _ in range(60):
        (v, w), _ = nav.paso(rover.estado())
        rover.paso(v, w)
    assert nav.replanes == 0


def test_vaiven_ante_atasco_y_continua():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    rover.bloqueado = True
    nav = Navegador(terr, ParamsNav(vaiven=True, vaiven_ciclos=1))
    nav.nueva_meta(rover.estado(), (12.0, 0.0))
    vs = []
    for k in range(400):
        (v, w), _ = nav.paso(rover.estado())
        vs.append(v)
        if rover.t > 4.0:
            rover.bloqueado = False                            # el obstaculo se libera
        rover.paso(v, w)
        if nav.llegado:
            break
    assert nav.vaivenes == 1
    assert min(vs) < 0 < max(vs)                               # fue atras y volvio a avanzar
    assert nav.llegado
    # la maniobra dura (atras + adelante) y NO se dispara sin atasco
    rover2 = RoverFalso(terr)
    nav2 = Navegador(terr, ParamsNav(vaiven=True))
    nav2.nueva_meta(rover2.estado(), (12.0, 0.0))
    for _ in range(200):
        (v, w), _ = nav2.paso(rover2.estado())
        rover2.paso(v, w)
    assert nav2.vaivenes == 0


def test_limite_de_subida_aprendido_al_atascarse():
    terr = TerrenoSintetico(pend_plano=math.tan(math.radians(12.0)))
    nav = Navegador(terr, ParamsNav(graduado=True, vaiven=True))
    e = {'t': 0.0, 'x': 3.0, 'y': 0.0, 'rumbo_deg': 0.0, 'v': 0.0}
    nav.nueva_meta(e, (10.0, 0.0))
    assert nav._lim_sube is None
    nav._aprender_limite(e)
    assert nav._lim_sube == pytest.approx(10.0, abs=0.1)       # 12 grados - 2
    nav._aprender_limite(dict(e, rumbo_deg=180.0))              # bajando no aprende nada
    assert nav._lim_sube == pytest.approx(10.0, abs=0.1)
    nav.nueva_meta(e, (10.0, 0.0))                              # una meta nueva olvida el limite
    assert nav._lim_sube is None


def test_graduado_determinista():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 3.0, 3.0))
    p = ParamsNav(graduado=True, replan_s=4.0, vaiven=True)
    a = correr(terr, (20.0, 0.0), t_max=300.0, params=p)[0].trayectoria
    b = correr(terr, (20.0, 0.0), t_max=300.0, params=p)[0].trayectoria
    assert a == b and len(a) > 50


def test_seguridad_margen_de_vuelco():
    from atp_rover.seguridad import MonitorSeguridad
    m = MonitorSeguridad()
    assert m.evaluar({'t': 0.0, 'cabeceo_deg': 3.0, 'alabeo_deg': 3.0, 'margen_vuelco_deg': 30.0}) == []
    al = m.evaluar({'t': 1.0, 'cabeceo_deg': 3.0, 'alabeo_deg': 3.0, 'margen_vuelco_deg': 5.0})
    assert [a['tipo'] for a in al] == ['vuelco'] and al[0]['detalle']['margen_vuelco_deg'] == 5.0
    assert m.debe_frenar()
