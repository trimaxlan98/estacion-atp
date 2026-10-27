"""Pruebas de R2: navegacion, traccion y seguridad contra un rover cinematico falso (unicycle) y un terreno sintetico.
No usan el terreno ni el rover de R1: solo el contrato (duck-type altura/normal y el JSON de estado)."""
import math

from atp_rover.navegacion import (ControlTraccion, Navegador, ParamsNav, PurePursuit, limite_velocidad,
                                  pendiente_deg, planificar_ruta, rugosidad)
from atp_rover.seguridad import MonitorSeguridad, ParamsSeguridad


class TerrenoSintetico:
    """Plano + colina gaussiana (opcional) + ondulacion sinusoidal (opcional)."""

    def __init__(self, colina=None, rizo=0.0, pend_plano=0.0):
        self.colina = colina        # (cx, cy, altura, sigma)
        self.rizo = rizo            # amplitud (m) de sin(2x)*sin(2y)
        self.pend_plano = pend_plano  # dz/dx

    def altura(self, x, y):
        z = self.pend_plano * x + self.rizo * math.sin(2 * x) * math.sin(2 * y)
        if self.colina:
            cx, cy, h, s = self.colina
            z += h * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s))
        return z

    def _grad(self, x, y):
        e = 1e-4
        return ((self.altura(x + e, y) - self.altura(x - e, y)) / (2 * e),
                (self.altura(x, y + e) - self.altura(x, y - e)) / (2 * e))

    def normal(self, x, y):
        gx, gy = self._grad(x, y)
        n = math.sqrt(gx * gx + gy * gy + 1.0)
        return (-gx / n, -gy / n, 1.0 / n)


class RoverFalso:
    """Unicycle a dt fijo con la interfaz de estado del CONTRATO sec. 18 (los campos que usa R2)."""

    def __init__(self, terr, x=0.0, y=0.0, rumbo_deg=0.0, dt=0.1):
        self.terr, self.dt = terr, dt
        self.x, self.y, self.rumbo = x, y, math.radians(rumbo_deg)
        self.t, self.v, self.w, self.e = 0.0, 0.0, 0.0, 0.0
        self.desliz = [0.0] * 6
        self.bloqueado = False
        self.trayectoria = []

    def estado(self):
        nx, ny, nz = self.terr.normal(self.x, self.y)
        c, s = math.cos(self.rumbo), math.sin(self.rumbo)
        cab = math.degrees(math.atan2(-(nx * c + ny * s), nz))
        ala = math.degrees(math.atan2(nx * s - ny * c, nz))
        return {'t': round(self.t, 6), 'x': self.x, 'y': self.y, 'z': self.terr.altura(self.x, self.y),
                'rumbo_deg': math.degrees(self.rumbo), 'cabeceo_deg': cab, 'alabeo_deg': ala,
                'v': self.v, 'w': self.w, 'pendiente_deg': pendiente_deg(self.terr, self.x, self.y),
                'deslizamiento': list(self.desliz), 'energia_wh': self.e, 'atascado': False}

    def paso(self, v, w):
        v = 0.0 if self.bloqueado else v
        self.v, self.w = v, w
        self.rumbo += w * self.dt
        self.x += v * math.cos(self.rumbo) * self.dt
        self.y += v * math.sin(self.rumbo) * self.dt
        self.e += (5.0 + 20.0 * abs(v)) * self.dt / 3600.0
        self.t += self.dt
        self.trayectoria.append((self.x, self.y))


def correr(terr, meta, t_max=200.0, params=None, rover=None, nav=None):
    rover = rover or RoverFalso(terr)
    nav = nav or Navegador(terr, params)
    nav.nueva_meta(rover.estado(), meta)
    alarmas = []
    while rover.t < t_max and not nav.llegado:
        (v, w), al = nav.paso(rover.estado())
        alarmas += al
        rover.paso(v, w)
    return rover, nav, alarmas


def test_alcanza_meta_llano():
    rover, nav, al = correr(TerrenoSintetico(), (12.0, 5.0))
    assert nav.llegado and not al
    assert math.hypot(rover.x - 12.0, rover.y - 5.0) <= 0.4


def test_meta_detras_gira_y_llega():
    rover = RoverFalso(TerrenoSintetico(), rumbo_deg=180.0)
    rover, nav, _ = correr(TerrenoSintetico(), (6.0, 0.0), rover=rover)
    assert nav.llegado


def test_sigue_varias_metas_en_secuencia():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    for meta in [(5, 0), (5, 5), (0, 5)]:
        rover, nav, _ = correr(terr, meta, rover=rover)
        assert nav.llegado
    assert math.hypot(rover.x, rover.y - 5.0) <= 0.4


def test_rodea_pendiente_prohibida():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 6.0, 2.0))
    p = ParamsNav()
    ruta = planificar_ruta(terr, (0.0, 0.0), (20.0, 0.0), p)
    assert ruta is not None and len(ruta) > 2          # no es recta
    rover, nav, al = correr(terr, (20.0, 0.0), t_max=300.0)
    assert nav.llegado
    # nunca pisa pendiente por encima del limite
    assert max(pendiente_deg(terr, x, y) for x, y in rover.trayectoria) < p.pend_max_deg
    # y efectivamente se desvio de la recta
    assert max(abs(y) for _, y in rover.trayectoria) > 2.0
    assert not [a for a in al if a['tipo'] in ('vuelco', 'pendiente')]


def test_sin_ruta_si_meta_intransitable():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 6.0, 2.0))
    assert planificar_ruta(terr, (0.0, 0.0), (10.0, 1.5)) is None
    nav = Navegador(terr)
    nav.nueva_meta({'x': 0.0, 'y': 0.0}, (10.0, 1.5))
    assert nav.sin_ruta
    (v, w), _ = nav.paso({'t': 0.0, 'x': 0.0, 'y': 0.0, 'rumbo_deg': 0.0, 'v': 0.0})
    assert (v, w) == (0.0, 0.0)


def test_velocidad_baja_con_pendiente_y_rugosidad():
    p = ParamsNav()
    v_llano = limite_velocidad(TerrenoSintetico(), 0, 0, p)
    v_cuesta = limite_velocidad(TerrenoSintetico(pend_plano=math.tan(math.radians(12))), 0, 0, p)
    v_roca = limite_velocidad(TerrenoSintetico(rizo=0.08), 0.39, 0.39, p)
    assert v_llano == p.v_max
    assert p.v_min <= v_cuesta < v_llano
    assert p.v_min <= v_roca < v_llano
    # rampa lisa: sin rugosidad aunque haya pendiente
    assert rugosidad(TerrenoSintetico(pend_plano=0.3), 1.0, 1.0) < 1e-9


def test_pure_pursuit_no_excede_limites():
    pp = PurePursuit([(0, 0), (10, 0)])
    v, w = pp.comando({'x': 0.0, 'y': 3.0, 'rumbo_deg': 90.0})
    assert 0 <= v <= 1.0 and abs(w) <= 0.8


def test_traccion_reduce_par_con_deslizamiento_y_recupera():
    ct = ControlTraccion()
    assert ct.actualizar([0.0, 0.1, 0.05, 0, 0, 0]) == 1.0
    f = ct.actualizar([0.0, 0.0, 0.7, 0.0, 0.0, 0.0])
    assert f < 1.0
    pares = ct.pares([10.0] * 6, [0.0, 0.0, 0.7, 0.0, 0.0, 0.0])
    assert pares[2] < pares[0] <= 10.0
    for _ in range(40):
        f = ct.actualizar([0.0] * 6)
    assert f == 1.0
    assert ct.p.factor_min <= ControlTraccion().actualizar([0.99] * 6)


def test_navegador_modera_velocidad_si_patina():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    rover.desliz = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
    nav = Navegador(terr)
    nav.nueva_meta(rover.estado(), (20.0, 0.0))
    (v_pat, _), _ = nav.paso(rover.estado())
    nav2 = Navegador(terr)
    nav2.nueva_meta(rover.estado(), (20.0, 0.0))
    rover.desliz = [0.0] * 6
    (v_ok, _), _ = nav2.paso(rover.estado())
    assert v_pat < v_ok


def test_alarma_vuelco_frena():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    nav = Navegador(terr)
    nav.nueva_meta(rover.estado(), (30.0, 0.0))
    for _ in range(20):
        (v, w), _ = nav.paso(rover.estado())
        rover.paso(v, w)
    assert v > 0
    e = rover.estado()
    e['alabeo_deg'] = 50.0
    (v, w), al = nav.paso(e)
    assert (v, w) == (0.0, 0.0)
    assert [a['tipo'] for a in al] == ['vuelco']
    assert al[0]['t'] == e['t'] and 'detalle' in al[0]
    # se repite solo cada repetir_s
    (_, _), al = nav.paso(dict(e, t=e['t'] + 1.0))
    assert al == []
    (_, _), al = nav.paso(dict(e, t=e['t'] + 6.0))
    assert len(al) == 1
    # al recuperarse reanuda
    e2 = rover.estado()
    (v, _), al = nav.paso(e2)
    assert v > 0 and not nav.seg.activas


def test_alarma_pendiente_y_energia():
    m = MonitorSeguridad(ParamsSeguridad(pendiente_deg=25.0, energia_max_wh=10.0))
    al = m.evaluar({'t': 1.0, 'pendiente_deg': 30.0, 'energia_wh': 11.0})
    assert {a['tipo'] for a in al} == {'pendiente', 'energia'}
    assert m.debe_frenar()
    assert m.evaluar({'t': 2.0, 'pendiente_deg': 5.0, 'energia_wh': 1.0}) == []
    assert not m.debe_frenar()


def test_alarma_atasco():
    terr = TerrenoSintetico()
    rover = RoverFalso(terr)
    rover.bloqueado = True
    nav = Navegador(terr)
    nav.nueva_meta(rover.estado(), (30.0, 0.0))
    tipos = []
    for _ in range(80):                     # 8 s simulados
        (v, w), al = nav.paso(rover.estado())
        tipos += [(a['tipo'], a['t']) for a in al]
        rover.paso(v, w)
    assert tipos and tipos[0][0] == 'atasco'
    assert 2.9 <= tipos[0][1] <= 3.6        # tras ~atasco_s
    # sin orden no hay atasco
    m = MonitorSeguridad()
    assert all(m.evaluar({'t': k * 0.1, 'v': 0.0}, 0.0) == [] for k in range(100))


def test_determinista():
    terr = TerrenoSintetico(colina=(10.0, 0.0, 6.0, 2.0))
    a = correr(terr, (20.0, 0.0), t_max=300.0)[0].trayectoria
    b = correr(terr, (20.0, 0.0), t_max=300.0)[0].trayectoria
    assert a == b
