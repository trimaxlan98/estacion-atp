"""Conversion de acimut continuo, flip, banda muerta, tasa maxima."""
import pytest

from atp_hw import rotor_logic as rl


def test_wrap180():
    assert rl.wrap180(190.0) == -170.0
    assert rl.wrap180(-190.0) == 170.0
    assert rl.wrap180(360.0) == 0.0
    assert rl.wrap180(179.9) == pytest.approx(179.9)


def test_candidatos_rango_360():
    # 0..360: un solo valor valido (salvo en los extremos exactos)
    assert rl.azimuth_candidates(140.0, 0.0, 360.0) == [140.0]
    assert rl.azimuth_candidates(500.0, 0.0, 360.0) == [140.0]
    assert rl.azimuth_candidates(-20.0, 0.0, 360.0) == [340.0]


def test_candidatos_con_solape():
    # 0..450 (G-5500): entre 0 y 90 hay dos valores (a y a+360)
    assert rl.azimuth_candidates(30.0, 0.0, 450.0) == [30.0, 390.0]
    assert rl.azimuth_candidates(200.0, 0.0, 450.0) == [200.0]
    # -180..540 (720 deg de recorrido): dos valores
    assert rl.azimuth_candidates(10.0, -180.0, 540.0) == [10.0, 370.0]
    assert rl.azimuth_candidates(-170.0, -180.0, 540.0) == [-170.0, 190.0]


def test_primera_orden_prefiere_bajo_o_alto():
    assert rl.map_azimuth(30.0, None, 0.0, 450.0, 'low') == (30.0, 'ok')
    assert rl.map_azimuth(30.0, None, 0.0, 450.0, 'high') == (390.0, 'ok')


def test_continuidad_pasando_el_norte_con_solape():
    """Pase que cruza el norte (azimut continuo 350 -> 370) con rango 0..450:
    NO hay flip, la orden sigue creciendo hasta 370 > 360."""
    az_min, az_max = 0.0, 450.0
    last = None
    cmds = []
    for az_c in (350.0, 355.0, 359.9, 360.1, 365.0, 370.0):
        cmd, st = rl.map_azimuth(az_c, last, az_min, az_max)
        assert st == 'ok'
        assert not rl.is_flip(cmd, last)
        cmds.append(cmd)
        last = cmd
    assert cmds == [350.0, 355.0, 359.9, 360.1, 365.0, 370.0]


def test_flip_con_rango_360():
    """Con 0..360 el mismo pase salta de 359.9 a 0.1: flip de 359.8 deg."""
    last = None
    flips = 0
    for az_c in (350.0, 359.9, 360.1, 365.0):
        cmd, _ = rl.map_azimuth(az_c, last, 0.0, 360.0)
        flips += rl.is_flip(cmd, last)
        last = cmd
    assert flips == 1
    assert last == pytest.approx(5.0)


def test_flip_al_salir_por_el_extremo_del_solape():
    """Con 0..450, si la orden ya va por la parte alta (primera orden 'high'),
    el flip llega cuando la referencia pasa de 450."""
    last = None
    seq = []
    for az_c in (440.0, 449.0, 451.0):
        cmd, _ = rl.map_azimuth(az_c, last, 0.0, 450.0, 'high')
        seq.append((cmd, rl.is_flip(cmd, last)))
        last = cmd
    assert seq == [(440.0, False), (449.0, False), (91.0, True)]


def test_primera_orden_cerca_del_limite_provoca_flip_si_se_viene_de_alto():
    # rotor parado en 440 (ancla), el objetivo continuo 451 solo cabe como 91
    cmd, _ = rl.map_azimuth(451.0, 440.0, 0.0, 450.0)
    assert cmd == 91.0 and rl.is_flip(cmd, 440.0)


def test_rango_menor_de_360_recorta_al_limite_cercano():
    assert rl.map_azimuth(320.0, None, 0.0, 300.0) == (300.0, 'clamped')
    assert rl.map_azimuth(350.0, None, 0.0, 300.0) == (0.0, 'clamped')
    cmd, st = rl.map_azimuth(5.0, None, 20.0, 340.0)
    assert (cmd, st) == (20.0, 'clamped')


def test_map_azimuth_rango_invalido():
    with pytest.raises(ValueError):
        rl.map_azimuth(10.0, None, 100.0, 100.0)


def test_clamp_elevacion():
    assert rl.clamp_elevation(45.0, 0.0, 90.0) == (45.0, False)
    assert rl.clamp_elevation(-3.0, 0.0, 90.0) == (0.0, True)
    assert rl.clamp_elevation(95.0, 0.0, 90.0) == (90.0, True)


def test_banda_muerta():
    last = (100.0, 30.0)
    assert not rl.needs_update((100.05, 30.05), last, 0.1, 0.1)
    assert rl.needs_update((100.1, 30.0), last, 0.1, 0.1)      # justo la resolucion
    assert rl.needs_update((100.0, 29.8), last, 0.1, 0.1)
    assert not rl.needs_update((100.4, 30.4), last, 1.0, 1.0)  # rotor de 1 deg
    assert rl.needs_update((100.0, 30.0), None, 1.0, 1.0)      # primera orden


def test_meas_en_marco_continuo():
    # referencia 365 (continua), rotor reporta 5: el error fisico es 0
    assert rl.meas_in_ref_frame(5.0, 365.0) == pytest.approx(365.0)
    # referencia 358, rotor 2 (cruzo el norte antes): 362, error -4
    assert rl.meas_in_ref_frame(2.0, 358.0) == pytest.approx(362.0)
    # rotor en marco -180..540 reportando 190 con referencia 190
    assert rl.meas_in_ref_frame(190.0, 190.0) == pytest.approx(190.0)
    # error pequeno se conserva con signo
    assert 140.0 - rl.meas_in_ref_frame(139.5, 140.0) == pytest.approx(0.5)


def test_limitador_de_tasa():
    lim = rl.RateLimiter(5.0)           # 0.2 s
    assert lim.ready(0.0)
    lim.mark(0.0)
    assert not lim.ready(0.1)
    assert not lim.ready(0.199)
    assert lim.ready(0.2)
    lim.mark(0.2)
    assert not lim.ready(0.3)
    assert rl.RateLimiter(0).ready(1e-9)   # 0 = sin limite
