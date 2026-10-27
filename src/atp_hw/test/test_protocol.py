"""Parseo y formato del protocolo rotctld (logica pura)."""
import pytest

from atp_hw import protocol as pr


def test_format_set_position():
    assert pr.format_set_position(123.456789, 45.0) == 'P 123.4568 45.0000\n'
    assert pr.format_set_position(-10.5, 0.0) == 'P -10.5000 0.0000\n'


def test_format_rechaza_no_finito():
    with pytest.raises(ValueError):
        pr.format_set_position(float('nan'), 10.0)
    with pytest.raises(ValueError):
        pr.format_set_position(10.0, float('inf'))


def test_parse_rprt():
    assert pr.parse_rprt('RPRT 0\n') == 0
    assert pr.parse_rprt('RPRT -1') == -1
    assert pr.parse_rprt('123.4') is None
    with pytest.raises(pr.RotctldProtocolError):
        pr.parse_rprt('RPRT')
    with pytest.raises(pr.RotctldProtocolError):
        pr.parse_rprt('RPRT xx')


def test_check_rprt():
    pr.check_rprt('RPRT 0')
    with pytest.raises(pr.RotctldRPRT) as e:
        pr.check_rprt('RPRT -1')
    assert e.value.code == -1
    assert 'EINVAL' in str(e.value)
    with pytest.raises(pr.RotctldProtocolError):
        pr.check_rprt('hola')


def test_rprt_codigo_desconocido():
    with pytest.raises(pr.RotctldRPRT) as e:
        pr.check_rprt('RPRT -99')
    assert 'desconocido' in str(e.value)


def test_parse_position():
    assert pr.parse_position(['143.27', '61.5']) == (143.27, 61.5)
    assert pr.parse_position(['-5.000000\n', '0.0\n']) == (-5.0, 0.0)


def test_parse_position_errores():
    with pytest.raises(pr.RotctldRPRT):
        pr.parse_position(['RPRT -5'])
    with pytest.raises(pr.RotctldProtocolError):
        pr.parse_position(['10.0'])
    with pytest.raises(pr.RotctldProtocolError):
        pr.parse_position(['abc', '10.0'])
    with pytest.raises(pr.RotctldProtocolError):
        pr.parse_position(['nan', '10.0'])


def test_parse_dump_state_hamlib_4_5():
    lines = ['1', '1', 'min_az=-180.000000', 'max_az=450.000000',
             'min_el=0.000000', 'max_el=90.000000', 'south_zero=0',
             'rot_type=AzEl']
    st = pr.parse_dump_state(lines)
    assert st['min_az'] == -180.0 and st['max_az'] == 450.0
    assert st['max_el'] == 90.0
    assert st['rot_type'] == 'AzEl'
