"""Lecturas del sistema (carga, memoria, diario del vigilante) con entradas simuladas."""
import subprocess
from types import SimpleNamespace

from station import sistema

MEMINFO = """MemTotal:       11534336 kB
MemFree:          500000 kB
MemAvailable:    4194304 kB
SwapTotal:       2097152 kB
SwapFree:        1048576 kB
"""

JOURNAL = """2026-10-05T12:00:00-0600 host atp-vigilante[812]: activo: load>12.0 o MemAvailable<800 MB durante 3 lecturas
2026-10-05T12:20:05-0600 host atp-vigilante[812]: SOBRECARGA load=13.2 mem=4200MB: deteniendo contenedores ROS
-- No entries --
"""


def test_leer_memoria(tmp_path):
    p = tmp_path / 'meminfo'
    p.write_text(MEMINFO)
    m = sistema.leer_memoria(str(p))
    assert m == {'total_mb': 11264, 'disponible_mb': 4096, 'usada_pct': 63.6,
                 'swap_total_mb': 2048, 'swap_usada_mb': 1024}
    assert sistema.leer_memoria(str(tmp_path / 'no-existe')) == {}


def test_leer_cargas_son_tres_numeros():
    c = sistema.leer_cargas()
    assert len(c) == 3 and all(x >= 0 for x in c)


def fake(rc=0, out='', err=''):
    return lambda cmd, **kw: SimpleNamespace(returncode=rc, stdout=out, stderr=err)


def test_leer_vigilante_parsea_el_diario():
    v = sistema.leer_vigilante(15, fake(0, JOURNAL))
    assert len(v['lineas']) == 2
    assert v['lineas'][0] == {'t': '2026-10-05T12:00:00-0600',
                              'texto': 'activo: load>12.0 o MemAvailable<800 MB durante 3 lecturas'}
    assert v['ultima_sobrecarga']['texto'].startswith('SOBRECARGA load=13.2')


def test_leer_vigilante_sin_entradas_es_lista_vacia():
    assert sistema.leer_vigilante(5, fake(0, '-- No entries --\n')) == {'lineas': [], 'ultima_sobrecarga': None}


def test_leer_vigilante_no_legible_es_none():
    assert sistema.leer_vigilante(5, fake(1, '', 'Failed to open journal')) is None
    assert sistema.leer_vigilante(5, fake(0, 'No journal files were found.\n')) is None

    def sin_binario(cmd, **kw):
        raise FileNotFoundError('journalctl')

    def lento(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, 3)

    assert sistema.leer_vigilante(5, sin_binario) is None
    assert sistema.leer_vigilante(5, lento) is None


def test_vigilancia_cachea_y_tolera_errores():
    llamadas = []

    def lector(n):
        llamadas.append(n)
        return {'lineas': [], 'ultima_sobrecarga': None}

    v = sistema.Vigilancia(7, lector, ttl_s=60)
    assert v.leer() == v.leer() and llamadas == [7]

    def roto(n):
        raise RuntimeError('boom')

    assert sistema.Vigilancia(7, roto).leer() is None
