"""Exportacion MCAP (escritor propio, sin la libreria `mcap`) de la telemetria de una corrida."""
import json
import struct
from datetime import datetime, timezone

import pytest

from conftest import wait_status
from station import mcap


def corrida(tmp_path, tele=3, red=2, eventos=1):
    d = tmp_path / 'run'
    d.mkdir()
    if tele:
        (d / 'telemetry.jsonl').write_text(''.join(json.dumps({'t': i * 0.1, 'az': 10.0 + i}) + '\n' for i in range(tele)))
    if red:
        (d / 'red.jsonl').write_text(''.join(json.dumps({'t': float(i), 'geo': {}, 'antenas': {}}) + '\n' for i in range(red)))
    if eventos:
        (d / 'eventos.jsonl').write_text(json.dumps({'t': 1.5, 'antena': 'a/b', 'tipo': 'perdida'}) + '\n')
    return d


def archivo(d, base=0.0, bloque=1 << 18):
    return b''.join(mcap.generar(d, base, bloque))


def test_estructura_valida_segun_la_especificacion(tmp_path):
    datos = archivo(corrida(tmp_path), base=1000.0)
    assert datos[:8] == b'\x89MCAP0\r\n' and datos[-8:] == b'\x89MCAP0\r\n'
    o = mcap.leer(datos)                                       # lector independiente del escritor
    assert o['libreria'] == 'estacion-atp' and o['perfil'] == ''
    assert o['canales'] == {1: '/telemetria', 2: '/red', 3: '/eventos'}
    assert o['esquemas'] == {1: 'telemetria', 2: 'red', 3: 'eventos'}
    assert len(o['mensajes']) == 3 + 2 + 1
    por_canal = {c: [m for m in o['mensajes'] if m[0] == c] for c in (1, 2, 3)}
    assert [m[3]['az'] for m in por_canal[1]] == [10.0, 11.0, 12.0]
    # log_time = base + t en ns; las secuencias crecen
    assert [m[2] for m in por_canal[1]] == [1000_000_000_000, 1000_100_000_000, 1000_200_000_000]
    assert por_canal[3][0][2] == 1001_500_000_000
    assert [m[1] for m in o['mensajes']] == list(range(1, 7))


def test_registros_de_bajo_nivel_segun_la_especificacion():
    # Header: opcode 0x01, longitud u64, dos strings (u32 + bytes)
    h = mcap.header('', 'x')
    assert h == b'\x01' + struct.pack('<Q', 8 + 1) + struct.pack('<I', 0) + struct.pack('<I', 1) + b'x'
    # Message: opcode 0x05; canal u16, secuencia u32, log_time u64, publish_time u64, datos
    m = mcap.message(7, 3, 99, b'{}')
    assert m[0] == 0x05 and struct.unpack_from('<Q', m, 1)[0] == 22 + 2
    assert struct.unpack_from('<HIQQ', m, 9) == (7, 3, 99, 99) and m[-2:] == b'{}'
    # cierre: DataEnd (0x0F, crc 0), Footer (0x02, 20 bytes) y el magic
    c = mcap.cierre()
    assert c[0] == 0x0F and c[13] == 0x02 and struct.unpack_from('<Q', c, 14)[0] == 20 and c.endswith(mcap.MAGIC)
    # canal: sin metadatos (u32 = 0 al final)
    ch = mcap.channel(1, 1, '/t')
    assert ch[0] == 0x04 and ch[-4:] == b'\x00\x00\x00\x00'


def test_solo_los_archivos_que_existen(tmp_path):
    o = mcap.leer(archivo(corrida(tmp_path, tele=0, red=4, eventos=0)))
    assert o['canales'] == {1: '/red'} and len(o['mensajes']) == 4
    vacio = tmp_path / 'vacia'
    vacio.mkdir()
    assert mcap.archivos_disponibles(vacio) == []
    assert mcap.leer(archivo(vacio))['mensajes'] == []         # sigue siendo un MCAP valido


def test_lineas_a_medio_escribir_se_saltan(tmp_path):
    d = corrida(tmp_path, red=0, eventos=0)
    with open(d / 'telemetry.jsonl', 'a') as f:
        f.write('{"t": 9.0, "az":')                             # corrida en curso: ultima linea cortada
    assert len(mcap.leer(archivo(d))['mensajes']) == 3


def test_se_emite_en_bloques_sin_cargar_todo(tmp_path):
    d = corrida(tmp_path, tele=2000, red=0, eventos=0)
    bloques = list(mcap.generar(d, 0.0, bloque=4096))
    assert len(bloques) > 3 and max(len(b) for b in bloques) < 4096 + 400
    assert len(mcap.leer(b''.join(bloques))['mensajes']) == 2000


@pytest.mark.parametrize('malo', [b'', b'no es mcap', mcap.MAGIC, mcap.MAGIC + mcap.MAGIC])
def test_el_lector_rechaza_basura(malo):
    with pytest.raises(Exception):
        mcap.leer(malo)


def test_el_lector_rechaza_truncados(tmp_path):
    datos = archivo(corrida(tmp_path))
    with pytest.raises(ValueError):
        mcap.leer(datos[:-20] + mcap.MAGIC)


# ------------------------------------------------------------------ por la API
def test_api_export_mcap_de_una_corrida_de_lazo(client):
    rid = client.post('/api/runs', json={'experiment_id': 'n1-rampa'}).json()['run_id']
    r = wait_status(client, rid)
    assert r['status'] == 'done'
    resp = client.get(f'/api/runs/{rid}/export.mcap')
    assert resp.status_code == 200 and resp.headers['content-type'] == 'application/octet-stream'
    assert resp.headers['content-disposition'] == f'attachment; filename="{rid}.mcap"'
    o = mcap.leer(resp.content)
    assert o['canales'] == {1: '/telemetria'} and len(o['mensajes']) == 8
    base = datetime.strptime(r['created_at'], '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()
    assert o['mensajes'][0][2] == int(base * 1e9) and o['mensajes'][3][2] == int((base + 0.3) * 1e9)


def test_api_export_mcap_de_una_corrida_de_red(client_real):
    c = client_real
    rid = c.post('/api/runs', json={'experiment_id': 'prueba-proceso', 'args': {'dur_s': 5}}).json()['run_id']
    wait_status(c, rid)
    o = mcap.leer(c.get(f'/api/runs/{rid}/export.mcap').content)
    assert o['canales'] == {1: '/red', 2: '/eventos'} and len(o['mensajes']) == 6 + 1


def test_api_export_mcap_sin_series_es_404(client):
    rid = client.post('/api/runs', json={'experiment_id': 's6-plan-24h', 'args': {'horas': 6, 'sin': ''}}).json()['run_id']
    wait_status(client, rid)
    assert client.get(f'/api/runs/{rid}/export.mcap').status_code == 404
    assert client.get('/api/runs/malo/export.mcap').status_code == 404
