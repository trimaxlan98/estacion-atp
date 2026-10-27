"""Servir medios con Range (video en el navegador) confinados a runs/<run>/media/<job>/."""
import os

import pytest

from conftest import wait_job, wait_status

VIDEO = bytes((i * 7 + 3) % 256 for i in range(5000))        # lo que escribe tests/fixtures/render_falso.py


@pytest.fixture
def medio(client_real):
    c = client_real
    rid = c.post('/api/runs', json={'experiment_id': 'prueba-proceso', 'args': {'dur_s': 3}}).json()['run_id']
    assert wait_status(c, rid)['status'] == 'done'
    job = c.post(f'/api/runs/{rid}/renders', json={'render_id': 'render-falso'}).json()['job_id']
    assert wait_job(c, job)['estado'] == 'hecho'
    c.job, c.rid, c.url = job, rid, f'/api/medios/{job}/visor.mp4'
    return c


def rng(c, valor, url=None):
    return c.get(url or c.url, headers={'Range': valor})


def test_archivo_completo(medio):
    r = medio.get(medio.url)
    assert r.status_code == 200 and r.content == VIDEO
    assert r.headers['content-type'] == 'video/mp4' and r.headers['accept-ranges'] == 'bytes'
    assert r.headers['content-length'] == '5000' and r.headers['etag']
    assert 'content-range' not in r.headers


@pytest.mark.parametrize('cabecera,ini,fin', [
    ('bytes=0-99', 0, 99),
    ('bytes=100-199', 100, 199),
    ('bytes=4900-', 4900, 4999),            # hasta el final
    ('bytes=-50', 4950, 4999),              # los ultimos 50
    ('bytes=4999-5000', 4999, 4999),        # el fin se recorta al tamano
    ('bytes=0-0', 0, 0),
    ('bytes=0-99999', 0, 4999),
    ('bytes=-99999', 0, 4999),              # un sufijo mayor que el archivo = todo
])
def test_rangos_validos_dan_206(medio, cabecera, ini, fin):
    r = rng(medio, cabecera)
    assert r.status_code == 206
    assert r.headers['content-range'] == f'bytes {ini}-{fin}/5000'
    assert r.headers['content-length'] == str(fin - ini + 1)
    assert r.content == VIDEO[ini:fin + 1]
    assert r.headers['accept-ranges'] == 'bytes' and r.headers['content-type'] == 'video/mp4'


@pytest.mark.parametrize('cabecera', ['bytes=5000-', 'bytes=6000-7000', 'bytes=-0', 'bytes=200-100'])
def test_rangos_insatisfacibles_dan_416(medio, cabecera):
    r = rng(medio, cabecera)
    assert r.status_code == 416 and r.headers['content-range'] == 'bytes */5000'


@pytest.mark.parametrize('cabecera', ['bytes=abc', 'bytes=', 'bytes=-', 'items=0-5', 'bytes=0-1,5-6', '0-5', 'bytes=1-2-3'])
def test_rango_mal_formado_se_ignora(medio, cabecera):
    r = rng(medio, cabecera)
    assert r.status_code == 200 and r.content == VIDEO and 'content-range' not in r.headers


def test_head_y_descarga(medio):
    r = medio.head(medio.url)
    assert r.status_code == 200 and r.content == b'' and r.headers['content-length'] == '5000'
    r = medio.head(medio.url, headers={'Range': 'bytes=10-19'})
    assert r.status_code == 206 and r.headers['content-length'] == '10' and r.content == b''
    r = medio.get(medio.url + '?descargar=true')
    assert r.headers['content-disposition'] == 'attachment; filename="visor.mp4"'


def test_otros_archivos_del_trabajo(medio):
    base = f'/api/medios/{medio.job}'
    assert medio.get(f'{base}/miniatura.jpg').headers['content-type'] == 'image/jpeg'
    m = medio.get(f'{base}/media.json')
    assert m.status_code == 200 and m.json()['job_id'] == medio.job
    assert 'PROGRESO' in medio.get(f'{base}/render.log').text


@pytest.mark.parametrize('ruta', [
    '../params.json', '..%2Fparams.json', '%2e%2e/params.json', '..%2f..%2f..%2fdata%2fstation.db',
    '/etc/passwd', '%2Fetc%2Fpasswd', 'a/../../params.json', 'visor.mp4/', 'no-existe.mp4',
    'visor.mp4%00.jpg', '.', '', '..\\params.json'])
def test_rutas_hostiles_son_404(medio, ruta):
    r = medio.get(f'/api/medios/{medio.job}/{ruta}')
    assert r.status_code in (404, 405), ruta
    assert b'secreto' not in r.content


def test_jobs_invalidos_o_ajenos(medio):
    for job in ('malo', '../x', 'r20200101-000000-abcdef', 'R20261005-120000-abc123'):
        assert medio.get(f'/api/medios/{job}/visor.mp4').status_code == 404
    # el id de una CORRIDA no es un trabajo
    assert medio.get(f'/api/medios/{medio.rid}/params.json').status_code == 404


def test_enlace_simbolico_que_sale_de_la_carpeta(medio):
    d = medio.settings.runs_dir / medio.rid / 'media' / medio.job
    (medio.settings.runs_dir / medio.rid / 'secreto.txt').write_text('secreto')
    os.symlink('../../secreto.txt', d / 'enlace.txt')
    os.symlink(os.path.abspath(medio.settings.db_path), d / 'db.bin')
    for n in ('enlace.txt', 'db.bin'):
        r = medio.get(f'/api/medios/{medio.job}/{n}')
        assert r.status_code == 404 and b'secreto' not in r.content


def test_directorio_no_se_sirve(medio):
    d = medio.settings.runs_dir / medio.rid / 'media' / medio.job / 'sub'
    d.mkdir()
    (d / 'x.txt').write_text('x')
    assert medio.get(f'/api/medios/{medio.job}/sub').status_code == 404
    assert medio.get(f'/api/medios/{medio.job}/sub/x.txt').text == 'x'      # los subdirectorios del trabajo si


def test_unidad_parse_rango():
    from fastapi import HTTPException
    from station.medios import parse_rango
    assert parse_rango(None, 10) is None and parse_rango('', 10) is None
    assert parse_rango('bytes=2-4', 10) == (2, 4) and parse_rango(' bytes=2-4 ', 10) == (2, 4)
    with pytest.raises(HTTPException) as ex:
        parse_rango('bytes=10-', 10)
    assert ex.value.status_code == 416
