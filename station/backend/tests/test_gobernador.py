"""Gobernador de recursos (docs/APP.md 3) con la carga del host simulada: sin procesos ni red."""
import asyncio
import signal

import pytest

from station.config import CLASES, Settings, es_final
from station.governor import Governor, muerte_por_senal


@pytest.fixture
def s(monkeypatch):
    for k in list(__import__('os').environ):
        if k.startswith('STATION_'):
            monkeypatch.delenv(k)
    monkeypatch.setenv('STATION_RECHEQUEO_S', '0.02')
    st = Settings()
    st.max_concurrent = 10            # el tope global se prueba aparte
    return st


def correr(coro):
    return asyncio.run(coro)


def test_valores_por_defecto_de_la_tabla_de_la_app(monkeypatch):
    for k in list(__import__('os').environ):
        if k.startswith('STATION_'):
            monkeypatch.delenv(k)
    st = Settings()
    assert st.cupos == {'ligero': 2, 'pesado': 1, 'gazebo': 1, 'manim': 1}
    assert st.umbrales == {'ligero': 10.0, 'pesado': 6.0, 'gazebo': 4.0, 'manim': 5.0}
    assert st.recheck_s == 10 and st.reintentos == 2
    assert (st.pantalla_min, st.pantalla_max) == (80, 83)


def test_cupos_y_umbrales_por_entorno(monkeypatch):
    monkeypatch.setenv('STATION_CUPO_GAZEBO', '3')
    monkeypatch.setenv('STATION_UMBRAL_MANIM', '7.5')
    monkeypatch.setenv('STATION_REINTENTOS', '5')
    monkeypatch.setenv('STATION_PANTALLA_MAX', '85')
    st = Settings()
    assert st.cupos['gazebo'] == 3 and st.umbrales['manim'] == 7.5 and st.reintentos == 5
    assert st.pantalla_max == 85


def test_cupo_y_orden_de_llegada(s):
    async def main():
        g = Governor(s, lambda: 0.0)
        a = await g.acquire('pesado')
        orden = []

        async def pide(n):
            lease = await g.acquire('pesado')
            orden.append(n)
            return lease

        t1, t2 = asyncio.create_task(pide(1)), asyncio.create_task(pide(2))
        await asyncio.sleep(0.15)
        assert orden == []                                    # cupo 1: esperan
        sn = g.snapshot()['clases']['pesado']
        assert (sn['corriendo'], sn['en_cola'], sn['esperando_carga']) == (1, 2, 0)
        g.release(a)
        l1 = await asyncio.wait_for(t1, 1)
        await asyncio.sleep(0.1)
        assert orden == [1]                                   # el 2 sigue esperando
        g.release(l1)
        l2 = await asyncio.wait_for(t2, 1)
        assert orden == [1, 2]
        g.release(l2)
        g.release(l2)                                         # devolver dos veces no descuenta dos
        assert g.corriendo['pesado'] == 0
    correr(main())


def test_clases_independientes(s):
    async def main():
        g = Governor(s, lambda: 0.0)
        ls = [await asyncio.wait_for(g.acquire(c), 1) for c in CLASES]     # una de cada una
        l2 = await asyncio.wait_for(g.acquire('ligero'), 1)                 # ligero admite 2
        assert g.total() == 5
        t = asyncio.create_task(g.acquire('ligero'))
        await asyncio.sleep(0.1)
        assert not t.done()                                                 # el tercer ligero espera
        for lease in ls + [l2]:
            g.release(lease)
        await asyncio.wait_for(t, 1)
    correr(main())


def test_tope_global(s):
    s.max_concurrent = 2

    async def main():
        g = Governor(s, lambda: 0.0)
        a = await g.acquire('ligero')
        b = await g.acquire('manim')
        t = asyncio.create_task(g.acquire('gazebo'))                       # hay cupo de clase, no de tope
        await asyncio.sleep(0.1)
        assert not t.done() and g.snapshot()['esperando'][0]['motivo'] == 'cola'
        g.release(a)
        await asyncio.wait_for(t, 1)
        g.release(b)
    correr(main())


def test_espera_por_carga_y_se_libera_al_bajar(s):
    carga = {'v': 9.0}

    async def main():
        g = Governor(s, lambda: carga['v'])
        estados = []
        t = asyncio.create_task(g.acquire('pesado', on_state=estados.append, label='corrida:x'))
        await asyncio.sleep(0.15)
        assert not t.done() and estados == ['carga']                        # avisa una sola vez
        sn = g.snapshot()
        assert sn['clases']['pesado']['esperando_carga'] == 1 and sn['clases']['pesado']['en_cola'] == 0
        assert sn['esperando'][0]['tarea'] == 'corrida:x' and sn['esperando'][0]['motivo'] == 'carga'
        carga['v'] = 5.9
        lease = await asyncio.wait_for(t, 1)
        assert lease.clase == 'pesado' and g.snapshot()['corriendo'][0]['tarea'] == 'corrida:x'
        g.release(lease)
    correr(main())


@pytest.mark.parametrize('carga,admitidas', [
    (0.0, {'ligero', 'pesado', 'gazebo', 'manim'}),
    (3.99, {'ligero', 'pesado', 'gazebo', 'manim'}),
    (4.0, {'ligero', 'pesado', 'manim'}),                 # el umbral es estricto: "< 4"
    (4.5, {'ligero', 'pesado', 'manim'}),
    (5.0, {'ligero', 'pesado'}),
    (6.0, {'ligero'}),
    (9.99, {'ligero'}),
    (10.0, set()),
])
def test_umbral_de_carga_por_clase(s, carga, admitidas):
    async def main():
        g = Governor(s, lambda: carga)
        hechas = set()
        for c in CLASES:
            t = asyncio.create_task(g.acquire(c))
            await asyncio.sleep(0.05)
            if t.done():
                hechas.add(c)
                g.release(t.result())
            else:
                t.cancel()
        assert hechas == admitidas
    correr(main())


def test_si_la_lectura_de_carga_falla_no_se_bloquea(s):
    def mala():
        raise OSError('sin /proc')

    async def main():
        g = Governor(s, mala)
        g.release(await asyncio.wait_for(g.acquire('pesado'), 1))
    correr(main())


def test_pantallas_del_pool(s):
    s.cupos['gazebo'] = 5

    async def main():
        g = Governor(s, lambda: 0.0)
        ls = [await asyncio.wait_for(g.acquire('gazebo', pantalla=True), 1) for _ in range(4)]
        assert [x.pantalla for x in ls] == [':80', ':81', ':82', ':83']
        t = asyncio.create_task(g.acquire('gazebo', pantalla=True))        # sin pantallas libres
        await asyncio.sleep(0.1)
        assert not t.done() and g.snapshot()['pantallas']['libres'] == []
        sin = await asyncio.wait_for(g.acquire('gazebo'), 1)                # sin pantalla no la necesita
        assert sin.pantalla is None
        g.release(ls[1])                                                    # se libera :81
        nueva = await asyncio.wait_for(t, 1)
        assert nueva.pantalla == ':81'
        for x in (ls[0], ls[2], ls[3], nueva, sin):
            g.release(x)
        assert g.snapshot()['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    correr(main())


def test_cancelar_mientras_espera_no_deja_nada(s):
    async def main():
        g = Governor(s, lambda: 9.0)
        t = asyncio.create_task(g.acquire('pesado'))
        await asyncio.sleep(0.05)
        t.cancel()
        with pytest.raises(asyncio.CancelledError):
            await t
        assert g.waiting == [] and g.leases == [] and g.total() == 0
    correr(main())


def test_cancelar_justo_despues_de_concederse_devuelve_el_turno(s):
    async def main():
        g = Governor(s, lambda: 0.0)
        a = await g.acquire('pesado')
        t = asyncio.create_task(g.acquire('pesado'))
        await asyncio.sleep(0.05)
        g.release(a)            # concede el turno a `t` (su futuro ya tiene el resultado)...
        t.cancel()              # ...pero la tarea se cancela antes de reanudarse
        with pytest.raises(asyncio.CancelledError):
            await t
        assert g.total() == 0 and g.leases == []
    correr(main())


def test_dominios_ros_unicos(s):
    g = Governor(s, lambda: 0.0)
    ds = {g.alloc_domain() for _ in range(100)}
    assert len(ds) == 100 and all(100 <= d <= 199 for d in ds)
    with pytest.raises(RuntimeError):
        g.alloc_domain()
    g.free_domain(next(iter(ds)))
    assert 100 <= g.alloc_domain() <= 199


def test_snapshot_estructura(s):
    sn = Governor(s, lambda: 1.5).snapshot()
    assert set(sn['clases']) == set(CLASES)
    assert sn['clases']['pesado'] == {'cupo': 1, 'corriendo': 0,
                                      'umbral_carga': 6.0, 'en_cola': 0, 'esperando_carga': 0,
                                      'admite_ahora': True}
    assert sn['pantallas']['pool'] == [':80', ':81', ':82', ':83'] and sn['tope_global'] == 10


@pytest.mark.parametrize('rc,esperado', [
    (-signal.SIGKILL, True), (-signal.SIGTERM, True), (-signal.SIGHUP, True),
    (137, True), (143, True), (129, True),
    (0, False), (1, False), (2, False), (3, False), (130, False), (None, False)])
def test_muerte_por_senal(rc, esperado):
    assert muerte_por_senal(rc) is esperado


def test_estados_finales():
    for e in ('done', 'error', 'cancelled', 'timeout', 'hecho', 'cancelado'):
        assert es_final(e)
    for e in ('queued', 'running', 'esperando_carga', 'en_cola', 'corriendo'):
        assert not es_final(e)
    assert not es_final('interrumpida') and not es_final('interrumpida', None)    # espera el reintento
    assert es_final('interrumpida', '2026-10-05T12:00:00Z')                       # agoto los reintentos


# ------------------------------------------------------------- try_acquire (trabajador del host)
def test_try_acquire_respeta_cupo_carga_y_pantallas(s):
    carga = {'v': 0.0}
    g = Governor(s, lambda: carga['v'])
    a, motivo = g.try_acquire('gazebo', pantalla=True, label='render:a')
    assert motivo is None and a.pantalla == ':80' and g.corriendo['gazebo'] == 1
    assert g.try_acquire('gazebo') == (None, 'cola')                       # cupo de gazebo = 1
    carga['v'] = 6.0
    assert g.try_acquire('pesado') == (None, 'carga')                       # pesado admite < 6
    ligero, motivo = g.try_acquire('ligero')
    assert motivo is None
    g.release(ligero)
    carga['v'] = 11.0
    assert g.try_acquire('ligero') == (None, 'carga')
    g.release(a)
    assert g.snapshot()['pantallas']['libres'] == [':80', ':81', ':82', ':83']
    with pytest.raises(ValueError):
        g.try_acquire('enorme')


def test_try_acquire_no_se_salta_a_las_tareas_que_esperan(s):
    async def main():
        g = Governor(s, lambda: 0.0)
        a = await g.acquire('pesado')
        t = asyncio.create_task(g.acquire('pesado'))                        # espera por cupo
        await asyncio.sleep(0.05)
        assert g.try_acquire('pesado') == (None, 'cola')
        assert g.try_acquire('ligero')[1] is None                           # otra clase si pasa
        g.release(a)
        g.release(await asyncio.wait_for(t, 1))
        lease, motivo = g.try_acquire('pesado')
        assert motivo is None
        g.release(lease)
    correr(main())
