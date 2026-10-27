"""Cliente rotctld + rotor falso + nucleo del puente, sin ROS (puertos efimeros)."""
import socket
import threading
import time

import pytest

from atp_hw import protocol as pr
from atp_hw.bridge_core import BridgeConfig, BridgeCore
from atp_hw.fake_rotctld import FakeRotor, handle_line, serve_in_thread
from atp_hw.rotctl_client import RotctldClient


class Reloj:
    """Reloj manual para la dinamica del rotor falso."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


# ---------------------------------------------------------------- dinamica
def test_rotor_falso_velocidad_maxima_por_eje():
    clk = Reloj()
    r = FakeRotor(0.0, 0.0, az_speed=6.0, el_speed=3.0, clock=clk)
    assert r.set_target(90.0, 45.0)
    clk.t = 5.0
    assert r.position() == pytest.approx((30.0, 15.0))
    clk.t = 100.0                       # llega y se queda
    assert r.position() == pytest.approx((90.0, 45.0))
    assert not r.moving()


def test_rotor_falso_retarget_y_stop():
    clk = Reloj()
    r = FakeRotor(0.0, 0.0, 6.0, 3.0, clock=clk)
    r.set_target(100.0, 0.0)
    clk.t = 2.0
    r.set_target(0.0, 0.0)              # da la vuelta a mitad de camino
    clk.t = 3.0
    assert r.position()[0] == pytest.approx(12.0 - 6.0)
    r.stop()
    clk.t = 10.0
    assert r.position()[0] == pytest.approx(6.0)


def test_rotor_falso_fuera_de_rango_y_cuantizacion():
    clk = Reloj()
    r = FakeRotor(10.2, 20.4, az_res=1.0, el_res=1.0, az_range=(0, 360),
                  el_range=(0, 90), clock=clk)
    assert not r.set_target(361.0, 10.0)
    assert not r.set_target(10.0, 91.0)
    assert r.n_rejected == 2
    assert r.position() == (10.0, 20.0)  # GS-232: enteros


def test_handle_line_protocolo():
    r = FakeRotor(clock=Reloj())
    assert handle_line(r, 'P 10 20\n') == ('RPRT 0\n', False)
    assert handle_line(r, 'P 10\n') == ('RPRT -1\n', False)
    assert handle_line(r, 'P abc 1\n') == ('RPRT -1\n', False)
    assert handle_line(r, 'P 500 20\n') == ('RPRT -1\n', False)
    assert handle_line(r, 'p\n')[0] == '0.000000\n0.000000\n'
    assert handle_line(r, 'S\n') == ('RPRT 0\n', False)
    assert handle_line(r, '\\set_pos 5 5\n') == ('RPRT 0\n', False)
    assert handle_line(r, 'zzz\n') == ('RPRT -11\n', False)
    assert handle_line(r, 'q\n') == ('', True)
    assert 'min_az=0.000000' in handle_line(r, '\\dump_state\n')[0]


# ----------------------------------------------------------------- cliente
@pytest.fixture
def servidor():
    rotor = FakeRotor(0.0, 0.0, 600.0, 300.0)   # rapido: no esperar en pruebas
    srv = serve_in_thread(rotor)
    yield rotor, srv.server_address[1]
    srv.shutdown()
    srv.server_close()


def test_cliente_set_get_stop_limites(servidor):
    rotor, port = servidor
    c = RotctldClient('127.0.0.1', port, timeout=1.0)
    c.connect()
    assert c.get_limits() == {'min_az': 0.0, 'max_az': 360.0,
                              'min_el': 0.0, 'max_el': 90.0}
    c.set_position(100.0, 45.0)
    time.sleep(0.5)                              # 600 deg/s: llega rapido
    az, el = c.get_position()
    assert (az, el) == pytest.approx((100.0, 45.0), abs=1e-3)
    with pytest.raises(pr.RotctldRPRT) as e:
        c.set_position(400.0, 10.0)
    assert e.value.code == -1
    assert c.connected                           # un RPRT no cierra el socket
    c.stop()
    c.close()


def test_cliente_timeout_cierra_y_resincroniza():
    """Un servidor mudo: el cliente da timeout, cierra, y reconecta limpio."""
    ls = socket.socket()
    ls.bind(('127.0.0.1', 0))
    ls.listen(1)
    port = ls.getsockname()[1]
    c = RotctldClient('127.0.0.1', port, timeout=0.3)
    c.connect()
    conn, _ = ls.accept()
    t0 = time.monotonic()
    with pytest.raises(pr.RotctldConnectionError):
        c.get_position()
    assert 0.25 < time.monotonic() - t0 < 1.0
    assert not c.connected
    conn.close()
    ls.close()


def test_cliente_servidor_cierra_conexion():
    ls = socket.socket()
    ls.bind(('127.0.0.1', 0))
    ls.listen(1)
    c = RotctldClient('127.0.0.1', ls.getsockname()[1], timeout=0.5)
    c.connect()
    conn, _ = ls.accept()
    conn.close()
    with pytest.raises(pr.RotctldConnectionError):
        c.get_position()
    ls.close()


def test_cliente_conexion_rechazada():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()                                    # puerto libre: nadie escucha
    with pytest.raises(OSError):
        RotctldClient('127.0.0.1', port, timeout=0.3).connect()


# ---------------------------------------------------------------- el puente
def _esperar(cond, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.02)
    return False


def _cfg(port, **kw):
    base = dict(port=port, timeout_s=0.5, cmd_rate_hz=20.0, read_rate_hz=20.0,
                backoff_min_s=0.1, backoff_max_s=0.3, ready_hold_s=0.2,
                az_res_deg=0.1, el_res_deg=0.1)
    base.update(kw)
    return BridgeConfig(**base)


def test_puente_preposiciona_sigue_y_aplica_banda_muerta(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port), log=lambda *_: None)
    core.start()
    try:
        assert _esperar(lambda: core.state == 'WAIT_REF')
        core.set_reference(120.0, 30.0)
        assert _esperar(lambda: core.state == 'TRACKING')
        assert rotor.position() == pytest.approx((120.0, 30.0), abs=0.5)
        n0 = core.stats['cmds_sent']
        # misma referencia repetida: la banda muerta no manda nada
        for _ in range(20):
            core.set_reference(120.0, 30.0)
            time.sleep(0.02)
        assert core.stats['cmds_sent'] == n0
        assert core.stats['skipped_deadband'] > 0
        # un cambio de 5 deg si se manda
        core.set_reference(125.0, 30.0)
        assert _esperar(lambda: core.stats['cmds_sent'] > n0)
        assert core.pop_readings()               # hay lecturas para publicar
    finally:
        core.stop()


def test_puente_tasa_maxima_de_ordenes(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port, cmd_rate_hz=5.0, preposition=False),
                      log=lambda *_: None)
    core.start()
    try:
        t0 = time.monotonic()
        az = 10.0
        # referencia cambiando a 100 Hz durante >= 2 s (y hasta que haya salido algo)
        while True:
            transcurrido = time.monotonic() - t0
            if (transcurrido >= 2.0 and core.stats['cmds_sent'] >= 3) or transcurrido > 10.0:
                break
            az += 0.5
            core.set_reference(az, 30.0)
            time.sleep(0.01)
        n = core.stats['cmds_sent']
        transcurrido = time.monotonic() - t0    # medido DESPUES de leer n: cota segura
        # Tope: nunca mas de 5 Hz (+1 de la orden inmediata y +1 de margen), mida lo que
        # mida la ventana. Antes se exigia 8 <= n <= 12 en 2 s de pared: con la maquina
        # cargada el hilo del puente se atrasaba y salian 6 (falso negativo ~1 de cada 15).
        # La cota inferior es solo de vivacidad: el limitador no bloquea.
        assert 3 <= n <= 5.0 * transcurrido + 2
    finally:
        core.stop()


def test_puente_rechazo_del_rotor_cuenta_y_no_cae(servidor):
    rotor, port = servidor
    # el puente cree que el rotor llega a 450, el falso solo a 360
    core = BridgeCore(_cfg(port, az_max=450.0, query_limits=False,
                           preposition=False, az_prefer='high'), log=lambda *_: None)
    core.start()
    try:
        assert _esperar(lambda: core.state == 'WAIT_REF')
        core.set_reference(400.0, 30.0)
        assert _esperar(lambda: core.stats['rejected'] >= 1)
        assert core.state != 'DISCONNECTED'
        assert core.last_cmd is None
    finally:
        core.stop()


def test_puente_limites_se_intersecan_con_dump_state(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port, az_max=450.0, el_max=180.0), log=lambda *_: None)
    core.start()
    try:
        assert _esperar(lambda: core.state == 'WAIT_REF')
        assert (core.az_max, core.el_max) == (360.0, 90.0)
    finally:
        core.stop()


def test_puente_reconecta_con_backoff():
    """Arranca SIN rotctld, el rotor aparece despues: el puente conecta solo."""
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    core = BridgeCore(_cfg(port), log=lambda *_: None)
    core.start()
    try:
        time.sleep(0.5)
        assert core.state == 'DISCONNECTED' and core.stats['connect_fail'] >= 1
        rotor = FakeRotor(0.0, 0.0, 600.0, 300.0)
        srv = serve_in_thread(rotor, port=port)
        try:
            assert _esperar(lambda: core.state == 'WAIT_REF', timeout=5.0)
            core.set_reference(50.0, 20.0)
            assert _esperar(lambda: core.state == 'TRACKING')
        finally:
            srv.shutdown()
            srv.server_close()
    finally:
        core.stop()


def test_puente_cae_el_rotor_en_pleno_pase_y_vuelve(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port), log=lambda *_: None)
    core.start()
    try:
        core.set_reference(60.0, 20.0)
        assert _esperar(lambda: core.state == 'TRACKING')
        n = core.stats['cmds_sent']
        # se corta el socket a la fuerza (como un cable USB que se afloja)
        core.client.sock.shutdown(socket.SHUT_RDWR)
        assert _esperar(lambda: core.stats['disconnects'] >= 1)
        assert _esperar(lambda: core.stats['connects'] >= 2)
        # `connects` sube al abrir el socket; el estado se restaura DESPUES, tras el
        # \dump_state de la reconexion (otro viaje de ida y vuelta). Mirarlo en cuanto
        # cambia el contador es una carrera: con la maquina cargada fallaba ~1 de cada 4
        # veces con 'DISCONNECTED'. Se espera al estado, no al contador.
        assert _esperar(lambda: core.state == 'TRACKING')   # sigue en el pase
        assert _esperar(lambda: core.stats['cmds_sent'] > n)  # reenvia tras reconectar
    finally:
        core.stop()


def test_puente_timeout_de_referencia_manda_stop(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port, ref_timeout_s=0.5, preposition=False),
                      log=lambda *_: None)
    core.start()
    try:
        core.set_reference(300.0, 30.0)
        assert _esperar(lambda: core.stats['cmds_sent'] >= 1)
        # sin mas referencias: a los 0.5 s manda S y el rotor se detiene
        assert _esperar(lambda: core.stats['ref_timeouts'] == 1)
        time.sleep(0.1)
        az0 = rotor.position()[0]
        time.sleep(0.3)
        assert rotor.position()[0] == az0
    finally:
        core.stop()


def test_puente_halt_detiene_y_no_manda(servidor):
    rotor, port = servidor
    core = BridgeCore(_cfg(port, preposition=False), log=lambda *_: None)
    core.start()
    try:
        core.set_reference(300.0, 30.0)
        assert _esperar(lambda: core.stats['cmds_sent'] >= 1)
        core.set_halt(True)
        assert _esperar(lambda: core.stats['halts'] == 1)
        n = core.stats['cmds_sent']
        core.set_reference(200.0, 30.0)
        time.sleep(0.4)
        assert core.stats['cmds_sent'] == n
        core.set_halt(False)
        assert _esperar(lambda: core.stats['cmds_sent'] > n)
    finally:
        core.stop()


def test_puente_envia_stop_al_salir():
    rotor = FakeRotor(0.0, 0.0, 20.0, 10.0)      # lento: tarda 15 s en llegar a 300
    srv = serve_in_thread(rotor)
    core = BridgeCore(_cfg(srv.server_address[1], preposition=False),
                      log=lambda *_: None)
    core.start()
    try:
        core.set_reference(300.0, 30.0)
        assert _esperar(lambda: core.stats['cmds_sent'] >= 1)
        time.sleep(0.3)
        core.stop()
        p0 = rotor.position()
        time.sleep(0.3)
        assert rotor.position() == p0            # detenido, no sigue hacia 300
        assert 0.0 < p0[0] < 300.0
    finally:
        core.stop()
        srv.shutdown()
        srv.server_close()
