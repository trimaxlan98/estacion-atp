"""Rotor falso con el protocolo de `rotctld` y dinamica realista.

Sirve para medir el error de seguimiento contra un rotor LENTO sin hardware.
(`rotctld -m 1`, el rotor dummy de Hamlib, tampoco es instantaneo: medido
aqui se mueve a ~6 deg/s en AMBOS ejes, sin tope de velocidad por eje ni
cuantizacion ni latencia configurables; este modulo si los tiene.)

Dinamica: cada eje avanza hacia su objetivo a velocidad maxima constante
(sin aceleracion: un G-5500 arranca y frena casi de golpe, los contactores
no dan rampa). La posicion se integra "perezosamente" con el reloj
monotonic en cada consulta, asi no hay hilo de fisica ni deriva.

Velocidades por defecto: 6 deg/s en acimut y 3 deg/s en elevacion. SUPUESTO
cercano a un Yaesu G-5500 (del manual, de memoria y NO verificado aqui:
~58 s por 360 deg en acimut = 6.2 deg/s y ~67 s por 180 deg en elevacion
= 2.7 deg/s). Son parametros: --az-speed / --el-speed.

Comandos: P, p, S, K (park), _ (info), q/Q (cerrar), \\dump_state, \\set_pos,
\\get_pos, \\stop. Un comando desconocido contesta `RPRT -11` (el rotctld real
de Hamlib 4.5 no contesta nada; aqui es mas amable).
"""
import argparse
import socketserver
import threading
import time


class FakeRotor:
    def __init__(self, az0=0.0, el0=0.0, az_speed=6.0, el_speed=3.0,
                 az_range=(0.0, 360.0), el_range=(0.0, 90.0),
                 az_res=0.0, el_res=0.0, clock=time.monotonic):
        self.pos = [float(az0), float(el0)]
        self.target = list(self.pos)
        self.speed = (float(az_speed), float(el_speed))
        self.az_range, self.el_range = az_range, el_range
        self.res = (float(az_res), float(el_res))
        self.clock = clock
        self.t = clock()
        self.lock = threading.Lock()
        self.n_set = 0          # estadisticas para las pruebas
        self.n_rejected = 0

    def _advance(self):
        now = self.clock()
        dt = max(0.0, now - self.t)
        self.t = now
        for i in (0, 1):
            step = self.speed[i] * dt
            d = self.target[i] - self.pos[i]
            self.pos[i] += max(-step, min(step, d))

    def set_target(self, az, el):
        """False si esta fuera de rango (rotctld contesta RPRT -1)."""
        with self.lock:
            lo, hi = self.az_range
            elo, ehi = self.el_range
            if not (lo <= az <= hi and elo <= el <= ehi):
                self.n_rejected += 1
                return False
            self._advance()
            self.target = [float(az), float(el)]
            self.n_set += 1
            return True

    def stop(self):
        with self.lock:
            self._advance()
            self.target = list(self.pos)

    def park(self):
        return self.set_target(self.az_range[0], self.el_range[0])

    def position(self):
        """Posicion actual, cuantizada a la resolucion del rotor (0 = sin)."""
        with self.lock:
            self._advance()
            out = []
            for p, r in zip(self.pos, self.res):
                out.append(round(p / r) * r if r > 0 else p)
            return out[0], out[1]

    def moving(self):
        with self.lock:
            self._advance()
            return self.pos != self.target


def handle_line(rotor, line, delay_s=0.0):
    """Atiende un comando. Devuelve (texto_de_respuesta, cerrar_conexion)."""
    parts = line.strip().split()
    if not parts:
        return '', False
    cmd, args = parts[0], parts[1:]
    if cmd.startswith('\\'):
        cmd = {'\\set_pos': 'P', '\\get_pos': 'p', '\\stop': 'S',
               '\\park': 'K', '\\get_info': '_',
               '\\dump_state': 'dump_state'}.get(cmd, cmd)
    if delay_s > 0:
        time.sleep(delay_s)
    if cmd == 'P':
        try:
            az, el = float(args[0]), float(args[1])
        except (IndexError, ValueError):
            return 'RPRT -1\n', False
        return ('RPRT 0\n' if rotor.set_target(az, el) else 'RPRT -1\n'), False
    if cmd == 'p':
        az, el = rotor.position()
        return f'{az:.6f}\n{el:.6f}\n', False
    if cmd == 'S':
        rotor.stop()
        return 'RPRT 0\n', False
    if cmd == 'K':
        return ('RPRT 0\n' if rotor.park() else 'RPRT -1\n'), False
    if cmd == '_':
        return 'Fake rotator (atp_hw)\n', False
    if cmd == 'dump_state':
        return (f'1\n1\nmin_az={rotor.az_range[0]:.6f}\n'
                f'max_az={rotor.az_range[1]:.6f}\n'
                f'min_el={rotor.el_range[0]:.6f}\n'
                f'max_el={rotor.el_range[1]:.6f}\n'
                'south_zero=0\nrot_type=AzEl\ndone\n'), False
    if cmd in ('q', 'Q'):
        return '', True
    return 'RPRT -11\n', False


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for raw in self.rfile:
            try:
                text, close = handle_line(self.server.rotor,
                                          raw.decode('ascii', 'replace'),
                                          self.server.delay_s)
            except Exception as e:                  # no tumbar el servidor
                text, close = f'RPRT -7\n', False
                print(f'fake_rotctld: error interno: {e}', flush=True)
            if text:
                self.wfile.write(text.encode('ascii'))
            if close:
                return


class FakeRotctldServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, host, port, rotor, delay_s=0.0):
        super().__init__((host, port), _Handler)
        self.rotor, self.delay_s = rotor, float(delay_s)


def serve_in_thread(rotor, host='127.0.0.1', port=0, delay_s=0.0):
    """Arranca el servidor en un hilo (puerto 0 = efimero). Para pruebas."""
    srv = FakeRotctldServer(host, port, rotor, delay_s)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser(description='Rotor falso con protocolo rotctld')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=4690)
    ap.add_argument('--az-speed', type=float, default=6.0, help='deg/s')
    ap.add_argument('--el-speed', type=float, default=3.0, help='deg/s')
    ap.add_argument('--az-min', type=float, default=0.0)
    ap.add_argument('--az-max', type=float, default=360.0)
    ap.add_argument('--el-min', type=float, default=0.0)
    ap.add_argument('--el-max', type=float, default=90.0)
    ap.add_argument('--az0', type=float, default=0.0, help='posicion inicial')
    ap.add_argument('--el0', type=float, default=0.0)
    ap.add_argument('--az-res', type=float, default=0.0,
                    help='cuantizacion de la lectura (0 = sin; GS-232: 1.0)')
    ap.add_argument('--el-res', type=float, default=0.0)
    ap.add_argument('--delay', type=float, default=0.0,
                    help='latencia por comando en s (emula el puerto serie)')
    a = ap.parse_args()
    rotor = FakeRotor(a.az0, a.el0, a.az_speed, a.el_speed,
                      (a.az_min, a.az_max), (a.el_min, a.el_max),
                      a.az_res, a.el_res)
    srv = FakeRotctldServer(a.host, a.port, rotor, a.delay)
    print(f'fake_rotctld en {a.host}:{a.port} | az {a.az_speed:g} deg/s '
          f'[{a.az_min:g},{a.az_max:g}] | el {a.el_speed:g} deg/s '
          f'[{a.el_min:g},{a.el_max:g}] | inicio az={a.az0:g} el={a.el0:g}',
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == '__main__':
    main()
