"""Cliente TCP bloqueante del protocolo `rotctld` de Hamlib.

Un cliente = una conexion = un hilo. Cualquier timeout o respuesta truncada
cierra el socket y lanza RotctldConnectionError: el protocolo de texto de
rotctld no tiene numeros de secuencia, asi que tras un timeout la unica
forma segura de resincronizar es reconectar (un comando desconocido, por
ejemplo, no recibe respuesta en Hamlib 4.5 y desfasaria las siguientes).
"""
import socket
import time

from atp_hw import protocol as pr


class RotctldClient:
    def __init__(self, host='127.0.0.1', port=4533, timeout=1.0):
        self.host, self.port, self.timeout = host, int(port), float(timeout)
        self.sock = None
        self.buf = b''

    @property
    def connected(self):
        return self.sock is not None

    def connect(self):
        self.close()
        # OSError (ConnectionRefused, timeout, ...) sube tal cual: lo maneja
        # el que reconecta con backoff.
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self.sock, self.buf = s, b''

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock, self.buf = None, b''

    # ---- E/S de bajo nivel -------------------------------------------------
    def _send(self, text):
        if self.sock is None:
            raise pr.RotctldConnectionError('no conectado')
        try:
            self.sock.sendall(text.encode('ascii'))
        except OSError as e:
            self.close()
            raise pr.RotctldConnectionError(f'envio fallido: {e}') from e

    def _readline(self, deadline):
        while b'\n' not in self.buf:
            left = deadline - time.monotonic()
            if left <= 0:
                self.close()
                raise pr.RotctldConnectionError('timeout esperando respuesta')
            try:
                self.sock.settimeout(left)
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue                 # el bucle decide por el deadline
            except OSError as e:
                self.close()
                raise pr.RotctldConnectionError(f'lectura fallida: {e}') from e
            if not chunk:
                self.close()
                raise pr.RotctldConnectionError('el servidor cerro la conexion')
            self.buf += chunk
        line, _, self.buf = self.buf.partition(b'\n')
        return line.decode('ascii', errors='replace').rstrip('\r')

    def _transact(self, cmd, n_lines):
        """Envia `cmd` y lee `n_lines` lineas (si la primera es RPRT, solo esa)."""
        self._send(cmd)
        deadline = time.monotonic() + self.timeout
        first = self._readline(deadline)
        if n_lines == 1 or pr.parse_rprt(first) is not None:
            return [first]
        return [first] + [self._readline(deadline) for _ in range(n_lines - 1)]

    # ---- Comandos ----------------------------------------------------------
    def set_position(self, az_deg, el_deg):
        """`P az el`. Lanza RotctldRPRT si el demonio rechaza la orden."""
        (line,) = self._transact(pr.format_set_position(az_deg, el_deg), 1)
        pr.check_rprt(line)

    def get_position(self):
        """`p` -> (az, el) en grados."""
        return pr.parse_position(self._transact('p\n', 2))

    def stop(self):
        """`S`: detiene el movimiento."""
        (line,) = self._transact('S\n', 1)
        pr.check_rprt(line)

    def get_limits(self):
        """Limites del rotor segun `\\dump_state` (min_az, max_az, min_el,
        max_el). Devuelve {} si el demonio no lo soporta."""
        self._send('\\dump_state\n')
        deadline = time.monotonic() + self.timeout
        lines = []
        while len(lines) < 64:
            ln = self._readline(deadline)
            if ln.strip() == 'done' or pr.parse_rprt(ln) is not None:
                break               # 'done' normal; RPRT = no soportado
            lines.append(ln)
        st = pr.parse_dump_state(lines)
        return {k: st[k] for k in ('min_az', 'max_az', 'min_el', 'max_el')
                if isinstance(st.get(k), float)}
