"""Ejecucion de UN contacto como una corrida `ros2 launch atp_mision mision_pase`.

Cada corrida es un subproceso con su `ROS_DOMAIN_ID` (la red del contenedor es
`host`: sin dominios distintos dos corridas se mezclarian `/clock`, `/atp/*`).
Los dominios salen de un pool PROPIO (200-229) que no toca el de la Estacion
(100-199) ni los de las pruebas (61-99), y nunca se comparten entre corridas
activas. Misma idea que `station/backend/station/runner.py` (subproceso sin shell,
grupo de procesos propio, SIGINT y luego SIGKILL al vencer el plazo); aqui con
hilos porque el orquestador es sincrono.

`EjecutorROS` es el ejecutor real; las pruebas lo sustituyen por una funcion con
la misma firma `(args, carpeta) -> dict` (`estado`: 'ok' | 'fallo' | 'plazo').
"""
import os
import queue
import re
import signal
import subprocess
import threading
import time

DOMINIOS = range(200, 230)
LINEA_RESUMEN_SDR = re.compile(r'RESUMEN SDR .*')


def argumentos(args):
    """dict -> ['k:=v', ...] (booleanos como true/false). Sin shell: no hay inyeccion."""
    def v(x):
        return 'true' if x is True else 'false' if x is False else str(x)
    return [f'{k}:={v(x)}' for k, x in args.items() if x is not None and x != '']


class PoolDominios:
    """Dominios ROS libres. `tomar` bloquea hasta que haya uno."""

    def __init__(self, dominios=DOMINIOS):
        self._q = queue.Queue()
        for d in dominios:
            self._q.put(d)

    def tomar(self):
        return self._q.get()

    def devolver(self, d):
        self._q.put(d)


class EjecutorROS:
    """Lanza `ros2 launch` por contacto. `args` son los argumentos del launch (ya
    con `record_dir`); `carpeta` recibe `launch.log`."""

    def __init__(self, dominios=None, plazo_s=1800.0, gracia_s=8.0, paquete='atp_mision',
                 launch='mision_pase.launch.py', comando=None, inactividad_s=420.0):
        self.pool = dominios or PoolDominios()
        self.plazo_s = float(plazo_s)
        # una corrida colgada (p. ej. DDS que no empareja con la maquina muy cargada: el
        # generador espera para siempre el saludo de arranque) no escribe nada; si ni
        # `launch.log` ni `telemetry.jsonl` crecen en este tiempo se mata y se reintenta
        self.inactividad_s = float(inactividad_s)
        self.gracia_s = float(gracia_s)
        self.paquete, self.launch = paquete, launch
        # `comando(args, carpeta) -> argv` sustituye al `ros2 launch` (las pruebas)
        self.comando = comando

    def __call__(self, args, carpeta):
        os.makedirs(carpeta, exist_ok=True)
        dom = self.pool.tomar()
        t0 = time.monotonic()
        try:
            env = dict(os.environ, ROS_DOMAIN_ID=str(dom), PYTHONUNBUFFERED='1',
                       RCUTILS_COLORIZED_OUTPUT='0')
            cmd = (self.comando(args, carpeta) if self.comando else
                   ['ros2', 'launch', self.paquete, self.launch] + argumentos(args))
            with open(os.path.join(carpeta, 'launch.log'), 'ab') as log:
                proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, env=env,
                                        start_new_session=True)
                estado = 'ok'
                rc = self._esperar(proc, carpeta)
                if rc == 'plazo':
                    estado = 'plazo'
                    rc = self._matar(proc)
                elif rc == 'inactiva':
                    estado = 'inactiva'
                    rc = self._matar(proc)
                else:
                    self._barrer(proc)       # nodos que sobrevivan al lanzador
            ok = os.path.exists(os.path.join(carpeta, 'metrics.json'))
            if estado == 'ok' and not ok:
                estado = 'fallo'
            return {'estado': estado, 'rc': rc, 'dominio': dom,
                    'pared_s': round(time.monotonic() - t0, 1)}
        finally:
            self.pool.devolver(dom)

    def _esperar(self, proc, carpeta):
        """Espera al proceso: devuelve su codigo, 'plazo' (excedio `plazo_s`) o 'inactiva'
        (`launch.log` y `telemetry.jsonl` sin crecer `inactividad_s` segundos)."""
        t0 = ultimo = time.monotonic()
        tam = None
        while True:
            try:
                return proc.wait(2.0)
            except subprocess.TimeoutExpired:
                pass
            ahora = time.monotonic()
            if ahora - t0 > self.plazo_s:
                return 'plazo'
            nuevo = tuple(_tamano(os.path.join(carpeta, f)) for f in ('launch.log', 'telemetry.jsonl'))
            if nuevo != tam:
                tam, ultimo = nuevo, ahora
            elif self.inactividad_s > 0 and ahora - ultimo > self.inactividad_s:
                return 'inactiva'

    def _matar(self, proc):
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            rc = proc.wait(self.gracia_s)
        except subprocess.TimeoutExpired:
            rc = None
        self._barrer(proc)
        proc.wait()
        return rc

    @staticmethod
    def _barrer(proc):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _tamano(ruta):
    try:
        return os.path.getsize(ruta)
    except OSError:
        return -1


def _es_ruido_de_apagado(siguientes):
    """Carrera conocida de rclpy (Jazzy) al destruir un `Task` en el apagado: el proceso
    termina bien y los datos no cambian, pero el Traceback depende de la carga y haria
    distinto un informe que debe ser identico (CONTRATO 8)."""
    texto = ''.join(siguientes)
    return 'rclpy/task.py' in texto and '__del__' in texto


def incidencias_de_log(ruta):
    """Lineas relevantes de un `launch.log`: el RESUMEN SDR (texto), y la cuenta de
    enganches y perdidas de enganche del autotrack, errores y avisos de lockstep.
    Devuelve (dict, [str])."""
    info = {'enganches': 0, 'perdidas_enganche': 0, 'sdr': None}
    avisos = []
    try:
        with open(ruta, encoding='utf-8', errors='replace') as f:
            lineas = f.readlines()
        for i, linea in enumerate(lineas):
            if '[autotrack' in linea and 'ENGANCHE' in linea and 'perdida' not in linea:
                info['enganches'] += 1
            elif '[autotrack' in linea and 'perdida de enganche' in linea:
                info['perdidas_enganche'] += 1
            m = LINEA_RESUMEN_SDR.search(linea)
            if m:
                info['sdr'] = m.group(0)
            if 'Traceback' in linea and _es_ruido_de_apagado(lineas[i + 1:i + 4]):
                continue
            if ('Traceback' in linea or '[ERROR]' in linea or 'sin confirmar' in linea
                    or 'no repetible' in linea or 'los sellos no coinciden' in linea
                    or 'se empareja con' in linea):
                avisos.append(linea.strip()[:240])
    except OSError:
        pass
    return info, avisos


def parse_resumen_sdr(texto):
    """Cifras del `RESUMEN SDR` (o None): Doppler rms (Hz), sesgo del C/N0 contra el
    presupuesto (dB), % enganchado, huecos."""
    if not texto:
        return None
    def num(pat):
        m = re.search(pat, texto)
        return float(m.group(1)) if m else None
    return {'doppler_rms_hz': num(r'Doppler rms=([\d.]+)'),
            'cn0_sesgo_db': num(r'sesgo=([+\-][\d.]+) dB'),
            'cn0_rms_db': num(r'dB rms=([\d.]+) dB'),
            'enganchado_pct': num(r'enganchado ([\d.]+)%'),
            'huecos': int(num(r'huecos=(\d+)') or 0)}
