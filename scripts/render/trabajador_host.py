#!/usr/bin/env python3
"""Trabajador de render del HOST (docs/APP.md 4.1).

El backend corre en el contenedor `station`, sin Docker ni pantalla; los renders `ejecutor: host` (Gazebo, Manim)
los ejecuta este proceso, que corre en el host y habla con la API. El gobernador del backend sigue decidiendo: este
trabajador solo recibe un trabajo si el gobernador lo admite (clase, cupo, carga, pantalla libre).

  python3 scripts/render/trabajador_host.py                      # bucle permanente
  python3 scripts/render/trabajador_host.py --una-vez            # un trabajo (o ninguno) y sale
  python3 scripts/render/trabajador_host.py --url http://127.0.0.1:8090 --clases gazebo,manim

Ciclo de cada trabajo (solo biblioteca estandar):
  POST /api/renders/reclamar {trabajador, clases}  -> 200 {job_id, render, comando, salida, pantalla, entorno} | 204
  ejecuta `comando` desde la raiz del repo (grupo de procesos propio) con DISPLAY=<pantalla> y el `entorno` dado,
  POST /api/renders/<job>/latido {progreso?, log:[lineas]}  cada <= 10 s  -> {"cancelar": true} mata el grupo
  POST /api/renders/<job>/fin {exit_code, senal?}
`PROGRESO <0-1>` en la salida del comando alimenta `progreso`; la salida completa va tambien a <salida>/render.log.
Sin respuesta del backend durante --tolerancia s (90), el trabajo se detiene (el backend ya lo habria dado por
interrumpido tras 60 s sin latido). SIGTERM/SIGINT al trabajador detienen el trabajo y notifican `fin`.
"""
import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PROGRESO_RE = re.compile(r'^PROGRESO\s+([0-9]*\.?[0-9]+(?:[eE][-+]?\d+)?)\s*$')
LATIDO_S = 8.0           # el backend exige <= 10 s
PARADA_S = 15.0          # SIGTERM -> SIGKILL


def log(msg):
    print(time.strftime('%H:%M:%S'), 'trabajador:', msg, flush=True)


class Api:
    def __init__(self, url, timeout=15.0):
        self.url, self.timeout = url.rstrip('/'), timeout

    def post(self, ruta, cuerpo):
        """(codigo, json|None). Lanza OSError si no hay conexion."""
        req = urllib.request.Request(self.url + ruta, data=json.dumps(cuerpo).encode('utf-8'),
                                     headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                datos = r.read()
                return r.status, (json.loads(datos) if datos else None)
        except urllib.error.HTTPError as e:
            try:
                cuerpo = json.loads(e.read() or b'null')
            except ValueError:
                cuerpo = None
            return e.code, cuerpo


class Trabajo:
    """Un proceso del host con su grupo, su log y su progreso."""

    def __init__(self, t, raiz):
        self.t, self.raiz = t, raiz
        self.id = t['job_id']
        self.salida = os.path.join(raiz, t['salida'])
        self.progreso = None
        self.nuevas = []                      # lineas aun no enviadas
        self.cola = deque(maxlen=200)
        self.lock = threading.Lock()
        self.cancelado = False
        env = dict(os.environ)
        env.update({k: str(v) for k, v in (t.get('entorno') or {}).items()})
        env.setdefault('PYTHONUNBUFFERED', '1')
        if t.get('pantalla'):
            # el escritorio real se conserva en DISPLAY_ANFITRION: los envoltorios lo dan a Xephyr como padre
            env.setdefault('DISPLAY_ANFITRION', os.environ.get('DISPLAY') or ':0')
            env['DISPLAY'] = t['pantalla']
        os.makedirs(self.salida, exist_ok=True)
        self.archivo_log = open(os.path.join(self.salida, 'render.log'), 'a', encoding='utf-8')
        self.proc = subprocess.Popen(t['comando'], cwd=raiz, env=env, stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     errors='replace', start_new_session=True)
        self.hilo = threading.Thread(target=self._leer, daemon=True)
        self.hilo.start()

    def _leer(self):
        for linea in self.proc.stdout:
            linea = linea.rstrip('\n')
            m = PROGRESO_RE.match(linea)
            with self.lock:
                if m:
                    self.progreso = min(max(float(m.group(1)), 0.0), 1.0)
                else:
                    self.nuevas.append(linea)
                    self.cola.append(linea)
            self.archivo_log.write(linea + '\n')
            self.archivo_log.flush()

    def latido(self):
        with self.lock:
            cuerpo = {'log': [x[:7000] for x in self.nuevas[-50:]]}
            if self.progreso is not None:
                cuerpo['progreso'] = self.progreso
            self.nuevas = []
        return cuerpo

    def devolver(self, cuerpo):                # si el latido fallo, las lineas vuelven a la cola
        with self.lock:
            self.nuevas = cuerpo.get('log', []) + self.nuevas

    def detener(self):
        """SIGTERM al grupo (los envoltorios limpian contenedor y pantalla), SIGKILL si no sale."""
        self.cancelado = True
        if self.proc.poll() is not None:
            return
        for sig, espera in ((signal.SIGTERM, PARADA_S), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(espera)
                return
            except subprocess.TimeoutExpired:
                continue

    def cerrar(self):
        self.hilo.join(timeout=5)
        self.archivo_log.close()


class Trabajador:
    def __init__(self, a):
        self.a = a
        self.api = Api(a.url)
        self.nombre = a.nombre or f'{socket.gethostname()}-{os.getpid()}'
        self.clases = [c.strip() for c in a.clases.split(',') if c.strip()]
        self.actual = None
        self.parar = False

    def reclamar(self):
        try:
            cod, cuerpo = self.api.post('/api/renders/reclamar',
                                        {'trabajador': self.nombre, 'clases': self.clases})
        except OSError as e:
            log(f'sin conexion con {self.a.url}: {e}')
            return None
        if cod == 200 and cuerpo:
            return cuerpo
        if cod not in (200, 204):
            log(f'reclamar -> HTTP {cod}: {cuerpo}')
        return None

    def ejecutar(self, t):
        log(f'trabajo {t["job_id"]} ({t.get("render")}): {" ".join(t["comando"])} en {t.get("pantalla") or "-"}')
        try:
            self.actual = tr = Trabajo(t, RAIZ)
        except OSError as e:
            log(f'no pude lanzar el comando: {e}')
            self._fin(t['job_id'], 127, None, f'no pude lanzar el comando: {e}')
            return
        inicio = time.time()
        ultimo_ok = time.time()
        limite = t.get('timeout_s')
        proximo = 0.0
        while tr.proc.poll() is None:
            time.sleep(0.5)
            if self.parar:
                tr.detener()
                break
            if limite and time.time() - inicio > limite:
                log('excedio timeout_s: se detiene')
                tr.detener()
                break
            if time.time() < proximo:
                continue
            proximo = time.time() + self.a.latido
            cuerpo = tr.latido()
            try:
                cod, resp = self.api.post(f'/api/renders/{tr.id}/latido', cuerpo)
            except OSError as e:
                tr.devolver(cuerpo)
                if time.time() - ultimo_ok > self.a.tolerancia:
                    log(f'{self.a.tolerancia:g} s sin backend ({e}): detengo el trabajo')
                    tr.detener()
                continue
            if cod == 200:
                ultimo_ok = time.time()
                if (resp or {}).get('cancelar'):
                    log('cancelacion pedida: detengo el grupo de procesos')
                    tr.detener()
            elif cod in (404, 409, 410):
                log(f'latido -> HTTP {cod}: el backend ya no espera este trabajo; lo detengo')
                tr.detener()
            else:
                tr.devolver(cuerpo)
        rc = tr.proc.wait()
        tr.cerrar()
        self.actual = None
        senal = -rc if rc < 0 else None
        exit_code = 128 + senal if senal else rc
        # ultimo latido con lo pendiente (progreso 1 y las lineas finales) y luego `fin`
        try:
            self.api.post(f'/api/renders/{tr.id}/latido', tr.latido())
        except OSError:
            pass
        self._fin(tr.id, exit_code, senal)
        log(f'trabajo {tr.id} termino: codigo {exit_code}' + (f' (senal {senal})' if senal else '')
            + f' en {time.time() - inicio:.0f} s')

    def _fin(self, job_id, exit_code, senal, error=None):
        cuerpo = {'exit_code': exit_code}
        if senal:
            cuerpo['senal'] = senal
        if error:
            cuerpo['error'] = error
        for intento in range(6):
            try:
                cod, resp = self.api.post(f'/api/renders/{job_id}/fin', cuerpo)
                if cod < 500:
                    if cod >= 400:
                        log(f'fin -> HTTP {cod}: {resp}')
                    return
            except OSError as e:
                log(f'fin sin conexion ({e}); reintento')
            time.sleep(2 + 3 * intento)

    def bucle(self):
        def terminar(sig, _frame):
            log(f'senal {sig}: termino')
            self.parar = True
        signal.signal(signal.SIGTERM, terminar)
        signal.signal(signal.SIGINT, terminar)
        log(f'{self.nombre}: clases {self.clases}, backend {self.a.url}, raiz {RAIZ}')
        while not self.parar:
            t = self.reclamar()
            if t:
                self.ejecutar(t)
                if self.a.una_vez:
                    return 0
                continue
            if self.a.una_vez:
                log('no hay trabajos admisibles ahora')
                return 0
            for _ in range(int(self.a.espera * 10)):
                if self.parar:
                    break
                time.sleep(0.1)
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default=os.environ.get('STATION_URL', 'http://127.0.0.1:8090'))
    ap.add_argument('--una-vez', action='store_true', help='reclama una vez, ejecuta si hay trabajo y sale')
    ap.add_argument('--clases', default='gazebo,manim,ligero', help='clases del gobernador que este host sabe ejecutar')
    ap.add_argument('--nombre', default='')
    ap.add_argument('--espera', type=float, default=5.0, help='s entre reclamos cuando no hay trabajo')
    ap.add_argument('--latido', type=float, default=LATIDO_S, help='s entre latidos (el backend exige <= 10)')
    ap.add_argument('--tolerancia', type=float, default=90.0, help='s sin backend antes de detener el trabajo')
    return Trabajador(ap.parse_args(argv)).bucle()


if __name__ == '__main__':
    sys.exit(main())
