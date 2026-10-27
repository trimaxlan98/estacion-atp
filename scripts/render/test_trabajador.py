#!/usr/bin/env python3
"""Prueba del trabajador del host contra un backend FALSO (http.server) que habla el protocolo de APP.md 4.1.

  python3 -m unittest scripts/render/test_trabajador.py -v
  python3 scripts/render/test_trabajador.py --real-graficas <run_dir>   # ademas, un envoltorio real (graficas)

Cubre: reclamar (200/204), comando con progreso y log, latidos <= 10 s, `fin` con exit_code, DISPLAY y entorno,
cancelacion por latido y señal externa.
"""
import http.server
import json
import os
import re
import sys
import tempfile
import threading
import time
import unittest

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
sys.path.insert(0, AQUI)
import trabajador_host as th  # noqa: E402


class Falso(http.server.ThreadingHTTPServer):
    def __init__(self, trabajos, cancelar_en=None):
        self.cola = list(trabajos)
        self.eventos = []                       # (ruta, cuerpo)
        self.cancelar_en = cancelar_en or {}    # job_id -> nº de latido que responde cancelar
        self.latidos = {}
        self.lock = threading.Lock()
        super().__init__(('127.0.0.1', 0), Manejador)


class Manejador(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        s = self.server
        n = int(self.headers.get('Content-Length') or 0)
        cuerpo = json.loads(self.rfile.read(n) or b'null')
        with s.lock:
            s.eventos.append((self.path, cuerpo))
            resp, cod = None, 200
            if self.path == '/api/renders/reclamar':
                if s.cola:
                    resp = s.cola.pop(0)
                else:
                    cod = 204
            elif self.path.endswith('/latido'):
                j = self.path.split('/')[3]
                s.latidos[j] = s.latidos.get(j, 0) + 1
                resp = {'cancelar': s.cancelar_en.get(j) == s.latidos[j]}
            elif self.path.endswith('/fin'):
                resp = {'ok': True}
        datos = json.dumps(resp).encode() if resp is not None else b''
        self.send_response(cod)
        self.send_header('Content-Length', str(len(datos)))
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(datos)


def job(job_id, comando, salida, pantalla=':80', entorno=None, **kw):
    d = {'job_id': job_id, 'render': 'falso', 'comando': comando, 'salida': salida, 'pantalla': pantalla,
         'entorno': entorno or {}}
    d.update(kw)
    return d


def servir(srv):
    h = threading.Thread(target=srv.serve_forever, daemon=True)
    h.start()
    return f'http://127.0.0.1:{srv.server_address[1]}'


def correr(url, *extra):
    return th.main(['--url', url, '--una-vez', '--latido', '1', *extra])


class Pruebas(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='trab-', dir=os.path.join(RAIZ, 'runs') if os.path.isdir(os.path.join(RAIZ, 'runs')) else None)
        self.salida = os.path.relpath(self.tmp, RAIZ)

    def fins(self, srv):
        return [c for r, c in srv.eventos if r.endswith('/fin')]

    def test_204_sin_trabajo(self):
        srv = Falso([])
        self.addCleanup(srv.shutdown)
        self.assertEqual(correr(servir(srv)), 0)
        self.assertEqual([r for r, _ in srv.eventos], ['/api/renders/reclamar'])
        self.assertEqual(srv.eventos[0][1]['clases'], ['gazebo', 'manim', 'ligero'])

    def test_ciclo_completo_progreso_log_entorno_display(self):
        prog = ("import os,sys,time\n"
                "print('PROGRESO 0.25',flush=True); time.sleep(1.6); print('hola', os.environ['DISPLAY'], "
                "os.environ['RENDER_JOB_ID'], os.getcwd(), flush=True); print('PROGRESO 0.9',flush=True); time.sleep(1.2)\n")
        srv = Falso([job('j1', [sys.executable, '-c', prog], self.salida, ':81',
                         {'RENDER_JOB_ID': 'j1', 'ROS_DOMAIN_ID': '150'})])
        self.addCleanup(srv.shutdown)
        self.assertEqual(correr(servir(srv)), 0)
        rutas = [r for r, _ in srv.eventos]
        self.assertEqual(rutas[0], '/api/renders/reclamar')
        self.assertEqual(rutas[-1], '/api/renders/j1/fin')
        self.assertIn('/api/renders/j1/latido', rutas)
        self.assertEqual(self.fins(srv), [{'exit_code': 0}])
        lat = [c for r, c in srv.eventos if r.endswith('/latido')]
        self.assertTrue(any('progreso' in c for c in lat))
        lineas = sum((c.get('log', []) for c in lat), [])
        self.assertTrue(any(l == f'hola :81 j1 {RAIZ}' for l in lineas), lineas)
        self.assertTrue(all(not l.startswith('PROGRESO') for l in lineas))
        self.assertIn('hola :81', open(os.path.join(self.tmp, 'render.log')).read())

    def test_exit_code_distinto_de_cero(self):
        srv = Falso([job('j2', [sys.executable, '-c', 'import sys; print("mal"); sys.exit(3)'], self.salida)])
        self.addCleanup(srv.shutdown)
        correr(servir(srv))
        self.assertEqual(self.fins(srv), [{'exit_code': 3}])

    def test_cancelacion_por_latido_mata_el_grupo(self):
        hijo = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); time.sleep(60)"
        srv = Falso([job('j3', [sys.executable, '-c', hijo], self.salida)], cancelar_en={'j3': 2})
        self.addCleanup(srv.shutdown)
        t0 = time.time()
        correr(servir(srv))
        self.assertLess(time.time() - t0, 30)
        f = self.fins(srv)[0]
        self.assertEqual(f['senal'], 15)
        self.assertEqual(f['exit_code'], 143)      # 128 + SIGTERM: el backend lo reconoce como muerte por senal

    def test_senal_externa_se_reporta(self):
        srv = Falso([job('j4', [sys.executable, '-c', 'import os,signal,time; time.sleep(0.5); os.kill(os.getpid(), signal.SIGKILL)'],
                         self.salida)])
        self.addCleanup(srv.shutdown)
        correr(servir(srv))
        self.assertEqual(self.fins(srv), [{'exit_code': 137, 'senal': 9}])


def real_graficas(run):
    """Un trabajo REAL: el envoltorio graficas.sh, como lo lanzaria el backend."""
    salida = os.path.join('runs', '_prueba', 'trab-real')
    srv = Falso([job('trab-real', ['bash', 'scripts/render/graficas.sh', run, salida], salida, '',
                     {'RENDER_JOB_ID': 'trab-real', 'RENDER_ID': 'graficas'})])
    url = servir(srv)
    rc = th.main(['--url', url, '--una-vez', '--latido', '2'])
    srv.shutdown()
    print('eventos:', [(r.split('/')[-1], list(c)) for r, c in srv.eventos])
    return rc


if __name__ == '__main__':
    if '--real-graficas' in sys.argv:
        sys.exit(real_graficas(sys.argv[sys.argv.index('--real-graficas') + 1]))
    unittest.main()
