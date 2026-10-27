"""Renderizador de prueba (sin Gazebo ni ffmpeg): deja en <salida> un video falso, su miniatura y un
`media.json` del contrato de docs/APP.md 4.

    python3 render_falso.py <run_dir> <salida> <pantalla> <job_id> [--modo ok] [--pasos 3]

`--modo`: ok | sinmedia | sha_malo | sinminiatura | falla | dormir | senal | senal_una_vez
"""
import argparse
import hashlib
import json
import os
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('salida')
    ap.add_argument('pantalla')
    ap.add_argument('job_id')
    ap.add_argument('--modo', default='ok')
    ap.add_argument('--pasos', type=int, default=3)
    ap.add_argument('--acelerar', type=float, default=30)
    a = ap.parse_args()
    run_dir, salida = Path(a.run_dir), Path(a.salida)
    t0 = time.time()
    print('pantalla', repr(a.pantalla), 'DISPLAY', os.environ.get('DISPLAY'),
          'LP', os.environ.get('LP_NUM_THREADS'), flush=True)
    if a.modo in ('senal', 'senal_una_vez'):
        cont = run_dir / f'contador_{a.job_id}'
        n = len(cont.read_text()) if cont.exists() else 0
        with open(cont, 'a') as f:
            f.write('x')
        if a.modo == 'senal' or n == 0:
            os.kill(os.getpid(), signal.SIGKILL)
    if a.modo == 'dormir':
        time.sleep(120)
    for i in range(a.pasos):
        print(f'PROGRESO {(i + 1) / a.pasos:.3f}', flush=True)
        time.sleep(0.02)
    if a.modo == 'falla':
        sys.exit(4)
    video = bytes((i * 7 + 3) % 256 for i in range(5000))          # 5000 bytes deterministas
    (salida / 'visor.mp4').write_bytes(video)
    if a.modo != 'sinminiatura':
        (salida / 'miniatura.jpg').write_bytes(b'\xff\xd8\xff\xe0' + b'\x00' * 200)
    if a.modo == 'sinmedia':
        return
    sha = hashlib.sha256(video).hexdigest()
    if a.modo == 'sha_malo':
        sha = '0' * 64
    archivos = [{'ruta': 'visor.mp4', 'tipo': 'video', 'ancho': 1920, 'alto': 1080, 'duracion_s': 80.0,
                 'sha256': sha, 'principal': True}]
    if a.modo != 'sinminiatura':
        archivos.append({'ruta': 'miniatura.jpg', 'tipo': 'imagen', 'ancho': 640, 'alto': 360})
    media = {'version': 1, 'job_id': a.job_id, 'render': os.environ.get('RENDER_ID', 'render-falso'), 'run_id': run_dir.name,
             'creado_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
             'duracion_pared_s': round(time.time() - t0, 3), 'archivos': archivos,
             'procedencia': {'commit': 'deadbeef', 'sucio': False, 'comando': sys.argv,
                             'args': {'acelerar': a.acelerar},
                             'corrida': {'run_id': run_dir.name, 'experiment_id': 'prueba-proceso', 'args': {}}},
             'notas': 'render de prueba'}
    (salida / 'media.json').write_text(json.dumps(media))
    print('listo', flush=True)


if __name__ == '__main__':
    main()
