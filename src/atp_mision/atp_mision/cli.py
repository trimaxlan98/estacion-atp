"""Linea de comandos de la mision integrada (`scripts/mision.py`, `ros2 run atp_mision mision`).

Dentro del contenedor, con ROS cargado (cada contacto es un `ros2 launch`):

  ./run.sh python3 scripts/mision.py --inicio 2026-10-02T12:00:00 --horas 24 \\
      --id m1-dia --salida runs

Escribe `<salida>/<id>/`: plan.json, informe.json, informe.txt, metrics.json,
mision.json, mundo/, od/ y contactos/cNNN/ (telemetry.jsonl, metrics.json,
launch.log de cada corrida). Con la misma configuracion y fecha de inicio el
informe es identico byte a byte.
"""
import argparse
import datetime as dt
import os
import secrets
import sys

from atp_mision.mision import Config, Mision


def tope_concurrencia():
    """Maximo seguro de contactos en paralelo: la mitad de los hilos (cada contacto
    lanza ~9 nodos ROS en lockstep a maxima velocidad). Un congelamiento total de la
    maquina con --concurrencia 6 en 8 hilos motivo este tope. Se ajusta con
    ATP_MAX_CONCURRENCIA."""
    try:
        hilos = len(os.sched_getaffinity(0))
    except AttributeError:
        hilos = os.cpu_count() or 2
    return max(1, int(os.environ.get('ATP_MAX_CONCURRENCIA', hilos // 2)))


def _lista(texto):
    return tuple(x.strip() for x in str(texto).split(',') if x.strip())


def main(argv=None):
    d = Config()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sats', default=','.join(d.sats))
    ap.add_argument('--estaciones', default=','.join(d.estaciones),
                    help='claves de data/estaciones.yaml')
    ap.add_argument('--inicio', default=d.inicio, help='UTC ISO 8601 FIJO (repetible)')
    ap.add_argument('--horas', type=float, default=d.horas)
    ap.add_argument('--n-od', type=int, default=d.n_od,
                    help='contactos de calibracion por satelite (0: sin OD)')
    ap.add_argument('--n-min-od', type=int, default=d.n_min_od,
                    help='pases de calibracion para volar con la orbita estimada')
    ap.add_argument('--latencia-od', type=float, default=d.latencia_od_s,
                    help='s del LOS del ultimo pase de la OD a poder usarla')
    ap.add_argument('--el-cal-min', type=float, default=d.el_cal_min_deg)
    ap.add_argument('--sdr', default=d.sdr, choices=['cal', 'todos', 'ninguno'])
    ap.add_argument('--gemelos', default=d.gemelos, choices=['programa', 'todos', 'ninguno'])
    ap.add_argument('--concurrencia', type=int, default=d.concurrencia,
                    help='contactos en paralelo (se recorta a la mitad de los hilos; ver --forzar)')
    ap.add_argument('--forzar', action='store_true',
                    help='no recortar --concurrencia al tope de la maquina')
    ap.add_argument('--margen-acm', type=float, default=d.margen_acm_db)
    ap.add_argument('--keyhole', default=d.keyhole, choices=['marcar', 'recortar', 'descartar'])
    ap.add_argument('--el-min', type=float, default=d.el_min_deg)
    ap.add_argument('--metodo', default=d.metodo, choices=['auto', 'bb', 'milp'])
    ap.add_argument('--prioridades', default='', help='"NOAA 19=2,ISS=1"')
    ap.add_argument('--plazo', type=float, default=d.plazo_s, help='plazo por corrida (s de pared)')
    ap.add_argument('--semilla', type=int, default=d.semilla)
    ap.add_argument('--solo-plan', action='store_true', help='planifica y sale')
    ap.add_argument('--reanudar', action='store_true',
                    help='reutiliza las corridas ya terminadas de <salida>/<id> (mismos argumentos)')
    ap.add_argument('--id', default='', help='run_id fijo (por defecto, uno nuevo)')
    ap.add_argument('--salida', default='runs')
    a = ap.parse_args(argv)
    tope = tope_concurrencia()
    if a.concurrencia > tope and not a.forzar:
        print(f'aviso: --concurrencia {a.concurrencia} recortada a {tope} '
              f'(mitad de los hilos; --forzar la respeta)', file=sys.stderr)
        a.concurrencia = tope

    prior = []
    for par in _lista(a.prioridades):
        n, _, v = par.rpartition('=')
        prior.append((n.strip(), float(v)))
    cfg = Config(sats=_lista(a.sats), estaciones=_lista(a.estaciones), inicio=a.inicio,
                 horas=a.horas, n_od=a.n_od, n_min_od=a.n_min_od, latencia_od_s=a.latencia_od,
                 el_cal_min_deg=a.el_cal_min, sdr=a.sdr, gemelos=a.gemelos,
                 concurrencia=a.concurrencia, margen_acm_db=a.margen_acm, keyhole=a.keyhole,
                 el_min_deg=a.el_min, metodo=a.metodo, prioridades=tuple(prior),
                 plazo_s=a.plazo, semilla=a.semilla, reanudar=a.reanudar)
    run_id = a.id or (dt.datetime.now(dt.timezone.utc).strftime('%Y%m%d-%H%M%S-')
                      + secrets.token_hex(3))
    carpeta = os.path.join(a.salida, run_id)
    m = Mision(cfg, carpeta)
    if a.solo_plan:
        os.makedirs(carpeta, exist_ok=True)
        m.planificar()
        m.preparar()
        n_cal = sum(1 for c in m.contactos if c.rol == 'cal')
        print(f'{len(m.contactos)} contactos ({n_cal} de calibracion)')
        return 0
    m.correr()
    return 0


if __name__ == '__main__':
    sys.exit(main())
