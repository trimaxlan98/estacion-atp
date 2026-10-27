"""Configuracion de la Estacion (variables de entorno con prefijo STATION_)."""
import os
import re
from pathlib import Path

# station/backend/station/config.py -> raiz del repo (o /ws en el contenedor)
REPO_ROOT = Path(__file__).resolve().parents[3]

# ids estrictos: nunca se arma una ruta con texto del usuario sin pasar por aqui
RUN_ID_RE = re.compile(r'^\d{8}-\d{6}-[0-9a-f]{6}$')
EXPERIMENT_ID_RE = re.compile(r'^[a-z0-9][a-z0-9-]{0,62}$')
# contacto de una mision (`kind: mision`): c012 o su gemelo con TLE c012_tle
CONTACT_RE = re.compile(r'^c\d{3}(_tle)?$')
ARG_KEY_RE = re.compile(r'^[a-z][a-z0-9_]{0,40}$')

# ids de trabajos de render: r<fecha>-<hora>-<6 hex> (APP 4); id de un render del catalogo
JOB_ID_RE = re.compile(r'^r\d{8}-\d{6}-[0-9a-f]{6}$')
RENDER_ID_RE = EXPERIMENT_ID_RE

# clases del gobernador (APP 3) y sus valores por defecto: (cupo, umbral de carga 1 min)
CLASES = ('ligero', 'pesado', 'gazebo', 'manim')
CUPO_DEFECTO = {'ligero': 2, 'pesado': 1, 'gazebo': 1, 'manim': 1}
UMBRAL_DEFECTO = {'ligero': 10.0, 'pesado': 6.0, 'gazebo': 4.0, 'manim': 5.0}
# clase por defecto de una corrida del catalogo segun su kind (APP 3)
CLASE_POR_KIND = {'loop': 'pesado', 'adcs': 'pesado', 'campaign': 'pesado',
                  'mision': 'pesado', 'plan': 'ligero', 'proceso': 'pesado'}

# estados de las corridas: los de siempre mas `esperando_carga` (el gobernador la retiene por la
# carga del host) e `interrumpida` (la mato una senal externa). `interrumpida` SIN finished_at es
# transitoria (espera para reintentar); con finished_at es final: usar `es_final`.
TERMINAL = ('done', 'error', 'cancelled', 'timeout')
# estados de los trabajos de render (APP 4)
RENDER_FINALES = ('hecho', 'error', 'cancelado')


def es_final(estado, finished_at=None):
    """True si la corrida/trabajo ya no cambiara de estado."""
    if estado in TERMINAL or estado in RENDER_FINALES:
        return True
    return estado == 'interrumpida' and bool(finished_at)


class Settings:
    def __init__(self):
        e = os.environ.get
        self.root = Path(e('STATION_ROOT', str(REPO_ROOT)))
        self.runs_dir = Path(e('STATION_RUNS_DIR', str(self.root / 'runs')))
        self.db_path = Path(e('STATION_DB_PATH',
                              str(self.root / 'station' / 'data' / 'station.db')))
        self.experiments_dir = Path(e('STATION_EXPERIMENTS_DIR',
                                      str(self.root / 'experiments')))
        self.frontend_dist = Path(e('STATION_FRONTEND_DIST',
                                    str(self.root / 'station' / 'frontend' / 'dist')))
        self.max_concurrent = int(e('STATION_MAX_CONCURRENT', '2'))
        # timeout duro de una corrida (s); un experimento puede pedir menos
        self.default_timeout = int(e('STATION_DEFAULT_TIMEOUT', '1200'))
        self.max_timeout = int(e('STATION_MAX_TIMEOUT', '3600'))
        # al cancelar o expirar: SIGINT al grupo, y SIGKILL si no sale a tiempo
        self.kill_grace_s = float(e('STATION_KILL_GRACE', '8'))
        # pool de ROS_DOMAIN_ID (red host: el aislamiento por dominio es obligatorio)
        self.domain_min = int(e('STATION_DOMAIN_MIN', '100'))
        self.domain_max = int(e('STATION_DOMAIN_MAX', '199'))
        self.ros_distro = e('ROS_DISTRO', '')
        # --- gobernador de recursos (APP 3). `max_concurrent` es el tope GLOBAL de tareas a la vez
        # (corridas + renders); cada clase tiene ademas su cupo y su umbral de carga (1 min).
        self.cupos = {c: int(e(f'STATION_CUPO_{c.upper()}', str(CUPO_DEFECTO[c]))) for c in CLASES}
        self.umbrales = {c: float(e(f'STATION_UMBRAL_{c.upper()}', str(UMBRAL_DEFECTO[c])))
                         for c in CLASES}
        self.recheck_s = float(e('STATION_RECHEQUEO_S', '10'))     # reevaluar la carga cada 10 s
        self.reintentos = int(e('STATION_REINTENTOS', '2'))        # tras una muerte por senal externa
        self.reintento_espera_s = float(e('STATION_REINTENTO_ESPERA', '30'))   # 30, 60, 120... (tope 600)
        # pool de pantallas Xephyr para los trabajos con `pantalla: true` (:80-:83)
        self.pantalla_min = int(e('STATION_PANTALLA_MIN', '80'))
        self.pantalla_max = int(e('STATION_PANTALLA_MAX', '83'))
        self.lp_threads = e('STATION_LP_THREADS', '3')             # hilos de llvmpipe en los renders
        # --- renders, sistema
        self.render_dir = Path(e('STATION_RENDER_DIR', str(self.root / 'render')))
        self.render_timeout = int(e('STATION_RENDER_TIMEOUT', '3600'))
        self.vigilante_lineas = int(e('STATION_VIGILANTE_LINEAS', '15'))
        # renders `ejecutor: host` (APP 4.1): sin latido en este tiempo el trabajo queda `interrumpida`
        self.latido_timeout_s = float(e('STATION_LATIDO_TIMEOUT', '60'))
