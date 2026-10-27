import json
import shutil
import sys
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from station.app import create_app  # noqa: E402
from station.config import Settings  # noqa: E402
from station.runner import default_command  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).resolve().parent / 'fixtures'

# Proceso falso en lugar de `ros2 launch`: escribe lo mismo que el grabador.
FAKE = r'''
import json, os, sys, time
mode, run_dir = sys.argv[1], sys.argv[2]
print("dominio", os.environ["ROS_DOMAIN_ID"], flush=True)
print("pid", os.getpid(), flush=True)
if mode in ("senal", "senal_una_vez"):     # muerte por senal externa (como OOM o el vigilante)
    import signal
    cnt = os.environ["FAKE_COUNTER"]
    n = len(open(cnt).read()) if os.path.exists(cnt) else 0
    open(cnt, "a").write("x")
    if mode == "senal" or n == 0:
        os.kill(os.getpid(), signal.SIGKILL)
    mode = "ok"
if mode == "sleep":
    time.sleep(120)
if mode == "fail":
    print("algo salio mal", flush=True)
    sys.exit(3)
if mode == "proceso":   # lo que deja un `kind: proceso` de red (contrato red + metricas.json)
    json.dump({"version": 1, "inicio_utc": "2026-10-05T12:00:00Z", "dt_s": 1.0,
               "estaciones": [{"id": "cdmx", "nombre": "CDMX", "lat_deg": 19.4, "lon_deg": -99.1, "alt_m": 2240,
                               "antenas": [{"id": "A1", "banda": "X", "pos_enu_m": [0, 0, 0]}]}],
               "satelites": [{"id": "S00", "nombre": "SAT-0"}], "parametros": {}},
              open(os.path.join(run_dir, "escena.json"), "w"))
    with open(os.path.join(run_dir, "red.jsonl"), "w") as g:
        for k in range(5):
            g.write(json.dumps({"t": float(k), "geo": {"cdmx": [["S00", 359.0 + 2 * k, 30.0, 1500.0, -1.0]]},
                                "antenas": {"cdmx/A1": {"sat": "S00", "estado": "seguimiento",
                                                        "ref": [359.0 + 2 * k, 30.0], "med": [359.0 + 2 * k, 30.0]}}}) + "\n")
    json.dump({"veredicto": "OK", "n": 5}, open(os.path.join(run_dir, "metricas.json"), "w"))
    sys.exit(0)
if mode == "plan":      # lo que deja scripts/planificar.py (solo los campos que usa el backend)
    ven = lambda e, ids: {"contactos": len(ids), "minutos": 10.0 * len(ids), "gb": 1.5 * len(ids), "gb_shannon": 2.0 * len(ids), "por_satelite": {}, "por_estacion": {}}
    plan = {"banda": "X", "horas": 24.0, "inicio_utc": "2026-10-02T12:00:00.000Z", "n_ventanas": 6,
            "estaciones": {"cdmx": {}, "tromso": {}}, "estaciones_lista": ["cdmx", "tromso"],
            "satelites": [{"nombre": "ISS (ZARYA)", "norad": 25544}],
            "disponible": {"contactos": 6, "minutos": 60.0, "gb": 9.0},
            "ventanas": [{"id": i, "sat": "ISS (ZARYA)", "estacion": "cdmx" if i % 2 else "tromso",
                          "aos": "2026-10-02T12:00:00.000Z", "los": "2026-10-02T12:10:00.000Z",
                          "el_max_deg": 30.0, "vol_gb": 1.5} for i in range(6)],
            "planes": {"codicioso_peso": {"algoritmo": "codicioso-peso", "ids": [0, 1], "optimo_probado": False,
                                          "resumen": ven(0, [0, 1])},
                       "optimo": {"algoritmo": "optimo-bb", "ids": [0, 1, 2], "optimo_probado": True,
                                  "resumen": ven(0, [0, 1, 2])}},
            "brecha_codicioso_pct": 33.3,
            "aporte_estacion": {"tromso": {"gb_con": 4.5, "gb_sin": 3.0, "delta_gb": 1.5, "delta_pct": 50.0,
                                           "contactos_con": 3, "contactos_sin": 2, "ids_sin": [1, 3]}}}
    json.dump(plan, open(os.path.join(run_dir, "plan.json"), "w"))
    print("plan listo", flush=True)
    sys.exit(0)
if mode == "mision":    # lo que deja scripts/mision.py (informe.json, metrics.json y un contacto)
    c = os.path.join(run_dir, "contactos", "c001")
    os.makedirs(c)
    with open(os.path.join(c, "telemetry.jsonl"), "w") as g:
        for i in range(5):
            g.write(json.dumps({"t": i * 0.1, "az": 10.0 + i, "el": 30.0, "e_pt": 0.01, "cn0_dbhz": 80.0}) + "\n")
    json.dump({"n": 5}, open(os.path.join(c, "metrics.json"), "w"))
    open(os.path.join(c, "launch.log"), "w").write("[autotrack] ENGANCHE\n")
    json.dump({"vel": 1}, open(os.path.join(c, "params.json"), "w"))
    inf = {"mision": {"inicio_utc": "2026-10-02T12:00:00Z", "horas": 3.0, "estaciones": ["cdmx"], "satelites": ["ISS (ZARYA)"]},
           "totales": {"contactos_planificados": 1, "contactos_ejecutados": 1, "gb": 1.5, "gb_plan": 1.6, "minutos": 9.0},
           "contactos": [{"id": 1, "contacto": "c001", "sat": "ISS (ZARYA)", "estacion": "cdmx", "estado": "ok", "gb": 1.5, "carpeta": "contactos/c001"}],
           "od": [], "mejora_od": {"contactos_con_od": 0, "contactos_con_gemelo": 0, "pares": []}, "incidencias": []}
    json.dump(inf, open(os.path.join(run_dir, "informe.json"), "w"))
    json.dump({"kind": "mision", "gb": 1.5, "contactos_ejecutados": 1}, open(os.path.join(run_dir, "metrics.json"), "w"))
    print("mision lista", flush=True)
    sys.exit(0)
f = open(os.path.join(run_dir, "telemetry.jsonl"), "w")
for i in range(8):
    f.write(json.dumps({"t": i * 0.1, "az": 10.0 + i, "el": 30.0, "e_pt": 0.01 * i,
                        "margin_db": 15.0, "link_closed": True}) + "\n")
    f.flush()
    print("muestra", i, flush=True)
    time.sleep(float(os.environ.get("FAKE_DELAY", "0.05")))
if mode == "campaign":
    json.dump({"mode": "analitico", "n": 3, "p95": 0.05, "pasa": True,
               "rms_deg": [0.01, 0.02, 0.03]}, open(os.path.join(run_dir, "campaign.json"), "w"))
elif mode != "nometrics":
    json.dump({"n": 8, "duration_s": 0.7, "verdict": "CUMPLE",
               "pointing": {"rms": 0.02, "p95": 0.03, "max": 0.07}},
              open(os.path.join(run_dir, "metrics.json"), "w"))
'''


@pytest.fixture
def behavior():
    return {'mode': 'ok', 'delay': '0.05'}


@pytest.fixture
def extra_env():
    """Variables STATION_* adicionales de una prueba (se sobreescribe con parametrize)."""
    return {}


def preparar_entorno(tmp_path, monkeypatch, extra_env):
    """Directorios temporales, catalogo de experimentos (los reales + los de prueba de
    tests/fixtures) y de renders (solo los de prueba); las esperas del gobernador, cortas."""
    exps = tmp_path / 'catalogo' / 'experiments'
    exps.mkdir(parents=True)
    for p in list((REPO / 'experiments').glob('*.yaml')) + list((FIXTURES / 'experiments').glob('*.yaml')):
        shutil.copy(p, exps / p.name)
    rend = tmp_path / 'catalogo' / 'render'
    rend.mkdir()
    for p in (FIXTURES / 'render').glob('*.yaml'):
        shutil.copy(p, rend / p.name)
    env = {'STATION_RUNS_DIR': tmp_path / 'runs', 'STATION_DB_PATH': tmp_path / 'data' / 'station.db',
           'STATION_FRONTEND_DIST': tmp_path / 'sin-frontend', 'STATION_KILL_GRACE': '3',
           'STATION_EXPERIMENTS_DIR': exps, 'STATION_RENDER_DIR': rend,
           'STATION_CUPO_PESADO': '2',            # las pruebas de la ronda 3 esperan 2 corridas a la vez
           'STATION_RECHEQUEO_S': '0.05', 'STATION_REINTENTO_ESPERA': '0.05',
           'FAKE_COUNTER': tmp_path / 'contador'}
    env.update(extra_env)
    for k, v in env.items():
        monkeypatch.setenv(k, str(v))


class Carga:
    """Carga del host simulada: `c.valor` es lo que lee el gobernador."""
    valor = 0.0

    def __call__(self):
        return self.valor


@pytest.fixture
def client(tmp_path, behavior, monkeypatch, extra_env):
    """App con directorios temporales, un lanzador falso (sin ROS) y la carga del host simulada."""
    preparar_entorno(tmp_path, monkeypatch, extra_env)
    monkeypatch.setenv('FAKE_DELAY', behavior['delay'])
    settings = Settings()
    carga, client_journal = Carga(), {'v': None}

    comandos = []

    def fake_command(run, run_dir):
        mode = behavior['mode']
        if mode == 'ok' and run['launch'].startswith('campaign'):
            mode = 'campaign'
        elif mode == 'ok' and run['launch'] == 'planificar.py':
            mode = 'plan'
        elif mode == 'ok' and run['launch'] == 'mision.py':
            mode = 'mision'
        elif mode == 'ok' and run.get('kind') == 'proceso':
            mode = 'proceso'
        comandos.append(default_command(run, run_dir))       # el real, que aqui no se ejecuta
        return [sys.executable, '-c', FAKE, mode, str(run_dir)]

    app = create_app(settings, fake_command, load_reader=carga, journal_reader=lambda n: client_journal['v'])
    with TestClient(app) as c:
        c.settings = settings
        c.comandos = comandos
        c.carga, c.journal = carga, client_journal
        c.contador = tmp_path / 'contador'
        yield c


@pytest.fixture
def client_real(tmp_path, monkeypatch, extra_env):
    """App con el lanzador REAL (`default_command`): para `kind: proceso` y renders de prueba, que
    solo ejecutan scripts de tests/fixtures (sin ROS)."""
    preparar_entorno(tmp_path, monkeypatch, extra_env)
    settings = Settings()
    carga, client_journal = Carga(), {'v': None}
    with TestClient(create_app(settings, load_reader=carga, journal_reader=lambda n: client_journal['v'])) as c:
        c.settings = settings
        c.carga, c.journal = carga, client_journal
        yield c


def wait_status(client, run_id, targets=('done', 'error', 'cancelled', 'timeout'),
                timeout=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        r = client.get(f'/api/runs/{run_id}').json()
        if r['status'] in targets:
            return r
        time.sleep(0.1)
    raise AssertionError(f'{run_id}: estado {r["status"]} tras {timeout} s')


def parse_sse(text):
    """Texto SSE -> lista de (evento, datos)."""
    out = []
    for block in text.split('\n\n'):
        ev, data = None, None
        for line in block.splitlines():
            if line.startswith('event: '):
                ev = line[7:]
            elif line.startswith('data: '):
                data = json.loads(line[6:])
        if ev:
            out.append((ev, data))
    return out


def wait_job(client, job_id, finales=('hecho', 'error', 'cancelado', 'interrumpida'), timeout=30.0):
    """Espera a que un trabajo de render llegue a un estado final (`interrumpida` cuenta solo si ya
    tiene finished_at: antes es la espera del reintento)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        j = client.get(f'/api/renders/{job_id}').json()
        if j['estado'] in finales and (j['estado'] != 'interrumpida' or j['finished_at']):
            return j
        time.sleep(0.05)
    raise AssertionError(f'{job_id}: estado {j["estado"]} tras {timeout} s')


def wait_until(cond, timeout=15.0, paso=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        v = cond()
        if v:
            return v
        time.sleep(paso)
    raise AssertionError('la condicion no se cumplio a tiempo')
