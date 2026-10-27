"""Persistencia SQLite (WAL) de las corridas.

Un solo proceso y bajo volumen: sqlite3 sincrono con lock, como en ManimStudio.
La verdad de las series es el disco (runs/<id>/); la base indexa y guarda el
estado y las metricas para listar sin abrir cada carpeta.
"""
import json
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    id TEXT PRIMARY KEY,
    experiment_id TEXT NOT NULL,
    title TEXT NOT NULL,
    status TEXT NOT NULL,
    args TEXT NOT NULL,
    launch TEXT NOT NULL,
    ros_domain_id INTEGER,
    timeout_s INTEGER NOT NULL,
    exit_code INTEGER,
    error TEXT,
    metrics TEXT,
    created_at TEXT NOT NULL,
    started_at TEXT,
    finished_at TEXT
);
CREATE INDEX IF NOT EXISTS idx_runs_created ON runs(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_runs_exp ON runs(experiment_id, created_at DESC);

-- trabajos de render (APP 4): uno por (corrida, render). La verdad de los archivos es
-- runs/<run_id>/media/<job_id>/ (con su media.json); la tabla indexa estado y procedencia.
CREATE TABLE IF NOT EXISTS renders (
    id TEXT PRIMARY KEY,
    run_id TEXT NOT NULL,
    render_id TEXT NOT NULL,
    title TEXT NOT NULL,
    estado TEXT NOT NULL,
    args TEXT NOT NULL,
    spec TEXT NOT NULL,
    clase TEXT NOT NULL,
    pantalla TEXT,
    ros_domain_id INTEGER,
    timeout_s INTEGER NOT NULL,
    reintentos INTEGER NOT NULL DEFAULT 0,
    exit_code INTEGER,
    error TEXT,
    media TEXT,
    created_at TEXT NOT NULL,
    started_at TEXT,
    finished_at TEXT
);
CREATE INDEX IF NOT EXISTS idx_renders_created ON renders(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_renders_run ON renders(run_id, created_at DESC);
"""

# columnas anadidas a `runs` despues de la ronda 3 (migracion in situ de bases existentes)
RUNS_EXTRA = (('kind', 'TEXT'), ('clase', 'TEXT'), ('spec', 'TEXT'),
              ('reintentos', 'INTEGER NOT NULL DEFAULT 0'))


def now_iso():
    return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


class DB:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(str(path), check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        self.lock = threading.Lock()
        with self.lock:
            self.conn.execute('PRAGMA journal_mode=WAL')
            self.conn.execute('PRAGMA synchronous=NORMAL')
            self.conn.executescript(SCHEMA)
            have = {r[1] for r in self.conn.execute('PRAGMA table_info(runs)')}
            for col, tipo in RUNS_EXTRA:
                if col not in have:
                    self.conn.execute(f'ALTER TABLE runs ADD COLUMN {col} {tipo}')
            have = {r[1] for r in self.conn.execute('PRAGMA table_info(renders)')}
            if 'trabajador' not in have:        # renders `ejecutor: host`: quien reclamo el trabajo
                self.conn.execute('ALTER TABLE renders ADD COLUMN trabajador TEXT')
            self.conn.commit()

    def close(self):
        with self.lock:
            self.conn.close()

    def _row(self, r):
        if r is None:
            return None
        d = dict(r)
        d['run_id'] = d.pop('id')
        d['args'] = json.loads(d['args'])
        d['metrics'] = json.loads(d['metrics']) if d['metrics'] else None
        d['spec'] = json.loads(d['spec']) if d.get('spec') else None
        return d

    def create_run(self, run_id, exp_id, title, args, launch, timeout_s,
                   kind=None, clase=None, spec=None):
        with self.lock:
            self.conn.execute(
                'INSERT INTO runs (id, experiment_id, title, status, args, launch,'
                ' timeout_s, created_at, kind, clase, spec) VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                (run_id, exp_id, title, 'queued', json.dumps(args), launch,
                 timeout_s, now_iso(), kind, clase, json.dumps(spec) if spec else None))
            self.conn.commit()

    def get_run(self, run_id):
        with self.lock:
            r = self.conn.execute('SELECT * FROM runs WHERE id=?', (run_id,)).fetchone()
        return self._row(r)

    def list_runs(self, experiment_id=None, limit=100):
        q, a = 'SELECT * FROM runs', []
        if experiment_id:
            q += ' WHERE experiment_id=?'
            a.append(experiment_id)
        q += ' ORDER BY created_at DESC, rowid DESC LIMIT ?'
        a.append(limit)
        with self.lock:
            rows = self.conn.execute(q, a).fetchall()
        return [self._row(r) for r in rows]

    def count_active(self):
        with self.lock:
            return self.conn.execute(
                "SELECT COUNT(*) FROM runs WHERE status IN ('queued','running','esperando_carga')"
                " OR (status='interrumpida' AND finished_at IS NULL)"
            ).fetchone()[0]

    def update_run(self, run_id, **fields):
        if not fields:
            return
        if 'metrics' in fields and fields['metrics'] is not None:
            fields['metrics'] = json.dumps(fields['metrics'])
        cols = ', '.join(f'{k}=?' for k in fields)
        with self.lock:
            self.conn.execute(f'UPDATE runs SET {cols} WHERE id=?',
                              (*fields.values(), run_id))
            self.conn.commit()

    def delete_run(self, run_id):
        with self.lock:
            self.conn.execute('DELETE FROM runs WHERE id=?', (run_id,))
            self.conn.commit()

    def mark_interrupted(self):
        """Al arrancar: lo que quedo queued/running de un proceso anterior ya
        no existe; se marca como error en vez de dejarlo colgado."""
        with self.lock:
            self.conn.execute(
                "UPDATE runs SET status='error', error='interrumpida: la Estacion se reinicio',"
                " finished_at=? WHERE status IN ('queued','running','esperando_carga')"
                " OR (status='interrumpida' AND finished_at IS NULL)", (now_iso(),))
            self.conn.execute(
                "UPDATE renders SET estado='interrumpida', error='la Estacion se reinicio',"
                " finished_at=? WHERE estado IN ('en_cola','esperando_carga','corriendo')"
                " OR (estado='interrumpida' AND finished_at IS NULL)", (now_iso(),))
            self.conn.commit()

    # ------------------------------------------------------------------ renders
    @staticmethod
    def _job(r):
        if r is None:
            return None
        d = dict(r)
        d['job_id'] = d.pop('id')
        for k in ('args', 'spec', 'media'):
            d[k] = json.loads(d[k]) if d.get(k) else None
        return d

    def create_job(self, job_id, run_id, render_id, title, args, spec, clase, timeout_s):
        with self.lock:
            self.conn.execute(
                'INSERT INTO renders (id, run_id, render_id, title, estado, args, spec, clase,'
                ' timeout_s, created_at) VALUES (?,?,?,?,?,?,?,?,?,?)',
                (job_id, run_id, render_id, title, 'en_cola', json.dumps(args),
                 json.dumps(spec), clase, timeout_s, now_iso()))
            self.conn.commit()

    def get_job(self, job_id):
        with self.lock:
            r = self.conn.execute('SELECT * FROM renders WHERE id=?', (job_id,)).fetchone()
        return self._job(r)

    def list_jobs(self, run_id=None, estado=None, limit=100):
        q, a, w = 'SELECT * FROM renders', [], []
        if run_id:
            w.append('run_id=?')
            a.append(run_id)
        if estado:
            w.append('estado=?')
            a.append(estado)
        if w:
            q += ' WHERE ' + ' AND '.join(w)
        q += ' ORDER BY created_at DESC, rowid DESC LIMIT ?'
        a.append(limit)
        with self.lock:
            rows = self.conn.execute(q, a).fetchall()
        return [self._job(r) for r in rows]

    def update_job(self, job_id, **fields):
        if not fields:
            return
        if fields.get('media') is not None:
            fields['media'] = json.dumps(fields['media'])
        cols = ', '.join(f'{k}=?' for k in fields)
        with self.lock:
            self.conn.execute(f'UPDATE renders SET {cols} WHERE id=?',
                              (*fields.values(), job_id))
            self.conn.commit()

    def delete_job(self, job_id):
        with self.lock:
            self.conn.execute('DELETE FROM renders WHERE id=?', (job_id,))
            self.conn.commit()

    def delete_jobs_of_run(self, run_id):
        with self.lock:
            self.conn.execute('DELETE FROM renders WHERE run_id=?', (run_id,))
            self.conn.commit()

    def pending_host_jobs(self):
        """Trabajos `ejecutor: host` esperando que un trabajador los reclame (mas viejos primero)."""
        with self.lock:
            rows = self.conn.execute(
                "SELECT * FROM renders WHERE estado IN ('en_cola','esperando_carga')"
                " ORDER BY created_at ASC, rowid ASC").fetchall()
        return [j for j in map(self._job, rows) if (j['spec'] or {}).get('ejecutor') == 'host']

    def count_active_jobs(self):
        with self.lock:
            return self.conn.execute(
                "SELECT COUNT(*) FROM renders WHERE estado IN ('en_cola','esperando_carga','corriendo')"
                " OR (estado='interrumpida' AND finished_at IS NULL)").fetchone()[0]
