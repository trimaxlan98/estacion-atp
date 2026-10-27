"""Lanzador de corridas: un subprocess por corrida (grupo propio) con su propio ROS_DOMAIN_ID,
gobernado por el `Governor` (cupos por clase y admision por carga, docs/APP.md 3).

La red del contenedor es `host`: sin un dominio DDS distinto por corrida, dos
corridas se mezclarian los topicos (/atp/reference, /clock, ...). Por eso el
dominio sale de un pool (100-199) y nunca se comparte entre corridas activas.

Estados de una corrida: `queued` (espera cupo), `esperando_carga` (hay cupo pero la carga del host
supera el umbral de su clase), `running`, `done | error | cancelled | timeout`, e `interrumpida` (la
mato una senal externa; transitoria mientras espera el reintento, final al agotarlos).
"""
import json
import logging
import os
import secrets
import shutil
import sys
from datetime import datetime, timezone

from station.config import CLASE_POR_KIND, REPO_ROOT, RUN_ID_RE, es_final
from station.db import now_iso
from station.executor import GovernedExecutor
from station.experiments import (ADCS_LAUNCH, MISION_SCRIPT, PLAN_SCRIPT, PROCESO_LAUNCH, cli_args,
                                 clase_de, kind_of_run, proceso_command, proceso_spec,
                                 result_file, script_args)
from station.governor import Governor
from station.plans import plan_metrics

log = logging.getLogger('station.runner')


def new_run_id():
    return (datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
            + '-' + secrets.token_hex(3))


def default_command(run, run_dir):
    """Comando real. Una lista (sin shell): los args ya fueron validados.
    `kind: plan` ejecuta scripts/planificar.py, que guarda plan.json en la carpeta de
    la corrida; `kind: adcs` lanza el paquete sat_adcs; `kind: proceso` ejecuta el `comando` de su
    YAML (marcadores resueltos, args al final); el resto, atp_ros."""
    if kind_of_run(run) == 'proceso' or run['launch'] == PROCESO_LAUNCH:
        ctx = {'run_id': run_dir.name, 'run_dir': run_dir, 'runs_dir': run_dir.parent,
               'domain': run.get('ros_domain_id'), 'root': REPO_ROOT}
        return proceso_command(run['spec'], run['args'], ctx)
    if run['launch'] in (PLAN_SCRIPT, MISION_SCRIPT):
        return ([sys.executable, str(REPO_ROOT / 'scripts' / run['launch'])]
                + script_args(run['args']) + [f'--id={run_dir.name}', f'--salida={run_dir.parent}'])
    pkg = 'sat_adcs' if run['launch'] == ADCS_LAUNCH else 'atp_ros'
    return (['ros2', 'launch', pkg, run['launch']]
            + cli_args(run['args']) + [f'record_dir:={run_dir}'])


class Manager(GovernedExecutor):
    def __init__(self, settings, db, command=default_command, governor=None):
        super().__init__(settings, governor or Governor(settings))
        self.db = db
        self.command = command          # (run, run_dir) -> argv; los tests lo sustituyen

    # ------------------------------------------------------------ ciclo de vida
    async def start(self):
        self.s.runs_dir.mkdir(parents=True, exist_ok=True)
        self.db.mark_interrupted()

    async def stop(self):
        await self.detener()

    # ---------------------------------------------------------------- API interna
    @property
    def domains(self):                  # compat: los dominios en uso viven en el gobernador
        return self.gov.dominios

    def run_dir(self, run_id):
        if not RUN_ID_RE.match(run_id):
            raise ValueError('run_id invalido')
        return self.s.runs_dir / run_id

    def active_count(self):
        return self.db.count_active()

    def submit(self, exp, args):
        run_id = new_run_id()
        d = self.run_dir(run_id)
        d.mkdir(parents=True)
        timeout = min(int(exp.get('timeout_s', self.s.default_timeout)),
                      self.s.max_timeout)
        spec = proceso_spec(exp) if exp['kind'] == 'proceso' else None
        self.db.create_run(run_id, exp['id'], exp['title'], args, exp['launch'], timeout,
                           kind=exp['kind'], clase=clase_de(exp), spec=spec)
        self._write_params(run_id)
        self.lanzar(run_id)
        return run_id

    def _write_params(self, run_id):
        r = self.db.get_run(run_id)
        p = {'experiment_id': r['experiment_id'], 'launch': r['launch'],
             'args': r['args'], 'ros_domain_id': r['ros_domain_id'],
             'created_at': r['created_at']}
        (self.run_dir(run_id) / 'params.json').write_text(
            json.dumps(p, indent=1), encoding='utf-8')

    async def cancel(self, run_id):
        """True si se cancelo; False si ya no estaba activa."""
        r = self.db.get_run(run_id)
        if r is None or es_final(r['status'], r['finished_at']):
            return False
        return await self.cancelar(run_id)

    def delete(self, run_id):
        d = self.run_dir(run_id)
        if d.exists() and d.resolve().parent == self.s.runs_dir.resolve():
            shutil.rmtree(d, ignore_errors=True)
        self.db.delete_run(run_id)
        self.db.delete_jobs_of_run(run_id)

    # --------------------------------------------------------- ganchos del ciclo
    def _get(self, run_id):
        return self.db.get_run(run_id)

    def _clase(self, r):
        return r.get('clase') or CLASE_POR_KIND.get(kind_of_run(r), 'pesado')

    def _label(self, run_id):
        return f'corrida:{run_id}'

    def _estado_espera(self, run_id, motivo):
        r = self.db.get_run(run_id)
        if r is None or r['status'] not in ('queued', 'esperando_carga'):
            return
        nuevo = 'esperando_carga' if motivo == 'carga' else 'queued'
        if nuevo != r['status']:
            self.db.update_run(run_id, status=nuevo)

    def _arranca(self, run_id, domain, pantalla):
        self.db.update_run(run_id, status='running', started_at=now_iso(), ros_domain_id=domain)
        self._write_params(run_id)

    def _lanzamiento(self, run_id, r, domain, pantalla):
        r = self.db.get_run(run_id)       # con el dominio ya registrado (marcador {domain})
        d = self.run_dir(run_id)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), PYTHONUNBUFFERED='1',
                   RCUTILS_COLORIZED_OUTPUT='0')
        return self.command(r, d), env, str(self.s.root), d / 'launch.log'

    def _fallo(self, run_id, ex):
        self.db.update_run(run_id, status='error', error=str(ex)[:500], finished_at=now_iso())

    def _marcar_cancelada(self, run_id):
        r = self.db.get_run(run_id)
        if r and not es_final(r['status'], r['finished_at']):
            self.db.update_run(run_id, status='cancelled', finished_at=now_iso())

    def _interrumpida(self, run_id, intento, espera, rc):
        nota = (f'interrumpida por senal externa (codigo {rc}); reintento {intento}/{self.s.reintentos}'
                f' en {espera:g} s')
        self.db.update_run(run_id, status='interrumpida', exit_code=rc, error=nota,
                           reintentos=intento)
        self._log(run_id, nota)

    def _reintenta(self, run_id, intento):
        """Deja la carpeta como recien creada (menos params.json y el log, que se conserva con su
        marca) y vuelve a la cola."""
        d = self.run_dir(run_id)
        for p in d.iterdir():
            if p.name in ('params.json', 'launch.log'):
                continue
            shutil.rmtree(p, ignore_errors=True) if p.is_dir() else p.unlink(missing_ok=True)
        self.db.update_run(run_id, status='queued', started_at=None, error=None, exit_code=None)
        self._log(run_id, f'reintento {intento}: de nuevo a la cola')

    def _log(self, run_id, texto):
        try:
            with open(self.run_dir(run_id) / 'launch.log', 'ab') as f:
                f.write(f'[estacion] {texto}\n'.encode('utf-8'))
        except OSError:
            pass

    # ----------------------------------------------------------- fin de la corrida
    @staticmethod
    def _plan_metrics(d):
        """metrics de un plan (derivadas de plan.json y guardadas en metrics.json);
        None si plan.json no es legible."""
        try:
            m = plan_metrics(json.loads((d / 'plan.json').read_text(encoding='utf-8')))
        except (OSError, ValueError, KeyError):
            return None
        (d / 'metrics.json').write_text(json.dumps(m, indent=1), encoding='utf-8')
        return m

    def _proceso_hecho(self, r, d, rc):
        """`kind: proceso` esta hecho si salio con 0 Y existe `hecho_cuando` (APP 2)."""
        return rc == 0 and (d / r['spec']['hecho_cuando']).is_file()

    def _resultado_ok(self, run_id, rc):
        r = self.db.get_run(run_id)
        d = self.run_dir(run_id)
        if kind_of_run(r) == 'proceso':
            return self._proceso_hecho(r, d, rc)
        return (d / result_file(r['launch'])).exists()

    def _metricas(self, r, d):
        if kind_of_run(r) == 'proceso':
            nombre = (r['spec'] or {}).get('metricas')
            mp = d / nombre if nombre else None
        else:
            mp = d / 'metrics.json'
        if mp is None or not mp.is_file():
            return None
        try:
            m = json.loads(mp.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None
        return m if isinstance(m, dict) else {'valor': m}

    def _final(self, run_id, rc, senal=False):
        d = self.run_dir(run_id)
        r = self.db.get_run(run_id)
        kind = kind_of_run(r)
        metrics = self._metricas(r, d)
        has_result = self._resultado_ok(run_id, rc)
        if has_result and kind == 'plan':
            metrics = self._plan_metrics(d)
            has_result = metrics is not None
        error = None
        if run_id in self.cancelled:
            status = 'cancelled'
        elif run_id in self.timed_out:
            status, error = 'timeout', f'excedio el limite de {r["timeout_s"]} s'
        elif has_result:
            status = 'done'
        elif senal:
            status = 'interrumpida'
            error = (f'interrumpida por senal externa (codigo {rc}) y agoto los '
                     f'{self.s.reintentos} reintentos')
        else:
            status = 'error'
            if kind == 'proceso':
                falta = '' if rc != 0 else f' ({r["spec"]["hecho_cuando"]} no existe)'
                error = f'el proceso termino (codigo {rc}) sin producir el resultado{falta}'
            else:
                quien = r['launch'] if kind in ('plan', 'mision') else 'ros2 launch'
                error = f'{quien} termino (codigo {rc}) sin producir el resultado'
        self.db.update_run(run_id, status=status, exit_code=rc, error=error,
                           metrics=metrics, finished_at=now_iso())
